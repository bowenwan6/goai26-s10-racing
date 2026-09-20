"""ROS-free core of the S10 ROS 1 navigation node: the team's route runner on the real robot.

It runs the same code as the MuJoCo harness and the ROS 2 ``rl_nav`` node
(``s10_auto_nav.rl_nav``: route follower core + route runner), fed from ROS 1 topics on the AGX:

* pose from x_nav (``/base_link/odom``), moved into the route's frame by a fixed transform
  (identity until the x_nav map is registered to the route's map);
* the 13x9 height grid, the conservative scan and the yaw-frame points, built here from the
  gateway's ``/LIDAR/POINTS`` exactly as the simulator's sensor model builds them
  (``real_transfer.geometry.height_grid``, ``native_transfer.contracts.conservative_scan``).

Real-robot differences, all outside the runner:

* the runner's joint-owner request (``official`` walking actor / ``stairs_stable`` stairs actor)
  becomes a gait request (``flat`` 0x3002 / ``stairs`` 0x3003) for ``s10_ros1_control``, which
  switches only at a standstill and reports the confirmed gait back; the report is mapped back
  to the owner the runner expects;
* speeds come from ``nav.yaml`` (``runner_params``): the vendor gaits do not need the policies'
  0.5 m/s floor, and the first field tests run slower than the simulation.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from geometry import height_grid, points_in_yaw_frame
from s10_auto_nav.pure_pursuit import PurePursuitController, PursuitGains
from s10_auto_nav.rl_nav import maneuvers as maneuver_io
from s10_auto_nav.rl_nav.capability import PolicyProfile
from s10_auto_nav.rl_nav.map_check import MapSurface
from s10_auto_nav.rl_nav.map_planner import MapPlanner
from s10_auto_nav.rl_nav.route_runner import Mode, NavInput, RouteRunner, RunnerParams
from s10_auto_nav.route_follower import RouteFollowerConfig, RouteFollowerCore
from s10_auto_nav.route_planner import LocalGridConfig
from s10_auto_nav.route_v2 import CrossCheckConfig, RouteV2

OWNER_TO_GAIT = {"official": "flat", "stairs_stable": "stairs", "stop": None}
GAIT_TO_OWNER = {"flat": "official", "stairs": "stairs_stable"}


# ----------------------------------------------------------------------------- geometry
def rpy_from_quat(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def pose_matrix(x, y, z, roll, pitch, yaw):
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(roll, pitch, yaw)
    t[:3, 3] = [x, y, z]
    return t


def rpy_of(m):
    pitch = math.asin(max(-1.0, min(1.0, -m[2, 0])))
    roll = math.atan2(m[2, 1], m[2, 2])
    yaw = math.atan2(m[1, 0], m[0, 0])
    return roll, pitch, yaw


def transform_from_dict(d) -> np.ndarray:
    """{"translation": [x,y,z], "rpy_deg": [r,p,y]} or {"matrix": 4x4} -> 4x4."""
    if d is None:
        return np.eye(4)
    if "matrix" in d:
        m = np.asarray(d["matrix"], float)
        if m.shape != (4, 4):
            raise ValueError("matrix must be 4x4")
        return m
    t = [float(v) for v in d.get("translation", [0, 0, 0])]
    rpy = [math.radians(float(v)) for v in d.get("rpy_deg", [0, 0, 0])]
    return pose_matrix(*t, *rpy)


class FrameTransform:
    """route_map_T_xnav_map: applied to the x_nav pose before the runner sees it."""

    def __init__(self, matrix=None):
        self.m = np.eye(4) if matrix is None else np.asarray(matrix, float)
        self.identity = bool(np.allclose(self.m, np.eye(4)))

    @classmethod
    def load(cls, path):
        if not path:
            return cls()
        with open(path) as f:
            d = json.load(f)
        return cls(transform_from_dict(d.get("route_from_xnav", d)))

    def pose(self, x, y, z, roll, pitch, yaw):
        if self.identity:
            return x, y, z, roll, pitch, yaw
        m = self.m @ pose_matrix(x, y, z, roll, pitch, yaw)
        r, p, yw = rpy_of(m)
        return float(m[0, 3]), float(m[1, 3]), float(m[2, 3]), r, p, yw


# ----------------------------------------------------------------------------- perception
def conservative_scan(points, bins=72):
    """native_transfer.contracts.conservative_scan: observed extent per 5 deg bin, shortened by
    returns in the body-height band; NaN where nothing was seen."""
    p = np.asarray(points, float)
    p = p[np.isfinite(p).all(axis=1)]
    d = np.linalg.norm(p[:, :2], axis=1)
    keep = (d >= 0.25) & (d <= 10.0)
    p, d = p[keep], d[keep]
    ids = np.floor((np.arctan2(p[:, 1], p[:, 0]) + np.pi) / (2 * np.pi) * bins).astype(int)
    ids = np.clip(ids, 0, bins - 1)
    extent = np.zeros(bins)
    np.maximum.at(extent, ids, d)
    obstacle = np.full(bins, np.inf)
    band = (p[:, 2] >= -0.25) & (p[:, 2] <= 0.75)
    np.minimum.at(obstacle, ids[band], d[band])
    ranges = np.minimum(extent, obstacle)
    ranges[extent == 0] = np.nan
    return ranges


def decode_pointcloud2(msg):
    """Duck-typed ROS PointCloud2 -> Nx3 float (x, y, z), honouring row padding and endianness."""
    w, h, step, row = int(msg.width), int(msg.height), int(msg.point_step), int(msg.row_step)
    if min(w, h, step) <= 0 or w * h > 1_000_000 or row < w * step:
        raise ValueError("invalid cloud dimensions")
    if len(msg.data) < h * row:
        raise ValueError("truncated cloud buffer")
    fields = {f.name: f for f in msg.fields}
    cols = []
    for name in ("x", "y", "z"):
        f = fields.get(name)
        if f is None or f.datatype not in (7, 8) or f.count != 1:
            raise ValueError("cloud requires scalar float x/y/z")
        dtype = np.dtype((">" if msg.is_bigendian else "<") + ("f4" if f.datatype == 7 else "f8"))
        if f.offset < 0 or f.offset + dtype.itemsize > step:
            raise ValueError("invalid field offset")
        a = np.ndarray((h, w), dtype=dtype, buffer=memoryview(msg.data), offset=f.offset, strides=(row, step))
        cols.append(a.reshape(-1))
    return np.column_stack(cols).astype(float)


class Perception:
    """Cloud in the lidar frame + robot pose -> the runner's grid, scan and yaw-frame points."""

    def __init__(self, base_from_lidar, max_points=60000, seed=0):
        self.base_from_lidar = np.asarray(base_from_lidar, float)
        self.max_points = int(max_points)
        self.blind_radius = 0.0
        self.rng = np.random.default_rng(seed)

    def fill_blind(self, grid, valid):
        """The real lidars cannot see the floor under and right around the body (about 0.6 m on
        dog 048), and the planner treats unknown cells inside its commit length as blocked. Cells
        within blind_radius of the body centre that have no return take the height of the nearest
        seen cell: the robot is standing on that ground and saw it before walking onto it. A small
        object inside the blind zone is NOT detected (keep the route clear)."""
        r = self.blind_radius
        if r <= 0 or valid.all() or not valid.any():
            return grid, valid
        from scipy.ndimage import distance_transform_edt
        n, m = grid.shape
        xs = (np.arange(n) - (n - 1) / 2.0) * 0.15
        ys = (np.arange(m) - (m - 1) / 2.0) * 0.15
        near = np.hypot(xs[:, None], ys[None, :]) <= r
        idx = distance_transform_edt(~valid, return_distances=False, return_indices=True)
        fill = near & ~valid
        grid = np.array(grid, float)
        grid[fill] = grid[idx[0][fill], idx[1][fill]]
        return grid, valid | fill

    def observe(self, cloud_xyz, pose6):
        pts = np.asarray(cloud_xyz, float)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) > self.max_points:
            pts = pts[self.rng.choice(len(pts), self.max_points, replace=False)]
        x, y, z, roll, pitch, yaw = pose6
        # The transform removes the position and heading only; roll/pitch level the cloud.
        world = pose_matrix(0.0, 0.0, 0.0, roll, pitch, yaw)
        pts_yaw = points_in_yaw_frame(pts, self.base_from_lidar, world)
        grid, valid, counts = height_grid(pts_yaw)
        grid, valid = self.fill_blind(grid, valid)
        scan = conservative_scan(pts_yaw)
        return dict(grid=grid, valid=valid, counts=counts, scan=scan, points_yaw=pts_yaw, n_points=len(pts_yaw))


def flat_observation(body_z_offset=0.27):
    """A synthetic flat, fully known observation (tests without a lidar): ground body_z_offset
    below the body origin, nothing in the body-height band."""
    grid = np.full((13, 9), -float(body_z_offset))
    valid = np.ones((13, 9), bool)
    scan = np.full(72, 8.0)
    return dict(grid=grid, valid=valid, scan=scan, points_yaw=None)


# ----------------------------------------------------------------------------- the core
DEFAULTS = dict(
    control_rate=20.0,
    body_z_offset=0.27,
    lookahead=1.0,
    fuse_frames=3,
    max_step_flat=0.07,
    pose_timeout=0.3,
    obs_timeout=0.5,
    gains=dict(max_forward=0.6, max_lateral=0.2, max_yaw_rate=0.6, lookahead=1.0, lookahead_speed_gain=0.0,
               pivot_threshold_deg=35.0, align_falloff_deg=60.0, brake_distance=0.05),
    runner_params={},
)


@dataclass
class StepResult:
    command: tuple
    owner: str
    gait_request: str | None
    mode: str
    reason: str
    status: dict
    reached: tuple = ()
    finished: bool = False


@dataclass
class RouteBundle:
    route_path: str
    maneuvers_path: str | None = None
    map_surface_path: str | None = None
    profile_path: str | None = None
    warnings: list = field(default_factory=list)

    @classmethod
    def from_dir(cls, d):
        d = Path(d)
        route = next((p for p in (d / "route_rl.json", d / "route_v2.json") if p.exists()), None)
        if route is None:
            cands = sorted(d.glob("route*.json"))
            if not cands:
                raise FileNotFoundError(f"no route_rl.json / route_v2.json in {d}")
            route = cands[0]
        man = d / "maneuvers.json"
        surf = d / "map_surface.npz"
        prof = d / "policy_profile.json"
        return cls(str(route), str(man) if man.exists() else None, str(surf) if surf.exists() else None,
                   str(prof) if prof.exists() else None)


class NavCore:
    def __init__(self, bundle: RouteBundle, cfg: dict | None = None):
        c = dict(DEFAULTS)
        c.update(cfg or {})
        self.cfg = c
        self.rate = float(c["control_rate"])
        self.profile = PolicyProfile.load(bundle.profile_path) if bundle.profile_path else PolicyProfile.default()
        route = RouteV2.load(bundle.route_path)
        g = dict(DEFAULTS["gains"])
        g.update(c.get("gains") or {})
        gains = PursuitGains(
            max_forward=float(g["max_forward"]),
            max_lateral=float(g["max_lateral"]),
            max_yaw_rate=float(g["max_yaw_rate"]),
            lookahead=float(g["lookahead"]),
            lookahead_speed_gain=float(g["lookahead_speed_gain"]),
            pivot_threshold=math.radians(float(g["pivot_threshold_deg"])),
            align_falloff=math.radians(float(g["align_falloff_deg"])),
            brake_distance=float(g["brake_distance"]),
        )
        self.follower = RouteFollowerCore(
            route,
            RouteFollowerConfig(
                body_z_offset=float(c["body_z_offset"]),
                control_rate=self.rate,
                lookahead=float(c["lookahead"]),
                cross_check=CrossCheckConfig(stairs_pitch_deg=25.0),
                grid=LocalGridConfig(max_step_flat=float(c["max_step_flat"]), fuse_frames=int(c["fuse_frames"])),
            ),
            controller=PurePursuitController(gains),
        )
        mans = maneuver_io.load(bundle.maneuvers_path) if bundle.maneuvers_path else []
        surface = MapSurface.load(bundle.map_surface_path) if bundle.map_surface_path else None
        params = RunnerParams.from_profile(self.profile)
        for k, v in (c.get("runner_params") or {}).items():
            if not hasattr(params, k):
                raise ValueError(f"runner_params: unknown parameter {k!r}")
            setattr(params, k, type(getattr(params, k))(v) if not isinstance(getattr(params, k), tuple) else tuple(v))
        self.runner = RouteRunner(self.follower.path, mans, params, surface=surface,
                                  planner=MapPlanner(surface) if surface is not None else None)
        self.route = route
        self.maneuvers = mans
        self.has_surface = surface is not None
        self.n_wp = len(route.waypoints)
        self.reached = 0
        self.last_mode = None
        self.owner_requested = "official"
        self.warnings = [f"{m.id} (s {m.s0:.1f}-{m.s1:.1f}): {w}" for m in mans for w in m.warnings]
        if surface is None:
            self.warnings.append("no map_surface: no obstacle detection or planned detours; the runner holds where it would need one")

    # -------------------------------------------------------------------------------
    def describe(self):
        return dict(waypoints=self.n_wp, maneuvers=len(self.maneuvers), length_m=round(float(self.follower.path.length), 1),
                    map_id=self.route.map_id, profile=self.profile.source, map_surface=self.has_surface,
                    runner_params={k: v for k, v in vars(self.runner.p).items()} if hasattr(self.runner, "p") else {})

    def zero(self, reason, gait_reported=None):
        owner = GAIT_TO_OWNER.get(gait_reported) or self.owner_requested
        return StepResult((0.0, 0.0, 0.0), owner, OWNER_TO_GAIT.get(owner), self.runner.mode.value, reason,
                          dict(reason=reason, owner_requested=owner, gait_reported=gait_reported))

    def step(self, t, pose, obs, gait_reported, obs_age=None):
        """pose = (x, y, z, roll, pitch, yaw, v_forward, yaw_rate) in the route frame;
        obs = dict(grid, valid, scan, points_yaw) or None; gait_reported = 'flat'|'stairs'|None."""
        x, y, z, roll, pitch, yaw, v_fwd, yaw_rate = pose
        fresh = obs is not None and (obs_age is None or obs_age < float(self.cfg["obs_timeout"]))
        grid = obs["grid"] if fresh else None
        valid = obs["valid"] if fresh else None
        scan = obs["scan"] if fresh else None
        points = obs.get("points_yaw") if fresh else None
        owner_reported = GAIT_TO_OWNER.get(gait_reported)
        f = self.follower.step(t, (x, y, z, yaw), grid, valid, scan, None, pitch=pitch, roll=roll)
        reached = tuple(f.reached)
        if reached:
            self.reached += len(reached)
        path = self.follower.path
        s_gate = path.length if self.follower.finished else float(path.waypoint_s[self.follower.cursor])
        out = self.runner.step(NavInput(t, x, y, z, yaw, pitch, roll, yaw_rate, v_fwd, grid, valid, f, s_gate,
                                        owner_reported, points))
        self.owner_requested = out.owner
        log = self.runner.log[-1] if self.runner.log else {}
        changed = out.mode != self.last_mode
        self.last_mode = out.mode
        m = self.runner._zone()
        status = dict(
            mode=out.mode.value, reason=out.reason, maneuver=m.id if m else None,
            cmd=[round(float(v), 3) for v in out.command], owner_requested=out.owner,
            gait_request=OWNER_TO_GAIT.get(out.owner), gait_reported=gait_reported,
            target=f.target_id, s=round(float(f.s), 2) if math.isfinite(f.s) else None, d=round(float(f.d), 2) if math.isfinite(f.d) else None,
            follower=f.status, follower_reason=f.reason, reached=self.reached, total=self.n_wp,
            progress=round(self.reached / self.n_wp, 3), info=out.info, why=log.get("why", "") if changed else "",
            obs_fresh=fresh,
        )
        return StepResult(tuple(float(v) for v in out.command), out.owner, OWNER_TO_GAIT.get(out.owner), out.mode.value,
                          out.reason, status, reached, out.mode == Mode.DONE)


def load_yaml(path):
    import yaml

    with open(path) as f:
        return yaml.safe_load(f) or {}
