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
import os
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from geometry import height_grid, points_in_yaw_frame

from s10_auto_nav.pure_pursuit import PurePursuitController, PursuitGains
from s10_auto_nav.rl_nav import maneuvers as maneuver_io
from s10_auto_nav.rl_nav.capability import PolicyProfile
from s10_auto_nav.rl_nav.map_check import MapSurface
from s10_auto_nav.rl_nav.map_planner import MapPlanner
from s10_auto_nav.rl_nav.route_runner import Mode, NavInput, NavOutput, RouteRunner, RunnerParams
from s10_auto_nav.route_follower import RouteFollowerConfig, RouteFollowerCore
from s10_auto_nav.route_planner import FREE, LocalGrid, LocalGridConfig, RoutePlannerConfig
from s10_auto_nav.route_v2 import CrossCheckConfig, RouteV2

OWNER_TO_GAIT = {"official": "flat", "stairs_stable": "stairs", "fast": "fast", "platform": "platform", "stop": None}
GAIT_TO_OWNER = {"flat": "official", "stairs": "stairs_stable", "fast": "fast", "platform": "platform"}


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
        self.self_box = (0.0, 0.0, -0.30)          # half length, half width (m), keep points below this z
        self.self_clear_radius = 0.0
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
        xs, ys = GRID_X[:n], GRID_Y[:m]
        near = np.hypot(xs[:, None], ys[None, :]) <= r
        idx = distance_transform_edt(~valid, return_distances=False, return_indices=True)
        fill = near & ~valid
        grid = np.array(grid, float)
        grid[fill] = grid[idx[0][fill], idx[1][fill]]
        return grid, valid | fill

    def clear_under_body(self, grid, valid):
        """The ground the robot stands on is traversable by definition: cells within self_clear_radius
        take the median floor height of the rest of the grid. Without this, levelling errors while
        trotting (body pitch, 40 Hz attitude against a 10 Hz cloud) and leg returns make the planner's
        footprint check fail at the robot's own position, which stops it every other second."""
        r = self.self_clear_radius
        if r <= 0 or not valid.any():
            return grid, valid
        n, m = grid.shape
        xs, ys = GRID_X[:n], GRID_Y[:m]
        near = np.hypot(xs[:, None], ys[None, :]) <= r
        ring = valid & ~near
        if ring.sum() < 8:
            return grid, valid
        grid = np.array(grid, float)
        grid[near] = float(np.median(grid[ring]))
        return grid, valid | near

    def observe(self, cloud_xyz, pose6):
        pts = np.asarray(cloud_xyz, float)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) > self.max_points:
            pts = pts[self.rng.choice(len(pts), self.max_points, replace=False)]
        # The robot's own legs and body: returns inside the body box that are not floor. While trotting
        # the swinging legs otherwise show up as obstacles AT the robot ("blocked at 0.00 m").
        bx, by, bz = self.self_box
        if bx > 0:
            own = (np.abs(pts[:, 0]) < bx) & (np.abs(pts[:, 1]) < by) & (pts[:, 2] > bz)
            pts = pts[~own]
        x, y, z, roll, pitch, yaw = pose6
        # The transform removes the position and heading only; roll/pitch level the cloud.
        world = pose_matrix(0.0, 0.0, 0.0, roll, pitch, yaw)
        pts_yaw = points_in_yaw_frame(pts, self.base_from_lidar, world)
        grid, valid, counts = height_grid(pts_yaw)
        grid, valid = self.fill_blind(grid, valid)
        grid, valid = self.clear_under_body(grid, valid)
        scan = conservative_scan(pts_yaw)
        return dict(grid=grid, valid=valid, counts=counts, scan=scan, points_yaw=pts_yaw, n_points=len(pts_yaw))


# Cell centres of geometry.height_grid (13 x 9 cells of 0.15 m; x -0.675..1.275 m, y -0.675..0.675 m).
GRID_X = -0.6 + 0.15 * np.arange(13)
GRID_Y = -0.6 + 0.15 * np.arange(9)


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


LANE_DEFAULTS = dict(enabled=True, half_width=0.10, release=0.05, heading_deg=6.0, heading_gain=0.8, heading_max_wz=0.35,
                     return_gain=0.8, return_max_vy=0.15, steer=True, steer_gain=0.9, steer_max_deg=20.0, steer_vy_share=0.3, handback_d=0.35, handback_heading_deg=25.0, end_zone=1.0, corner_deg=12.0)
SMOOTH_DEFAULTS = dict(enabled=True, accel=0.6, decel=1.0, vy_accel=0.5, wz_accel=1.5, vy_tau=0.25, wz_tau=0.25)


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
        clr = d / "clearance.json"
        if clr.exists():
            try:
                hard = int(json.loads(clr.read_text()).get("hard_violating_cells", 0))
            except (ValueError, TypeError):
                hard = 0
            if hard > 0:
                raise ValueError(f"{d}: clearance.json reports the body on {hard} obstacle cells (tools/verify_route_clearance.py); "
                                 "this route must not be driven")
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
        route = RouteV2.load(self._route_with_speed_override(bundle.route_path, c))
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
        for k in ("yaw_gain", "lateral_gain", "min_speed_fraction", "forward_slew", "yaw_slew", "lateral_slew"):
            if k in g:
                setattr(gains, k, float(g[k]))
        planner = RoutePlannerConfig()
        for k, v in (c.get("planner") or {}).items():      # e.g. w_lat, w_side_switch, switch_margin, astar_*
            if not hasattr(planner, k):
                raise ValueError(f"planner: unknown parameter {k!r}")
            setattr(planner, k, type(getattr(planner, k))(v))
        self.lane = dict(LANE_DEFAULTS)
        self.lane.update(c.get("lane") or {})
        self.smooth = dict(SMOOTH_DEFAULTS)
        self.smooth.update(c.get("smooth") or {})
        self._lane_out = False
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_t = None
        self.follower = RouteFollowerCore(
            route,
            RouteFollowerConfig(
                body_z_offset=float(c["body_z_offset"]),
                control_rate=self.rate,
                # adaptive lookahead (gains.lookahead + lookahead_speed_gain * v) when the speed gain is set
                lookahead=None if float(g["lookahead_speed_gain"]) > 0 else float(c["lookahead"]),
                planner=planner,
                # how far the projection may fall BACK along the route. 1.0 m (upstream default) lets it jump onto
                # the way in at a dead-end turn-around, where both lanes are 0.4 m apart: the robot then spins.
                tracker_back=float(c.get("tracker_back", 1.0)),
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
        # zone_mode "gait_only" (native vendor gaits): a stairs zone only means "stairs gait + its speed cap".
        # The operator walks whole stretches in the stairs gait, turning freely, so the approach / align /
        # climb state machine (made for the RL stairs policy, which needs the stair edge) is not used:
        # the runner stays in WALK and NavCore requests the gait by arc length.
        self.zone_mode = str(c.get("zone_mode", "maneuver"))
        self.zone_blind = bool(c.get("zone_blind", True))
        # terrain.json (tools/teach_line.py): steep stretches (arc length) and waypoints the dog drives INTO (wall /
        # platform edge). zone_speed: speed caps inside a stairs-gait zone, slower where the ground really is steep.
        self.steep, self.contact = [], []
        tj = Path(bundle.route_path).parent / "terrain.json"
        if tj.exists():
            doc_t = json.loads(tj.read_text())
            self.steep = [(float(a_), float(b_)) for a_, b_ in doc_t.get("steep", [])]
            self.contact = [(float(q["xy"][0]), float(q["xy"][1]), float(q.get("radius", 1.0))) for q in doc_t.get("contact", [])]
        # Ledges climbed in the platform gait (terrain.json "jumps": crossing point + direction in MAP coordinates, so a
        # route cut for --from needs no shifting). 2026-09-21: the gap between the steel stools on the ledge at WP17 is
        # about the dog's own width, so the dog arrives slowly on a straight lane and is CHECKED before it takes off.
        self.jump_cfg = dict(enabled=True, approach_speed=0.35, check_at_m=0.7, tol_lateral=0.05, tol_yaw_deg=5.0, backup_m=0.5,
                             backup_speed=0.2, retries=2, hold_beyond_m=0.25, blind_after_m=1.5, pivot_timeout_s=8.0, track=True, track_lookahead=0.5, track_yaw_gain=2.0, track_max_yaw=0.6)
        self.jump_cfg.update(c.get("jump") or {})
        self.jumps, self._jump_override, self._jump_phase, self._track_override = [], False, None, False
        self._plan_k, self._left_k = None, None
        self._pose_log, self._pose_n = None, 0
        if os.environ.get("S10_POSE_LOG_DIR"):                    # set by scripts/start_nav.sh; tests and simulations write nothing
            try:
                import time as _time
                self._pose_log = open(os.path.join(os.environ["S10_POSE_LOG_DIR"], "navpose-%s.csv" % _time.strftime("%Y%m%d-%H%M%S")), "w")
                self._pose_log.write("t,x,y,yaw,s,d,cmd_vx,cmd_vy,cmd_wz,v_fwd,gait,mode\n")
            except OSError:
                self._pose_log = None
        if tj.exists() and self.jump_cfg["enabled"]:
            for q in doc_t.get("jumps", []):
                try:
                    pr_ = self.follower.path.project(np.array([float(q["xy"][0]), float(q["xy"][1])]), None)
                    s_j, d_j = float(pr_.s), abs(float(getattr(pr_, "d", 0.0)))
                except Exception:
                    continue
                if d_j < 0.5 and 0.5 < s_j < self.follower.path.length - 0.5:     # not on this (shortened) route otherwise
                    self.jumps.append(dict(s=s_j, xy=(float(q["xy"][0]), float(q["xy"][1])), yaw=float(q["yaw"]), before=float(q.get("before", 1.5)),
                                           after=float(q.get("after", 1.0)), phase="approach", tries=0, cleared=False, t0=None, back_to=None))
        self.jumps.sort(key=lambda j_: j_["s"])
        for k_, j_ in enumerate(self.jumps):                      # slow, precisely tracked approach: from 1 m before the straight lane,
            j_["s_from"] = j_["s"] - j_["before"] - 1.0           # or from the landing of a ledge less than 8 m back (S-bend WP24 -> WP26)
            if k_ and j_["s"] - self.jumps[k_ - 1]["s"] < 8.0:
                j_["s_from"] = self.jumps[k_ - 1]["s"] + self.jumps[k_ - 1]["after"]
        zs_ = c.get("zone_speed") or {}
        self.zone_v = (float(zs_.get("cruise", 0.0)), float(zs_.get("steep", 0.0)))
        # Gait plan (terrain.json "gaits": [[s0, s1, "fast"|"stairs"|"platform"], ...]; everything else = "flat", the
        # navigation walk). With a plan the walk is the SLOW gait (zone_speed.walk) and "fast" (0xF002) runs at the
        # run's --speed; "platform" (0x1002) climbs a ledge at zone_speed.platform. Without a plan: two gaits as before.
        self.gait_plan = []
        if tj.exists():
            self.gait_plan = [(float(a_), float(b_), str(n_)) for a_, b_, n_ in doc_t.get("gaits", []) if n_ in ("fast", "stairs", "platform")]
        self.walk_v, self.platform_v = float(zs_.get("walk", 0.0)), float(zs_.get("platform", 0.4))   # walk 0 = no cap: the run's --speed
        self.gait_zones = [(float(m.s0), float(m.s1)) for m in mans] if self.zone_mode == "gait_only" else []
        if self.zone_mode == "gait_only" and self.gait_plan:      # blind + zone speed wherever the plan says stairs / platform
            self.gait_zones = [(a_, b_) for a_, b_, n_ in self.gait_plan if n_ in ("stairs", "platform")]
        # Zone ENTRY (2026-09-21 15:51 / 15:54, --speed 1.67: the dog reached the first stairs zone at 1.5 m/s, ran 1.1 m
        # past its start onto the steps in the walk gait, and the robot then never confirmed the stairs gait: latched
        # stop). The first version did not have this because its stairs segments were limited to 0.30 m/s, a limit
        # strategy1 removed without replacing the braking it gave. Now: every zone starts lead_m earlier, and the speed
        # before it follows v^2 = entry_speed^2 + 2 * decel * distance, so the dog ARRIVES slowly whatever --speed is.
        ze_ = c.get("zone_entry") or {}
        self.entry_lead, self.entry_v, self.entry_decel = float(ze_.get("lead_m", 0.4)), float(ze_.get("entry_speed", 0.3)), float(ze_.get("decel", 0.8))
        def _lead(zs):
            out, prev_end = [], 0.0
            for z_ in sorted(zs, key=lambda q: q[0]):
                out.append((max(prev_end, z_[0] - self.entry_lead),) + tuple(z_[1:])); prev_end = z_[1]
            return out
        self.gait_plan = _lead(self.gait_plan)
        self.gait_zones = _lead(self.gait_zones)
        # In a blind zone the local planner gets an all-free grid: what it fuses from the small 13 x 9 patch is
        # too little for its footprint sweep (any bend pushes the footprint into "unknown" = blocked).
        self._blind_now = False
        _build = self.follower.grid_builder.build
        _gc = self.follower.grid_builder.config

        def _build_or_free(center_xy, *args, **kwargs):
            if self._blind_now:
                return LocalGrid.centred(center_xy, _gc.size, _gc.resolution, fill=FREE)
            return _build(center_xy, *args, **kwargs)
        self.follower.grid_builder.build = _build_or_free
        # The team planner slows down (to 50 % at most) whenever obstacle OR UNKNOWN cells are near the body. Our live
        # grid only reaches ~0.9 m and its rim is unknown, so that factor is below 1 even on an open road. While the
        # plan is the taught line itself (verified offline by the body sweep) the scale is set to planner_scale_on_line;
        # real detours, A* and BLOCKED keep the planner's own value. 0 = the planner's behaviour unchanged.
        self.scale_on_line = float(c.get("planner_scale_on_line", 1.0))
        _plan = self.follower.planner.plan

        def _plan_full_speed_on_line(*args, **kwargs):
            r = _plan(*args, **kwargs)
            if self.scale_on_line > 0.0 and r.status in ("TRACK", "DETOUR") and abs(r.d_target) < 1e-6:
                # On the taught line only the PROXIMITY rule slows the robot: obstacle or unknown cells within
                # planner.inflation (0.20 m) of the body -> proportionally slower, at most half. The "free length
                # seen ahead" taper is dropped here (the 0.9 m grid cannot see far enough for it to mean anything).
                import dataclasses
                prox = next((float(c_.proximity) for c_ in r.candidates if getattr(c_, "valid", True) and abs(c_.d_target - r.d_target) < 1e-6), 0.0)
                want = self.scale_on_line * (1.0 - 0.5 * float(np.clip(prox, 0.0, 1.0)))
                if want > r.speed_scale:
                    r = dataclasses.replace(r, speed_scale=want)
            return r
        self.follower.planner.plan = _plan_full_speed_on_line
        self.runner = RouteRunner(self.follower.path, [] if self.zone_mode == "gait_only" else mans, params, surface=surface,
                                  planner=MapPlanner(surface) if surface is not None else None)
        self._install_readjust(c)
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

    def _install_readjust(self, c):
        """READJUST instead of HOLD (operator, 2026-09-21 18:40). The team runner (rl_nav/route_runner.py, _plan) holds
        whenever a recovery needs a planning map: "0.8 m off the route", "no progress for 8 s", a gate it may not
        approach. No route of ours has a map_surface.npz, so every such case ended the run. The team copy is not
        edited; its instance is wrapped here:
          * without a planner, a rejoin request puts the runner in TRACK (its own pursuit of the taught line) instead
            of HOLD. A request about a NEW OBSTACLE still holds.
          * TRACK used to command forward speed walk_floor = 0 with a 0.25 m/s side-step only, which the dog does not
            do in the stairs gait on steps (18:31 run: 8 s, offset unchanged). Now it walks forward at readjust.speed
            and STEERS back to a carrot on the line.
          * safety stays: farther than readjust.hold_beyond_m from the line, or readjust.timeout_s in TRACK without
            0.5 m of progress -> HOLD."""
        import types
        rj = dict(enabled=True, speed=0.5, lookahead=1.2, max_yaw=0.8, lateral=0.15, hold_beyond_m=1.5, timeout_s=20.0)
        rj.update(c.get("readjust") or {})
        self.readjust = rj
        if not rj["enabled"]:
            return
        runner = self.runner
        team_plan, team_pursue = runner._plan, runner._pursue
        state = dict(t0=None, s0=0.0)
        self._readjust_state = state

        def plan(_self, inp, s_, why, kind="rejoin"):
            if _self.planner is not None or kind == "obstacle":
                return team_plan(inp, s_, why, kind)
            if abs(getattr(_self, "d", 0.0)) > float(rj["hold_beyond_m"]):
                _self._set(Mode.HOLD, inp.t, s_, f"{why}; more than {rj['hold_beyond_m']} m from the line")
                return
            if state["t0"] is not None and inp.t - state["t0"] > float(rj["timeout_s"]) and s_ < state["s0"] + 0.5:
                _self._set(Mode.HOLD, inp.t, s_, f"{why}; readjusting for {rj['timeout_s']:.0f} s without progress")
                return
            _self.best_s_t = inp.t                          # the stall clock restarts: the dog is being driven back
            if _self.mode != Mode.TRACK:
                if state["t0"] is None or s_ > state["s0"] + 0.5:
                    state["t0"], state["s0"] = inp.t, s_
                _self._set(Mode.TRACK, inp.t, s_, f"readjust: {why}")

        def pursue(_self, s_, inp, v, wmax, lookahead=0.8, lateral=0.0):
            if _self.mode == Mode.TRACK:                     # steer back while walking, not a side-step on the spot
                return team_pursue(s_, inp, max(v, float(rj["speed"])), max(wmax, float(rj["max_yaw"])),
                                   max(lookahead, float(rj["lookahead"])), min(lateral, float(rj["lateral"])) if lateral else 0.0)
            return team_pursue(s_, inp, v, wmax, lookahead, lateral)

        runner._plan = types.MethodType(plan, runner)
        runner._pursue = types.MethodType(pursue, runner)

    def _route_observation(self, x, y, z, yaw):
        """Perception replaced by the route's own height profile: every grid cell gets the height of the
        taught line next to it. On a slope or on steps a FLAT synthetic grid would contradict the rising
        route and block it; this one agrees with it by construction."""
        path = self.follower.path
        c, s_ = math.cos(yaw), math.sin(yaw)
        gx, gy = np.meshgrid(GRID_X, GRID_Y, indexing="ij")
        pts = np.c_[(x + gx * c - gy * s_).ravel(), (y + gx * s_ + gy * c).ravel()]
        s0 = getattr(self, "_last_s", None)
        s0 = float(path.project(np.array([x, y]), None).s) if s0 is None else s0
        ss, _ = path.project_many(pts, max(0.0, s0 - 1.5), min(float(path.length), s0 + 2.5))
        zc = np.asarray(path.point_at(ss))[:, 2]
        grid = (zc - z).reshape(len(GRID_X), len(GRID_Y))
        return grid, np.ones(grid.shape, bool), np.full(72, 8.0), None

    def _in_gait_zone(self):
        """Hysteresis-free by construction: zones are arc-length intervals on the route and the
        projection s only moves forward."""
        if not self.gait_zones:
            return False
        s = getattr(self, "_last_s", None)
        return s is not None and any(s0 <= s <= s1 for s0, s1 in self.gait_zones)

    def _lane(self, out, f, x, y, yaw):
        """Bowling lane. While the planner wants the taught line itself (no obstacle offset) and the
        runner is simply walking, the body within +-half_width of the line and roughly along it gets
        NO lateral or heading correction: it walks straight. Outside, the correction grows smoothly
        from the lane edge (not from the line) and stops again at `release`; far outside, or badly
        mis-headed, the follower's own command is used unchanged."""
        L = self.lane
        vx, vy, wz = (float(v) for v in out.command)
        if not L.get("enabled", True) or out.mode != Mode.WALK or (out.owner != "official" and self.zone_mode != "gait_only"):
            self._lane_out = False
            return (vx, vy, wz), "off"
        plan = getattr(f, "plan", None)
        if f.status not in ("RUNNING", "DETOUR", "TRACK") or plan is None or abs(getattr(plan, "d_target", 0.0)) > 1e-6 \
                or not math.isfinite(f.s) or not math.isfinite(f.d):
            self._lane_out = False
            return (vx, vy, wz), "follower"
        path = self.follower.path
        if path.length - f.s < float(L["end_zone"]):          # final approach: the follower brakes and aligns
            return (vx, vy, wz), "end"
        d = float(f.d)                                        # + = left of travel
        wrap = lambda a_: math.atan2(math.sin(a_), math.cos(a_))
        herr = wrap(float(path.tangent_yaw_at(f.s)) - yaw)
        # A corner ahead is the follower's business: the lane only straightens what is straight. (Without this
        # the lane held heading on the current tangent while the follower wanted to turn: a standstill.)
        ahead = max(abs(wrap(float(path.tangent_yaw_at(min(path.length, f.s + ds_))) - float(path.tangent_yaw_at(f.s)))) for ds_ in (0.4, 0.8, 1.2))
        pivoting = vx < 0.05 and abs(wz) > 0.15
        if pivoting or ahead > math.radians(float(L["corner_deg"])):
            self._lane_out = False
            return (vx, vy, wz), "corner"
        if abs(d) > float(L["handback_d"]) or abs(herr) > math.radians(float(L["handback_heading_deg"])):
            self._lane_out = True
            return (vx, vy, wz), "follower"
        half, release = float(L["half_width"]), float(L["release"])
        self._lane_out = abs(d) > (release if self._lane_out else half)
        hdead = math.radians(float(L["heading_deg"]))
        h_excess = math.copysign(max(0.0, abs(herr) - hdead), herr)
        wz_new = float(np.clip(float(L["heading_gain"]) * h_excess, -float(L["heading_max_wz"]), float(L["heading_max_wz"])))
        if not self._lane_out:
            return (vx, 0.0, wz_new), "in"
        e = math.copysign(abs(d) - release, d)                # smooth: zero at the release line
        vy_new = float(np.clip(-float(L["return_gain"]) * e, -float(L["return_max_vy"]), float(L["return_max_vy"])))
        if L.get("steer", True):
            # Come back by STEERING (Stanley): aim the body a few degrees towards the line, more when slow or far
            # out, and keep walking forward; only a share of the sideways step remains. A legged robot shuffles and
            # slows down when it has to side-step at speed.
            delta = math.atan2(float(L["steer_gain"]) * e, max(vx, 0.3))
            delta = float(np.clip(delta, -math.radians(float(L["steer_max_deg"])), math.radians(float(L["steer_max_deg"]))))
            wz_new = float(np.clip(float(L["heading_gain"]) * wrap(herr - delta), -float(L["heading_max_wz"]), float(L["heading_max_wz"])))
            return (vx, vy_new * float(L["steer_vy_share"]), wz_new), "steer"
        return (vx, vy_new, wz_new), "return"

    def _jump_step(self, t, x, y, yaw, s_now, cmd):
        """Before a ledge: slow straight approach, then ONE check of the body against the jump lane (sideways offset and
        heading) check_at_m before the ledge. Heading off -> pivot on the spot. Sideways off -> back up backup_m along the
        lane and come again (the lane steering has that distance to pull the body in), `retries` times; still off ->
        HOLD, because going on means hitting what stands beside the gap. Returns (command, reason); the SAME command
        object when it does not interfere."""
        self._jump_override, self._jump_phase = False, None
        if not self.jumps or s_now is None or self.runner.mode != Mode.WALK:
            return cmd, None
        J = next((j for j in self.jumps if j["s_from"] <= s_now <= j["s"] + 0.3 and not j["cleared"]), None)
        if J is None:
            return cmd, None
        jp = self.jump_cfg
        u = (math.cos(J["yaw"]), math.sin(J["yaw"]))
        su = (x - J["xy"][0]) * u[0] + (y - J["xy"][1]) * u[1]            # along the lane, 0 = the ledge
        sv = -(x - J["xy"][0]) * u[1] + (y - J["xy"][1]) * u[0]           # + = left of the lane
        herr = math.atan2(math.sin(J["yaw"] - yaw), math.cos(J["yaw"] - yaw))
        tol_d, tol_h, check = float(jp["tol_lateral"]), math.radians(float(jp["tol_yaw_deg"])), float(jp["check_at_m"])
        self._jump_phase = J["phase"]
        if hasattr(self.runner, "best_s_t") and J["phase"] != "approach":
            self.runner.best_s_t = t                                       # standing / backing here is not "no progress"

        def decide():
            if abs(sv) <= tol_d and abs(herr) <= tol_h:
                J["cleared"] = True; J["phase"] = "go"; return None
            if abs(sv) <= tol_d:
                J["phase"], J["t0"] = "pivot", t; return None
            if J["tries"] < int(jp["retries"]):
                J["tries"] += 1; J["phase"], J["t0"], J["back_to"] = "back", t, su - float(jp["backup_m"]); return None
            if abs(sv) <= float(jp["hold_beyond_m"]):              # close enough: take off rather than end the run
                J["cleared"] = True; J["phase"] = "go"; return None
            return "jump lane: %.2f m beside the lane, heading %.0f deg off after %d tries (limits %.2f m / %.0f deg, hold beyond %.2f m)" % (
                sv, math.degrees(herr), J["tries"], tol_d, math.degrees(tol_h), float(jp["hold_beyond_m"]))

        hold = None
        if J["phase"] == "approach":
            if su > -check + 0.4:                                          # started past the check point (--from here): not ours
                J["cleared"] = True
                return cmd, None
            if su >= -check:
                hold = decide()
            elif su < 0.0 and cmd[0] > float(jp["approach_speed"]):
                k_ = float(jp["approach_speed"]) / cmd[0]
                return (cmd[0] * k_, cmd[1] * k_, cmd[2]), None
        if J["phase"] == "pivot":
            if abs(herr) <= 0.6 * tol_h or t - J["t0"] > float(jp["pivot_timeout_s"]):
                J["phase"] = "approach"; hold = decide()
        elif J["phase"] == "back":
            if su <= J["back_to"] or t - J["t0"] > 8.0:
                J["phase"] = "approach"
        self._jump_phase = J["phase"]
        if hold:
            self.runner._set(Mode.HOLD, t, s_now, hold)
            self._jump_override = True
            return (0.0, 0.0, 0.0), hold
        if J["phase"] == "pivot":
            self._jump_override = True
            return (0.0, 0.0, float(np.clip(1.5 * herr, -0.5, 0.5))), "jump lane: turning square to the ledge (%.0f deg off)" % math.degrees(herr)
        if J["phase"] == "back":
            self._jump_override = True
            return (-float(jp["backup_speed"]), 0.0, float(np.clip(1.5 * herr, -0.4, 0.4))), "jump lane: %.2f m beside the lane, backing up %.1f m to come again (try %d)" % (sv, float(jp["backup_m"]), J["tries"])
        if J["phase"] == "go" and cmd[0] > float(jp["approach_speed"]) and su < 0.0:
            k_ = float(jp["approach_speed"]) / cmd[0]
            return (cmd[0] * k_, cmd[1] * k_, cmd[2]), None
        return cmd, None

    def _smooth(self, t, command, moving=True):
        """Acceleration limits on all three axes plus a first-order low pass on lateral and yaw.
        Stops are not delayed beyond `decel`; HOLD / DONE / WAIT pass straight through."""
        S = self.smooth
        if not S.get("enabled", True) or not moving or self._cmd_t is None or t <= self._cmd_t or t - self._cmd_t > 0.5:
            self._cmd, self._cmd_t = tuple(float(v) for v in command), t
            return self._cmd
        dt = t - self._cmd_t
        pvx, pvy, pwz = self._cmd
        vx, vy, wz = (float(v) for v in command)
        a = float(S["accel"]) if abs(vx) > abs(pvx) else float(S["decel"])
        vx = pvx + float(np.clip(vx - pvx, -a * dt, a * dt))
        for_lp = lambda prev, new, tau: prev + (new - prev) * (dt / (tau + dt)) if tau > 0 else new
        vy = for_lp(pvy, vy, float(S["vy_tau"]))
        wz = for_lp(pwz, wz, float(S["wz_tau"]))
        vy = pvy + float(np.clip(vy - pvy, -float(S["vy_accel"]) * dt, float(S["vy_accel"]) * dt))
        wz = pwz + float(np.clip(wz - pwz, -float(S["wz_accel"]) * dt, float(S["wz_accel"]) * dt))
        self._cmd, self._cmd_t = (vx, vy, wz), t
        return self._cmd

    @staticmethod
    def _route_with_speed_override(route_path, c):
        """flat_speed_override / stairs_speed_override (m/s) replace every segment's speed_limit, so
        the run's speed is ONE number whatever the route was built with. 0 / missing = keep the file."""
        flat, stairs = float(c.get("flat_speed_override") or 0.0), float(c.get("stairs_speed_override") or 0.0)
        if (c.get("zone_speed") or {}).get("cruise") and flat > 0.0:
            stairs = flat                                  # the stairs-gait caps are applied by arc length (zone_speed), not per segment
        if flat <= 0.0 and stairs <= 0.0:
            return route_path
        import json
        import tempfile
        with open(route_path) as f:
            doc = json.load(f)
        for seg in doc.get("segments", []):
            v = stairs if seg.get("gait") == "stairs" else flat
            if v > 0.0:
                seg["speed_limit"] = v
        tmp = tempfile.NamedTemporaryFile("w", suffix=".route_v2.json", delete=False)
        json.dump(doc, tmp)
        tmp.close()
        return tmp.name

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
        if self.cfg.get("ignore_pose_z", False):
            # x_nav's height is not reliable (the same spot read 0.5 m apart on 2026-09-20), and the follower
            # drops the route when the height disagrees by 0.6 m. Single-level course: take the height from
            # the route at the last known arc length, so only x, y and heading come from the localisation.
            path = self.follower.path
            s_ref = getattr(self, "_last_s", None)
            if s_ref is None:
                s_ref = float(path.project(np.array([x, y]), None).s)
            z = float(np.asarray(path.point_at(s_ref)).ravel()[2]) + float(self.cfg["body_z_offset"])
        in_zone = self._in_gait_zone()
        # blind = inside a gait zone, or on a segment marked stairs (detours are off there and the real height
        # grid would stop the robot at the first riser a metre before the zone starts)
        s_now = getattr(self, "_last_s", None)
        on_stairs_segment = self.zone_mode == "gait_only" and s_now is not None and self.follower.path.gait_at(s_now) == "stairs"
        at_contact = any(math.hypot(x - cx, y - cy) < cr for cx, cy, cr in self.contact)   # the wall there is the target
        after_jump = s_now is not None and any(j_["s"] - 0.5 <= s_now <= j_["s"] + j_["after"] + float(self.jump_cfg.get("blind_after_m", 1.5)) + 1.2 for j_ in self.jumps)
        self._blind_now = bool((self.zone_blind and (in_zone or on_stairs_segment)) or at_contact or after_jump)
        if self._blind_now:
            # Stairs / rough ground: risers and slopes look like obstacles to the flat-ground height grid and
            # to the body-height scan. The taught line was walked by the operator, so follow it as taught.
            grid, valid, scan, points = self._route_observation(x, y, z, yaw)
        f = self.follower.step(t, (x, y, z, yaw), grid, valid, scan, None, pitch=pitch, roll=roll)
        if math.isfinite(f.s):
            self._last_s = float(f.s)
        reached = tuple(f.reached)
        if reached:
            self.reached += len(reached)
        path = self.follower.path
        s_gate = path.length if self.follower.finished else float(path.waypoint_s[self.follower.cursor])
        out = self.runner.step(NavInput(t, x, y, z, yaw, pitch, roll, yaw_rate, v_fwd, grid, valid, f, s_gate,
                                        owner_reported, points))
        if self.zone_mode == "gait_only":
            want = "stairs_stable" if in_zone else "official"
            cmd = out.command
            planned = None
            if self.gait_plan and s_now is not None:
                # The plan only moves FORWARD (2026-09-21 23:57: standing for the switch on a zone boundary, s rocked back
                # over it: 7 switches stairs <-> platform in 17 s after the WP26 ledge). Back only if s really went back 2 m.
                k_now = next((k_ for k_, (a_, b_, n_) in enumerate(self.gait_plan) if a_ <= s_now <= b_), None)
                k_prev, k_left = self._plan_k, self._left_k
                if k_prev is not None and (k_now is None or k_now < k_prev) and self.gait_plan[k_prev][0] - 2.0 < s_now <= self.gait_plan[k_prev][1]:
                    k_now = k_prev                                # rocked back over the START of the current zone: stay in it
                elif k_prev is not None and k_now is None and s_now > self.gait_plan[k_prev][1]:
                    k_left = k_prev                               # left the zone at its far end
                if k_now is not None and k_now == k_left and s_now > self.gait_plan[k_left][1] - 2.0 and k_prev is None:
                    k_now = None                                  # rocked back over the END of the zone just left: stay out
                self._plan_k, self._left_k = k_now, k_left
                planned = self.gait_plan[k_now][2] if k_now is not None else "flat"
                if any(j_["tries"] > 0 and not j_["cleared"] for j_ in self.jumps):
                    planned = "platform"                          # backing up 0.5 m leaves the short platform zone: no switch back and forth
                want = GAIT_TO_OWNER[planned]
            if s_now is not None and self.entry_decel > 0.0:       # brake BEFORE the next gait change (2026-09-21: also from
                # INSIDE a zone: stairs -> platform would otherwise arrive at the climb speed, a metre before a ledge)
                if self.gait_plan:
                    ahead_ = [a_ - s_now for a_, _b, n_ in self.gait_plan if a_ > s_now and n_ != planned]
                else:
                    ahead_ = [] if in_zone else [a_ - s_now for a_, _b in self.gait_zones if a_ > s_now]
                if ahead_:
                    v_in = math.sqrt(self.entry_v ** 2 + 2.0 * self.entry_decel * min(ahead_))
                    if cmd[0] > v_in:
                        cmd = (v_in, cmd[1] * v_in / cmd[0], cmd[2])
                        out = NavOutput(cmd, out.owner, out.mode, out.reason, out.info)
            if planned in ("flat", "platform"):
                cap = self.walk_v if planned == "flat" else self.platform_v
                if cap > 0.0 and cmd[0] > cap:
                    cmd = (cap, cmd[1] * cap / cmd[0], cmd[2])
                    out = NavOutput(cmd, out.owner, out.mode, out.reason, out.info)
            elif in_zone and s_now is not None:
                cap = self.zone_v[1] if (self.zone_v[1] > 0.0 and any(a_ <= s_now <= b_ for a_, b_ in self.steep)) else self.zone_v[0]   # steep 0 = no separate cap
                if cap > 0.0 and cmd[0] > cap:
                    k_ = cap / cmd[0]
                    cmd = (cap, cmd[1] * k_, cmd[2])
                    out = NavOutput(cmd, out.owner, out.mode, out.reason, out.info)
            # Platform zones are walked at 0.4 m/s towards a gap the width of the body: there the carrot of the team
            # follower (1.0 m + 0.8 s * v, yaw gain 1.0) is too lazy: in the simulation it cut the S-bend between the two
            # ledges WP24 / WP26 by 0.6 m and reached the second lane 0.5 m off. Short carrot, firm yaw, no side-step.
            self._track_override = False
            jp_ = self.jump_cfg
            near_jump_ = s_now is not None and any(j_["s_from"] <= s_now <= j_["s"] + j_["after"] for j_ in self.jumps)
            if near_jump_ and cmd[0] > float(jp_["approach_speed"]) and planned != "platform":
                k_ = float(jp_["approach_speed"]) / cmd[0]
                cmd = (cmd[0] * k_, cmd[1] * k_, cmd[2])
                out = NavOutput(cmd, out.owner, out.mode, out.reason, out.info)
            if (planned == "platform" or near_jump_) and jp_.get("track", True) and out.mode == Mode.WALK and s_now is not None and cmd[0] > 0.05:
                path_ = self.follower.path
                P_ = np.asarray(path_.point_at(min(path_.length, s_now + float(jp_.get("track_lookahead", 0.5))))).ravel()
                err_ = math.atan2(P_[1] - y, P_[0] - x) - yaw
                err_ = math.atan2(math.sin(err_), math.cos(err_))
                w_ = float(np.clip(float(jp_.get("track_yaw_gain", 2.0)) * err_, -float(jp_.get("track_max_yaw", 0.6)), float(jp_.get("track_max_yaw", 0.6))))
                cmd = (cmd[0] * max(0.4, math.cos(err_)), 0.0, w_)
                out = NavOutput(cmd, out.owner, out.mode, out.reason, out.info)
                self._track_override = True
            cmd_j, why_j = self._jump_step(t, x, y, yaw, s_now, cmd)
            if cmd_j is not cmd:
                cmd = cmd_j
                out = NavOutput(cmd, out.owner, out.mode, why_j or out.reason, out.info)
            if owner_reported is not None and owner_reported != want:
                cmd = (0.0, 0.0, 0.0)                 # stand still while s10_ros1_control switches the gait
            out = NavOutput(cmd, want, out.mode, out.reason if cmd is out.command else "waiting for the %s gait" % OWNER_TO_GAIT[want], out.info)
        # Regulated speed for corners: above corner_v_ref the forward speed drops with the turn the line makes within
        # the next ~1.5 s of travel (a 1.4 m/s dog cannot take a 90 deg corner the way a 0.5 m/s dog can).
        cs_ = self.cfg.get("corner_speed") or {}
        if cs_.get("enabled", True) and s_now is not None and out.command[0] > float(cs_.get("v_ref", 0.6)):
            path_ = self.follower.path
            wrap_ = lambda a_: math.atan2(math.sin(a_), math.cos(a_))
            reach_ = max(1.0, 1.5 * float(out.command[0]))
            th0_ = float(path_.tangent_yaw_at(s_now))
            turn_ = max(abs(wrap_(float(path_.tangent_yaw_at(min(path_.length, s_now + q_ * reach_))) - th0_)) for q_ in (0.33, 0.66, 1.0))
            herr_ = abs(wrap_(th0_ - yaw))                       # not yet pointing along the line (after a pivot / gait switch)
            turn_ = max(turn_, 2.0 * herr_)
            k_ = float(np.clip(1.0 - turn_ / math.radians(float(cs_.get("full_stop_deg", 90.0))), 0.0, 1.0))
            v_cap = float(cs_.get("v_ref", 0.6)) + (out.command[0] - float(cs_.get("v_ref", 0.6))) * k_
            if v_cap < out.command[0]:
                c_ = out.command
                out = NavOutput((v_cap, c_[1] * v_cap / c_[0], c_[2]), out.owner, out.mode, out.reason, out.info)
        self.owner_requested = out.owner
        command, lane_state = (tuple(out.command), "jump" if self._jump_override else "platform") if (self._jump_override or getattr(self, "_track_override", False)) else self._lane(out, f, x, y, yaw)
        command = self._smooth(t, command, moving=out.mode not in (Mode.HOLD, Mode.DONE, Mode.WAIT))
        out = NavOutput(command, out.owner, out.mode, out.reason, out.info) if hasattr(out, "info") else out
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
            lane=lane_state, obs_fresh=fresh, jump=self._jump_phase,
            blocked=[f"{c.d_target:+.1f}:{c.reason}" for c in (f.plan.candidates if getattr(f, "plan", None) is not None else ())
                     if not c.valid][:5] if f.status == "BLOCKED" else None, speed_limit=getattr(f, "speed_limit", None),
            plan_scale=round(float(f.plan.speed_scale), 2) if getattr(f, "plan", None) is not None else None,
        )
        if self._pose_log is not None:                           # 10 Hz: what the dog DID against what it was told (yaw response)
            self._pose_n += 1
            if self._pose_n % 2 == 0:
                try:
                    self._pose_log.write("%.2f,%.3f,%.3f,%.4f,%s,%s,%.3f,%.3f,%.3f,%.3f,%s,%s\n" % (
                        t, x, y, yaw, status["s"], status["d"], out.command[0], out.command[1], out.command[2], float(v_fwd or 0.0), gait_reported, out.mode.value))
                    if self._pose_n % 40 == 0: self._pose_log.flush()
                except Exception:                                  # noqa: BLE001  (a log must never stop a run)
                    self._pose_log = None
        return StepResult(tuple(float(v) for v in out.command), out.owner, OWNER_TO_GAIT.get(out.owner), out.mode.value,
                          out.reason, status, reached, out.mode == Mode.DONE)


def load_yaml(path):
    import yaml

    with open(path) as f:
        return yaml.safe_load(f) or {}
