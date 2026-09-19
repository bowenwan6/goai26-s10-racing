"""Planning-level full-course harness (pure Python + numpy, no ROS, no MuJoCo dynamics).

    python -m sim_full_course.harness --scenario nominal
    python -m sim_full_course.harness --scenario detour_box --s0 40 --s1 60
    python -m sim_full_course.harness --controller my_pkg.my_mod:MyController

Per tick (dt = 0.1 s): true pose -> synthetic cloud -> real height_grid / conservative_scan
-> controller.step(...) -> native limits + slew -> kinematic move on the heightfield ->
metrics. Outputs <out>/<run_name>/{log.csv, summary.json, topview.png}.
"""

from __future__ import annotations

import argparse
import csv
import importlib
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from sim_full_course.controller import PurePursuitController
from sim_full_course.io_utils import ARTIFACTS, REAL_ROUTE
from sim_full_course.obstacles import Obstacle
from sim_full_course.robot import KinematicRobot, RobotLimits
from sim_full_course.route import PLACEHOLDER_PATH, Route, subroute
from sim_full_course.sensors import REAL_CONTRACTS, SensorConfig, SensorModel
from sim_full_course.terrain import Terrain

TERRAIN_NPZ = ARTIFACTS / "terrain" / "course_terrain.npz"


@dataclass
class RunConfig:
    dt: float = 0.1
    body_z_offset: float = 0.43
    timeout: float | None = None  # default: 3 * length / 0.1 + 120 s
    progress_timeout: float = 60.0  # s without 0.1 m of progress -> "stuck"
    stall_speed: float = 0.02
    stall_timeout: float = 4.0
    missed_wp_margin: float = 1.0  # WP counted missed once progress passes it by this much
    sensor_every: int = 1
    collision_mode: str = "block"  # "block": refuse moves into obstacles; "record": count only
    seed: int = 0
    # Static obstacle cells within this distance of the taught centreline are removed: the
    # mapping robot drove through them (people beside it), and the v3 surface is 0.1-0.3 m
    # thick, which fattens hedges and poles. 0.45 = body half-width 0.25 + 0.2 map noise.
    # Without it the ~1 m gap between a signpost and a hedge before WP03 reads as 0.63 m.
    # 0 keeps them.
    clear_taught_path: float = 0.45
    limits: RobotLimits = field(default_factory=RobotLimits)
    sensors: SensorConfig = field(default_factory=SensorConfig)


class Metrics:
    def __init__(self, route: Route, terrain: Terrain, cfg: RunConfig):
        self.route, self.terrain, self.cfg = route, terrain, cfg
        self.wp_ids = route.wp_order
        self.wp_by_id = {w["id"]: w for w in route.waypoints}
        self.next_wp = 0
        self.reached: list[dict] = []
        self.missed: list[str] = []
        self.lat_abs: list[float] = []
        self.corridor_violations = 0
        self.min_clear_static = math.inf
        self.min_clear_injected = math.inf
        self.collisions = 0
        self.collision_ticks = 0
        self.step_violations = 0
        self.step_violation_ticks = 0
        self.max_step = 0.0
        self.max_unknown_frac = 0.0
        self.stalls = 0
        self.stall_time = 0.0
        self._still = 0.0
        self._in_stall = False
        self._prev_coll = False
        self._prev_stepv = False
        self.gait_switches: list[dict] = []
        self.gait_mismatch_ticks = 0
        self.height_valid = []
        self.scan_known = []
        self.perception_publishable = 0
        self.ticks = 0
        self._df = terrain.obstacle_distance_field()

    def static_clearance(self, px, py):
        iy, ix, inside = self.terrain.cell(px, py)
        d = np.where(inside, self._df[iy, ix], np.inf)
        return float(np.min(d) - self.terrain.res / 2)

    def update(self, t, robot: KinematicRobot, s, lat, ev, obs, switching):
        st = robot.state
        self.ticks += 1
        # Waypoints in order.
        z_ground = st.z - robot.body_z_offset
        while self.next_wp < len(self.wp_ids):
            wid = self.wp_ids[self.next_wp]
            w = self.wp_by_id[wid]
            p = w["position"]
            dxy = math.hypot(st.x - p[0], st.y - p[1])
            if dxy <= w.get("radius_xy", 0.2) and abs(z_ground - p[2]) <= w.get("tol_z", 0.2):
                self.reached.append({"id": wid, "t": round(t, 2), "dxy": round(dxy, 3),
                                     "dz": round(z_ground - p[2], 3)})
                self.next_wp += 1
                continue
            if s > self.route.wp_s[self.next_wp] + self.cfg.missed_wp_margin:
                self.missed.append(wid)
                self.next_wp += 1
                continue
            break
        seg = self.route.segment_at(s)
        self.lat_abs.append(abs(lat))
        if abs(lat) > seg["corridor_half_width"]:
            self.corridor_violations += 1
        if st.gait != seg["gait"] and not switching:
            self.gait_mismatch_ticks += 1
        px, py = robot.footprint_points()
        self.min_clear_static = min(self.min_clear_static, self.static_clearance(px, py))
        for o in self.terrain.injected:
            self.min_clear_injected = min(self.min_clear_injected, float(np.min(o.distance(px, py))))
        coll = bool(ev.get("collision") or ev.get("blocked"))
        self.collision_ticks += coll
        self.collisions += coll and not self._prev_coll
        self._prev_coll = coll
        sv = bool(ev.get("step_violation"))
        self.step_violation_ticks += sv
        self.step_violations += sv and not self._prev_stepv
        self._prev_stepv = sv
        self.max_step = max(self.max_step, ev.get("step", 0.0))
        self.max_unknown_frac = max(self.max_unknown_frac, ev.get("unknown_frac", 0.0))
        speed = math.hypot(st.vx, st.vy)
        if speed < self.cfg.stall_speed and abs(st.wz) < 0.02 and not switching:
            self._still += self.cfg.dt
            if self._still >= self.cfg.stall_timeout:
                if not self._in_stall:
                    self.stalls += 1
                    self._in_stall = True
                self.stall_time += self.cfg.dt
        else:
            self._still = 0.0
            self._in_stall = False
        if ev.get("gait_switched"):
            self.gait_switches.append({"t": round(t, 2), "s": round(s, 2),
                                       "gait": ev["gait_switched"]})
        if obs is not None:
            self.height_valid.append(float(obs["valid"].mean()))
            self.scan_known.append(float(np.isfinite(obs["scan"]).mean()))
            self.perception_publishable += bool(obs["valid"].all() and
                                                np.isfinite(obs["scan"]).all())

    def summary(self):
        n = len(self.wp_ids)
        in_order = [r["id"] for r in self.reached] == [w for w in self.wp_ids if w in
                                                       {r["id"] for r in self.reached}]
        return {
            "waypoints_total": n,
            "waypoints_reached": len(self.reached),
            "waypoints_reached_in_order": bool(in_order),
            "waypoints_missed": self.missed,
            "waypoints_not_attempted": self.wp_ids[self.next_wp:],
            "reached": self.reached,
            "lateral_dev_max_m": round(max(self.lat_abs, default=0.0), 3),
            "lateral_dev_mean_m": round(float(np.mean(self.lat_abs)) if self.lat_abs else 0, 3),
            "corridor_violation_ticks": self.corridor_violations,
            "min_clearance_static_m": _r(self.min_clear_static),
            "min_clearance_injected_m": _r(self.min_clear_injected),
            "collisions": self.collisions,
            "collision_ticks": self.collision_ticks,
            "step_violations": self.step_violations,
            "step_violation_ticks": self.step_violation_ticks,
            "max_footprint_step_m": round(self.max_step, 3),
            "max_footprint_unknown_frac": round(self.max_unknown_frac, 3),
            "stalls": self.stalls,
            "stall_time_s": round(self.stall_time, 1),
            "gait_switches": len(self.gait_switches),
            "gait_switch_log": self.gait_switches,
            "gait_mismatch_ticks": self.gait_mismatch_ticks,
            "height_grid_valid_frac_mean": _r(np.mean(self.height_valid) if self.height_valid
                                              else float("nan")),
            "scan_known_frac_mean": _r(np.mean(self.scan_known) if self.scan_known
                                       else float("nan")),
            "runtime_would_publish_frac": _r(self.perception_publishable / max(
                len(self.height_valid), 1)),
        }


def _r(v, n=3):
    v = float(v)
    return None if not math.isfinite(v) else round(v, n)


def run(route: Route, terrain: Terrain, controller, cfg: RunConfig | None = None,
        out_dir: Path | None = None, name: str = "run", obstacles: list[Obstacle] = (),
        verbose: bool = True, meta: dict | None = None) -> dict:
    cfg = cfg or RunConfig()
    terrain.clear_injected()
    if cfg.clear_taught_path > 0:
        cleared = terrain.clear_static_along(
            np.vstack([np.asarray(sg["centerline"], float) for sg in route.segments]),
            cfg.clear_taught_path)
        if verbose and cleared:
            print(f"[harness] cleared {cleared} static obstacle cells on the taught path")
    terrain.inject(*obstacles)
    p0, h0 = route.point_at(0.0)
    robot = KinematicRobot(terrain, p0[0], p0[1], h0, gait=route.segments[0]["gait"],
                           body_z_offset=cfg.body_z_offset, limits=cfg.limits)
    sensors = SensorModel(terrain, cfg.sensors, seed=cfg.seed)
    metrics = Metrics(route, terrain, cfg)
    controller.reset(route)
    timeout = cfg.timeout or (3 * route.length / 0.1 + 120)
    rows = []
    s_true, lat = route.project((p0[0], p0[1]))[:2]
    best_s, best_s_t = s_true, 0.0
    obs = None
    status, reason = "RUNNING", "timeout"
    t = 0.0
    wall = time.time()
    k = 0
    while t < timeout:
        pose6 = robot.pose6()
        est = sensors.estimated_pose(pose6)
        if k % cfg.sensor_every == 0 or obs is None:
            obs = sensors.observe(pose6, est)
        if hasattr(controller, "on_feedback"):
            controller.on_feedback({"gait": robot.state.gait, "gait_switching": robot.switching})
        vx, vy, wz, gait_req, status = controller.step(
            t, (est[0], est[1], est[2], est[5]), obs["grid"].copy(), obs["valid"].copy(),
            obs["scan"].copy())
        robot.request_gait(gait_req)
        seg = route.segment_at(s_true)
        ev = robot.step((vx, vy, wz), cfg.dt, speed_cap=float(seg["speed_limit"]),
                        block_on_collision=cfg.collision_mode == "block")
        t += cfg.dt
        k += 1
        st = robot.state
        s_true, lat, _ = route.project((st.x, st.y), s_true, back=0.5, ahead=2.0)
        metrics.update(t, robot, s_true, lat, ev, obs, robot.switching)
        rows.append([round(t, 2), st.x, st.y, st.z, st.yaw, st.pitch, st.roll, st.vx, st.vy,
                     st.wz, vx, vy, wz, gait_req or "", st.gait, int(robot.switching), status,
                     s_true, lat, int(ev["collision"]), int(ev["blocked"]),
                     int(ev["step_violation"]), ev["step"], ev["unknown_frac"],
                     float(obs["valid"].mean()), float(np.isfinite(obs["scan"]).mean()),
                     metrics.next_wp])
        if s_true > best_s + 0.1:
            best_s, best_s_t = s_true, t
        if status in ("DONE", "ABORT"):
            reason = status.lower()
            break
        if t - best_s_t > cfg.progress_timeout and not robot.switching:
            reason = "stuck_no_progress"
            break
    summary = {
        "name": name,
        "finish_reason": reason,
        "last_status": status,
        "sim_time_s": round(t, 1),
        "wall_time_s": round(time.time() - wall, 1),
        "route_status": route.data.get("status", ""),
        "route_length_m": round(route.length, 2),
        "progress_m": round(s_true, 2),
        "real_contract_functions": REAL_CONTRACTS,
        "obstacles": [o.to_dict() for o in obstacles],
        **metrics.summary(),
        **(meta or {}),
    }
    if out_dir is not None:
        d = Path(out_dir) / name
        d.mkdir(parents=True, exist_ok=True)
        with open(d / "log.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "z", "yaw", "pitch", "roll", "vx", "vy", "wz", "cmd_vx",
                        "cmd_vy", "cmd_wz", "gait_request", "gait", "gait_switching", "status",
                        "s", "lateral", "collision", "blocked", "step_violation",
                        "footprint_step", "footprint_unknown_frac", "height_valid_frac",
                        "scan_known_frac", "next_wp_index"])
            for r in rows:
                w.writerow([f"{v:.4f}" if isinstance(v, float) else v for v in r])
        (d / "summary.json").write_text(json.dumps(summary, indent=2, default=_json_default))
        plot_run(d / "topview.png", route, terrain, np.array([r[1:3] for r in rows]), summary,
                 obstacles)
    if verbose:
        keys = ["name", "finish_reason", "sim_time_s", "wall_time_s", "progress_m",
                "waypoints_reached", "waypoints_total", "waypoints_missed", "lateral_dev_max_m",
                "lateral_dev_mean_m", "min_clearance_static_m", "min_clearance_injected_m",
                "collisions", "step_violations", "stalls", "gait_switches",
                "runtime_would_publish_frac"]
        print(json.dumps({k: summary[k] for k in keys}, default=_json_default))
    return summary


def _json_default(o):
    if isinstance(o, np.generic):
        return o.item()
    raise TypeError(type(o))


def plot_run(path, route, terrain, xy, summary, obstacles):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon

    pad = 3.0
    allxy = np.vstack([route.xyz[:, :2], xy]) if len(xy) else route.xyz[:, :2]
    x0, y0 = allxy.min(0) - pad
    x1, y1 = allxy.max(0) + pad
    sub = terrain.crop(x0, x1, y0, y1)
    ny, nx = sub.shape
    ext = [sub.origin[0], sub.origin[0] + nx * sub.res, sub.origin[1], sub.origin[1] + ny * sub.res]
    span = max(x1 - x0, y1 - y0)
    fig, ax = plt.subplots(figsize=(14, 14 * (y1 - y0) / max(x1 - x0, 1e-3) if span < 60 else 10),
                           dpi=110)
    g = np.ma.masked_where(~sub.known, sub.ground)
    im = ax.imshow(g, origin="lower", extent=ext, cmap="terrain", interpolation="nearest")
    ax.imshow(np.ma.masked_where(sub.known, np.ones(sub.shape)), origin="lower", extent=ext,
              cmap="gray", vmin=0, vmax=2, alpha=0.6, interpolation="nearest")
    ax.imshow(np.ma.masked_where(sub.obstacle_height <= 0, np.ones(sub.shape)), origin="lower",
              extent=ext, cmap="autumn", alpha=0.9, interpolation="nearest")
    ax.plot(route.xyz[:, 0], route.xyz[:, 1], "-", color="m", lw=1.0, label="route centerline")
    for i, seg in enumerate(route.segments):
        if seg["gait"] == "stairs":
            m = route.seg_of_pt == i
            ax.plot(route.xyz[m, 0], route.xyz[m, 1], "-", color="orange", lw=3, alpha=0.6)
    reached = {r["id"] for r in summary["reached"]}
    for w in route.waypoints:
        p = w["position"]
        c = "lime" if w["id"] in reached else ("red" if w["id"] in summary["waypoints_missed"]
                                               else "white")
        ax.add_patch(Circle((p[0], p[1]), w.get("radius_xy", 0.2), fc=c, ec="k", lw=0.5,
                            zorder=5))
        if len(route.waypoints) <= 60:
            ax.annotate(w["id"], (p[0], p[1]), fontsize=6, xytext=(3, 3),
                        textcoords="offset points")
    if len(xy):
        ax.plot(xy[:, 0], xy[:, 1], "-", color="b", lw=1.2, label="robot (true)")
    for o in obstacles:
        if hasattr(o, "corners"):
            ax.add_patch(Polygon(o.corners(), fc="k", alpha=0.7, zorder=6))
        else:
            ax.add_patch(Circle((o.x, o.y), o.radius, fc="k", alpha=0.7, zorder=6))
    fig.colorbar(im, ax=ax, shrink=0.5, label="ground z (m)")
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.legend(loc="best", fontsize=8)
    ax.set_title(f"{summary['name']}: {summary['finish_reason']}, WP "
                 f"{summary['waypoints_reached']}/{summary['waypoints_total']}, "
                 f"lat max {summary['lateral_dev_max_m']} m, coll {summary['collisions']}, "
                 f"t {summary['sim_time_s']} s\n(lime=reached, red=missed, grey=unknown, "
                 f"orange cells=obstacle layer, orange line=stairs segments)", fontsize=9)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def default_route_path() -> Path:
    return REAL_ROUTE if REAL_ROUTE.exists() else PLACEHOLDER_PATH


def load_controller(spec: str):
    if spec == "pure_pursuit":
        return PurePursuitController()
    mod, _, cls = spec.partition(":")
    return getattr(importlib.import_module(mod), cls)()


def main(argv=None):
    from sim_full_course import scenarios

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--route", type=Path, default=default_route_path(),
                    help="route_v2.json (default: matched route if present, else placeholder)")
    ap.add_argument("--terrain", type=Path, default=TERRAIN_NPZ)
    ap.add_argument("--scenario", default="nominal", choices=sorted(scenarios.SCENARIOS))
    ap.add_argument("--controller", default="pure_pursuit",
                    help="'pure_pursuit' or 'module.path:ClassName' (no-arg constructor)")
    ap.add_argument("--s0", type=float, default=None, help="clip route start (arc length m)")
    ap.add_argument("--s1", type=float, default=None, help="clip route end (arc length m)")
    ap.add_argument("--obstacle-s", type=float, default=None,
                    help="scenario obstacle arc length on the FULL route (m)")
    ap.add_argument("--pose-noise-xy", type=float, default=0.0)
    ap.add_argument("--pose-noise-yaw", type=float, default=0.0)
    ap.add_argument("--dropout", type=float, default=0.10)
    ap.add_argument("--self-occlusion", action="store_true")
    ap.add_argument("--sensor-every", type=int, default=1)
    ap.add_argument("--collision-mode", choices=("block", "record"), default="block",
                    help="block: moves into obstacles are refused (default); record: counted only")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=ARTIFACTS / "runs")
    ap.add_argument("--name", default=None)
    a = ap.parse_args(argv)
    route = Route.load(a.route)
    if a.s0 is not None or a.s1 is not None:
        route = subroute(route, a.s0 or 0.0, a.s1 or route.length)
    terrain = Terrain.load(a.terrain)
    obs_s = a.obstacle_s
    if obs_s is not None and a.s0 is not None:
        obs_s -= a.s0  # --obstacle-s is given in full-route arc length
    obstacles = scenarios.SCENARIOS[a.scenario](route, obs_s)
    cfg = RunConfig(seed=a.seed, sensor_every=a.sensor_every, collision_mode=a.collision_mode)
    cfg.sensors.pose_noise_xy = a.pose_noise_xy
    cfg.sensors.pose_noise_yaw = a.pose_noise_yaw
    cfg.sensors.dropout = a.dropout
    cfg.sensors.self_occlusion = a.self_occlusion
    name = a.name or f"{a.scenario}_{a.controller.split(':')[-1]}" + (
        f"_s{a.s0 or 0:g}-{a.s1 or route.length:.0f}" if a.s0 is not None or a.s1 is not None else "")
    run(route, terrain, load_controller(a.controller), cfg, a.out, name, obstacles,
        meta={"scenario": a.scenario, "controller": a.controller, "route_file": str(a.route),
              "collision_mode": a.collision_mode})


if __name__ == "__main__":
    main()
