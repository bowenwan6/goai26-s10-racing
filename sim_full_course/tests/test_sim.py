"""Unit tests on synthetic terrains (no map artifacts needed) + optional artifact checks."""

from __future__ import annotations

import math

import numpy as np
import pytest

from sim_full_course import _contract_copies as copies
from sim_full_course.controller import PurePursuitController
from sim_full_course.harness import TERRAIN_NPZ, RunConfig, run
from sim_full_course.obstacles import Box, Cylinder, blocked_corridor, place_on_route
from sim_full_course.robot import KinematicRobot
from sim_full_course.route import PLACEHOLDER_PATH, Route, subroute
from sim_full_course.sensors import (
    REAL_CONTRACTS,
    SensorConfig,
    SensorModel,
    conservative_scan,
    height_grid,
    points_in_yaw_frame,
    pose_to_matrix,
    raycast,
)
from sim_full_course.terrain import Terrain


def straight_route(length=8.0, gait="flat", y=0.0, z=0.0, n_wp=3):
    xs = np.linspace(0, length, n_wp)
    wps = [{"id": f"WP{i + 1:02d}", "position": [float(x), y, z], "yaw": 0.0, "radius_xy": 0.2,
            "tol_z": 0.2, "terrain": "test", "confidence": "high"} for i, x in enumerate(xs)]
    segs = []
    for i in range(n_wp - 1):
        cl = [[float(x), y, z] for x in np.arange(xs[i], xs[i + 1] + 1e-9, 0.2)]
        segs.append({"id": f"s{i}", "from": wps[i]["id"], "to": wps[i + 1]["id"], "gait": gait,
                     "speed_limit": 0.2 if gait == "flat" else 0.15,
                     "allow_detour": gait == "flat",
                     "corridor_half_width": 0.8 if gait == "flat" else 0.25, "centerline": cl})
    return Route({"schema": "s10_route_v2", "frame": "map", "status": "TEST",
                  "waypoints": wps, "segments": segs})


def stair_terrain(riser=0.15, tread=0.30, x_start=3.0, n=5):
    t = Terrain.flat(size_x=20, size_y=6, origin=(-5, -3))
    ny, nx = t.shape
    cx = t.origin[0] + (np.arange(nx) + 0.5) * t.res
    g = np.clip(np.floor((cx - x_start) / tread) + 1, 0, n) * riser
    return Terrain(np.tile(g, (ny, 1)), t.known, t.obstacle_height, t.origin, t.res)


# ---------------------------------------------------------------- heightfield sampling
def test_ground_bilinear_and_cells():
    t = Terrain.flat(size_x=4, size_y=4, origin=(0, 0))
    g = t.ground.copy()
    ny, nx = g.shape
    xx = t.origin[0] + (np.arange(nx) + 0.5) * t.res
    t = Terrain(np.tile(0.5 * xx, (ny, 1)), t.known, t.obstacle_height, t.origin, t.res)
    x = np.array([0.5, 1.23, 2.77])
    np.testing.assert_allclose(t.ground_at(x, np.full(3, 2.0)), 0.5 * x, atol=1e-9)
    iy, ix, inside = t.cell(0.05, 0.15)
    assert (iy, ix, bool(inside)) == (1, 0, True)
    assert not t.cell(-0.01, 1.0)[2]


def test_ground_plane_pitch_roll_signs():
    t = Terrain.flat(size_x=6, size_y=6, origin=(-3, -3))
    ny, nx = t.shape
    xx = t.origin[0] + (np.arange(nx) + 0.5) * t.res
    up_x = Terrain(np.tile(0.1 * xx, (ny, 1)), t.known, t.obstacle_height, t.origin, t.res)
    z, pitch, roll = up_x.ground_plane(0, 0, 0.0)
    assert z == pytest.approx(0, abs=1e-9)
    assert pitch == pytest.approx(-math.atan(0.1), abs=1e-6)  # nose up = negative pitch
    assert roll == pytest.approx(0, abs=1e-6)
    _, pitch, roll = up_x.ground_plane(0, 0, math.pi / 2)  # facing +y: slope rises to the right
    assert pitch == pytest.approx(0, abs=1e-6)
    assert roll == pytest.approx(-math.atan(0.1), abs=1e-6)


def test_unknown_mask_and_obstacle_layer():
    t = Terrain.flat(size_x=4, size_y=4, origin=(0, 0))
    known = t.known.copy()
    known[:, :10] = False
    oh = t.obstacle_height.copy()
    oh[20, 20] = 0.5
    t = Terrain(t.ground, known, oh, t.origin, t.res)
    assert not t.known_at(0.5, 1.0) and t.known_at(1.5, 1.0)
    assert t.static_obstacle_at(2.05, 2.05) and not t.static_obstacle_at(2.25, 2.05)
    assert t.surface_at(2.05, 2.05) == pytest.approx(0.5)


@pytest.mark.skipif(not TERRAIN_NPZ.exists(), reason="course_terrain.npz not built")
def test_course_heightfield_matches_driven_heights():
    d = np.load(TERRAIN_NPZ)
    t = Terrain.load(TERRAIN_NPZ)
    c = d["course_trajectory"]
    dz = t.ground_at(c[:, 1], c[:, 2]) - (c[:, 3] - 0.43)
    assert np.median(np.abs(dz)) < 0.05
    assert np.percentile(np.abs(dz), 90) < 0.12
    assert t.known_at(c[:, 1], c[:, 2]).mean() > 0.99


# ---------------------------------------------------------------- sensor contracts
def test_contract_copies_match_real():
    rng = np.random.default_rng(1)
    pts = rng.uniform([-1, -1, -0.6], [2, 1, 0.4], (5000, 3))
    for a, b in zip(height_grid(pts), copies.height_grid(pts), strict=True):
        np.testing.assert_array_equal(a, b)
    far = rng.uniform([-10, -10, -0.5], [10, 10, 1.0], (5000, 3))
    np.testing.assert_array_equal(conservative_scan(far), copies.conservative_scan(far))
    t = pose_to_matrix(1, 2, 0.4, 0.05, -0.1, 0.7)
    np.testing.assert_allclose(points_in_yaw_frame(pts, np.eye(4), t),
                               copies.points_in_yaw_frame(pts, np.eye(4), t))
    assert REAL_CONTRACTS, "real real_transfer / native_transfer contracts should import"


@pytest.mark.parametrize("yaw", [0.0, 1.1, -2.5])
def test_flat_height_grid_semantics(yaw):
    t = Terrain.flat(origin=(-10, -10), size_x=20, size_y=20)
    s = SensorModel(t, SensorConfig(dropout=0.0, range_noise=0.0), seed=0)
    obs = s.observe((0.0, 0.0, 0.43, 0.0, 0.0, yaw))
    assert obs["grid"].shape == (13, 9) and obs["valid"].shape == (13, 9)
    assert obs["valid"].mean() > 0.95
    np.testing.assert_allclose(obs["grid"][obs["valid"]], -0.43, atol=0.02)
    assert (obs["grid"][~obs["valid"]] == -1).all()
    assert obs["scan"].shape == (72,) and np.isfinite(obs["scan"]).all()
    # Flat ground is outside the -0.25..0.75 m band: ranges are the visibility extent.
    assert np.nanmin(obs["scan"]) > 1.0


def test_step_cell_becomes_invalid_and_riser_raises_grid():
    t = stair_terrain(riser=0.15, tread=0.30, x_start=0.60)
    s = SensorModel(t, SensorConfig(dropout=0.0, range_noise=0.0), seed=2)
    obs = s.observe((0.0, 0.0, 0.43, 0.0, 0.0, 0.0))
    grid, valid = obs["grid"], obs["valid"]
    # rows are X (-0.6..1.2 step 0.15): risers at x = 0.6, 0.9, 1.2 fall inside rows 8, 10,
    # 12 (cells straddling two levels -> spread > 0.08 -> invalid); rows 9 / 11 are one and
    # two risers up.
    assert valid[:8].all()
    np.testing.assert_allclose(grid[:8], -0.43, atol=0.02)
    assert not valid[8].any() and not valid[10].any() and not valid[12].any()
    np.testing.assert_allclose(grid[9], -0.28, atol=0.02)
    np.testing.assert_allclose(grid[11], -0.13, atol=0.02)


def test_unknown_terrain_gives_no_returns_and_invalid_cells():
    t = Terrain.flat(origin=(-10, -10), size_x=20, size_y=20)
    known = t.known.copy()
    iy, ix, _ = t.cell(np.array([0.5]), np.array([0.0]))
    known[:, int(ix[0]):] = False  # everything ahead of x = 0.5 unobserved
    t = Terrain(t.ground, known, t.obstacle_height, t.origin, t.res)
    s = SensorModel(t, SensorConfig(dropout=0.0), seed=0)
    obs = s.observe((0.0, 0.0, 0.43, 0.0, 0.0, 0.0))
    assert not obs["valid"][9:].any()  # x >= 0.75 rows unknown
    assert obs["valid"][:4].all()
    ahead = obs["scan"][34:38]
    assert np.all(np.nan_to_num(ahead, nan=0) < 1.0)  # no far ground seen straight ahead


def test_injected_box_shortens_scan_and_raises_grid():
    t = Terrain.flat(origin=(-10, -10), size_x=20, size_y=20)
    t.inject(Box(2.0, 0.0, 0.5, size_x=0.4, size_y=1.0))
    s = SensorModel(t, SensorConfig(dropout=0.0, range_noise=0.0), seed=0)
    obs = s.observe((0.0, 0.0, 0.43, 0.0, 0.0, 0.0))
    front = obs["scan"][36]  # bin 36 = [0, 5) deg
    assert front == pytest.approx(1.8, abs=0.1)
    t.clear_injected()
    t.inject(Box(1.0, 0.0, 0.3, size_x=0.3, size_y=0.3))
    obs = s.observe((0.0, 0.0, 0.43, 0.0, 0.0, 0.0))
    row = int(round((1.05 + 0.6) / 0.15))
    assert obs["grid"][row, 4] == pytest.approx(0.3 - 0.43, abs=0.03)


def test_pose_noise_only_affects_estimate():
    t = Terrain.flat(origin=(-10, -10), size_x=20, size_y=20)
    s = SensorModel(t, SensorConfig(pose_noise_xy=0.05, pose_noise_yaw=0.02), seed=0)
    est = s.estimated_pose((1.0, 2.0, 0.43, 0, 0, 0.3))
    assert est != (1.0, 2.0, 0.43, 0, 0, 0.3) and abs(est[0] - 1) < 0.3


def test_raycast_hits_flat_ground():
    t = Terrain.flat(origin=(-10, -10), size_x=20, size_y=20)
    d = np.array([[math.cos(-0.3), 0, math.sin(-0.3)]])
    th = raycast(t, np.array([0, 0, 0.5]), d, 10.0, 0.04)
    assert th[0] == pytest.approx(0.5 / math.sin(0.3), abs=0.01)


# ---------------------------------------------------------------- obstacle injection
def test_place_on_route_lateral_and_blocked_corridor():
    r = straight_route()
    b = place_on_route(r, 3.0, 0.4, "box", size_x=0.4, size_y=0.4)
    assert (b.x, b.y) == pytest.approx((3.0, 0.4))
    c = place_on_route(r, 5.0, -0.3, "cylinder", radius=0.1)
    assert isinstance(c, Cylinder) and (c.x, c.y) == pytest.approx((5.0, -0.3))
    w = blocked_corridor(r, 4.0)
    assert w.size_y >= 2 * 0.8 and w.contains(4.0, 0.79) and w.contains(4.0, -0.79)
    t = Terrain.flat()
    t.inject(b)
    assert b.base_z == pytest.approx(0.0)
    assert t.obstacle_at(3.0, 0.4) and not t.obstacle_at(3.0, 0.0)
    assert float(b.distance(3.0, 0.0)) == pytest.approx(0.2)


def test_robot_collision_blocks_motion_and_limits():
    t = Terrain.flat()
    t.inject(Box(1.2, 0.0, 0.5, size_x=0.2, size_y=1.0))
    r = KinematicRobot(t, 0.0, 0.0, 0.0)
    for _ in range(100):
        ev = r.step((1.0, 1.0, 1.0), 0.1)
    assert abs(r.state.vy) <= 0.05 + 1e-9 and abs(r.state.wz) <= 0.2 + 1e-9
    r2 = KinematicRobot(t, 0.0, 0.0, 0.0)
    blocked = False
    for _ in range(200):
        ev = r2.step((0.5, 0, 0), 0.1)
        blocked |= ev["blocked"]
    assert blocked and r2.state.x < 1.2 - 0.1 - 0.45 + 0.03
    # slew: vx after one tick is 0.02 (0.2 m/s^2 * 0.1 s)
    r3 = KinematicRobot(Terrain.flat(), 0, 0, 0)
    r3.step((0.2, 0, 0), 0.1)
    assert r3.state.vx == pytest.approx(0.02)


def test_gait_switch_requires_stop():
    r = KinematicRobot(Terrain.flat(), 0, 0, 0)
    for _ in range(20):
        r.step((0.2, 0, 0), 0.1)
    r.request_gait("stairs")
    switched_at = None
    for k in range(100):
        ev = r.step((0.2, 0, 0), 0.1)
        if ev["gait_switched"]:
            switched_at = k
            break
    assert r.state.gait == "stairs" and switched_at is not None and switched_at >= 30
    for _ in range(40):
        r.step((0.5, 0, 0), 0.1)
    assert r.state.vx == pytest.approx(0.15)


# ---------------------------------------------------------------- harness / metrics
def test_run_straight_reaches_all_waypoints(tmp_path):
    t = Terrain.flat()
    r = straight_route(6.0)
    s = run(r, t, PurePursuitController(), RunConfig(), tmp_path, "straight", verbose=False)
    assert s["finish_reason"] == "done"
    assert s["waypoints_reached"] == 3 and s["waypoints_reached_in_order"]
    assert s["collisions"] == 0 and s["lateral_dev_max_m"] < 0.05
    assert (tmp_path / "straight" / "log.csv").exists()
    assert (tmp_path / "straight" / "topview.png").exists()
    # time ~ 6 m / 0.2 m/s plus acceleration
    assert 29 < s["sim_time_s"] < 40


def test_run_blocked_box_reports_stall_and_clearance(tmp_path):
    t = Terrain.flat()
    r = straight_route(6.0)
    box = place_on_route(r, 3.0, 0.0, "box", size_x=0.5, size_y=0.5, height=0.5)
    cfg = RunConfig(progress_timeout=15.0)
    s = run(r, t, PurePursuitController(), cfg, tmp_path, "box", [box], verbose=False)
    assert s["finish_reason"] == "stuck_no_progress"
    assert s["collisions"] == 0  # reference controller stops on the scan
    assert s["stalls"] >= 1
    assert 0.0 < s["min_clearance_injected_m"] < 0.6
    assert s["waypoints_missed"] == [] and "WP03" in s["waypoints_not_attempted"]


def test_run_gait_switch_and_step_metric(tmp_path):
    t = stair_terrain(riser=0.15, tread=0.30, x_start=3.0, n=4)
    wps_x = [0.0, 2.4, 6.0]
    r = straight_route(6.0)
    data = r.data
    data["waypoints"][1]["position"][0] = wps_x[1]
    data["segments"][0]["centerline"] = [[x, 0, 0] for x in np.arange(0, 2.41, 0.2)]
    data["segments"][1]["centerline"] = [[x, 0, 0] for x in np.arange(2.4, 6.01, 0.2)]
    data["segments"][1]["gait"] = "stairs"
    data["segments"][1]["speed_limit"] = 0.15
    for w in data["waypoints"]:
        w["position"][2] = float(t.ground_at(w["position"][0], 0.0))
    r = Route(data)
    s = run(r, t, PurePursuitController(), RunConfig(), tmp_path, "stairs", verbose=False)
    assert s["gait_switches"] == 1 and s["gait_switch_log"][0]["gait"] == "stairs"
    assert s["waypoints_reached"] == 3
    assert s["max_footprint_step_m"] == pytest.approx(0.15, abs=0.01)
    assert s["step_violations"] == 0  # 0.15 m risers are within the stairs-gait limit


def test_missed_waypoint_is_reported(tmp_path):
    t = Terrain.flat()
    r = straight_route(6.0)
    r.waypoints[1]["position"][1] = 0.6  # WP02 0.6 m off the centerline
    s = run(r, t, PurePursuitController(), RunConfig(), tmp_path, "miss", verbose=False)
    assert s["waypoints_missed"] == ["WP02"]
    assert s["waypoints_reached"] == 2


def test_subroute_and_placeholder_schema():
    if not PLACEHOLDER_PATH.exists():
        pytest.skip("placeholder route not generated")
    r = Route.load(PLACEHOLDER_PATH)
    assert "PLACEHOLDER" in r.data["status"]
    assert r.segments[0]["gait"] in ("flat", "stairs")
    for seg in r.segments:
        c = np.asarray(seg["centerline"])
        assert np.linalg.norm(np.diff(c[:, :2], axis=0), axis=1).max() <= 0.25 + 1e-6
    sub = subroute(r, 10.0, 30.0)
    assert sub.length == pytest.approx(20.0, abs=0.3)
    assert sub.wp_order[0] == "S0" and sub.wp_order[-1] == "S1"


def test_clear_static_along_removes_only_cells_near_the_path():
    import numpy as np

    from sim_full_course.terrain import Terrain

    t = Terrain.flat(size_x=4.0, size_y=4.0, res=0.1, origin=(0.0, 0.0))
    t.obstacle_height[20, 20] = 0.5  # on the path (x=2.05, y=2.05)
    t.obstacle_height[35, 20] = 0.5  # 1.5 m off the path
    t.__post_init__()
    path = np.array([[0.0, 2.05], [4.0, 2.05]])
    assert t.clear_static_along(path, 0.35) == 1
    assert not t.static_obstacle_at(2.05, 2.05)
    assert t.static_obstacle_at(2.05, 3.55)
