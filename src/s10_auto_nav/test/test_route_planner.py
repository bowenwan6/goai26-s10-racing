"""Route-relative local planner on synthetic local grids (no ROS)."""

from __future__ import annotations

import math

import numpy as np
import pytest
from route_v2_helpers import make_route

from s10_auto_nav.route_planner import (
    FREE,
    OBSTACLE,
    SCAN_CLEAR,
    UNKNOWN,
    LocalGrid,
    LocalGridBuilder,
    LocalGridConfig,
    Observation,
    RoutePlanner,
    quintic_offset,
    step_obstacles,
)


def straight(corridor=0.8, allow_detour=True, gait="flat", length=10.0):
    route = make_route(
        [[0, 0, 0], [length, 0, 0]],
        corridor=corridor,
        allow_detour=allow_detour,
        gaits=[gait],
    )
    return route, route.path()


def free_grid(center=(0.0, 0.0), fill=FREE):
    return LocalGrid.centred(center, 8.0, 0.1, fill=fill)


def plan(route, path, grid, pose=(0.0, 0.0, 0.0), prev=0.0, planner=None):
    planner = planner or RoutePlanner()
    x, y, yaw = pose
    proj = path.project([x, y])
    return planner.plan(
        path, proj, pose, grid,
        segment=route.segments[0], s_gate=path.waypoint_s[1],
        gate_xy=route.waypoints[1].xy, prev_choice=prev, lookahead=0.5,
    )


def min_clearance(result, x0, x1, y0, y1):
    """Smallest distance from the planned path to a rectangle."""
    p = result.path
    dx = np.maximum(np.maximum(x0 - p[:, 0], 0), p[:, 0] - x1)
    dy = np.maximum(np.maximum(y0 - p[:, 1], 0), p[:, 1] - y1)
    return float(np.hypot(dx, dy).min())


def test_quintic_boundary_conditions():
    s = np.linspace(0, 2, 201)
    d = quintic_offset(s, 0.1, 0.0, 0.6, 2.0)
    assert d[0] == pytest.approx(0.1) and d[-1] == pytest.approx(0.6)
    grad = np.gradient(d, s)
    assert abs(grad[0]) < 1e-2 and abs(grad[-1]) < 1e-2
    assert np.all(np.diff(d) >= -1e-12)  # monotone, no overshoot


def test_no_obstacle_tracks_the_centreline():
    route, path = straight()
    res = plan(route, path, free_grid())
    assert res.status == "TRACK" and res.d_target == 0.0
    assert abs(res.carrot[1]) < 1e-9 and res.carrot[0] == pytest.approx(0.5, abs=0.05)
    assert res.speed_scale == pytest.approx(1.0)
    assert np.abs(res.path[:, 1]).max() < 1e-9


def test_offset_robot_rejoins_the_line_when_clear():
    route, path = straight()
    res = plan(route, path, free_grid(), pose=(1.0, 0.6, 0.0), prev=0.6)
    assert res.d_target == 0.0  # w_lat > w_smooth: rejoin beats holding the offset
    assert res.carrot[1] < 0.6  # heading back toward the line


def test_obstacle_on_line_detours_within_corridor():
    route, path = straight(corridor=0.8)
    grid = free_grid()
    grid.fill_rect(2.0, 2.4, -0.2, 0.2, OBSTACLE)
    res = plan(route, path, grid)
    assert res.status == "DETOUR"
    assert 0.0 < abs(res.d_target) <= 0.8
    # The committed part of the path keeps the footprint off the box.
    assert min_clearance(res, 2.0, 2.4, -0.2, 0.2) >= 0.3 - 1e-6


def test_detour_forbidden_stops_instead():
    route, path = straight(corridor=0.25, allow_detour=False, gait="stairs")
    grid = free_grid()
    grid.fill_rect(1.2, 1.6, -0.2, 0.2, OBSTACLE)
    # Far enough: keep creeping (collision-free length >= commit_length_stairs, 0.6 m)...
    far = plan(route, path, grid)
    assert not far.blocked and far.free_length < 1.0 and far.speed_scale < 1.0
    # ...then stop, never swerve, once the free length drops below the commitment.
    res = plan(route, path, grid, pose=(0.3, 0.0, 0.0))
    assert res.blocked and res.reason == "centerline_blocked_detour_forbidden"
    assert res.carrot is None and res.speed_scale == 0.0
    assert len(res.candidates) == 1 and res.candidates[0].d_target == 0.0


def test_no_detour_segment_still_tracks_when_clear():
    route, path = straight(corridor=0.25, allow_detour=False, gait="stairs")
    res = plan(route, path, free_grid(), pose=(0.0, 0.1, 0.0))
    assert res.status in ("TRACK", "DETOUR") and res.d_target == 0.0
    assert not res.blocked


def test_unknown_cells_are_not_free():
    route, path = straight()
    grid = free_grid()
    grid.fill_rect(1.5, 2.5, -0.3, 0.3, UNKNOWN)
    res = plan(route, path, grid)
    assert res.status == "DETOUR"  # treated like an obstacle
    assert min_clearance(res, 1.5, 2.5, -0.3, 0.3) >= 0.3 - 1e-6
    route2, path2 = straight(corridor=0.25, allow_detour=False, gait="stairs")
    assert plan(route2, path2, grid, pose=(0.6, 0.0, 0.0)).blocked


def test_everything_unknown_is_blocked_not_invented():
    route, path = straight()
    res = plan(route, path, free_grid(fill=UNKNOWN))
    assert res.blocked
    assert res.carrot is None


def test_all_candidates_blocked_falls_back_to_astar():
    # A wall close ahead that no smooth lateral transition can clear in time, with a gap
    # on the left inside a wide corridor.
    route, path = straight(corridor=1.3)
    grid = free_grid()
    grid.fill_rect(0.9, 1.1, -1.5, 0.35, OBSTACLE)
    res = plan(route, path, grid)
    assert all(not c.valid for c in res.candidates)
    assert res.status == "ASTAR", res.reason
    assert res.carrot is not None and 0 < res.speed_scale <= 0.5
    assert min_clearance(res, 0.9, 1.1, -1.5, 0.35) >= 0.25
    # The A* path never leaves the corridor.
    assert np.abs(res.path[:, 1]).max() <= 1.3 + 0.1


def test_astar_failure_stops_with_reason():
    route, path = straight(corridor=0.8)
    grid = free_grid()
    grid.fill_rect(0.9, 1.1, -3.0, 3.0, OBSTACLE)  # wall across the whole corridor
    res = plan(route, path, grid)
    assert res.blocked and res.reason.startswith("astar")
    assert res.carrot is None


def test_astar_never_enters_unknown_frontier_only():
    # Shadow behind a wall is UNKNOWN; A* may only drive to the known frontier.
    route, path = straight(corridor=1.3)
    grid = free_grid()
    grid.fill_rect(0.9, 1.1, -1.5, 0.35, OBSTACLE)
    grid.fill_rect(1.1, 8.0, -4.0, 4.0, UNKNOWN)
    res = plan(route, path, grid)
    if not res.blocked:
        states = grid.lookup(res.path)
        assert not np.any(states == UNKNOWN)


def test_gate_approach_uses_the_waypoint_itself():
    route, path = straight(length=3.0)
    res = plan(route, path, free_grid(), pose=(2.9, 0.05, 0.0))
    assert res.status == "GATE"
    assert res.carrot == pytest.approx([3.0, 0.0])


def test_path_never_runs_past_the_unscored_gate():
    route, path = straight(length=2.0)
    grid = free_grid()
    res = plan(route, path, grid, pose=(0.5, 0.0, 0.0))
    assert res.path[:, 0].max() <= 2.0 + 1e-9


def test_pivot_is_swept_for_a_long_body():
    # Robot facing north, path east; a post at the rear-left corner's sweep blocks the pivot.
    route, path = straight()
    grid = free_grid()
    grid.fill_rect(0.35, 0.45, 0.35, 0.45, OBSTACLE)
    res = plan(route, path, grid, pose=(0.0, 0.0, math.pi / 2))
    assert all(c.free_length == 0.0 for c in res.candidates if not c.valid)


# ------------------------------------------------------------- grid builder


def flat_obs(t=0.0, pose=(0.0, 0.0, 0.42, 0.0), h=None, mask=None, scan=None):
    height = np.full((13, 9), -0.42) if h is None else h
    return Observation(
        t=t, pose=pose, height=height,
        mask=np.ones((13, 9), bool) if mask is None else mask,
        scan_ranges=np.full(72, 10.0) if scan is None else scan,
    )


def test_builder_marks_box_top_not_the_ground_beside_it():
    h = np.full((13, 9), -0.42)
    h[8:10, 3:6] = 0.0  # a 0.42 m box ahead
    flags = step_obstacles(h, np.ones_like(h, bool), 0.12, reference=-0.42)
    assert flags[8:10, 3:6].all()
    assert not flags[7, 4] and not flags[8, 2]  # ground next to it stays ground
    builder = LocalGridBuilder()
    builder.add(flat_obs(h=h))
    grid = builder.build((0.0, 0.0))
    assert grid.lookup(np.array([[0.7, 0.0]]))[0] == OBSTACLE
    assert grid.lookup(np.array([[0.3, 0.0]]))[0] == FREE
    assert grid.lookup(np.array([[3.0, 0.0]]))[0] == SCAN_CLEAR


def test_builder_keeps_unknown_unknown():
    mask = np.ones((13, 9), bool)
    mask[6:9, :] = False  # hole / occlusion in the height map
    scan = np.full(72, 10.0)
    scan[:10] = np.nan  # unknown bins behind-left
    builder = LocalGridBuilder()
    builder.add(flat_obs(mask=mask, scan=scan))
    grid = builder.build((0.0, 0.0))
    # Inside the height ROI an invalid cell is UNKNOWN even though the scan passed over it.
    assert grid.lookup(np.array([[0.45, 0.0]]))[0] == UNKNOWN
    # A NaN bin gives no free space.
    angle = -math.pi + 5 * (2 * math.pi / 72)
    assert grid.lookup(np.array([[3 * math.cos(angle), 3 * math.sin(angle)]]))[0] == UNKNOWN


def test_scan_returns_become_obstacles_and_shadow_unknown():
    scan = np.full(72, 10.0)
    scan[35:37] = 2.0  # the two bins either side of straight ahead
    builder = LocalGridBuilder()
    builder.add(flat_obs(scan=scan))
    grid = builder.build((0.0, 0.0))
    assert grid.lookup(np.array([[1.97, 0.0]]))[0] == OBSTACLE
    assert grid.lookup(np.array([[3.0, 0.0]]))[0] == UNKNOWN
    assert grid.lookup(np.array([[1.5, 0.0]]))[0] == SCAN_CLEAR


def test_scan_ignored_on_stairs_segments():
    scan = np.full(72, 10.0)
    scan[35:37] = 1.0  # a riser in the body band
    builder = LocalGridBuilder()
    builder.add(flat_obs(scan=scan))
    grid = builder.build((0.0, 0.0), gait="stairs")
    assert grid.lookup(np.array([[1.0, 0.0]]))[0] == FREE  # height map says step-able
    assert grid.lookup(np.array([[2.5, 0.0]]))[0] == UNKNOWN  # beyond the height ROI


def test_fusion_keeps_older_ground_but_expires_it():
    cfg = LocalGridConfig(fuse_frames=5, max_age=1.0)
    builder = LocalGridBuilder(cfg)
    builder.add(flat_obs(t=0.0, pose=(0.0, 0.0, 0.42, 0.0)))
    # Robot moved 1 m; the new frame cannot see under/behind itself.
    mask = np.ones((13, 9), bool)
    mask[:5, :] = False
    builder.add(flat_obs(t=0.5, pose=(1.0, 0.0, 0.42, 0.0), mask=mask))
    grid = builder.build((1.0, 0.0), now=0.5)
    assert grid.lookup(np.array([[0.2, 0.0]]))[0] == FREE  # from the older frame
    grid = builder.build((1.0, 0.0), now=1.4)
    assert grid.lookup(np.array([[0.2, 0.0]]))[0] != FREE  # older frame expired
    # A storey change clears the fusion.
    builder.add(flat_obs(t=1.5, pose=(1.0, 0.0, 1.5, 0.0)))
    assert len(builder.frames) == 1


def test_fill_isolated_holes_fills_single_cell_but_not_gaps():
    from s10_auto_nav.route_planner import fill_isolated_holes

    h = np.zeros((13, 9))
    m = np.ones((13, 9), bool)
    m[6, 4] = False  # isolated sparse cell on flat ground -> filled
    m[2:5, 2:5] = False  # 3x3 gap (possible hole) -> centre keeps too few neighbours
    h2, m2 = fill_isolated_holes(h, m, limit=0.12)
    assert m2[6, 4] and h2[6, 4] == 0.0
    assert not m2[3, 3]

    h[5:8, 3:6] = 0.0
    h[5, 3] = 0.4  # neighbours disagree by more than the step limit -> not filled
    h3, m3 = fill_isolated_holes(h, m, limit=0.12)
    assert not m3[6, 4]


def test_stair_riser_rows_are_filled_under_stairs_limit_only():
    """A riser inside one 0.15 m cell fails the 0.08 m spread test, leaving a whole invalid
    row across the grid. Under the stairs step limit (0.22 m) interior riser cells have 6
    agreeing neighbours and are filled; under the flat limit (0.12 m) they are not."""
    from s10_auto_nav.route_planner import fill_isolated_holes

    h = np.zeros((13, 9))
    for ix in range(13):
        h[ix, :] = 0.15 * (ix // 2)  # 0.15 m rise every two 0.15 m cells
    m = np.ones((13, 9), bool)
    riser_rows = [5, 7, 9]
    m[riser_rows, :] = False
    h2, m2 = fill_isolated_holes(h, m, limit=0.22)
    assert m2[riser_rows, 1:-1].all()  # interior columns filled
    assert not m2[riser_rows][:, [0, -1]].any()  # border columns keep < 6 neighbours
    _, m3 = fill_isolated_holes(h, m, limit=0.12)
    assert not m3[riser_rows, 1:-1].any()


def test_own_footprint_is_cleared_but_cells_ahead_are_not():
    import math

    from s10_auto_nav.route_planner import FREE, UNKNOWN, LocalGridBuilder, Observation

    b = LocalGridBuilder()
    h = np.zeros((13, 9))
    m = np.zeros((13, 9), bool)  # nothing valid: everything under/around the body UNKNOWN
    b.add(Observation(t=0.0, pose=(0.0, 0.0, 0.0, 0.0), height=h, mask=m, scan_ranges=None))
    g = b.build((0.0, 0.0), yaw=0.0)
    i0 = g.index(np.array([[0.0, 0.0]]))[:2]
    i1 = g.index(np.array([[1.0, 0.0]]))[:2]
    assert g.state[i0[0][0], i0[1][0]] == FREE
    assert g.state[i1[0][0], i1[1][0]] == UNKNOWN
    g2 = b.build((0.0, 0.0), yaw=math.pi / 2)  # rotated body: (0, 0.4) now under it
    j = g2.index(np.array([[0.0, 0.4]]))[:2]
    assert g2.state[j[0][0], j[1][0]] == FREE
