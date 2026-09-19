"""The router's answers when things go off-plan: the planner refuses on terrain the map knows
(follow the route), something new stands on the route (give way to A*), the stairs actor wedges
(back off, retry, then hold), and a measured edge that disagrees with the plan (the route decides
the heading)."""

import math

import numpy as np
from route_v2_helpers import make_route
from test_rl_edge_tracker import synthetic_grid
from test_rl_maneuver_router import FakeFollower, maneuver, run, step_surface, straight_route

from s10_auto_nav.rl_nav.maneuver_router import ManeuverRouter, Mode, NavInput
from s10_auto_nav.rl_nav.map_check import MapSurface, unexpected_ahead
from s10_auto_nav.route_planner import FREE, OBSTACLE, LocalGrid
from s10_auto_nav.route_v2 import RoutePath


def flat_map(size=40.0, res=0.05):
    n = int(size / res)
    return MapSurface(
        np.zeros((n, n)), np.ones((n, n), bool), np.zeros((n, n)), (-10.0, -10.0), res
    )


def ground_points(x, y, yaw, base_z=0.42, extra=()):
    """A synthetic cloud in the robot's yaw frame: the flat ground ahead, plus ``extra`` map-frame
    boxes (x0, x1, y0, y1, top)."""
    u, v = np.meshgrid(np.arange(0.3, 3.0, 0.1), np.arange(-1.0, 1.01, 0.1))
    pts = [np.column_stack([u.ravel(), v.ravel(), np.full(u.size, -base_z)])]
    c, s = math.cos(yaw), math.sin(yaw)
    for x0, x1, y0, y1, top in extra:
        gx, gy, gz = np.meshgrid(
            np.arange(x0, x1, 0.05), np.arange(y0, y1, 0.05), np.arange(0.05, top, 0.05)
        )
        dx, dy = gx.ravel() - x, gy.ravel() - y
        pts.append(np.column_stack([c * dx + s * dy, -s * dx + c * dy, gz.ravel() - base_z]))
    return np.vstack(pts)


def refusing_run(router, path, steps=120, dt=0.1, boxes=(), local_grid=None):
    """The follower refuses throughout (BLOCKED); the router has to decide."""
    x, y, yaw = 0.5, 0.0, 0.0
    flat, valid = synthetic_grid(lambda a, b: np.zeros_like(a))
    trace = []
    for k in range(steps):
        t = k * dt
        f = FakeFollower((0.0, 0.0, 0.0), "BLOCKED", "astar_no_free_goal", x, y, 1, "WP02")
        pts = ground_points(x, y, yaw, extra=boxes)
        out = router.step(
            NavInput(
                t,
                x,
                y,
                0.42,
                yaw,
                0.0,
                0.0,
                0.0,
                0.3,
                flat,
                valid,
                t,
                f,
                local_grid,
                None,
                None,
                pts,
            )
        )
        vx, vy, wz = out.command
        yaw += wz * dt
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        trace.append((t, x, y, out.mode, out.reason, yaw))
    return trace


def footprint_hits(x, y, yaw, box, half_length=0.45, half_width=0.25):
    """Does the robot's rotated 0.9 x 0.5 m footprint overlap the axis-aligned box (x0, x1, y0,
    y1)?"""
    u, v = np.meshgrid(
        np.linspace(-half_length, half_length, 10), np.linspace(-half_width, half_width, 6)
    )
    px = x + math.cos(yaw) * u - math.sin(yaw) * v
    py = y + math.sin(yaw) * u + math.cos(yaw) * v
    x0, x1, y0, y1 = box
    return bool(((px >= x0) & (px <= x1) & (py >= y0) & (py <= y1)).any())


def test_map_check_tells_terrain_from_a_new_box():
    path = RoutePath(straight_route())
    surface = flat_map()
    assert (
        unexpected_ahead(ground_points(1.0, 0.0, 0.0), (1.0, 0.0, 0.42, 0.0), path, 1.0, surface)
        is None
    )
    box = [(2.0, 2.4, -0.3, 0.3, 0.4)]
    d = unexpected_ahead(
        ground_points(1.0, 0.0, 0.0, extra=box), (1.0, 0.0, 0.42, 0.0), path, 1.0, surface
    )
    assert d is not None and 0.9 <= d <= 1.1
    # Beside the corridor it is not in the way.
    side = [(2.0, 2.4, 0.8, 1.2, 0.4)]
    assert (
        unexpected_ahead(
            ground_points(1.0, 0.0, 0.0, extra=side), (1.0, 0.0, 0.42, 0.0), path, 1.0, surface
        )
        is None
    )


def test_planner_refusing_on_known_terrain_follows_the_route():
    path = RoutePath(straight_route())
    router = ManeuverRouter(path, [], map_surface=flat_map())
    trace = refusing_run(router, path, steps=80)
    assert any(tr[3] == Mode.FOLLOW_ROUTE for tr in trace)
    assert trace[-1][1] > 2.5  # it kept going along the route
    assert max(abs(tr[2]) for tr in trace) < 0.15


def test_new_obstacle_on_the_route_hands_over_to_astar():
    path = RoutePath(straight_route())
    router = ManeuverRouter(path, [], map_surface=flat_map())
    grid = LocalGrid.centred((2.0, 0.0), 8.0, 0.1, fill=FREE)
    grid.fill_rect(1.8, 2.2, -0.3, 0.3, OBSTACLE)
    trace = refusing_run(
        router, path, steps=140, boxes=[(1.8, 2.2, -0.3, 0.3, 0.4)], local_grid=grid
    )
    modes = {tr[3] for tr in trace}
    assert Mode.REJOIN in modes
    # The body (0.9 x 0.5 m) never overlapped the box, and it went on past it.
    assert not any(footprint_hits(tr[1], tr[2], tr[5], (1.8, 2.2, -0.3, 0.3)) for tr in trace)
    assert trace[-1][1] > 1.9


def test_wedged_climb_backs_off_retries_then_holds():
    path = RoutePath(straight_route())
    router = ManeuverRouter(path, [maneuver()])
    wall_x = 5.2  # before the manoeuvre ends (s_last 5.0 + 0.3)

    x, y, yaw = 0.0, 0.0, 0.0
    reasons, modes = [], []
    for k in range(900):
        t = k * 0.1
        wp_s = path.waypoint_s
        target = int(np.searchsorted(wp_s, x + 1e-6))
        f = FakeFollower(
            (0.5, 0.0, -0.8 * y), "RUNNING", "", x, y, min(target, len(wp_s) - 1), "WP"
        )
        grid, valid = synthetic_grid(step_surface(), pose=(x, y, yaw))
        out = router.step(NavInput(t, x, y, 0.42, yaw, 0.0, 0.0, 0.0, 0.3, grid, valid, t, f))
        vx, vy, wz = out.command
        yaw += wz * 0.1
        x = min(wall_x, x + (vx * math.cos(yaw) - vy * math.sin(yaw)) * 0.1)  # wedged at the wall
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * 0.1
        reasons.append(out.reason)
        modes.append(out.mode)
        if out.mode == Mode.HOLD:
            break
    assert "backing off" in reasons
    assert modes[-1] == Mode.HOLD
    assert "3 retries" in router.log[-1]["why"]


def test_align_keeps_the_route_heading_when_the_measured_edge_disagrees():
    # The real edge is 17 deg off the route (inside the tracker's 20 deg gate, beyond the 12 deg the
    # router lets it refine the plan): the robot enters on the route's heading, not square to it.
    path = RoutePath(straight_route())
    router = ManeuverRouter(path, [maneuver(prior_point=(5.0, 0.0), prior_yaw=0.0)])
    ang = math.radians(17.0)

    def skew_step(x, y):
        return np.where((x - 5.0) * math.cos(ang) + y * math.sin(ang) >= 0, 0.15, 0.0)

    trace = run(router, path, skew_step, start=(0.0, 0.0, 0.0))
    climb_start = next(tr for tr in trace if tr[4] == Mode.CLIMB)
    assert abs(climb_start[3]) < 9.0


def test_route_with_a_turn_is_followed_when_refused():
    route = make_route([[0, 0, 0], [3, 0, 0], [3, 3, 0]])
    path = RoutePath(route)
    router = ManeuverRouter(path, [], map_surface=flat_map())
    trace = refusing_run(router, path, steps=200)
    assert any(tr[3] == Mode.FOLLOW_ROUTE for tr in trace)
    end = trace[-1]
    assert end[2] > 1.0  # turned the corner and went up the second leg


def test_align_waits_until_past_the_last_bend_before_the_entry():
    # A 90 deg corner at s = 4, the manoeuvre's first edge 1 m after it: ALIGN (which creeps
    # straight at the edge) may not start before the corner, or it would cut it.
    route = make_route([[0, 0, 0], [4, 0, 0], [4, 3, 0.15]])
    path = RoutePath(route)
    router = ManeuverRouter(path, [])
    m = maneuver(prior_point=(4.0, 1.0), prior_yaw=math.pi / 2)
    m.s_first, m.s0, m.s1, m.s_last = 5.0, 4.2, 5.7, 5.0
    s_from = router._last_bend_before(m)
    assert 4.0 <= s_from <= 4.6


def test_sliding_along_a_riser_squares_up_instead_of_pushing():
    # Past x = 5.2 the kinematic robot no longer climbs: pushed forward it slides sideways along
    # the riser (what the stairs actor does when it meets one at an angle). The router must stop
    # pushing and turn square to the riser.
    path = RoutePath(straight_route())
    router = ManeuverRouter(path, [maneuver()])
    x, y, yaw = 0.0, 0.0, 0.0
    reasons = []
    for k in range(600):
        t = k * 0.1
        wp_s = path.waypoint_s
        target = int(np.searchsorted(wp_s, x + 1e-6))
        f = FakeFollower(
            (0.5, 0.0, -0.8 * y), "RUNNING", "", x, y, min(target, len(wp_s) - 1), "WP"
        )
        grid, valid = synthetic_grid(step_surface(), pose=(x, y, yaw))
        out = router.step(NavInput(t, x, y, 0.42, yaw, 0.0, 0.0, 0.0, 0.3, grid, valid, t, f))
        vx, vy, wz = out.command
        reasons.append(out.reason)
        if out.mode == Mode.CLIMB and x >= 5.2 and vx > 0:
            yaw = math.radians(20.0)  # skewed on the riser...
            y -= 0.15 * 0.1  # ...and sliding sideways, not forward
        else:
            yaw += wz * 0.1
            x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * 0.1
            y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * 0.1
        if "squaring up" in reasons:
            break
    assert "squaring up" in reasons
    assert any("sliding along a riser" in entry["why"] for entry in router.log)
