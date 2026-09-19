"""The manoeuvre router on a kinematic robot: walk, approach, align on the MEASURED edge, climb,
verify, hand back -- with the map's edge prior deliberately wrong -- plus A* rejoin and missed-gate
recovery."""

import math
from dataclasses import dataclass

import numpy as np
from route_v2_helpers import make_route
from test_rl_edge_tracker import synthetic_grid

from s10_auto_nav.rl_nav import local_astar
from s10_auto_nav.rl_nav.maneuver_router import ManeuverRouter, Mode, NavInput
from s10_auto_nav.rl_nav.maneuvers import Maneuver
from s10_auto_nav.route_planner import FREE, OBSTACLE, LocalGrid
from s10_auto_nav.route_v2 import RoutePath


@dataclass
class FakeFollower:
    command: tuple
    status: str
    reason: str
    s: float
    d: float
    target_index: int
    target_id: str
    finished: bool = False


def straight_route():
    return make_route([[0, 0, 0], [4, 0, 0], [9, 0, 0.15]])


def step_surface(edge_x=5.0, h=0.15):
    return lambda x, y: np.where(x >= edge_x, h, 0.0)


def run(router, path, surface, steps=400, dt=0.1, start=(0.0, 0.0, 0.0), follower_status="RUNNING"):
    x, y, yaw = start
    trace = []
    for k in range(steps):
        t = k * dt
        s = x
        wp_s = path.waypoint_s
        target = int(np.searchsorted(wp_s, s + 1e-6))
        f = FakeFollower(
            (0.5, 0.0, -0.8 * y), follower_status, "", s, y, min(target, len(wp_s) - 1), "WP"
        )
        f.finished = s >= wp_s[-1] - 0.05
        grid, valid = synthetic_grid(surface, pose=(x, y, yaw))
        out = router.step(
            NavInput(t, x, y, 0.42, yaw, 0.0, 0.0, 0.0, 0.3, grid, valid, t, f, None, None)
        )
        vx, vy, wz = out.command
        yaw += wz * dt
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        trace.append((t, x, y, math.degrees(yaw), out.mode, out.owner))
        if out.mode in (Mode.DONE, Mode.HOLD, Mode.ABORT):
            break
    return trace


def maneuver(prior_point=(5.1, 0.1), prior_yaw=math.radians(6.0)):
    return Maneuver(
        id="M00",
        s0=4.2,
        s1=5.7,
        s_first=5.0,
        s_last=5.0,
        policy="stairs",
        kinds=["edge_up"],
        max_edge_up=0.15,
        prior_point=list(prior_point),
        prior_normal_yaw=prior_yaw,
        prior_height=0.15,
        deck_z=0.15,
    )


def test_full_sequence_uses_the_measured_edge_not_the_prior():
    path = RoutePath(straight_route())
    router = ManeuverRouter(path, [maneuver()])
    trace = run(router, path, step_surface(), start=(0.0, 0.0, 0.0))
    modes = [m.value for m in dict.fromkeys(tr[4] for tr in trace)]
    assert modes[:4] == ["NAVIGATE", "APPROACH", "ALIGN", "CLIMB"]
    assert "VERIFY" in modes and "HANDBACK" in modes
    climb_start = next(tr for tr in trace if tr[4] == Mode.CLIMB)
    # The robot entered square to the real edge (yaw 0), not to the prior's 6 deg.
    assert abs(climb_start[3]) < 4.0
    assert 5.0 - 0.72 <= climb_start[1] <= 5.0 - 0.36
    owners = [tr[5] for tr in trace]
    assert "stairs_stable" in owners and owners[-1] == "official"


def test_prior_far_off_holds_instead_of_climbing_blind():
    path = RoutePath(straight_route())
    # The map puts the edge 1.5 m away from where it is: every fix is gated out, the router never
    # climbs.
    router = ManeuverRouter(path, [maneuver(prior_point=(6.6, 0.0), prior_yaw=0.0)])
    trace = run(router, path, step_surface(), steps=600)
    assert all(tr[5] != "stairs_stable" for tr in trace)
    assert trace[-1][4] == Mode.HOLD


def test_rejoin_plans_around_an_obstacle():
    path = RoutePath(straight_route())
    grid = LocalGrid.centred((2.0, 0.0), 8.0, 0.1, fill=FREE)
    grid.fill_rect(2.2, 2.6, -0.6, 0.6, OBSTACLE)
    xy = local_astar.plan(grid, (1.5, 0.8, 0.0), path, 1.5, goal_ahead=(1.5, 3.0))
    assert xy is not None
    # It goes around the box (never through x in [2.2, 2.6] with |y| < 0.6 + body half width).
    through = (xy[:, 0] > 2.1) & (xy[:, 0] < 2.7) & (np.abs(xy[:, 1]) < 0.85)
    assert not through.any()
    assert np.hypot(*(xy[-1] - np.asarray(path.point_at(3.0))[:2])) < 1.6


def test_blocked_follower_triggers_rejoin_and_back():
    path = RoutePath(straight_route())
    router = ManeuverRouter(path, [])
    grid = LocalGrid.centred((1.0, 0.0), 8.0, 0.1, fill=FREE)
    flat, valid = synthetic_grid(lambda x, y: np.zeros_like(x))
    f = FakeFollower((0.0, 0.0, 0.0), "OFF_CORRIDOR", "|d|=1.0", 1.0, 1.0, 1, "WP02")
    out = None
    for k in range(15):
        out = router.step(
            NavInput(0.1 * k, 1.0, 1.0, 0.42, 0.0, 0, 0, 0, 0, flat, valid, 0.1 * k, f, grid)
        )
    # A* heads back to the route: 1 m to the left, so it first turns right towards it.
    assert out.mode == Mode.REJOIN
    assert out.command[2] < 0.0 or out.command[0] > 0.0
    assert out.reason == "A*"
