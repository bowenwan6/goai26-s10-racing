"""The route runner on a kinematic robot: the first version's sequence where nothing goes wrong,
and recovery only where something does -- a box the map does not have, a push off the route, a
way back across a hazard, a stall, a closed passage."""

import itertools
import math
from dataclasses import dataclass

import numpy as np
import pytest
from route_v2_helpers import make_route, polyline
from test_rl_edge_tracker import synthetic_grid

from s10_auto_nav.rl_nav.maneuvers import Maneuver
from s10_auto_nav.rl_nav.map_check import MapSurface, unexpected_ahead
from s10_auto_nav.rl_nav.map_planner import MapPlanner
from s10_auto_nav.rl_nav.route_runner import Mode, NavInput, RouteRunner
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


def flat_map(size=40.0, res=0.05, holes=()):
    """Flat known ground; ``holes`` (x0, x1, y0, y1) are unknown to the map."""
    n = int(size / res)
    known = np.ones((n, n), bool)
    origin = (-10.0, -20.0)
    for x0, x1, y0, y1 in holes:
        i0, i1 = int((y0 - origin[1]) / res), int((y1 - origin[1]) / res)
        j0, j1 = int((x0 - origin[0]) / res), int((x1 - origin[0]) / res)
        known[i0:i1, j0:j1] = False
    return MapSurface(np.zeros((n, n)), known, np.zeros((n, n)), origin, res)


def ground_points(x, y, yaw, base_z=0.42, boxes=()):
    """A synthetic cloud in the robot's yaw frame: the flat ground ahead, plus map-frame boxes
    (x0, x1, y0, y1, top) where the ray reaches them."""
    u, v = np.meshgrid(np.arange(0.3, 3.0, 0.1), np.arange(-1.5, 1.51, 0.1))
    pts = [np.column_stack([u.ravel(), v.ravel(), np.full(u.size, -base_z)])]
    c, s = math.cos(yaw), math.sin(yaw)
    for x0, x1, y0, y1, top in boxes:
        gx, gy, gz = np.meshgrid(
            np.arange(x0, x1, 0.05), np.arange(y0, y1, 0.05), np.arange(0.05, top, 0.05)
        )
        dx, dy = gx.ravel() - x, gy.ravel() - y
        keep = np.hypot(dx, dy) < 3.0
        pts.append(np.column_stack([c * dx + s * dy, -s * dx + c * dy, gz.ravel() - base_z])[keep])
    return np.vstack(pts)


def footprint_hits(x, y, yaw, box, half_length=0.45, half_width=0.25):
    u, v = np.meshgrid(
        np.linspace(-half_length, half_length, 10), np.linspace(-half_width, half_width, 6)
    )
    px = x + math.cos(yaw) * u - math.sin(yaw) * v
    py = y + math.sin(yaw) * u + math.cos(yaw) * v
    x0, x1, y0, y1 = box[:4]
    return bool(((px >= x0) & (px <= x1) & (py >= y0) & (py <= y1)).any())


def drive(
    runner,
    path,
    surface=lambda x, y: np.zeros_like(x),
    steps=400,
    dt=0.1,
    start=(0.0, 0.0, 0.0),
    status=lambda t, x, y: ("RUNNING", ""),
    boxes=lambda t: (),
    disturb=None,
    stop_on=(Mode.DONE, Mode.HOLD),
):
    """Kinematic robot + a follower that pursues the route and scores waypoints within 0.3 m."""
    x, y, yaw = start
    cursor = 1
    owner = "official"  # the SDK's joint owner, reported back: the last one asked for
    trace = []
    for k in range(steps):
        t = k * dt
        pr = path.project((x, y))
        wp_s = path.waypoint_s
        wp = np.asarray(path.point_at(wp_s[min(cursor, len(wp_s) - 1)]))[:2]
        if cursor < len(wp_s) and math.hypot(wp[0] - x, wp[1] - y) < 0.3:
            cursor += 1
        st, why = status(t, x, y)
        carrot = np.asarray(path.point_at(min(pr.s + 1.0, path.length)))[:2]
        err = math.atan2(carrot[1] - y, carrot[0] - x) - yaw
        err = (err + math.pi) % (2 * math.pi) - math.pi
        cmd = (0.5, 0.0, float(np.clip(1.5 * err, -0.6, 0.6))) if st == "RUNNING" else (0, 0, 0)
        f = FakeFollower(cmd, st, why, pr.s, pr.d, cursor, f"WP{cursor:02d}")
        f.finished = cursor >= len(wp_s)
        s_gate = float(wp_s[cursor]) if cursor < len(wp_s) else path.length
        grid, valid = synthetic_grid(surface, pose=(x, y, yaw))
        pts = ground_points(x, y, yaw, boxes=boxes(t))
        out = runner.step(
            NavInput(t, x, y, 0.42, yaw, 0.0, 0.0, 0.0, 0.4, grid, valid, f, s_gate, owner, pts)
        )
        owner = out.owner
        vx, vy, wz = out.command
        yaw += wz * dt
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        if disturb is not None:
            x, y, yaw = disturb(t, out.mode, x, y, yaw)
        trace.append((t, x, y, yaw, out.mode, out.owner, out.command, pr.s, pr.d))
        if out.mode in stop_on:
            break
    return trace


def straight(length=12.0, z_end=0.0, **kw):
    return make_route([[0, 0, 0], [length / 2, 0, 0], [length, 0, z_end]], **kw)


def climb(s0=4.2, s1=5.7, first=5.0, last=5.0, policy="stairs"):
    return Maneuver(
        id="M00",
        s0=s0,
        s1=s1,
        s_first=first,
        s_last=last,
        policy=policy,
        kinds=["edge_up"],
        max_edge_up=0.15,
    )


def modes(trace):
    return [m.value for m in dict.fromkeys(tr[4] for tr in trace)]


def transitions(runner):
    return [e["to"] for e in runner.log]


def runner_on(path, maneuvers=(), surface=None):
    surface = surface or flat_map()
    return RouteRunner(path, list(maneuvers), surface=surface, planner=MapPlanner(surface))


# ---------------------------------------------------------------------------- nominal
def test_nominal_climb_is_the_first_versions_sequence():
    path = RoutePath(make_route([[0, 0, 0], [4, 0, 0], [9, 0, 0.15]]))
    runner = runner_on(path, [climb()])
    trace = drive(runner, path, surface=lambda x, y: np.where(x >= 5.0, 0.15, 0.0), steps=600)
    assert transitions(runner) == ["APPROACH", "ALIGN", "CLIMB", "WALK", "DONE"]
    owners = [tr[5] for tr in trace]
    first_stairs = owners.index("stairs_stable")
    assert "official" in owners[first_stairs:]  # handed back after the climb
    # Nothing else engaged: no recovery on a clean run.
    assert not any(e["to"] in ("DETOUR", "WAIT", "HOLD", "TRACK") for e in runner.log)


def test_refusing_follower_on_known_ground_tracks_the_route():
    path = RoutePath(straight())
    runner = runner_on(path)
    trace = drive(
        runner,
        path,
        status=lambda t, x, y: ("BLOCKED", "centerline_blocked_detour_forbidden"),
        steps=160,
    )
    assert Mode.TRACK in {tr[4] for tr in trace}
    assert trace[-1][1] > 6.0
    assert max(abs(tr[2]) for tr in trace) < 0.1
    assert not any(e["to"] in ("DETOUR", "WAIT", "HOLD") for e in runner.log)


def test_align_holds_the_line_against_a_downhill_slide():
    path = RoutePath(make_route([[0, 0, 0], [4, 0, 0], [9, 0, 0.15]]))
    runner = runner_on(path, [climb()])

    def slide(t, mode, x, y, yaw):  # 0.1 m/s downhill (-y) while turning in place
        return (x, y - 0.01, yaw) if mode == Mode.ALIGN else (x, y, yaw)

    trace = drive(
        runner,
        path,
        surface=lambda x, y: np.where(x >= 5.0, 0.15, 0.0),
        start=(4.25, 0.0, math.radians(40.0)),  # at the manoeuvre start, 40 deg off
        steps=500,
        disturb=slide,
    )
    in_align = [tr for tr in trace if tr[4] == Mode.ALIGN]
    assert len(in_align) > 10
    # Turning 40 deg at 0.5 rad/s with a 0.1 m/s slide would leave it ~0.15 m off; it holds.
    assert max(abs(tr[8]) for tr in in_align) < 0.12
    assert "CLIMB" in transitions(runner)


def test_drop_ahead_slows_the_climb():
    path = RoutePath(make_route([[0, 0, 0], [4, 0, 0], [9, 0, 0.15]]))
    runner = runner_on(path, [climb(s1=8.5, last=7.5)])

    def hole(x, y):  # a 0.6 m deep hole in the flight, 1 m past its first edge
        return np.where((x >= 6.0) & (np.abs(y) < 0.4), -0.6, np.where(x >= 5.0, 0.15, 0.0))

    trace = drive(runner, path, surface=hole, steps=300, stop_on=(Mode.DONE, Mode.HOLD))
    climbing = [tr for tr in trace if tr[4] == Mode.CLIMB and 5.2 <= tr[1] <= 5.8]
    assert climbing and all(tr[6][0] <= 0.1 + 1e-9 for tr in climbing)


def test_stepping_off_a_ledge_goes_straight_across_it():
    # A 0.19 m ledge at x = 5, the route bending 60 deg right 0.6 m past it (no waypoint there):
    # the pursuit's carrot rounds the bend while the robot is on the ledge and would turn it, and a
    # turn with the rear wheels on the edge catches a knee.
    bend = [[0, 0, 0.19], [5.6, 0, 0], [7.0, -2.4, 0]]
    path = RoutePath(make_route([bend[0], bend[-1]], centerlines=[polyline(bend)]))
    ledge = Maneuver(
        id="M00",
        s0=4.2,
        s1=5.7,
        s_first=5.0,
        s_last=5.0,
        policy="walk_descend",
        kinds=["edge_down"],
        max_edge_down=0.19,
    )
    runner = runner_on(path, [ledge])
    trace = drive(runner, path, surface=lambda x, y: np.where(x < 5.0, 0.19, 0.0), steps=300)
    straddling = [tr for tr in trace if tr[4] == Mode.DESCEND and 4.6 < tr[7] < 5.0 + 0.6]
    assert straddling
    assert all(tr[6][0] > 0.0 and abs(tr[6][2]) < 0.1 for tr in straddling)
    assert trace[-1][4] == Mode.DONE


def test_stuck_at_a_ledge_backs_off_and_tries_again():
    path = RoutePath(make_route([[0, 0, 0.19], [5.6, 0, 0], [9, 0, 0]]))
    ledge = Maneuver(
        id="M00",
        s0=4.2,
        s1=5.7,
        s_first=5.0,
        s_last=5.0,
        policy="walk_descend",
        kinds=["edge_down"],
        max_edge_down=0.19,
    )
    runner = runner_on(path, [ledge])

    def lip(t, mode, x, y, yaw):  # the first go jams a knee on the lip; the second clears it
        first = runner.descend_tries == 0
        return (min(x, 4.95), y, yaw) if mode == Mode.DESCEND and first else (x, y, yaw)

    trace = drive(
        runner, path, surface=lambda x, y: np.where(x < 5.0, 0.19, 0.0), steps=600, disturb=lip
    )
    kinds = transitions(runner)
    assert kinds[kinds.index("DESCEND") :][:3] == ["DESCEND", "BACKUP", "DESCEND"]
    assert trace[-1][4] == Mode.DONE


def test_stalled_stairs_actor_is_not_asked_to_turn_hard():
    path = RoutePath(make_route([[0, 0, 0], [4, 0, 0], [9, 0, 0.15]]))
    runner = runner_on(path, [climb(s1=8.5, last=8.0)])
    runner.mode, runner.k = Mode.CLIMB, 0
    grid, valid = synthetic_grid(lambda x, y: np.where(x >= 5.0, 0.15, 0.0), pose=(5.2, 0.0, 0.6))

    def wz_after(v_forward, ticks=15):
        out = None
        for k in range(ticks):
            f = FakeFollower((0.0, 0.0, 0.0), "BLOCKED", "", 5.2, 0.0, 2, "WP03")
            nav = NavInput(
                k * 0.1,
                5.2,
                0.0,
                0.42,
                0.6,
                0.0,
                0.0,
                0.0,
                v_forward,
                grid,
                valid,
                f,
                9.0,
                "stairs_stable",
                None,
            )
            out = runner.step(nav)
        return out.command[2]

    # 34 deg left of the route: the law asks for the full right turn when it climbs ...
    assert wz_after(0.3) == pytest.approx(-0.35)
    # ... and at most 0.1 rad/s while it is stalled against the riser.
    assert abs(wz_after(0.0)) <= 0.1 + 1e-9


# ---------------------------------------------------------------------------- recovery
def test_box_on_the_route_is_detoured_on_the_map():
    path = RoutePath(straight())
    runner = runner_on(path)
    box = (4.0, 4.4, -0.35, 0.35, 0.4)
    trace = drive(runner, path, boxes=lambda t: [box], steps=500)
    assert "DETOUR" in modes(trace)
    assert not any(footprint_hits(tr[1], tr[2], tr[3], box) for tr in trace)
    assert trace[-1][4] == Mode.DONE
    back = next(e for e in runner.log if e["why"] == "back on the route")
    assert back["s"] > 4.4


def test_pushed_far_off_the_route_plans_back():
    path = RoutePath(straight())
    runner = runner_on(path)
    pushed = []

    def push(t, mode, x, y, yaw):
        if not pushed and x > 3.0:
            pushed.append(t)
            return x, y + 1.3, yaw
        return x, y, yaw

    trace = drive(runner, path, steps=400, disturb=push)
    detour = next(e for e in runner.log if e["to"] == "DETOUR")
    assert "off the route" in detour["why"]
    assert trace[-1][4] == Mode.DONE
    after = [tr for tr in trace if tr[0] > pushed[0] + 8.0]
    assert max(abs(tr[2]) for tr in after) < 0.2


def test_way_back_across_a_hazard_is_planned_round_it():
    # A hole in the map between the robot (pushed 0.6 m left) and the route ahead of it.
    hole = (3.4, 5.0, 0.05, 0.45)
    surface = flat_map(holes=[hole])
    path = RoutePath(straight())
    runner = runner_on(path, surface=surface)
    pushed = []

    def push(t, mode, x, y, yaw):
        if not pushed and x > 2.8:
            pushed.append(t)
            return x, 0.75, yaw
        return x, y, yaw

    trace = drive(
        runner,
        path,
        steps=400,
        disturb=push,
        status=lambda t, x, y: ("OFF_CORRIDOR", "") if abs(y) > 0.35 else ("RUNNING", ""),
    )
    assert any("hazard" in e["why"] for e in runner.log if e["to"] == "DETOUR")
    after = [tr for tr in trace if pushed and tr[0] > pushed[0]]
    assert not any(footprint_hits(tr[1], tr[2], tr[3], hole, half_width=0.2) for tr in after)
    assert trace[-1][4] == Mode.DONE


def test_stalled_follower_is_planned_round():
    path = RoutePath(straight())
    runner = runner_on(path)
    trace = drive(runner, path, status=lambda t, x, y: ("HOLD_TERRAIN", ""), steps=300)
    # Backs off first (nothing behind on the map), then plans the way on.
    assert transitions(runner)[:2] == ["BACKUP", "DETOUR"]
    assert any(tr[4] == Mode.BACKUP and tr[6][0] < 0 for tr in trace)
    assert any("no progress" in e["why"] for e in runner.log if e["to"] == "DETOUR")
    # The follower here never drives again: each cycle (8 s, back 0.3 m, plan 1 m on) gains ground.
    assert trace[-1][1] > 1.0


def test_stall_with_a_drop_behind_does_not_back_off():
    # The map ends 0.3 m behind the robot's tail: no backing off, plan at once.
    surface = flat_map(holes=[(-10.0, -0.7, -10.0, 10.0)])
    path = RoutePath(straight())
    runner = runner_on(path, surface=surface)
    drive(runner, path, status=lambda t, x, y: ("HOLD_TERRAIN", ""), steps=120)
    assert "BACKUP" not in transitions(runner)
    assert "DETOUR" in transitions(runner)


def test_closed_passage_waits_then_goes_on_when_it_clears():
    # A 1.2 m wide passage (the map's walls either side) with a person standing in it for 6 s.
    walls = [(3.0, 6.0, 0.6, 5.0), (3.0, 6.0, -5.0, -0.6)]
    surface = flat_map(holes=walls)
    path = RoutePath(straight())
    runner = runner_on(path, surface=surface)
    person = (4.3, 4.7, -0.3, 0.3, 1.6)
    trace = drive(runner, path, boxes=lambda t: [person] if t < 12.0 else [], steps=500)
    assert "WAIT" in modes(trace)
    assert not any(footprint_hits(tr[1], tr[2], tr[3], person) for tr in trace if tr[0] < 12.0)
    assert trace[-1][4] == Mode.DONE


def test_passage_that_never_clears_holds():
    walls = [(3.0, 6.0, 0.6, 5.0), (3.0, 6.0, -5.0, -0.6)]
    surface = flat_map(holes=walls)
    path = RoutePath(straight())
    runner = runner_on(path, surface=surface)
    person = (4.3, 4.7, -0.3, 0.3, 1.6)
    trace = drive(runner, path, boxes=lambda t: [person], steps=600)
    assert trace[-1][4] == Mode.HOLD


def test_stairs_actor_past_a_waypoint_goes_back_for_it():
    path = RoutePath(make_route([[0, 0, 0], [4, 0, 0], [5.4, 0, 0.15], [9, 0, 0.15]]))
    runner = runner_on(path, [climb(s1=6.2)])

    def veer(t, mode, x, y, yaw):  # the stairs actor drifts left past WP03
        return (x, y + 0.03, yaw) if mode == Mode.CLIMB and 5.0 < x < 5.8 else (x, y, yaw)

    trace = drive(
        runner,
        path,
        surface=lambda x, y: np.where(x >= 5.0, 0.15, 0.0),
        steps=600,
        disturb=veer,
    )
    assert "RECOVER" in modes(trace)
    assert trace[-1][4] == Mode.DONE


def test_without_a_map_a_needed_detour_holds_still():
    path = RoutePath(straight())
    runner = RouteRunner(path, [])  # no map surface, no planner: the first version alone

    def push(t, mode, x, y, yaw):
        return (x, y + 1.3, yaw) if 3.0 < x < 3.06 else (x, y, yaw)

    trace = drive(runner, path, steps=200, disturb=push)
    assert trace[-1][4] == Mode.HOLD
    assert trace[-1][6] == (0.0, 0.0, 0.0)


def tent(x, y, half_span=None):
    """A 0.6 m ridge across x = 5 with 24 deg flanks: the stairs actor climbs it, the walking
    actor must not (over 15 deg). Across everything, or only |y| < half_span with flat ground
    beside it."""
    z = np.clip(0.6 - 0.45 * np.abs(np.asarray(x) - 5.0), 0.0, None)
    return z if half_span is None else np.where(np.abs(np.asarray(y)) < half_span, z, 0.0)


def tent_map(half_span=None, res=0.05):
    n = int(40.0 / res)
    origin = (-10.0, -20.0)
    ax = origin[0] + (np.arange(n) + 0.5) * res
    gx, gy = np.meshgrid(ax, origin[1] + (np.arange(n) + 0.5) * res)
    return MapSurface(tent(gx, gy, half_span), np.ones((n, n), bool), np.zeros((n, n)), origin, res)


def tent_run(half_span=None):
    # WP02 is on the far flank (x 5.8): the walking actor cannot stand there, so a miss can only
    # be made good by climbing the ridge again.
    path = RoutePath(make_route([[0, 0, 0], [3, 0, 0], [5.8, 0, 0.24], [9, 0, 0]]))
    ridge = Maneuver(
        id="M00",
        s0=3.0,
        s1=6.9,
        s_first=3.8,
        s_last=6.3,
        policy="stairs",
        kinds=["slope_up"],
        max_slope_deg=24.0,
    )
    runner = runner_on(path, [ridge], surface=tent_map(half_span))

    def veer(t, mode, x, y, yaw):  # the first climb drifts left past WP02
        first = all(e["to"] != "RECOVER" for e in runner.log)
        drift = mode == Mode.CLIMB and first and 5.1 < x < 5.8
        return (x, y + 0.008, yaw) if drift else (x, y, yaw)

    trace = drive(
        runner, path, surface=lambda x, y: tent(x, y, half_span), steps=2500, disturb=veer
    )
    return runner, trace


def test_waypoint_missed_on_a_ridge_with_no_walking_way_back_holds():
    runner, trace = tent_run()
    assert "RECOVER" in transitions(runner)
    assert trace[-1][4] == Mode.HOLD
    assert "no walking way back" in runner.log[-1]["why"]
    # The walking actor never drove on the flanks (|x - 5| < 1.33, off the top).
    walked = [
        tr
        for tr in trace
        if tr[5] == "official" and 0.3 < abs(tr[1] - 5.0) < 1.3 and any(tr[6]) and tr[0] > 15.0
    ]
    assert not walked


def test_waypoint_missed_on_a_ridge_is_climbed_again():
    runner, trace = tent_run(half_span=1.5)
    assert any("climbing it again" in e["why"] for e in runner.log)
    assert trace[-1][4] == Mode.DONE


# ---------------------------------------------------------------------------- pieces
def test_map_check_tells_terrain_from_a_new_box():
    path = RoutePath(straight())
    surface = flat_map()
    pose = (1.0, 0.0, 0.42, 0.0)
    assert unexpected_ahead(ground_points(1.0, 0.0, 0.0), pose, path, 1.0, surface) is None
    box = [(2.0, 2.4, -0.3, 0.3, 0.4)]
    d = unexpected_ahead(ground_points(1.0, 0.0, 0.0, boxes=box), pose, path, 1.0, surface)
    assert d is not None and 0.9 <= d <= 1.1
    side = [(2.0, 2.4, 0.8, 1.2, 0.4)]
    assert (
        unexpected_ahead(ground_points(1.0, 0.0, 0.0, boxes=side), pose, path, 1.0, surface) is None
    )


def test_map_planner_goes_round_a_seen_box_and_stays_off_map_hazards():
    surface = flat_map(holes=[(4.0, 6.0, 0.8, 3.0)])
    planner = MapPlanner(surface)
    path = RoutePath(straight())
    box = np.array([[x, y] for x in np.arange(4.0, 4.4, 0.05) for y in np.arange(-0.4, 0.4, 0.05)])
    xy = planner.plan((2.5, 0.0), path, (5.5, 7.0), extra_xy=box)
    assert xy is not None
    pts = np.vstack([np.linspace(a, b, 20) for a, b in itertools.pairwise(xy)])
    assert (
        np.min(np.hypot(pts[:, None, 0] - box[None, :, 0], pts[:, None, 1] - box[None, :, 1]))
        > 0.35
    )
    assert np.all(planner.clearance_at(pts) >= 0.3)
    # Boxed in on every side: no path.
    ring = np.array([[2.5 + 0.8 * math.cos(a), 0.8 * math.sin(a)] for a in np.linspace(0, 6.3, 80)])
    assert planner.plan((2.5, 0.0), path, (5.5, 7.0), extra_xy=ring) is None
