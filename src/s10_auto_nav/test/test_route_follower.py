"""RouteFollowerCore: ordered gating, safety stops and a closed-loop kinematic sim."""

from __future__ import annotations

import math

import numpy as np
import pytest
from route_v2_helpers import FIXTURE, make_route

from s10_auto_nav.pure_pursuit import PurePursuitController
from s10_auto_nav.route_follower import (
    RouteFollowerConfig,
    RouteFollowerCore,
    native_gains,
)
from s10_auto_nav.route_sim import (
    Box,
    HeightField,
    KinematicBody,
    run_closed_loop,
    sense_height_grid,
    sense_scan,
)
from s10_auto_nav.route_v2 import RouteV2

OFFSET = 0.42


def core_for(route, **config):
    return RouteFollowerCore(
        route,
        RouteFollowerConfig(body_z_offset=OFFSET, **config),
        controller=PurePursuitController(native_gains()),
    )


def sensed(field, pose):
    grid, mask = sense_height_grid(field, pose)
    ranges, angles = sense_scan(field, pose)
    return grid, mask, ranges, angles


def test_from_file_entry_point_and_first_tick():
    core = RouteFollowerCore.from_file(FIXTURE, body_z_offset=OFFSET)
    field = HeightField()
    pose = (0.0, 0.0, OFFSET, 0.0)
    out = core.step(0.0, pose, *sensed(field, pose))
    # Standing on WP01 scores it at once and starts driving toward WP02.
    assert out.reached == ("WP01",)
    assert out.target_id == "WP02" and out.gait_request == "flat" and out.gait_code == 0x3002
    assert out.status == "RUNNING" and out.command[0] >= 0.0
    assert out.speed_limit == 0.20 and out.corridor_half_width == 0.8


def test_ordered_gating_never_skips_a_waypoint():
    route = RouteV2.load(FIXTURE)
    core = core_for(route)
    field = HeightField()
    core.step(0.0, (0.0, 0.0, OFFSET, 0.0), *sensed(field, (0.0, 0.0, OFFSET, 0.0)))
    # Teleport onto WP03 (upper level): WP02 is still the target.
    pose = (6.0, 3.0, 1.0 + OFFSET, 0.0)
    out = core.step(0.1, pose, *sensed(field, pose))
    assert out.target_id == "WP02" and "WP03" not in out.reached
    assert out.status != "DONE"


def test_required_gait_and_wait_gait():
    route = RouteV2.load(FIXTURE)
    core = core_for(route)
    field = HeightField()
    pose = (6.0, 0.0, OFFSET, math.pi / 2)
    core.course._cursor = 2  # target WP03: the incoming segment is stairs
    core.tracker.reset(6.0)
    out = core.step(0.0, pose, *sensed(field, pose), current_gait="flat")
    assert out.gait_request == "stairs" and out.status == "WAIT_GAIT"
    assert out.command == (0.0, 0.0, 0.0)
    out = core.step(0.1, pose, *sensed(field, pose), current_gait=0x3003)
    assert out.status != "WAIT_GAIT"


def test_stale_inputs_stop():
    core = core_for(make_route([[0, 0, 0], [5, 0, 0]]))
    out = core.step(0.0, (0.0, 0.0, OFFSET, 0.0))  # no height/scan ever
    assert out.status == "STALE_INPUT" and out.command == (0.0, 0.0, 0.0)
    field = HeightField()
    pose = (0.5, 0.0, OFFSET, 0.0)
    assert core.step(0.1, pose, *sensed(field, pose)).moving
    out = core.step(1.0, pose)  # 0.9 s later without a new observation
    assert out.status == "STALE_INPUT"


def test_off_corridor_stops_using_the_segment_width():
    route = make_route([[0, 0, 0], [5, 0, 0], [10, 0, 0]], corridor=0.8)
    core = core_for(route)
    field = HeightField()
    pose = (0.0, 0.0, OFFSET, 0.0)
    core.step(0.0, pose, *sensed(field, pose))
    pose = (1.0, 0.85, OFFSET, 0.0)  # inside 0.8 + 0.1 margin
    assert core.step(0.1, pose, *sensed(field, pose)).status != "OFF_CORRIDOR"
    pose = (1.0, 0.95, OFFSET, 0.0)
    out = core.step(0.2, pose, *sensed(field, pose))
    assert out.status == "OFF_CORRIDOR" and out.command == (0.0, 0.0, 0.0)


def test_terrain_hold_on_flat_segment_pitch_and_no_gait_change():
    core = core_for(make_route([[0, 0, 0], [5, 0, 0]]))
    field = HeightField()
    pose = (0.5, 0.0, OFFSET, 0.0)
    statuses = []
    for k in range(10):
        out = core.step(0.1 * k, pose, *sensed(field, pose), pitch=math.radians(16))
        statuses.append(out.status)
        assert out.gait_request == "flat"
    assert statuses[-1] == "HOLD_TERRAIN" and out.command == (0.0, 0.0, 0.0)


def test_closed_loop_detour_and_rejoin_on_straight_line():
    """Unicycle sim: box on the taught line; detour within corridor, then rejoin."""
    route = make_route([[0, 0, 0], [7, 0, 0], [10, 0, 0]], corridor=0.8)
    core = core_for(route)
    field = HeightField([Box(3.8, 4.2, -0.2, 0.2, 0.4)])
    body = KinematicBody(0.0, 0.0, 0.0, OFFSET, field)
    trace = run_closed_loop(core, body, duration=120.0, dt=0.1)
    assert trace[-1]["status"] == "DONE", trace[-1]
    d = np.array([r["d"] for r in trace if np.isfinite(r["d"])])
    statuses = {r["status"] for r in trace}
    assert "DETOUR" in statuses
    assert np.abs(d).max() < 0.8  # never outside the corridor
    assert np.abs(d).max() > 0.45  # really went round the 0.4 m box
    # Rejoined: the last few metres are on the line.
    tail = [abs(r["d"]) for r in trace if np.isfinite(r["d"]) and r["pose"][0] > 7.0]
    assert max(tail) < 0.1
    # Never reversed, and the 0.9 x 0.5 m body (any yaw) never overlapped the box.
    assert min(r["command"][0] for r in trace) >= 0.0
    assert not _body_hits(trace, Box(3.8, 4.2, -0.2, 0.2, 0.4))


def _body_hits(trace, box, length=0.9, width=0.5):
    xs = np.linspace(-length / 2, length / 2, 19)
    ys = np.linspace(-width / 2, width / 2, 11)
    body = np.array([[a, b] for a in xs for b in ys])
    for r in trace:
        x, y, _, yaw = r["pose"]
        c, s = math.cos(yaw), math.sin(yaw)
        px = x + c * body[:, 0] - s * body[:, 1]
        py = y + s * body[:, 0] + c * body[:, 1]
        if np.any((px >= box.x0) & (px <= box.x1) & (py >= box.y0) & (py <= box.y1)):
            return True
    return False


def test_closed_loop_no_detour_segment_stops_before_obstacle():
    route = make_route([[0, 0, 0], [6, 0, 0]], gaits=["stairs"])  # allow_detour=false, 0.25
    core = core_for(route)
    field = HeightField([Box(3.0, 3.4, -0.5, 0.5, 0.5)])
    body = KinematicBody(0.0, 0.0, 0.0, OFFSET, field)
    trace = run_closed_loop(core, body, duration=40.0, dt=0.1)
    final = trace[-1]
    assert final["status"] == "BLOCKED"
    assert final["reason"] == "centerline_blocked_detour_forbidden"
    assert final["pose"][0] + 0.45 < 3.0  # stopped with the front short of the box
    assert max(abs(r["d"]) for r in trace if np.isfinite(r["d"])) < 0.25
    assert min(r["command"][0] for r in trace) >= 0.0


@pytest.mark.parametrize("fuse", [1, 8])
def test_clear_route_tracks_with_and_without_fusion(fuse):
    from s10_auto_nav.route_planner import LocalGridConfig

    route = make_route([[0, 0, 0], [4, 0, 0], [4, 3, 0]], corridor=0.8)
    core = core_for(route, grid=LocalGridConfig(fuse_frames=fuse))
    body = KinematicBody(0.0, 0.0, 0.0, OFFSET, HeightField())
    trace = run_closed_loop(core, body, duration=90.0, dt=0.1)
    assert trace[-1]["status"] == "DONE"
    d = np.array([r["d"] for r in trace if np.isfinite(r["d"])])
    assert np.abs(d).max() < 0.25  # corner cut bounded, no detours on a clear route


def _stub_ros(monkeypatch):
    """Minimal fake rclpy/msgs so follower_node imports without ROS (test-scoped)."""
    import sys
    import types

    class Msg:
        def __init__(self, **kw):
            self.__dict__.update(kw)

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    mods = {
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.executors": types.ModuleType("rclpy.executors"),
        "rclpy.node": types.ModuleType("rclpy.node"),
        "rclpy.qos": types.ModuleType("rclpy.qos"),
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
        "nav_msgs": types.ModuleType("nav_msgs"),
        "nav_msgs.msg": types.ModuleType("nav_msgs.msg"),
        "sensor_msgs": types.ModuleType("sensor_msgs"),
        "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": types.ModuleType("std_msgs.msg"),
    }
    mods["rclpy.executors"].ExternalShutdownException = RuntimeError
    mods["rclpy.node"].Node = object
    for name in ("QoSDurabilityPolicy", "QoSHistoryPolicy", "QoSProfile", "QoSReliabilityPolicy"):
        setattr(mods["rclpy.qos"], name, types.SimpleNamespace)
    mods["geometry_msgs.msg"].Twist = Twist
    mods["nav_msgs.msg"].Odometry = Msg
    mods["sensor_msgs.msg"].LaserScan = Msg
    for name in ("Bool", "Float32", "Float32MultiArray", "String"):
        setattr(mods["std_msgs.msg"], name, Msg)
    for name, mod in mods.items():
        monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.delitem(sys.modules, "s10_auto_nav.follower_node", raising=False)
    import s10_auto_nav.follower_node as follower_node

    return follower_node


def test_node_route_v2_branch_drives_core_and_shares_course(monkeypatch):
    import types

    follower_node = _stub_ros(monkeypatch)
    source = follower_node.__file__
    text = open(source).read()
    assert 'declare_parameter("route_v2_path", "")' in text  # default = legacy behaviour

    class Capture:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    route = RouteV2.load(FIXTURE)
    node = object.__new__(follower_node.WaypointFollowerNode)
    node.control_rate = 10.0
    node.course = route.to_course(OFFSET)
    node.controller = PurePursuitController(native_gains())
    node.route_core = RouteFollowerCore(
        route, RouteFollowerConfig(body_z_offset=OFFSET), controller=node.controller,
        course=node.course,
    )
    node._route_v2_time = 0.0
    node._log_countdown = 0.0
    node._last_forward = 0.0
    for name in ("cmd_pub", "terrain_pub", "progress_pub", "finished_pub"):
        setattr(node, name, Capture())
    node.get_logger = lambda: types.SimpleNamespace(info=lambda *_: None, warn=lambda *_: None)
    field = HeightField()
    pose = (0.0, 0.0, OFFSET, 0.0)
    grid, mask, ranges, angles = sensed(field, pose)
    node._pose_xy, node._pose_z, node._yaw, node._pitch = np.zeros(2), OFFSET, 0.0, 0.0
    node._heightmap, node._heightmap_age = np.where(mask, grid, -1.0), 0.1
    node._ranges, node._beam_angles, node._scan_age = ranges, angles, 0.1
    node._route_v2_control_step(0.1)
    assert node.course.cursor == 1  # WP01 scored through the shared Course
    twist = node.cmd_pub.messages[-1]
    assert twist.linear.x >= 0.0
    assert node.terrain_pub.messages[-1].data.startswith("route_v2:RUNNING")
    # Old observation (no new message this tick) -> the core goes stale and stops.
    for _ in range(6):
        node._heightmap_age += 0.1
        node._scan_age += 0.1
        node._route_v2_control_step(0.1)
    assert node.terrain_pub.messages[-1].data.startswith("route_v2:STALE_INPUT")
    assert node.cmd_pub.messages[-1].linear.x == 0.0
