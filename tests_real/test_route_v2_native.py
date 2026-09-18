"""route_v2 at the native boundary: per-segment gait, speed cap and corridor (no ROS)."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from native_transfer.route_v2_bridge import (
    LEGACY_CORRIDOR,
    CorridorMonitor,
    TerrainMonitor,
    is_route_v2,
    load_route,
    native_router_for,
)
from native_transfer.router import GAITS, NativeGaitRouter
from s10_auto_nav.strategy.router import RobotState

ROOT = Path(__file__).parents[1]
FIXTURE = ROOT / "src/s10_auto_nav/test/fixtures/route_v2_synthetic.json"
LEGACY = ROOT / "native_transfer/config/start_b.draft.json"


def fixture():
    return json.loads(FIXTURE.read_text())


def sample(t, gait, command=(0.3, 0.0, 0.0), speed=0.0, travelled=0.0):
    from native_transfer.router import Feedback

    robot = RobotState(
        t=t, segment=(0, 1), position=np.zeros(3), yaw=0, pitch=0, roll=0, speed=speed,
        yaw_rate=0, odom_time=t, lidar_time=t, heightmap_time=t, travelled=travelled,
    )
    return Feedback(robot, gait, 17, 0, t, t, t, t, True, True, True, command=command)


def activate(gate, target, gait, start):
    gate.arm(sample(start, gait), admission=True)
    for i in range(1, 16):
        result = gate.tick(sample(start + i * 0.1, gait), target)
        if result.state == "active":
            return start + i * 0.1
    pytest.fail("never activated")


def test_route_v2_points_carry_incoming_segment_gait():
    points, route = load_route(fixture())
    assert route is not None and is_route_v2(fixture())
    assert [p["kind"] for p in points] == ["flat", "flat", "stairs", "flat"]
    assert [p["speed_limit"] for p in points] == [0.2, 0.2, 0.15, 0.2]
    assert points[2]["corridor_half_width"] == 0.25


def test_legacy_draft_still_loads_unchanged():
    data = json.loads(LEGACY.read_text())
    points, route = load_route(data)
    assert route is None
    assert points == data["waypoints"]


def test_router_uses_route_gait_and_keeps_settle_switch_confirm():
    points, route = load_route(fixture())
    gate = native_router_for(points, route)
    assert gate.kinds == ("flat", "flat", "stairs", "flat")
    t = activate(gate, 1, GAITS["flat"], 10.0)
    # Moving to target 2 (stairs incoming): zero command, stop, then one gait request.
    decisions = []
    for i in range(1, 12):
        decisions.append(gate.tick(sample(t + i * 0.1, GAITS["flat"]), 2))
    assert all(d.command == (0.0, 0.0, 0.0) for d in decisions)
    requests = [d.request_gait for d in decisions if d.request_gait is not None]
    assert requests == [GAITS["stairs"]]
    assert gate.state == "switching"


def test_route_speed_cap_only_lowers_the_gait_cap():
    points, route = load_route(fixture())
    for p in points:
        p["speed_limit"] = 0.12
    gate = native_router_for(points, route)
    t = activate(gate, 1, GAITS["flat"], 10.0)
    d = gate.tick(sample(t + 0.1, GAITS["flat"], command=(0.3, 0, 0)), 1)
    assert d.command[0] == pytest.approx(0.12)
    # A route cap above the gait cap cannot raise it.
    for p in points:
        p["speed_limit"] = 1.0
    gate = native_router_for(points, route)
    t = activate(gate, 1, GAITS["flat"], 20.0)
    d = gate.tick(sample(t + 0.1, GAITS["flat"], command=(0.3, 0, 0)), 1)
    assert d.command[0] == pytest.approx(0.20)


def test_router_default_is_unchanged_without_caps():
    gate = NativeGaitRouter(["flat", "flat"])
    assert gate.speed_caps is None
    with pytest.raises(ValueError):
        NativeGaitRouter(["flat"], speed_caps=[0.1, 0.2])
    with pytest.raises(ValueError):
        NativeGaitRouter(["flat"], speed_caps=[math.nan])


def test_corridor_uses_segment_width_not_fixed_035():
    points, route = load_route(fixture())
    mon = CorridorMonitor(points, route)
    # WP01 -> WP02 is flat with corridor 0.8: a 0.6 m detour is allowed.
    assert mon.check(0, [0.0, 0.0, 0.0])[0]
    for x in np.arange(0.0, 3.0, 0.1):  # the runtime rejects pose jumps; walk there
        mon.check(1, [x, 0.6 * x / 3.0, 0.0])
    ok, why, along, info = mon.check(1, [3.0, 0.6, 0.0])
    assert ok and along == pytest.approx(3.0) and info["allowed"] == pytest.approx(0.9)
    ok, why, *_ = mon.check(1, [3.5, 0.95, 0.0])
    assert not ok and why == "route_corridor_exceeded"
    # WP02 -> WP03 is stairs with corridor 0.25 (+0.10).
    mon = CorridorMonitor(points, route)
    for x in np.arange(0.0, 6.0, 0.1):
        mon.check(1, [x, 0.0, 0.0])
    for y in np.arange(0.0, 1.0, 0.1):
        mon.check(2, [6.0, y, y / 3.0])
    ok, *_ = mon.check(2, [6.2, 1.0, 0.33])
    assert ok
    ok, why, *_ = mon.check(2, [6.45, 1.5, 0.5])
    assert not ok and why == "route_corridor_exceeded"


def test_corridor_rejects_wrong_level():
    points, route = load_route(fixture())
    mon = CorridorMonitor(points, route)
    mon.check(1, [1.0, 0.0, 0.0])
    ok, why, *_ = mon.check(1, [1.2, 0.0, 3.0])
    assert not ok and why == "route_level_mismatch"


def test_legacy_corridor_is_byte_for_byte_the_old_rule():
    points = json.loads(LEGACY.read_text())["waypoints"]
    mon = CorridorMonitor(points)
    a, b = np.array(points[0]["position"][:2]), np.array(points[1]["position"][:2])
    mid = (a + b) / 2
    normal = np.array([-(b - a)[1], (b - a)[0]]) / np.linalg.norm(b - a)
    assert mon.check(1, [*(mid + normal * (LEGACY_CORRIDOR - 0.01)), 0.0])[0]
    assert not mon.check(1, [*(mid + normal * (LEGACY_CORRIDOR + 0.01)), 0.0])[0]


def test_terrain_monitor_never_requests_gait_and_holds_only_on_flat():
    points, route = load_route(fixture())
    mon = TerrainMonitor(route)
    for _ in range(10):
        res = mon.update(1, 0.1, pitch=math.radians(18))
    assert res.hold  # flat segment into WP02 with stairs-like pitch
    mon = TerrainMonitor(route)
    for _ in range(10):
        res = mon.update(2, 0.1, pitch=math.radians(18))
    assert res.level == "ok"  # stairs segment: expected
    assert not hasattr(res, "request_gait")
    assert TerrainMonitor(None).update(0, 0.1, pitch=1.0).level == "ok"


def test_runtime_wires_route_v2_without_changing_legacy_defaults():
    source = (ROOT / "native_transfer/runtime.py").read_text()
    assert "load_route(" in source and "CorridorMonitor(" in source
    assert "> 0.35" not in source  # the fixed corridor lives only in the legacy monitor
    assert '"route_v2_path": "" if self.route_v2 is None' in source
    # The gait handshake is still the router's; the runtime never requests a gait itself
    # except through decision.request_gait.
    assert source.count("gait.data.gait = ") == 1


def test_route_v2_fixture_validates_against_native_contract():
    from native_transfer.contracts import validate_route

    points, _ = load_route(fixture())
    assert validate_route({"waypoints": points}) == points
