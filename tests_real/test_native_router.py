import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from native_transfer.contracts import SourceClock, conservative_scan, validate_route
from native_transfer.router import Feedback, Limits, NativeGaitRouter
from s10_auto_nav.strategy.router import RobotState, Router
from s10_auto_nav.waypoints import Course, Waypoint


def sample(t=10.0, gait=0x3002, **changes):
    robot = RobotState(
        t=t,
        segment=(0, 1),
        position=np.zeros(3),
        yaw=0,
        pitch=0,
        roll=0,
        speed=0,
        yaw_rate=0,
        odom_time=t,
        lidar_time=t,
        heightmap_time=t,
        travelled=0,
    )
    return replace(
        Feedback(robot, gait, 17, 0, t, t, t, t, True, True, True, command=(0.15, 0, 0)), **changes
    )


def activate(gate, target=0, gait=0x3002, start=10.0):
    gate.arm(sample(start, gait), admission=True)
    requests = []
    for i in range(1, 16):
        result = gate.tick(sample(start + i * 0.1, gait), target)
        if result.request_gait is not None:
            requests.append(result.request_gait)
        if result.state == "active":
            return start + i * 0.1, requests
    pytest.fail("never activated")


def test_observer_never_requests_policy_or_velocity():
    gate = NativeGaitRouter(["flat"])
    for i in range(20):
        decision = gate.tick(sample(10 + i * 0.1), 0)
        assert decision.command == (0, 0, 0)
        assert decision.request_gait is None and not decision.publish
    assert isinstance(gate.router, Router)


def test_arm_requires_admission_and_rl():
    gate = NativeGaitRouter(["flat"])
    with pytest.raises(ValueError):
        gate.arm(sample(), admission=False)
    with pytest.raises(ValueError):
        gate.arm(sample(motion_state=0), admission=True)
    assert not gate.ever_armed


def test_fresh_ack_and_one_request_before_velocity():
    gate = NativeGaitRouter(["flat"])
    t, requests = activate(gate)
    assert requests == [0x3002]
    assert gate.accepted_gaits == {0x3002}
    result = gate.tick(sample(t + 0.1), 0)
    assert result.command[0] > 0


def test_cached_matching_gait_cannot_ack_request():
    gate = NativeGaitRouter(["flat"])
    gate.arm(sample(), admission=True)
    requested = None
    for i in range(1, 16):
        t = 10 + i * 0.1
        f = sample(t)
        if requested is not None:
            f = replace(f, motion_received=requested - 0.01)
        out = gate.tick(f, 0)
        if out.request_gait:
            requested = t
        assert out.command == (0, 0, 0)
    assert gate.state == "fault"


def test_switch_to_stairs_holds_zero_until_new_feedback():
    gate = NativeGaitRouter(["flat", "stairs", "flat"])
    t, _ = activate(gate)
    for i in range(1, 8):
        out = gate.tick(sample(t + i * 0.1, 0x3002), 1)
        assert out.command == (0, 0, 0)
    for i in range(8, 15):
        out = gate.tick(sample(t + i * 0.1, 0x3003), 1)
    assert gate.state == "active"
    assert out.command[0] <= 0.15
    assert gate.tick(sample(t + 1.5, 0x3003), 2).command == (0, 0, 0)


@pytest.mark.parametrize(
    "change",
    [
        {"localized": False},
        {"perception_valid": False},
        {"exclusive_control": False},
        {"hes": 1},
        {"motion_state": 4},
        {"command": (math.nan, 0, 0)},
        {"gait": 0x1001},
        {"external_fault": "operator_stick_active"},
        {"motion_received": 1.0},
        {"status_received": 1.0},
        {"hes_received": 1000.0},
        {"command": (-0.2, 0, 0)},
    ],
)
def test_fault_latches_and_packets_cannot_resume(change):
    gate = NativeGaitRouter(["flat"])
    t, _ = activate(gate)
    out = gate.tick(sample(t + 0.1, **change), 0)
    assert out.state == "fault" and out.command == (0, 0, 0)
    assert gate.tick(sample(t + 0.2), 0).state == "fault"
    with pytest.raises(ValueError):
        gate.arm(sample(t + 0.3), admission=True)


def test_robot_must_stop_before_switch():
    gate = NativeGaitRouter(["flat"])
    gate.arm(sample(), admission=True)
    for i in range(1, 44):
        f = sample(10 + i * 0.1)
        f = replace(f, robot=replace(f.robot, speed=0.2))
        out = gate.tick(f, 0)
        assert out.request_gait is None and out.command == (0, 0, 0)
    assert gate.state == "fault"


def test_cancel_done_tilt_and_loop_stall_zero():
    for case in ["cancel", "done", "tilt", "gap"]:
        gate = NativeGaitRouter(["flat"])
        t, _ = activate(gate)
        f = sample(t + (0.8 if case == "gap" else 0.1))
        if case == "cancel":
            gate.cancel(t)
        elif case == "done":
            f = replace(f, robot=replace(f.robot, course_finished=True))
        elif case == "tilt":
            f = replace(f, robot=replace(f.robot, roll=math.radians(40)))
        out = gate.tick(f, 0)
        assert out.command == (0, 0, 0) and out.state in {"stopped", "done", "fault"}


def test_probe_cannot_emit_velocity_even_if_follower_requests_it():
    gate = NativeGaitRouter(["stairs"], probe_only=True)
    t, _ = activate(gate, gait=0x3003)
    f = sample(
        t + 0.1,
        gait=0x3003,
        command=(1, 1, 1),
        localized=False,
        perception_valid=False,
        status_received=0,
        command_received=0,
    )
    assert gate.tick(f, 0).command == (0, 0, 0)
    assert gate.state == "active"


def test_height_gate_rejects_wrong_floor_and_missing_z():
    course = Course([Waypoint(0, np.array([0, 0, 2]))], height_tolerance=0.2)
    assert not course.update(np.array([0, 0]))
    assert not course.update(np.array([0, 0, 0]))
    assert not course.update(np.array([0, 0, np.nan]))
    assert course.update(np.array([0, 0, 1.9]))


def test_source_stamp_cannot_be_refreshed_by_receipt():
    clock = SourceClock()
    clock.check("odom", 100, 100.1)
    with pytest.raises(ValueError):
        clock.check("odom", 100, 100.2)
    with pytest.raises(ValueError):
        clock.check("cloud", 10, 100.2)


def test_cloud_unknown_sectors_remain_unknown():
    ranges = conservative_scan(np.array([[2, 0, -0.42], [1, 0, 0]]))
    assert np.isnan(ranges).sum() == 71
    assert np.nanmin(ranges) == 1


def test_configuration_rejects_unknown_gaits_and_limits():
    with pytest.raises(ValueError):
        NativeGaitRouter(["high_platform"])
    with pytest.raises(ValueError):
        Limits(max_flat=math.nan)
    with pytest.raises(ValueError):
        validate_route({"waypoints": [{"index": 0, "position": [0, 0, 0]}]})


def test_start_b_draft_has_three_flights_and_flat_landings():
    path = Path(__file__).parents[1] / "native_transfer/config/start_b.draft.json"
    route = json.loads(path.read_text())
    points = validate_route(route)
    assert len(points) == 18
    kinds = [points[0]["kind"]]
    for point in points[1:]:
        if point["kind"] != kinds[-1]:
            kinds.append(point["kind"])
    assert kinds == ["flat", "stairs", "flat", "stairs", "flat", "stairs", "flat"]
    assert "DRAFT" in route["metadata"]["status"]
    assert points[-1]["position"][:2] == [26.516563974, 21.678986371]
