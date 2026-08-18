from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
import onnxruntime as ort
import pytest

from s10_auto_nav.strategy.policy import ActionKind
from s10_auto_nav.strategy.router import (
    Mode,
    RobotState,
    Router,
    RouterConfig,
    Source,
    observation_from_state,
)
from s10_auto_nav.strategy.stairs57_policy import Stairs57Config, Stairs57Policy

ROOT = Path(__file__).resolve().parents[3]
BUNDLE = ROOT / "policy/stairs57"
STAIRS_SEGMENT = (18, 19)


def _state(t: float, *, segment=STAIRS_SEGMENT, owner="official", **changes):
    values = dict(
        t=t,
        segment=segment,
        position=np.array([25.8, 29.955, 2.3]),
        yaw=0.0,
        pitch=0.0,
        roll=0.0,
        speed=0.35,
        forward_speed=0.35,
        yaw_rate=0.0,
        odom_time=t,
        lidar_time=t,
        heightmap_time=t,
        obstacle_distance=float("inf"),
        lateral_error=0.0,
        heading_error=0.0,
        travelled=t,
        joint_positions=np.zeros(16),
        joint_velocities=np.zeros(16),
        joint_torques=np.zeros(16),
        actual_joint_owner=owner,
    )
    values.update(changes)
    return RobotState(**values)


def test_stairs57_asset_hash_and_graph_contract():
    manifest = json.loads((BUNDLE / "policy_manifest.json").read_text())
    model = BUNDLE / manifest["onnx"]
    assert hashlib.sha256(model.read_bytes()).hexdigest() == manifest["sha256"]
    assert (manifest["observation_dim"], manifest["action_dim"]) == (57, 16)
    session = ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
    input_meta = session.get_inputs()[0]
    output_meta = session.get_outputs()[0]
    assert (input_meta.name, input_meta.type, input_meta.shape) == (
        "obs",
        "tensor(float)",
        [1, 57],
    )
    assert (output_meta.name, output_meta.type, output_meta.shape) == (
        "actions",
        "tensor(float)",
        [1, 16],
    )
    action = session.run(None, {"obs": np.zeros((1, 57), np.float32)})[0]
    assert action.shape == (1, 16)
    assert action.dtype == np.float32
    assert np.all(np.isfinite(action))


def test_stairs57_rejects_a_command_outside_its_training_range():
    with pytest.raises(ValueError):
        Stairs57Config(command_forward=0.50)


def test_stairs57_is_delegated_and_carries_the_035_observation_command():
    policy = Stairs57Policy()
    state = _state(1.0)
    observation = observation_from_state(state)
    policy.start(observation)
    action = policy.step(observation)
    assert action.kind is ActionKind.DELEGATED
    assert action.twist == pytest.approx((0.35, 0.0, 0.0))
    assert action.info["owner"] == "stairs57"


def test_segment_policy_owns_until_strict_target_then_waits_for_official_ack():
    policy = Stairs57Policy()
    router = Router(
        RouterConfig(ready_dwell=0.0),
        policies={"stairs57_policy": policy},
        segment_policies={STAIRS_SEGMENT: "stairs57_policy"},
    )
    state = _state(0.0)
    out = router.tick(state, (0.7, 0.0, 0.0), observation_from_state(state))
    assert (router.mode, out.source) == (Mode.ALIGN, Source.NAV)
    out = router.tick(_state(0.02), (0.7, 0.0, 0.0), observation_from_state(_state(0.02)))
    assert router.mode is Mode.CLIMB_READY
    out = router.tick(_state(0.04), (0.7, 0.0, 0.0), observation_from_state(_state(0.04)))
    assert router.mode is Mode.CLIMB
    state = _state(0.06, owner="stairs57")
    out = router.tick(state, (0.7, 0.0, 0.0), observation_from_state(state))
    assert out.source is Source.POLICY

    reached = _state(1.0, segment=(19, 20), owner="stairs57")
    out = router.tick(reached, (0.7, 0.0, 0.0), observation_from_state(reached))
    assert (router.mode, out.source) == (Mode.HANDOFF, Source.ROUTER)
    acknowledged = _state(1.02, segment=(19, 20), owner="official")
    out = router.tick(
        acknowledged, (0.7, 0.0, 0.0), observation_from_state(acknowledged)
    )
    assert (router.mode, out.source) == (Mode.NAVIGATE, Source.NAV)


def test_stairs57_can_take_control_from_rest_after_alignment():
    policy = Stairs57Policy()
    router = Router(
        RouterConfig(ready_dwell=0.0),
        policies={"stairs57_policy": policy},
        segment_policies={STAIRS_SEGMENT: "stairs57_policy"},
    )
    state = _state(0.0, speed=0.0, forward_speed=0.0)
    router.tick(state, (0.0, 0.0, 0.0), observation_from_state(state))
    state = _state(0.02, speed=0.0, forward_speed=0.0)
    out = router.tick(state, (0.0, 0.0, 0.0), observation_from_state(state))
    assert (router.mode, out.command) == (Mode.CLIMB_READY, (0.35, 0.0, 0.0))


def test_stairs57_alignment_does_not_consume_the_runway_while_turning():
    policy = Stairs57Policy()
    router = Router(
        RouterConfig(ready_dwell=0.0),
        policies={"stairs57_policy": policy},
        segment_policies={STAIRS_SEGMENT: "stairs57_policy"},
    )
    state = _state(0.0, heading_error=math.radians(-15.0))
    router.tick(state, (0.7, 0.0, 0.0), observation_from_state(state))
    state = _state(0.02, heading_error=math.radians(-15.0))
    out = router.tick(state, (0.7, 0.0, 0.0), observation_from_state(state))
    assert out.command[0] == 0.0
    assert out.command[2] > 0.0


def test_stairs57_hands_back_after_four_wheels_hold_on_target_platform():
    policy = Stairs57Policy(Stairs57Config(completion_hold=0.04))
    router = Router(
        RouterConfig(ready_dwell=0.0),
        policies={"stairs57_policy": policy},
        segment_policies={STAIRS_SEGMENT: "stairs57_policy"},
    )
    for tick in range(4):
        state = _state(
            tick * 0.02,
            owner="stairs57" if tick >= 3 else "official",
            segment_target_z=2.36,
        )
        router.tick(state, (0.7, 0.0, 0.0), observation_from_state(state))

    wheels = np.array(
        [
            [26.6, 30.2, 2.45],
            [26.6, 29.8, 2.45],
            [26.1, 30.2, 2.45],
            [26.1, 29.8, 2.45],
        ]
    )
    for tick in range(2):
        state = _state(
            1.0 + tick * 0.02,
            owner="stairs57",
            position=np.array([26.5, 29.955, 2.60]),
            wheel_positions=wheels,
            wheel_contacts=np.ones(4, dtype=bool),
            segment_target_z=2.36,
        )
        out = router.tick(state, (0.7, 0.0, 0.0), observation_from_state(state))
    assert (router.mode, out.source) == (Mode.HANDOFF, Source.ROUTER)
    assert "4/4 wheels" in out.reason


def test_consecutive_stair_segments_keep_one_policy_history():
    policy = Stairs57Policy()
    router = Router(
        RouterConfig(ready_dwell=0.0),
        policies={"stairs57_policy": policy},
        segment_policies={(5, 6): "stairs57_policy", (6, 7): "stairs57_policy"},
    )
    for tick in range(4):
        state = _state(
            tick * 0.02,
            segment=(5, 6),
            owner="stairs57" if tick >= 3 else "official",
        )
        router.tick(state, (0.7, 0.0, 0.0), observation_from_state(state))
    resets = policy.reset_count
    continued = _state(1.0, segment=(6, 7), owner="stairs57")
    out = router.tick(continued, (0.7, 0.0, 0.0), observation_from_state(continued))
    assert (router.mode, out.source) == (Mode.CLIMB, Source.POLICY)
    assert policy.reset_count == resets


def test_stairs57_runtime_does_not_depend_on_lidar_or_heightmap_freshness():
    policy = Stairs57Policy()
    router = Router(
        RouterConfig(ready_dwell=0.0, sensor_timeout=0.1),
        policies={"stairs57_policy": policy},
        segment_policies={STAIRS_SEGMENT: "stairs57_policy"},
    )
    for tick in range(4):
        state = _state(
            tick * 0.02,
            owner="stairs57" if tick >= 3 else "official",
        )
        router.tick(state, (0.7, 0.0, 0.0), observation_from_state(state))

    state = _state(
        1.0,
        owner="stairs57",
        odom_time=1.0,
        lidar_time=0.0,
        heightmap_time=0.0,
    )
    out = router.tick(state, (0.7, 0.0, 0.0), observation_from_state(state))
    assert (router.mode, out.source) == (Mode.CLIMB, Source.POLICY)


def test_stairs57_never_owns_the_gate16_segment():
    policy = Stairs57Policy()
    router = Router(
        policies={"stairs57_policy": policy},
        segment_policies={(18, 19): "stairs57_policy"},
    )
    assert router.policy_for((15, 16)) == ""
