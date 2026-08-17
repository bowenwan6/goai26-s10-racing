from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import onnxruntime as ort

from s10_auto_nav.strategy.gate16_policy import StableGate16Policy
from s10_auto_nav.strategy.policy import ActionKind
from s10_auto_nav.strategy.router import (
    Mode,
    RobotState,
    Router,
    RouterConfig,
    Source,
    observation_from_state,
)


ROOT = Path(__file__).resolve().parents[3]
BUNDLE = ROOT / "policy/gate16"


def test_frozen_gate16_assets_match_manifest_and_graph_contract():
    manifest = json.loads((BUNDLE / "climb_policy_manifest.json").read_text())
    for filename, key in (
        ("policy.onnx", "base_sha256"),
        ("climb_residual.onnx", "residual_sha256"),
    ):
        path = BUNDLE / filename
        assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest[key]
        session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        input_meta = session.get_inputs()[0]
        output_meta = session.get_outputs()[0]
        assert (input_meta.name, input_meta.type, input_meta.shape[-1]) == (
            "obs",
            "tensor(float)",
            174,
        )
        assert input_meta.shape[0] in (1, "batch")
        assert (output_meta.name, output_meta.type, output_meta.shape[-1]) == (
            "actions",
            "tensor(float)",
            16,
        )
        assert output_meta.shape[0] in (1, "batch")
        action = session.run(["actions"], {"obs": np.zeros((1, 174), np.float32)})[0]
        assert action.shape == (1, 16)
        assert action.dtype == np.float32
        assert np.all(np.isfinite(action))


def _state(t: float, *, clear=False, owner="official") -> RobotState:
    if clear:
        wheels = np.array(
            [[0.10, 0.2, 0.60], [0.10, -0.2, 0.60], [0.08, 0.2, 0.60], [0.08, -0.2, 0.60]]
        )
        distance = -0.1
    else:
        wheels = np.array(
            [[-0.3, 0.2, 0.20], [-0.3, -0.2, 0.20], [-0.8, 0.2, 0.20], [-0.8, -0.2, 0.20]]
        )
        distance = 0.65
    return RobotState(
        t=t,
        segment=(15, 16),
        position=np.array([-distance, 0.0, 0.6]),
        yaw=0.0,
        pitch=0.0,
        roll=0.0,
        speed=0.25,
        yaw_rate=0.0,
        odom_time=t,
        lidar_time=t,
        heightmap_time=t,
        obstacle_distance=distance,
        travelled=max(0.0, t),
        joint_positions=np.zeros(16),
        joint_velocities=np.zeros(16),
        joint_torques=np.zeros(16),
        wheel_positions=wheels,
        wheel_contacts=np.ones(4, bool),
        obstacle_edge=np.array([0.0, 0.0]),
        obstacle_normal=np.array([1.0, 0.0]),
        actual_joint_owner=owner,
    )


def test_gate16_uses_delegated_owner_and_waits_for_acknowledged_handoff():
    policy = StableGate16Policy()
    config = RouterConfig(
        ready_dwell=0.04,
        verify_hold=0.04,
        min_entry_speed=0.10,
        max_entry_speed=0.30,
        target_entry_speed=0.25,
        verify_clearance=0.02,
    )
    router = Router(
        config,
        policies={"gate16": policy},
        segment_policies={(15, 16): "gate16"},
    )

    t = 0.0
    for _ in range(20):
        state = _state(t)
        out = router.tick(state, (0.25, 0.0, 0.0), observation_from_state(state))
        t += 0.02
        if router.mode is Mode.CLIMB:
            break
    assert router.mode is Mode.CLIMB

    state = _state(t, owner="gate16")
    out = router.tick(state, (0.25, 0.0, 0.0), observation_from_state(state))
    assert out.source is Source.POLICY
    assert router.active_action_kind is ActionKind.DELEGATED
    assert out.joints is None and out.command == (0.25, 0.0, 0.0)

    for _ in range(10):
        t += 0.02
        state = _state(t, clear=True, owner="gate16")
        out = router.tick(state, (0.5, 0.0, 0.0), observation_from_state(state))
        if router.mode is Mode.HANDOFF:
            break
    assert router.mode is Mode.HANDOFF
    assert out.source is Source.ROUTER

    t += 0.02
    state = _state(t, clear=True, owner="official")
    out = router.tick(state, (0.5, 0.0, 0.0), observation_from_state(state))
    assert router.mode is Mode.NAVIGATE
    assert out.source is Source.NAV
