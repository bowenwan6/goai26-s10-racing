from __future__ import annotations

import hashlib
import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import onnxruntime as ort

from s10_auto_nav.strategy.gate16_policy import (
    Gate16Config,
    StableGate16Policy,
    gate16_owner_request,
    gate16_should_own,
)
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
    assert manifest["format"] == "s10-gated-residual-front-tuck-v1.5"
    assert manifest["source_commit"].startswith("216b77a")
    profile_ref = manifest["front_tuck_command_profile"]
    profile = BUNDLE / profile_ref["file"]
    assert profile.name == "front_tuck_command_profiles.json"
    assert hashlib.sha256(profile.read_bytes()).hexdigest() == profile_ref["sha256"]
    profile_data = json.loads(profile.read_text())
    assert profile_data["version"] == 1
    assert any(item["name"] == "v025_yaw0_precontact_tuck" for item in profile_data["profiles"])
    assert profile_ref["runner_applies_profile"] is True
    integration = manifest["racing_integration"]
    assert integration["canonical_handoff_source"].startswith("216b77a")
    assert integration["entry_distance_m"] == [0.60, 0.65]
    assert integration["entry_speed_mps"] == [0.23, 0.27]
    assert integration["base_owner_prewarms_before_residual"] is False
    assert integration["fallback_owner_request"] == "gate16_climb_fallback"
    runtime = manifest["full_stack_runtime"]
    assert runtime["fallback_max_forward_mps"] == 0.15
    assert runtime["confidence_gated_fast_adapter"]["enabled"] is True
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
        min_entry_speed=0.23,
        max_entry_speed=0.27,
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
    assert out.command == (0.25, 0.0, 0.0)

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
    assert out.command == (0.5, 0.0, 0.0)

    t += 0.4
    state = _state(t, clear=True, owner="official")
    out = router.tick(state, (0.7, 0.2, 0.1), observation_from_state(state))
    assert out.command == (0.5, 0.0, 0.0)

    t += 0.5
    state = _state(t, clear=True, owner="official")
    out = router.tick(state, (0.7, 0.2, 0.1), observation_from_state(state))
    assert out.command == (0.7, 0.2, 0.1)


def test_gate16_adapter_identifies_v15_source():
    policy = StableGate16Policy(Gate16Config(fallback_command_forward=0.15))
    state = _state(0.0)
    policy.start(observation_from_state(state))
    action = policy.step(observation_from_state(state))
    assert action.info["owner"] == "gate16"
    assert action.info["runtime"] == "confidence_fallback_v1_5"
    assert action.info["profile"] == "unmatched"
    assert action.info["phase"] == "entry"


def test_gate16_adapter_uses_stable_command_when_fast_profile_is_unmatched():
    policy = StableGate16Policy(
        Gate16Config(
            command_forward=0.25,
            fallback_command_forward=0.15,
            obstacle_edge=(0.0, 0.0),
            obstacle_normal=(1.0, 0.0),
        )
    )
    entry = replace(_state(0.0), speed=0.15, forward_speed=0.15)
    policy.start(observation_from_state(entry))
    action = policy.step(observation_from_state(entry))

    assert action.info["runtime"] == "confidence_fallback_v1_5"
    assert action.info["entry_mode"] == "stable_fallback"
    assert action.info["profile"] == "unmatched"
    assert action.twist == (0.15, 0.0, 0.0)


def test_gate16_ros_adapter_executes_profile_from_verified_front_wheels():
    policy = StableGate16Policy(
        Gate16Config(
            profile_file=str(BUNDLE / "front_tuck_command_profiles.json"),
            obstacle_edge=(0.0, 0.0),
            obstacle_normal=(1.0, 0.0),
            deck_z=0.4787248,
            front_clearance=0.02,
        )
    )
    entry = replace(_state(0.0), yaw=np.deg2rad(-2.5))
    policy.start(observation_from_state(entry))
    action = policy.step(observation_from_state(entry))
    assert action.info["profile"] == "v025_yawm02p5_mirror_compensated"
    assert action.info["phase"] == "entry"
    assert action.twist == (0.25, 0.0, 0.0)

    front_clear = replace(
        entry,
        wheel_positions=np.array(
            [[0.03, 0.2, 0.60], [0.03, -0.2, 0.60], [-0.3, 0.2, 0.20], [-0.3, -0.2, 0.20]]
        ),
    )
    for step in range(30):
        observation = observation_from_state(
            replace(
                front_clear,
                t=(step + 1) * 0.02,
                odom_time=(step + 1) * 0.02,
                lidar_time=(step + 1) * 0.02,
                heightmap_time=(step + 1) * 0.02,
            )
        )
        action = policy.step(observation)
        assert action.info["phase"] == "settle"
        assert action.twist == (0.20, 0.0, 0.0)
    action = policy.step(observation)
    assert action.info["phase"] == "push"
    assert action.twist == (0.10, 0.0, 0.0)


def test_gate16_base_owns_only_after_official_far_field_alignment():
    policy = StableGate16Policy()
    for mode in (Mode.CLIMB_READY, Mode.CLIMB, Mode.VERIFY_CLEAR):
        assert gate16_should_own(policy, mode.value)
    for mode in (
        Mode.NAVIGATE,
        Mode.APPROACH,
        Mode.ALIGN,
        Mode.HANDOFF,
        Mode.RECOVER,
        Mode.ABORT,
        Mode.DONE,
    ):
        assert not gate16_should_own(policy, mode.value)
    assert not gate16_should_own(policy, Mode.ALIGN.value, prewarm_ready=True)
    assert gate16_owner_request(policy, Mode.ALIGN.value, prewarm_ready=True) == "official"
    assert gate16_owner_request(policy, Mode.APPROACH.value) == "gate16_shadow"
    assert gate16_owner_request(policy, Mode.ALIGN.value) == "gate16_shadow"
    for mode in (Mode.CLIMB_READY, Mode.CLIMB, Mode.VERIFY_CLEAR):
        assert gate16_owner_request(policy, mode.value) == "gate16_climb"
        assert (
            gate16_owner_request(policy, mode.value, entry_mode="stable_fallback")
            == "gate16_climb_fallback"
        )
    assert gate16_owner_request(policy, Mode.HANDOFF.value) == "official"


def test_official_actor_aligns_position_and_yaw_before_gate16_prewarm():
    policy = StableGate16Policy()
    router = Router(
        RouterConfig(
            ready_distance_min=0.60,
            ready_distance_max=0.65,
            gate16_staging_lead=0.25,
            gate16_staging_tolerance=0.10,
            min_entry_speed=0.23,
            max_entry_speed=0.27,
        ),
        policies={"gate16": policy},
        segment_policies={(15, 16): "gate16"},
    )
    router.mode = Mode.ALIGN

    far = replace(
        _state(0.02),
        obstacle_distance=1.50,
        lateral_error=0.45,
        heading_error=np.deg2rad(12.0),
    )
    out = router.tick(far, (0.7, 0.0, 0.0), observation_from_state(far))
    assert out.source is Source.ROUTER
    assert out.command[0] > 0.0
    assert out.command[1] == 0.0 and out.command[2] < 0.0
    assert not router.gate16_prewarm_ready

    staged = replace(
        far,
        t=0.04,
        odom_time=0.04,
        lidar_time=0.04,
        heightmap_time=0.04,
        obstacle_distance=0.90,
        lateral_error=0.10,
        heading_error=np.deg2rad(8.0),
    )
    out = router.tick(staged, (0.7, 0.0, 0.0), observation_from_state(staged))
    assert not router.gate16_prewarm_ready
    assert out.command[0] > 0.0

    staged = replace(
        staged,
        t=0.06,
        odom_time=0.06,
        lidar_time=0.06,
        heightmap_time=0.06,
        lateral_error=0.07,
    )
    out = router.tick(staged, (0.7, 0.0, 0.0), observation_from_state(staged))
    assert not router.gate16_prewarm_ready
    assert out.command[0] > 0.0

    staged = replace(
        staged,
        t=0.08,
        odom_time=0.08,
        lidar_time=0.08,
        heightmap_time=0.08,
        heading_error=np.deg2rad(4.0),
    )
    out = router.tick(staged, (0.7, 0.0, 0.0), observation_from_state(staged))
    assert router.gate16_prewarm_ready
    assert np.isclose(out.command[0], 0.25)
    assert (
        gate16_owner_request(policy, out.mode.value, prewarm_ready=router.gate16_prewarm_ready)
        == "official"
    )


def test_v15_prefers_fast_contract_for_strict_staging_pose():
    policy = StableGate16Policy()
    router = Router(
        RouterConfig(
            ready_distance_min=0.60,
            ready_distance_max=0.65,
            ready_dwell=0.0,
            min_entry_speed=0.23,
            max_entry_speed=0.27,
            target_entry_speed=0.25,
            gate16_fallback_enabled=True,
            gate16_fast_adapter_enabled=True,
        ),
        policies={"gate16": policy},
        segment_policies={(15, 16): "gate16"},
    )
    router.mode = Mode.ALIGN

    staged = replace(_state(0.02), obstacle_distance=0.90)
    out = router.tick(staged, (0.7, 0.0, 0.0), observation_from_state(staged))
    assert router.gate16_entry_mode == "fast_profile"
    assert router.gate16_prewarm_ready
    assert out.command == (0.25, 0.0, 0.0)

    entry = replace(
        staged,
        t=0.04,
        odom_time=0.04,
        lidar_time=0.04,
        heightmap_time=0.04,
        obstacle_distance=0.65,
    )
    out = router.tick(entry, (0.7, 0.0, 0.0), observation_from_state(entry))
    assert router.mode is Mode.CLIMB_READY
    assert out.command == (0.25, 0.0, 0.0)
    assert "fast_profile" in out.reason


def test_v15_uses_stable_contract_for_normal_imperfect_staging_pose():
    policy = StableGate16Policy(Gate16Config(command_forward=0.25, fallback_command_forward=0.15))
    router = Router(
        RouterConfig(
            ready_distance_min=0.60,
            ready_distance_max=0.65,
            ready_dwell=0.0,
            min_entry_speed=0.23,
            max_entry_speed=0.27,
            target_entry_speed=0.25,
            max_heading_error=np.deg2rad(2.5),
            max_lateral_error=0.08,
            gate16_fallback_enabled=True,
            gate16_fallback_ready_dwell=0.04,
            gate16_fallback_target_entry_speed=0.15,
            gate16_fallback_max_heading_error=np.deg2rad(6.0),
            gate16_fallback_max_lateral_error=0.25,
        ),
        policies={"gate16": policy},
        segment_policies={(15, 16): "gate16"},
    )
    router.mode = Mode.ALIGN

    staged = replace(
        _state(0.02),
        obstacle_distance=0.90,
        lateral_error=0.12,
        heading_error=np.deg2rad(4.0),
        speed=0.15,
        forward_speed=0.15,
    )
    out = router.tick(staged, (0.7, 0.0, 0.0), observation_from_state(staged))
    assert router.gate16_entry_mode == "stable_fallback"
    assert router.gate16_prewarm_ready
    assert out.command[0] == 0.15

    for index in range(1, 6):
        t = 0.02 + index * 0.02
        entry = replace(
            staged,
            t=t,
            odom_time=t,
            lidar_time=t,
            heightmap_time=t,
            obstacle_distance=0.66,
        )
        out = router.tick(entry, (0.7, 0.0, 0.0), observation_from_state(entry))
        if router.mode is Mode.CLIMB_READY:
            break

    assert router.mode is Mode.CLIMB_READY
    assert out.command == (0.15, 0.0, 0.0)
    assert "stable_fallback" in out.reason

    t += 0.02
    entry = replace(
        entry,
        t=t,
        odom_time=t,
        lidar_time=t,
        heightmap_time=t,
    )
    out = router.tick(entry, (0.7, 0.0, 0.0), observation_from_state(entry))
    assert router.mode is Mode.CLIMB
    assert out.command == (0.15, 0.0, 0.0)
    assert "stable_fallback" in out.reason


def test_v15_competition_default_does_not_admit_fast_adapter():
    policy = StableGate16Policy(Gate16Config(command_forward=0.25, fallback_command_forward=0.18))
    router = Router(
        RouterConfig(
            ready_distance_min=0.60,
            ready_distance_max=0.65,
            ready_dwell=0.0,
            min_entry_speed=0.23,
            max_entry_speed=0.27,
            target_entry_speed=0.25,
            gate16_fallback_enabled=True,
            gate16_fast_adapter_enabled=False,
            gate16_fallback_target_entry_speed=0.18,
        ),
        policies={"gate16": policy},
        segment_policies={(15, 16): "gate16"},
    )
    router.mode = Mode.ALIGN
    staged = replace(_state(0.02), obstacle_distance=0.90)
    out = router.tick(staged, (0.7, 0.0, 0.0), observation_from_state(staged))
    assert router.gate16_entry_mode == "stable_fallback"
    assert np.isclose(out.command[0], 0.18)


def test_v15_runner_preserves_owner_safety_and_confidence_fallback():
    source = (ROOT / "integration/gate16_policy_runner.hpp").read_text()
    assert "BeginClimb(gate, uc, ro.base_rpy(2))" in source
    assert "heightmap_yaw=" in source
    assert "const float base_heading_deg = base_yaw_rad" in source
    assert "climb_entry_heading_error_deg_ = base_heading_deg" in source
    assert "UpdatePolicyFrameBeforeArm(ro.base_rpy(2))" in source
    assert "staged_mirror_policy_frame" in source
    assert "action history seeded from measured state on actuator takeover" in source
    assert "SeedActionHistoryFromMeasuredState" in source
    assert "SetActuatorOwnership" in source
    assert "residual_available_ && climb_armed_" in source
    assert "S10 climb residual " in source
    assert "S10 v1.5 confidence-fallback climb config loaded" in source
    assert "fallback_forced_by_router_" in source
    assert "ShouldUseFastAdapter" in source
    assert "SetForceFallback" in source
    assert "fast_adapter=" in source
    assert "fallback_forward_cap_mps_" in source
    assert "residual_engaged_ = !fast_adapter_active_" in source
    assert "if (gate.active && residual_engaged_)" in source
    assert "supported_forward_floor_mps_" in source
    assert "supported_policy_frame_yaw_bias_rps_" in source


def test_v15_manifest_has_confidence_gate_and_native_fallback():
    manifest = json.loads((BUNDLE / "climb_policy_manifest.json").read_text())
    assert manifest["policy_symmetry"]["yaw_bands_deg"] == [
        [-27.5, -22.5],
        [-7.5, 7.5],
        [17.5, 90.0],
    ]
    runtime = manifest["full_stack_runtime"]
    confidence = runtime["confidence_gated_fast_adapter"]
    assert confidence["max_abs_entry_yaw_deg"] == 8.0
    assert confidence["min_edge_heading_samples"] == 4
    assert confidence["max_side_edge_skew_m"] == 0.2
    assert confidence["allow_policy_mirroring"] is False
    assert runtime["fallback_max_forward_mps"] == 0.15
    profile = manifest["front_tuck_command_profile"]
    assert profile["execution_owner"] == "gate16_policy_runner"
    assert profile["runner_applies_profile"] is True
    assert profile["unmatched_behavior"].startswith("use frozen release policy")
