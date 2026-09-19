from __future__ import annotations

import copy
import math

import numpy as np
import pytest
from conftest import admitted_session, snapshot

from real_transfer.shadow import VERIFICATIONS, ShadowSession


def blocked(result, text):
    assert not result["candidate_computed"], result
    assert result["candidate"] == [0, 0, 0]
    assert result["transport_command"] == [0, 0, 0]
    assert not result["motion_enabled"]
    assert any(text in reason for reason in result["reasons"]), result


def test_healthy_inputs_compute_bounded_candidate_but_never_motion(config):
    session = admitted_session(config)
    for i in range(62, 180):
        result = session.step(snapshot(i * 0.02))
        assert result["candidate_computed"]
        assert 0 < result["candidate"][0] <= 0.2
        assert np.linalg.norm(result["candidate"][:2]) <= 0.2 + 1e-12
        assert abs(result["candidate"][2]) <= 0.3
        assert result["transport_command"] == [0, 0, 0]
        assert result["motion_enabled"] is False


@pytest.mark.parametrize("mode", ["real", "armed", "live", "simulation", True, None])
def test_cannot_enable_hardware_by_config(config, mode):
    config["mode"] = mode
    with pytest.raises(ValueError, match="shadow_only"):
        ShadowSession(config)


@pytest.mark.parametrize("key", VERIFICATIONS)
def test_unverified_contract_never_admits(config, key):
    config["verified"][key] = False
    session = ShadowSession(config)
    for i in range(70):
        result = session.step(snapshot(i * 0.02))
    blocked(result, "unverified_" + key)


@pytest.mark.parametrize("name", ["pose", "cloud", "scan", "localization"])
def test_missing_input_revokes_admission_and_requires_session_restart(config, name):
    session = admitted_session(config)
    data = snapshot(1.22)
    del data["inputs"][name]
    blocked(session.step(data), "missing_" + name)
    blocked(session.step(snapshot(1.24)), "session_requires_restart")


@pytest.mark.parametrize("name", ["pose", "cloud", "scan", "localization"])
@pytest.mark.parametrize("offset", [-10, 10, math.nan, math.inf])
def test_bad_source_times_cannot_be_freshened_by_arrival(config, name, offset):
    session = admitted_session(config)
    data = snapshot(1.22)
    data["inputs"][name]["stamp"] += offset
    result = session.step(data)
    assert not result["candidate_computed"]
    assert result["transport_command"] == [0, 0, 0]
    assert any(name in r for r in result["reasons"])


@pytest.mark.parametrize("name", ["pose", "cloud", "scan", "localization"])
def test_retransmitted_source_sample_is_not_new_evidence(config, name):
    session = admitted_session(config)
    data = snapshot(1.22)
    data["inputs"][name]["stamp"] = 1001.2
    blocked(session.step(data), "replayed_or_reordered_" + name)


def test_reusing_cached_status_keeps_original_arrival_and_is_allowed(config):
    session = admitted_session(config)
    data = snapshot(1.22)
    data["inputs"]["localization"] = snapshot(1.2)["inputs"]["localization"]
    assert session.step(data)["candidate_computed"]


@pytest.mark.parametrize(
    "field,value,reason",
    [
        ("code", 3, "global_localization_not_valid"),
        ("global", False, "global_localization_not_valid"),
        ("map_id", "old-map", "map_mismatch"),
        ("session_id", "restarted-service", "localization_session_changed"),
    ],
)
def test_localization_state_not_frame_name_controls_admission(config, field, value, reason):
    session = admitted_session(config)
    data = snapshot(1.22)
    data["inputs"]["localization"][field] = value
    blocked(session.step(data), reason)


def test_wall_clock_jump_is_latched_even_if_all_source_clocks_jump_together(config):
    session = admitted_session(config)
    data = snapshot(1.22)
    data["wall_time"] += 1
    for item in data["inputs"].values():
        item["stamp"] += 1
    blocked(session.step(data), "clock_jump")


def test_pose_jump_requires_restart(config):
    session = admitted_session(config)
    data = snapshot(1.22)
    data["inputs"]["pose"]["position"][0] = 2
    blocked(session.step(data), "pose_jump")


def test_sensor_timestamp_skew_is_rejected(config):
    session = admitted_session(config)
    data = snapshot(1.28)
    data["inputs"]["cloud"]["stamp"] -= 0.07
    blocked(session.step(data), "pose_cloud_not_synchronized")


def test_intermittent_wifi_or_frozen_callbacks_expire_using_monotonic_time(config):
    session = admitted_session(config)
    frozen = snapshot(1.2)
    for i in range(61, 81):
        frozen["wall_time"], frozen["monotonic_time"] = 1000 + i * 0.02, 10 + i * 0.02
        result = session.step(frozen)
    blocked(result, "arrival_age_pose")


@pytest.mark.parametrize("name", ["pose", "cloud", "scan"])
def test_wrong_frame_is_rejected(config, name):
    session = admitted_session(config)
    data = snapshot(1.22)
    data["inputs"][name]["frame"] = "wrong"
    blocked(session.step(data), "frame_mismatch")


def test_partial_heightmap_never_falls_back_to_plain_pursuit(config):
    session = admitted_session(config)
    data = snapshot(1.22)
    data["inputs"]["cloud"]["points"] = data["inputs"]["cloud"]["points"][3:]
    blocked(session.step(data), "heightmap_unknown_or_ambiguous")


@pytest.mark.parametrize("value", [math.nan, -math.inf, 0, -1, 20])
def test_invalid_scan_is_not_converted_to_clear_space(config, value):
    session = admitted_session(config)
    data = snapshot(1.22)
    data["inputs"]["scan"]["ranges"][36] = value
    blocked(session.step(data), "invalid_or_unknown_scan")


def test_json_no_return_mask_preserves_free_vs_unknown(config):
    session = admitted_session(config)
    data = snapshot(1.22)
    data["inputs"]["scan"]["ranges"] = [None] * 72
    data["inputs"]["scan"]["no_return"] = [True] * 72
    assert session.step(data)["candidate_computed"]
    data = copy.deepcopy(data)
    data["wall_time"] += 0.02
    data["monotonic_time"] += 0.02
    data["inputs"]["scan"]["no_return"][36] = False
    blocked(session.step(data), "invalid_or_unknown_scan")


def test_same_xy_on_other_floor_does_not_consume_waypoint(config):
    config["route"][0]["position"] = [0, 0, 3.42]
    session = ShadowSession(config)
    for i in range(70):
        result = session.step(snapshot(i * 0.02))
    blocked(result, "target_level_mismatch")
    assert session.cursor == 0


def test_only_one_waypoint_consumed_per_tick_and_hold_is_zero(config):
    wp = config["route"][0]
    wp["position"] = [0, 0, 0.42]
    config["route"] = [copy.deepcopy(wp) for _ in range(3)]
    session = ShadowSession(config)
    for i in range(51):
        result = session.step(snapshot(i * 0.02))
    assert session.cursor == 1
    blocked(result, "waypoint_reached_hold_one_tick")


@pytest.mark.parametrize("kind", ["stairs", "ramp", "unknown"])
def test_special_terrain_cannot_silently_activate_gate16(config, kind):
    config["route"][0]["kind"] = kind
    session = ShadowSession(config)
    for i in range(70):
        result = session.step(snapshot(i * 0.02))
    blocked(result, "segment_requires_separate_validation")


def test_all_around_obstacle_blocks_without_reverse_recovery(config):
    session = admitted_session(config)
    data = snapshot(1.22)
    data["inputs"]["scan"]["ranges"] = [0.3] * 72
    blocked(session.step(data), "obstacle_blocks_path")


def test_control_stall_does_not_get_large_slew_budget(config):
    session = admitted_session(config)
    blocked(session.step(snapshot(1.5)), "control_tick_gap")


def test_initial_bad_pose_followed_by_valid_pose_does_not_crash(config):
    session = ShadowSession(config)
    bad = snapshot()
    bad["inputs"]["pose"]["orientation"] = [0, 0, 0, 0]
    blocked(session.step(bad), "invalid_input")
    result = session.step(snapshot(0.02))
    assert not result["candidate_computed"]


@pytest.mark.parametrize("bad", [None, [], {"inputs": []}, {"inputs": {"pose": 5}}])
def test_malformed_snapshot_has_explicit_error(config, bad):
    with pytest.raises(ValueError):
        ShadowSession(config).step(bad)
