"""The observation layout must stay locked to what the deployment SDK expects."""

import pytest

from s10_rl.observation import (
    ACTION_SCALE,
    BASELINE,
    DEFAULT_JOINT_POS,
    MOTOR_NUM,
    PERCEPTIVE,
    PHASE,
    POLICY_ORDER,
    ROBOT_ORDER,
    ObservationTerm,
    cpp_constant,
)


def test_baseline_matches_the_shipped_policy():
    """57 is hardcoded in s10_policy_runner.hpp; drifting from it breaks deployment."""
    assert BASELINE.dim == 57


def test_baseline_offsets_are_contiguous():
    offsets = BASELINE.offsets()
    assert offsets["base_angular_velocity"] == (0, 3)
    assert offsets["projected_gravity"] == (3, 6)
    assert offsets["velocity_command"] == (6, 9)
    assert offsets["joint_position"] == (9, 25)
    assert offsets["joint_velocity"] == (25, 41)
    assert offsets["last_action"] == (41, 57)


def test_perception_is_appended_not_inserted():
    """Baseline offsets must survive so perceptive policies can warm-start from them."""
    baseline_offsets = BASELINE.offsets()
    perceptive_offsets = PERCEPTIVE.offsets()
    for name, span in baseline_offsets.items():
        assert perceptive_offsets[name] == span

    assert perceptive_offsets["heightmap"][0] == BASELINE.dim
    assert PERCEPTIVE.dim > BASELINE.dim


def test_phase_preserves_the_official_prefix():
    assert PHASE.dim == 59 and PHASE.offsets()['phase'] == (57, 59)
    assert PHASE.terms[:-1] == BASELINE.terms


def test_joint_orders_are_permutations_of_each_other():
    assert len(ROBOT_ORDER) == MOTOR_NUM
    assert len(POLICY_ORDER) == MOTOR_NUM
    assert sorted(ROBOT_ORDER) == sorted(POLICY_ORDER)


def test_policy_order_groups_legs_before_wheels():
    wheels = [i for i, name in enumerate(POLICY_ORDER) if "wheel" in name]
    assert wheels == [12, 13, 14, 15]


def test_defaults_and_scales_cover_every_joint():
    assert len(DEFAULT_JOINT_POS) == MOTOR_NUM
    assert len(ACTION_SCALE) == MOTOR_NUM


def test_wheels_default_to_zero():
    assert DEFAULT_JOINT_POS[12:] == [0.0, 0.0, 0.0, 0.0]


def test_extending_does_not_mutate_the_source_spec():
    before = BASELINE.dim
    BASELINE.extended_with("scratch", ObservationTerm("extra", 5, "unused"))
    assert BASELINE.dim == before


def test_describe_lists_every_term():
    text = PERCEPTIVE.describe()
    for term in PERCEPTIVE.terms:
        assert term.name in text


@pytest.mark.parametrize("spec", [BASELINE, PERCEPTIVE])
def test_cpp_constant_reports_the_spec_dimension(spec):
    assert f"= {spec.dim};" in cpp_constant(spec)
