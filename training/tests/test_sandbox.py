"""The sandbox is only useful if it is the race's dynamics and not an approximation of them.

Every manoeuvre designed offline is trusted on the strength of that equivalence, so the
things that would silently break it -- the control law, the gain layout, the joint ordering,
the torque limits -- are pinned here rather than left to be discovered when a motion that
worked in search does nothing on the course.
"""

import numpy as np
import pytest
from s10_climb.sandbox import (
    JOINT_INIT,
    JOINT_NAMES,
    POLICY_KD,
    POLICY_KP,
    SIM_TIMESTEP,
    Sandbox,
    find_track_xml,
    gains,
)


def _has_track():
    try:
        find_track_xml()
    except FileNotFoundError:
        return False
    return True


#: The scene is fetched by scripts/setup_upstream.sh rather than committed, so these skip
#: rather than fail on a checkout that has not been set up yet.
pytestmark = pytest.mark.skipif(
    not _has_track(), reason="track scene not installed; run scripts/setup_upstream.sh"
)


def test_joint_order_is_the_actuator_order():
    """The simulator indexes ctrl by actuator, so a reordering here mislabels every torque."""
    assert JOINT_NAMES[:4] == ["fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint"]
    assert len(JOINT_NAMES) == 16
    assert JOINT_NAMES[3::4] == [f"{leg}_wheel_joint" for leg in ("fl", "fr", "hl", "hr")]


def test_timestep_is_the_races_and_not_the_scenes():
    """The simulator overrides the XML, and the coarser step cannot resolve the Gate 16 face.

    Left at the scene's own 0.002 the contact solver lets a wheel arrive at the vertical wall
    with an unresolvable overlap and answers with an impulse that throws the robot metres up,
    which looks like a manoeuvre and is not one.
    """
    assert SIM_TIMESTEP == 0.001
    assert Sandbox().model.opt.timestep == pytest.approx(SIM_TIMESTEP)


def test_wheels_are_velocity_controlled():
    """kp on a wheel is forced to zero by the SDK; a position command there does nothing."""
    assert list(POLICY_KP[3::4]) == [0.0] * 4
    assert all(POLICY_KD[3::4] > 0.0)

    kp, _kd = gains(300.0)
    assert list(kp[3::4]) == [0.0] * 4
    assert list(kp[::4]) == [300.0] * 4


def test_damping_scales_with_stiffness():
    """Raising kp alone leaves the leg underdamped, which shows up as bouncing off the lip."""
    _, kd_soft = gains(80.0)
    _, kd_stiff = gains(320.0)
    assert kd_soft[0] == pytest.approx(2.0)
    # Four times the stiffness wants twice the damping to hold the same ratio.
    assert kd_stiff[0] == pytest.approx(4.0)


def test_joint_init_is_upstreams_pose():
    """Copied from mujoco_simulation_ros2.py; drift makes every offline result meaningless."""
    assert JOINT_INIT.shape == (16,)
    assert list(JOINT_INIT[3::4]) == [0.0] * 4
    assert JOINT_INIT[1] == pytest.approx(-1.16)
    assert JOINT_INIT[9] == pytest.approx(1.16)


def test_torque_limits_match_the_model():
    box = Sandbox()
    # Legs are the three non-wheel joints of each leg; wheels are far weaker.
    assert box.torque_hi[0] == pytest.approx(50.0)
    assert box.torque_hi[3] == pytest.approx(14.0)
    assert np.all(box.torque_lo == -box.torque_hi)


def test_control_law_is_the_simulators():
    """kp*(pos-q) + kd*(vel-dq) + tau_ff, clipped, written to ctrl. Checked by construction."""
    box = Sandbox()
    box.reset((11.60, 32.50, 0.45))

    target = JOINT_INIT.copy()
    target[1] += 0.10
    kp, kd = gains(150.0)
    expected = np.clip(kp * (target - box.q) + kd * (0.0 - box.dq), box.torque_lo, box.torque_hi)
    box.step(target, kp=kp, kd=kd)
    assert np.allclose(box.data.ctrl, expected)


def test_saturation_is_reported():
    """A manoeuvre that silently clips is one whose force margins are imaginary."""
    box = Sandbox()
    box.reset((11.60, 32.50, 0.45))
    assert not box.saturated.any()

    absurd = JOINT_INIT.copy()
    absurd[1] += 5.0
    box.step(absurd, kp=np.full(16, 1000.0), kd=np.zeros(16))
    assert box.saturated[1]


def test_robot_stands_still_on_the_pit_floor():
    """The whole method rests on this: the deployed gains hold the shipped pose upright."""
    box = Sandbox()
    box.reset((11.60, 32.50, 0.102 + 0.35))
    start = box.base_pos
    box.hold(3.0, JOINT_INIT)
    end = box.base_pos

    assert end[2] > 0.102, "fell through the floor"
    assert np.linalg.norm(end[:2] - start[:2]) < 0.10, "drifted while asked to stand still"
    assert box.tilt_deg() < 10.0, "toppled while asked to stand still"
