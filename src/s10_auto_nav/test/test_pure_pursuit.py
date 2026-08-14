"""Steering behaviour of the pure-pursuit controller."""

import math

import numpy as np
import pytest

from s10_auto_nav.pure_pursuit import PurePursuitController, PursuitGains, wrap_angle

DT = 0.02

#: 27 degrees off the nose: inside the 75-degree pivot threshold, but far enough to one
#: side that the cross-track term saturates lateral.
OFFSET_TARGET = np.array([2.0, 1.0])


def settled(controller, position, yaw, target, steps=200):
    """Run long enough for the slew limiter to reach steady state."""
    command = None
    for _ in range(steps):
        command = controller.compute(np.asarray(position), yaw, np.asarray(target), DT)
    return command


def test_wrap_angle():
    assert wrap_angle(0.0) == 0.0
    assert math.isclose(wrap_angle(2 * math.pi), 0.0, abs_tol=1e-9)
    assert math.isclose(wrap_angle(math.pi / 2 + 2 * math.pi), math.pi / 2, abs_tol=1e-9)
    assert math.isclose(wrap_angle(-math.pi / 2 - 2 * math.pi), -math.pi / 2, abs_tol=1e-9)
    # The +/-pi boundary may land on either sign; only the magnitude is meaningful.
    assert math.isclose(abs(wrap_angle(3 * math.pi)), math.pi, abs_tol=1e-9)


def test_drives_forward_when_aligned():
    controller = PurePursuitController()
    command = settled(controller, [0.0, 0.0], 0.0, [50.0, 0.0])
    assert command.forward == pytest.approx(PursuitGains.max_forward)
    assert abs(command.yaw_rate) < 1e-3


def test_pivots_in_place_when_facing_away():
    controller = PurePursuitController()
    command = settled(controller, [0.0, 0.0], 0.0, [-50.0, 0.0])
    assert math.isclose(command.forward, 0.0, abs_tol=1e-6)
    assert abs(command.yaw_rate) > 0.5


def test_yaw_rate_points_toward_the_target():
    controller = PurePursuitController()
    left = settled(controller, [0.0, 0.0], 0.0, [10.0, 3.0])
    assert left.yaw_rate > 0.0

    controller.reset()
    right = settled(controller, [0.0, 0.0], 0.0, [10.0, -3.0])
    assert right.yaw_rate < 0.0


def test_slows_down_when_misaligned():
    controller = PurePursuitController()
    aligned = settled(controller, [0.0, 0.0], 0.0, [50.0, 0.0]).forward

    controller.reset()
    skewed = settled(controller, [0.0, 0.0], 0.0, [50.0, 40.0]).forward

    assert skewed < aligned


def test_brakes_on_final_approach():
    controller = PurePursuitController()
    far = settled(controller, [0.0, 0.0], 0.0, [50.0, 0.0]).forward

    controller.reset()
    near = settled(controller, [0.0, 0.0], 0.0, [0.2, 0.0]).forward

    assert near < far


def test_commands_respect_configured_limits():
    gains = PursuitGains(max_forward=1.0, max_lateral=0.3, max_yaw_rate=0.5)
    controller = PurePursuitController(gains)
    for target in ([50.0, 0.0], [10.0, 25.0], [-5.0, 5.0]):
        controller.reset()
        command = settled(controller, [0.0, 0.0], 0.0, target)
        assert abs(command.forward) <= gains.max_forward + 1e-6
        assert abs(command.lateral) <= gains.max_lateral + 1e-6
        assert abs(command.yaw_rate) <= gains.max_yaw_rate + 1e-6


def test_slew_limits_the_first_step():
    gains = PursuitGains(forward_slew=1.0)
    controller = PurePursuitController(gains)
    first = controller.compute(np.array([0.0, 0.0]), 0.0, np.array([50.0, 0.0]), DT)
    assert first.forward <= gains.forward_slew * DT + 1e-9


def test_slew_limits_lateral_too():
    """Lateral was unlimited and could snap to full scale in a single 20 ms step.

    That is an input the policy never saw in training, and it lands hardest where the
    robot is straddling a ledge: told to move sideways, it scrubs along the edge instead
    of rolling over it.
    """
    gains = PursuitGains(lateral_slew=1.0)
    controller = PurePursuitController(gains)
    # Off to one side but still inside the pivot threshold, so the controller actually
    # asks for lateral. Past that threshold it rotates on the spot and lateral is zero,
    # which would pass this assertion while testing nothing.
    first = controller.compute(np.array([0.0, 0.0]), 0.0, OFFSET_TARGET, DT)
    assert abs(first.lateral) <= gains.lateral_slew * DT + 1e-9


def test_lateral_reaches_its_target_over_successive_steps():
    controller = PurePursuitController(PursuitGains(lateral_slew=2.0))
    command = settled(controller, [0.0, 0.0], 0.0, OFFSET_TARGET)
    assert abs(command.lateral) == pytest.approx(controller.gains.max_lateral, abs=1e-6)


def test_zero_distance_target_is_safe():
    controller = PurePursuitController()
    command = controller.compute(np.array([1.0, 1.0]), 0.0, np.array([1.0, 1.0]), DT)
    assert command.forward == 0.0
    assert command.yaw_rate == 0.0
