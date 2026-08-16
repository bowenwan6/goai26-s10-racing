"""What the router must never do, expressed as tests.

These are mostly negative: the router's value is in its refusals, so most of the file is
about states it must not enter and hand-overs it must not make. The positive path is short
and appears once.

The whole file runs against the pure state machine with no ROS and no simulator, which is why
a fall, a dead lidar and a lying policy can each be driven in a few microseconds.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from s10_auto_nav.strategy.gates import Debounced, Hysteresis
from s10_auto_nav.strategy.mock_policy import MockClimbPolicy, MockScenario
from s10_auto_nav.strategy.policy import ActionKind, PolicyStatus
from s10_auto_nav.strategy.router import (
    Mode,
    RobotState,
    Router,
    RouterConfig,
    Source,
    observation_from_state,
)

SEGMENT = (15, 16)


def state(t: float, **kw) -> RobotState:
    """A robot that is doing everything right, unless the test says otherwise."""
    base = dict(
        t=t,
        segment=SEGMENT,
        position=np.array([0.0, 0.0, 0.5]),
        yaw=0.0,
        pitch=0.0,
        roll=0.0,
        speed=0.0,
        yaw_rate=0.0,
        odom_time=t,
        lidar_time=t,
        heightmap_time=t,
        obstacle_distance=10.0,
        lateral_error=0.0,
        heading_error=0.0,
        travelled=0.0,
    )
    base.update(kw)
    return RobotState(**base)


def cleared(t: float, **kw) -> RobotState:
    """A robot that has genuinely got all four wheels over the edge."""
    wheels = np.array([[0.3, 0.2, 0.6], [0.3, -0.2, 0.6], [-0.3, 0.2, 0.6], [-0.3, -0.2, 0.6]])
    kw.setdefault("obstacle_distance", -1.0)
    kw.setdefault("wheel_positions", wheels)
    kw.setdefault("wheel_contacts", np.ones(4, bool))
    return state(t, **kw)


def make_router(scenario=MockScenario.SUCCEED, *, config=None, **policy_kw):
    policy = MockClimbPolicy(scenario, **policy_kw)
    router = Router(
        config or RouterConfig(),
        policies={"climb_policy": policy},
        segment_policies={SEGMENT: "climb_policy"},
    )
    return router, policy


def drive(router, states, nav=(0.5, 0.0, 0.0)):
    """Feed a sequence of states; return every output."""
    outs = []
    for s in states:
        outs.append(router.tick(s, nav, observation_from_state(s)))
    return outs


def run_to_climb(router, *, t0=0.0, rate=50.0):
    """Take a fresh router through NAVIGATE -> APPROACH -> ALIGN -> CLIMB."""
    dt = 1.0 / rate
    t = t0
    for _ in range(int(6.0 * rate)):
        s = state(t, obstacle_distance=0.6)
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += dt
        if router.mode is Mode.CLIMB:
            break
    return t


# --------------------------------------------------------------- gates


def test_hysteresis_does_not_chatter_at_the_threshold():
    h = Hysteresis(enter=1.2, exit_=1.5, rising=False)
    assert h.update(1.3) is False
    assert h.update(1.19) is True
    # Back above the entry threshold but below the exit one: it must stay latched.
    assert h.update(1.35) is True
    assert h.update(1.55) is False


def test_debounce_requires_the_condition_to_persist():
    d = Debounced(dwell_true=0.1, dwell_false=0.04)
    for _ in range(4):
        assert d.update(True, 0.02) is False
    assert d.update(True, 0.02) is True
    # Falling is faster than rising, by construction, but still not instant: the dwell is
    # reached when the accumulated time *meets* it, so 0.04 takes two ticks of 0.02.
    assert d.update(False, 0.02) is True
    assert d.update(False, 0.02) is False


def test_debounce_forgets_a_condition_that_does_not_hold():
    """A blip must not accumulate towards the dwell across the gaps between blips."""
    d = Debounced(dwell_true=0.1)
    for _ in range(20):
        assert d.update(True, 0.02) is False
        assert d.update(False, 0.02) is False


def test_hysteresis_rejects_a_degenerate_configuration():
    with pytest.raises(ValueError):
        Hysteresis(enter=1.0, exit_=1.0, rising=False)


# --------------------------------------------------------------- entry gating


def test_climb_does_not_start_outside_the_capture_envelope():
    router, policy = make_router()
    t = 0.0
    for _ in range(600):
        # Close to the obstacle, but skewed: heading error far outside the envelope.
        s = state(t, obstacle_distance=0.6, heading_error=math.radians(40.0))
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
    assert policy.start_count == 0
    assert router.mode is not Mode.CLIMB


def test_climb_does_not_start_while_still_moving():
    router, policy = make_router()
    t = 0.0
    for _ in range(400):
        s = state(t, obstacle_distance=0.6, speed=0.6)
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
    assert policy.start_count == 0


def test_entry_requires_the_envelope_to_be_held_not_merely_touched():
    """One good tick is not an entry state. This is the dwell doing its job."""
    router, policy = make_router()
    t = 0.0
    for i in range(400):
        # Alternate in and out of the envelope every tick.
        bad = math.radians(30.0) if i % 2 else 0.0
        s = state(t, obstacle_distance=0.6, heading_error=bad)
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
    assert policy.start_count == 0


def test_the_happy_path_reaches_climb_and_resets_before_starting():
    router, policy = make_router()
    run_to_climb(router)
    assert router.mode is Mode.CLIMB
    assert policy.start_count == 1
    # reset before start, every time -- a policy carrying history from a previous attempt is
    # being fed a discontinuity it never saw in training.
    assert policy.reset_count >= 1


# --------------------------------------------------------------- command ownership


def test_exactly_one_source_owns_the_boundary_every_tick():
    router, _ = make_router()
    t, dt = 0.0, 0.02
    seen = set()
    for _ in range(1500):
        s = (
            state(t, obstacle_distance=0.6)
            if router.mode is not Mode.CLIMB
            else cleared(t, obstacle_distance=0.6)
        )
        out = router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        assert isinstance(out.source, Source)
        # A joint action and a twist may never be present together.
        assert out.joints is None or out.command == (0.0, 0.0, 0.0)
        seen.add(out.source)
        t += dt
    assert Source.NAV in seen and Source.ROUTER in seen


def test_navigation_never_owns_the_boundary_during_a_climb():
    router, _ = make_router(duration=1.0)
    t = run_to_climb(router)
    for _ in range(30):
        s = state(t, obstacle_distance=0.4)
        out = router.tick(s, (0.9, 0.0, 0.0), observation_from_state(s))
        if router.mode is not Mode.CLIMB:
            break
        assert out.source is Source.POLICY
        assert out.command != (0.9, 0.0, 0.0)
        t += 0.02


def test_joint_policy_actions_reach_the_boundary_as_joints():
    router, _ = make_router(action_kind=ActionKind.JOINT, duration=1.0)
    t = run_to_climb(router)
    s = state(t, obstacle_distance=0.4)
    out = router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
    assert out.source is Source.POLICY
    assert out.joints is not None and out.joints.shape == (16,)
    assert out.command == (0.0, 0.0, 0.0)


# --------------------------------------------------------------- safety


def test_stale_sensors_stop_the_robot_immediately():
    router, _ = make_router()
    s = state(1.0, odom_time=0.0)  # a full second old, twice the timeout
    out = router.tick(s, (0.7, 0.0, 0.0), observation_from_state(s))
    assert out.source is Source.ROUTER
    assert out.command == (0.0, 0.0, 0.0)
    # Not Source.NONE: publishing nothing would leave the previous command driving.
    assert out.source is not Source.NONE


def test_persistently_stale_sensors_abort_the_run():
    router, _ = make_router()
    for i in range(400):
        t = 1.0 + i * 0.02
        s = state(t, odom_time=0.0)
        router.tick(s, (0.7, 0.0, 0.0), observation_from_state(s))
    assert router.mode is Mode.ABORT


def test_stale_sensors_during_a_climb_cancel_the_policy():
    router, policy = make_router(duration=10.0)
    t = run_to_climb(router)
    s = state(t + 1.0, obstacle_distance=0.4, odom_time=t - 1.0)
    router.tick(s, (0.0, 0.0, 0.0), observation_from_state(s))
    assert router.mode is Mode.RECOVER
    assert policy.cancel_count == 1


def test_excessive_tilt_aborts_and_cancels():
    router, policy = make_router(duration=10.0)
    t = run_to_climb(router)
    s = state(t, obstacle_distance=0.4, pitch=math.radians(70.0))
    out = router.tick(s, (0.0, 0.0, 0.0), observation_from_state(s))
    assert router.mode is Mode.ABORT
    assert out.command == (0.0, 0.0, 0.0)
    assert policy.cancel_count == 1


def test_a_single_torque_spike_is_not_a_violation():
    """Contacts here are 5 ms springs; a one-tick test would fire constantly."""
    router, _ = make_router()
    t = 0.0
    for i in range(10):
        tau = np.zeros(16)
        if i == 5:
            tau[0] = 400.0
        s = state(t, joint_torques=tau)
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
    assert router.mode is not Mode.ABORT


def test_sustained_torque_over_the_ceiling_aborts():
    router, _ = make_router()
    t = 0.0
    for _ in range(200):
        s = state(t, joint_torques=np.full(16, 400.0))
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
    assert router.mode is Mode.ABORT


# --------------------------------------------------------------- policy outcomes


@pytest.mark.parametrize("scenario", [MockScenario.FAIL, MockScenario.RAISE])
def test_a_failing_policy_routes_to_recover(scenario):
    router, _ = make_router(scenario, duration=0.5)
    t = run_to_climb(router)
    for _ in range(200):
        s = state(t, obstacle_distance=0.4)
        router.tick(s, (0.0, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        if router.mode in (Mode.RECOVER, Mode.ALIGN, Mode.ABORT):
            break
    assert router.mode in (Mode.RECOVER, Mode.ALIGN, Mode.ABORT)


def test_a_policy_that_never_finishes_is_ended_by_the_timeout():
    cfg = RouterConfig(climb_timeout=2.0, climb_progress_window=30.0)
    router, policy = make_router(MockScenario.NEVER_FINISH, config=cfg)
    t = run_to_climb(router)
    for i in range(400):
        s = state(t, obstacle_distance=0.4, travelled=i * 0.02)
        router.tick(s, (0.0, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        if router.mode is not Mode.CLIMB:
            break
    assert router.mode in (Mode.RECOVER, Mode.ALIGN)
    assert policy.cancel_count >= 1


def test_a_policy_that_goes_silent_is_caught_by_progress_not_status():
    cfg = RouterConfig(climb_timeout=60.0, climb_progress_window=1.0)
    router, _ = make_router(MockScenario.GO_SILENT, config=cfg, duration=0.2)
    t = run_to_climb(router)
    for _ in range(400):
        s = state(t, obstacle_distance=0.4, travelled=0.0)
        router.tick(s, (0.0, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        if router.mode is not Mode.CLIMB:
            break
    assert router.mode is Mode.RECOVER
    assert "no progress" in router.history[-1][3]


def test_cancel_stops_the_policy_immediately():
    router, policy = make_router(duration=10.0)
    run_to_climb(router)
    router._cancel_active("test")
    assert policy.cancel_count == 1
    assert policy.result().status is PolicyStatus.CANCELLED


# --------------------------------------------------------------- verification


def test_a_lying_policy_does_not_get_back_to_navigate():
    """The single most important test here.

    ``LIE`` reports SUCCEEDED with the robot still short of the edge. If this passes into
    NAVIGATE then VERIFY_CLEAR is decorative and the router is just a relay.
    """
    router, _ = make_router(MockScenario.LIE, duration=0.5)
    t = run_to_climb(router)
    modes = []
    for _ in range(600):
        s = state(t, obstacle_distance=0.4)  # still in front of the edge, no wheel data
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        modes.append(router.mode)
        t += 0.02
    assert Mode.VERIFY_CLEAR in modes
    # The claim is precisely that the lie never buys a hand-back, at any point in the run --
    # not that the router ends in some particular one of the states that are not NAVIGATE.
    assert Mode.NAVIGATE not in modes
    # It must have gone through RECOVER rather than straight back to driving the course.
    assert any(to is Mode.RECOVER for _, _, to, _ in router.history)


def test_verification_fails_closed_when_wheel_state_is_missing():
    router, _ = make_router(duration=0.5)
    t = run_to_climb(router)
    for _ in range(600):
        # obstacle_distance says the base is past the edge, but there is no wheel data at
        # all: "cannot check" must not read as "fine".
        s = state(t, obstacle_distance=-1.0)
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        if router.mode is Mode.RECOVER:
            break
    assert router.mode is Mode.RECOVER


def test_verification_rejects_too_few_contacts():
    router, _ = make_router(duration=0.5)
    t = run_to_climb(router)
    for _ in range(600):
        s = cleared(t, wheel_contacts=np.array([True, False, False, False]))
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        if router.mode is Mode.RECOVER:
            break
    assert router.mode is Mode.RECOVER


def test_a_genuine_climb_is_verified_and_hands_back_to_navigate():
    router, _ = make_router(duration=0.5)
    t = run_to_climb(router)
    for _ in range(600):
        s = cleared(t)
        out = router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        if router.mode is Mode.NAVIGATE:
            assert out.source is Source.NAV
            break
    assert router.mode is Mode.NAVIGATE
    assert any(to is Mode.VERIFY_CLEAR for _, _, to, _ in router.history)


def test_verification_is_retracted_if_the_robot_cannot_drive_away():
    """Standing on the lip with nowhere to go is not a completed climb."""
    cfg = RouterConfig(resume_timeout=1.0, resume_distance=0.5)
    router, _ = make_router(config=cfg, duration=0.5)
    t = run_to_climb(router)
    for _ in range(600):
        s = cleared(t, travelled=0.0)
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        if router.mode is Mode.NAVIGATE:
            break
    assert router.mode is Mode.NAVIGATE
    for _ in range(200):  # never moves
        s = cleared(t, travelled=0.0)
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        if router.mode is Mode.RECOVER:
            break
    assert router.mode is Mode.RECOVER


def test_driving_away_confirms_the_climb_and_stays_in_navigate():
    router, _ = make_router(duration=0.5)
    t = run_to_climb(router)
    travelled = 0.0
    for _ in range(600):
        s = cleared(t, travelled=travelled)
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        if router.mode is Mode.NAVIGATE:
            break
    for _ in range(200):
        travelled += 0.02
        s = cleared(t, travelled=travelled, obstacle_distance=-5.0)
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
    assert router.mode is Mode.NAVIGATE


def test_a_cleared_segment_is_not_climbed_a_second_time():
    """The regression that the rest of this file missed.

    Immediately after a verified climb the edge is a metre *behind* the robot -- which is
    still inside the approach radius, because that radius is a distance band and does not
    care about sign. The router used to re-arm on it, re-align, and replay the climb, and
    since verification then passed again it did so forever. Two ticks of the fix are being
    checked here: the segment latch, and the requirement that the edge be ahead.
    """
    router, policy = make_router(duration=0.5)
    t = run_to_climb(router)
    for _ in range(600):
        s = cleared(t, travelled=0.0)
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        if router.mode is Mode.NAVIGATE:
            break
    assert router.mode is Mode.NAVIGATE
    starts_after_first_climb = policy.start_count
    assert starts_after_first_climb == 1

    # Now drive on, with the cleared edge sitting right behind at a distance the approach
    # band happily accepts, and the robot making the progress the resume check wants.
    travelled = 0.0
    for _ in range(400):
        travelled += 0.02
        s = cleared(t, travelled=travelled, obstacle_distance=-1.0)
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        assert router.mode is Mode.NAVIGATE, f"re-armed on a cleared segment: {router.mode}"
    assert policy.start_count == starts_after_first_climb


def test_a_different_segment_still_arms_after_one_is_cleared():
    """The latch must be per segment, or the second obstacle of a course is ignored."""
    policy = MockClimbPolicy(MockScenario.SUCCEED, duration=0.5)
    router = Router(
        RouterConfig(),
        policies={"climb_policy": policy},
        segment_policies={SEGMENT: "climb_policy", (20, 21): "climb_policy"},
    )
    t = run_to_climb(router)
    for _ in range(600):
        s = cleared(t, travelled=0.0)
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        if router.mode is Mode.NAVIGATE:
            break
    assert router.mode is Mode.NAVIGATE

    travelled = 0.0
    for _ in range(200):
        travelled += 0.02
        s = state(t, segment=(20, 21), obstacle_distance=2.0, travelled=travelled)
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        if router.mode is Mode.APPROACH:
            break
    assert router.mode is Mode.APPROACH


# --------------------------------------------------------------- recovery


def test_retries_are_bounded_and_end_in_abort():
    cfg = RouterConfig(max_retries=2, recover_duration=0.1, align_timeout=0.2, climb_timeout=0.2)
    router, _ = make_router(MockScenario.FAIL, config=cfg, duration=0.05)
    t = 0.0
    for _ in range(4000):
        s = state(t, obstacle_distance=0.6, heading_error=math.radians(40.0))
        router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        t += 0.02
        if router.mode is Mode.ABORT:
            break
    assert router.mode is Mode.ABORT
    assert router.retries > cfg.max_retries


def test_abort_is_terminal_and_holds_the_robot_still():
    router, _ = make_router()
    s = state(0.0, pitch=math.radians(80.0))
    router.tick(s, (0.7, 0.0, 0.0), observation_from_state(s))
    assert router.mode is Mode.ABORT
    for _ in range(50):
        out = router.tick(state(1.0), (0.7, 0.0, 0.0), observation_from_state(state(1.0)))
        assert out.mode is Mode.ABORT
        assert out.source is Source.ROUTER
        assert out.command == (0.0, 0.0, 0.0)


# --------------------------------------------------------------- configuration


def test_a_segment_without_a_policy_is_ordinary_navigation():
    policy = MockClimbPolicy()
    router = Router(
        RouterConfig(),
        policies={"climb_policy": policy},
        segment_policies={(20, 21): "climb_policy"},
    )
    t = 0.0
    for _ in range(500):
        s = state(t, segment=(3, 4), obstacle_distance=0.2)
        out = router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
        assert out.source is Source.NAV
        assert router.mode is Mode.NAVIGATE
        t += 0.02
    assert policy.start_count == 0


def test_the_segment_to_policy_mapping_is_the_only_place_the_gate_is_named():
    router, _ = make_router()
    assert router.policy_for(SEGMENT) == "climb_policy"
    assert router.policy_for((3, 4)) == ""


def test_course_completion_ends_in_done():
    router, _ = make_router()
    s = state(0.0, course_finished=True)
    out = router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
    assert out.mode is Mode.DONE
    assert out.source is Source.ROUTER
