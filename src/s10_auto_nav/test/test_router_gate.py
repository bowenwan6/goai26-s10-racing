"""The entry gate, and the two ways it was blind.

``test_router.py`` covers the state machine. This file covers one thing only: the condition
under which ``ALIGN`` is allowed to hand the robot to a climb policy, and it exists because
an audit found that condition weaker than it read.

Two holes, both of which let the gate pass on a robot that was not in an entry state:

* **No distance band.** ``_align`` tested lateral error, heading error, speed and tilt, and
  never asked where the edge was. Anywhere inside the align radius counted, so a robot 1.2 m
  short of the step and a robot already half on top of it were equally "ready".
* **No yaw-rate limit.** ``max_entry_speed`` is a ground-speed limit, and a pivot in place has
  no ground speed -- so a robot swinging through its target heading passed the heading check
  on the way past and the speed check throughout.

The numbers pinned down here are of two kinds and they must not be confused. The pre-existing
defaults are asserted because the acceptance tests, ``strategy.yaml`` and phase 2's entry-cost
measurements are all written against them, and a silent edit to one invalidates results that
are already in the report. ``ready_distance_min``, ``ready_distance_max`` and
``max_entry_yaw_rate`` are asserted because they are *provisional* -- nothing has yet measured
the entry distance a climb needs -- and a provisional number nobody can find later becomes a
permanent one by accident.

Shadow mode is here for the same reason the gate is: the pit-failure experiment's
natural-arrival arm is a measurement of the production stack only if the router watching it
cannot touch it.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from s10_auto_nav.strategy.mock_policy import MockClimbPolicy, MockScenario
from s10_auto_nav.strategy.router import (
    Mode,
    RobotState,
    Router,
    RouterConfig,
    Source,
    observation_from_state,
)

SEGMENT = (15, 16)
DT = 0.02
NAV = (0.6, 0.0, 0.0)

#: The middle of the provisional band, where a run meant to reach the gate is held: far
#: enough from either end that a failure here is about the gate and not about rounding.
IN_BAND = 0.60
#: Long enough for the gate to open (0.40 s of dwell after two ticks of arming) and short
#: enough that the mock policy has not yet finished its 2 s and moved the router on.
OPENS = 1.5
#: Long enough that a gate which was going to open has, and far short of ``align_timeout``.
REFUSES = 2.0


def state(t: float, **kw) -> RobotState:
    """A robot in a perfect entry state, unless the test says otherwise."""
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
        obstacle_distance=IN_BAND,
        lateral_error=0.0,
        heading_error=0.0,
        travelled=0.0,
    )
    base.update(kw)
    return RobotState(**base)


def make_router(config: RouterConfig | None = None) -> tuple[Router, MockClimbPolicy]:
    policy = MockClimbPolicy(MockScenario.SUCCEED)
    router = Router(
        config or RouterConfig(),
        policies={"climb_policy": policy},
        segment_policies={SEGMENT: "climb_policy"},
    )
    return router, policy


def hold(router: Router, *, seconds: float, t0: float = 0.0, **kw) -> list:
    """Feed one unchanging state for ``seconds``; return every output."""
    outs = []
    t = t0
    for _ in range(round(seconds / DT)):
        s = state(t, **kw)
        outs.append(router.tick(s, NAV, observation_from_state(s)))
        t += DT
    return outs


def align_dwell(router: Router, **kw) -> float:
    """Seconds spent in ALIGN before the gate opened; ``inf`` if it never did."""
    entered = None
    t = 0.0
    for _ in range(round(RouterConfig.align_timeout / DT)):
        s = state(t, **kw)
        router.tick(s, NAV, observation_from_state(s))
        if entered is None and router.mode is Mode.ALIGN:
            entered = t
        if router.mode in (Mode.CLIMB_READY, Mode.CLIMB):
            return t - entered
        t += DT
    return math.inf


# --------------------------------------------------------------- the distance band


@pytest.mark.parametrize("distance", [1.1, 0.9, 0.75, 0.40, 0.3, 0.1])
def test_the_gate_refuses_outside_the_distance_band(distance):
    """Perfect in every other respect, and refused anyway, because of where the robot is.

    0.9 m is inside the align radius and was accepted before the band existed; 0.3 m is a
    robot whose wheels are already against the riser. Both read as "aligned" and neither was
    in an entry state.
    """
    router, policy = make_router()
    hold(router, seconds=REFUSES, obstacle_distance=distance)
    assert router.mode is Mode.ALIGN
    assert policy.start_count == 0


def test_the_gate_opens_inside_the_band():
    """The positive control. Without it every refusal above passes on a gate welded shut."""
    router, policy = make_router()
    hold(router, seconds=OPENS)
    assert router.mode is Mode.CLIMB
    assert policy.start_count == 1


def test_measurement_only_holds_at_ready_without_starting_policy():
    router, policy = make_router(RouterConfig(measurement_only=True))
    outputs = hold(router, seconds=OPENS + 1.0)
    assert router.mode is Mode.CLIMB_READY
    assert policy.start_count == 0
    assert outputs[-1].source is Source.ROUTER
    assert outputs[-1].command == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("distance", [0.45, 0.70])
def test_the_band_is_inclusive_at_both_ends(distance):
    router, policy = make_router()
    hold(router, seconds=OPENS, obstacle_distance=distance)
    assert policy.start_count == 1


# --------------------------------------------------------------- the yaw-rate limit


def test_the_gate_refuses_on_yaw_rate_alone():
    """On the line, at the right distance, pointing the right way -- and turning.

    Ground speed is zero throughout, which is exactly why ``max_entry_speed`` never saw this.
    """
    router, policy = make_router()
    hold(router, seconds=REFUSES, yaw_rate=0.2)
    assert router.mode is Mode.ALIGN
    assert policy.start_count == 0


def test_the_sign_of_the_yaw_rate_does_not_matter():
    router, policy = make_router()
    hold(router, seconds=REFUSES, yaw_rate=-0.2)
    assert policy.start_count == 0


def test_a_yaw_rate_at_the_limit_is_still_allowed():
    router, policy = make_router()
    hold(router, seconds=OPENS, yaw_rate=RouterConfig.max_entry_yaw_rate)
    assert policy.start_count == 1


# --------------------------------------------------------------- the dwell


def test_the_gate_requires_the_whole_dwell():
    """0.40 s of holding it, not one lucky tick. Measured, not read back off the config."""
    router, _ = make_router()
    dwell = align_dwell(router)
    assert RouterConfig.ready_dwell <= dwell < RouterConfig.ready_dwell + 2 * DT


def test_one_bad_tick_restarts_the_dwell():
    """The 0.40 s has to be continuous.

    Sixteen good ticks, one tick of yaw rate, sixteen good ticks is 0.64 s of good ticks in
    total and must still be refused. Otherwise the dwell is a counter rather than a claim
    that the entry state persisted.
    """
    router, policy = make_router()
    almost = RouterConfig.ready_dwell - 4 * DT

    hold(router, seconds=0.5, yaw_rate=0.5)  # arms and enters ALIGN, banking nothing
    assert router.mode is Mode.ALIGN
    hold(router, seconds=almost, t0=1.0)
    hold(router, seconds=DT, t0=2.0, yaw_rate=0.5)
    hold(router, seconds=almost, t0=3.0)
    assert router.mode is Mode.ALIGN
    assert policy.start_count == 0

    hold(router, seconds=4 * DT, t0=4.0)
    assert router.mode in (Mode.CLIMB_READY, Mode.CLIMB)


# --------------------------------------------------------------- shadow mode


def test_shadow_mode_forwards_the_follower_command_untouched():
    """Every tick, in every mode, unscaled. This is the natural-arrival arm's whole validity.

    ``speed_scale`` is checked as well as the command: APPROACH caps the follower at half
    speed, and an observer that slows the approach down is not observing the run it claims
    to be.
    """
    router, _ = make_router(RouterConfig(shadow=True))
    outs = hold(router, seconds=OPENS)
    assert all(out.source is Source.NAV for out in outs)
    assert all(out.command == NAV for out in outs)
    assert all(out.speed_scale == 1.0 for out in outs)
    assert all(out.joints is None for out in outs)


def test_shadow_mode_is_not_the_router_switched_off():
    """It still arms, still aligns, still opens the gate. It just does not drive.

    The comparison is against an identical router with shadow off, fed identical states,
    because the claim is precisely that the two decide the same thing and differ only in who
    was holding the wheel while they did.
    """
    live, live_policy = make_router()
    shadow, shadow_policy = make_router(RouterConfig(shadow=True))
    live_out = hold(live, seconds=OPENS)
    shadow_out = hold(shadow, seconds=OPENS)

    assert [o.mode for o in shadow_out] == [o.mode for o in live_out]
    assert shadow.history == live.history
    assert shadow_policy.start_count == live_policy.start_count == 1
    assert any(o.source is not Source.NAV for o in live_out)
    assert any(o.command != NAV for o in live_out)
    # What the router would have done survives in the reason, and therefore in the status
    # topic and in the recorder's trace. Nothing is lost by not acting on it.
    assert all("shadow" in o.reason for o in shadow_out)
    assert any("shadow (router)" in o.reason for o in shadow_out)


# --------------------------------------------------------------- the defaults

#: Everything ``RouterConfig`` promised before the entry gate was touched. Asserted by value
#: rather than against the class, so that changing a default is a two-line edit somebody has
#: to mean.
PRE_EXISTING_DEFAULTS = {
    "control_rate": 50.0,
    "approach_enter": 3.0,
    "approach_exit": 3.6,
    "approach_behind": 0.6,
    "align_enter": 1.2,
    "align_exit": 1.5,
    "max_lateral_error": 0.08,
    "max_heading_error": math.radians(6.0),
    "max_entry_speed": 0.05,
    "max_entry_tilt": math.radians(12.0),
    "ready_dwell": 0.40,
    "unready_dwell": 0.15,
    "align_speed": 0.25,
    "align_yaw_rate": 0.35,
    "align_lateral": 0.20,
    "align_timeout": 12.0,
    "approach_speed_scale": 0.5,
    "climb_timeout": 20.0,
    "climb_progress_epsilon": 0.05,
    "climb_progress_window": 5.0,
    "verify_hold": 2.0,
    "verify_min_contacts": 3,
    "verify_clearance": 0.0,
    "resume_distance": 0.5,
    "resume_timeout": 15.0,
    "recover_speed": 0.35,
    "recover_duration": 2.0,
    "max_retries": 3,
    "sensor_timeout": 0.5,
    "sensor_abort_timeout": 3.0,
    "fall_tilt": math.radians(60.0),
    "torque_ceiling": 50.0,
    "torque_grace": 1.0,
}

#: Added for the pit-failure experiment and provisional until it reports. Pinned so the
#: numbers a result was collected under can be recovered from the repository alone.
PROVISIONAL_DEFAULTS = {
    "shadow": False,
    "ready_distance_min": 0.45,
    "ready_distance_max": 0.70,
    "max_entry_yaw_rate": 0.10,
}


@pytest.mark.parametrize(("name", "expected"), sorted(PRE_EXISTING_DEFAULTS.items()))
def test_pre_existing_defaults_are_unchanged(name, expected):
    assert getattr(RouterConfig(), name) == expected


@pytest.mark.parametrize(("name", "expected"), sorted(PROVISIONAL_DEFAULTS.items()))
def test_provisional_defaults_are_what_the_experiment_ran_under(name, expected):
    assert getattr(RouterConfig(), name) == expected


def test_the_band_sits_inside_the_align_radius():
    """A band the router can never be in would refuse every climb for the wrong reason."""
    config = RouterConfig()
    assert config.ready_distance_min < config.ready_distance_max <= config.align_enter
