"""The interesting cases are the ones where two sensors disagree, so most of these are that.

The numbers in ``test_the_waypoint_17_staircase_*`` are lifted from a full-stack run that
failed, not invented, which is the only reason to trust that the classifier is aimed at
the real problem rather than at a tidy version of it.
"""

from __future__ import annotations

import math
from dataclasses import replace

import pytest

from s10_auto_nav.terrain import (
    DRIVE_AT_IT,
    TerrainClassifier,
    TerrainConfig,
    TerrainKind,
    TerrainReading,
)

TICK = 0.02


def settle(classifier: TerrainClassifier, reading: TerrainReading, seconds: float = 2.0):
    """Hold one reading long enough that any dwell has elapsed."""
    verdict = classifier.verdict
    for _ in range(int(seconds / TICK)):
        verdict = classifier.update(reading, TICK)
    return verdict


FLAT = TerrainReading(lidar_clearance=8.0, obstacle_distance=8.0)


def test_flat_ground_is_flat():
    assert settle(TerrainClassifier(), FLAT).kind is TerrainKind.FLAT


def test_it_says_nothing_until_it_has_seen_something():
    """The opening verdict must not be FLAT, or the robot trusts an empty classifier."""
    assert TerrainClassifier().verdict.kind is TerrainKind.UNKNOWN


# --------------------------------------------------------------------------------------
# The failure this module was written for.
# --------------------------------------------------------------------------------------

#: Waypoint 17, approaching the staircase, taken from 17_18_seed0. The lidar return sat at
#: 2.2 m and never moved; the height map relief climbed 0.06 -> 0.19 -> 0.31 as the robot
#: closed on the first riser. The production follower steered around it for 100 s.
#:
#: These are readings from a run that *failed* here, and the 0.19 and 0.31 are now known to
#: be the artefact described on ``TerrainConfig.barrier_rise``: relief is a residual from a
#: plane fitted across the corridor, and it inflates once the body is up against a face
#: rather than approaching it. Replaying the run that scored this gate puts waypoint 17 on
#: flat ground -- relief 0.000 for the whole approach, scored at 13.7 s -- and puts the
#: staircase itself on the leg beyond it at 0.128. So what this fixture is good for is the
#: question it was written for, which is whether a return with ground rising under it gets
#: called a wall. It is no longer evidence about what to *do*; that is the next test, and it
#: uses the measured numbers.
WAYPOINT_17_APPROACH = [
    TerrainReading(obstacle_distance=2.49, relief_rise=0.06, lidar_clearance=2.49),
    TerrainReading(obstacle_distance=2.26, relief_rise=0.19, lidar_clearance=2.26),
    TerrainReading(obstacle_distance=2.22, relief_rise=0.31, lidar_clearance=2.22),
]

#: The same staircase, measured on the run that got up it: ``16_32__concede_seed0`` at
#: t=75.4, at (18.91, 29.76) on the leg from waypoint 17 to 18. This is what a climbable
#: full-width feature actually reads -- and note where the height goes, because it is the
#: whole distinction: over the next four seconds the relief *falls* 0.128 -> 0.05 while the
#: fitted slope *rises* 7 -> 17 degrees. An incline moves into the plane. A wall cannot,
#: which is why it stays behind in the residual.
WAYPOINT_17_STAIRCASE = TerrainReading(
    relief_rise=0.128,
    rise_fraction=0.6,
    slope_deg=7.1,
    obstacle_distance=2.2,
    lidar_clearance=2.2,
)


def test_the_waypoint_17_staircase_is_not_called_a_wall():
    """The whole point. A return with ground rising under it is the route, not an obstacle."""
    classifier = TerrainClassifier()
    kinds = [settle(classifier, reading, 0.5).kind for reading in WAYPOINT_17_APPROACH]
    assert kinds[-1] in {TerrainKind.STAIRS, TerrainKind.HIGH_BARRIER}
    assert TerrainKind.BLOCKED not in kinds, "steering around the staircase is what failed"


def test_the_waypoint_17_staircase_is_something_to_drive_at():
    assert settle(TerrainClassifier(), WAYPOINT_17_STAIRCASE).drive_at_it


def test_the_same_return_over_flat_ground_is_a_wall():
    """Identical lidar, no rise. This is the case where the detour is correct."""
    wall = TerrainReading(obstacle_distance=2.22, relief_rise=0.0, lidar_clearance=2.22)
    verdict = settle(TerrainClassifier(), wall)
    assert verdict.kind is TerrainKind.BLOCKED
    assert not verdict.drive_at_it


def test_a_wall_and_a_staircase_differ_only_in_the_height_map():
    """Stated as an equality so it cannot rot into two unrelated tests."""
    common = {"obstacle_distance": 2.2, "lidar_clearance": 2.2}
    wall = settle(TerrainClassifier(), TerrainReading(relief_rise=0.0, **common))
    stairs = settle(TerrainClassifier(), TerrainReading(relief_rise=0.31, **common))
    assert (wall.kind, stairs.kind) == (TerrainKind.BLOCKED, TerrainKind.HIGH_BARRIER)


# --------------------------------------------------------------------------------------
# The rest of the taxonomy.
# --------------------------------------------------------------------------------------


def test_a_uniform_slope_is_a_ramp_and_not_a_step():
    """A ramp's gradient is in the fitted plane, so its residual relief is near zero."""
    ramp = TerrainReading(slope_deg=11.0, relief_rise=0.02, obstacle_distance=6.0)
    assert settle(TerrainClassifier(), ramp).kind is TerrainKind.RAMP


def test_a_riser_past_the_wheels_asks_for_the_climb_policy():
    """The measured Gate 16 riser. More throttle does not get up this."""
    gate16 = TerrainReading(relief_rise=0.377, obstacle_distance=1.0)
    verdict = settle(TerrainClassifier(), gate16)
    assert verdict.kind is TerrainKind.HIGH_BARRIER


# --------------------------------------------------------------------------------------
# The third failure: a wall the plane fit hid.
# --------------------------------------------------------------------------------------

#: Replayed from ``continuous/16_32__concede_seed0`` at t=295.7 to 297.2, on the leg from
#: waypoint 23 to 24, 2.9 m short of a wall the deck map measures at 0.465 m. ``relief_rise``
#: is a residual from the plane fitted across the corridor, and a wall wide enough to tilt
#: that fit hides in it: the nine replayed samples report 0.181 to 0.296, never the 0.30 that
#: ``barrier_rise`` used to ask for. So it read STAIRS, avoidance was switched off, the speed
#: scale of 0.25 was suppressed, and the robot arrived at 0.78 m/s. It reared to 28 degrees at
#: t=297.8 -- which is the first tick HIGH_BARRIER was ever reached in the whole 300 s run --
#: and the run ended on its side at 70 degrees with nine gates unscored.
WAYPOINT_23_WALL = TerrainReading(
    relief_rise=0.211,
    rise_fraction=1.0,
    slope_deg=5.2,
    obstacle_distance=2.9,
    lidar_clearance=2.9,
    commanded_forward=0.67,
    speed=0.78,
)


def test_a_wall_the_plane_fit_flattened_is_not_a_staircase():
    verdict = settle(TerrainClassifier(), WAYPOINT_23_WALL)
    assert verdict.kind is TerrainKind.HIGH_BARRIER


def test_a_rise_past_the_wheels_is_not_something_to_drive_at():
    """The other half of the pair. Recognising the wall earlier is no use on its own.

    HIGH_BARRIER was in ``DRIVE_AT_IT``, so classifying this wall sooner would only have
    meant charging it sooner. What makes recognition worth anything is that it now turns the
    avoidance planner on -- and there is somewhere for it to go: 5.904 m round against
    5.431 m straight, at the body's full 0.45 m half-width.
    """
    assert not settle(TerrainClassifier(), WAYPOINT_23_WALL).drive_at_it
    assert TerrainKind.HIGH_BARRIER not in DRIVE_AT_IT


def test_the_course_steps_the_robot_already_clears_are_still_stairs():
    """The margin this costs, stated as a test because it is the thinnest one here.

    Replaying the whole waypoint 16 to 32 run gives three drivable full-width rises, in three
    separate places, reading 0.124, 0.125 and 0.128 -- against the wall's 0.181. 0.16 sits in
    that gap. The four 0.125 m steps between gates 5 and 6 are the tightest case and are not
    in that replay, so they are pinned here instead. A continuous waypoint 0 to 16 run at
    this threshold has since taken gate 6 along with the rest of gates 1 to 15.
    """
    for rise in (0.124, 0.125, 0.128):
        step = TerrainReading(relief_rise=rise, rise_fraction=1.0, obstacle_distance=2.2)
        verdict = settle(TerrainClassifier(), step)
        assert verdict.kind is TerrainKind.STAIRS, f"rise {rise} must still be climbable"
        assert verdict.drive_at_it


#: The step at x=27.05 that is the only way from waypoint 18 to 19, replayed from
#: ``continuous/17_32_barrier2_seed0``. It is 0.16 m as geometry -- the deck goes 2.20 to 2.36
#: -- and the plane-fit residual reports it as 0.26, which is *above* what the same residual
#: reports for the 0.465 m wall before waypoint 24. On that number alone the required step
#: outranks the impassable wall, which is why the number alone is not enough.
#:
#: What tells them apart is the scan ring, and it is not a close call: 2609 samples of one
#: run pinned in front of this step have a median ``scan_ahead`` of 99 m, the recorder's
#: no-return sentinel. The lidar is at body height and a 0.16 m step passes under it.
WAYPOINT_18_STEP = TerrainReading(
    relief_rise=0.26,
    rise_fraction=1.0,
    slope_deg=4.4,
    obstacle_distance=math.inf,
    lidar_clearance=math.inf,
)


def test_the_step_onto_the_waypoint_19_deck_is_not_called_a_wall():
    """The regression that cost two runs, pinned by the reading that caused it.

    Called HIGH_BARRIER this step is throttled to 0.24 m/s, and the robot cannot get up it at
    0.24 m/s: one run held that command in a 0.25 m box for 1100 s, the next was deflected
    onto the 0.5 m block beside the lane and went over at 72 degrees.
    """
    verdict = settle(TerrainClassifier(), WAYPOINT_18_STEP)
    assert verdict.kind is not TerrainKind.HIGH_BARRIER
    assert verdict.drive_at_it, "the step is the route; it must keep its speed"


def test_the_wall_and_the_required_step_are_inseparable_by_rise_alone():
    """Stated as an inequality so no future tuning of ``barrier_rise`` can look right.

    The step the robot must climb reads *higher* than the wall it must not. Any threshold on
    this quantity that stops the wall also stops the step.
    """
    assert WAYPOINT_18_STEP.relief_rise > WAYPOINT_23_WALL.relief_rise
    assert settle(TerrainClassifier(), WAYPOINT_23_WALL).kind is TerrainKind.HIGH_BARRIER
    assert settle(TerrainClassifier(), WAYPOINT_18_STEP).kind is not TerrainKind.HIGH_BARRIER


def test_a_wall_the_lidar_cannot_confirm_is_not_yet_a_wall():
    """The second witness has to be load-bearing, or the pair above is decoration."""
    unseen = replace(WAYPOINT_23_WALL, obstacle_distance=math.inf, lidar_clearance=math.inf)
    assert settle(TerrainClassifier(), unseen).kind is not TerrainKind.HIGH_BARRIER


# --------------------------------------------------------------------------------------
# The second failure: a rise that was not the route.
# --------------------------------------------------------------------------------------

#: Replayed from ``continuous/16_32__fixed_seed0`` at t=259.2 s, on the leg from waypoint 24
#: to waypoint 25. Geom g1870 is a pillar 1.94 m tall whose face sits 0.28 m from waypoint 24,
#: and the approach from waypoint 23 points straight at it. The height map showed the wheel
#: corridor flat and two of its nine columns standing 0.32 m and 0.80 m up, which
#: ``terrain_relief`` reduced to ``rise 0.29m`` -- the same number a staircase gives. It was
#: called STAIRS, so avoidance was switched off and the speed scale forced to 1.0, and the
#: robot drove into it: pitch -13.9, -40.1, -59.5 degrees over the next second, then the run
#: ended at 70 degrees of tilt with sixteen gates unscored.
WAYPOINT_24_PILLAR = TerrainReading(
    relief_rise=0.29,
    rise_fraction=0.20,
    obstacle_distance=0.17,
    lidar_clearance=0.17,
    commanded_forward=0.25,
    speed=0.24,
)


def test_a_rise_that_only_clips_the_edge_of_the_patch_is_not_a_staircase():
    verdict = settle(TerrainClassifier(), WAYPOINT_24_PILLAR)
    assert verdict.kind is TerrainKind.BLOCKED
    assert not verdict.drive_at_it, "this is the flag that turned the avoidance planner off"


def test_the_same_rise_across_the_path_is_not_something_to_steer_around():
    """The guard is the width of the rise, not the height of it.

    The pillar's 0.29 m across the full width is past ``barrier_rise``, so this is a
    HIGH_BARRIER rather than STAIRS -- but the distinction being tested is the other one:
    width decides whether a feature is the route or something standing beside it, and only
    the thing beside it is BLOCKED. Climbable heights are the next test.
    """
    spanning = replace(WAYPOINT_24_PILLAR, rise_fraction=1.0, obstacle_distance=1.0)
    assert settle(TerrainClassifier(), spanning).kind is TerrainKind.HIGH_BARRIER


def test_a_climbable_rise_across_the_path_is_still_a_staircase():
    """Climbing must still work: 0.14 is inside the band the scored legs actually used."""
    stairs = replace(
        WAYPOINT_24_PILLAR, relief_rise=0.14, rise_fraction=1.0, obstacle_distance=1.0
    )
    verdict = settle(TerrainClassifier(), stairs)
    assert verdict.kind is TerrainKind.STAIRS
    assert verdict.drive_at_it


def test_a_barrier_that_does_not_span_the_path_is_not_handed_to_the_climb_policy():
    """A 0.5 m rise in one column is a bollard. Rearing up at it accomplishes nothing."""
    bollard = replace(WAYPOINT_24_PILLAR, relief_rise=0.5, rise_fraction=0.2)
    assert settle(TerrainClassifier(), bollard).kind is TerrainKind.BLOCKED


def test_the_far_approach_to_a_staircase_is_not_called_blocked():
    """The regression the first version of the width test caused, and it cost a whole run.

    At 2.2 m from the bottom of the waypoint 17 staircase only the near corner of the first
    tread is inside the corridor, so the rise is both small -- 0.06 m, well under a step --
    and one-sided. Calling that BLOCKED turns the avoidance planner loose on the route
    itself: the robot at (18.8, 29.6) steered away from the staircase, never squared up to
    it, and was still oscillating there a minute later, too fast for the stall recovery to
    fire and too slowly for the wedge detector. The width test only gets to speak about
    rises that are large enough to be steps in the first place.
    """
    approach = TerrainReading(
        relief_rise=0.06, rise_fraction=0.2, obstacle_distance=2.2, lidar_clearance=2.2
    )
    assert settle(TerrainClassifier(), approach).kind is not TerrainKind.BLOCKED


def test_the_reason_says_which_of_the_two_it_was():
    """The verdict is what the log carries; a BLOCKED with no width in it is undiagnosable."""
    assert "20% of the width" in settle(TerrainClassifier(), WAYPOINT_24_PILLAR).reason


def test_ground_falling_away_is_a_drop():
    assert settle(TerrainClassifier(), TerrainReading(relief_drop=0.4)).kind is TerrainKind.DROP


#: Reconstructed from ``continuous/16_32_seed0``, which stood at (27.5, 29.6) on the leg from
#: waypoint 18 to waypoint 19 for the entire 900 s budget. The log gives the fall directly and
#: the rise through the speed scale it produced: ``terrain=0.35`` inverts through
#: ``ground_clearance`` to 0.26 m of rise against ``max_step`` 0.35.
WAYPOINT_18_LEDGE = TerrainReading(
    relief_rise=0.26, relief_drop=0.21, obstacle_distance=1.0, commanded_forward=0.24
)


def test_an_edge_that_rises_more_than_it_falls_is_a_step_and_not_a_drop():
    """The plane is fitted across the corridor, so a step up reads as a fall as well.

    This is what pinned the run: a fall of 0.21 m in front of a rise of 0.26 m was called
    DROP, which is not a kind the follower drives at, so it braked and steered at ground it
    should have driven straight over.

    The rise wins, which is the assertion. Which *kind* of rise it is has since moved: 0.26
    is past ``barrier_rise``, so this reads HIGH_BARRIER rather than STAIRS. That is not a
    weakening of this test -- the fall losing to the rise is the whole subject -- and on this
    particular leg it is arguably the better answer, the deck having been clear 0.2 m to the
    left for its whole length.
    """
    verdict = settle(TerrainClassifier(), WAYPOINT_18_LEDGE)
    assert verdict.kind is not TerrainKind.DROP
    assert verdict.kind is TerrainKind.HIGH_BARRIER


def test_the_fall_still_wins_when_it_is_the_larger_feature():
    """The guard is which of the two is bigger, not that falls stopped counting."""
    edge = TerrainReading(relief_rise=0.10, relief_drop=0.40, obstacle_distance=1.0)
    assert settle(TerrainClassifier(), edge).kind is TerrainKind.DROP


def test_a_drop_held_from_before_does_not_survive_the_ground_starting_to_rise():
    """Hysteresis is what made this permanent: staying in DROP needs only 0.16 m.

    So the approach is replayed rather than asserted from a standing start -- a fall first,
    which is what the robot saw coming over the lip, and then the ledge.
    """
    classifier = TerrainClassifier()
    assert settle(classifier, TerrainReading(relief_drop=0.45), 1.0).kind is TerrainKind.DROP
    assert settle(classifier, WAYPOINT_18_LEDGE).kind is not TerrainKind.DROP


def test_a_hazard_is_believed_on_the_first_tick():
    """DROP and UNSTABLE have no dwell: believing them late is the expensive direction."""
    classifier = TerrainClassifier()
    settle(classifier, FLAT)
    assert classifier.update(TerrainReading(relief_drop=0.4), TICK).kind is TerrainKind.DROP


def test_lying_on_its_side_outranks_whatever_is_ahead():
    fallen = TerrainReading(roll_deg=60.0, relief_rise=0.31, obstacle_distance=1.0)
    assert settle(TerrainClassifier(), fallen).kind is TerrainKind.UNSTABLE


def test_being_asked_to_move_and_not_moving_is_eventually_unstable():
    """Seed 1 of 17_18 sat at exactly one position for 100 s while commanding +0.22."""
    wedged = TerrainReading(commanded_forward=0.22, speed=0.0, relief_rise=0.31)
    classifier = TerrainClassifier()
    assert classifier.update(wedged, TICK).kind is not TerrainKind.UNSTABLE
    assert settle(classifier, wedged, 6.0).kind is TerrainKind.UNSTABLE


def test_standing_still_on_purpose_is_not_wedged():
    """Otherwise every deliberate stop decays into a recovery."""
    holding = TerrainReading(commanded_forward=0.0, speed=0.0)
    assert settle(TerrainClassifier(), holding, 10.0).kind is not TerrainKind.UNSTABLE


# --------------------------------------------------------------------------------------
# Freshness, dwell, hysteresis.
# --------------------------------------------------------------------------------------


def test_stale_sensors_produce_unknown_however_good_the_numbers_look():
    stale = TerrainReading(sensor_age=2.0, obstacle_distance=8.0, lidar_clearance=8.0)
    verdict = settle(TerrainClassifier(), stale)
    assert verdict.kind is TerrainKind.UNKNOWN
    assert verdict.confidence == 0.0
    assert "old" in verdict.reason


def test_going_stale_takes_a_settled_verdict_away():
    classifier = TerrainClassifier()
    assert settle(classifier, FLAT).kind is TerrainKind.FLAT
    stale = TerrainReading(sensor_age=2.0, obstacle_distance=8.0, lidar_clearance=8.0)
    assert settle(classifier, stale).kind is TerrainKind.UNKNOWN


def test_a_single_odd_tick_does_not_move_the_verdict():
    """One noisy height map cell must not flip the robot between driving and detouring."""
    classifier = TerrainClassifier()
    settle(classifier, FLAT)
    verdict = classifier.update(TerrainReading(obstacle_distance=1.0, relief_rise=0.0), TICK)
    assert verdict.kind is TerrainKind.FLAT, "committed verdict should not have moved"
    assert verdict.candidate is TerrainKind.BLOCKED, "but the evidence should be visible"


def test_the_candidate_is_reported_while_a_change_is_being_debounced():
    classifier = TerrainClassifier()
    settle(classifier, FLAT)
    wall = TerrainReading(obstacle_distance=1.0, relief_rise=0.0)
    seen = {classifier.update(wall, TICK).candidate for _ in range(5)}
    assert seen == {TerrainKind.BLOCKED}


def test_hysteresis_holds_stairs_through_the_dip_between_two_risers():
    """The rise falls under the entry threshold between steps; letting go there is the bug."""
    classifier = TerrainClassifier()
    settle(classifier, TerrainReading(relief_rise=0.14, obstacle_distance=1.5))
    assert classifier.verdict.kind is TerrainKind.STAIRS

    between = TerrainReading(relief_rise=0.10, obstacle_distance=1.5)
    assert settle(classifier, between).kind is TerrainKind.STAIRS

    # Genuinely flat ground still ends it, or the hysteresis would be a latch.
    assert settle(classifier, FLAT).kind is TerrainKind.FLAT


def test_prior_knowledge_lowers_the_bar_but_does_not_clear_it():
    """A course that says "stairs here" may not manufacture stairs on empty ground."""
    marginal = TerrainReading(relief_rise=0.11, obstacle_distance=1.5)
    assert settle(TerrainClassifier(), marginal).kind is not TerrainKind.STAIRS

    told = TerrainReading(relief_rise=0.11, obstacle_distance=1.5, expected=TerrainKind.STAIRS)
    assert settle(TerrainClassifier(), told).kind is TerrainKind.STAIRS

    # The prior alone, over ground with nothing on it, still gets nothing.
    empty = TerrainReading(obstacle_distance=8.0, lidar_clearance=8.0, expected=TerrainKind.STAIRS)
    assert settle(TerrainClassifier(), empty).kind is TerrainKind.FLAT


def test_confidence_grows_with_the_margin_over_the_threshold():
    barely = settle(TerrainClassifier(), TerrainReading(relief_rise=0.13, obstacle_distance=1.5))
    clearly = settle(TerrainClassifier(), TerrainReading(relief_rise=0.28, obstacle_distance=1.5))
    assert 0.0 < barely.confidence < clearly.confidence <= 1.0


def test_every_kind_is_reachable():
    """A taxonomy with an unreachable member is a taxonomy with a bug in it."""
    cases = {
        TerrainKind.FLAT: FLAT,
        TerrainKind.RAMP: TerrainReading(slope_deg=12.0),
        TerrainKind.STAIRS: TerrainReading(relief_rise=0.14, obstacle_distance=1.5),
        TerrainKind.BLOCKED: TerrainReading(obstacle_distance=1.0),
        # A return as well as a rise: a wall the scan ring cannot see is a step.
        TerrainKind.HIGH_BARRIER: TerrainReading(relief_rise=0.5, obstacle_distance=3.2),
        TerrainKind.DROP: TerrainReading(relief_drop=0.5),
        TerrainKind.UNSTABLE: TerrainReading(roll_deg=70.0),
        TerrainKind.UNKNOWN: TerrainReading(sensor_age=9.0),
    }
    assert {kind: settle(TerrainClassifier(), r).kind for kind, r in cases.items()} == {
        kind: kind for kind in cases
    }


def test_reset_forgets_the_history_and_keeps_the_configuration():
    config = TerrainConfig(step_rise=0.99)
    classifier = TerrainClassifier(config)
    settle(classifier, FLAT)
    classifier.reset()
    assert classifier.verdict.kind is TerrainKind.UNKNOWN
    assert classifier.config is config


def test_tilt_combines_pitch_and_roll():
    reading = TerrainReading(pitch_deg=30.0, roll_deg=30.0)
    expected = math.degrees(math.acos(math.cos(math.radians(30.0)) ** 2))
    assert reading.tilt_deg == pytest.approx(expected)


def test_the_verdict_prints_something_a_log_line_can_use():
    text = str(settle(TerrainClassifier(), TerrainReading(relief_rise=0.31, obstacle_distance=2.2)))
    assert "high_barrier" in text and "0.31" in text
