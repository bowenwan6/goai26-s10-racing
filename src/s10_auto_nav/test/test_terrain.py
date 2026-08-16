"""The interesting cases are the ones where two sensors disagree, so most of these are that.

The numbers in ``test_the_waypoint_17_staircase_*`` are lifted from a full-stack run that
failed, not invented, which is the only reason to trust that the classifier is aimed at
the real problem rather than at a tidy version of it.
"""

from __future__ import annotations

import math

import pytest

from s10_auto_nav.terrain import (
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
WAYPOINT_17_APPROACH = [
    TerrainReading(obstacle_distance=2.49, relief_rise=0.06, lidar_clearance=2.49),
    TerrainReading(obstacle_distance=2.26, relief_rise=0.19, lidar_clearance=2.26),
    TerrainReading(obstacle_distance=2.22, relief_rise=0.31, lidar_clearance=2.22),
]


def test_the_waypoint_17_staircase_is_not_called_a_wall():
    """The whole point. A return with ground rising under it is the route, not an obstacle."""
    classifier = TerrainClassifier()
    kinds = [settle(classifier, reading, 0.5).kind for reading in WAYPOINT_17_APPROACH]
    assert kinds[-1] in {TerrainKind.STAIRS, TerrainKind.HIGH_BARRIER}
    assert TerrainKind.BLOCKED not in kinds, "steering around the staircase is what failed"


def test_the_waypoint_17_staircase_is_something_to_drive_at():
    classifier = TerrainClassifier()
    for reading in WAYPOINT_17_APPROACH:
        verdict = settle(classifier, reading, 0.5)
    assert verdict.drive_at_it


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


def test_ground_falling_away_is_a_drop():
    assert settle(TerrainClassifier(), TerrainReading(relief_drop=0.4)).kind is TerrainKind.DROP


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
    settle(classifier, TerrainReading(relief_rise=0.20, obstacle_distance=1.5))
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
        TerrainKind.STAIRS: TerrainReading(relief_rise=0.2, obstacle_distance=1.5),
        TerrainKind.BLOCKED: TerrainReading(obstacle_distance=1.0),
        TerrainKind.HIGH_BARRIER: TerrainReading(relief_rise=0.5),
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
