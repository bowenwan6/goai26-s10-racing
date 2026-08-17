"""Committing to a step rather than steering around it."""

import math

import pytest

from s10_auto_nav.pure_pursuit import Command
from s10_auto_nav.step_commit import StepCommit, StepCommitConfig

DT = 0.02

#: What the base actually reported while wedged on the drop between gates 5 and 6.
WEDGED_PITCH = math.radians(-17.0)

#: Gait pitch on the flat ran 1-3 degrees, so this must not read as a step.
WALKING_PITCH = math.radians(3.0)


#: Ground speed while genuinely wedged: the base logged 0.00-0.04 m/s for forty seconds.
WEDGED_SPEED = 0.02

#: Ground speed on the +8 degree approach to gate 1, which needs no help whatsoever.
SLOPE_SPEED = 0.83


def hold(commit, pitch, seconds, speed=WEDGED_SPEED):
    """Run the state machine at control rate; return whether it still claims a step."""
    climbing = False
    for _ in range(int(round(seconds / DT))):
        climbing = commit.update(pitch, speed, DT)
    return climbing


def test_gait_pitch_is_not_a_step():
    commit = StepCommit()
    assert not hold(commit, WALKING_PITCH, 5.0)


def test_a_descent_counts_as_a_step():
    """Nose-down 17 degrees is the wedge; tilt alone could not tell it from a climb."""
    commit = StepCommit()
    assert commit.update(WEDGED_PITCH, WEDGED_SPEED, DT)


def test_a_climb_counts_too():
    commit = StepCommit()
    assert commit.update(-WEDGED_PITCH, WEDGED_SPEED, DT)


def test_a_slope_taken_at_speed_is_not_a_step():
    """Pitch alone over-triggers, and the cost of that is paid over the whole lap.

    The approach to gate 1 holds a steady +8 degrees for four seconds while the robot
    crosses it at 0.83 m/s. An earlier version committed there, capping forward at
    ``speed`` and clamping steering for no reason -- on a course where 12 of 32 legs cross
    raised terrain. Being pitched is not the signal; being pitched and going nowhere is.
    """
    commit = StepCommit()
    assert not hold(commit, math.radians(8.0), 4.0, speed=SLOPE_SPEED)


def test_getting_moving_again_releases_the_commit():
    commit = StepCommit()
    assert hold(commit, WEDGED_PITCH, 2.0)
    assert not commit.update(WEDGED_PITCH, SLOPE_SPEED, DT)


def test_the_run_up_does_not_abort_itself():
    """Reversing at ``speed`` trivially passes the progress test, so it must be exempt.

    Otherwise the run-up ends one control step after it starts and the robot never gets
    far enough back to have a run at anything.
    """
    config = StepCommitConfig(timeout=8.0, backup=1.5, speed=0.5, progress_speed=0.35)
    commit = StepCommit(config)
    assert hold(commit, WEDGED_PITCH, 8.5)
    assert commit.backing
    # Now moving backwards at the commanded 0.5, comfortably past progress_speed.
    assert commit.update(WEDGED_PITCH, config.speed, DT)
    assert commit.backing


def test_levelling_out_releases_immediately():
    commit = StepCommit()
    hold(commit, WEDGED_PITCH, 2.0)
    assert not commit.update(WALKING_PITCH, WEDGED_SPEED, DT)
    assert commit.elapsed == 0.0


def test_push_drives_straight_and_floors_forward():
    commit = StepCommit()
    commit.update(WEDGED_PITCH, WEDGED_SPEED, DT)
    # The thrash that lost the second run had exactly these saturated inputs.
    out = commit.command(Command(forward=0.11, lateral=0.40, yaw_rate=0.70))
    assert out.forward == pytest.approx(commit.config.speed)
    assert out.lateral == 0.0
    assert out.yaw_rate == pytest.approx(commit.config.yaw_rate)


def test_push_floors_forward_even_where_pure_pursuit_would_pivot():
    """Pure pursuit commands zero forward past its pivot threshold. On an edge that is fatal.

    Turning in place is the one manoeuvre that cannot get a wheel over a lip, and the
    observed failure was precisely the robot rotating on the spot while y never advanced.
    """
    commit = StepCommit()
    commit.update(WEDGED_PITCH, WEDGED_SPEED, DT)
    out = commit.command(Command(forward=0.0, lateral=0.0, yaw_rate=0.70))
    assert out.forward == pytest.approx(commit.config.speed)


def test_yaw_is_clamped_both_ways():
    commit = StepCommit(StepCommitConfig(yaw_rate=0.15))
    commit.update(WEDGED_PITCH, WEDGED_SPEED, DT)
    assert commit.command(Command(yaw_rate=-0.70)).yaw_rate == pytest.approx(-0.15)
    assert commit.command(Command(yaw_rate=0.05)).yaw_rate == pytest.approx(0.05)


def test_the_attempt_becomes_a_straight_run_up_after_the_timeout():
    config = StepCommitConfig(timeout=8.0, backup=1.5, speed=0.5)
    commit = StepCommit(config)
    assert hold(commit, WEDGED_PITCH, 8.5)
    assert commit.backing
    out = commit.command(Command(forward=0.3, lateral=0.4, yaw_rate=0.7))
    assert out.forward == pytest.approx(-config.speed)
    assert out.lateral == 0.0
    assert out.yaw_rate == 0.0


def test_the_run_up_is_followed_by_another_attempt():
    """The cycle repeats; one failure does not hand a still-pitched robot back to steering.

    The version that expired into free steering after a single attempt spent the following
    minute with lateral and yaw alternating sign every second, scrabbling along the lip -- x
    drifting 0.4 m while y did not advance at all. Retrying is the part that works; what is
    bounded is how many times, and that is the next two tests.
    """
    commit = StepCommit(StepCommitConfig(timeout=8.0, backup=1.5))
    # A tick past the 9.5 s cycle, not exactly on it: the accumulator sums 0.02 at a time
    # and lands either side of the boundary on float dust. One control step of slop in when
    # the run-up ends is beneath notice; a test that pins it is testing arithmetic.
    assert hold(commit, WEDGED_PITCH, 9.6)
    assert not commit.backing, "should be pushing again, not still reversing"
    assert commit.command(Command()).forward > 0.0

    # And round again, for as long as the body stays pitched and attempts remain.
    assert hold(commit, WEDGED_PITCH, 8.5)
    assert commit.backing


def test_a_wall_is_conceded_rather_than_leant_on_forever():
    """The 0.544 m rise at (27.8, 29.5) on the leg to waypoint 19.

    Unbounded, the commit held 0.7 m/s against it for 130 s and then reared over at 70
    degrees of tilt, with a clear route 0.2 m to the left the entire time. Three complete
    cycles of 9.5 s is 28.5 s, so 30 s of unbroken wedging is comfortably past the limit and
    the state machine must have handed back by then.
    """
    commit = StepCommit(StepCommitConfig(timeout=8.0, backup=1.5, attempts=3))
    assert not hold(commit, WEDGED_PITCH, 30.0)
    assert commit.conceded
    assert commit.failures == 3


def test_conceding_stays_conceded_while_the_body_is_still_on_it():
    """Otherwise the follower alternates between committing and steering every control step.

    Handing back is only useful if it lasts long enough for the planner to pick a side and
    act on it; a state that re-commits on the next tick is the flip-flop with extra steps.
    """
    commit = StepCommit(StepCommitConfig(timeout=8.0, backup=1.5, attempts=3))
    hold(commit, WEDGED_PITCH, 30.0)
    assert not hold(commit, WEDGED_PITCH, 20.0)
    assert commit.failures == 3, "conceding must not keep counting attempts it is not making"


def test_getting_over_the_step_clears_the_tally():
    """Two hard steps in a row are two problems, not one problem with six attempts.

    The staircases on this course are eight and thirteen treads; a count that survived a
    successful crossing would concede partway up one of them.
    """
    commit = StepCommit(StepCommitConfig(timeout=8.0, backup=1.5, attempts=3))
    assert hold(commit, WEDGED_PITCH, 15.0)
    assert commit.failures == 1
    # Over it: still pitched, but now moving at the speed of a slope being driven up.
    assert not hold(commit, WEDGED_PITCH, 1.0, speed=SLOPE_SPEED)
    assert commit.failures == 0
    assert not commit.conceded
