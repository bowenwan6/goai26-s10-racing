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

#: Ground speed on the +8 degree approach to gate 1, which needs no help whatsoever.
SLOPE_SPEED = 0.83

#: Half the width of the band the base swung through while wedged on the 0.16 m step to
#: waypoint 19: 0.13 m in x over 53 s, for +0.01 m of net progress. Modelled as a sway
#: rather than as stillness because standing still is precisely what it did not do, and
#: that is what defeated the speed test this class used to run.
WEDGE_SWAY = 0.065


class Body:
    """A position the commit can measure, driven at a velocity with an optional sway.

    The commit judges progress by displacement, so the tests have to move something. A
    plain velocity is not enough to reproduce the failure that motivated the change: the
    wedge oscillated, which is why sampling instantaneous speed said "moving" ninety-four
    times while the body stayed put.
    """

    def __init__(self) -> None:
        self.centre = 0.0
        self.t = 0.0

    @property
    def xy(self) -> tuple[float, float]:
        return (self.centre + WEDGE_SWAY * math.sin(2 * math.pi * self.t / 1.5), 0.0)

    def run(self, commit, pitch, seconds, velocity=0.0, sway=True) -> bool:
        """Drive for ``seconds``; return whether the commit still claims a step."""
        climbing = False
        for _ in range(int(round(seconds / DT))):
            self.t += DT if sway else 0.0
            self.centre += velocity * DT
            climbing = commit.update(pitch, self.xy, DT)
        return climbing


def hold(commit, pitch, seconds, velocity=0.0):
    """Wedge in place for ``seconds`` unless a velocity is given."""
    return Body().run(commit, pitch, seconds, velocity=velocity)


def test_gait_pitch_is_not_a_step():
    commit = StepCommit()
    assert not hold(commit, WALKING_PITCH, 5.0)


def test_a_descent_counts_as_a_step():
    """Nose-down 17 degrees is the wedge; tilt alone could not tell it from a climb."""
    commit = StepCommit()
    assert commit.update(WEDGED_PITCH, (0.0, 0.0), DT)


def test_a_climb_counts_too():
    commit = StepCommit()
    assert commit.update(-WEDGED_PITCH, (0.0, 0.0), DT)


def test_a_slope_taken_at_speed_is_not_a_step():
    """Pitch alone over-triggers, and the cost of that is paid over the whole lap.

    The approach to gate 1 holds a steady +8 degrees for four seconds while the robot
    crosses it at 0.83 m/s. An earlier version committed there, capping forward at
    ``speed`` and clamping steering for no reason -- on a course where 12 of 32 legs cross
    raised terrain. Being pitched is not the signal; being pitched and going nowhere is.
    """
    commit = StepCommit()
    assert not hold(commit, math.radians(8.0), 4.0, velocity=SLOPE_SPEED)


def test_getting_moving_again_releases_the_commit():
    commit = StepCommit()
    body = Body()
    assert body.run(commit, WEDGED_PITCH, 2.0)
    assert not body.run(commit, WEDGED_PITCH, 1.0, velocity=SLOPE_SPEED)


def test_a_shuffle_on_the_spot_is_not_getting_moving():
    """The measured failure, and the reason progress is a distance and not a speed.

    On the 0.16 m step to waypoint 19 the base was pitched over 8 degrees for 1004 of
    1060 samples across 53 s, and moved +0.01 m. Ninety-four of those samples touched
    0.35 m/s as the gait swung it through a 0.13 m band, and under the old speed test each
    one zeroed both the push clock and the failure tally: the longest unbroken push the
    12 s timeout ever saw was 3.9 s, so the run-up never fired once. The run-up is how
    the baseline got up this step -- it reached the lip at 1.16 m/s off the flat.
    """
    commit = StepCommit()
    body = Body()
    assert not body.run(commit, WEDGED_PITCH, 53.0), "must have conceded by now"
    assert abs(body.xy[0]) < commit.config.progress_distance, "the body must not have left"
    assert commit.conceded
    assert commit.failures == 3, "53 s of going nowhere has to spend every attempt"


def test_the_run_up_does_not_abort_itself():
    """Reversing covers ``progress_distance`` at once, so the run-up must be exempt.

    Otherwise it ends within a control step or two of starting and the robot never gets
    far enough back to have a run at anything.
    """
    config = StepCommitConfig(timeout=8.0, backup=1.5, speed=0.5)
    commit = StepCommit(config)
    body = Body()
    assert body.run(commit, WEDGED_PITCH, 8.5)
    assert commit.backing
    # Now reversing at the commanded 0.5, which covers 0.25 m in half a second.
    assert body.run(commit, WEDGED_PITCH, 1.0, velocity=-config.speed)
    assert commit.backing


def test_levelling_out_releases_once_it_is_sustained():
    """A single level sample is gait noise; a second of them is the robot off the ledge.

    On the waypoint 19 wedge the pitch dipped under the threshold seven times in 53 s,
    median 0.40 s, none of them a departure -- and each dip used to zero the 12 s clock.
    """
    commit = StepCommit()
    body = Body()
    assert body.run(commit, WEDGED_PITCH, 2.0)
    assert commit.update(WALKING_PITCH, body.xy, DT), "one level tick is not a departure"
    assert body.run(commit, WALKING_PITCH, commit.config.level_dwell + 0.1) is False
    assert commit.elapsed == 0.0


def test_push_drives_straight_and_floors_forward():
    commit = StepCommit()
    commit.update(WEDGED_PITCH, (0.0, 0.0), DT)
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
    commit.update(WEDGED_PITCH, (0.0, 0.0), DT)
    out = commit.command(Command(forward=0.0, lateral=0.0, yaw_rate=0.70))
    assert out.forward == pytest.approx(commit.config.speed)


def test_yaw_is_clamped_both_ways():
    commit = StepCommit(StepCommitConfig(yaw_rate=0.15))
    commit.update(WEDGED_PITCH, (0.0, 0.0), DT)
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
    body = Body()
    # A tick past the 9.5 s cycle, not exactly on it: the accumulator sums 0.02 at a time
    # and lands either side of the boundary on float dust. One control step of slop in when
    # the run-up ends is beneath notice; a test that pins it is testing arithmetic.
    assert body.run(commit, WEDGED_PITCH, 9.6)
    assert not commit.backing, "should be pushing again, not still reversing"
    assert commit.command(Command()).forward > 0.0

    # And round again, for as long as the body stays pitched and attempts remain.
    assert body.run(commit, WEDGED_PITCH, 8.5)
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
    body = Body()
    body.run(commit, WEDGED_PITCH, 30.0)
    assert not body.run(commit, WEDGED_PITCH, 20.0)
    assert commit.failures == 3, "conceding must not keep counting attempts it is not making"


def test_getting_over_the_step_clears_the_tally():
    """Two hard steps in a row are two problems, not one problem with six attempts.

    The staircases on this course are eight and thirteen treads; a count that survived a
    successful crossing would concede partway up one of them.
    """
    commit = StepCommit(StepCommitConfig(timeout=8.0, backup=1.5, attempts=3))
    body = Body()
    assert body.run(commit, WEDGED_PITCH, 15.0)
    assert commit.failures == 1
    # Over it: still pitched, but now covering ground like a slope being driven up.
    assert not body.run(commit, WEDGED_PITCH, 1.0, velocity=SLOPE_SPEED)
    assert commit.failures == 0
    assert not commit.conceded


def test_the_run_up_does_not_cancel_the_failure_it_just_earned():
    """The reverse leg moves the body metres, which must not read as progress afterwards.

    Without re-anchoring at the top of each push the first tick of the new attempt would
    measure the whole run-up as displacement, release the commit and zero the tally -- so
    ``attempts`` could never be reached and the wall case would be unbounded again.
    """
    config = StepCommitConfig(timeout=8.0, backup=1.5, speed=0.5, attempts=3)
    commit = StepCommit(config)
    body = Body()
    body.run(commit, WEDGED_PITCH, 8.5)
    assert commit.backing
    body.run(commit, WEDGED_PITCH, 1.1, velocity=-config.speed)  # a real 0.55 m of reverse
    assert commit.failures == 1
    assert body.run(commit, WEDGED_PITCH, 0.5), "the next push must start, not release"
    assert commit.failures == 1, "the run-up's own travel must not count as getting over it"
