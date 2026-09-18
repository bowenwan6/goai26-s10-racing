"""Getting a wheeled base over a ledge.

The course between gates 5 and 6 is a staircase of 0.125 m steps -- two up, two down,
full width, no way around. Measured on the shipped track the robot crossed the two rises
at 0.6 m/s without noticing them, and then stopped dead on the first drop, pitched 16
degrees nose-down, for a minute at a time: lidar clear, no geometry within 8 m, forward
still commanded. Both escapes it ever managed came the same way, yaw settled near zero and
forward held steady for twenty-odd seconds until the wheels finally rolled off the edge.

So the response to being astride an edge is to drive squarely at it and not stop. Every
instinct the rest of the follower has is wrong here, and each was observed being wrong:

* **Sideways is wrong.** Told to translate laterally, a robot on a lip scrubs its wheels
  along the edge instead of rolling over it.
* **Turning is wrong**, for the same reason, and worse: yaw walks the contact points along
  the lip. The run that lost the most time had yaw saturated at +/-0.7 and alternating sign
  every second, x drifting 0.4 m while y never advanced at all.
* **Braking is wrong.** Momentum is the mechanism by which a wheel gets over an edge.
* **The obstacle avoidance is wrong.** The ``/scan`` ring is in the body frame, so nose-down
  17 degrees aims its nominal +2 degrees about 15 degrees *below* horizontal, into the floor
  a couple of metres ahead. The planner then dutifully steers around ground returns.
* **The stall watchdog is wrong.** Its reverse-and-retry interrupts, every 2.5 s, the one
  behaviour that has ever worked -- which is why the first stall eventually resolved and
  the second, once the watchdog got involved, never did.

What is *not* wrong is retrying. This class therefore does not hand back to free steering
after one failure; it cycles, pushing for :attr:`timeout` and then reversing straight for
:attr:`backup` to build a genuine run-up.

It used to do that indefinitely, on the argument that the wall case loses a run that was
already lost while handing back mid-ledge loses a run that was still winnable. The first
half of that turned out to be false. On the leg from waypoint 18 to 19 the robot put its
nose on a 0.544 m rise at (27.8, 29.5), held 0.7 m/s against it for 130 seconds, reared up
and went over at 70 degrees of tilt -- and the way past it was a **0.2 m sidestep**, the deck
being clear at y=30.2 for the whole length of the leg. Measured: 8.966 m round against 8.88 m
straight, a 1 per cent detour, with the body's full 0.45 m half-width. A wall the planner can
walk around is not a run that was already lost, and leaning on it costs every gate after it.

So the cycling is bounded by :attr:`attempts`. What makes that safe is what the count means:
conceding takes three consecutive *complete* failures to cross, and the slowest genuine
crossing ever measured on this track took 7.1 s against a 12 s push. A step that has resisted
three run-ups is not a step.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from s10_auto_nav.pure_pursuit import Command


@dataclass
class StepCommitConfig:
    #: Pitch past which the body is taken to be on a step, radians. Comfortably above the
    #: 1-3 degrees of gait pitch seen on the flat, and well under the 14.5 degrees a
    #: 0.125 m step puts under a 0.5 m wheelbase.
    pitch_threshold: float = math.radians(8.0)
    #: Forward speed held while crossing, and the reverse speed of the run-up.
    #:
    #: The policy's own ceiling, and deliberately so: this is the one number the step turns
    #: out to be sensitive to. Backing the base up to a fixed start line and driving the
    #: y=20.5 drop repeatedly, changing nothing but the command, gave 0.50 wedged for the
    #: full 20 s trial, 0.60 across in 7.1 s, 0.70 across in 4.3 s with 0.2 s of struggle.
    #: An earlier 0.5 here was what pinned the robot on that edge for fifty seconds: pure
    #: pursuit was asking for 0.69 on the approach and this clamp *lowered* it at the worst
    #: possible moment. Committing to a step means committing, not easing off.
    speed: float = 0.7
    #: Yaw authority retained while pitched: enough to hold a line, not enough to walk the
    #: wheels along the lip.
    yaw_rate: float = 0.15
    #: Ground the body must cover within :attr:`progress_window` before a pitched robot
    #: counts as driving up a slope rather than leaning on a ledge, metres.
    #:
    #: This was a ground *speed* of 0.35 m/s, on the reasoning that pitch alone is not the
    #: signal -- the approach to gate 1 pitches the base a steady +8 degrees for four seconds
    #: and the robot crosses it at 0.83 m/s, and committing there would clamp its steering
    #: for no reason. The reasoning holds. The proxy does not: the gait oscillates, so
    #: instantaneous speed spikes while the body goes nowhere.
    #:
    #: Measured on the wedge at the 0.16 m step to waypoint 19: 53 s pitched over 8 degrees
    #: on 1004 of 1060 samples, for **+0.01 m** of net progress while drifting 0.53 m
    #: sideways out of the lane. Ninety-four of those samples touched 0.35 m/s, and each one
    #: zeroed the clock *and* the failure tally, so the longest unbroken push the 12 s
    #: timeout ever saw was **3.9 s**. The run-up never fired, and the run-up is how the
    #: baseline got up this step -- it reached the lip at 1.16 m/s off the flat.
    #:
    #: Displacement over a window is not fooled the same way, because it is what "getting
    #: under way" actually means. Swept over that same wedge, the furthest the body ever got
    #: from where it had been 1.5 s earlier was 0.125 m, so 0.25 m has a factor of two on the
    #: worst case it has to reject, and a real crossing covers it in under a second.
    progress_distance: float = 0.25
    #: How far back to look when asking whether the body is getting anywhere, seconds.
    #:
    #: Long enough that a gait cycle averages out, short enough to notice a stop. The
    #: measurement is flat in this parameter -- the wedge's furthest excursion was 0.116 m
    #: over 0.5 s and 0.125 m over 1.5 s -- so it is not a knife edge.
    progress_window: float = 1.5
    #: How long the body must read level before the commit accepts it is off the ledge,
    #: seconds.
    #:
    #: Not instantaneous, for the same reason the progress test is not: single samples of
    #: contrary evidence are gait noise. On the waypoint 19 wedge the pitch dipped under
    #: :attr:`pitch_threshold` seven times in 53 s, median 0.40 s and longest 1.05 s, none of
    #: them the robot getting off anything -- and each dip zeroed a 12 s clock. Debounced at
    #: 1.5 s the same trace yields a run-up at exactly 15 s and concedes after three, against
    #: never conceding at all.
    #:
    #: This debounces *levelling out* only. Genuine displacement releases the commit at once,
    #: because :attr:`progress_window` has already smoothed it; making that wait as well kept
    #: the robot committed for 99 per cent of a ramp it was climbing perfectly well, against
    #: 91 per cent, for no gain on the wedge.
    level_dwell: float = 1.5
    #: Seconds of pushing before conceding this attempt and backing up for another.
    #:
    #: This was 40 s on the reasoning that both escapes ever seen took twenty-odd seconds of
    #: steady push, so interrupting one would repeat the stall watchdog's mistake. That
    #: reasoning was sound but the evidence under it was an artefact: those escapes were slow
    #: because ``speed`` was too low, and at 0.7 the same edge goes by in 4.3 s. The slowest
    #: genuine crossing measured on it is 7.1 s, so this is roughly that with margin -- long
    #: enough that no real attempt is cut short, short enough that a true wedge buys a
    #: run-up in seconds rather than in most of a minute.
    timeout: float = 12.0
    #: Seconds of straight reverse to build a run-up.
    backup: float = 3.0
    #: Complete push-and-run-up cycles to fail before conceding the obstacle is not a step.
    #:
    #: Three, at 15 s each, so a wall costs 45 s and then the planner gets to look for a way
    #: round -- against the 130 s and a fall it cost on the leg to waypoint 19. It cannot cut
    #: a real climb short: the slowest genuine crossing on this track is 7.1 s inside a 12 s
    #: push, so three consecutive failures is not a slow step, it is a different kind of
    #: object. Raising this trades gates after the obstacle for attempts at it, and the
    #: evidence says the attempts are not the scarce thing.
    attempts: int = 3


class StepCommit:
    """Tracks whether the body is on a step, and what to command while it is."""

    def __init__(self, config: StepCommitConfig | None = None) -> None:
        self.config = config or StepCommitConfig()
        self._elapsed = 0.0
        self._failures = 0
        #: Recent (age, x, y), oldest first. Progress is the distance across this window
        #: rather than an integral of speed, because the body oscillates and a speed
        #: magnitude does not cancel out when it does.
        self._trail: deque[tuple[float, float, float]] = deque()
        self._clock = 0.0
        #: Whether the body is currently taken to be on a step. Held separately from the
        #: clock because entering and leaving are deliberately not symmetric: one pitched
        #: sample is enough to engage, but leaving has to be sustained.
        self._engaged = False
        self._level_for = 0.0

    def reset(self) -> None:
        self._release()
        self._trail.clear()

    def _release(self) -> None:
        """Let go of the ledge, without forgetting where the body has just been.

        The trail deliberately survives. Clearing it here is what an earlier version did,
        and it made the commit oscillate: releasing wiped the evidence of the motion that
        justified releasing, so the next tick saw a pitched body that had gone nowhere and
        engaged again. On a slope taken at 0.83 m/s that flip-flopped several times a
        second. Only a completed run-up clears the trail, because only then is the recent
        travel -- two metres of reverse -- something that must not count.
        """
        self._elapsed = 0.0
        self._failures = 0
        self._engaged = False
        self._level_for = 0.0

    @property
    def elapsed(self) -> float:
        """Seconds into the current push-or-run-up cycle."""
        return self._elapsed

    @property
    def failures(self) -> int:
        """Complete push-and-run-up cycles that have not got the body over."""
        return self._failures

    @property
    def conceded(self) -> bool:
        """True once this obstacle has been given up on as not being a step."""
        return self._failures >= self.config.attempts

    @property
    def backing(self) -> bool:
        """True during the reverse half of the cycle."""
        return self._elapsed >= self.config.timeout

    def update(self, pitch: float, position: tuple[float, float], dt: float) -> bool:
        """Advance the cycle; True while the follower should be committing to a step.

        Takes the body's planar position as well as pitch because pitch alone
        over-triggers: a slope being climbed looks identical in attitude to a lip being
        leant on, and only one of them wants the follower to stop steering. What tells them
        apart is whether the body is getting anywhere -- see
        :attr:`StepCommitConfig.progress_distance` for why the ground speed this used to
        read cannot answer that, and could not be made to.

        Engaging and disengaging are deliberately asymmetric. One pitched sample that is
        going nowhere is enough to commit, because the cost of a moment's straight push is
        nil and the cost of missing a ledge is the run. Letting go has to be earned, because
        every way of letting go cheaply has already been measured failing on this course.
        """
        self._clock += dt
        self._trail.append((self._clock, position[0], position[1]))
        while self._trail and self._clock - self._trail[0][0] > self.config.progress_window:
            self._trail.popleft()

        level = abs(pitch) < self.config.pitch_threshold
        # The progress test is suspended mid-run-up: reversing at ``speed`` crosses
        # ``progress_distance`` in well under a second by construction, so applying it there
        # would abort the run-up almost as it began and leave the robot shuffling on the lip.
        moving = self._travelled(position) >= self.config.progress_distance and not self.backing

        if not self._engaged:
            if level or moving:
                return False
            self._engaged = True
            self._level_for = 0.0
        elif moving:
            # Real ground covered, over a window that has already averaged the gait out.
            # Clears the tally as well as the clock: the next ledge is a fresh problem, and
            # a robot that has just driven off this one is not carrying its failures on.
            self._release()
            return False
        elif level:
            # A dip in gait pitch is not the robot getting off anything; seven of them cost
            # this exact wedge every run-up it should have had. Sustained is different.
            self._level_for += dt
            if self._level_for >= self.config.level_dwell:
                self._release()
                return False
        else:
            self._level_for = 0.0

        if self.conceded:
            # Still pitched, still stuck, but out of attempts. Handing back is the whole
            # point -- the follower's classifier and planner get to look for a way round,
            # which on the one measured case was 0.2 m to the left.
            return False
        self._elapsed += dt
        if self._elapsed >= self.config.timeout + self.config.backup:
            self._elapsed = 0.0  # Run-up finished; charge the step again.
            self._failures += 1
            # Forgotten, or the run-up's own couple of metres of reverse would read as
            # progress on the first tick of the next push and cancel the failure it just
            # earned -- which would put ``attempts`` back out of reach.
            self._trail.clear()
        return not self.conceded

    def _travelled(self, position: tuple[float, float]) -> float:
        """How far the body is from where it was a window ago."""
        if not self._trail:
            return 0.0
        _, x, y = self._trail[0]
        return math.hypot(position[0] - x, position[1] - y)

    def command(self, steering: Command) -> Command:
        """Rewrite the follower's command into a straight push, or a straight run-up."""
        c = self.config
        if self.backing:
            # Backing up crooked just walks along the lip, which is the failure this whole
            # class exists to avoid, so the run-up is straight even at the cost of aim.
            return Command(forward=-c.speed, lateral=0.0, yaw_rate=0.0)

        # Floored unconditionally, including where pure pursuit would pivot in place: on an
        # edge, turning is exactly what does not work, and there is nowhere to go but over.
        yaw_rate = max(-c.yaw_rate, min(c.yaw_rate, steering.yaw_rate))
        return Command(forward=c.speed, lateral=0.0, yaw_rate=yaw_rate)
