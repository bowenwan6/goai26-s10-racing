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
    #: Ground speed above which a pitched body is simply driving up a slope and wants no
    #: help at all. Pitch alone is not the signal: measured on the shipped track, the
    #: approach to gate 1 pitches the base a steady +8 degrees for four seconds and the
    #: robot crosses it at 0.83 m/s. Committing there would have capped it at ``speed`` and
    #: clamped its steering for no reason, on a course where 12 of 32 legs cross raised
    #: terrain. What distinguishes the ledge is not the angle, it is being pitched and
    #: getting nowhere -- at the wedge the base was doing 0.08 to 0.30 m/s.
    progress_speed: float = 0.35
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

    def reset(self) -> None:
        self._elapsed = 0.0
        self._failures = 0

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

    def update(self, pitch: float, speed: float, dt: float) -> bool:
        """Advance the cycle; True while the follower should be committing to a step.

        Takes ground speed as well as pitch because pitch alone over-triggers: a slope
        being climbed at speed looks identical in attitude to a lip being leant on, and
        only one of them wants the follower to stop steering.
        """
        # The speed test is suspended mid-run-up: reversing at ``speed`` is by construction
        # fast enough to pass it, so applying it there would abort the run-up one control
        # step after it began and leave the robot shuffling on the lip.
        moving = speed >= self.config.progress_speed and not self.backing
        if abs(pitch) < self.config.pitch_threshold or moving:
            # Levelling out or getting under way is the only evidence of success there is,
            # and it clears the tally as well as the clock: the next ledge starts fresh, and
            # a robot that has just driven off this one is not carrying its failures onward.
            self._elapsed = 0.0
            self._failures = 0
            return False
        if self.conceded:
            # Still pitched, still stuck, but out of attempts. Handing back is the whole
            # point -- the follower's classifier and planner get to look for a way round,
            # which on the one measured case was 0.2 m to the left.
            return False
        self._elapsed += dt
        if self._elapsed >= self.config.timeout + self.config.backup:
            self._elapsed = 0.0  # Run-up finished; charge the step again.
            self._failures += 1
        return not self.conceded

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
