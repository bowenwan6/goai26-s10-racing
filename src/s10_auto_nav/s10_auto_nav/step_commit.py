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

What is *not* wrong is retrying. This class therefore never hands back to free steering
while still pitched; it cycles, pushing for :attr:`timeout` and then reversing straight for
:attr:`backup` to build a genuine run-up, indefinitely. Leaning on a wall forever is the
cost of that, and it is the cheaper mistake: the wall case loses a run that was already
lost, while handing back mid-ledge loses a run that was still winnable.
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


class StepCommit:
    """Tracks whether the body is on a step, and what to command while it is."""

    def __init__(self, config: StepCommitConfig | None = None) -> None:
        self.config = config or StepCommitConfig()
        self._elapsed = 0.0

    def reset(self) -> None:
        self._elapsed = 0.0

    @property
    def elapsed(self) -> float:
        """Seconds into the current push-or-run-up cycle."""
        return self._elapsed

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
            self._elapsed = 0.0
            return False
        self._elapsed += dt
        if self._elapsed >= self.config.timeout + self.config.backup:
            self._elapsed = 0.0  # Run-up finished; charge the step again.
        return True

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
