"""What kind of ground is in front of the robot, decided from live sensors.

The follower already has two opinions about the terrain ahead and no way to reconcile
them. The lidar ring reports a clear distance to the nearest return; the height map
reports how far the ground rises and falls inside the wheel corridor. On flat ground they
agree and nothing is at stake. At the foot of a staircase they disagree completely: the
lidar sees a solid return a couple of metres out and asks for a detour, the height map
sees ground climbing and asks for a slower, straighter approach.

Measured at waypoint 17 on the full stack, with the production follower:

    relief=0.06  scan_ahead=2.49   the stairs are still outside the height map
    relief=0.19  scan_ahead=2.26   the near rows start to pick up the first riser
    relief=0.31  scan_ahead=2.22   the rise is unmistakable, the lidar has not changed
    ... and then a hundred seconds of yaw saturating alternately at -0.70 and +0.70

Nothing arbitrated, so the avoidance re-chose a side every tick and the robot span on the
spot until the clock ran out. The rise and the return are not two obstacles, they are one
staircase seen twice, and the whole job of this module is to say so.

The classifier is deliberately free of ROS and of MuJoCo. It takes numbers and returns a
verdict, which is what makes the interesting cases -- a stair edge, a wall, a wall with a
ramp beside it -- cheap to write down as tests instead of as simulator runs.

Prior knowledge is allowed in but never decides: ``TerrainReading.expected`` lets a caller
say "the course says there are stairs here", and all that does is lower the evidence
needed to confirm stairs. It cannot conjure a verdict the sensors do not support. A rule
of the form "switch to stair mode at waypoint 17" would pass today's course and fail the
first time the course moved, so it is not available.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum


class TerrainKind(Enum):
    """What the robot is about to drive into."""

    #: Nothing in the way and nothing underfoot worth slowing for.
    FLAT = "flat"
    #: Ground tilted enough to matter but continuous. Drive it, slower.
    RAMP = "ramp"
    #: Ground rising in risers the wheels can take one at a time. Drive at it, do not
    #: steer around it, and brake late so the approach does not die on the first edge.
    STAIRS = "stairs"
    #: A return the height map does not corroborate: a wall, a pillar, a parked crate.
    #: This is the one case where steering around is right.
    BLOCKED = "blocked"
    #: A rise past what the wheels can step. Needs the climb policy, not more throttle.
    HIGH_BARRIER = "high_barrier"
    #: Ground falling away past what a normal descent handles.
    DROP = "drop"
    #: The robot itself is not in a fit state to be driven anywhere.
    UNSTABLE = "unstable"
    #: Sensors too old, or evidence too contradictory, to say. Slow down.
    UNKNOWN = "unknown"


#: Kinds where steering around the obstruction is the wrong answer, because the
#: obstruction is the route. Exported because the follower needs exactly this question.
DRIVE_AT_IT = frozenset({TerrainKind.RAMP, TerrainKind.STAIRS, TerrainKind.HIGH_BARRIER})


@dataclass(frozen=True)
class TerrainReading:
    """One tick of evidence. Every field is a live measurement except ``expected``.

    ``relief_rise`` and ``relief_drop`` are metres of residual from the plane fitted to
    the wheel corridor, so a uniform ramp reads near zero on both and shows up in
    ``slope_deg`` instead. That split is what lets a ramp be told from a stair edge at
    all; see ``local_planner.ground_clearance``, which fits the same plane.
    """

    #: Metres of clear space along the direction of travel, from the lidar ring.
    lidar_clearance: float = math.inf
    #: Metres to the nearest return anywhere in the forward arc.
    obstacle_distance: float = math.inf
    #: Metres the corridor rises above the fitted plane. Positive.
    relief_rise: float = 0.0
    #: Metres the corridor falls below the fitted plane. Positive.
    relief_drop: float = 0.0
    #: Pitch of the fitted plane, degrees. Positive is uphill.
    slope_deg: float = 0.0
    #: Metres of laterally free space at the obstruction. Narrow means no detour exists.
    corridor_width: float = math.inf
    #: Robot attitude, degrees.
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    #: Body speed, m/s, and the component of it going where the robot was asked to go.
    speed: float = 0.0
    forward_progress: float = 0.0
    #: What the robot was asked for this tick. Without it a robot standing still on
    #: purpose is indistinguishable from one that is wedged.
    commanded_forward: float = 0.0
    #: Age of the oldest input the verdict would rest on, seconds.
    sensor_age: float = 0.0
    #: Prior knowledge, if the caller has any. Lowers the bar for confirming this kind;
    #: never sufficient on its own.
    expected: TerrainKind | None = None

    @property
    def tilt_deg(self) -> float:
        return math.degrees(
            math.acos(
                max(
                    -1.0,
                    min(
                        1.0,
                        math.cos(math.radians(self.pitch_deg))
                        * math.cos(math.radians(self.roll_deg)),
                    ),
                )
            )
        )


@dataclass
class TerrainConfig:
    """Thresholds, and how long each has to hold before it is believed.

    The dwell times are the reason this is a class and not a function. A single tick of a
    height map is noisy enough that any threshold will flicker across it, and a verdict
    that flickers is worse than a wrong one: the follower would alternate between driving
    at the stairs and steering around them, which is precisely the failure this module
    exists to remove.
    """

    #: A rise past this is a step rather than texture, metres.
    step_rise: float = 0.12
    #: A rise past this cannot be taken by driving at it however slowly, metres. The
    #: measured Gate 16 riser is 0.377 m and needs the climb policy; the ordinary course
    #: steps are well under this.
    barrier_rise: float = 0.30
    #: A fall past this is a drop rather than a dip, metres.
    drop_fall: float = 0.25
    #: Fitted-plane pitch past this is a ramp, degrees.
    ramp_slope: float = 7.0
    #: A return closer than this is in the way, metres.
    blocked_distance: float = 2.6
    #: Below this much rise, a return has nothing underneath it to explain it, so it is a
    #: wall. Above it, the return and the rise are the same object.
    #:
    #: Set below ``step_rise`` on purpose, which leaves a band where the evidence is
    #: genuinely mixed and the answer is UNKNOWN rather than either confident kind. The
    #: band is not symmetric in cost: calling a staircase BLOCKED steers the robot around
    #: the route and it span at waypoint 17 until the clock ran out, whereas calling a
    #: wall UNKNOWN only slows it down while it keeps looking. The first sample of that
    #: approach read 0.06 m of rise at 2.49 m, which is why the ceiling is not 0.06.
    wall_rise_ceiling: float = 0.04
    #: Robot tilt past this is not a slope being climbed, degrees.
    unstable_tilt: float = 35.0
    #: Asked to move this fast, going slower than ``wedged_speed``, for ``wedged_dwell``
    #: seconds, means wedged rather than merely slow.
    wedged_command: float = 0.15
    wedged_speed: float = 0.03
    wedged_dwell: float = 4.0
    #: Older than this and the verdict is UNKNOWN whatever the numbers say, seconds.
    stale_after: float = 0.5

    #: How long a candidate must hold before it replaces the current verdict, seconds.
    #: UNSTABLE and DROP are hazards: believing them late is worse than believing them
    #: wrongly, so they commit immediately.
    dwell: dict[TerrainKind, float] = field(
        default_factory=lambda: {
            TerrainKind.FLAT: 0.30,
            TerrainKind.RAMP: 0.20,
            TerrainKind.STAIRS: 0.20,
            TerrainKind.BLOCKED: 0.40,
            TerrainKind.HIGH_BARRIER: 0.30,
            TerrainKind.DROP: 0.0,
            TerrainKind.UNSTABLE: 0.0,
            TerrainKind.UNKNOWN: 0.60,
        }
    )
    #: Hysteresis. Thresholds are scaled by this while the kind they select is already
    #: held, so leaving a state needs a clearer signal than entering it did. Without it
    #: the robot lets go of STAIRS in the dip between two risers.
    hold_margin: float = 0.65
    #: Prior knowledge is worth this much slack on the thresholds for the expected kind.
    prior_margin: float = 0.85


@dataclass(frozen=True)
class TerrainVerdict:
    """The committed answer, plus enough to explain it in a log line."""

    kind: TerrainKind
    confidence: float
    reason: str
    #: What the evidence said this tick, before dwell. Differs from ``kind`` exactly while
    #: a change is being debounced, which is the interesting moment to see in a trace.
    candidate: TerrainKind

    @property
    def drive_at_it(self) -> bool:
        """True when steering around the obstruction would be a mistake."""
        return self.kind in DRIVE_AT_IT

    def __str__(self) -> str:
        return f"{self.kind.value}({self.confidence:.2f}) {self.reason}"


class TerrainClassifier:
    """Turns a stream of readings into a stable verdict.

    Call :meth:`update` once per control tick. The classifier keeps only the current
    verdict, how long the current candidate has been arguing for itself, and how long the
    robot has been failing to move; everything else is derived per tick.
    """

    def __init__(self, config: TerrainConfig | None = None) -> None:
        self.config = config or TerrainConfig()
        self._verdict = TerrainVerdict(
            kind=TerrainKind.UNKNOWN,
            confidence=0.0,
            reason="nothing measured yet",
            candidate=TerrainKind.UNKNOWN,
        )
        self._candidate = TerrainKind.UNKNOWN
        self._candidate_for = 0.0
        self._not_moving_for = 0.0

    @property
    def verdict(self) -> TerrainVerdict:
        return self._verdict

    def reset(self) -> None:
        """Forget the history but not the configuration. Used when a run restarts."""
        self.__init__(self.config)

    def update(self, reading: TerrainReading, dt: float) -> TerrainVerdict:
        self._track_wedging(reading, dt)
        candidate, confidence, reason = self._classify(reading)

        if candidate is self._candidate:
            self._candidate_for += dt
        else:
            self._candidate = candidate
            self._candidate_for = dt

        if candidate is not self._verdict.kind and self._candidate_for >= self._dwell_for(
            candidate
        ):
            self._verdict = TerrainVerdict(candidate, confidence, reason, candidate)
        else:
            # Keep the committed kind but refresh the commentary, so a trace shows the
            # evidence moving even while the verdict is holding still.
            self._verdict = TerrainVerdict(self._verdict.kind, confidence, reason, candidate)
        return self._verdict

    def _dwell_for(self, kind: TerrainKind) -> float:
        return self.config.dwell.get(kind, 0.3)

    def _track_wedging(self, reading: TerrainReading, dt: float) -> None:
        """Being asked to move and not moving is the only evidence of being stuck."""
        cfg = self.config
        if (
            abs(reading.commanded_forward) >= cfg.wedged_command
            and reading.speed <= cfg.wedged_speed
        ):
            self._not_moving_for += dt
        else:
            self._not_moving_for = 0.0

    def _threshold(self, value: float, kind: TerrainKind, reading: TerrainReading) -> float:
        """Scale a threshold by hysteresis and by prior knowledge, in that order.

        Both make a kind easier to reach, and they compose: on a segment the course says
        is stairs, while already in STAIRS, the rise needed to stay there is smallest.
        """
        if self._verdict.kind is kind:
            value *= self.config.hold_margin
        if reading.expected is kind:
            value *= self.config.prior_margin
        return value

    def _classify(self, reading: TerrainReading) -> tuple[TerrainKind, float, str]:
        cfg = self.config

        # Freshness first: every rule below is a statement about now, and a stale height
        # map describes ground the robot has already driven over.
        if reading.sensor_age > cfg.stale_after:
            return (
                TerrainKind.UNKNOWN,
                0.0,
                f"sensors {reading.sensor_age:.2f}s old (limit {cfg.stale_after:.2f}s)",
            )

        # The robot's own state outranks the terrain's. There is no useful classification
        # of the ground in front of a robot that is on its side or wedged against a riser.
        if reading.tilt_deg >= self._threshold(cfg.unstable_tilt, TerrainKind.UNSTABLE, reading):
            return TerrainKind.UNSTABLE, 1.0, f"tilt {reading.tilt_deg:.0f}deg"
        if self._not_moving_for >= cfg.wedged_dwell:
            return (
                TerrainKind.UNSTABLE,
                0.9,
                f"asked for {reading.commanded_forward:+.2f} and not moving for "
                f"{self._not_moving_for:.1f}s",
            )

        # A fall outranks a rise only when the fall is the larger of the two. An edge the
        # robot is about to climb shows up as both: the plane is fitted across the whole
        # corridor, so the near ground sits below it exactly as far as the far ground sits
        # above it, and a bare "is there a fall" test then reads every step up as a step down.
        #
        # Measured on the WP18 to WP19 leg. The robot stood at (27.5, 29.6) for the whole
        # 900 s budget reporting "drop(0.4x) ground falls 0.20m" while the corridor was in
        # fact rising 0.26 m in front of it, and DROP is not a kind the follower drives at, so
        # it steered and braked at a step it should have taken. The fall latched first on the
        # approach and hold_margin then kept it: staying in DROP needs only 0.16 m.
        drop = reading.relief_drop
        if drop >= self._threshold(cfg.drop_fall, TerrainKind.DROP, reading) and (
            drop >= reading.relief_rise
        ):
            return (
                TerrainKind.DROP,
                _ramp_confidence(drop, cfg.drop_fall),
                (f"ground falls {drop:.2f}m"),
            )

        rise = reading.relief_rise
        barrier = self._threshold(cfg.barrier_rise, TerrainKind.HIGH_BARRIER, reading)
        if rise >= barrier:
            return (
                TerrainKind.HIGH_BARRIER,
                _ramp_confidence(rise, barrier),
                (f"rise {rise:.2f}m exceeds what the wheels can step"),
            )

        # The arbitration this module exists for. A return with ground rising underneath
        # it is the near face of the thing the robot is meant to climb, not a wall beside
        # it, and steering around it only finds another part of the same staircase.
        step = self._threshold(cfg.step_rise, TerrainKind.STAIRS, reading)
        if rise >= step:
            return (
                TerrainKind.STAIRS,
                _ramp_confidence(rise, step),
                (f"rise {rise:.2f}m with return at {reading.obstacle_distance:.1f}m"),
            )

        blocked = self._threshold(cfg.blocked_distance, TerrainKind.BLOCKED, reading)
        if reading.obstacle_distance <= blocked and rise <= cfg.wall_rise_ceiling:
            return (
                TerrainKind.BLOCKED,
                _near_confidence(reading.obstacle_distance, blocked),
                (
                    f"return at {reading.obstacle_distance:.1f}m over flat ground "
                    f"(rise {rise:.2f}m)"
                ),
            )

        slope = self._threshold(cfg.ramp_slope, TerrainKind.RAMP, reading)
        if abs(reading.slope_deg) >= slope:
            return (
                TerrainKind.RAMP,
                _ramp_confidence(abs(reading.slope_deg), slope),
                (f"plane pitched {reading.slope_deg:+.0f}deg"),
            )

        # A return that is neither explained by a rise nor close enough to matter, over
        # ground with no slope, is scenery.
        if reading.obstacle_distance <= blocked:
            return (
                TerrainKind.UNKNOWN,
                0.3,
                (
                    f"return at {reading.obstacle_distance:.1f}m with rise {rise:.2f}m: "
                    "neither clearly wall nor clearly step"
                ),
            )
        return TerrainKind.FLAT, 1.0, "clear ahead, corridor level"


def _ramp_confidence(value: float, threshold: float) -> float:
    """How far past its threshold a measurement is, saturating at twice over."""
    if threshold <= 0.0:
        return 1.0
    return float(min(1.0, 0.5 + 0.5 * (value - threshold) / threshold))


def _near_confidence(distance: float, threshold: float) -> float:
    """The converse, for measurements that get more convincing as they get smaller."""
    if threshold <= 0.0:
        return 1.0
    return float(min(1.0, max(0.0, 1.0 - distance / (2.0 * threshold)) + 0.5))
