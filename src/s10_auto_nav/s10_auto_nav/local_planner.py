"""Steering around obstacles the lidar can see.

Pure pursuit assumes the straight line to the next gate is drivable. On this course it
usually is not: sweeping the shipped track shows **12 of 32 legs** obstructed by terrain
more than 0.35 m above the gate plane, including a 2.59 m wall lying directly across the
line from gate 11 to gate 12. Following that line drove the robot into the wall face and
tipped it over, which is where every run ended.

So the follower cannot steer at the gate; it has to steer at the best *drivable* heading
that still makes progress toward the gate. This module makes that choice.

The method is the standard candidate-heading scan used by VFH and DWA, reduced to its
essentials. Headings are proposed either side of the bearing to the carrot, each is given
the distance it could be driven before something blocks a corridor as wide as the robot,
and the winner trades clearance against how far it deviates from the goal. The chosen
clearance also caps forward speed, so the robot slows as it commits to a gap rather than
arriving at it at full tilt.

Only the horizontal lidar ring is used. Its beams sit slightly above horizontal, so flat
ground never returns a hit and any short range is a genuine vertical obstruction. Drops
and steps are invisible to it -- those are the height map's job, see ``ground_clearance``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class AvoidanceConfig:
    """Tuning for the candidate-heading search."""

    #: Widest deviation from the goal bearing that may be considered, radians. Ninety
    #: degrees lets the robot set off along a wall it must round; much more and it will
    #: happily choose to drive backwards away from the gate.
    max_deviation: float = math.radians(90.0)
    n_candidates: int = 31

    #: Half-width of the corridor swept along a candidate heading, metres. The S10 is
    #: about 0.5 m across, so this is the half-width plus margin for pose error.
    corridor_half_width: float = 0.45

    #: Clearance beyond this is treated as unobstructed; obstacles further away should
    #: not influence steering.
    probe_distance: float = 4.0

    #: Clearance below this blocks a heading outright rather than merely penalising it.
    blocked_distance: float = 0.8

    #: Relative weight of keeping clearance versus holding the goal bearing.
    clearance_weight: float = 1.6
    deviation_weight: float = 1.0

    #: Forward speed is scaled by clearance and floored here, so the robot keeps creeping
    #: toward a gap instead of stopping dead in front of one.
    min_speed_fraction: float = 0.15
    #: Clearance at which forward speed is no longer reduced.
    full_speed_clearance: float = 3.0


@dataclass
class Steering:
    """Outcome of one planning step, in the robot's body frame."""

    #: Chosen heading, radians, relative to the robot's current heading.
    heading: float
    #: Drivable distance along that heading, metres.
    clearance: float
    #: Multiplier to apply to the commanded forward speed, in ``[0, 1]``.
    speed_scale: float
    #: True when nothing scored above the blocked threshold.
    blocked: bool


def wrap_angle(angle: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalPlanner:
    """Chooses a drivable heading from a horizontal lidar ring."""

    def __init__(self, config: AvoidanceConfig | None = None) -> None:
        self.cfg = config or AvoidanceConfig()

    def clearances(
        self, ranges: np.ndarray, angles: np.ndarray, headings: np.ndarray
    ) -> np.ndarray:
        """Drivable distance along each heading, metres.

        A beam obstructs a heading when it falls inside the corridor swept along it:
        decompose each return into components along and across the candidate heading, and
        keep the nearest along-distance whose across-distance is within the corridor.
        Returns behind the robot are ignored.
        """
        cfg = self.cfg
        ranges = np.asarray(ranges, float)
        angles = np.asarray(angles, float)

        # Drop non-returns so they cannot masquerade as obstacles at range_max.
        valid = np.isfinite(ranges) & (ranges > 0.0)
        if not np.any(valid):
            return np.full(len(headings), cfg.probe_distance)
        ranges, angles = ranges[valid], angles[valid]

        # (n_headings, n_beams): each beam expressed in each candidate's frame.
        relative = angles[None, :] - np.asarray(headings, float)[:, None]
        along = ranges[None, :] * np.cos(relative)
        across = ranges[None, :] * np.sin(relative)

        obstructs = (along > 0.0) & (np.abs(across) < cfg.corridor_half_width)
        distances = np.where(obstructs, along, np.inf)
        return np.minimum(distances.min(axis=1), cfg.probe_distance)

    def plan(self, ranges: np.ndarray, angles: np.ndarray, goal_bearing: float) -> Steering:
        """Pick a heading toward ``goal_bearing`` that is actually drivable.

        ``goal_bearing`` is relative to the robot's heading, as is the result.
        """
        cfg = self.cfg
        headings = goal_bearing + np.linspace(
            -cfg.max_deviation, cfg.max_deviation, cfg.n_candidates
        )
        headings = np.array([wrap_angle(h) for h in headings])

        clearance = self.clearances(ranges, angles, headings)

        # Deviation is measured from the goal bearing, not from straight ahead: turning is
        # cheap, giving up progress toward the gate is not.
        deviation = np.abs(np.array([wrap_angle(h - goal_bearing) for h in headings]))

        score = (
            cfg.clearance_weight * (clearance / cfg.probe_distance)
            - cfg.deviation_weight * (deviation / cfg.max_deviation)
        )

        # Prefer the goal bearing when scores tie, rather than whichever end of the sweep
        # numpy happens to visit first.
        best = int(np.lexsort((deviation, -score))[0])

        chosen_clearance = float(clearance[best])
        blocked = chosen_clearance < cfg.blocked_distance

        span = max(cfg.full_speed_clearance - cfg.blocked_distance, 1e-6)
        openness = (chosen_clearance - cfg.blocked_distance) / span
        speed_scale = cfg.min_speed_fraction + (1.0 - cfg.min_speed_fraction) * float(
            np.clip(openness, 0.0, 1.0)
        )

        return Steering(
            heading=float(headings[best]),
            clearance=chosen_clearance,
            speed_scale=0.0 if blocked else speed_scale,
            blocked=blocked,
        )


#: Residual above which a height map cell is not treated as ground at all, metres. The
#: course runs under arches whose decks sit ~2.3 m up; a downward ray that starts above one
#: returns the deck, and read as terrain that is an impassable wall on ground the robot
#: walks straight through. Anything this far above the local surface is a wall or a
#: ceiling, and walls are the lidar's department.
CEILING_RESIDUAL = 0.6

#: Speed is never scaled below this by terrain alone. Reaching zero is unrecoverable: the
#: robot stops, which stops the height map changing, which keeps the robot stopped. The
#: course climbs 6.7 m over ledges that have to be walked up, so creeping is always a
#: better answer than freezing.
MIN_TERRAIN_FRACTION = 0.25

#: Relief below this fraction of the declared limit costs no speed at all.
#:
#: Slowing down is the wrong response to terrain the robot is built to cross. The course
#: is a staircase of 0.125 m steps -- four of them between gates 5 and 6 alone -- and each
#: read as a quarter of the drop limit and cost a fifth of the speed, at exactly the moment
#: momentum is what carries the wheels over the edge. Braking is for relief approaching the
#: limit, where the robot really might not make it; a quarter of the limit is pavement.
FREE_RELIEF_FRACTION = 0.25

#: Fraction of the height map's width the wheels can actually reach.
#:
#: The grid spans 1.2 m laterally; the S10 is about 0.5 m across. The difference is not
#: academic. Gate 9 crosses a bridge with a parapet along its edge, which lands in the
#: outermost column 0.55 m above the deck and so reads as a step no machine could climb --
#: and at 0.55 it falls just under CEILING_RESIDUAL, so it is admitted as ground rather
#: than rejected as wall. One run spent 275 s creeping at 0.07 m/s beside a railing it was
#: never going to touch, on a deck the base itself reported dead level.
#:
#: Speed is a question about the ground under the wheels. Keeping clear of things beside
#: the robot is the lidar planner's department, and it had the corridor at 1.00 throughout.
WHEEL_CORRIDOR_FRACTION = 5 / 9


def _wheel_corridor(grid: np.ndarray) -> np.ndarray:
    """The central columns of the patch: the strip the wheels will actually cross.

    Kept centred and symmetric, so the body axis stays in the middle however many columns
    the grid has. Grids too narrow to trim are returned whole.
    """
    if grid.ndim != 2 or grid.shape[1] < 3:
        return grid
    n_y = grid.shape[1]
    keep = max(1, int(round(n_y * WHEEL_CORRIDOR_FRACTION)))
    # Trim the same number of columns from each side, so widen by one rather than sit
    # off-centre when the arithmetic does not divide evenly.
    if (n_y - keep) % 2:
        keep += 1
    margin = (n_y - keep) // 2
    return grid[:, margin : n_y - margin]


#: Longitudinal spacing of the shipped height map, metres: 1.8 m of span over 13 rows.
#: Only used to turn the fitted plane's per-cell gradient into a slope in degrees.
DEFAULT_CELL_X = 1.8 / 12


@dataclass(frozen=True)
class Relief:
    """How the ground under the wheels departs from the plane fitted through it.

    Split into a rise, a drop and the slope of the plane itself because those three ask
    different questions. A ramp is all slope and almost no residual; a stair edge is all
    residual and, over a long enough patch, almost no slope. Collapsing them into one
    "roughness" number is what makes a ramp indistinguishable from a step, and the follower
    has to treat those two completely differently.
    """

    #: Metres the corridor rises above the fitted plane, and falls below it. Both positive.
    rise: float = 0.0
    drop: float = 0.0
    #: Pitch of the fitted plane along the direction of travel, degrees, uphill positive.
    slope_deg: float = 0.0
    #: False when the patch was too small or too occluded to fit anything to, in which
    #: case every other field is zero and means "not measured" rather than "flat".
    valid: bool = False


def terrain_relief(heightmap: np.ndarray, cell_x: float = DEFAULT_CELL_X) -> Relief:
    """Measure the ground under the wheels. See :class:`Relief` for what comes back.

    Shared with :func:`ground_clearance` on purpose: the speed scale and the terrain
    classifier disagreeing about how high the step ahead is would be its own bug.
    """
    grid = _wheel_corridor(np.asarray(heightmap, float))
    if grid.size == 0:
        return Relief()

    # Reject ceilings before fitting, not after. A deck occupying a third of the patch tilts
    # the plane hard enough that the flat ground beneath it comes out as a pit, and the robot
    # brakes for the hole it just invented.
    #
    # The reference comes from the near rows rather than from the whole patch, because the
    # robot is standing on those: they are ground by definition. Taking the median of
    # everything fails exactly when it matters, since walking head-on into an arch fills most
    # of the patch with deck and makes the deck the median.
    near = grid[: max(1, grid.shape[0] // 3)] if grid.ndim == 2 else grid
    is_ground = grid <= float(np.median(near)) + CEILING_RESIDUAL
    if is_ground.sum() < 3:
        return Relief()

    if grid.ndim == 2:
        plane = _fitted_plane(grid, is_ground)
        residual = grid - plane
        # The plane is evaluated on cell indices, so its x gradient is metres per cell.
        rows = plane[:, plane.shape[1] // 2]
        gradient = float(np.polyfit(np.arange(rows.size), rows, 1)[0]) if rows.size > 1 else 0.0
        slope_deg = math.degrees(math.atan2(gradient, max(cell_x, 1e-6)))
    else:
        residual = grid - float(np.median(grid))
        slope_deg = 0.0

    ground = residual[is_ground]
    return Relief(
        rise=float(max(0.0, ground.max())),
        drop=float(max(0.0, -ground.min())),
        slope_deg=slope_deg,
        valid=True,
    )


def ground_clearance(heightmap: np.ndarray, max_step: float, max_drop: float) -> float:
    """Fraction of commanded speed the terrain underfoot allows.

    The lidar ring rides above horizontal and so is blind to steps and drop-offs. The
    height map sees both: it reports terrain height relative to the base, so flat ground
    under a standing S10 reads about -0.42 m.

    Relief is measured against a plane fitted to the patch, not against its median. The
    course climbs 6.7 m, so the ground under the robot is usually a ramp, and a ramp's own
    gradient across a 1.8 m patch reads as a step of the same size. Measured against the
    median, the uniform 9-degree ramp out of the start box scored 0.59 and cost a third of
    the speed on ground that is perfectly drivable. Only the residual from the fitted plane
    distinguishes a ramp the robot can walk up from a stair edge it cannot.

    Only the wheel corridor is judged; see WHEEL_CORRIDOR_FRACTION.
    """
    relief = terrain_relief(heightmap)
    if not relief.valid:
        return 1.0

    # A step of max_step and a drop of max_drop are equally disqualifying, so each is
    # normalised against its own limit before the worse of the two is taken.
    worst = max(relief.rise / max(max_step, 1e-6), relief.drop / max(max_drop, 1e-6))

    # Re-scale so the free band costs nothing and the limit still costs everything.
    span = max(1.0 - FREE_RELIEF_FRACTION, 1e-6)
    penalty = (worst - FREE_RELIEF_FRACTION) / span
    return float(np.clip(1.0 - penalty, MIN_TERRAIN_FRACTION, 1.0))


def _fitted_plane(grid: np.ndarray, is_ground: np.ndarray) -> np.ndarray:
    """Plane fitted to the cells flagged as ground, evaluated over the whole grid."""
    n_x, n_y = grid.shape
    xs, ys = np.meshgrid(np.arange(n_x), np.arange(n_y), indexing="ij")
    basis = np.column_stack([xs.ravel(), ys.ravel(), np.ones(grid.size)])
    keep = is_ground.ravel()
    coefficients, *_ = np.linalg.lstsq(basis[keep], grid.ravel()[keep], rcond=None)
    return (basis @ coefficients).reshape(grid.shape)
