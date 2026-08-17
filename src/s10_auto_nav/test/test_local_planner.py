"""Obstacle-aware steering.

The geometry in ``test_rounds_the_wall_between_gates_11_and_12`` is taken from the run
that ended every lap: a 2.59 m wall across the straight line to gate 12.
"""

import math

import numpy as np
import pytest

from s10_auto_nav.local_planner import (
    AvoidanceConfig,
    LocalPlanner,
    ground_clearance,
    terrain_relief,
    wrap_angle,
)

N_BEAMS = 64
RANGE_MAX = 12.0


def ring(angles_deg_blocked=(), distance=1.0, n=N_BEAMS):
    """A lidar ring that is clear everywhere except the given bearings."""
    angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
    ranges = np.full(n, RANGE_MAX)
    for deg in angles_deg_blocked:
        target = math.radians(deg)
        idx = int(np.argmin(np.abs(np.array([wrap_angle(a - target) for a in angles]))))
        ranges[idx] = distance
    return ranges, angles


def wall(normal_deg, half_width_deg, distance, n=N_BEAMS):
    """A continuous obstruction spanning a range of bearings."""
    angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
    ranges = np.full(n, RANGE_MAX)
    for i, a in enumerate(angles):
        if abs(wrap_angle(a - math.radians(normal_deg))) <= math.radians(half_width_deg):
            # Range to a flat face, not a constant radius.
            ranges[i] = distance / max(math.cos(wrap_angle(a - math.radians(normal_deg))), 1e-3)
    return ranges, angles


def test_open_ground_steers_straight_at_the_goal():
    planner = LocalPlanner()
    ranges, angles = ring()
    steering = planner.plan(ranges, angles, goal_bearing=0.0)
    assert steering.heading == pytest.approx(0.0, abs=1e-9)
    assert not steering.blocked
    assert steering.speed_scale == pytest.approx(1.0)


def test_open_ground_holds_an_off_axis_goal():
    planner = LocalPlanner()
    ranges, angles = ring()
    goal = math.radians(40.0)
    steering = planner.plan(ranges, angles, goal_bearing=goal)
    assert steering.heading == pytest.approx(goal, abs=1e-9)


def test_clearance_sees_an_obstacle_dead_ahead():
    planner = LocalPlanner()
    ranges, angles = wall(normal_deg=0.0, half_width_deg=30.0, distance=2.0)
    clear = planner.clearances(ranges, angles, np.array([0.0]))
    assert clear[0] == pytest.approx(2.0, abs=0.1)


def test_clearance_ignores_obstacles_outside_the_corridor():
    planner = LocalPlanner()
    # A single return 3 m away at 90 degrees is far outside a 0.45 m corridor ahead.
    ranges, angles = ring(angles_deg_blocked=(90.0,), distance=3.0)
    clear = planner.clearances(ranges, angles, np.array([0.0]))
    assert clear[0] == pytest.approx(planner.cfg.probe_distance)


def test_clearance_ignores_obstacles_behind():
    planner = LocalPlanner()
    ranges, angles = ring(angles_deg_blocked=(180.0,), distance=0.3)
    clear = planner.clearances(ranges, angles, np.array([0.0]))
    assert clear[0] == pytest.approx(planner.cfg.probe_distance)


def test_steers_around_a_wall_across_the_goal_bearing():
    planner = LocalPlanner()
    ranges, angles = wall(normal_deg=0.0, half_width_deg=25.0, distance=1.5)
    steering = planner.plan(ranges, angles, goal_bearing=0.0)
    assert abs(steering.heading) > math.radians(20.0), "must turn away from the wall"
    assert steering.clearance > 1.5


#: Ranges measured by casting the ``/scan`` ring against the shipped track from gate 11,
#: the pose the robot arrives at after gate 10. Facing south, having come down the leg
#: from gate 10; the goal is gate 12, roughly west.
GATE11_FACING_SOUTH = [
    12.0, 12.0, 2.77, 1.86, 1.41, 1.15, 0.97, 0.85, 0.76, 0.70, 0.65, 0.61, 0.59, 0.62,
    0.93, 1.85, 12.0, 12.0, 12.0, 12.0, 12.0, 3.32, 3.32, 12.0, 12.0, 12.0, 12.0, 12.0,
    12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 2.20, 2.26, 12.0,
    12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 1.68, 1.80,
    12.0, 12.0, 12.0, 12.0, 3.92, 12.0, 12.0, 12.0,
]

#: The same spot after turning to face the goal. The block's east face is 0.34 m off the
#: nose, which is what pure pursuit drove into.
GATE11_FACING_WEST = [
    12.0, 12.0, 12.0, 12.0, 1.91, 1.81, 10.35, 12.0, 12.0, 12.0, 12.0, 3.88, 4.01, 6.46,
    12.0, 12.0, 12.0, 4.32, 1.75, 1.17, 0.89, 0.72, 0.61, 0.54, 0.48, 0.44, 0.41, 0.39,
    0.37, 0.36, 0.35, 0.34, 0.34, 12.0, 12.0, 12.0, 12.0, 12.0, 3.24, 12.0, 12.0, 12.0,
    12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 2.51,
    2.54, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0,
]

BEAM_ANGLES = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False)


def test_bypasses_the_block_on_arrival_at_gate_11():
    """The failure that ended every run, from a ray-cast of the shipped track.

    A block 2.6 m tall spans x in [-4.6, -11.2], y in [42.1, 47.0]. Gate 11 sits at
    (-4.245, 42.54), a third of a metre off its east face, and gate 12 is 9 m west on the
    far side of it. The only way through is to dip south around the block's south face,
    so a correct planner gives up bearing here rather than clearance.
    """
    planner = LocalPlanner()
    # Facing south down the leg from gate 10; gate 12 bears 84 degrees to starboard.
    steering = planner.plan(
        np.array(GATE11_FACING_SOUTH), BEAM_ANGLES, goal_bearing=math.radians(-83.9)
    )
    assert not steering.blocked
    assert steering.clearance > 3.0, "the southern bypass is wide open"
    # South-west: committed to the goal side, but bowed away from the block's face.
    assert math.radians(-60.0) < steering.heading < math.radians(-40.0)


def test_turns_out_when_pinned_against_the_block_face():
    """Nose 0.34 m off the face, goal dead ahead: every heading but south is a wall."""
    planner = LocalPlanner()
    steering = planner.plan(
        np.array(GATE11_FACING_WEST), BEAM_ANGLES, goal_bearing=math.radians(6.1)
    )
    assert steering.clearance > 3.0
    assert steering.heading > math.radians(70.0), "must turn south, the only open bearing"


def test_declares_blocked_when_boxed_in():
    planner = LocalPlanner()
    angles = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False)
    ranges = np.full(N_BEAMS, 0.4)
    steering = planner.plan(ranges, angles, goal_bearing=0.0)
    assert steering.blocked
    assert steering.speed_scale == 0.0


def test_speed_falls_off_as_clearance_shrinks():
    """Enclosing the robot isolates the speed law: with no way out, only range varies.

    A flat wall will not do. However wide it is made, the planner steers around its edge
    and keeps full speed, which is the right behaviour but tests the search rather than
    the speed schedule.
    """
    planner = LocalPlanner()
    angles = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False)
    scales = [
        planner.plan(np.full(N_BEAMS, d), angles, goal_bearing=0.0).speed_scale
        for d in (2.8, 2.0, 1.2)
    ]
    assert scales[0] > scales[1] > scales[2]
    assert all(0.0 <= s <= 1.0 for s in scales)


def test_deviation_is_bounded_by_config():
    cfg = AvoidanceConfig(max_deviation=math.radians(30.0))
    planner = LocalPlanner(cfg)
    ranges, angles = wall(normal_deg=0.0, half_width_deg=25.0, distance=1.0)
    steering = planner.plan(ranges, angles, goal_bearing=0.0)
    assert abs(wrap_angle(steering.heading)) <= math.radians(30.0) + 1e-9


def test_no_returns_reads_as_open():
    planner = LocalPlanner()
    angles = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False)
    ranges = np.full(N_BEAMS, np.nan)
    steering = planner.plan(ranges, angles, goal_bearing=0.0)
    assert not steering.blocked


def test_ground_clearance_is_full_on_flat_terrain():
    flat = np.full((13, 9), -0.42)
    assert ground_clearance(flat, max_step=0.35, max_drop=0.5) == pytest.approx(1.0)


def test_ground_clearance_is_full_on_a_uniform_ramp():
    """The ramp out of the start box, measured off the track: 0.0236 m per cell.

    Braking for this cost a third of the speed on ground the robot walks up without
    noticing, because relief taken against the patch median reads a ramp's own gradient
    as a step of the same size.
    """
    ramp = -0.42 + 0.0236 * np.arange(13)[:, None] * np.ones((13, 9))
    assert ground_clearance(ramp, max_step=0.35, max_drop=0.5) == pytest.approx(1.0)


def test_ground_clearance_ignores_an_overhead_deck():
    """The course runs under arches whose decks sit about 2.3 m up.

    A downward ray that starts above the deck returns the deck, so the height map reports
    a wall on ground the robot walks straight through. Reading that literally parked a run
    under the first arch for five minutes.
    """
    grid = np.full((13, 9), -0.42)
    grid[-4:, :] = 1.0                     # the deck, as the sampler clips it
    assert ground_clearance(grid, max_step=0.35, max_drop=0.5) == pytest.approx(1.0)


def test_ground_clearance_ignores_a_deck_that_fills_the_patch():
    """Walking head-on into an arch puts deck in most of the map, making it the median.

    Referencing the whole patch fails exactly here, which is the case that matters: the
    robot crawled the first arch at a quarter speed because the deck outvoted the ground.
    """
    grid = np.full((13, 9), -0.42)
    grid[4:, :] = 1.0
    assert ground_clearance(grid, max_step=0.35, max_drop=0.5) == pytest.approx(1.0)


def test_ground_clearance_is_full_on_the_courses_own_steps():
    """The staircase between gates 5 and 6 is four 0.125 m steps, and must not cost speed.

    Measured on the shipped track, and measured again in the run that stalled on it: each
    step read as relief 0.13 and scaled speed to 0.83. The robot cleared the two rises and
    stopped dead on the drop, because momentum is the whole mechanism by which a wheeled
    base gets over an edge, and braking for the edge removes it.
    """
    for far_rows, offset in [(2, +0.125), (2, -0.125), (4, +0.125), (4, -0.125)]:
        grid = np.full((13, 9), -0.42)
        grid[-far_rows:, :] = -0.42 + offset
        clearance = ground_clearance(grid, max_step=0.35, max_drop=0.5)
        assert clearance == pytest.approx(1.0), f"{offset:+.3f} m over {far_rows} rows"


def test_ground_clearance_drops_for_a_step():
    grid = np.full((13, 9), -0.42)
    grid[-3:, :] = -0.42 + 0.30            # a 0.30 m step in the far rows
    assert ground_clearance(grid, max_step=0.35, max_drop=0.5) < 0.9


def test_ground_clearance_drops_for_a_hole():
    grid = np.full((13, 9), -0.42)
    grid[-3:, :] = -1.0                    # a drop-off ahead
    assert ground_clearance(grid, max_step=0.35, max_drop=0.5) < 0.9


def test_ground_clearance_ignores_a_parapet_beside_the_wheels():
    """The gate 9 bridge: a railing in the outermost column is not terrain to brake for.

    At 0.55 m above the deck it sits just under CEILING_RESIDUAL, so it is admitted as
    ground rather than rejected as a wall, and it floored the terrain scale for 275 s of
    creeping alongside it.
    """
    grid = np.full((13, 9), -0.42)
    grid[:, -1] = -0.42 + 0.55             # parapet down the right-hand edge
    grid[:4, -2] = -0.42 + 0.79            # and its taller post behind the robot
    assert ground_clearance(grid, max_step=0.35, max_drop=0.5) == pytest.approx(1.0)


def test_ground_clearance_still_brakes_for_a_step_across_the_corridor():
    """Narrowing the judged strip must not blind it to what is genuinely in the way."""
    grid = np.full((13, 9), -0.42)
    grid[-3:, :] = -0.42 + 0.55            # spans the full width, wheels included
    assert ground_clearance(grid, max_step=0.35, max_drop=0.5) < 0.9


def test_ground_clearance_never_reaches_zero():
    """Zero is unrecoverable: a stopped robot sees a frozen height map and stays stopped."""
    grid = np.full((13, 9), -0.42)
    grid[6:, :] = -0.42 + 0.55             # a step well past anything walkable
    assert ground_clearance(grid, max_step=0.35, max_drop=0.5) > 0.0


# --------------------------------------------------------------------------------------
# Across the path, or beside it.
# --------------------------------------------------------------------------------------
#
# ``rise`` is a maximum over the corridor, so a stair and a post give the same number while
# needing opposite manoeuvres. ``rise_fraction`` is the number that tells them apart, and
# these fix the two ends of it. The threshold that reads it lives in ``terrain.py``; what is
# asserted here is only that the measurement separates the two shapes.


def test_a_step_across_the_path_rises_across_the_whole_width():
    grid = np.full((13, 9), -0.42)
    grid[-3:, :] = -0.42 + 0.30
    assert terrain_relief(grid).rise_fraction == pytest.approx(1.0)


def test_something_standing_beside_the_path_does_not():
    """The pillar at (32.03, 15.66), which was called a staircase and driven into.

    Its face clipped the left edge of the patch, reaching one column into the five-column
    wheel corridor at 0.32 m above the deck while the other four stayed flat. ``rise`` alone
    reported 0.29 m and was indistinguishable from the step above. One column in five is
    where the measured 0.20 comes from.
    """
    grid = np.full((13, 9), -0.43)
    grid[6:, :3] = -0.43 + 0.32            # column 2 is the corridor's left-hand edge
    relief = terrain_relief(grid)
    assert relief.rise > 0.12, "the rise is real; it is the width that is wrong"
    assert relief.rise_fraction <= 0.4


def test_flat_ground_has_no_rise_to_apportion():
    assert terrain_relief(np.full((13, 9), -0.42)).rise_fraction == pytest.approx(0.0)
