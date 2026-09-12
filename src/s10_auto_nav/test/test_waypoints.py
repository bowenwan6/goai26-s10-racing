"""Course progress and lookahead behaviour."""

import numpy as np
import pytest

from s10_auto_nav.waypoints import Course, Waypoint


def straight_course(n: int = 5, spacing: float = 2.0, **kwargs) -> Course:
    return Course(
        [Waypoint(i, np.array([i * spacing, 0.0, 0.0])) for i in range(n)],
        **kwargs,
    )


def test_requires_waypoints():
    with pytest.raises(ValueError):
        Course([])


def test_cursor_advances_only_within_radius():
    course = straight_course(score_radius=0.5)
    assert course.cursor == 0

    assert not course.update(np.array([1.0, 0.0]))
    assert course.cursor == 0

    assert course.update(np.array([0.2, 0.0]))
    assert course.cursor == 1


# --------------------------------------------------------------------------------------
# What counts as reaching a gate.
# --------------------------------------------------------------------------------------
#
# The cursor used to move at advance_radius, 0.35 m, against a scorer that wants 0.2 m. On
# a corner that is the difference between a gate and a near miss, because the carrot swings
# onto the next leg the moment the cursor moves. Measured on the continuous waypoint 16 to
# 32 run: gates 17, 21 and 22 approached to 0.338, 0.320 and 0.336 m and scored nothing.


def approach(course: Course, gate_xy, distances: list[float]) -> list[bool]:
    """Walk the robot straight at ``gate_xy`` through each distance in turn."""
    gate = np.asarray(gate_xy, float)
    return [course.update(gate - np.array([d, 0.0])) for d in distances]


def test_coming_inside_the_advance_radius_is_not_by_itself_reaching_the_gate():
    """0.34 m from a gate is a miss, and the cursor must not call it anything else."""
    course = straight_course(n=3, spacing=2.0)
    course.update(np.array([0.0, 0.0]))
    assert course.cursor == 1

    assert approach(course, (2.0, 0.0), [0.60, 0.45, 0.34]) == [False, False, False]
    assert course.cursor == 1, "still approaching; the closest point has not happened yet"


def test_entering_the_internal_radius_boundary_advances():
    course = straight_course(n=3, spacing=2.0)
    course.update(np.array([0.0, 0.0]))
    approach(course, (2.0, 0.0), [0.60, 0.181, 0.18])
    assert course.cursor == 2


def test_receding_after_a_near_miss_never_releases_the_cursor():
    """A 0.181 m miss cannot discard the gate even after the robot moves away."""
    course = straight_course(n=3, spacing=2.0)
    course.update(np.array([0.0, 0.0]))
    moved = approach(course, (2.0, 0.0), [0.60, 0.30, 0.181, 0.26, 0.60])
    assert moved == [False] * 5
    assert course.cursor == 1


def test_receding_from_a_gate_never_approached_does_not_release_it():
    """Otherwise a detour around an obstacle would discard the gate it was detouring to."""
    course = straight_course(n=3, spacing=2.0)
    course.update(np.array([0.0, 0.0]))
    assert approach(course, (2.0, 0.0), [1.2, 0.9, 1.4, 2.0]) == [False] * 4
    assert course.cursor == 1


def test_advance_bound_cannot_be_looser_than_local_scoring():
    course = straight_course(n=3, spacing=2.0, advance_radius=0.35, score_radius=0.18)
    course.update(np.array([0.0, 0.0]))
    assert not course.update(np.array([2.181, 0.0]))
    assert course.cursor == 1


def test_one_waypoint_consumed_per_update():
    """A pose jump past several gates must not skip the course."""
    course = straight_course(score_radius=100.0)
    course.update(np.array([0.0, 0.0]))
    assert course.cursor == 1


def test_course_finishes():
    course = straight_course(n=3, score_radius=10.0)
    for _ in range(3):
        course.update(np.array([0.0, 0.0]))
    assert course.finished
    assert course.target is None
    assert not course.update(np.array([0.0, 0.0]))


def test_lookahead_rides_the_line_to_the_current_gate():
    course = straight_course(n=5, spacing=2.0)
    course.update(np.array([0.0, 0.0]))
    assert course.cursor == 1

    # Gate closer than the lookahead: steer at the gate itself.
    point = course.lookahead_point(np.array([0.0, 0.0]), distance=3.0)
    np.testing.assert_allclose(point, [2.0, 0.0], atol=1e-6)

    # Gate further than the lookahead: steer at a point on the line to it.
    point = course.lookahead_point(np.array([0.0, 0.0]), distance=1.0)
    np.testing.assert_allclose(point, [1.0, 0.0], atol=1e-6)


def test_lookahead_never_runs_past_an_uncleared_gate():
    """Rounding a corner early loses the gate, and the scorer requires every gate.

    Approaching gate 1 the carrot must stay on the northward leg. Allowing it onto the
    westward leg 1->2 took the robot 2.1 m past the gate, outside the advance radius,
    and the run stalled there permanently.
    """
    course = Course(
        [
            Waypoint(0, np.array([-0.7125, 11.6550, 0.475])),
            Waypoint(1, np.array([-8.7300, 11.8425, 0.475])),
        ],
        advance_radius=0.35,
    )
    robot = np.array([-0.4740, 10.6307])
    point = course.lookahead_point(robot, distance=1.43)
    assert point[0] > -0.75, "carrot must not be dragged west onto the next leg"
    np.testing.assert_allclose(point, [-0.7125, 11.6550], atol=1e-6)


def test_lookahead_does_not_fold_back_at_a_switchback():
    """Regression: the carrot must not land on top of a robot that overshot a gate.

    Standing 0.69 m past gate 2 used to yield a carrot 0.12 m away, which the speed
    schedule read as an arrival, braking to 0.06 m/s and stalling the run.
    """
    course = Course(
        [
            Waypoint(0, np.array([-0.7125, 11.6550, 0.475])),
            Waypoint(1, np.array([-8.7300, 11.8425, 0.475])),
            Waypoint(2, np.array([-10.5375, 16.3275, 0.475])),
        ],
        advance_radius=0.35,
    )
    course.update(np.array([-0.7125, 11.6550]))
    assert course.cursor == 1

    robot = np.array([-8.885, 12.514])
    point = course.lookahead_point(robot, distance=1.43)
    assert np.linalg.norm(point - robot) == pytest.approx(0.689, abs=0.01)


def test_lookahead_falls_back_to_the_current_gate_when_it_is_close():
    course = straight_course(n=3, spacing=2.0)
    point = course.lookahead_point(np.array([0.0, 0.0]), distance=1000.0)
    np.testing.assert_allclose(point, [0.0, 0.0], atol=1e-6)


def test_remaining_distance_shrinks_as_the_robot_advances():
    course = straight_course(n=5, spacing=2.0)
    start = course.remaining_distance(np.array([0.0, 0.0]))
    assert start == pytest.approx(8.0)

    course.update(np.array([0.0, 0.0]))
    assert course.remaining_distance(np.array([2.0, 0.0])) == pytest.approx(6.0)


def test_remaining_distance_is_zero_once_finished():
    course = straight_course(n=2, score_radius=10.0)
    for _ in range(2):
        course.update(np.array([0.0, 0.0]))
    assert course.remaining_distance(np.array([0.0, 0.0])) == 0.0


def test_reset_restores_the_start():
    course = straight_course(score_radius=10.0)
    course.update(np.array([0.0, 0.0]))
    course.reset()
    assert course.cursor == 0
