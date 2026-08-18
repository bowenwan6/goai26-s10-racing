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
    course = straight_course(advance_radius=0.5)
    assert course.cursor == 0

    assert not course.update(np.array([1.0, 0.0]))
    assert course.cursor == 0

    assert course.update(np.array([0.2, 0.0]))
    assert course.cursor == 1


def test_one_waypoint_consumed_per_update():
    """A pose jump past several gates must not skip the course."""
    course = straight_course(advance_radius=100.0)
    course.update(np.array([0.0, 0.0]))
    assert course.cursor == 1


def test_course_finishes():
    course = straight_course(n=3, advance_radius=10.0)
    for _ in range(3):
        course.update(np.array([0.0, 0.0]))
    assert course.finished
    assert course.target is None
    assert not course.update(np.array([0.0, 0.0]))


def test_lookahead_walks_along_the_polyline():
    course = straight_course(n=5, spacing=2.0)
    point = course.lookahead_point(np.array([0.0, 0.0]), distance=3.0)
    np.testing.assert_allclose(point, [3.0, 0.0], atol=1e-6)


def test_lookahead_clamps_to_final_waypoint():
    course = straight_course(n=3, spacing=2.0)
    point = course.lookahead_point(np.array([0.0, 0.0]), distance=1000.0)
    np.testing.assert_allclose(point, [4.0, 0.0], atol=1e-6)


def test_remaining_distance_shrinks_as_the_robot_advances():
    course = straight_course(n=5, spacing=2.0)
    start = course.remaining_distance(np.array([0.0, 0.0]))
    assert start == pytest.approx(8.0)

    course.update(np.array([0.0, 0.0]))
    assert course.remaining_distance(np.array([2.0, 0.0])) == pytest.approx(6.0)


def test_remaining_distance_is_zero_once_finished():
    course = straight_course(n=2, advance_radius=10.0)
    for _ in range(2):
        course.update(np.array([0.0, 0.0]))
    assert course.remaining_distance(np.array([0.0, 0.0])) == 0.0


def test_reset_restores_the_start():
    course = straight_course(advance_radius=10.0)
    course.update(np.array([0.0, 0.0]))
    course.reset()
    assert course.cursor == 0


def test_custom_start_index_is_preserved_by_reset():
    course = straight_course(start_index=3)
    assert course.cursor == 3
    course.update(np.array([6.0, 0.0]))
    assert course.cursor == 4
    course.reset()
    assert course.cursor == 3
