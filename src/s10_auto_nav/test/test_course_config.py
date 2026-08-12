"""Validate the shipped racing course against the contest's own constraints."""

from pathlib import Path

import numpy as np
import pytest

from s10_auto_nav.waypoints import Course

COURSE_FILE = Path(__file__).resolve().parents[2] / "s10_bringup" / "config" / "course.yaml"

#: The scorer checks a 0.2 m horizontal radius, so consecutive gates must be further
#: apart than that or one pose could satisfy two of them.
SCORER_RADIUS = 0.2


@pytest.fixture(scope="module")
def course() -> Course:
    return Course.from_yaml(COURSE_FILE)


def test_course_file_is_present():
    assert COURSE_FILE.is_file(), f"missing {COURSE_FILE}"


def test_course_matches_the_track_scene(course):
    assert len(course) == 33


def test_waypoints_are_indexed_in_order(course):
    assert [wp.index for wp in course.waypoints] == list(range(len(course)))


def test_gates_are_separable_by_the_scorer(course):
    for a, b in zip(course.waypoints, course.waypoints[1:], strict=False):
        separation = float(np.linalg.norm(b.xy - a.xy))
        assert separation > 2 * SCORER_RADIUS, (
            f"waypoints {a.index} and {b.index} are {separation:.3f} m apart"
        )


def test_course_geometry_is_unchanged(course):
    """Guards against a silent scene edit changing what we tuned against."""
    length = course.remaining_distance(course.waypoints[0].xy)
    assert length == pytest.approx(224.2, abs=0.5)

    climb = sum(
        max(0.0, b.position[2] - a.position[2])
        for a, b in zip(course.waypoints, course.waypoints[1:], strict=False)
    )
    assert climb == pytest.approx(6.70, abs=0.05)


def test_start_and_finish_are_labelled(course):
    positions = [wp.position for wp in course.waypoints]
    np.testing.assert_allclose(positions[0], [0.0, -1.725, 0.0], atol=1e-3)
    np.testing.assert_allclose(positions[-1], [32.925, 18.45, 3.75], atol=1e-3)
