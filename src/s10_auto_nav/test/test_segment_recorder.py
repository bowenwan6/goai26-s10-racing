"""What the segment recorder is allowed to call a success.

The recorder decides, for every run in the archive, whether the segment was passed. It was
deciding that with the follower's ``advance_radius`` of 0.35 m while the contest scores at
0.2 m, so ``18_19_baseline_seed0`` was filed as ``"reached": true`` with its closest approach
to waypoint 19 at 0.348 m. That is not a rounding difference; it is a miss recorded as a pass,
and every count derived from the archive inherited it.

These tests drive ``summary()`` directly off a constructed recorder rather than through a
simulation, because the thing under test is the arithmetic of the verdict, not the robot.
"""

from __future__ import annotations

from pathlib import Path

import pytest

try:
    import rclpy
except ImportError:  # pragma: no cover - depends on the environment, not the code
    rclpy = None

needs_ros = pytest.mark.skipif(rclpy is None, reason="ROS 2 is not installed here")


def test_the_harness_does_not_stop_outside_the_scoring_radius():
    source = (
        Path(__file__).resolve().parents[1] / "s10_auto_nav" / "segment_recorder.py"
    ).read_text()
    assert 'declare_parameter("reach_radius", 0.18)' in source
    assert 'declare_parameter("score_radius", 0.18)' in source


class _Verdict:
    """The verdict half of the recorder, with the ROS half left out.

    ``summary()`` reads only these attributes, so binding them to a bare object exercises
    the real method without a node, a course file or a running graph.
    """

    def __init__(self, closest: float, outcome: str, score_radius: float = 0.18):
        from s10_auto_nav.segment_recorder import SegmentRecorder

        self.summary = SegmentRecorder.summary.__get__(self)
        self._closest_goal = closest
        self._outcome = outcome
        self.score_radius = score_radius
        self.start, self.end, self.seed = 18, 19, 0
        self._rows: list[dict] = []
        self._dt = 0.05
        self._travelled = 0.0
        self._max_tilt = 0.0
        self._stalls = 0
        self._invalid = False
        self._transitions: list[str] = []
        self._status = ""
        self.out_dir = Path("/tmp")
        self.run_name = "18_19_for_the_test"


@needs_ros
def test_a_run_that_stopped_at_the_advance_radius_did_not_score():
    """The measured case: 0.348 m from waypoint 19, filed as reached, worth nothing."""
    summary = _Verdict(0.348, "reached the end waypoint").summary()
    assert summary["reached"] is False
    assert summary["arrived"] is True, "the harness did stop on purpose; that fact is kept"
    assert summary["closest_goal_m"] == pytest.approx(0.348)


@needs_ros
def test_a_run_that_passed_through_the_gate_scores():
    summary = _Verdict(0.18, "reached the end waypoint").summary()
    assert summary["reached"] is True


@needs_ros
def test_a_run_beyond_the_internal_radius_is_rejected():
    summary = _Verdict(0.181, "reached the end waypoint").summary()
    assert summary["reached"] is False


@needs_ros
def test_passing_through_the_gate_scores_even_if_the_run_then_failed():
    """Closest approach, not final distance: crossing the gate is what the scorer sees."""
    summary = _Verdict(0.05, "fell over (tilt 80 deg)").summary()
    assert summary["reached"] is True
    assert summary["arrived"] is False


@needs_ros
def test_an_invalid_spawn_is_neither_a_pass_nor_a_failure():
    verdict = _Verdict(4.2, "invalid spawn: never stood up at the start pose")
    verdict._invalid = True
    summary = verdict.summary()
    assert summary["valid"] is False
    assert summary["reached"] is False
