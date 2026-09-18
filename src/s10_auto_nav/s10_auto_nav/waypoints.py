"""Waypoint course loading.

The course is stored as YAML rather than parsed from the MJCF at runtime, so a run is
reproducible even if the scene is edited. ``scripts/extract_waypoints.py`` regenerates the
YAML from the upstream track overlay.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class Waypoint:
    index: int
    position: np.ndarray  # (3,) world frame

    @property
    def xy(self) -> np.ndarray:
        return self.position[:2]


class Course:
    """An ordered waypoint course with progress tracking.

    Progress is strictly sequential, mirroring the contest scorer: waypoint ``i + 1``
    only becomes the target once ``i`` has been reached.

    What counts as reached is the part that had to be rewritten. The official scorer's radius
    is 0.2 m; this class now uses a stricter 0.18 m internal radius. It once advanced at 0.35 m,
    on the reasoning that committing to the next leg
    early avoids braking into every gate. On a straight leg the robot carries on through
    and scores anyway, which is why it survived so long. On a corner it does not: the
    carrot swings onto the next leg the instant the cursor moves, and the robot turns away
    from a gate it was still 0.34 m from. Scored offline against the continuous waypoint 16
    to 32 run, that cost gates 17 (closest 0.338 m), 21 (0.320 m) and 22 (0.336 m) out of a
    run that was otherwise navigating them correctly -- three of sixteen, thrown away by the
    bookkeeping rather than by the driving.

    The cursor now moves only when the current gate is inside ``score_radius``. There is no
    receding/closest-point or ``advance_radius`` fallback: missing a strict gate must hold the
    cursor instead of invalidating every later gate. Both shipped radii are 0.18 m, but only
    the score radius is allowed to consume a waypoint.
    """

    def __init__(
        self,
        waypoints: list[Waypoint],
        advance_radius: float = 0.18,
        score_radius: float = 0.18,
        height_tolerance: float | None = None,
    ) -> None:
        if not waypoints:
            raise ValueError("Course requires at least one waypoint")
        self.waypoints = waypoints
        self.advance_radius = float(advance_radius)
        self.score_radius = float(score_radius)
        if height_tolerance is not None and (
            not math.isfinite(height_tolerance) or height_tolerance <= 0
        ):
            raise ValueError("height_tolerance must be finite and positive")
        self.height_tolerance = height_tolerance
        self._cursor = 0

    @classmethod
    def from_yaml(cls, path: str | Path, **kwargs) -> Course:
        data = yaml.safe_load(Path(path).read_text())
        entries = data["waypoints"] if isinstance(data, dict) else data
        waypoints = [
            Waypoint(
                index=int(entry.get("index", i)),
                position=np.asarray(entry["position"], float),
            )
            for i, entry in enumerate(entries)
        ]
        return cls(waypoints, **kwargs)

    def __len__(self) -> int:
        return len(self.waypoints)

    @property
    def cursor(self) -> int:
        return self._cursor

    @property
    def finished(self) -> bool:
        return self._cursor >= len(self.waypoints)

    @property
    def target(self) -> Waypoint | None:
        return None if self.finished else self.waypoints[self._cursor]

    def update(self, position_xy: np.ndarray) -> bool:
        """Advance the cursor if the current target has been reached.

        Returns ``True`` if the cursor moved. Only one waypoint is consumed per call, so a
        course cannot be skipped through by a single large position jump.
        """
        if self.finished:
            return False
        position = np.asarray(position_xy, dtype=float)
        if position.shape not in ((2,), (3,)) or not np.isfinite(position).all():
            return False
        if self.height_tolerance is not None and (
            position.shape != (3,)
            or abs(position[2] - self.waypoints[self._cursor].position[2]) > self.height_tolerance
        ):
            return False
        distance = float(np.linalg.norm(self.waypoints[self._cursor].xy - position[:2]))
        if distance <= self.score_radius:
            self._cursor += 1
            return True
        return False

    def lookahead_point(self, position_xy: np.ndarray, distance: float) -> np.ndarray:
        """Return the point ``distance`` metres ahead that the follower should steer at.

        The carrot rides the line from the robot to the current gate and stops there; it
        never runs onto the next leg. Both looser rules were tried on the real course and
        both deadlock:

        * Advancing a fixed *arc length* along the whole remaining polyline folds the
          carrot back on top of the robot at a switchback. Standing 0.69 m past gate 2,
          the follower spent its 1.4 m walking back to the gate and out the far side, so
          the carrot sat 0.12 m away, the speed schedule read that as an arrival and
          braked to 0.06 m/s, and the run never finished.
        * Taking the point where a circle of radius ``distance`` leaves the polyline fixes
          the fold-back but rounds corners early: once the gate is closer than the radius
          the carrot is already on the next leg. Approaching gate 1 that pulled the robot
          2.1 m west of it, past the 0.35 m advance radius, and it never recovered.

        Cutting the corner is worth real lap time and is worth revisiting, but only with
        the cut bounded below the scorer's 0.2 m radius. Unbounded, it loses the gate.
        """
        if self.finished:
            return self.waypoints[-1].xy

        centre = np.asarray(position_xy, float)
        gate = self.waypoints[self._cursor].xy

        # Inside the radius there is no crossing: aim at the gate itself and let the
        # caller brake into it.
        exit_point = _segment_circle_exit(centre, distance, centre, gate)
        return gate if exit_point is None else exit_point

    def remaining_distance(self, position_xy: np.ndarray) -> float:
        """Straight-line course length still to be covered, for logging and pacing."""
        if self.finished:
            return 0.0
        total = float(np.linalg.norm(self.waypoints[self._cursor].xy - position_xy))
        for a, b in zip(
            self.waypoints[self._cursor :], self.waypoints[self._cursor + 1 :], strict=False
        ):
            total += float(np.linalg.norm(b.xy - a.xy))
        return total

    def reset(self) -> None:
        self._cursor = 0


def _segment_circle_exit(
    centre: np.ndarray, radius: float, start: np.ndarray, end: np.ndarray
) -> np.ndarray | None:
    """Point where segment ``start`` -> ``end`` last crosses a circle, or ``None``.

    The far root is taken so a segment that clips through the circle and continues does
    not stop the search early; the caller wants the point at which the course leaves the
    robot's lookahead radius for good.
    """
    d = np.asarray(end, float) - np.asarray(start, float)
    f = np.asarray(start, float) - np.asarray(centre, float)

    a = float(d @ d)
    if a < 1e-12:
        return None

    b = 2.0 * float(f @ d)
    c = float(f @ f) - radius * radius
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0.0:
        return None

    root = math.sqrt(discriminant)
    for t in ((-b + root) / (2.0 * a), (-b - root) / (2.0 * a)):
        if 0.0 <= t <= 1.0:
            return np.asarray(start, float) + t * d
    return None
