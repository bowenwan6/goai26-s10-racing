"""Waypoint course loading.

The course is stored as YAML rather than parsed from the MJCF at runtime, so a run is
reproducible even if the scene is edited. ``scripts/extract_waypoints.py`` regenerates the
YAML from the upstream track overlay.
"""

from __future__ import annotations

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
    only becomes the target once ``i`` has been reached. The scorer uses a 0.2 m
    horizontal radius; we advance on a slightly larger radius so the follower commits to
    the next leg before the scorer's check fires, which avoids braking at every gate.
    """

    def __init__(self, waypoints: list[Waypoint], advance_radius: float = 0.35) -> None:
        if not waypoints:
            raise ValueError("Course requires at least one waypoint")
        self.waypoints = waypoints
        self.advance_radius = float(advance_radius)
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
        distance = float(np.linalg.norm(self.waypoints[self._cursor].xy - position_xy))
        if distance <= self.advance_radius:
            self._cursor += 1
            return True
        return False

    def lookahead_point(self, position_xy: np.ndarray, distance: float) -> np.ndarray:
        """Return a carrot point ``distance`` metres along the remaining course.

        Walking forward along the polyline rather than simply targeting the next waypoint
        lets the follower cut corners on tight gates instead of stopping to pivot at each
        one, which is where most of the lap time is otherwise lost.
        """
        if self.finished:
            return self.waypoints[-1].xy

        remaining = distance
        cursor = self.waypoints[self._cursor].xy
        origin = position_xy

        for wp in self.waypoints[self._cursor :]:
            leg = wp.xy - origin
            leg_len = float(np.linalg.norm(leg))
            if leg_len >= remaining:
                return origin + leg * (remaining / max(leg_len, 1e-6))
            remaining -= leg_len
            origin = wp.xy
            cursor = wp.xy

        return cursor

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
