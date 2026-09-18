"""Geometry and sampling helpers for the Gate 16 arrival-state experiment."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class ObstacleFrame:
    obstacle_id: str
    edge_center: np.ndarray
    normal: np.ndarray
    tangent: np.ndarray

    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> ObstacleFrame:
        normal = np.asarray(config["normal"], dtype=float)
        tangent = np.asarray(config["tangent"], dtype=float)
        if normal.shape != (2,) or tangent.shape != (2,):
            raise ValueError("obstacle normal and tangent must each have two elements")
        if not math.isclose(float(np.linalg.norm(normal)), 1.0, abs_tol=1e-6):
            raise ValueError("obstacle normal must be a unit vector")
        if not math.isclose(float(np.dot(normal, tangent)), 0.0, abs_tol=1e-6):
            raise ValueError("obstacle normal and tangent must be orthogonal")
        return cls(
            obstacle_id=str(config["id"]),
            edge_center=np.asarray(config["edge_center"], dtype=float),
            normal=normal,
            tangent=tangent,
        )

    @property
    def normal_yaw(self) -> float:
        return math.atan2(float(self.normal[1]), float(self.normal[0]))

    def measure(self, position_xy, yaw: float, velocity_xy) -> dict[str, float]:
        position = np.asarray(position_xy, dtype=float)
        velocity = np.asarray(velocity_xy, dtype=float)
        delta = self.edge_center[:2] - position
        return {
            "obstacle_distance": float(np.dot(delta, self.normal)),
            "lateral_error": float(np.dot(position - self.edge_center[:2], self.tangent)),
            "heading_error": wrap_to_pi(float(yaw) - self.normal_yaw),
            "forward_speed": float(np.dot(velocity, self.normal)),
            "lateral_speed": float(np.dot(velocity, self.tangent)),
        }


@dataclass(frozen=True)
class EntryGateConfig:
    distance_min: float = 0.45
    distance_max: float = 0.70
    max_heading: float = math.radians(6.0)
    max_lateral: float = 0.08
    max_speed: float = 0.05
    max_yaw_rate: float = 0.10
    max_tilt_deg: float = 12.0
    dwell: float = 0.40


class EntryGate:
    """Tracks continuous gate eligibility; a single bad sample resets the dwell."""

    def __init__(self, config: EntryGateConfig | None = None) -> None:
        self.config = config or EntryGateConfig()
        self._since: float | None = None

    def conditions(self, row: Mapping[str, Any]) -> dict[str, bool]:
        c = self.config
        distance = float(row["obstacle_distance"])
        speed = math.hypot(float(row["vx"]), float(row["vy"]))
        return {
            "distance": c.distance_min <= distance <= c.distance_max,
            "heading": abs(float(row["heading_error"])) <= c.max_heading,
            "lateral": abs(float(row["lateral_error"])) <= c.max_lateral,
            "speed": speed <= c.max_speed,
            "yaw_rate": abs(float(row["yaw_rate"])) <= c.max_yaw_rate,
            "tilt": float(row["tilt_deg"]) <= c.max_tilt_deg,
        }

    def update(self, t: float, row: Mapping[str, Any]) -> tuple[bool, dict[str, bool]]:
        checks = self.conditions(row)
        if all(checks.values()):
            if self._since is None:
                self._since = t
            return t - self._since + 1e-9 >= self.config.dwell, checks
        self._since = None
        return False, checks


def interpolate_row(
    before: Mapping[str, Any], after: Mapping[str, Any], level: float
) -> dict[str, Any]:
    """Interpolate one crossing while keeping labels from the nearest sample."""
    d0 = float(before["obstacle_distance"])
    d1 = float(after["obstacle_distance"])
    fraction = 0.0 if d0 == d1 else (d0 - level) / (d0 - d1)
    fraction = min(max(fraction, 0.0), 1.0)
    result: dict[str, Any] = {}
    for key in before.keys() | after.keys():
        a = before.get(key)
        b = after.get(key)
        if isinstance(a, int | float | np.integer | np.floating) and isinstance(
            b, int | float | np.integer | np.floating
        ):
            av, bv = float(a), float(b)
            result[key] = (
                av + fraction * (bv - av) if math.isfinite(av) and math.isfinite(bv) else math.nan
            )
        else:
            result[key] = a if fraction < 0.5 else b
    result["obstacle_distance"] = float(level)
    return result
