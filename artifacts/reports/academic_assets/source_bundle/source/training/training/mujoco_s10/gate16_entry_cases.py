"""Deterministic Gate-16 entry cases from synthetic grids or navigation logs."""

from __future__ import annotations

import csv
import math
from pathlib import Path


GATE16_ROUTE_INTERSECTION_Y = 32.49969


def entry_cases(
    distances: list[float],
    laterals: list[float],
    yaw_degrees: list[float],
    speeds: list[float],
) -> list[dict[str, float | str | None]]:
    """Build the legacy Cartesian pose grid with a stationary initial state."""
    return [
        {
            "source_run": "synthetic_grid",
            "distance": float(distance),
            "lateral": float(lateral),
            "yaw_deg": float(yaw_deg),
            "yaw_rad": math.radians(float(yaw_deg)),
            "speed": float(speed),
            "cmd_lateral": 0.0,
            # None preserves the legacy heading-feedback command in the environment.
            "cmd_yaw": None,
            "forward_speed": 0.0,
            "lateral_speed": 0.0,
            "yaw_rate": 0.0,
            "entry_center_y": 33.365,
        }
        for speed in speeds
        for distance in distances
        for lateral in laterals
        for yaw_deg in yaw_degrees
    ]


def load_navigation_cases(path: Path) -> list[dict[str, float | str | None]]:
    """Load measured navigation arrivals into the simulator entry contract.

    The input columns are the compact subset exported by Bowen's WP15->WP16 handoff.
    Body-frame forward/lateral speeds and yaw rate initialize the dynamic state; the
    command columns remain the observation supplied to the frozen locomotion actor.
    """
    required = {
        "run_id",
        "obstacle_distance_m",
        "heading_error_deg",
        "lateral_error_m",
        "cmd_forward_mps",
        "cmd_lateral",
        "cmd_yaw",
        "forward_speed_mps",
        "lateral_speed_mps",
        "yaw_rate_dps",
    }
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(f"navigation case CSV is missing columns: {sorted(missing)}")
        cases = []
        for row in reader:
            yaw_deg = float(row["heading_error_deg"])
            cases.append(
                {
                    "source_run": row["run_id"],
                    "distance": float(row["obstacle_distance_m"]),
                    "lateral": float(row["lateral_error_m"]),
                    "yaw_deg": yaw_deg,
                    "yaw_rad": math.radians(yaw_deg),
                    "speed": float(row["cmd_forward_mps"]),
                    "cmd_lateral": float(row["cmd_lateral"]),
                    "cmd_yaw": float(row["cmd_yaw"]),
                    "forward_speed": float(row["forward_speed_mps"]),
                    "lateral_speed": float(row["lateral_speed_mps"]),
                    "yaw_rate": math.radians(float(row["yaw_rate_dps"])),
                    "entry_center_y": GATE16_ROUTE_INTERSECTION_Y,
                }
            )
    if not cases:
        raise ValueError("navigation case CSV contains no rows")
    return cases
