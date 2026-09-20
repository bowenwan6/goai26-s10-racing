"""Stateful height-map gate for the S10 climbing residual.

The locomotion actor remains in charge on flat terrain.  The residual actor is only
allowed to modify its actions after a sufficiently tall upward edge is visible in the
forward centre corridor.  Hysteresis keeps the skill active while the edge moves under
and behind the robot, which is essential for getting the rear wheels onto the platform.

This Python implementation is the training/evaluation reference for the C++ deployment
gate in ``integration/s10_skill_gate.hpp``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from .observation import HEIGHTMAP_COLS, HEIGHTMAP_ROWS, HEIGHTMAP_VOID_SENTINEL


@dataclass(frozen=True)
class SkillGateConfig:
    """Parameters shared by simulation and deployment (50 Hz policy rate)."""

    # Activate for wheel-sized obstacles as well as the 0.377 m contest exit.
    # With the corrected terrain-only height map, 0.04 m is safely above flat
    # ground noise and lets the same residual cover the continuous 5-40 cm range.
    enter_step_height: float = 0.04
    exit_step_height: float = 0.02
    enter_frames: int = 2
    exit_frames: int = 15
    min_active_frames: int = 100
    max_active_frames: int = 600
    first_boundary_row: int = 4
    last_boundary_row: int = 8
    first_retention_row: int = 0
    last_retention_row: int = 8
    first_column: int = 2
    last_column: int = 6

    def validate(self) -> None:
        if not 0.0 < self.exit_step_height < self.enter_step_height:
            raise ValueError("gate thresholds must satisfy 0 < exit < enter")
        if min(self.enter_frames, self.exit_frames, self.min_active_frames) < 1:
            raise ValueError("gate frame counts must be positive")
        if self.max_active_frames < self.min_active_frames:
            raise ValueError("max_active_frames must be >= min_active_frames")
        if not 0 <= self.first_boundary_row <= self.last_boundary_row < HEIGHTMAP_ROWS - 1:
            raise ValueError("gate boundary rows are outside the 13-row height map")
        if not 0 <= self.first_retention_row <= self.last_retention_row < HEIGHTMAP_ROWS - 1:
            raise ValueError("gate retention rows are outside the 13-row height map")
        if not 0 <= self.first_column <= self.last_column < HEIGHTMAP_COLS:
            raise ValueError("gate columns are outside the 9-column height map")


@dataclass(frozen=True)
class SkillGateState:
    active: bool
    max_up_step: float
    retained_up_step: float
    active_frames: int
    valid_edges: int


class HeightmapSkillGate:
    """Detect a tall upward edge and retain the climb skill through rear-wheel exit."""

    def __init__(self, config: SkillGateConfig | None = None) -> None:
        self.config = config or SkillGateConfig()
        self.config.validate()
        self.reset()

    def reset(self) -> None:
        self.active = False
        self._enter_count = 0
        self._exit_count = 0
        self._active_frames = 0

    def _edge_measurement(
        self, flat_heightmap: Sequence[float], first_row: int, last_row: int
    ) -> tuple[float, int]:
        values = [float(value) for value in flat_heightmap]
        expected = HEIGHTMAP_ROWS * HEIGHTMAP_COLS
        if len(values) != expected:
            raise ValueError(f"expected {expected} height cells, got {len(values)}")

        # Policy convention is base_z - terrain_z - 0.5.  Therefore an upward
        # terrain edge appears as a decrease from the nearer row to the farther row.
        maximum = 0.0
        valid_edges = 0
        cfg = self.config
        for row in range(first_row, last_row + 1):
            for column in range(cfg.first_column, cfg.last_column + 1):
                near = values[row * HEIGHTMAP_COLS + column]
                far = values[(row + 1) * HEIGHTMAP_COLS + column]
                if near <= HEIGHTMAP_VOID_SENTINEL or far <= HEIGHTMAP_VOID_SENTINEL:
                    continue
                valid_edges += 1
                maximum = max(maximum, near - far)
        return maximum, valid_edges

    def update(self, flat_heightmap: Sequence[float]) -> SkillGateState:
        cfg = self.config
        maximum, valid_edges = self._edge_measurement(
            flat_heightmap, cfg.first_boundary_row, cfg.last_boundary_row
        )
        retained, retained_valid_edges = self._edge_measurement(
            flat_heightmap, cfg.first_retention_row, cfg.last_retention_row
        )

        if not self.active:
            self._enter_count = self._enter_count + 1 if (
                valid_edges > 0 and maximum >= cfg.enter_step_height
            ) else 0
            if self._enter_count >= cfg.enter_frames:
                self.active = True
                self._active_frames = 0
                self._exit_count = 0
        else:
            self._active_frames += 1
            if self._active_frames >= cfg.min_active_frames:
                self._exit_count = self._exit_count + 1 if (
                    retained_valid_edges > 0 and retained <= cfg.exit_step_height
                ) else 0
            if (
                self._exit_count >= cfg.exit_frames
                or self._active_frames >= cfg.max_active_frames
            ):
                self.reset()

        return SkillGateState(
            active=self.active,
            max_up_step=maximum,
            retained_up_step=retained,
            active_frames=self._active_frames,
            valid_edges=valid_edges,
        )
