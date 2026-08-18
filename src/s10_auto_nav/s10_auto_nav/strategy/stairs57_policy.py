"""Router lifecycle adapter for the SDK-local 57D continuous-stair actor.

Inference and action decoding stay inside ``rl_deploy`` so this policy consumes the exact
official 57D observation.  The Python side owns only entry, command conditioning, physical
clearance and handoff decisions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from s10_auto_nav.strategy.policy import (
    ActionKind,
    PolicyAction,
    PolicyObservation,
    PolicyResult,
    PolicyStatus,
)


@dataclass(frozen=True)
class Stairs57Config:
    command_forward: float = 0.35
    command_lateral: float = 0.0
    command_yaw_rate: float = 0.0
    navigation_lateral_limit: float = 0.0
    navigation_yaw_rate_limit: float = 0.0
    navigation_lookahead: float = 0.8
    entry_speed_min: float = 0.25
    entry_speed_max: float = 0.45
    completion_min_progress: float = 0.60
    completion_wheel_clearance: float = 0.04
    completion_base_clearance: float = 0.15
    completion_max_tilt_deg: float = 15.0
    completion_min_contacts: int = 3
    completion_hold: float = 0.25

    def __post_init__(self) -> None:
        if not self.entry_speed_min <= self.command_forward <= self.entry_speed_max:
            raise ValueError("stairs57 command must remain inside its trained speed range")
        if self.navigation_lateral_limit < 0.0 or self.navigation_yaw_rate_limit < 0.0:
            raise ValueError("stairs57 navigation correction limits must be non-negative")
        if self.navigation_lookahead <= 0.0:
            raise ValueError("stairs57 navigation lookahead must be positive")


class Stairs57Policy:
    """Remote handle for ``s10_stairs_up_57d_model1800``."""

    action_kind = ActionKind.DELEGATED
    owner_name = "stairs57"
    is_stairs57_policy = True
    requires_moving_entry = True
    requires_physical_clear = False
    owns_entire_segment = True
    handoff_requires_strict_target = True
    start_from_rest = True
    climb_timeout = 90.0
    climb_progress_window = 10.0

    def __init__(self, config: Stairs57Config | None = None):
        self.config = config or Stairs57Config()
        self.command_forward = self.config.command_forward
        self.entry_speed_min = self.config.entry_speed_min
        self.entry_speed_max = self.config.entry_speed_max
        self.reset_count = 0
        self.reset()

    def reset(self) -> None:
        self.reset_count += 1
        self._status = PolicyStatus.IDLE
        self._started_at: float | None = None
        self._last_t = 0.0
        self._reason = ""
        self._clear_for = 0.0

    def continue_segment(self) -> None:
        """Keep the learned action history but verify the next upper platform afresh."""
        self._clear_for = 0.0

    def start(self, observation: PolicyObservation) -> None:
        self._started_at = float(observation.t)
        self._last_t = self._started_at
        self._status = PolicyStatus.RUNNING
        self._reason = "stairs57 model1800 actor requested"

    def step(self, observation: PolicyObservation) -> PolicyAction:
        self._last_t = float(observation.t)
        return PolicyAction(
            ActionKind.DELEGATED,
            self._status,
            twist=(
                self.config.command_forward,
                self.config.command_lateral,
                self.config.command_yaw_rate,
            ),
            info={
                "owner": self.owner_name,
                "runtime": "s10_stairs_up_57d_model1800",
                "command_forward_mps": self.config.command_forward,
            },
        )

    def completion_candidate(self, state, dt: float, entry_position) -> tuple[bool, str]:
        """Confirm that the chassis, not merely its front axle, reached the top deck."""
        target_z = getattr(state, "segment_target_z", None)
        if target_z is None or not math.isfinite(float(target_z)):
            self._clear_for = 0.0
            return False, "target platform height unavailable"

        entry = None if entry_position is None else np.asarray(entry_position, dtype=float)
        position = np.asarray(state.position, dtype=float)
        progress = (
            0.0
            if entry is None or entry.shape != (3,)
            else float(np.linalg.norm(position[:2] - entry[:2]))
        )
        enough_progress = progress >= self.config.completion_min_progress
        stable = state.tilt <= math.radians(self.config.completion_max_tilt_deg)

        wheels = getattr(state, "wheel_positions", None)
        contacts = getattr(state, "wheel_contacts", None)
        if wheels is not None and contacts is not None:
            wheels = np.asarray(wheels, dtype=float)
            contacts = np.asarray(contacts, dtype=bool)
            valid = (
                wheels.shape == (4, 3)
                and contacts.shape == (4,)
                and np.all(np.isfinite(wheels))
            )
            wheel_threshold = float(target_z) + self.config.completion_wheel_clearance
            upper = valid and bool(np.all(wheels[:, 2] >= wheel_threshold))
            supported = valid and int(np.sum(contacts)) >= self.config.completion_min_contacts
            clear = enough_progress and stable and upper and supported
            why = (
                f"4/4 wheels above z={wheel_threshold:.2f}m, "
                f"{int(np.sum(contacts)) if valid else 0}/4 contacts, "
                f"progress={progress:.2f}m"
            )
        else:
            # Hardware may not publish simulated wheel contact poses. The odometry fallback
            # is deliberately stricter in height and still requires stable attitude and
            # sustained forward progress before it can release actuator ownership.
            base_threshold = float(target_z) + self.config.completion_base_clearance
            clear = enough_progress and stable and position[2] >= base_threshold
            why = (
                f"base above z={base_threshold:.2f}m with stable attitude, "
                f"progress={progress:.2f}m"
            )

        self._clear_for = self._clear_for + dt if clear else 0.0
        return self._clear_for >= self.config.completion_hold, why

    def succeed(self, reason: str = "four wheels verified above the stair edge") -> None:
        self._status = PolicyStatus.SUCCEEDED
        self._reason = reason

    def fail(self, reason: str) -> None:
        self._status = PolicyStatus.FAILED
        self._reason = reason

    def cancel(self) -> None:
        if self._status is PolicyStatus.RUNNING:
            self._status = PolicyStatus.CANCELLED
            self._reason = "cancelled by router"

    def is_finished(self) -> bool:
        return self._status.terminal

    def result(self) -> PolicyResult:
        elapsed = 0.0 if self._started_at is None else self._last_t - self._started_at
        return PolicyResult(
            self._status,
            self._reason,
            elapsed,
            {
                "checkpoint": "s10_stairs_up_57d_model1800",
                "observation_dim": 57,
                "action_dim": 16,
            },
        )
