"""Router lifecycle adapter for the SDK-local 57D continuous-stair actor.

Inference and action decoding stay inside ``rl_deploy`` so this policy consumes the exact
official 57D observation.  The Python side owns only entry, command conditioning, physical
clearance and handoff decisions.
"""

from __future__ import annotations

from dataclasses import dataclass

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
    entry_speed_min: float = 0.25
    entry_speed_max: float = 0.45

    def __post_init__(self) -> None:
        if not self.entry_speed_min <= self.command_forward <= self.entry_speed_max:
            raise ValueError("stairs57 command must remain inside its trained speed range")


class Stairs57Policy:
    """Remote handle for ``s10_stairs_up_57d_model1800``."""

    action_kind = ActionKind.DELEGATED
    owner_name = "stairs57"
    is_stairs57_policy = True
    requires_moving_entry = True
    requires_physical_clear = False
    owns_entire_segment = True
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
