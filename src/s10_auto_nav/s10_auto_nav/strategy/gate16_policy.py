"""Lifecycle adapter for the competition-v4 SDK-local Gate 16 policy.

The ONNX graphs and actuator decoding live in ``rl_deploy`` so they consume the calibrated
``RobotBasicState`` and produce wheel velocity targets without a ROS actuator round trip.
This adapter deliberately contains no inference. It gives the router the ordinary policy
lifecycle while carrying the command that the low-level actor must observe.
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


@dataclass
class Gate16Config:
    command_forward: float = 0.25
    command_lateral: float = 0.0
    command_yaw_rate: float = 0.0


class StableGate16Policy:
    """Remote handle for the frozen-checkpoint, competition-v4 Gate16 actor."""

    action_kind = ActionKind.DELEGATED
    owner_name = "gate16"
    is_gate16_policy = True

    def __init__(self, config: Gate16Config | None = None):
        self.config = config or Gate16Config()
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
        self._reason = "Gate16 competition-v4 actor requested (source b6535a4)"

    def step(self, observation: PolicyObservation) -> PolicyAction:
        self._last_t = float(observation.t)
        return PolicyAction(
            ActionKind.DELEGATED,
            self._status,
            twist=(
                float(self.config.command_forward),
                float(self.config.command_lateral),
                float(self.config.command_yaw_rate),
            ),
            info={"owner": self.owner_name, "profile": "competition_v4_b6535a4"},
        )

    def succeed(self, reason: str = "four wheels verified on upper platform") -> None:
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
                "stable_gate16_checkpoint": True,
                "profile": "competition_v4_b6535a4",
            },
        )
