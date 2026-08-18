"""Lifecycle adapter for the adaptive-v3 SDK-local Gate 16 policy.

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


GATE16_ARMED_MODES = frozenset({"climb_ready", "climb", "verify_clear"})
def gate16_should_own(policy, mode: str, *, prewarm_ready: bool = False) -> bool:
    """Own only after the router proves the complete moving-entry envelope.

    The official follower first proves strict staging alignment. The unarmed Gate16 base
    then owns only the short, measured runway needed to make its action history causal;
    residual remains forbidden until the complete moving-entry envelope is proved.
    """
    return bool(
        getattr(policy, "is_gate16_policy", False)
        and (
            str(mode) in GATE16_ARMED_MODES
            or (str(mode) == "align" and prewarm_ready)
        )
    )


def gate16_owner_request(policy, mode: str, *, prewarm_ready: bool = False) -> str:
    """Return the two-phase SDK request for this router mode.

    ``gate16`` runs the base on the staging runway with residual disarmed.
    ``gate16_climb`` keeps that owner and arms residual only after the router proves the
    complete moving-entry contract.
    """
    if getattr(policy, "is_gate16_policy", False) and str(mode) in {
        "approach",
        "align",
    } and not prewarm_ready:
        # Build a causal last_action history without taking actuator ownership.
        return "gate16_shadow"
    if not gate16_should_own(policy, mode, prewarm_ready=prewarm_ready):
        return "official"
    return "gate16_climb" if str(mode) in GATE16_ARMED_MODES else "gate16"


@dataclass
class Gate16Config:
    command_forward: float = 0.25
    command_lateral: float = 0.0
    command_yaw_rate: float = 0.0


class StableGate16Policy:
    """Remote handle for the frozen-checkpoint, adaptive-v3 Gate16 actor."""

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
        self._reason = "Gate16 adaptive-v3 actor requested (source b824f7f)"

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
            info={"owner": self.owner_name, "profile": "adaptive_v3_b824f7f"},
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
                "profile": "adaptive_v3_b824f7f",
            },
        )
