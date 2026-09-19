"""Front-wheel retention state, reward shaping, and evaluation metrics.

The strict task success remains four-wheel clearance.  This module only makes the
failure mode between first front support and rear-wheel completion observable: losing
one or both front wheels after first support, rocking the base, and spending too long
waiting for the rear wheels.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FrontRetentionConfig:
    retained_front_reward: float = 0.08
    rear_follow_step_penalty: float = 0.03
    balance_penalty_scale: float = 0.50
    clearance_regression_scale: float = 25.0
    front_drop_penalty: float = 12.0
    rear_completed_bonus: float = 2.0


@dataclass(frozen=True)
class FrontRetentionSample:
    step: int
    front_supported: int
    rear_supported: int
    front_min_clearance_m: float
    balance_error: float

    def __post_init__(self) -> None:
        if self.step < 0:
            raise ValueError("step must be non-negative")
        if self.front_supported not in (0, 1, 2):
            raise ValueError("front_supported must be 0, 1, or 2")
        if self.rear_supported not in (0, 1, 2):
            raise ValueError("rear_supported must be 0, 1, or 2")


@dataclass(frozen=True)
class FrontRetentionMetrics:
    sample: FrontRetentionSample
    first_front_step: int | None
    first_rear_step: int | None
    front_to_rear_steps: int | None
    front_drop_count: int
    front_support_steps: int
    front_acquired_event: bool
    front_drop_event: bool
    rear_completed_event: bool
    phase_active: bool
    drop_free: bool
    clearance_regression_m: float


class FrontRetentionTracker:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.first_front_step: int | None = None
        self.first_rear_step: int | None = None
        self.front_drop_count = 0
        self.front_support_steps = 0
        self._previous_front_supported = 0
        self._previous_clearance: float | None = None

    def update(self, sample: FrontRetentionSample) -> FrontRetentionMetrics:
        front_acquired_event = (
            self.first_front_step is None and sample.front_supported == 2
        )
        if front_acquired_event:
            self.first_front_step = sample.step

        front_drop_event = bool(
            self.first_front_step is not None
            and self._previous_front_supported == 2
            and sample.front_supported < 2
            and self.first_rear_step is None
        )
        if front_drop_event:
            self.front_drop_count += 1

        rear_completed_event = bool(
            self.first_front_step is not None
            and self.first_rear_step is None
            and sample.rear_supported == 2
            and sample.front_supported == 2
        )
        if rear_completed_event:
            self.first_rear_step = sample.step

        phase_active = bool(
            self.first_front_step is not None and self.first_rear_step is None
        )
        if (
            self.first_front_step is not None
            and sample.front_supported == 2
            and self.first_rear_step is None
        ):
            self.front_support_steps += 1

        clearance_regression = 0.0
        if (
            self.first_front_step is not None
            and self.first_rear_step is None
            and self._previous_clearance is not None
        ):
            clearance_regression = max(
                0.0, self._previous_clearance - sample.front_min_clearance_m
            )

        self._previous_front_supported = sample.front_supported
        self._previous_clearance = sample.front_min_clearance_m
        latency = (
            None
            if self.first_front_step is None or self.first_rear_step is None
            else self.first_rear_step - self.first_front_step
        )
        return FrontRetentionMetrics(
            sample=sample,
            first_front_step=self.first_front_step,
            first_rear_step=self.first_rear_step,
            front_to_rear_steps=latency,
            front_drop_count=self.front_drop_count,
            front_support_steps=self.front_support_steps,
            front_acquired_event=front_acquired_event,
            front_drop_event=front_drop_event,
            rear_completed_event=rear_completed_event,
            phase_active=phase_active,
            drop_free=self.front_drop_count == 0,
            clearance_regression_m=clearance_regression,
        )


def front_retention_reward(
    metrics: FrontRetentionMetrics,
    config: FrontRetentionConfig | None = None,
) -> float:
    cfg = config or FrontRetentionConfig()
    reward = 0.0
    if metrics.phase_active:
        reward -= cfg.rear_follow_step_penalty
        reward -= cfg.balance_penalty_scale * metrics.sample.balance_error**2
        reward -= cfg.clearance_regression_scale * metrics.clearance_regression_m
        if metrics.sample.front_supported == 2:
            reward += cfg.retained_front_reward
    if metrics.front_drop_event:
        reward -= cfg.front_drop_penalty
    if metrics.rear_completed_event:
        reward += cfg.rear_completed_bonus
    return float(reward)


def summarize_retention(rows: list[dict[str, object]]) -> dict[str, float | int | None]:
    """Aggregate retention fields without changing strict-success semantics."""
    if not rows:
        raise ValueError("retention summary needs at least one row")
    successes = [row for row in rows if bool(row.get("success", False))]
    drop_free_successes = [
        row for row in successes if int(row.get("front_drop_count", 0)) == 0
    ]
    latencies = [
        int(row["front_to_rear_steps"])
        for row in successes
        if row.get("front_to_rear_steps") is not None
    ]
    return {
        "drop_free_successes": len(drop_free_successes),
        "drop_free_success_rate": len(drop_free_successes) / len(rows),
        "front_drop_events": sum(int(row.get("front_drop_count", 0)) for row in rows),
        "mean_front_to_rear_steps": (
            None if not latencies else sum(latencies) / len(latencies)
        ),
    }
