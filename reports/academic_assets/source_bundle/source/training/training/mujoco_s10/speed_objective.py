"""Speed-first Gate-16 reward and deterministic completion-time metrics."""

from __future__ import annotations

from dataclasses import dataclass
import math
from statistics import mean, pstdev


CONTROL_DT_SECONDS = 0.02
OFFICIAL_MAX_STEPS = 850


@dataclass(frozen=True)
class SpeedObjectiveConfig:
    """Reward terms that care about success and elapsed time, not motion style."""

    forward_progress_scale: float = 35.0
    front_progress_scale: float = 20.0
    rear_progress_scale: float = 70.0
    com_progress_scale: float = 35.0
    wheel_progress_scale: float = 6.0
    active_step_penalty: float = 0.05
    action_penalty_scale: float = 0.0005
    action_change_penalty_scale: float = 0.0005
    success_bonus: float = 300.0
    success_time_bonus_per_remaining_step: float = 0.50
    fall_penalty: float = 250.0
    divergence_penalty: float = 300.0


def speed_reward(
    *,
    dx: float,
    front_gain: float,
    rear_gain: float,
    com_gain: float,
    wheel_gain: int,
    action_mean_square: float,
    action_change_mean_square: float,
    success: bool,
    fallen: bool,
    diverged: bool,
    step_count: int,
    max_steps: int,
    active: bool,
    time_penalty_active: bool | None = None,
    phase_elapsed_steps: int | None = None,
    config: SpeedObjectiveConfig | None = None,
) -> float:
    cfg = config or SpeedObjectiveConfig()
    reward = 0.0
    if active:
        reward += cfg.forward_progress_scale * dx
        reward += cfg.front_progress_scale * front_gain
        reward += cfg.rear_progress_scale * rear_gain
        reward += cfg.com_progress_scale * com_gain
        reward += cfg.wheel_progress_scale * wheel_gain
        reward -= cfg.action_penalty_scale * action_mean_square
        reward -= cfg.action_change_penalty_scale * action_change_mean_square
    if active if time_penalty_active is None else time_penalty_active:
        reward -= cfg.active_step_penalty
    if success:
        completion_steps = (
            step_count if phase_elapsed_steps is None else phase_elapsed_steps
        )
        reward += cfg.success_bonus
        reward += cfg.success_time_bonus_per_remaining_step * max(
            0, max_steps - completion_steps
        )
    if fallen:
        reward -= cfg.fall_penalty
    if diverged:
        reward -= cfg.divergence_penalty
    return float(reward)


def summarize_speed(
    rows: list[dict[str, object]],
    *,
    failure_steps: int = OFFICIAL_MAX_STEPS,
) -> dict[str, float | int | None]:
    """Summarize only successful completion time without rewarding fast failures."""

    if not rows:
        raise ValueError("speed summary needs at least one row")
    successful_steps = [
        int(row["steps"]) for row in rows if bool(row.get("success", False))
    ]
    penalized_steps = [
        int(row["steps"])
        if bool(row.get("success", False))
        else int(failure_steps)
        for row in rows
    ]
    if successful_steps:
        mean_steps = float(mean(successful_steps))
        std_steps = float(pstdev(successful_steps))
        fastest_steps = min(successful_steps)
        slowest_steps = max(successful_steps)
    else:
        mean_steps = std_steps = None
        fastest_steps = slowest_steps = None
    return {
        "successful_completion_samples": len(successful_steps),
        "mean_success_steps": mean_steps,
        "std_success_steps": std_steps,
        "fastest_success_steps": fastest_steps,
        "slowest_success_steps": slowest_steps,
        "mean_penalized_steps": float(mean(penalized_steps)),
        "mean_success_seconds": (
            None if mean_steps is None else mean_steps * CONTROL_DT_SECONDS
        ),
        "std_success_seconds": (
            None if std_steps is None else std_steps * CONTROL_DT_SECONDS
        ),
        "fastest_success_seconds": (
            None
            if fastest_steps is None
            else fastest_steps * CONTROL_DT_SECONDS
        ),
        "slowest_success_seconds": (
            None
            if slowest_steps is None
            else slowest_steps * CONTROL_DT_SECONDS
        ),
        "mean_penalized_seconds": float(mean(penalized_steps))
        * CONTROL_DT_SECONDS,
    }


def finite_or(value: float | int | None, fallback: float) -> float:
    if value is None:
        return fallback
    number = float(value)
    return number if math.isfinite(number) else fallback
