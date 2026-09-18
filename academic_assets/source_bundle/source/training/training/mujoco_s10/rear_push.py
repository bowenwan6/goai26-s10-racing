"""Front-limb tuck and rear-push shaping after both front wheels reach the deck."""

from __future__ import annotations

from dataclasses import dataclass
from statistics import mean, pstdev

import numpy as np

from .front_retention import FrontRetentionMetrics
from .speed_objective import CONTROL_DT_SECONDS, OFFICIAL_MAX_STEPS


@dataclass(frozen=True)
class RearPushConfig:
    """Teach one deliberate front-limb tuck followed by a continuous rear push."""

    target_tuck_m: float = 0.03
    settle_window_steps: int = 30
    vertical_motion_deadband_m: float = 0.0005
    tuck_progress_scale: float = 40.0
    early_extension_penalty_scale: float = 15.0
    tuck_target_bonus: float = 2.0
    extra_reversal_penalty: float = 0.25

    def validate(self) -> None:
        if self.target_tuck_m <= 0.0:
            raise ValueError("target_tuck_m must be positive")
        if self.settle_window_steps < 1:
            raise ValueError("settle_window_steps must be positive")
        if self.vertical_motion_deadband_m < 0.0:
            raise ValueError("vertical_motion_deadband_m cannot be negative")
        if min(
            self.tuck_progress_scale,
            self.early_extension_penalty_scale,
            self.tuck_target_bonus,
            self.extra_reversal_penalty,
        ) < 0.0:
            raise ValueError("rear-push reward coefficients cannot be negative")


@dataclass(frozen=True)
class RearPushMetrics:
    phase: str
    phase_steps: int | None
    front_wheel_body_distance_m: float
    front_tuck_m: float
    front_tuck_progress_m: float
    early_front_extension_m: float
    front_tuck_target_reached: bool
    front_tuck_target_event: bool
    tuck_step: int | None
    body_lowering_m: float
    vertical_reversal_event: bool
    vertical_reversal_count: int
    extra_vertical_reversal_event: bool


def front_wheel_sagittal_distance_m(
    base_position: np.ndarray,
    base_rotation: np.ndarray,
    front_wheel_positions: np.ndarray,
) -> float:
    """Mean front-wheel distance to the body in the body-frame x-z plane."""

    base = np.asarray(base_position, dtype=np.float64)
    rotation = np.asarray(base_rotation, dtype=np.float64)
    wheels = np.asarray(front_wheel_positions, dtype=np.float64)
    if base.shape != (3,) or rotation.shape != (3, 3) or wheels.shape != (2, 3):
        raise ValueError("front-tuck geometry expects base (3,), rotation (3,3), wheels (2,3)")
    if not all(np.all(np.isfinite(value)) for value in (base, rotation, wheels)):
        raise ValueError("front-tuck geometry must be finite")
    # MuJoCo xmat maps body-frame column vectors into world coordinates.  With
    # row vectors, multiplying by xmat maps the world delta back into body axes.
    body_relative = (wheels - base) @ rotation
    sagittal = body_relative[:, [0, 2]]
    return float(np.mean(np.linalg.norm(sagittal, axis=1)))


class RearPushTracker:
    def __init__(self, config: RearPushConfig | None = None) -> None:
        self.config = config or RearPushConfig()
        self.config.validate()
        self.reset()

    def reset(self) -> None:
        self.first_front_step: int | None = None
        self.first_front_wheel_distance_m: float | None = None
        self.first_front_height_m: float | None = None
        self.previous_front_wheel_distance_m: float | None = None
        self.previous_height_m: float | None = None
        self.best_front_tuck_m = 0.0
        self.best_body_lowering_m = 0.0
        self.tuck_step: int | None = None
        self.vertical_reversal_count = 0
        self._vertical_direction = 0

    def update(
        self,
        retention: FrontRetentionMetrics,
        *,
        base_height_m: float,
        front_wheel_body_distance_m: float,
    ) -> RearPushMetrics:
        step = retention.sample.step
        if retention.front_acquired_event:
            self.first_front_step = step
            self.first_front_wheel_distance_m = float(front_wheel_body_distance_m)
            self.first_front_height_m = float(base_height_m)
            self.previous_front_wheel_distance_m = float(
                front_wheel_body_distance_m
            )
            self.previous_height_m = float(base_height_m)
            self.best_front_tuck_m = 0.0
            self.best_body_lowering_m = 0.0
            self.tuck_step = None
            self.vertical_reversal_count = 0
            self._vertical_direction = 0

        phase_steps = (
            None if self.first_front_step is None else step - self.first_front_step
        )
        tuck_progress = 0.0
        early_extension = 0.0
        target_event = False
        reversal_event = False
        extra_reversal_event = False

        if self.first_front_wheel_distance_m is not None:
            current_tuck = max(
                0.0,
                self.first_front_wheel_distance_m
                - float(front_wheel_body_distance_m),
            )
            tuck_progress = max(0.0, current_tuck - self.best_front_tuck_m)
            self.best_front_tuck_m = max(self.best_front_tuck_m, current_tuck)
            if (
                self.tuck_step is None
                and self.best_front_tuck_m + 1.0e-9 >= self.config.target_tuck_m
            ):
                self.tuck_step = step
                target_event = True

        if self.first_front_height_m is not None:
            body_lowering = max(
                0.0, self.first_front_height_m - float(base_height_m)
            )
            self.best_body_lowering_m = max(
                self.best_body_lowering_m, body_lowering
            )

        if self.previous_height_m is not None and retention.phase_active:
            delta = float(base_height_m) - self.previous_height_m
            if abs(delta) >= self.config.vertical_motion_deadband_m:
                direction = 1 if delta > 0.0 else -1
                reversal_event = bool(
                    self._vertical_direction and direction != self._vertical_direction
                )
                if reversal_event:
                    self.vertical_reversal_count += 1
                    # Lowering once and then rising to finish is intentional.  Only
                    # subsequent direction changes represent wasted rocking.
                    extra_reversal_event = self.vertical_reversal_count > 1
                self._vertical_direction = direction
        if (
            self.previous_front_wheel_distance_m is not None
            and retention.phase_active
            and self.tuck_step is None
            and phase_steps is not None
            and phase_steps < self.config.settle_window_steps
        ):
            early_extension = max(
                0.0,
                float(front_wheel_body_distance_m)
                - self.previous_front_wheel_distance_m,
            )
        self.previous_front_wheel_distance_m = float(front_wheel_body_distance_m)
        self.previous_height_m = float(base_height_m)

        if self.first_front_step is None:
            phase = "approach"
        elif retention.rear_completed_event or retention.first_rear_step is not None:
            phase = "complete"
        elif (
            self.tuck_step is None
            and phase_steps is not None
            and phase_steps < self.config.settle_window_steps
        ):
            phase = "settle"
        else:
            phase = "push"

        return RearPushMetrics(
            phase=phase,
            phase_steps=phase_steps,
            front_wheel_body_distance_m=float(front_wheel_body_distance_m),
            front_tuck_m=self.best_front_tuck_m,
            front_tuck_progress_m=tuck_progress,
            early_front_extension_m=early_extension,
            front_tuck_target_reached=self.tuck_step is not None,
            front_tuck_target_event=target_event,
            tuck_step=self.tuck_step,
            body_lowering_m=self.best_body_lowering_m,
            vertical_reversal_event=reversal_event,
            vertical_reversal_count=self.vertical_reversal_count,
            extra_vertical_reversal_event=extra_reversal_event,
        )


def rear_push_reward(
    metrics: RearPushMetrics,
    config: RearPushConfig | None = None,
) -> float:
    cfg = config or RearPushConfig()
    reward = 0.0
    if metrics.phase == "settle":
        reward += cfg.tuck_progress_scale * metrics.front_tuck_progress_m
        reward -= (
            cfg.early_extension_penalty_scale * metrics.early_front_extension_m
        )
    if metrics.front_tuck_target_event:
        reward += cfg.tuck_target_bonus
    if metrics.extra_vertical_reversal_event:
        reward -= cfg.extra_reversal_penalty
    return float(reward)


def summarize_rear_push(
    rows: list[dict[str, object]],
    *,
    failure_steps: int = OFFICIAL_MAX_STEPS,
) -> dict[str, float | int | None]:
    if not rows:
        raise ValueError("rear-push summary needs at least one row")
    successes = [row for row in rows if bool(row.get("success", False))]
    latencies = [
        int(row["front_to_rear_steps"])
        for row in successes
        if row.get("front_to_rear_steps") is not None
    ]
    penalized = [
        int(row["front_to_rear_steps"])
        if bool(row.get("success", False))
        and row.get("front_to_rear_steps") is not None
        else int(failure_steps)
        for row in rows
    ]
    tucks = [
        float(row["front_tuck_m"])
        for row in successes
        if row.get("front_tuck_m") is not None
    ]
    body_lowering = [
        float(row["body_lowering_m"])
        for row in successes
        if row.get("body_lowering_m") is not None
    ]
    reversals = [
        int(row["rear_push_vertical_reversals"])
        for row in successes
        if row.get("rear_push_vertical_reversals") is not None
    ]
    target_hits = sum(
        int(bool(row.get("front_tuck_target_reached", False)))
        for row in successes
    )
    climb_latencies = [
        int(row["climb_start_to_clear_steps"])
        for row in successes
        if row.get("climb_start_to_clear_steps") is not None
    ]
    return {
        "rear_push_samples": len(latencies),
        "mean_front_to_rear_steps": None if not latencies else float(mean(latencies)),
        "std_front_to_rear_steps": None if not latencies else float(pstdev(latencies)),
        "fastest_front_to_rear_steps": None if not latencies else min(latencies),
        "mean_penalized_front_to_rear_steps": float(mean(penalized)),
        "mean_penalized_front_to_rear_seconds": float(mean(penalized))
        * CONTROL_DT_SECONDS,
        "mean_front_tuck_m": None if not tucks else float(mean(tucks)),
        "front_tuck_target_successes": target_hits,
        "front_tuck_target_rate": target_hits / len(rows),
        "mean_body_lowering_m": (
            None if not body_lowering else float(mean(body_lowering))
        ),
        "mean_rear_push_vertical_reversals": (
            None if not reversals else float(mean(reversals))
        ),
        "mean_climb_start_to_clear_steps": (
            None if not climb_latencies else float(mean(climb_latencies))
        ),
    }
