"""State-feedback residual environment over the untouched official S10 track."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from s10_rl.skill_gate import HeightmapSkillGate, SkillGateConfig

from .distill_cem import _infer
from .env import ACTION_ABS_GUARD
from .evaluate_official_track_skill import OfficialTrackRuntime
from .front_retention import (
    FrontRetentionConfig,
    FrontRetentionTracker,
    front_retention_reward,
)
from .rear_push import (
    RearPushConfig,
    RearPushTracker,
    front_wheel_sagittal_distance_m,
    rear_push_reward,
)
from .speed_objective import SpeedObjectiveConfig, speed_reward


OBSERVATION_DIM = 174
CORRECTION_SCALE = np.asarray([0.25] * 12 + [1.5] * 4, dtype=np.float32)
FULL_CORRECTION_SCALE = np.asarray([1.0] * 12 + [6.0] * 4, dtype=np.float32)
CLIMB_START_FRONT_LIFT = 0.02


class OfficialClosedLoopResidualEnv:
    """Frozen 174-D seed actor plus a learned state-dependent 16-D correction."""

    def __init__(
        self,
        xml_path: Path,
        base_actor,
        *,
        seed: int,
        approach_speed: float = 0.10,
        activate_distance: float = 0.65,
        distance_range: tuple[float, float] = (0.8, 1.2),
        hard_distance_values: tuple[float, ...] | None = None,
        hard_distance_probability: float = 0.0,
        hard_yaw_values: tuple[float, ...] | None = None,
        hard_yaw_probability: float = 0.0,
        speed_range: tuple[float, float] | None = None,
        lateral_range: float = 0.04,
        yaw_range: float = 0.025,
        height_noise: float = 0.002,
        heading_gain: float = 1.5,
        correction_scale: np.ndarray = CORRECTION_SCALE,
        correction_limit: float = 1.0,
        activation_mode: str = "distance",
        gate_config: SkillGateConfig | None = None,
        retention_config: FrontRetentionConfig | None = None,
        objective_mode: str = "retention",
        speed_config: SpeedObjectiveConfig | None = None,
        rear_push_config: RearPushConfig | None = None,
        rear_push_settle_action_delta: np.ndarray | None = None,
        rear_push_push_action_delta: np.ndarray | None = None,
        rear_push_action_delta_scale: float = 0.0,
    ) -> None:
        self.runtime = OfficialTrackRuntime(xml_path, max_steps=850)
        self.runtime.cfg.command_forward = float(approach_speed)
        self.base_actor = base_actor
        self.rng = np.random.default_rng(seed)
        self.approach_speed = float(approach_speed)
        self.current_command_forward = float(approach_speed)
        self.current_command_lateral = 0.0
        self.current_command_yaw: float | None = None
        self.activate_distance = float(activate_distance)
        self.distance_range = distance_range
        self.hard_distance_values = (
            None
            if hard_distance_values is None
            else np.asarray(hard_distance_values, dtype=np.float64)
        )
        self.hard_distance_probability = float(hard_distance_probability)
        if not 0.0 <= self.hard_distance_probability <= 1.0:
            raise ValueError("hard_distance_probability must be in [0, 1]")
        if (
            self.hard_distance_probability > 0.0
            and (self.hard_distance_values is None or self.hard_distance_values.size == 0)
        ):
            raise ValueError("positive hard-distance probability needs hard distances")
        self.hard_yaw_values = (
            None
            if hard_yaw_values is None
            else np.asarray(hard_yaw_values, dtype=np.float64)
        )
        self.hard_yaw_probability = float(hard_yaw_probability)
        if not 0.0 <= self.hard_yaw_probability <= 1.0:
            raise ValueError("hard-yaw probability must be in [0, 1]")
        if (
            self.hard_yaw_probability > 0.0
            and (self.hard_yaw_values is None or self.hard_yaw_values.size == 0)
        ):
            raise ValueError("positive hard-yaw probability needs hard yaw values")
        self.speed_range = (
            (self.approach_speed, self.approach_speed)
            if speed_range is None
            else (float(speed_range[0]), float(speed_range[1]))
        )
        if self.speed_range[0] <= 0.0 or self.speed_range[1] < self.speed_range[0]:
            raise ValueError("speed_range must contain positive increasing speeds")
        self.lateral_range = float(lateral_range)
        self.yaw_range = float(yaw_range)
        self.height_noise = float(height_noise)
        self.heading_gain = float(heading_gain)
        self.correction_scale = np.asarray(correction_scale, dtype=np.float32)
        if self.correction_scale.shape != (16,) or np.any(self.correction_scale <= 0.0):
            raise ValueError("correction_scale must contain 16 positive values")
        self.correction_limit = float(correction_limit)
        if self.correction_limit <= 0.0:
            raise ValueError("correction_limit must be positive")
        if activation_mode not in {"distance", "heightmap"}:
            raise ValueError("activation_mode must be 'distance' or 'heightmap'")
        self.activation_mode = activation_mode
        self.skill_gate = HeightmapSkillGate(gate_config)
        self.retention_config = retention_config or FrontRetentionConfig()
        if objective_mode not in {"retention", "speed", "phase"}:
            raise ValueError("objective_mode must be 'retention', 'speed', or 'phase'")
        self.objective_mode = objective_mode
        self.speed_config = speed_config or SpeedObjectiveConfig()
        self.retention_tracker = FrontRetentionTracker()
        self.rear_push_config = rear_push_config or RearPushConfig()
        self.rear_push_tracker = RearPushTracker(self.rear_push_config)
        self.rear_push_settle_action_delta = self._action_delta(
            rear_push_settle_action_delta, "settle"
        )
        self.rear_push_push_action_delta = self._action_delta(
            rear_push_push_action_delta, "push"
        )
        self.rear_push_action_delta_scale = float(rear_push_action_delta_scale)
        if self.rear_push_action_delta_scale < 0.0:
            raise ValueError("rear-push action delta scale cannot be negative")
        if self.rear_push_action_delta_scale and (
            rear_push_settle_action_delta is None or rear_push_push_action_delta is None
        ):
            raise ValueError("positive rear-push delta scale needs settle and push deltas")
        self.last_observation = np.zeros(174, dtype=np.float32)
        self.previous_residual = np.zeros(16, dtype=np.float32)
        self.previous_x = 0.0
        self.previous_front = 0.0
        self.previous_rear = 0.0
        self.previous_com = 0.0
        self.previous_wheels = 0
        self.active = False
        self.first_climb_step: int | None = None
        self.current_rear_push_phase = "approach"

    @staticmethod
    def _action_delta(value: np.ndarray | None, phase: str) -> np.ndarray:
        if value is None:
            return np.zeros(16, dtype=np.float32)
        result = np.asarray(value, dtype=np.float32)
        if result.shape != (16,) or not np.all(np.isfinite(result)):
            raise ValueError(f"rear-push {phase} action delta must be finite shape (16,)")
        return result.copy()

    @property
    def step_count(self) -> int:
        return self.runtime.step_count

    def _navigation_observation(self, observation: np.ndarray) -> np.ndarray:
        result = observation.copy()
        result[6] = self.current_command_forward
        result[7] = self.current_command_lateral
        rotation = self.runtime.data.xmat[self.runtime.base_id].reshape(3, 3)
        heading_error = math.atan2(rotation[1, 0], rotation[0, 0])
        result[8] = (
            np.clip(-self.heading_gain * heading_error, -0.8, 0.8)
            if self.current_command_yaw is None
            else self.current_command_yaw
        )
        return result

    def reset(self) -> np.ndarray:
        if (
            self.hard_distance_values is not None
            and self.rng.random() < self.hard_distance_probability
        ):
            distance = float(self.rng.choice(self.hard_distance_values))
        else:
            distance = float(self.rng.uniform(*self.distance_range))
        speed = float(self.rng.uniform(*self.speed_range))
        lateral = float(self.rng.uniform(-self.lateral_range, self.lateral_range))
        if (
            self.hard_yaw_values is not None
            and self.rng.random() < self.hard_yaw_probability
        ):
            yaw = float(self.rng.choice(self.hard_yaw_values))
        else:
            yaw = float(self.rng.uniform(-self.yaw_range, self.yaw_range))
        return self.reset_entry(
            distance=distance,
            lateral=lateral,
            yaw=yaw,
            base_height_noise=float(
                self.rng.uniform(-self.height_noise, self.height_noise)
            ),
            command_forward=speed,
            forward_speed=speed,
        )

    def reset_entry(
        self,
        *,
        distance: float,
        lateral: float = 0.0,
        yaw: float = 0.0,
        base_height_noise: float = 0.0,
        entry_center_y: float = 33.365,
        command_forward: float | None = None,
        command_lateral: float = 0.0,
        command_yaw: float | None = None,
        forward_speed: float = 0.0,
        lateral_speed: float = 0.0,
        yaw_rate: float = 0.0,
    ) -> np.ndarray:
        """Reset to an exact Gate-16 entry pose.

        Training still uses :meth:`reset` and its randomized distribution.  Evaluation,
        demonstration collection and the navigation handoff contract need an exact pose
        so every checkpoint is compared on the same distance/lateral/yaw matrix.
        """
        self.active = False
        self.skill_gate.reset()
        self.retention_tracker.reset()
        self.rear_push_tracker.reset()
        self.current_rear_push_phase = "approach"
        self.previous_residual.fill(0.0)
        self.current_command_forward = float(
            self.approach_speed if command_forward is None else command_forward
        )
        self.current_command_lateral = float(command_lateral)
        self.current_command_yaw = (
            None if command_yaw is None else float(command_yaw)
        )
        observation = self.runtime.reset_floor(
            distance=float(distance),
            lateral=float(lateral),
            yaw=float(yaw),
            base_height_noise=float(base_height_noise),
            entry_center_y=float(entry_center_y),
            forward_speed=float(forward_speed),
            lateral_speed=float(lateral_speed),
            yaw_rate=float(yaw_rate),
        )
        self.last_observation = self._navigation_observation(observation)
        self.previous_x = float(self.runtime.data.qpos[0])
        self.previous_front = self.runtime.best_front_lift
        self.previous_rear = self.runtime.best_rear_lift
        self.previous_com = self.runtime.best_com_progress
        self.previous_wheels = self.runtime.best_right_wheels
        self.first_climb_step = None
        self.retention_tracker.update(self.runtime.front_retention_sample())
        return self.last_observation.copy()

    def step(
        self,
        normalized_residual: np.ndarray,
        *,
        base_action: np.ndarray | None = None,
    ):
        env = self.runtime
        distance = env.right_lip_x - float(env.data.qpos[0])
        if self.activation_mode == "heightmap":
            gate_state = self.skill_gate.update(self.last_observation[57:174])
            self.active = gate_state.active
        else:
            gate_state = None
            if distance <= self.activate_distance:
                self.active = True
        requested = np.clip(
            np.asarray(normalized_residual, dtype=np.float32),
            -self.correction_limit,
            self.correction_limit,
        )
        applied = requested if self.active else np.zeros(16, dtype=np.float32)
        if base_action is None:
            base_action = _infer(self.base_actor, self.last_observation[None, :])[0]
        else:
            base_action = np.asarray(base_action, dtype=np.float32)
            if base_action.shape != (16,):
                raise ValueError("base_action must have shape (16,)")
        phase_action_delta = np.zeros(16, dtype=np.float32)
        if self.active and self.current_rear_push_phase == "settle":
            phase_action_delta = self.rear_push_settle_action_delta
        elif self.active and self.current_rear_push_phase == "push":
            phase_action_delta = self.rear_push_push_action_delta
        phase_action_delta = (
            phase_action_delta * self.rear_push_action_delta_scale
        ).astype(np.float32)
        final_action = np.clip(
            base_action + applied * self.correction_scale + phase_action_delta,
            -ACTION_ABS_GUARD,
            ACTION_ABS_GUARD,
        ).astype(np.float32)
        observation, _, done, info = env.step(final_action)
        self.last_observation = self._navigation_observation(observation)

        x = float(env.data.qpos[0])
        dx = float(np.clip(x - self.previous_x, -0.05, 0.05))
        front_gain = max(0.0, env.best_front_lift - self.previous_front)
        rear_gain = max(0.0, env.best_rear_lift - self.previous_rear)
        com_gain = max(0.0, env.best_com_progress - self.previous_com)
        wheels = env.right_wheels_on_exit()
        best_wheels = env.best_right_wheels
        wheel_gain = max(0, best_wheels - self.previous_wheels)
        retention = self.retention_tracker.update(env.front_retention_sample())
        rear_push = self.rear_push_tracker.update(
            retention,
            base_height_m=float(env.data.qpos[2]),
            front_wheel_body_distance_m=front_wheel_sagittal_distance_m(
                env.data.xpos[env.base_id],
                env.data.xmat[env.base_id].reshape(3, 3),
                env.data.xpos[env.wheel_body_ids[:2]],
            ),
        )
        self.current_rear_push_phase = rear_push.phase
        if self.first_climb_step is None and env.best_front_lift >= CLIMB_START_FRONT_LIFT:
            self.first_climb_step = env.step_count
        if self.objective_mode in {"speed", "phase"}:
            reward = speed_reward(
                dx=dx,
                front_gain=front_gain,
                rear_gain=rear_gain,
                com_gain=com_gain,
                wheel_gain=wheel_gain,
                action_mean_square=float(np.mean(np.square(applied))),
                action_change_mean_square=float(
                    np.mean(np.square(applied - self.previous_residual))
                ),
                success=bool(info.get("success", False)),
                fallen=bool(info.get("fallen", False)),
                diverged=bool(info.get("diverged", False)),
                step_count=env.step_count,
                max_steps=env.max_steps,
                active=self.active,
                time_penalty_active=(
                    retention.phase_active
                    if self.objective_mode == "phase"
                    else self.active
                ),
                phase_elapsed_steps=(
                    rear_push.phase_steps
                    if self.objective_mode == "phase"
                    else None
                ),
                config=self.speed_config,
            )
            if self.objective_mode == "phase":
                reward += front_retention_reward(retention, self.retention_config)
                reward += rear_push_reward(rear_push, self.rear_push_config)
        else:
            reward = 0.0
            if self.active:
                reward = (
                    35.0 * dx
                    + 35.0 * front_gain
                    + 90.0 * rear_gain
                    + 45.0 * com_gain
                    + 8.0 * wheel_gain
                    - 0.01
                    - 0.002 * float(np.mean(np.square(applied)))
                    - 0.003
                    * float(np.mean(np.square(applied - self.previous_residual)))
                )
                reward += front_retention_reward(retention, self.retention_config)
            if info.get("success", False):
                reward += 300.0 + 0.08 * max(0, env.max_steps - env.step_count)
            if info.get("fallen", False):
                reward -= 150.0
            if info.get("diverged", False):
                reward -= 200.0

        self.previous_x = x
        self.previous_front = env.best_front_lift
        self.previous_rear = env.best_rear_lift
        self.previous_com = env.best_com_progress
        self.previous_wheels = best_wheels
        self.previous_residual = applied.copy()
        info = dict(info)
        info["skill_active"] = self.active
        info["residual_norm"] = float(np.sqrt(np.mean(np.square(applied))))
        info["front_supported"] = retention.sample.front_supported
        info["rear_supported"] = retention.sample.rear_supported
        info["front_min_clearance_m"] = retention.sample.front_min_clearance_m
        info["balance_error"] = retention.sample.balance_error
        info["front_drop_event"] = retention.front_drop_event
        info["front_drop_count"] = retention.front_drop_count
        info["front_support_steps"] = retention.front_support_steps
        info["first_front_step"] = retention.first_front_step
        info["first_rear_step"] = retention.first_rear_step
        info["front_to_rear_steps"] = retention.front_to_rear_steps
        info["drop_free"] = retention.drop_free
        info["climb_start_step"] = self.first_climb_step
        info["climb_start_to_clear_steps"] = (
            None
            if self.first_climb_step is None or not bool(info.get("success", False))
            else env.step_count - self.first_climb_step
        )
        info["rear_push_phase"] = rear_push.phase
        info["rear_push_phase_steps"] = rear_push.phase_steps
        info["front_wheel_body_distance_m"] = (
            rear_push.front_wheel_body_distance_m
        )
        info["front_tuck_m"] = rear_push.front_tuck_m
        info["front_tuck_target_reached"] = (
            rear_push.front_tuck_target_reached
        )
        info["front_tuck_step"] = rear_push.tuck_step
        info["body_lowering_m"] = rear_push.body_lowering_m
        info["rear_push_vertical_reversals"] = rear_push.vertical_reversal_count
        info["rear_push_action_delta_norm"] = float(
            np.sqrt(np.mean(np.square(phase_action_delta)))
        )
        # These are evidence/collection fields only.  They make the exact action that
        # reached MuJoCo auditable without changing the observation or reward contract.
        info["requested_residual"] = requested.copy()
        info["applied_residual"] = applied.copy()
        info["base_action"] = base_action.copy()
        info["final_action"] = final_action.copy()
        if gate_state is not None:
            info["gate_max_up_step"] = gate_state.max_up_step
            info["gate_active_frames"] = gate_state.active_frames
        return self.last_observation.copy(), float(reward), bool(done), info
