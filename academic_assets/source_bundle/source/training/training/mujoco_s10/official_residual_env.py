"""Phase-aware residual-RL environment over the untouched organizer track."""

from __future__ import annotations

from pathlib import Path
import math

import numpy as np

from .distill_cem import _infer
from .env import ACTION_ABS_GUARD
from .evaluate_official_track_skill import OfficialTrackRuntime


RESIDUAL_SCALE = np.asarray([1.0] * 12 + [6.0] * 4, dtype=np.float32)
CORRECTION_SCALE = np.asarray([0.25] * 12 + [1.5] * 4, dtype=np.float32)
SKILL_FEATURE_DIM = 36
OBSERVATION_DIM = 174 + SKILL_FEATURE_DIM


class OfficialResidualEnv:
    """Frozen locomotion actor plus a learned normalized 16-D residual."""

    def __init__(
        self,
        xml_path: Path,
        base_actor,
        *,
        seed: int,
        approach_speed: float = 0.10,
        activate_distance: float = 0.523,
        distance_range: tuple[float, float] = (0.8, 1.2),
        lateral_range: float = 0.0,
        yaw_range: float = 0.0,
        height_noise: float = 0.0,
        nominal_approach: np.ndarray | None = None,
        nominal_rear: np.ndarray | None = None,
        nominal_approach_actions: np.ndarray | None = None,
        nominal_rear_actions: np.ndarray | None = None,
        heading_align_gain: float = 1.5,
        correction_gain: float = 1.0,
    ) -> None:
        self.runtime = OfficialTrackRuntime(xml_path, max_steps=850)
        self.actor = base_actor
        self.rng = np.random.default_rng(seed)
        self.approach_speed = float(approach_speed)
        self.activate_distance = float(activate_distance)
        self.distance_range = distance_range
        self.lateral_range = float(lateral_range)
        self.yaw_range = float(yaw_range)
        self.height_noise = float(height_noise)
        self.nominal_approach = (
            np.asarray(nominal_approach, dtype=np.float32)
            if nominal_approach is not None
            else None
        )
        self.nominal_rear = (
            np.asarray(nominal_rear, dtype=np.float32)
            if nominal_rear is not None
            else None
        )
        self.nominal_approach_actions = (
            np.asarray(nominal_approach_actions, dtype=np.float32)
            if nominal_approach_actions is not None
            else None
        )
        self.nominal_rear_actions = (
            np.asarray(nominal_rear_actions, dtype=np.float32)
            if nominal_rear_actions is not None
            else None
        )
        self.heading_align_gain = float(heading_align_gain)
        self.correction_gain = float(correction_gain)
        self.phase = 0
        self.phase_step = 0
        self.previous_residual = np.zeros(16, dtype=np.float32)
        self.previous_x = 0.0
        self.previous_front_lift = 0.0
        self.previous_rear_lift = 0.0
        self.previous_com_progress = 0.0
        self.last_base_observation = np.zeros(174, dtype=np.float32)

    @property
    def step_count(self) -> int:
        return self.runtime.step_count

    def _skill_features(self) -> np.ndarray:
        env = self.runtime
        wheels = env.data.xpos[env.wheel_body_ids]
        distance = env.right_lip_x - float(env.data.qpos[0])
        phase_one_hot = np.zeros(3, dtype=np.float32)
        phase_one_hot[self.phase] = 1.0
        clocks = np.asarray(
            [
                np.clip(self.phase_step / 100.0, 0.0, 1.5)
                if self.phase == 1
                else 0.0,
                np.clip(self.phase_step / 180.0, 0.0, 1.5)
                if self.phase == 2
                else 0.0,
            ],
            dtype=np.float32,
        )
        return np.concatenate(
            [
                phase_one_hot,
                clocks,
                np.asarray([np.clip(distance / 1.2, -1.0, 1.0)], dtype=np.float32),
                np.clip((wheels[:, 0] - env.right_lip_x) / 0.60, -1.0, 1.0),
                np.clip((wheels[:, 2] - env.floor_z) / 0.60, -1.0, 1.0),
                env._projected_gravity().astype(np.float32),
                np.clip(np.asarray(env.data.qvel[:3]) / 2.0, -1.0, 1.0),
                self.previous_residual,
            ]
        ).astype(np.float32)

    def observation(self) -> np.ndarray:
        return np.concatenate([self.last_base_observation, self._skill_features()])

    def reset(self) -> np.ndarray:
        distance = float(self.rng.uniform(*self.distance_range))
        lateral = float(self.rng.uniform(-self.lateral_range, self.lateral_range))
        yaw = float(self.rng.uniform(-self.yaw_range, self.yaw_range))
        height = float(self.rng.uniform(-self.height_noise, self.height_noise))
        self.phase = 0
        self.phase_step = 0
        self.previous_residual.fill(0.0)
        self.last_base_observation = self.runtime.reset_floor(
            distance=distance,
            lateral=lateral,
            yaw=yaw,
            base_height_noise=height,
        )
        self.previous_x = float(self.runtime.data.qpos[0])
        self.previous_front_lift = self.runtime.best_front_lift
        self.previous_rear_lift = self.runtime.best_rear_lift
        self.previous_com_progress = self.runtime.best_com_progress
        return self.observation()

    def step(self, normalized_residual: np.ndarray):
        env = self.runtime
        distance = env.right_lip_x - float(env.data.qpos[0])
        if self.phase == 0 and distance <= self.activate_distance:
            self.phase = 1
            self.phase_step = 0

        actor_observation = self.last_base_observation.copy()
        rotation = env.data.xmat[env.base_id].reshape(3, 3)
        heading_error = math.atan2(rotation[1, 0], rotation[0, 0])
        actor_observation[8] = np.clip(
            -self.heading_align_gain * heading_error, -0.8, 0.8
        )
        if self.phase == 0:
            actor_observation[6] = self.approach_speed
            applied_residual = np.zeros(16, dtype=np.float32)
            nominal_residual = np.zeros(16, dtype=np.float32)
        else:
            applied_residual = np.clip(
                np.asarray(normalized_residual, dtype=np.float32), -1.0, 1.0
            )
            if self.phase == 1 and self.nominal_approach is not None:
                nominal_residual = self.nominal_approach[
                    min(self.phase_step, self.nominal_approach.shape[0] - 1)
                ]
            elif self.phase == 2 and self.nominal_rear is not None:
                nominal_residual = self.nominal_rear[
                    min(self.phase_step, self.nominal_rear.shape[0] - 1)
                ]
            else:
                nominal_residual = np.zeros(16, dtype=np.float32)
        base_action = _infer(self.actor, actor_observation[None, :])[0]
        if self.phase == 1 and self.nominal_approach_actions is not None:
            nominal_action = self.nominal_approach_actions[
                min(self.phase_step, self.nominal_approach_actions.shape[0] - 1)
            ]
        elif self.phase == 2 and self.nominal_rear_actions is not None:
            nominal_action = self.nominal_rear_actions[
                min(self.phase_step, self.nominal_rear_actions.shape[0] - 1)
            ]
        else:
            nominal_action = base_action + nominal_residual
        final_action = np.clip(
            nominal_action
            + self.correction_gain * applied_residual * CORRECTION_SCALE,
            -ACTION_ABS_GUARD,
            ACTION_ABS_GUARD,
        ).astype(np.float32)
        self.last_base_observation, _, done, info = env.step(final_action)
        if self.phase == 1 and int(info.get("right_wheels", 0)) >= 2:
            self.phase = 2
            self.phase_step = 0
        elif self.phase > 0:
            self.phase_step += 1

        x = float(env.data.qpos[0])
        dx = np.clip(x - self.previous_x, -0.05, 0.05)
        front_gain = max(0.0, env.best_front_lift - self.previous_front_lift)
        rear_gain = max(0.0, env.best_rear_lift - self.previous_rear_lift)
        com_gain = max(0.0, env.best_com_progress - self.previous_com_progress)
        reward = (
            30.0 * dx
            + 25.0 * front_gain
            + 60.0 * rear_gain
            + 30.0 * com_gain
            - 0.01
            - 0.002 * float(np.mean(np.square(applied_residual)))
            - 0.004
            * float(np.mean(np.square(applied_residual - self.previous_residual)))
        )
        if info.get("success", False):
            reward += 250.0 + 0.10 * max(0, env.max_steps - env.step_count)
        if info.get("fallen", False):
            reward -= 120.0
        if info.get("diverged", False):
            reward -= 180.0

        self.previous_x = x
        self.previous_front_lift = env.best_front_lift
        self.previous_rear_lift = env.best_rear_lift
        self.previous_com_progress = env.best_com_progress
        self.previous_residual = applied_residual.copy()
        info = dict(info)
        info["phase"] = self.phase
        info["phase_step"] = self.phase_step
        info["applied_residual_norm"] = float(
            np.sqrt(np.mean(np.square(applied_residual)))
        )
        return self.observation(), float(reward), bool(done), info
