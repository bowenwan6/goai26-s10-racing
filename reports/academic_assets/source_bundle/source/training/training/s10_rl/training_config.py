"""Dependency-free training recipe shared by tests, Isaac Lab and documentation."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PPOConfig:
    """Official M20 PPO defaults retained for the first S10 transfer run."""

    steps_per_env: int = 24
    max_iterations: int = 20_000
    save_interval: int = 100
    actor_hidden_dims: tuple[int, ...] = (512, 256, 128)
    critic_hidden_dims: tuple[int, ...] = (512, 256, 128)
    activation: str = "elu"
    clip_param: float = 0.2
    entropy_coef: float = 0.003
    learning_epochs: int = 5
    mini_batches: int = 4
    learning_rate: float = 1.0e-3
    gamma: float = 0.99
    gae_lambda: float = 0.95


@dataclass(frozen=True)
class RewardConfig:
    """Reward priorities for rolling, stepping and rough-ground locomotion."""

    track_linear_velocity: float = 5.0
    track_yaw_velocity: float = 3.0
    vertical_velocity_penalty: float = -2.0
    flat_orientation_penalty: float = -50.0
    leg_torque_penalty: float = -2.5e-5
    leg_acceleration_penalty: float = -2.0e-7
    wheel_acceleration_penalty: float = -1.0e-7
    action_rate_penalty: float = -0.01
    action_smoothness_penalty: float = -0.025
    undesired_contact_penalty: float = -1.0
    upward_velocity_reward: float = 0.0


@dataclass(frozen=True)
class CurriculumStage:
    name: str
    max_step_height: float
    max_roughness: float
    max_forward_command: float
    promote_success_rate: float = 0.80


# Stages 1-3 are deliberately easier than the official M20 terminal ranges.  Stage 4
# reaches the official 0.20 m boxes / 0.16 m roughness distribution.  Promotion is based
# on success, never merely on elapsed iterations.
TERRAIN_CURRICULUM = (
    CurriculumStage("flat", 0.02, 0.01, 0.6),
    CurriculumStage("low_step", 0.08, 0.04, 0.8),
    CurriculumStage("mixed_step", 0.14, 0.10, 1.2),
    CurriculumStage("m20_rough", 0.20, 0.16, 2.0),
)


def can_promote(success_rate: float, fall_rate: float, stage: CurriculumStage) -> bool:
    """A curriculum gate that rejects reward hacking and unstable fast policies."""

    return success_rate >= stage.promote_success_rate and fall_rate <= 0.10

