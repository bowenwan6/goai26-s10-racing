#!/usr/bin/env python3
"""PPO-train a closed-loop 174-D correction on the official S10 track."""

from __future__ import annotations

import argparse
import copy
import csv
import json
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import torch

from s10_rl.training_config import PPOConfig

from .distill_cem import _infer, _load_inference_actor
from .network import ActorCritic
from .official_policy_env import (
    FULL_CORRECTION_SCALE,
    OBSERVATION_DIM,
    OfficialClosedLoopResidualEnv,
)
from .variable_height_policy_env import VariableHeightResidualEnv
from .train import ppo_update
from .front_retention import FrontRetentionConfig
from .rear_push import RearPushConfig
from .speed_objective import SpeedObjectiveConfig, finite_or


@torch.no_grad()
def evaluate(model, envs, trials: int, device: torch.device):
    # Common-random-number evaluation: every call must replay the same reset
    # distribution.  Without restoring these RNG states, apparent changes in
    # success rate can come from drawing easier/harder start poses rather than
    # from the policy update itself.
    rng_states = [copy.deepcopy(env.rng.bit_generator.state) for env in envs]
    successes = []
    falls = []
    steps = []
    drop_free_successes = []
    front_to_rear_steps = []
    success_steps = []
    try:
        for trial in range(trials):
            env = envs[trial % len(envs)]
            observation = env.reset()
            done = False
            info = {"success": False, "fallen": False}
            while not done:
                correction = model.actor(
                    torch.as_tensor(observation, device=device).unsqueeze(0)
                ).squeeze(0)
                observation, _, done, info = env.step(correction.cpu().numpy())
            successes.append(float(info.get("success", False)))
            falls.append(float(info.get("fallen", False)))
            steps.append(float(env.step_count))
            if info.get("success", False):
                success_steps.append(float(env.step_count))
            drop_free_successes.append(
                float(info.get("success", False) and info.get("drop_free", True))
            )
            if info.get("success", False) and info.get("front_to_rear_steps") is not None:
                front_to_rear_steps.append(float(info["front_to_rear_steps"]))
    finally:
        for env, rng_state in zip(envs, rng_states):
            env.rng.bit_generator.state = rng_state
    return (
        float(np.mean(successes)),
        float(np.mean(falls)),
        float(np.mean(steps)),
        float(np.mean(drop_free_successes)),
        None if not front_to_rear_steps else float(np.mean(front_to_rear_steps)),
        None if not success_steps else float(np.mean(success_steps)),
        None if not success_steps else float(np.std(success_steps)),
        None if not success_steps else float(np.min(success_steps)),
    )


def save_checkpoint(
    path,
    model,
    iteration,
    args,
    eval_success,
    eval_fall,
    eval_drop_free=0.0,
    eval_front_to_rear_steps=None,
    eval_mean_success_steps=None,
    eval_std_success_steps=None,
    eval_fastest_success_steps=None,
):
    torch.save(
        {
            "actor_state_dict": model.actor.state_dict(),
            "critic_state_dict": model.critic.state_dict(),
            "iteration": iteration,
            "observation_dim": OBSERVATION_DIM,
            "base_checkpoint": str(args.base_checkpoint.resolve()),
            "correction_scale": FULL_CORRECTION_SCALE.tolist(),
            "correction_limit": args.correction_limit,
            "activation_mode": args.activation_mode,
            "terrain_mode": args.terrain_mode,
            "objective_mode": args.objective_mode,
            "depth_range": [args.depth_min, args.depth_max],
            "speed_range": [args.speed_min, args.speed_max],
            "yaw_range_rad": args.yaw_range,
            "training_hard_yaw_degrees": args.training_hard_yaw_degrees,
            "training_hard_yaw_probability": args.training_hard_yaw_probability,
            "retention_reward": {
                "retained_front_reward": args.retained_front_reward,
                "rear_follow_step_penalty": args.rear_follow_step_penalty,
                "balance_penalty_scale": args.balance_penalty_scale,
                "clearance_regression_scale": args.clearance_regression_scale,
                "front_drop_penalty": args.front_drop_penalty,
                "rear_completed_bonus": args.rear_completed_bonus,
            },
            "speed_reward": {
                "active_step_penalty": args.speed_step_penalty,
                "success_time_bonus_per_remaining_step": (
                    args.speed_success_time_scale
                ),
                "fall_penalty": args.speed_fall_penalty,
            },
            "rear_push_reward": {
                "target_tuck_m": args.rear_push_target_tuck,
                "settle_window_steps": args.rear_push_settle_steps,
                "tuck_progress_scale": args.rear_push_tuck_scale,
                "early_extension_penalty_scale": (
                    args.rear_push_early_extension_scale
                ),
                "tuck_target_bonus": args.rear_push_tuck_bonus,
                "extra_reversal_penalty": args.rear_push_reversal_penalty,
            },
            "initial_residual_checkpoint": str(
                args.initial_residual_checkpoint.resolve()
            ) if args.initial_residual_checkpoint is not None else None,
            "eval_success": eval_success,
            "eval_fall": eval_fall,
            "eval_drop_free_success": eval_drop_free,
            "eval_front_to_rear_steps": eval_front_to_rear_steps,
            "eval_mean_success_steps": eval_mean_success_steps,
            "eval_std_success_steps": eval_std_success_steps,
            "eval_fastest_success_steps": eval_fastest_success_steps,
        },
        path,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", type=Path, required=True)
    parser.add_argument(
        "--terrain-mode",
        choices=("official_track", "pit_fixture"),
        default="official_track",
        help="fixed organizer track or variable-height official-S10 fixture",
    )
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--initial-residual-checkpoint", type=Path)
    parser.add_argument("--initial-critic-checkpoint", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--envs", type=int, default=12)
    parser.add_argument("--iterations", type=int, default=10000)
    parser.add_argument("--steps-per-env", type=int, default=32)
    parser.add_argument("--learning-rate", type=float, default=1.0e-4)
    parser.add_argument(
        "--actor-learning-rate",
        type=float,
        help="optional actor-specific learning rate; critic keeps --learning-rate",
    )
    parser.add_argument("--learning-epochs", type=int, default=5)
    parser.add_argument("--mini-batches", type=int, default=4)
    parser.add_argument("--clip-param", type=float, default=0.2)
    parser.add_argument("--entropy-coef", type=float, default=0.003)
    parser.add_argument("--initial-log-std", type=float, default=-2.0)
    parser.add_argument(
        "--actor-hidden",
        type=int,
        nargs="+",
        default=[256, 128, 64],
        help="residual actor hidden widths; must match an initial residual checkpoint",
    )
    parser.add_argument("--zero-anchor-coef", type=float, default=0.05)
    parser.add_argument(
        "--critic-warmup-iterations",
        type=int,
        default=0,
        help="freeze the residual actor while the randomly initialized critic learns",
    )
    parser.add_argument("--eval-interval", type=int, default=50)
    parser.add_argument("--eval-trials", type=int, default=32)
    parser.add_argument(
        "--eval-distance-grid",
        type=int,
        default=0,
        help="evaluate one deterministic trial at each equally spaced distance",
    )
    parser.add_argument(
        "--eval-depth-grid",
        type=int,
        default=0,
        help="pit_fixture only: cross the distance grid with fixed ledge heights",
    )
    parser.add_argument(
        "--eval-speed-grid",
        type=int,
        default=0,
        help="pit_fixture only: cross evaluation with fixed approach speeds",
    )
    parser.add_argument("--save-interval", type=int, default=100)
    parser.add_argument("--approach-speed", type=float, default=0.10)
    parser.add_argument("--activate-distance", type=float, default=0.65)
    parser.add_argument(
        "--activation-mode",
        choices=("distance", "heightmap"),
        default="distance",
        help="legacy fixed distance or deploy-matched stateful height-map gate",
    )
    parser.add_argument("--distance-min", type=float, default=0.80)
    parser.add_argument("--distance-max", type=float, default=1.20)
    parser.add_argument("--depth-min", type=float, default=0.30)
    parser.add_argument("--depth-max", type=float, default=0.40)
    parser.add_argument("--episode-seconds", type=float, default=12.0)
    parser.add_argument("--speed-min", type=float)
    parser.add_argument("--speed-max", type=float)
    parser.add_argument("--training-hard-distances", type=float, nargs="+")
    parser.add_argument("--training-hard-probability", type=float, default=0.0)
    parser.add_argument(
        "--training-hard-yaw-degrees",
        type=float,
        nargs="+",
        help="signed yaw values to oversample during training",
    )
    parser.add_argument("--training-hard-yaw-probability", type=float, default=0.0)
    parser.add_argument(
        "--eval-yaw-degrees",
        type=float,
        nargs="+",
        help="cross deterministic evaluation with these signed yaw values",
    )
    parser.add_argument("--lateral-range", type=float, default=0.04)
    parser.add_argument("--yaw-range", type=float, default=0.025)
    parser.add_argument("--height-noise", type=float, default=0.002)
    parser.add_argument("--correction-limit", type=float, default=4.0)
    parser.add_argument(
        "--objective-mode",
        choices=("retention", "speed", "phase"),
        default="retention",
    )
    parser.add_argument("--retained-front-reward", type=float, default=0.08)
    parser.add_argument("--rear-follow-step-penalty", type=float, default=0.03)
    parser.add_argument("--balance-penalty-scale", type=float, default=0.50)
    parser.add_argument("--clearance-regression-scale", type=float, default=25.0)
    parser.add_argument("--front-drop-penalty", type=float, default=12.0)
    parser.add_argument("--rear-completed-bonus", type=float, default=2.0)
    parser.add_argument("--speed-step-penalty", type=float, default=0.05)
    parser.add_argument("--speed-success-time-scale", type=float, default=0.50)
    parser.add_argument("--speed-fall-penalty", type=float, default=250.0)
    parser.add_argument("--rear-push-target-tuck", type=float, default=0.03)
    parser.add_argument("--rear-push-settle-steps", type=int, default=30)
    parser.add_argument("--rear-push-tuck-scale", type=float, default=40.0)
    parser.add_argument(
        "--rear-push-early-extension-scale", type=float, default=15.0
    )
    parser.add_argument("--rear-push-tuck-bonus", type=float, default=2.0)
    parser.add_argument("--rear-push-reversal-penalty", type=float, default=0.25)
    parser.add_argument(
        "--early-stop-evals",
        type=int,
        default=0,
        help="stop after this many consecutive evaluations without a new best",
    )
    parser.add_argument(
        "--env-step-workers",
        type=int,
        default=1,
        help="parallel MuJoCo step workers; 1 preserves the legacy serial runner",
    )
    parser.add_argument("--seed", type=int, default=37723)
    args = parser.parse_args()
    if args.terrain_mode == "official_track" and args.eval_depth_grid > 0:
        parser.error("depth grids require --terrain-mode pit_fixture")
    if args.terrain_mode == "pit_fixture" and (
        args.training_hard_yaw_degrees or args.eval_yaw_degrees
    ):
        parser.error("signed hard-yaw controls require --terrain-mode official_track")
    if args.terrain_mode == "pit_fixture" and args.objective_mode in {"speed", "phase"}:
        parser.error("speed/phase objectives require --terrain-mode official_track")
    if not 0.0 <= args.depth_min <= args.depth_max <= 0.50:
        parser.error("depth range must lie inside [0, 0.50]")
    if args.terrain_mode == "pit_fixture" and args.training_hard_distances:
        parser.error("hard-distance oversampling is not implemented for pit_fixture")
    if (args.speed_min is None) != (args.speed_max is None):
        parser.error("provide both --speed-min and --speed-max")
    if args.speed_min is None:
        args.speed_min = args.speed_max = args.approach_speed
    if args.speed_min <= 0.0 or args.speed_max < args.speed_min:
        parser.error("speed range must contain positive increasing speeds")
    if not 0.0 <= args.training_hard_yaw_probability <= 1.0:
        parser.error("training hard-yaw probability must be in [0, 1]")
    if args.training_hard_yaw_probability > 0.0 and not args.training_hard_yaw_degrees:
        parser.error("hard-yaw probability requires hard yaw values")
    if args.env_step_workers < 1:
        parser.error("env-step-workers must be positive")
    if args.early_stop_evals < 0:
        parser.error("early-stop-evals cannot be negative")
    if args.speed_step_penalty < 0.0 or args.speed_success_time_scale < 0.0:
        parser.error("speed time coefficients cannot be negative")
    if args.speed_fall_penalty < 0.0:
        parser.error("speed fall penalty cannot be negative")
    if args.rear_push_target_tuck <= 0.0:
        parser.error("rear-push target tuck must be positive")
    if args.rear_push_settle_steps < 1:
        parser.error("rear-push settle steps must be positive")
    if min(
        args.rear_push_tuck_scale,
        args.rear_push_early_extension_scale,
        args.rear_push_tuck_bonus,
        args.rear_push_reversal_penalty,
    ) < 0.0:
        parser.error("rear-push reward coefficients cannot be negative")
    if args.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable")
    args.output.mkdir(parents=True, exist_ok=True)
    device = torch.device(args.device)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    base_actor = _load_inference_actor(args.base_checkpoint, 174)
    hard_yaw_values = (
        None
        if args.training_hard_yaw_degrees is None
        else tuple(np.deg2rad(args.training_hard_yaw_degrees).tolist())
    )
    retention_config = FrontRetentionConfig(
        retained_front_reward=args.retained_front_reward,
        rear_follow_step_penalty=args.rear_follow_step_penalty,
        balance_penalty_scale=args.balance_penalty_scale,
        clearance_regression_scale=args.clearance_regression_scale,
        front_drop_penalty=args.front_drop_penalty,
        rear_completed_bonus=args.rear_completed_bonus,
    )
    speed_config = SpeedObjectiveConfig(
        active_step_penalty=args.speed_step_penalty,
        success_time_bonus_per_remaining_step=args.speed_success_time_scale,
        fall_penalty=args.speed_fall_penalty,
    )
    rear_push_config = RearPushConfig(
        target_tuck_m=args.rear_push_target_tuck,
        settle_window_steps=args.rear_push_settle_steps,
        tuck_progress_scale=args.rear_push_tuck_scale,
        early_extension_penalty_scale=args.rear_push_early_extension_scale,
        tuck_target_bonus=args.rear_push_tuck_bonus,
        extra_reversal_penalty=args.rear_push_reversal_penalty,
    )

    model = ActorCritic(
        OBSERVATION_DIM,
        actor_hidden=tuple(args.actor_hidden),
        critic_hidden=(256, 128, 64),
    ).to(device)
    with torch.no_grad():
        model.actor.mlp[-1].weight.zero_()
        model.actor.mlp[-1].bias.zero_()
        model.actor.log_std.fill_(args.initial_log_std)
    if args.initial_residual_checkpoint is not None:
        initial_actor = _load_inference_actor(
            args.initial_residual_checkpoint, OBSERVATION_DIM
        )
        model.actor.mlp.load_state_dict(initial_actor.state_dict())
        with torch.no_grad():
            model.actor.log_std.fill_(args.initial_log_std)
    if args.initial_critic_checkpoint is not None:
        critic_payload = torch.load(
            args.initial_critic_checkpoint, map_location=device, weights_only=False
        )
        model.critic.load_state_dict(critic_payload["critic_state_dict"])
    zero_reference = copy.deepcopy(model.actor).eval()
    for parameter in zero_reference.parameters():
        parameter.requires_grad_(False)

    def make_env(
        index: int,
        evaluation: bool = False,
        distance_range: tuple[float, float] | None = None,
        depth_range: tuple[float, float] | None = None,
        speed_range: tuple[float, float] | None = None,
        fixed_yaw: float | None = None,
    ):
        common = {
            "seed": args.seed + (900_000 if evaluation else 0) + 1009 * index,
            "approach_speed": args.approach_speed,
            "activate_distance": args.activate_distance,
            "distance_range": (args.distance_min, args.distance_max)
            if distance_range is None
            else distance_range,
            "lateral_range": args.lateral_range,
            "yaw_range": args.yaw_range,
            "correction_scale": FULL_CORRECTION_SCALE,
            "correction_limit": args.correction_limit,
            "activation_mode": args.activation_mode,
        }
        if args.terrain_mode == "pit_fixture":
            return VariableHeightResidualEnv(
                args.xml,
                base_actor,
                depth_range=(args.depth_min, args.depth_max)
                if depth_range is None
                else depth_range,
                speed_range=(args.speed_min, args.speed_max)
                if speed_range is None
                else speed_range,
                episode_seconds=args.episode_seconds,
                **common,
            )
        return OfficialClosedLoopResidualEnv(
            args.xml,
            base_actor,
            speed_range=(args.speed_min, args.speed_max)
            if speed_range is None
            else speed_range,
            hard_distance_values=(
                tuple(args.training_hard_distances)
                if not evaluation and args.training_hard_distances is not None
                else None
            ),
            hard_distance_probability=(
                args.training_hard_probability if not evaluation else 0.0
            ),
            hard_yaw_values=(
                (float(fixed_yaw),) if fixed_yaw is not None else hard_yaw_values
            ),
            hard_yaw_probability=(
                1.0
                if fixed_yaw is not None
                else (args.training_hard_yaw_probability if not evaluation else 0.0)
            ),
            retention_config=retention_config,
            objective_mode=args.objective_mode,
            speed_config=speed_config,
            rear_push_config=rear_push_config,
            height_noise=args.height_noise,
            **common,
        )

    envs = [make_env(index) for index in range(args.envs)]
    if (
        args.eval_distance_grid > 0
        or args.eval_depth_grid > 0
        or args.eval_speed_grid > 0
        or args.eval_yaw_degrees
    ):
        eval_distances = (
            np.linspace(args.distance_min, args.distance_max, args.eval_distance_grid)
            if args.eval_distance_grid > 0
            else [None]
        )
        eval_depths = (
            np.linspace(args.depth_min, args.depth_max, args.eval_depth_grid)
            if args.eval_depth_grid > 0
            else [None]
        )
        eval_speeds = (
            np.linspace(args.speed_min, args.speed_max, args.eval_speed_grid)
            if args.eval_speed_grid > 0
            else [None]
        )
        eval_yaws = (
            np.deg2rad(args.eval_yaw_degrees)
            if args.eval_yaw_degrees
            else [None]
        )
        eval_envs = []
        for yaw in eval_yaws:
            for speed in eval_speeds:
                for depth in eval_depths:
                    for distance in eval_distances:
                        distance_range = (
                            None
                            if distance is None
                            else (float(distance), float(distance))
                        )
                        depth_range = (
                            None
                            if depth is None
                            else (float(depth), float(depth))
                        )
                        speed_range = (
                            None
                            if speed is None
                            else (float(speed), float(speed))
                        )
                        eval_envs.append(
                            make_env(
                                len(eval_envs),
                                True,
                                distance_range=distance_range,
                                depth_range=depth_range,
                                speed_range=speed_range,
                                fixed_yaw=None if yaw is None else float(yaw),
                            )
                        )
        eval_trials = len(eval_envs)
    else:
        eval_envs = [make_env(index, True) for index in range(min(4, args.envs))]
        eval_trials = args.eval_trials
    (
        initial_success,
        initial_fall,
        initial_steps,
        initial_drop_free,
        initial_front_to_rear,
        initial_mean_success_steps,
        initial_std_success_steps,
        initial_fastest_success_steps,
    ) = evaluate(
        model, eval_envs, eval_trials, device
    )
    print(
        f"initial deterministic eval success={initial_success:.1%} "
        f"fall={initial_fall:.1%} drop_free={initial_drop_free:.1%} "
        f"steps={initial_steps:.1f} "
        f"success_steps={finite_or(initial_mean_success_steps, float('nan')):.1f} "
        f"fastest={finite_or(initial_fastest_success_steps, float('nan')):.1f}",
        flush=True,
    )
    save_checkpoint(
        args.output / "checkpoint_000000.pt",
        model,
        0,
        args,
        initial_success,
        initial_fall,
        initial_drop_free,
        initial_front_to_rear,
        initial_mean_success_steps,
        initial_std_success_steps,
        initial_fastest_success_steps,
    )
    save_checkpoint(
        args.output / "best_checkpoint.pt",
        model,
        0,
        args,
        initial_success,
        initial_fall,
        initial_drop_free,
        initial_front_to_rear,
        initial_mean_success_steps,
        initial_std_success_steps,
        initial_fastest_success_steps,
    )
    best_eval_success = initial_success
    best_eval_fall = initial_fall
    best_eval_drop_free = initial_drop_free
    best_eval_front_to_rear = initial_front_to_rear
    best_eval_mean_success_steps = initial_mean_success_steps
    best_eval_std_success_steps = initial_std_success_steps
    best_eval_fastest_success_steps = initial_fastest_success_steps
    best_eval_steps = initial_steps
    best_iteration = 0

    observations = np.stack([env.reset() for env in envs])
    recent_success = deque(maxlen=100)
    recent_falls = deque(maxlen=100)
    actor_learning_rate = (
        args.learning_rate
        if args.actor_learning_rate is None
        else args.actor_learning_rate
    )
    optimizer = torch.optim.Adam(
        [
            {"params": model.actor.parameters(), "lr": actor_learning_rate},
            {"params": model.critic.parameters(), "lr": args.learning_rate},
        ]
    )
    ppo = PPOConfig(
        steps_per_env=args.steps_per_env,
        max_iterations=args.iterations,
        save_interval=args.save_interval,
        actor_hidden_dims=tuple(args.actor_hidden),
        critic_hidden_dims=(256, 128, 64),
        learning_rate=args.learning_rate,
        learning_epochs=args.learning_epochs,
        mini_batches=args.mini_batches,
        clip_param=args.clip_param,
        entropy_coef=args.entropy_coef,
    )
    log_stream = (args.output / "train.csv").open("w", newline="", buffering=1)
    fields = [
        "iteration", "timesteps", "reward", "success", "fall",
        "eval_success", "eval_fall", "eval_steps", "policy_loss",
        "eval_drop_free", "eval_front_to_rear_steps",
        "eval_mean_success_steps", "eval_std_success_steps",
        "eval_fastest_success_steps",
        "value_loss", "std", "fps",
        "actor_frozen",
    ]
    writer = csv.DictWriter(log_stream, fieldnames=fields)
    writer.writeheader()
    total_steps = 0
    started = time.monotonic()
    consecutive_passes = 0
    evals_without_improvement = 0
    early_stop_reason = None
    step_pool = (
        None
        if args.env_step_workers == 1
        else ThreadPoolExecutor(max_workers=args.env_step_workers)
    )
    try:
        for iteration in range(1, args.iterations + 1):
            shape = (args.steps_per_env, args.envs)
            obs_buffer = np.empty((*shape, OBSERVATION_DIM), np.float32)
            action_buffer = np.empty((*shape, 16), np.float32)
            reward_buffer = np.empty(shape, np.float32)
            done_buffer = np.empty(shape, np.float32)
            value_buffer = np.empty(shape, np.float32)
            log_prob_buffer = np.empty(shape, np.float32)
            for step in range(args.steps_per_env):
                tensor = torch.as_tensor(observations, device=device)
                actions, log_prob, values = model.act(tensor)
                action_array = actions.cpu().numpy().astype(np.float32)
                base_action_array = _infer(base_actor, observations)
                obs_buffer[step] = observations
                action_buffer[step] = action_array
                value_buffer[step] = values.cpu().numpy()
                log_prob_buffer[step] = log_prob.cpu().numpy()
                if step_pool is None:
                    step_results = [
                        env.step(action_array[index], base_action=base_action_array[index])
                        for index, env in enumerate(envs)
                    ]
                else:
                    step_results = list(
                        step_pool.map(
                            lambda item: item[1].step(
                                action_array[item[0]],
                                base_action=base_action_array[item[0]],
                            ),
                            enumerate(envs),
                        )
                    )
                next_observations = []
                for index, (next_obs, reward, done, info) in enumerate(step_results):
                    env = envs[index]
                    reward_buffer[step, index] = reward
                    done_buffer[step, index] = float(done)
                    if done:
                        recent_success.append(float(info.get("success", False)))
                        recent_falls.append(float(info.get("fallen", False)))
                        next_obs = env.reset()
                    next_observations.append(next_obs)
                observations = np.stack(next_observations)
                total_steps += args.envs

            with torch.no_grad():
                next_value = model.critic(
                    torch.as_tensor(observations, device=device)
                ).squeeze(-1).cpu().numpy()
            advantage = np.zeros_like(reward_buffer)
            gae = np.zeros(args.envs, np.float32)
            for step in reversed(range(args.steps_per_env)):
                nonterminal = 1.0 - done_buffer[step]
                following = next_value if step == args.steps_per_env - 1 else value_buffer[step + 1]
                delta = reward_buffer[step] + ppo.gamma * following * nonterminal - value_buffer[step]
                gae = delta + ppo.gamma * ppo.gae_lambda * nonterminal * gae
                advantage[step] = gae
            returns = advantage + value_buffer
            metrics = ppo_update(
                model,
                optimizer,
                torch.as_tensor(obs_buffer.reshape(-1, OBSERVATION_DIM), device=device),
                torch.as_tensor(action_buffer.reshape(-1, 16), device=device),
                torch.as_tensor(log_prob_buffer.reshape(-1), device=device),
                torch.as_tensor(returns.reshape(-1), device=device),
                torch.as_tensor(advantage.reshape(-1), device=device),
                ppo,
                reference_actor=zero_reference,
                anchor_coef=args.zero_anchor_coef,
                actor_frozen=iteration <= args.critic_warmup_iterations,
            )
            eval_success = eval_fall = eval_steps = eval_drop_free = 0.0
            eval_front_to_rear = None
            eval_mean_success_steps = None
            eval_std_success_steps = None
            eval_fastest_success_steps = None
            if iteration % args.eval_interval == 0:
                (
                    eval_success,
                    eval_fall,
                    eval_steps,
                    eval_drop_free,
                    eval_front_to_rear,
                    eval_mean_success_steps,
                    eval_std_success_steps,
                    eval_fastest_success_steps,
                ) = evaluate(
                    model, eval_envs, eval_trials, device
                )
                if args.objective_mode == "speed":
                    current_score = (
                        eval_success,
                        -finite_or(eval_mean_success_steps, 1.0e9),
                        -finite_or(eval_fastest_success_steps, 1.0e9),
                        -finite_or(eval_std_success_steps, 1.0e9),
                        -eval_fall,
                    )
                    best_score = (
                        best_eval_success,
                        -finite_or(best_eval_mean_success_steps, 1.0e9),
                        -finite_or(best_eval_fastest_success_steps, 1.0e9),
                        -finite_or(best_eval_std_success_steps, 1.0e9),
                        -best_eval_fall,
                    )
                else:
                    current_score = (
                        eval_success,
                        -eval_fall,
                        eval_drop_free,
                        -float(
                            1.0e9
                            if eval_front_to_rear is None
                            else eval_front_to_rear
                        ),
                        -eval_steps,
                    )
                    best_score = (
                        best_eval_success,
                        -best_eval_fall,
                        best_eval_drop_free,
                        -float(
                            1.0e9
                            if best_eval_front_to_rear is None
                            else best_eval_front_to_rear
                        ),
                        -best_eval_steps,
                    )
                if current_score > best_score:
                    best_eval_success = eval_success
                    best_eval_fall = eval_fall
                    best_eval_drop_free = eval_drop_free
                    best_eval_front_to_rear = eval_front_to_rear
                    best_eval_mean_success_steps = eval_mean_success_steps
                    best_eval_std_success_steps = eval_std_success_steps
                    best_eval_fastest_success_steps = eval_fastest_success_steps
                    best_eval_steps = eval_steps
                    best_iteration = iteration
                    save_checkpoint(
                        args.output / "best_checkpoint.pt",
                        model,
                        iteration,
                        args,
                        eval_success,
                        eval_fall,
                        eval_drop_free,
                        eval_front_to_rear,
                        eval_mean_success_steps,
                        eval_std_success_steps,
                        eval_fastest_success_steps,
                    )
                    print(
                        f"new best iter={iteration} success={eval_success:.1%} "
                        f"fall={eval_fall:.1%} "
                        f"success_steps={finite_or(eval_mean_success_steps, float('nan')):.1f} "
                        f"fastest={finite_or(eval_fastest_success_steps, float('nan')):.1f}",
                        flush=True,
                    )
                    evals_without_improvement = 0
                else:
                    evals_without_improvement += 1
                consecutive_passes = consecutive_passes + 1 if (
                    eval_success >= 0.95
                    and eval_fall <= 0.05
                    and eval_drop_free >= 0.95
                ) else 0
            elapsed = max(time.monotonic() - started, 1.0e-6)
            row = {
                "iteration": iteration,
                "timesteps": total_steps,
                "reward": float(np.mean(reward_buffer)),
                "success": float(np.mean(recent_success)) if recent_success else 0.0,
                "fall": float(np.mean(recent_falls)) if recent_falls else 0.0,
                "eval_success": eval_success,
                "eval_fall": eval_fall,
                "eval_steps": eval_steps,
                "eval_drop_free": eval_drop_free,
                "eval_front_to_rear_steps": eval_front_to_rear,
                "eval_mean_success_steps": eval_mean_success_steps,
                "eval_std_success_steps": eval_std_success_steps,
                "eval_fastest_success_steps": eval_fastest_success_steps,
                "policy_loss": metrics["policy_loss"],
                "value_loss": metrics["value_loss"],
                "std": float(model.actor.log_std.exp().mean().detach()),
                "fps": total_steps / elapsed,
                "actor_frozen": int(iteration <= args.critic_warmup_iterations),
            }
            writer.writerow(row)
            if iteration == 1 or iteration % 10 == 0 or iteration % args.eval_interval == 0:
                print(
                    f"iter={iteration:05d} reward={row['reward']:.4f} "
                    f"success={row['success']:.1%} fall={row['fall']:.1%} "
                    f"eval={eval_success:.1%} "
                    f"success_steps={finite_or(eval_mean_success_steps, float('nan')):.1f} "
                    f"fastest={finite_or(eval_fastest_success_steps, float('nan')):.1f} "
                    f"std={row['std']:.3f} "
                    f"frozen={row['actor_frozen']} fps={row['fps']:.0f}",
                    flush=True,
                )
            if iteration % args.save_interval == 0 or consecutive_passes >= 2:
                save_checkpoint(
                    args.output / f"checkpoint_{iteration:06d}.pt",
                    model,
                    iteration,
                    args,
                    eval_success,
                    eval_fall,
                    eval_drop_free,
                    eval_front_to_rear,
                    eval_mean_success_steps,
                    eval_std_success_steps,
                    eval_fastest_success_steps,
                )
            if consecutive_passes >= 2:
                print("acceptance reached twice; stopping", flush=True)
                break
            if (
                args.early_stop_evals > 0
                and evals_without_improvement >= args.early_stop_evals
            ):
                early_stop_reason = (
                    f"no improvement for {evals_without_improvement} evaluations"
                )
                print(f"early stopping: {early_stop_reason}", flush=True)
                break
    finally:
        if step_pool is not None:
            step_pool.shutdown(wait=True)
        log_stream.close()
    summary = {
        "iterations": iteration,
        "timesteps": total_steps,
        "consecutive_passes": consecutive_passes,
        "best_iteration": best_iteration,
        "best_eval_success": best_eval_success,
        "best_eval_fall": best_eval_fall,
        "best_eval_drop_free": best_eval_drop_free,
        "best_eval_front_to_rear_steps": best_eval_front_to_rear,
        "best_eval_mean_success_steps": best_eval_mean_success_steps,
        "best_eval_std_success_steps": best_eval_std_success_steps,
        "best_eval_fastest_success_steps": best_eval_fastest_success_steps,
        "best_eval_steps": best_eval_steps,
        "early_stop_reason": early_stop_reason,
        "output": str(args.output.resolve()),
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
