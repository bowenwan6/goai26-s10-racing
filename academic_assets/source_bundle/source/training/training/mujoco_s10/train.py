#!/usr/bin/env python3
"""Train the S10 directly in the official MuJoCo model with PPO.

The optional contest ``policy.onnx`` is used only as a locomotion teacher.  Its actions
are distilled into the PyTorch actor before PPO; PPO then learns the depression task
from real MuJoCo rollouts.  No synthetic reward samples are written.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import random
import time
from collections import deque
from pathlib import Path

import numpy as np
import torch
from torch import nn

from s10_rl.checkpoint import build_actor, extract_actor_state, policy_dimensions
from s10_rl.training_config import PPOConfig

from .config import EnvConfig, PIT_CURRICULUM
from .env import ACTION_ABS_GUARD, S10PitEnv, clone_config
from .network import ActorCritic


def _device(name: str) -> torch.device:
    requested = torch.device(name)
    if requested.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but torch.cuda.is_available() is false")
    return requested


def _expand_first_layer(state: dict[str, torch.Tensor], target_dim: int) -> dict[str, torch.Tensor]:
    copied = {key: value.detach().clone() for key, value in state.items()}
    candidates = [
        key for key, value in copied.items()
        if key.endswith("0.weight") and value.ndim == 2
    ]
    if not candidates:
        raise ValueError("checkpoint has no first linear layer")
    first = min(candidates, key=len)
    source = copied[first]
    if source.shape[1] == target_dim:
        return copied
    if source.shape[1] > target_dim:
        raise ValueError(f"cannot shrink checkpoint input {source.shape[1]} to {target_dim}")
    expanded = source.new_zeros((source.shape[0], target_dim))
    expanded[:, : source.shape[1]] = source
    copied[first] = expanded
    return copied


def load_checkpoint(model: ActorCritic, checkpoint_path: Path, device: torch.device) -> int:
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    actor_state = checkpoint.get("actor_state_dict")
    if not isinstance(actor_state, dict):
        raise ValueError("checkpoint has no actor_state_dict")
    actor_state = _expand_first_layer(actor_state, model.actor.mlp[0].in_features)
    model.actor.load_state_dict(actor_state)

    critic_state = checkpoint.get("critic_state_dict")
    if isinstance(critic_state, dict):
        critic_state = _expand_first_layer(critic_state, model.critic[0].in_features)
        model.critic.load_state_dict(critic_state)
    return int(checkpoint.get("iteration", checkpoint.get("iter", 0)))


def save_checkpoint(
    path: Path,
    model: ActorCritic,
    optimizer: torch.optim.Optimizer,
    iteration: int,
    observation_dim: int,
    curriculum_depth: float,
    metrics: dict[str, float],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "actor_state_dict": model.actor.state_dict(),
            "critic_state_dict": model.critic.state_dict(),
            "optimizer_state_dict": optimizer.state_dict(),
            "iteration": iteration,
            "observation_dim": observation_dim,
            "curriculum_depth": curriculum_depth,
            "metrics": metrics,
        },
        path,
    )


class CsvRunLogger:
    FIELDS = (
        "iteration",
        "timesteps",
        "mean_reward",
        "mean_episode_return",
        "success_rate",
        "fall_rate",
        "eval_success_rate",
        "eval_fall_rate",
        "all_success_rate",
        "all_fall_rate",
        "curriculum_depth",
        "episodes",
        "wall_seconds",
        "expert_bc_loss",
        "expert_bc_only",
        "stabilizing",
        "promotion_passes",
        "policy_lr",
        "policy_std",
        "actor_frozen",
    )

    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = path.open("w", newline="", buffering=1)
        self.writer = csv.DictWriter(self.stream, fieldnames=self.FIELDS)
        self.writer.writeheader()

    def write(self, row: dict[str, float | int]) -> None:
        self.writer.writerow(row)

    def close(self) -> None:
        self.stream.close()


class OnnxExpert:
    """Small CPU ONNX wrapper for phase-specific imitation targets."""

    def __init__(self, path: Path) -> None:
        try:
            import onnxruntime as ort
        except ImportError:
            raise RuntimeError("expert imitation requires onnxruntime") from None
        self.session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        width = self.input_shape[-1]
        if not isinstance(width, int) or width not in (57, 174):
            raise ValueError(f"expert input width is {width}, expected 57 or 174")
        self.input_width = width
        output_width = self.session.get_outputs()[0].shape[-1]
        if isinstance(output_width, int) and output_width != 16:
            raise ValueError(f"expert output width is {output_width}, expected 16")

    def __call__(self, observations: np.ndarray) -> np.ndarray:
        observations = observations[:, : self.input_width].astype(np.float32, copy=False)
        if isinstance(self.input_shape[0], int) and self.input_shape[0] == 1:
            return np.concatenate(
                [
                    self.session.run(
                        [self.output_name], {self.input_name: row[None, :]}
                    )[0]
                    for row in observations
                ],
                axis=0,
            ).astype(np.float32)
        return self.session.run(
            [self.output_name], {self.input_name: observations}
        )[0].astype(np.float32)


class CemTrajectoryExpert:
    """Feedback actor plus a phase-indexed residual from a verified CEM skill.

    The phase is used only to produce on-policy imitation labels.  The student still
    receives the unchanged 174-D deployment observation, so a successful student is
    a normal feed-forward actor rather than an open-loop trajectory player.
    """

    def __init__(
        self,
        trajectory_path: Path,
        checkpoint_path: Path,
        observation_dim: int,
    ) -> None:
        archive = np.load(trajectory_path)
        if "residual_actions" not in archive.files:
            raise ValueError("CEM trajectory has no residual_actions")
        self.residual = np.asarray(archive["residual_actions"], dtype=np.float32)
        if self.residual.ndim != 2 or self.residual.shape[1] != 16:
            raise ValueError("CEM residual_actions must have shape [steps, 16]")

        checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
        actor_state = extract_actor_state(checkpoint)
        if policy_dimensions(actor_state) != (observation_dim, 16):
            raise ValueError(
                f"CEM expert checkpoint dimensions are {policy_dimensions(actor_state)}, "
                f"expected {(observation_dim, 16)}"
            )
        self.actor = build_actor(actor_state).eval()

    @torch.no_grad()
    def __call__(self, observations: np.ndarray, phase_steps: np.ndarray) -> np.ndarray:
        base = self.actor(torch.as_tensor(observations, dtype=torch.float32))
        base_actions = base.cpu().numpy().astype(np.float32)
        phases = np.clip(
            np.asarray(phase_steps, dtype=np.int64), 0, self.residual.shape[0] - 1
        )
        actions = base_actions + self.residual[phases]
        return np.clip(actions, -ACTION_ABS_GUARD, ACTION_ABS_GUARD).astype(np.float32)


def distill_teacher(
    model: ActorCritic,
    envs: list[S10PitEnv],
    observations: np.ndarray,
    teacher_path: Path,
    rollout_steps: int,
    device: torch.device,
) -> np.ndarray:
    """Collect contest-policy actions and fit the actor mean before PPO."""
    try:
        import onnxruntime as ort
    except ImportError:
        raise RuntimeError("teacher distillation requires onnxruntime") from None

    session = ort.InferenceSession(str(teacher_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    input_shape = session.get_inputs()[0].shape
    teacher_width = input_shape[-1]
    if isinstance(teacher_width, int) and teacher_width != 57:
        raise ValueError(f"teacher input width is {teacher_width}, expected 57")

    samples_obs: list[np.ndarray] = []
    samples_action: list[np.ndarray] = []
    for _ in range(rollout_steps):
        teacher_obs = observations[:, :57].astype(np.float32, copy=False)
        # Organizer exports may fix the ONNX batch dimension to one.
        if isinstance(input_shape[0], int) and input_shape[0] == 1 and len(teacher_obs) > 1:
            actions = np.concatenate(
                [
                    session.run([output_name], {input_name: row[None, :]})[0]
                    for row in teacher_obs
                ],
                axis=0,
            ).astype(np.float32)
        else:
            actions = session.run([output_name], {input_name: teacher_obs})[0].astype(np.float32)
        samples_obs.append(observations.copy())
        samples_action.append(actions.copy())
        next_obs = []
        for index, env in enumerate(envs):
            obs, _, done, _ = env.step(actions[index])
            next_obs.append(env.reset() if done else obs)
        observations = np.stack(next_obs)

    x = torch.as_tensor(np.concatenate(samples_obs), device=device)
    y = torch.as_tensor(np.concatenate(samples_action), device=device)
    optimizer = torch.optim.Adam(model.actor.parameters(), lr=1.0e-3)
    batch_size = min(2048, len(x))
    model.train()
    for epoch in range(25):
        permutation = torch.randperm(len(x), device=device)
        losses = []
        for start in range(0, len(x), batch_size):
            indices = permutation[start : start + batch_size]
            loss = nn.functional.mse_loss(model.actor(x[indices]), y[indices])
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(model.actor.parameters(), 1.0)
            optimizer.step()
            losses.append(float(loss.detach()))
        if epoch in (0, 4, 9, 24):
            print(f"teacher epoch={epoch + 1:02d} mse={np.mean(losses):.6f}", flush=True)
    return observations


@torch.no_grad()
def evaluate_deterministic(
    model: ActorCritic,
    envs: list[S10PitEnv],
    pit_depth: float,
    trials: int,
    device: torch.device,
) -> tuple[float, float]:
    """Evaluate the deployed actor mean, not stochastic PPO exploration samples."""
    # Keep monitoring comparable across iterations.  Training RNG streams are
    # advanced by rehearsal and episode length, so they are unsuitable as an eval
    # seed source and made a frozen actor appear to change quality.
    for env in envs:
        env.rng = np.random.default_rng(env.cfg.seed)
    successes = 0
    falls = 0
    completed = 0
    while completed < trials:
        batch_size = min(len(envs), trials - completed)
        batch_envs = envs[:batch_size]
        observations = np.stack([env.reset(pit_depth=pit_depth) for env in batch_envs])
        active = np.ones(batch_size, dtype=bool)
        while np.any(active):
            actions = model.actor(torch.as_tensor(observations, device=device)).cpu().numpy()
            next_observations = observations.copy()
            for index, env in enumerate(batch_envs):
                if not active[index]:
                    continue
                next_obs, _, done, info = env.step(actions[index])
                next_observations[index] = next_obs
                if done:
                    active[index] = False
                    successes += int(info["success"])
                    falls += int(info["fallen"])
            observations = next_observations
        completed += batch_size
    return successes / trials, falls / trials


def load_onnx_teacher_exact(
    model: ActorCritic,
    teacher_path: Path,
    observations: np.ndarray,
    device: torch.device,
    log_std: float,
) -> None:
    """Copy the organizer ONNX MLP exactly and verify numerical parity."""
    try:
        import onnx
        import onnxruntime as ort
        from onnx import numpy_helper
    except ImportError:
        raise RuntimeError("exact ONNX warm-start requires onnx and onnxruntime") from None

    graph = onnx.load(str(teacher_path))
    initializers = {
        item.name: numpy_helper.to_array(item).copy() for item in graph.graph.initializer
    }
    actor_state = model.actor.state_dict()
    linear_keys = [
        key
        for key in actor_state
        if key.startswith("mlp.") and (key.endswith(".weight") or key.endswith(".bias"))
    ]
    missing = [key for key in linear_keys if key not in initializers]
    if missing:
        raise ValueError(f"teacher ONNX is missing actor tensors: {missing}")

    with torch.no_grad():
        for key in linear_keys:
            source = torch.as_tensor(
                initializers[key], dtype=actor_state[key].dtype, device=device
            )
            target = actor_state[key]
            if source.shape == target.shape:
                target.copy_(source)
            elif (
                key == "mlp.0.weight"
                and source.shape[0] == target.shape[0]
                and source.shape[1] < target.shape[1]
            ):
                target.zero_()
                target[:, : source.shape[1]].copy_(source)
            else:
                raise ValueError(
                    f"teacher tensor {key} has shape {tuple(source.shape)}, "
                    f"expected {tuple(target.shape)}"
                )
        model.actor.log_std.fill_(log_std)
    model.actor.load_state_dict(actor_state)

    session = ort.InferenceSession(str(teacher_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    probe = observations[: min(4, len(observations)), :57].astype(np.float32, copy=False)
    expected = np.concatenate(
        [session.run([output_name], {input_name: row[None, :]})[0] for row in probe], axis=0
    )
    with torch.no_grad():
        actual = model.actor(torch.as_tensor(observations[: len(probe)], device=device)).cpu().numpy()
    max_error = float(np.max(np.abs(actual - expected)))
    if max_error > 1.0e-5:
        raise RuntimeError(
            f"exact ONNX warm-start parity failed: max_abs_error={max_error:.3g}"
        )
    print(
        f"loaded exact ONNX teacher: max_abs_error={max_error:.3g}, "
        f"exploration_std={np.exp(log_std):.4f}",
        flush=True,
    )


def ppo_update(
    model: ActorCritic,
    optimizer: torch.optim.Optimizer,
    obs: torch.Tensor,
    actions: torch.Tensor,
    old_log_prob: torch.Tensor,
    returns: torch.Tensor,
    advantages: torch.Tensor,
    cfg: PPOConfig,
    *,
    reference_actor: nn.Module | None = None,
    anchor_coef: float = 0.0,
    actor_frozen: bool = False,
    expert_actions: torch.Tensor | None = None,
    expert_mask: torch.Tensor | None = None,
    expert_bc_coef: float = 0.0,
    ppo_actor_enabled: bool = True,
) -> dict[str, float]:
    count = obs.shape[0]
    mini_batch = max(1, count // cfg.mini_batches)
    policy_losses: list[float] = []
    value_losses: list[float] = []
    entropies: list[float] = []
    anchor_losses: list[float] = []
    expert_bc_losses: list[float] = []
    advantages = (advantages - advantages.mean()) / (advantages.std() + 1.0e-8)
    actor_parameters = list(model.actor.parameters())
    if actor_frozen:
        for parameter in actor_parameters:
            parameter.requires_grad_(False)
    try:
        for _ in range(cfg.learning_epochs):
            permutation = torch.randperm(count, device=obs.device)
            for start in range(0, count, mini_batch):
                indices = permutation[start : start + mini_batch]
                batch_obs = obs[indices]
                log_prob, entropy, value = model.evaluate(batch_obs, actions[indices])
                ratio = torch.exp(log_prob - old_log_prob[indices])
                unclipped = ratio * advantages[indices]
                clipped = torch.clamp(ratio, 1.0 - cfg.clip_param, 1.0 + cfg.clip_param) * advantages[indices]
                policy_loss = -torch.minimum(unclipped, clipped).mean()
                value_loss = nn.functional.mse_loss(value, returns[indices])
                entropy_mean = entropy.mean()
                anchor_loss = torch.zeros((), device=obs.device)
                if (
                    reference_actor is not None
                    and anchor_coef > 0.0
                    and not actor_frozen
                    and ppo_actor_enabled
                ):
                    with torch.no_grad():
                        reference_mean = reference_actor(batch_obs)
                    anchor_loss = nn.functional.mse_loss(model.actor(batch_obs), reference_mean)
                expert_bc_loss = torch.zeros((), device=obs.device)
                if (
                    expert_actions is not None
                    and expert_mask is not None
                    and expert_bc_coef > 0.0
                    and not actor_frozen
                ):
                    batch_expert_mask = expert_mask[indices]
                    if bool(torch.any(batch_expert_mask)):
                        expert_bc_loss = nn.functional.mse_loss(
                            model.actor(batch_obs[batch_expert_mask]),
                            expert_actions[indices][batch_expert_mask],
                        )
                loss = (
                    (0.0 if actor_frozen or not ppo_actor_enabled else 1.0) * policy_loss
                    + 0.5 * value_loss
                    - (
                        0.0 if actor_frozen or not ppo_actor_enabled else cfg.entropy_coef
                    ) * entropy_mean
                    + anchor_coef * anchor_loss
                    + expert_bc_coef * expert_bc_loss
                )
                optimizer.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), 1.0)
                optimizer.step()
                with torch.no_grad():
                    model.actor.log_std.clamp_(-5.0, 0.0)
                policy_losses.append(float(policy_loss.detach()))
                value_losses.append(float(value_loss.detach()))
                entropies.append(float(entropy_mean.detach()))
                anchor_losses.append(float(anchor_loss.detach()))
                expert_bc_losses.append(float(expert_bc_loss.detach()))
    finally:
        if actor_frozen:
            for parameter in actor_parameters:
                parameter.requires_grad_(True)
    return {
        "policy_loss": float(np.mean(policy_losses)),
        "value_loss": float(np.mean(value_losses)),
        "entropy": float(np.mean(entropies)),
        "anchor_loss": float(np.mean(anchor_losses)),
        "expert_bc_loss": float(np.mean(expert_bc_losses)),
    }


def train(args: argparse.Namespace) -> None:
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = _device(args.device)
    if device.type == "cuda":
        torch.cuda.manual_seed_all(args.seed)

    ppo = PPOConfig(learning_rate=args.learning_rate)
    target_stage = max(index for index, depth in enumerate(PIT_CURRICULUM) if depth <= args.target_depth)
    stage = min(
        range(len(PIT_CURRICULUM)),
        key=lambda index: abs(PIT_CURRICULUM[index] - args.start_depth),
    )
    stage = min(stage, target_stage)
    # BC-only warm-up is stage-local.  A global iteration threshold silently
    # expired before the 20 cm expert stage in 014, so PPO competed with the
    # demonstrated rear-push action as soon as that stage began.
    stage_start_iteration = 0
    env_config = EnvConfig(
        xml_path=args.xml,
        observation_dim=args.observation_dim,
        pit_depth=PIT_CURRICULUM[stage],
        command_forward=args.command_forward,
        episode_seconds=args.episode_seconds,
        pit_length_min=args.pit_length_min,
        pit_length_max=args.pit_length_max,
        reset_mode=args.reset_mode,
        trace_seed_probability=args.trace_seed_probability,
        exit_approach_min=args.exit_approach_min,
        exit_approach_max=args.exit_approach_max,
        reset_lateral_range=args.reset_lateral_range,
        reset_yaw_range=args.reset_yaw_range,
        seed=args.seed,
    )
    envs = [S10PitEnv(clone_config(env_config, seed=args.seed + 1009 * i)) for i in range(args.envs)]
    eval_envs = [
        S10PitEnv(clone_config(env_config, seed=args.seed + 900_000 + 1009 * i))
        for i in range(min(args.envs, 8))
    ]
    observations = np.stack([env.reset() for env in envs])

    model = ActorCritic(
        args.observation_dim,
        actor_hidden=ppo.actor_hidden_dims,
        critic_hidden=ppo.critic_hidden_dims,
    ).to(device)
    start_iteration = 0
    if args.resume is not None:
        start_iteration = load_checkpoint(model, args.resume, device)
        stage_start_iteration = start_iteration
        print(f"loaded {args.resume} at iteration {start_iteration}", flush=True)
    elif args.teacher_onnx is not None:
        load_onnx_teacher_exact(
            model,
            args.teacher_onnx,
            observations,
            device,
            args.teacher_log_std,
        )

    reference_actor = None
    if args.teacher_onnx is not None:
        reference_actor = copy.deepcopy(model.actor).to(device).eval()
        for parameter in reference_actor.parameters():
            parameter.requires_grad_(False)

    if args.expert_trajectory is not None:
        expert = CemTrajectoryExpert(
            args.expert_trajectory,
            args.expert_checkpoint,
            args.observation_dim,
        )
    else:
        expert = OnnxExpert(args.expert_onnx) if args.expert_onnx is not None else None
    if expert is not None:
        expert_source = (
            f"trajectory={args.expert_trajectory}, checkpoint={args.expert_checkpoint}"
            if args.expert_trajectory is not None
            else str(args.expert_onnx)
        )
        print(
            f"loaded phase expert {expert_source}; trigger={args.expert_trigger}, "
            f"depth={args.expert_min_depth:.3f}-{args.expert_max_depth:.3f} m",
            flush=True,
        )

    optimizer = torch.optim.Adam(model.parameters(), lr=ppo.learning_rate)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "run_config.json").write_text(
        json.dumps(vars(args), default=str, indent=2) + "\n", encoding="utf-8"
    )
    logger = CsvRunLogger(args.output / "metrics.csv")
    if args.teacher_onnx is not None:
        save_checkpoint(
            args.output / "teacher_warmstart.pt",
            model,
            optimizer,
            0,
            args.observation_dim,
            PIT_CURRICULUM[stage],
            {"mean_reward": 0.0, "mean_episode_return": 0.0, "success_rate": 0.0, "fall_rate": 0.0},
        )
    recent_returns: deque[float] = deque(maxlen=100)
    recent_success: deque[float] = deque(maxlen=100)
    recent_falls: deque[float] = deque(maxlen=100)
    recent_current_success: deque[float] = deque(maxlen=100)
    recent_current_falls: deque[float] = deque(maxlen=100)
    best_success = -1.0
    last_eval_success = 0.0
    last_eval_fall = 0.0
    consecutive_eval_failures = 0
    consecutive_promotion_passes = 0
    consecutive_stabilization_failures = 0
    stage_stabilizing = bool(args.start_stabilizing)
    start_time = time.monotonic()
    total_steps = 0

    rollout_shape = (ppo.steps_per_env, args.envs)
    try:
        for iteration in range(start_iteration + 1, args.iterations + 1):
            # Preserve the fragile contest locomotion teacher on the easy stages,
            # then expose progressively more exploration only as the obstacle grows.
            # Resetting this before every rollout prevents entropy optimization from
            # silently destroying the teacher while still giving the terminal stages
            # enough action diversity to discover a climb.
            if reference_actor is not None:
                active_log_std_increment = (
                    args.stabilization_log_std_increment
                    if stage_stabilizing
                    else args.log_std_increment
                )
                scheduled_log_std = min(
                    args.exploration_log_std_ceiling,
                    args.teacher_log_std + active_log_std_increment * stage,
                )
                with torch.no_grad():
                    model.actor.log_std.fill_(scheduled_log_std)
            active_learning_rate = args.learning_rate * (
                args.stabilization_learning_rate_scale if stage_stabilizing else 1.0
            )
            for parameter_group in optimizer.param_groups:
                parameter_group["lr"] = active_learning_rate
            obs_buffer = np.empty((*rollout_shape, args.observation_dim), dtype=np.float32)
            action_buffer = np.empty((*rollout_shape, 16), dtype=np.float32)
            reward_buffer = np.empty(rollout_shape, dtype=np.float32)
            done_buffer = np.empty(rollout_shape, dtype=np.float32)
            value_buffer = np.empty(rollout_shape, dtype=np.float32)
            log_prob_buffer = np.empty(rollout_shape, dtype=np.float32)
            expert_action_buffer = (
                np.zeros((*rollout_shape, 16), dtype=np.float32)
                if expert is not None
                else None
            )
            expert_mask_buffer = (
                np.zeros(rollout_shape, dtype=bool) if expert is not None else None
            )
            completed_returns: list[float] = []

            for step in range(ppo.steps_per_env):
                obs_tensor = torch.as_tensor(observations, device=device)
                action_tensor, log_prob, value = model.act(obs_tensor)
                actions = action_tensor.cpu().numpy().astype(np.float32)
                obs_buffer[step] = observations
                action_buffer[step] = actions
                value_buffer[step] = value.cpu().numpy()
                log_prob_buffer[step] = log_prob.cpu().numpy()
                if expert is not None:
                    def expert_active(env: S10PitEnv) -> bool:
                        in_depth = (
                            args.expert_min_depth
                            <= env.pit_depth
                            <= args.expert_max_depth
                        )
                        if args.expert_trigger == "front_wheels":
                            return in_depth and env.right_wheels_on_exit() >= 2
                        return in_depth and env.cfg.reset_mode == "front_up"

                    expert_mask = np.asarray(
                        [expert_active(env) for env in envs],
                        dtype=bool,
                    )
                    expert_mask_buffer[step] = expert_mask
                    if np.any(expert_mask):
                        if isinstance(expert, CemTrajectoryExpert):
                            expert_action_buffer[step, expert_mask] = expert(
                                observations[expert_mask],
                                np.asarray(
                                    [env.step_count for env in envs], dtype=np.int64
                                )[expert_mask],
                            )
                        else:
                            expert_action_buffer[step, expert_mask] = expert(
                                observations[expert_mask]
                            )

                next_observations = []
                for index, env in enumerate(envs):
                    next_obs, reward, done, info = env.step(actions[index])
                    reward_buffer[step, index] = reward
                    done_buffer[step, index] = float(done)
                    if done:
                        episode_return = float(info["episode_return"])
                        completed_returns.append(episode_return)
                        recent_returns.append(episode_return)
                        recent_success.append(float(info["success"]))
                        recent_falls.append(float(info["fallen"]))
                        if abs(float(info["pit_depth"]) - PIT_CURRICULUM[stage]) < 1.0e-9:
                            recent_current_success.append(float(info["success"]))
                            recent_current_falls.append(float(info["fallen"]))

                        # Keep the stage distribution explicit.  Near a new geometry
                        # boundary the old hard-coded 60% current-stage share let
                        # easier rehearsal samples dominate the PPO gradient even
                        # though promotion statistics themselves were correct.
                        draw = env.rng.random()
                        current_probability = (
                            max(0.80, args.current_stage_probability)
                            if stage == 0
                            else args.current_stage_probability
                        )
                        if draw < current_probability:
                            reset_depth = PIT_CURRICULUM[stage]
                        elif (
                            stage > 0
                            and draw
                            < current_probability + args.previous_stage_probability
                        ):
                            reset_depth = PIT_CURRICULUM[stage - 1]
                        else:
                            reset_depth = 0.0
                        next_obs = env.reset(pit_depth=reset_depth)
                    next_observations.append(next_obs)
                observations = np.stack(next_observations)
                total_steps += args.envs

            with torch.no_grad():
                next_value = model.critic(torch.as_tensor(observations, device=device)).squeeze(-1).cpu().numpy()
            advantage = np.zeros_like(reward_buffer)
            last_gae = np.zeros(args.envs, dtype=np.float32)
            for step in reversed(range(ppo.steps_per_env)):
                nonterminal = 1.0 - done_buffer[step]
                following = next_value if step == ppo.steps_per_env - 1 else value_buffer[step + 1]
                delta = reward_buffer[step] + ppo.gamma * following * nonterminal - value_buffer[step]
                last_gae = delta + ppo.gamma * ppo.gae_lambda * nonterminal * last_gae
                advantage[step] = last_gae
            returns = advantage + value_buffer

            flat = lambda array: torch.as_tensor(array.reshape((-1, *array.shape[2:])), device=device)
            anchor_fraction = max(
                0.05,
                1.0 - stage / max(1, target_stage),
            )
            expert_stage_active = (
                expert is not None
                and args.expert_min_depth
                <= PIT_CURRICULUM[stage]
                <= args.expert_max_depth
            )
            stage_bc_only = (
                expert_stage_active
                and iteration - stage_start_iteration
                <= args.expert_bc_only_iterations
            )
            stage_actor_frozen = (
                iteration <= args.actor_freeze_iterations
                or iteration - stage_start_iteration <= args.stage_freeze_iterations
            )
            update_metrics = ppo_update(
                model,
                optimizer,
                flat(obs_buffer),
                flat(action_buffer),
                torch.as_tensor(log_prob_buffer.reshape(-1), device=device),
                torch.as_tensor(returns.reshape(-1), device=device),
                torch.as_tensor(advantage.reshape(-1), device=device),
                ppo,
                reference_actor=reference_actor,
                anchor_coef=(
                    args.teacher_anchor_coef
                    * anchor_fraction
                    * (
                        args.stabilization_anchor_scale
                        if stage_stabilizing
                        else 1.0
                    )
                ),
                actor_frozen=stage_actor_frozen,
                expert_actions=(
                    flat(expert_action_buffer) if expert_action_buffer is not None else None
                ),
                expert_mask=(
                    torch.as_tensor(expert_mask_buffer.reshape(-1), device=device)
                    if expert_mask_buffer is not None
                    else None
                ),
                expert_bc_coef=args.expert_bc_coef,
                ppo_actor_enabled=not stage_bc_only,
            )

            did_eval = iteration % args.eval_interval == 0
            if did_eval:
                last_eval_success, last_eval_fall = evaluate_deterministic(
                    model,
                    eval_envs,
                    PIT_CURRICULUM[stage],
                    args.eval_trials,
                    device,
                )
                # Rehearsal episodes may belong to easier stages.  Start the next
                # rollout from a clean current-stage batch after every eval.
                observations = np.stack(
                    [env.reset(pit_depth=PIT_CURRICULUM[stage]) for env in envs]
                )
                print(
                    f"deterministic eval depth={PIT_CURRICULUM[stage]:.3f} "
                    f"success={last_eval_success:.1%} fall={last_eval_fall:.1%} "
                    f"trials={args.eval_trials}",
                    flush=True,
                )

            if did_eval:
                if last_eval_success >= args.promote_success and last_eval_fall <= 0.10:
                    consecutive_promotion_passes += 1
                else:
                    consecutive_promotion_passes = 0

            all_success_rate = float(np.mean(recent_success)) if recent_success else 0.0
            all_fall_rate = float(np.mean(recent_falls)) if recent_falls else 0.0
            success_rate = (
                float(np.mean(recent_current_success)) if recent_current_success else 0.0
            )
            fall_rate = float(np.mean(recent_current_falls)) if recent_current_falls else 0.0
            mean_episode_return = float(np.mean(recent_returns)) if recent_returns else 0.0
            mean_reward = float(np.mean(reward_buffer))
            elapsed = time.monotonic() - start_time
            row = {
                "iteration": iteration,
                "timesteps": total_steps,
                "mean_reward": mean_reward,
                "mean_episode_return": mean_episode_return,
                "success_rate": success_rate,
                "fall_rate": fall_rate,
                "eval_success_rate": last_eval_success,
                "eval_fall_rate": last_eval_fall,
                "all_success_rate": all_success_rate,
                "all_fall_rate": all_fall_rate,
                "curriculum_depth": PIT_CURRICULUM[stage],
                "episodes": len(recent_returns),
                "wall_seconds": elapsed,
                "expert_bc_loss": update_metrics["expert_bc_loss"],
                "expert_bc_only": int(stage_bc_only),
                "stabilizing": int(stage_stabilizing),
                "promotion_passes": consecutive_promotion_passes,
                "policy_lr": active_learning_rate,
                "policy_std": float(model.actor.log_std.exp().mean().detach()),
                "actor_frozen": int(stage_actor_frozen),
            }
            logger.write(row)

            if iteration == 1 or iteration % args.log_interval == 0:
                steps_per_second = total_steps / max(elapsed, 1.0e-6)
                print(
                    f"iter={iteration:05d} depth={PIT_CURRICULUM[stage]:.3f} "
                    f"reward={mean_reward:.5f} return={mean_episode_return:.3f} "
                    f"success={success_rate:.1%} fall={fall_rate:.1%} "
                    f"eval={last_eval_success:.1%} "
                    f"episodes={len(recent_returns)} fps={steps_per_second:.0f} "
                    f"ploss={update_metrics['policy_loss']:.4f} "
                    f"anchor={update_metrics['anchor_loss']:.5f} "
                    f"expert_bc={update_metrics['expert_bc_loss']:.5f} "
                    f"bc_only={int(stage_bc_only)} "
                    f"stabilizing={int(stage_stabilizing)} "
                    f"promote_passes={consecutive_promotion_passes}/{args.promotion_evals} "
                    f"lr={active_learning_rate:.2e} "
                    f"frozen={int(stage_actor_frozen)} "
                    f"std={float(model.actor.log_std.exp().mean().detach()):.4f}",
                    flush=True,
                )

            metrics = {
                "mean_reward": mean_reward,
                "mean_episode_return": mean_episode_return,
                "success_rate": success_rate,
                "fall_rate": fall_rate,
                "eval_success_rate": last_eval_success,
                "eval_fall_rate": last_eval_fall,
            }
            if iteration % args.save_interval == 0:
                save_checkpoint(
                    args.output / f"checkpoint_{iteration:06d}.pt",
                    model,
                    optimizer,
                    iteration,
                    args.observation_dim,
                    PIT_CURRICULUM[stage],
                    metrics,
                )
            if did_eval and last_eval_success > best_success:
                best_success = last_eval_success
                depth_tag = f"{PIT_CURRICULUM[stage]:.3f}".replace(".", "p")
                for best_path in (
                    args.output / "best.pt",
                    args.output / f"best_depth_{depth_tag}.pt",
                ):
                    save_checkpoint(
                        best_path,
                        model,
                        optimizer,
                        iteration,
                        args.observation_dim,
                        PIT_CURRICULUM[stage],
                        metrics,
                    )

            if (
                did_eval
                and not stage_stabilizing
                and last_eval_success >= args.stabilize_success
                and last_eval_fall <= 0.10
            ):
                stage_stabilizing = True
                print(
                    f"stage {PIT_CURRICULUM[stage]:.3f} m entering stabilization: "
                    f"eval success={last_eval_success:.1%}",
                    flush=True,
                )

            if did_eval and stage_stabilizing:
                if last_eval_success < args.stabilize_success:
                    consecutive_stabilization_failures += 1
                else:
                    consecutive_stabilization_failures = 0
                if (
                    args.rediscover_evals > 0
                    and consecutive_stabilization_failures >= args.rediscover_evals
                    and iteration - stage_start_iteration > args.stage_freeze_iterations
                ):
                    stage_stabilizing = False
                    consecutive_stabilization_failures = 0
                    print(
                        f"stage {PIT_CURRICULUM[stage]:.3f} m returning to discovery",
                        flush=True,
                    )

            if (
                did_eval
                and stage < target_stage
                and consecutive_promotion_passes >= args.promotion_evals
            ):
                stage += 1
                stage_start_iteration = iteration
                recent_returns.clear()
                recent_success.clear()
                recent_falls.clear()
                recent_current_success.clear()
                recent_current_falls.clear()
                best_success = -1.0
                last_eval_success = 0.0
                last_eval_fall = 0.0
                consecutive_eval_failures = 0
                consecutive_promotion_passes = 0
                consecutive_stabilization_failures = 0
                stage_stabilizing = bool(args.smooth_promotions)
                observations = np.stack(
                    [env.reset(pit_depth=PIT_CURRICULUM[stage]) for env in envs]
                )
                print(f"curriculum promoted to {PIT_CURRICULUM[stage]:.3f} m", flush=True)
            elif did_eval and stage > 0:
                if last_eval_success < 0.50:
                    consecutive_eval_failures += 1
                else:
                    consecutive_eval_failures = 0
                if consecutive_eval_failures >= args.demote_evals:
                    stage -= 1
                    stage_start_iteration = iteration
                    recent_returns.clear()
                    recent_success.clear()
                    recent_falls.clear()
                    recent_current_success.clear()
                    recent_current_falls.clear()
                    best_success = -1.0
                    last_eval_success = 0.0
                    last_eval_fall = 0.0
                    consecutive_eval_failures = 0
                    consecutive_promotion_passes = 0
                    consecutive_stabilization_failures = 0
                    stage_stabilizing = False
                    observations = np.stack(
                        [env.reset(pit_depth=PIT_CURRICULUM[stage]) for env in envs]
                    )
                    print(f"curriculum demoted to {PIT_CURRICULUM[stage]:.3f} m", flush=True)
    finally:
        final_iteration = iteration if "iteration" in locals() else start_iteration
        final_metrics = {
            "mean_reward": float(np.mean(reward_buffer)) if "reward_buffer" in locals() else 0.0,
            "mean_episode_return": float(np.mean(recent_returns)) if recent_returns else 0.0,
            "success_rate": last_eval_success,
            "fall_rate": last_eval_fall,
        }
        save_checkpoint(
            args.output / "last.pt",
            model,
            optimizer,
            final_iteration,
            args.observation_dim,
            PIT_CURRICULUM[stage],
            final_metrics,
        )
        logger.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", type=Path, required=True, help="official S10.xml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--observation-dim", type=int, choices=(57, 174), default=57)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--envs", type=int, default=16)
    parser.add_argument("--iterations", type=int, default=2000)
    parser.add_argument("--learning-rate", type=float, default=3.0e-4)
    parser.add_argument("--command-forward", type=float, default=0.6)
    parser.add_argument("--episode-seconds", type=float, default=10.0)
    parser.add_argument("--pit-length-min", type=float, default=2.25)
    parser.add_argument("--pit-length-max", type=float, default=2.25)
    parser.add_argument(
        "--reset-mode", choices=("full_pit", "exit_only", "front_up"), default="full_pit"
    )
    parser.add_argument("--trace-seed-probability", type=float, default=0.0)
    parser.add_argument("--exit-approach-min", type=float, default=0.80)
    parser.add_argument("--exit-approach-max", type=float, default=1.20)
    parser.add_argument("--reset-lateral-range", type=float, default=0.08)
    parser.add_argument("--reset-yaw-range", type=float, default=0.05)
    parser.add_argument("--start-depth", type=float, default=0.05)
    parser.add_argument("--target-depth", type=float, default=0.377)
    parser.add_argument("--promote-success", type=float, default=0.80)
    parser.add_argument("--promotion-evals", type=int, default=1)
    parser.add_argument(
        "--current-stage-probability",
        type=float,
        default=0.60,
        help="fraction of PPO resets at the active curriculum height",
    )
    parser.add_argument(
        "--previous-stage-probability",
        type=float,
        default=0.20,
        help="fraction of PPO resets rehearsing the previous height",
    )
    parser.add_argument(
        "--stabilize-success",
        type=float,
        default=1.01,
        help="switch a stage from discovery to stabilization at this eval success",
    )
    parser.add_argument("--stabilization-learning-rate-scale", type=float, default=1.0)
    parser.add_argument("--stabilization-log-std-increment", type=float, default=0.0)
    parser.add_argument("--stabilization-anchor-scale", type=float, default=1.0)
    parser.add_argument("--rediscover-evals", type=int, default=0)
    parser.add_argument("--start-stabilizing", action="store_true")
    parser.add_argument(
        "--smooth-promotions",
        action="store_true",
        help="keep low-noise stabilization settings after curriculum promotion",
    )
    parser.add_argument("--teacher-onnx", type=Path)
    parser.add_argument("--bc-rollout-steps", type=int, default=0)
    parser.add_argument("--teacher-log-std", type=float, default=-2.5)
    parser.add_argument("--exploration-log-std-ceiling", type=float, default=-2.0)
    parser.add_argument("--log-std-increment", type=float, default=0.20)
    parser.add_argument("--actor-freeze-iterations", type=int, default=100)
    parser.add_argument("--stage-freeze-iterations", type=int, default=0)
    parser.add_argument("--teacher-anchor-coef", type=float, default=1.0)
    parser.add_argument("--expert-onnx", type=Path)
    parser.add_argument(
        "--expert-trajectory",
        type=Path,
        help="verified CEM trajectory used as an on-policy phase-label expert",
    )
    parser.add_argument(
        "--expert-checkpoint",
        type=Path,
        help="feedback actor checkpoint paired with --expert-trajectory",
    )
    parser.add_argument("--expert-min-depth", type=float, default=0.15)
    parser.add_argument("--expert-max-depth", type=float, default=0.20)
    parser.add_argument(
        "--expert-trigger",
        choices=("front_up", "front_wheels"),
        default="front_up",
    )
    parser.add_argument("--expert-bc-coef", type=float, default=0.0)
    parser.add_argument("--expert-bc-only-iterations", type=int, default=0)
    parser.add_argument("--eval-interval", type=int, default=50)
    parser.add_argument("--eval-trials", type=int, default=16)
    parser.add_argument("--demote-evals", type=int, default=10)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--save-interval", type=int, default=100)
    parser.add_argument("--log-interval", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if min(
        args.envs,
        args.iterations,
        args.eval_interval,
        args.eval_trials,
        args.demote_evals,
        args.promotion_evals,
    ) < 1:
        parser.error(
            "envs, iterations, eval interval, eval trials, demote evals and "
            "promotion evals must be positive"
        )
    if args.log_std_increment < 0.0 or args.stabilization_log_std_increment < 0.0:
        parser.error("log std increments must be non-negative")
    if not 0.0 < args.stabilization_learning_rate_scale <= 1.0:
        parser.error("stabilization learning-rate scale must be in (0, 1]")
    if (
        not 0.0 < args.current_stage_probability <= 1.0
        or not 0.0 <= args.previous_stage_probability <= 1.0
        or args.current_stage_probability + args.previous_stage_probability > 1.0
    ):
        parser.error(
            "current-stage and previous-stage probabilities must be valid and sum to <= 1"
        )
    if args.stabilization_anchor_scale <= 0.0:
        parser.error("stabilization anchor scale must be positive")
    if args.rediscover_evals < 0:
        parser.error("rediscover evals must be non-negative")
    if not 0.0 <= args.stabilize_success <= 1.01:
        parser.error("stabilize success must be in [0, 1.01]")
    if (
        args.actor_freeze_iterations < 0
        or args.stage_freeze_iterations < 0
        or args.teacher_anchor_coef < 0.0
    ):
        parser.error(
            "actor freeze iterations and teacher anchor coefficient must be non-negative"
        )
    if args.expert_bc_coef < 0.0 or args.expert_bc_only_iterations < 0:
        parser.error("expert BC coefficient and BC-only iterations must be non-negative")
    if args.expert_min_depth < 0.0 or args.expert_max_depth < args.expert_min_depth:
        parser.error("invalid expert depth range")
    if not 0.0 <= args.trace_seed_probability <= 1.0:
        parser.error("trace seed probability must be in [0, 1]")
    if args.expert_onnx is not None and args.expert_trajectory is not None:
        parser.error("use either --expert-onnx or --expert-trajectory, not both")
    if (args.expert_trajectory is None) != (args.expert_checkpoint is None):
        parser.error("--expert-trajectory and --expert-checkpoint must be used together")
    if args.expert_onnx is None and args.expert_trajectory is None and (
        args.expert_bc_coef > 0.0 or args.expert_bc_only_iterations > 0
    ):
        parser.error("expert BC options require an ONNX or CEM trajectory expert")
    if args.episode_seconds <= 0.0:
        parser.error("episode seconds must be positive")
    if args.pit_length_min <= 0.0 or args.pit_length_max < args.pit_length_min:
        parser.error("invalid pit length range")
    if args.exit_approach_min <= 0.0 or args.exit_approach_max < args.exit_approach_min:
        parser.error("invalid exit approach range")
    if args.reset_lateral_range < 0.0 or args.reset_yaw_range < 0.0:
        parser.error("reset lateral and yaw ranges must be non-negative")
    if args.target_depth < min(PIT_CURRICULUM):
        parser.error("target depth is below the curriculum")
    return args


if __name__ == "__main__":
    train(parse_args())
