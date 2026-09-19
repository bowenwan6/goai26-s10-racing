#!/usr/bin/env python3
"""Repair a Gate-16 residual actor with successful traces and failure-state DAgger.

The student starts from an existing residual checkpoint, not from zero.  Successful
full-action demonstrations are converted back into normalized residual targets under the
same frozen base actor.  Initial-policy rollouts rehearse the existing behaviour, while
states visited during failed student rollouts receive nearest-success labels.  Every
cycle is evaluated on one immutable distance/lateral/yaw grid; only a strictly better
checkpoint is selected.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import torch
from torch import nn

from s10_rl.checkpoint import (
    build_actor,
    extract_actor_state,
    linear_layer_keys,
)

from .distill_cem import _infer, _load_inference_actor, train_epoch_group
from .distill_official_skill import NearestDemoTeacher, load_demonstrations
from .gate16_entry_cases import entry_cases, load_navigation_cases
from .front_retention import summarize_retention
from .official_policy_env import FULL_CORRECTION_SCALE, OfficialClosedLoopResidualEnv


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@torch.no_grad()
def infer(actor: nn.Module, observations: np.ndarray) -> np.ndarray:
    device = next(actor.parameters()).device
    tensor = torch.as_tensor(observations, dtype=torch.float32, device=device)
    return actor(tensor).cpu().numpy().astype(np.float32)


def demonstration_targets(
    paths: list[Path],
    base_actor: nn.Module,
    *,
    correction_limit: float,
    critical_repeat: int,
    critical_x: float,
) -> tuple[np.ndarray, np.ndarray, list[str]]:
    observations, actions, accepted = load_demonstrations(
        paths,
        critical_repeat=critical_repeat,
        critical_x=critical_x,
    )
    base_actions = _infer(base_actor, observations)
    targets = (actions - base_actions) / FULL_CORRECTION_SCALE
    targets = np.clip(targets, -correction_limit, correction_limit).astype(np.float32)
    return observations, targets, accepted


def make_env(xml, base_actor, speed, distance_range, args, seed):
    return OfficialClosedLoopResidualEnv(
        xml,
        base_actor,
        seed=seed,
        approach_speed=speed,
        activate_distance=args.activate_distance,
        distance_range=distance_range,
        lateral_range=0.0,
        yaw_range=0.0,
        height_noise=0.0,
        correction_scale=FULL_CORRECTION_SCALE,
        correction_limit=args.correction_limit,
        activation_mode=args.activation_mode,
    )


def rollout_case(env, actor, case):
    observation = env.reset_entry(
        distance=case["distance"],
        lateral=case["lateral"],
        yaw=case["yaw_rad"],
        entry_center_y=case.get("entry_center_y", 33.365),
        command_forward=case["speed"],
        command_lateral=case.get("cmd_lateral", 0.0),
        command_yaw=case.get("cmd_yaw"),
        forward_speed=case.get("forward_speed", 0.0),
        lateral_speed=case.get("lateral_speed", 0.0),
        yaw_rate=case.get("yaw_rate", 0.0),
    )
    active_observations: list[np.ndarray] = []
    active_actions: list[np.ndarray] = []
    phase_observations: list[np.ndarray] = []
    info: dict[str, object] = {"success": False, "fallen": False}
    done = False
    while not done:
        correction = infer(actor, observation[None, :])[0]
        current = observation.copy()
        observation, _, done, info = env.step(correction)
        if info.get("skill_active", False):
            active_observations.append(current)
            active_actions.append(correction.copy())
        if info.get("rear_push_phase") in {"settle", "push"}:
            phase_observations.append(current)
    return (
        np.asarray(active_observations, dtype=np.float32),
        np.asarray(active_actions, dtype=np.float32),
        np.asarray(phase_observations, dtype=np.float32),
        info,
        env.step_count,
    )


def evaluate_grid(xml, base_actor, actor, cases, args, cycle):
    # The formal collector evaluates a CPU actor and reuses one environment per
    # command speed.  Gate-16 trajectories are sensitive enough that CUDA rounding
    # or a different environment lifecycle can change terminal outcomes, so keep
    # this in-training gate bit-for-bit aligned with the formal evaluation path.
    evaluation_actor = copy.deepcopy(actor).cpu().eval()
    distance_range = (
        min(float(case["distance"]) for case in cases),
        max(float(case["distance"]) for case in cases),
    )
    env_by_speed = {
        speed: make_env(
            xml,
            base_actor,
            speed,
            distance_range,
            args,
            args.seed + index * 1009,
        )
        for index, speed in enumerate(
            sorted({float(case["speed"]) for case in cases})
        )
    }
    rows = []
    failed_observations = []
    active_observations = []
    active_actions = []
    for index, case in enumerate(cases, start=1):
        env = env_by_speed[float(case["speed"])]
        observations, actions, phase_observations, info, steps = rollout_case(
            env, evaluation_actor, case
        )
        success = bool(info.get("success", False))
        if observations.size:
            active_observations.append(observations)
            active_actions.append(actions)
            if not success:
                # Fast demonstrations cover only the front-supported rear-push phase.
                # Nearest-neighbour labels outside that phase are not meaningful.
                if phase_observations.size:
                    failed_observations.append(phase_observations)
        rows.append(
            {
                "case": index,
                **case,
                "success": int(success),
                "fallen": int(bool(info.get("fallen", False))),
                "right_wheels": int(info.get("right_wheels", 0)),
                "front_drop_count": int(info.get("front_drop_count", 0)),
                "front_support_steps": int(info.get("front_support_steps", 0)),
                "first_front_step": info.get("first_front_step"),
                "first_rear_step": info.get("first_rear_step"),
                "front_to_rear_steps": info.get("front_to_rear_steps"),
                "drop_free": int(bool(info.get("drop_free", True))),
                "steps": int(steps),
                "active_samples": int(observations.shape[0]),
            }
        )
    successes = sum(row["success"] for row in rows)
    falls = sum(row["fallen"] for row in rows)
    metrics = {
        "cases": len(rows),
        "successes": successes,
        "success_rate": successes / len(rows),
        "falls": falls,
        "fall_rate": falls / len(rows),
        "mean_steps": float(np.mean([row["steps"] for row in rows])),
    }
    metrics.update(summarize_retention(rows))
    empty_obs = np.empty((0, 174), dtype=np.float32)
    empty_actions = np.empty((0, 16), dtype=np.float32)
    return (
        metrics,
        rows,
        np.concatenate(failed_observations) if failed_observations else empty_obs,
        np.concatenate(active_observations) if active_observations else empty_obs,
        np.concatenate(active_actions) if active_actions else empty_actions,
    )


def score(metrics: dict[str, float]) -> tuple[float, float, float, float, float]:
    latency = metrics.get("mean_front_to_rear_steps")
    return (
        float(metrics["success_rate"]),
        -float(metrics["fall_rate"]),
        float(metrics.get("drop_free_success_rate", metrics["success_rate"])),
        -float(1.0e9 if latency is None else latency),
        -float(metrics["mean_steps"]),
    )


def write_matrix(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def save_student(
    path: Path,
    initial_payload: dict,
    initial_actor_state,
    student: nn.Module,
    *,
    cycle: int,
    metrics: dict[str, float],
    metadata: dict[str, object],
) -> None:
    payload = copy.deepcopy(initial_payload)
    state = dict(payload[initial_actor_state.container_key])
    linears = [module for module in student.modules() if isinstance(module, nn.Linear)]
    pairs = linear_layer_keys(initial_actor_state)
    if len(linears) != len(pairs):
        raise RuntimeError("student layer count does not match checkpoint")
    for layer, (weight_key, bias_key) in zip(linears, pairs):
        state[weight_key] = layer.weight.detach().cpu().clone()
        state[bias_key] = layer.bias.detach().cpu().clone()
    payload[initial_actor_state.container_key] = state
    payload["iteration"] = int(cycle)
    payload["eval_success"] = float(metrics["success_rate"])
    payload["eval_fall"] = float(metrics["fall_rate"])
    payload["gate16_residual_dagger"] = metadata
    torch.save(payload, path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path, nargs="+")
    parser.add_argument("--xml", type=Path, required=True)
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--initial-residual-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--cycles", type=int, default=8)
    parser.add_argument("--warmup-epochs", type=int, default=1000)
    parser.add_argument("--epochs-per-cycle", type=int, default=250)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--learning-rate", type=float, default=2.0e-5)
    parser.add_argument("--hard-weight", type=float, default=6.0)
    parser.add_argument("--rehearsal-weight", type=float, default=2.0)
    parser.add_argument("--parameter-anchor", type=float, default=1.0e-4)
    parser.add_argument("--critical-repeat", type=int, default=4)
    parser.add_argument("--critical-x", type=float, default=12.10)
    parser.add_argument("--rear-action-loss-scale", type=float, default=0.5)
    parser.add_argument("--max-dagger-samples", type=int, default=30000)
    parser.add_argument(
        "--distances",
        type=float,
        nargs="+",
        default=[0.80, 0.90, 1.00, 1.10, 1.20],
    )
    parser.add_argument(
        "--lateral-offsets", type=float, nargs="+", default=[-0.05, 0.0, 0.05]
    )
    parser.add_argument(
        "--yaw-degrees", type=float, nargs="+", default=[-5.0, 0.0, 5.0]
    )
    parser.add_argument("--approach-speeds", type=float, nargs="+", default=[0.10])
    parser.add_argument(
        "--entry-cases-csv",
        type=Path,
        help="measured navigation arrivals; replaces the Cartesian matrix",
    )
    parser.add_argument(
        "--activation-mode", choices=("distance", "heightmap"), default="heightmap"
    )
    parser.add_argument("--activate-distance", type=float, default=0.65)
    parser.add_argument("--correction-limit", type=float, default=4.0)
    parser.add_argument("--seed", type=int, default=37728)
    args = parser.parse_args()
    if args.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable")
    if args.cycles < 1 or args.warmup_epochs < 1 or args.epochs_per_cycle < 1:
        parser.error("cycles and epoch counts must be positive")
    if not 0.0 < args.rear_action_loss_scale <= 1.0:
        parser.error("rear-action-loss-scale must be in (0, 1]")
    args.output.mkdir(parents=True, exist_ok=True)

    device = torch.device(args.device)
    rng = np.random.default_rng(args.seed)
    torch.manual_seed(args.seed)
    base_actor = _load_inference_actor(args.base_checkpoint, 174)
    initial_payload = torch.load(
        args.initial_residual_checkpoint, map_location="cpu", weights_only=False
    )
    initial_actor_state = extract_actor_state(initial_payload)
    student = build_actor(initial_actor_state).to(device)
    reference = copy.deepcopy(student).eval()
    for parameter in reference.parameters():
        parameter.requires_grad_(False)

    demo_observations, demo_targets, demo_paths = demonstration_targets(
        args.trajectory,
        base_actor,
        correction_limit=args.correction_limit,
        critical_repeat=args.critical_repeat,
        critical_x=args.critical_x,
    )
    teacher = NearestDemoTeacher(demo_observations, demo_targets)
    cases = (
        load_navigation_cases(args.entry_cases_csv)
        if args.entry_cases_csv is not None
        else entry_cases(
            args.distances,
            args.lateral_offsets,
            args.yaw_degrees,
            args.approach_speeds,
        )
    )

    initial_metrics, initial_rows, _, rehearsal_observations, rehearsal_targets = (
        evaluate_grid(args.xml, base_actor, student, cases, args, cycle=0)
    )
    if not rehearsal_observations.size:
        raise RuntimeError("initial policy never activated on the Gate-16 matrix")
    write_matrix(args.output / "matrix_cycle_000.csv", initial_rows)
    print(
        f"cycle=00 success={initial_metrics['success_rate']:.1%} "
        f"fall={initial_metrics['fall_rate']:.1%} "
        f"rehearsal={rehearsal_observations.shape[0]}",
        flush=True,
    )

    hard_observations = demo_observations.copy()
    hard_targets = demo_targets.copy()
    action_scale = np.ones(16, dtype=np.float32)
    action_scale[6:12] = args.rear_action_loss_scale
    action_scale[14:16] = args.rear_action_loss_scale
    optimizer = torch.optim.Adam(student.parameters(), lr=args.learning_rate)
    best_metrics = initial_metrics
    best_cycle = 0
    best_path = args.initial_residual_checkpoint.resolve()
    rows = []

    for cycle in range(1, args.cycles + 1):
        losses = train_epoch_group(
            student,
            reference,
            optimizer,
            hard_observations,
            hard_targets,
            rehearsal_observations,
            rehearsal_targets,
            action_scale,
            epochs=args.warmup_epochs if cycle == 1 else args.epochs_per_cycle,
            batch_size=args.batch_size,
            hard_weight=args.hard_weight,
            rehearsal_weight=args.rehearsal_weight,
            parameter_anchor=args.parameter_anchor,
            rng=rng,
        )
        metrics, matrix_rows, failed, _, _ = evaluate_grid(
            args.xml, base_actor, student, cases, args, cycle=cycle
        )
        write_matrix(args.output / f"matrix_cycle_{cycle:03d}.csv", matrix_rows)
        demo_prediction = infer(student, demo_observations)
        demo_rmse = float(
            np.sqrt(np.mean(np.square(demo_prediction - demo_targets)))
        )
        if score(metrics) > score(best_metrics):
            best_metrics = metrics
            best_cycle = cycle
            best_path = (args.output / "best_checkpoint.pt").resolve()
            save_student(
                best_path,
                initial_payload,
                initial_actor_state,
                student,
                cycle=cycle,
                metrics=metrics,
                metadata={
                    "base_checkpoint": str(args.base_checkpoint.resolve()),
                    "initial_residual_checkpoint": str(
                        args.initial_residual_checkpoint.resolve()
                    ),
                    "demonstrations": demo_paths,
                    "entry_cases": cases,
                    "cycle": cycle,
                    "demo_rmse": demo_rmse,
                },
            )
        if failed.size:
            labels = teacher.label(failed)
            hard_observations = np.concatenate([hard_observations, failed])
            hard_targets = np.concatenate([hard_targets, labels])
            if hard_observations.shape[0] > args.max_dagger_samples:
                keep_demo = min(demo_observations.shape[0], args.max_dagger_samples)
                remaining = args.max_dagger_samples - keep_demo
                dagger_pool = np.arange(
                    demo_observations.shape[0], hard_observations.shape[0]
                )
                sampled = (
                    rng.choice(
                        dagger_pool,
                        size=min(remaining, dagger_pool.size),
                        replace=False,
                    )
                    if remaining and dagger_pool.size
                    else np.empty(0, dtype=np.int64)
                )
                hard_observations = np.concatenate(
                    [demo_observations[:keep_demo], hard_observations[sampled]]
                )
                hard_targets = np.concatenate(
                    [demo_targets[:keep_demo], hard_targets[sampled]]
                )
        row = {
            "cycle": cycle,
            **metrics,
            **losses,
            "demo_rmse": demo_rmse,
            "failed_samples_added": int(failed.shape[0]),
            "hard_samples": int(hard_observations.shape[0]),
            "best_cycle": best_cycle,
        }
        rows.append(row)
        print(
            f"cycle={cycle:02d} success={metrics['success_rate']:.1%} "
            f"fall={metrics['fall_rate']:.1%} demo_rmse={demo_rmse:.4f} "
            f"dagger+={failed.shape[0]} best={best_cycle}",
            flush=True,
        )

    with (args.output / "metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    improved = score(best_metrics) > score(initial_metrics)
    summary = {
        "initial_metrics": initial_metrics,
        "best_metrics": best_metrics,
        "best_cycle": best_cycle,
        "best_checkpoint": str(best_path),
        "improved": improved,
        "selection_rule": (
            "success_rate, lower fall_rate, drop_free_success_rate, lower "
            "front_to_rear_steps, then lower mean_steps"
        ),
        "cases": cases,
        "demonstrations": demo_paths,
        "base_sha256": sha256(args.base_checkpoint),
        "initial_residual_sha256": sha256(args.initial_residual_checkpoint),
    }
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
