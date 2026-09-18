#!/usr/bin/env python3
"""Supervise a 174-D residual actor from successful official-track trajectories."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import torch
from torch import nn

from .distill_cem import _infer, _load_inference_actor
from .distill_official_skill import load_demonstrations
from .network import ActorCritic
from .official_policy_env import FULL_CORRECTION_SCALE, OfficialClosedLoopResidualEnv
from .train import save_checkpoint


@torch.no_grad()
def evaluate(model, envs, trials: int, device: torch.device):
    success = []
    falls = []
    for trial in range(trials):
        env = envs[trial % len(envs)]
        observation = env.reset()
        done = False
        info = {"success": False, "fallen": False}
        while not done:
            residual = model.actor(
                torch.as_tensor(observation, device=device).unsqueeze(0)
            ).squeeze(0)
            observation, _, done, info = env.step(residual.cpu().numpy())
        success.append(float(info.get("success", False)))
        falls.append(float(info.get("fallen", False)))
    return float(np.mean(success)), float(np.mean(falls))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path, nargs="+")
    parser.add_argument("--xml", type=Path, required=True)
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--epochs", type=int, default=20000)
    parser.add_argument("--batch-size", type=int, default=1024)
    parser.add_argument("--learning-rate", type=float, default=1.0e-4)
    parser.add_argument(
        "--actor-hidden",
        type=int,
        nargs="+",
        default=[256, 128, 64],
        help="residual actor hidden widths",
    )
    parser.add_argument("--eval-trials", type=int, default=20)
    parser.add_argument("--distance-min", type=float, default=0.80)
    parser.add_argument("--distance-max", type=float, default=1.20)
    parser.add_argument("--lateral-range", type=float, default=0.0)
    parser.add_argument("--yaw-range", type=float, default=0.0)
    parser.add_argument("--height-noise", type=float, default=0.0)
    parser.add_argument("--correction-limit", type=float, default=4.0)
    parser.add_argument("--seed", type=int, default=37724)
    args = parser.parse_args()
    if args.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable")
    args.output.mkdir(parents=True, exist_ok=True)
    device = torch.device(args.device)
    torch.manual_seed(args.seed)
    rng = np.random.default_rng(args.seed)
    observations, actions, paths = load_demonstrations(args.trajectory)
    base_actor = _load_inference_actor(args.base_checkpoint, 174)
    base_actions = _infer(base_actor, observations)
    targets = (actions - base_actions) / FULL_CORRECTION_SCALE

    model = ActorCritic(
        174,
        actor_hidden=tuple(args.actor_hidden),
        critic_hidden=(256, 128, 64),
    ).to(device)
    with torch.no_grad():
        model.actor.mlp[-1].weight.zero_()
        model.actor.mlp[-1].bias.zero_()
        model.actor.log_std.fill_(-2.0)
    optimizer = torch.optim.Adam(model.actor.parameters(), lr=args.learning_rate)
    x = torch.as_tensor(observations, dtype=torch.float32, device=device)
    y = torch.as_tensor(targets, dtype=torch.float32, device=device)
    rows = []
    for epoch in range(1, args.epochs + 1):
        indices = torch.as_tensor(
            rng.integers(0, observations.shape[0], size=min(args.batch_size, observations.shape[0])),
            device=device,
        )
        prediction = model.actor(x[indices])
        loss = nn.functional.mse_loss(prediction, y[indices])
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        nn.utils.clip_grad_norm_(model.actor.parameters(), 5.0)
        optimizer.step()
        if epoch == 1 or epoch % 500 == 0 or epoch == args.epochs:
            with torch.no_grad():
                full_prediction = model.actor(x)
                residual_rmse = float(
                    torch.sqrt(torch.mean(torch.square(full_prediction - y))).cpu()
                )
                final_actions = (
                    torch.as_tensor(base_actions, device=device)
                    + full_prediction * torch.as_tensor(FULL_CORRECTION_SCALE, device=device)
                )
                action_rmse = float(
                    torch.sqrt(
                        torch.mean(
                            torch.square(final_actions - torch.as_tensor(actions, device=device))
                        )
                    ).cpu()
                )
            row = {
                "epoch": epoch,
                "batch_loss": float(loss.detach().cpu()),
                "residual_rmse": residual_rmse,
                "action_rmse": action_rmse,
            }
            rows.append(row)
            print(
                f"epoch={epoch:05d} residual_rmse={residual_rmse:.5f} "
                f"action_rmse={action_rmse:.5f}",
                flush=True,
            )

    eval_envs = [
        OfficialClosedLoopResidualEnv(
            args.xml,
            base_actor,
            seed=args.seed + 900_000 + 1009 * index,
            distance_range=(args.distance_min, args.distance_max),
            lateral_range=args.lateral_range,
            yaw_range=args.yaw_range,
            height_noise=args.height_noise,
            correction_scale=FULL_CORRECTION_SCALE,
            correction_limit=args.correction_limit,
        )
        for index in range(min(4, args.eval_trials))
    ]
    success_rate, fall_rate = evaluate(model, eval_envs, args.eval_trials, device)
    checkpoint = args.output / "official_residual_174.pt"
    save_checkpoint(
        checkpoint,
        model,
        optimizer,
        args.epochs,
        174,
        0.37691055,
        {
            "success_rate": success_rate,
            "fall_rate": fall_rate,
            "mean_reward": 0.0,
            "mean_episode_return": 0.0,
        },
    )
    payload = torch.load(checkpoint, map_location="cpu", weights_only=False)
    payload["official_residual_distillation"] = {
        "base_checkpoint": str(args.base_checkpoint.resolve()),
        "demonstrations": paths,
        "correction_scale": FULL_CORRECTION_SCALE.tolist(),
        "correction_limit": args.correction_limit,
        "actor_hidden": args.actor_hidden,
    }
    torch.save(payload, checkpoint)
    with (args.output / "metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "checkpoint": str(checkpoint.resolve()),
        "action_rmse": rows[-1]["action_rmse"],
        "residual_rmse": rows[-1]["residual_rmse"],
        "success_rate": success_rate,
        "fall_rate": fall_rate,
        "demonstrations": paths,
        "pass": bool(success_rate >= 0.95 and fall_rate <= 0.05),
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
