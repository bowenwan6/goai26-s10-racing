#!/usr/bin/env python3
"""Collect successful deterministic Gate-16 traces from a residual checkpoint.

Only strict four-wheel successes are written as demonstrations.  Failed cases remain in
the matrix CSV/JSON so the collection cannot silently present a cherry-picked success
rate.  The saved ``actions`` are the final raw 16-D actions that reached MuJoCo, while
``residual_actions`` retains the normalized correction emitted by the residual actor.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from .distill_cem import _infer, _load_inference_actor
from .gate16_entry_cases import entry_cases, load_navigation_cases
from .front_retention import summarize_retention
from .official_policy_env import FULL_CORRECTION_SCALE, OfficialClosedLoopResidualEnv
from .rear_push import summarize_rear_push
from .speed_objective import summarize_speed


PHASE_ACTION_INDICES = {
    # Front joints fold the legs while front-wheel drive preserves forward pull.
    "settle": np.asarray([0, 1, 2, 3, 4, 5, 12, 13], dtype=np.int64),
    # Once tucked, rear leg targets and rear wheel speeds provide the push.
    "push": np.asarray([6, 7, 8, 9, 10, 11, 14, 15], dtype=np.int64),
}


def load_rear_push_action_deltas(
    path: Path | None, *, statistic: str
) -> tuple[np.ndarray | None, np.ndarray | None]:
    if path is None:
        return None, None
    payload = json.loads(path.read_text(encoding="utf-8"))
    result = []
    for phase in ("settle", "push"):
        try:
            values = payload["phases"][phase]["action_delta"][statistic]
        except (KeyError, TypeError) as exc:
            raise ValueError(
                f"{path}: missing phases.{phase}.action_delta.{statistic}"
            ) from exc
        source = np.asarray(values, dtype=np.float32)
        if source.shape != (16,) or not np.all(np.isfinite(source)):
            raise ValueError(f"{path}: {phase} action delta must be finite shape (16,)")
        phase_only = np.zeros(16, dtype=np.float32)
        indices = PHASE_ACTION_INDICES[phase]
        phase_only[indices] = source[indices]
        result.append(phase_only)
    return result[0], result[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rollout(env, residual_actor, case: dict[str, object]) -> dict[str, object]:
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
    observations: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    residual_actions: list[np.ndarray] = []
    rewards: list[float] = []
    qpos: list[np.ndarray] = []
    qvel: list[np.ndarray] = []
    active: list[bool] = []
    info: dict[str, object] = {
        "success": False,
        "fallen": False,
        "right_wheels": 0,
    }
    done = False
    while not done:
        correction = _infer(residual_actor, observation[None, :])[0]
        observations.append(observation.copy())
        qpos.append(env.runtime.data.qpos.copy())
        qvel.append(env.runtime.data.qvel.copy())
        observation, reward, done, info = env.step(correction)
        actions.append(np.asarray(info["final_action"], dtype=np.float32).copy())
        residual_actions.append(
            np.asarray(info["applied_residual"], dtype=np.float32).copy()
        )
        rewards.append(float(reward))
        active.append(bool(info.get("skill_active", False)))
    return {
        "observations": np.asarray(observations, dtype=np.float32),
        "actions": np.asarray(actions, dtype=np.float32),
        "residual_actions": np.asarray(residual_actions, dtype=np.float32),
        "rewards": np.asarray(rewards, dtype=np.float32),
        "qpos": np.asarray(qpos, dtype=np.float64),
        "qvel": np.asarray(qvel, dtype=np.float64),
        "skill_active": np.asarray(active, dtype=np.bool_),
        "info": info,
        "steps": len(observations),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", type=Path, required=True)
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--residual-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--distances",
        type=float,
        nargs="+",
        default=[0.80, 0.90, 1.00, 1.10, 1.20],
    )
    parser.add_argument("--lateral-offsets", type=float, nargs="+", default=[0.0])
    parser.add_argument("--yaw-degrees", type=float, nargs="+", default=[0.0])
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
    parser.add_argument("--rear-push-action-delta-json", type=Path)
    parser.add_argument("--rear-push-action-delta-scale", type=float, default=0.0)
    parser.add_argument(
        "--rear-push-action-delta-statistic",
        choices=("mean", "median"),
        default="median",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if any(distance <= 0.0 for distance in args.distances):
        parser.error("distances must be positive")
    if any(speed <= 0.0 for speed in args.approach_speeds):
        parser.error("approach speeds must be positive")
    if args.rear_push_action_delta_scale < 0.0:
        parser.error("rear-push action delta scale cannot be negative")
    settle_delta, push_delta = load_rear_push_action_deltas(
        args.rear_push_action_delta_json,
        statistic=args.rear_push_action_delta_statistic,
    )

    base_actor = _load_inference_actor(args.base_checkpoint, 174)
    residual_actor = _load_inference_actor(args.residual_checkpoint, 174)
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
    env_by_speed = {
        speed: OfficialClosedLoopResidualEnv(
            args.xml,
            base_actor,
            seed=args.seed + index * 1009,
            approach_speed=speed,
            activate_distance=args.activate_distance,
            distance_range=(
                min(float(case["distance"]) for case in cases),
                max(float(case["distance"]) for case in cases),
            ),
            lateral_range=0.0,
            yaw_range=0.0,
            height_noise=0.0,
            correction_scale=FULL_CORRECTION_SCALE,
            correction_limit=args.correction_limit,
            activation_mode=args.activation_mode,
            rear_push_settle_action_delta=settle_delta,
            rear_push_push_action_delta=push_delta,
            rear_push_action_delta_scale=args.rear_push_action_delta_scale,
        )
        for index, speed in enumerate(
            sorted({float(case["speed"]) for case in cases})
        )
    }

    rows = []
    trajectories = []
    for index, case in enumerate(cases, start=1):
        result = rollout(env_by_speed[case["speed"]], residual_actor, case)
        info = result["info"]
        success = bool(info.get("success", False))
        saved = None
        if success:
            saved = args.output / f"demo_{index:04d}.npz"
            np.savez_compressed(
                saved,
                observations=result["observations"],
                actions=result["actions"],
                residual_actions=result["residual_actions"],
                rewards=result["rewards"],
                qpos=result["qpos"],
                qvel=result["qvel"],
                skill_active=result["skill_active"],
                initial_distance=np.asarray(case["distance"]),
                initial_lateral=np.asarray(case["lateral"]),
                initial_yaw=np.asarray(case["yaw_rad"]),
                approach_speed=np.asarray(case["speed"]),
                command_lateral=np.asarray(case.get("cmd_lateral", 0.0)),
                command_yaw=np.asarray(
                    np.nan if case.get("cmd_yaw") is None else case["cmd_yaw"]
                ),
                initial_forward_speed=np.asarray(case.get("forward_speed", 0.0)),
                initial_lateral_speed=np.asarray(case.get("lateral_speed", 0.0)),
                initial_yaw_rate=np.asarray(case.get("yaw_rate", 0.0)),
                correction_scale=FULL_CORRECTION_SCALE,
            )
            trajectories.append(str(saved.resolve()))
        row = {
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
            "climb_start_step": info.get("climb_start_step"),
            "climb_start_to_clear_steps": info.get("climb_start_to_clear_steps"),
            "front_wheel_body_distance_m": info.get(
                "front_wheel_body_distance_m"
            ),
            "front_tuck_m": info.get("front_tuck_m"),
            "front_tuck_target_reached": int(
                bool(info.get("front_tuck_target_reached", False))
            ),
            "front_tuck_step": info.get("front_tuck_step"),
            "body_lowering_m": info.get("body_lowering_m"),
            "rear_push_vertical_reversals": info.get(
                "rear_push_vertical_reversals"
            ),
            "steps": int(result["steps"]),
            "trajectory": str(saved.resolve()) if saved else "",
        }
        rows.append(row)
        print(
            f"case={index:04d} d={case['distance']:.3f} "
            f"lat={case['lateral']:+.3f} yaw={case['yaw_deg']:+.1f} "
            f"v={case['speed']:.2f} success={row['success']} "
            f"fall={row['fallen']} wheels={row['right_wheels']}",
            flush=True,
        )

    with (args.output / "matrix.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    successes = sum(row["success"] for row in rows)
    falls = sum(row["fallen"] for row in rows)
    summary = {
        "cases": len(rows),
        "successes": successes,
        "success_rate": successes / len(rows),
        "falls": falls,
        "fall_rate": falls / len(rows),
        "strict_success_definition": "OfficialTrackRuntime four-wheel clearance",
        "base_checkpoint": str(args.base_checkpoint.resolve()),
        "base_sha256": sha256(args.base_checkpoint),
        "residual_checkpoint": str(args.residual_checkpoint.resolve()),
        "residual_sha256": sha256(args.residual_checkpoint),
        "activation_mode": args.activation_mode,
        "correction_limit": args.correction_limit,
        "rear_push_action_delta_json": (
            None
            if args.rear_push_action_delta_json is None
            else str(args.rear_push_action_delta_json.resolve())
        ),
        "rear_push_action_delta_scale": args.rear_push_action_delta_scale,
        "rear_push_action_delta_statistic": args.rear_push_action_delta_statistic,
        "entry_cases_csv": (
            str(args.entry_cases_csv.resolve())
            if args.entry_cases_csv is not None
            else None
        ),
        "trajectories": trajectories,
        "results": rows,
    }
    summary.update(summarize_retention(rows))
    summary.update(summarize_speed(rows))
    summary.update(summarize_rear_push(rows))
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {key: value for key, value in summary.items() if key not in {"results"}},
            indent=2,
        ),
        flush=True,
    )
    return 0 if successes else 2


if __name__ == "__main__":
    raise SystemExit(main())
