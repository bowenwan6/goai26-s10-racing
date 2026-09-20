#!/usr/bin/env python3
"""Select a Gate-16 rear-push policy behind reliability and stability gates."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path

from .front_retention import summarize_retention
from .rear_push import summarize_rear_push
from .speed_objective import finite_or, summarize_speed


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def matrix(summary: dict) -> list[tuple[float, float, float, float]]:
    return [
        (
            float(row["distance"]),
            float(row["lateral"]),
            float(row["yaw_deg"]),
            float(row["speed"]),
        )
        for row in summary["results"]
    ]


def metrics(summary: dict) -> dict[str, float | int | None]:
    rows = summary["results"]
    return {
        "cases": len(rows),
        "successes": sum(int(bool(row.get("success", False))) for row in rows),
        "success_rate": float(summary["success_rate"]),
        "fall_rate": float(summary["fall_rate"]),
        **summarize_speed(rows),
        **summarize_retention(rows),
        **summarize_rear_push(rows),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--candidate",
        nargs=3,
        action="append",
        metavar=("NAME", "CHECKPOINT", "FORMAL_SUMMARY"),
        required=True,
    )
    parser.add_argument("--baseline-name", default="speed_core")
    parser.add_argument("--target-front-to-rear-steps", type=float, default=170.0)
    parser.add_argument("--target-drop-free-rate", type=float, default=0.55)
    parser.add_argument("--target-front-tuck-rate", type=float, default=0.55)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.target_front_to_rear_steps <= 0.0:
        parser.error("target front-to-rear steps must be positive")
    if not 0.0 <= args.target_drop_free_rate <= 1.0:
        parser.error("target drop-free rate must be in [0, 1]")
    if not 0.0 <= args.target_front_tuck_rate <= 1.0:
        parser.error("target front-tuck rate must be in [0, 1]")
    args.output.mkdir(parents=True, exist_ok=True)

    candidates = []
    formal_matrix = None
    for name, checkpoint_text, formal_text in args.candidate:
        checkpoint = Path(checkpoint_text)
        formal_path = Path(formal_text)
        summary = json.loads(formal_path.read_text())
        candidate_matrix = matrix(summary)
        if formal_matrix is None:
            formal_matrix = candidate_matrix
        elif candidate_matrix != formal_matrix:
            raise ValueError(f"{name}: formal evaluation matrix differs")
        candidates.append(
            {
                "name": name,
                "checkpoint": str(checkpoint.resolve()),
                "checkpoint_sha256": sha256(checkpoint),
                "formal_summary": str(formal_path.resolve()),
                "formal": metrics(summary),
            }
        )

    baseline = next(
        (row for row in candidates if row["name"] == args.baseline_name), None
    )
    if baseline is None:
        raise ValueError(f"baseline candidate {args.baseline_name!r} is missing")
    success_floor = float(baseline["formal"]["success_rate"])
    fall_ceiling = float(baseline["formal"]["fall_rate"])
    drop_free_floor = float(baseline["formal"]["drop_free_success_rate"])

    for row in candidates:
        formal = row["formal"]
        eligible = bool(
            float(formal["success_rate"]) + 1.0e-12 >= success_floor
            and float(formal["fall_rate"]) <= fall_ceiling + 1.0e-12
            and float(formal["drop_free_success_rate"]) + 1.0e-12
            >= drop_free_floor
        )
        target_hit = bool(
            eligible
            and float(formal["drop_free_success_rate"])
            >= args.target_drop_free_rate
            and float(formal["front_tuck_target_rate"])
            >= args.target_front_tuck_rate
            and finite_or(formal["mean_front_to_rear_steps"], 1.0e9)
            <= args.target_front_to_rear_steps
        )
        row["eligible"] = eligible
        row["target_hit"] = target_hit
        row["score"] = [
            int(target_hit),
            -finite_or(formal["mean_penalized_front_to_rear_steps"], 1.0e9),
            float(formal["drop_free_success_rate"]),
            float(formal["front_tuck_target_rate"]),
            finite_or(formal["mean_front_tuck_m"], 0.0),
            -finite_or(formal["mean_front_to_rear_steps"], 1.0e9),
            -finite_or(formal["mean_rear_push_vertical_reversals"], 1.0e9),
            float(formal["success_rate"]),
            -float(formal["fall_rate"]),
        ]

    selected = max(
        (row for row in candidates if row["eligible"]),
        key=lambda row: tuple(row["score"]),
    )
    destination = args.output / "best_checkpoint.pt"
    shutil.copy2(selected["checkpoint"], destination)
    payload = {
        "selection_rule": (
            "formal success, fall, and drop-free rates cannot regress from the "
            "speed_core baseline; then prefer the 170-step, 55%-drop-free, and "
            "55%-front-tuck target and minimize failure-penalized front-to-rear time"
        ),
        "baseline_name": args.baseline_name,
        "baseline_success_floor": success_floor,
        "baseline_fall_ceiling": fall_ceiling,
        "baseline_drop_free_floor": drop_free_floor,
        "target_front_to_rear_steps": args.target_front_to_rear_steps,
        "target_drop_free_rate": args.target_drop_free_rate,
        "target_front_tuck_rate": args.target_front_tuck_rate,
        "selected": selected["name"],
        "selected_checkpoint": str(destination.resolve()),
        "selected_sha256": sha256(destination),
        "formal_matrix_cases": len(formal_matrix or []),
        "candidates": candidates,
    }
    (args.output / "selection.json").write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
