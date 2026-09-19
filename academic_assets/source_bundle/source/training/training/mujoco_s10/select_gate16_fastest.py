#!/usr/bin/env python3
"""Select the fastest Gate-16 policy after paired success-rate gates."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path

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
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--candidate",
        nargs=4,
        action="append",
        metavar=("NAME", "CHECKPOINT", "FORMAL_SUMMARY", "SPEED_SUMMARY"),
        required=True,
    )
    parser.add_argument("--baseline-name", default="baseline")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    candidates = []
    formal_matrix = None
    speed_matrix = None
    for name, checkpoint_text, formal_text, speed_text in args.candidate:
        checkpoint = Path(checkpoint_text)
        formal_path = Path(formal_text)
        speed_path = Path(speed_text)
        formal_summary = json.loads(formal_path.read_text())
        speed_summary = json.loads(speed_path.read_text())
        candidate_formal_matrix = matrix(formal_summary)
        candidate_speed_matrix = matrix(speed_summary)
        if formal_matrix is None:
            formal_matrix = candidate_formal_matrix
            speed_matrix = candidate_speed_matrix
        elif candidate_formal_matrix != formal_matrix:
            raise ValueError(f"{name}: formal evaluation matrix differs")
        elif candidate_speed_matrix != speed_matrix:
            raise ValueError(f"{name}: speed evaluation matrix differs")
        candidates.append(
            {
                "name": name,
                "checkpoint": str(checkpoint.resolve()),
                "checkpoint_sha256": sha256(checkpoint),
                "formal_summary": str(formal_path.resolve()),
                "speed_summary": str(speed_path.resolve()),
                "formal": metrics(formal_summary),
                "speed": metrics(speed_summary),
            }
        )

    baseline = next(
        (row for row in candidates if row["name"] == args.baseline_name), None
    )
    if baseline is None:
        raise ValueError(f"baseline candidate {args.baseline_name!r} is missing")
    formal_floor = float(baseline["formal"]["success_rate"])
    speed_floor = float(baseline["speed"]["success_rate"])
    for row in candidates:
        formal = row["formal"]
        speed = row["speed"]
        eligible = bool(
            float(formal["success_rate"]) + 1.0e-12 >= formal_floor
            and float(speed["success_rate"]) + 1.0e-12 >= speed_floor
        )
        row["eligible"] = eligible
        # Once both paired success-rate floors pass, elapsed time is the objective.
        # Failed trials are charged the full 850-step budget so an early fall can
        # never masquerade as a fast trajectory.
        row["score"] = [
            -finite_or(speed["mean_penalized_steps"], 1.0e9),
            -finite_or(formal["mean_penalized_steps"], 1.0e9),
            min(float(formal["success_rate"]), float(speed["success_rate"])),
            float(formal["success_rate"]),
            float(speed["success_rate"]),
            -finite_or(speed["std_success_steps"], 1.0e9),
            -finite_or(speed["fastest_success_steps"], 1.0e9),
        ]

    selected = max(
        (row for row in candidates if row["eligible"]),
        key=lambda row: tuple(row["score"]),
    )
    destination = args.output / "best_checkpoint.pt"
    shutil.copy2(selected["checkpoint"], destination)
    payload = {
        "selection_rule": (
            "formal and speed-matrix success cannot fall below baseline; then "
            "minimize speed-matrix and formal penalized completion time, where "
            "every failure costs the full 850-step budget"
        ),
        "baseline_name": args.baseline_name,
        "baseline_formal_success_rate": formal_floor,
        "baseline_speed_success_rate": speed_floor,
        "selected": selected["name"],
        "selected_checkpoint": str(destination.resolve()),
        "selected_sha256": sha256(destination),
        "formal_matrix_cases": len(formal_matrix or []),
        "speed_matrix_cases": len(speed_matrix or []),
        "candidates": candidates,
    }
    (args.output / "selection.json").write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
