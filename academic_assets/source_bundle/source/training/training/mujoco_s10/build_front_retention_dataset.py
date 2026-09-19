#!/usr/bin/env python3
"""Build a focused Gate-16 dataset from drop-free strict-success trajectories."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco
import numpy as np

from .evaluate_official_track_skill import OfficialTrackRuntime


@dataclass(frozen=True)
class PhaseWindow:
    first_front_step: int
    first_rear_step: int
    start: int
    stop: int
    front_drop_count: int

    @property
    def front_to_rear_steps(self) -> int:
        return self.first_rear_step - self.first_front_step


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def phase_window(
    front_supported: np.ndarray,
    rear_supported: np.ndarray,
    *,
    pre_steps: int,
    post_steps: int,
) -> PhaseWindow | None:
    front = np.asarray(front_supported, dtype=np.int64).reshape(-1)
    rear = np.asarray(rear_supported, dtype=np.int64).reshape(-1)
    if front.shape != rear.shape:
        raise ValueError("front and rear support arrays must have equal length")
    first_front_matches = np.flatnonzero(front == 2)
    if not first_front_matches.size:
        return None
    first_front = int(first_front_matches[0])
    rear_matches = np.flatnonzero(
        (np.arange(rear.size) >= first_front) & (rear == 2) & (front == 2)
    )
    if not rear_matches.size:
        return None
    first_rear = int(rear_matches[0])
    # Do not count the acquisition frame itself; only a 2 -> <2 transition is a drop.
    drops = int(
        np.count_nonzero(
            (front[first_front + 1 : first_rear] < 2)
            & (front[first_front:first_rear - 1] == 2)
        )
    )
    return PhaseWindow(
        first_front_step=first_front,
        first_rear_step=first_rear,
        start=max(0, first_front - int(pre_steps)),
        stop=min(front.size, first_rear + int(post_steps) + 1),
        front_drop_count=drops,
    )


def expanded_phase_indices(window: PhaseWindow, *, phase_repeat: int) -> list[int]:
    if phase_repeat < 1:
        raise ValueError("phase_repeat must be positive")
    result: list[int] = []
    for index in range(window.start, window.stop):
        repeat = (
            phase_repeat
            if window.first_front_step <= index < window.first_rear_step
            else 1
        )
        result.extend([index] * repeat)
    return result


def validate_trajectory_arrays(arrays, *, source: str) -> None:
    required = {
        "observations": (174,),
        "actions": (16,),
        "qpos": None,
        "qvel": None,
    }
    count = None
    for name, trailing in required.items():
        if name not in arrays:
            raise ValueError(f"{source}: missing {name}")
        value = np.asarray(arrays[name])
        if value.ndim != 2 or (trailing is not None and value.shape[1:] != trailing):
            expected = "[steps, ...]" if trailing is None else f"[steps, {trailing[0]}]"
            raise ValueError(f"{source}: {name} must have shape {expected}")
        if count is None:
            count = value.shape[0]
        elif value.shape[0] != count:
            raise ValueError(f"{source}: {name} step count differs")
        if not np.all(np.isfinite(value)):
            raise ValueError(f"{source}: {name} contains non-finite values")
    if not count:
        raise ValueError(f"{source}: trajectory is empty")


def select_sources(
    sources: list[dict[str, object]],
    *,
    max_front_to_rear_steps: int | None,
    max_sources: int | None,
) -> tuple[list[dict[str, object]], int]:
    accepted = [
        item
        for item in sources
        if item["terminal_four_wheels"]
        and item["window"] is not None
        and item["window"].front_drop_count == 0
        and (
            max_front_to_rear_steps is None
            or item["window"].front_to_rear_steps <= max_front_to_rear_steps
        )
    ]
    accepted.sort(
        key=lambda item: (
            item["window"].front_to_rear_steps,
            item["arrays"]["observations"].shape[0],
            str(item["path"]),
        )
    )
    qualifying_sources = len(accepted)
    if max_sources is not None:
        accepted = accepted[:max_sources]
    return accepted, qualifying_sources


def support_trace(
    runtime: OfficialTrackRuntime,
    qpos: np.ndarray,
    qvel: np.ndarray,
    actions: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    front: list[int] = []
    rear: list[int] = []
    for pose, velocity in zip(qpos, qvel):
        runtime.data.qpos[:] = pose
        runtime.data.qvel[:] = velocity
        runtime.data.ctrl[:] = 0.0
        mujoco.mj_forward(runtime.model, runtime.data)
        state = runtime.front_retention_sample()
        front.append(state.front_supported)
        rear.append(state.rear_supported)

    # Collector snapshots are recorded immediately before each action.  Replaying the
    # final action from the final stored state reconstructs the strict terminal frame.
    runtime.data.qpos[:] = qpos[-1]
    runtime.data.qvel[:] = qvel[-1]
    runtime.data.ctrl[:] = 0.0
    mujoco.mj_forward(runtime.model, runtime.data)
    runtime._apply_action(actions[-1])
    terminal = runtime.front_retention_sample()
    # Attribute the post-action terminal support to the final stored action.  Keep
    # the trace length equal to the trajectory arrays so every selected index is
    # valid for observations/actions/qpos/qvel.
    front[-1] = terminal.front_supported
    rear[-1] = terminal.rear_supported
    return np.asarray(front, np.int8), np.asarray(rear, np.int8)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path, nargs="+")
    parser.add_argument("--xml", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--pre-steps", type=int, default=10)
    parser.add_argument("--post-steps", type=int, default=20)
    parser.add_argument("--phase-repeat", type=int, default=6)
    parser.add_argument("--min-drop-free-sources", type=int, default=3)
    parser.add_argument(
        "--max-front-to-rear-steps",
        type=int,
        help="reject demonstrations slower than this front-to-rear latency",
    )
    parser.add_argument(
        "--max-sources",
        type=int,
        help="keep only this many fastest accepted demonstrations",
    )
    args = parser.parse_args(argv)
    if args.pre_steps < 0 or args.post_steps < 0:
        parser.error("pre/post steps must be non-negative")
    if args.phase_repeat < 1 or args.min_drop_free_sources < 1:
        parser.error("phase repeat and source count must be positive")
    if args.max_front_to_rear_steps is not None and args.max_front_to_rear_steps < 1:
        parser.error("max front-to-rear steps must be positive")
    if args.max_sources is not None and args.max_sources < 1:
        parser.error("max sources must be positive")
    if args.max_sources is not None and args.max_sources < args.min_drop_free_sources:
        parser.error("max sources cannot be smaller than min drop-free sources")
    args.output.mkdir(parents=True, exist_ok=True)

    runtime = OfficialTrackRuntime(args.xml)
    sources = []
    for path in args.trajectory:
        with np.load(path) as archive:
            arrays = {name: np.asarray(archive[name]) for name in archive.files}
        validate_trajectory_arrays(arrays, source=str(path))
        front, rear = support_trace(
            runtime,
            arrays["qpos"],
            arrays["qvel"],
            arrays["actions"],
        )
        window = phase_window(
            front,
            rear,
            pre_steps=args.pre_steps,
            post_steps=args.post_steps,
        )
        sources.append(
            {
                "path": path,
                "sha256": sha256(path),
                "arrays": arrays,
                "window": window,
                "terminal_four_wheels": bool(front[-1] == 2 and rear[-1] == 2),
            }
        )

    accepted, qualifying_sources = select_sources(
        sources,
        max_front_to_rear_steps=args.max_front_to_rear_steps,
        max_sources=args.max_sources,
    )
    if len(accepted) < args.min_drop_free_sources:
        raise RuntimeError(
            f"need {args.min_drop_free_sources} qualifying drop-free strict successes, "
            f"found {len(accepted)}"
        )

    observations = []
    actions = []
    qpos = []
    qvel = []
    source_ids = []
    source_steps = []
    rows = []
    for source_id, item in enumerate(accepted):
        window = item["window"]
        indices = expanded_phase_indices(window, phase_repeat=args.phase_repeat)
        arrays = item["arrays"]
        observations.append(arrays["observations"][indices])
        actions.append(arrays["actions"][indices])
        qpos.append(arrays["qpos"][indices])
        qvel.append(arrays["qvel"][indices])
        source_ids.extend([source_id] * len(indices))
        source_steps.extend(indices)
        rows.append(
            {
                "source_id": source_id,
                "trajectory": str(item["path"].resolve()),
                "sha256": item["sha256"],
                **asdict(window),
                "front_to_rear_steps": window.front_to_rear_steps,
                "selected_samples": len(indices),
            }
        )

    dataset_path = args.output / "front_retention_v1.npz"
    np.savez_compressed(
        dataset_path,
        observations=np.concatenate(observations).astype(np.float32),
        actions=np.concatenate(actions).astype(np.float32),
        qpos=np.concatenate(qpos).astype(np.float64),
        qvel=np.concatenate(qvel).astype(np.float64),
        source_id=np.asarray(source_ids, np.int32),
        source_step=np.asarray(source_steps, np.int32),
    )
    csv_path = args.output / "front_retention_v1.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    payload = {
        "dataset": str(dataset_path.resolve()),
        "dataset_sha256": sha256(dataset_path),
        "sources_seen": len(sources),
        "qualifying_sources": qualifying_sources,
        "drop_free_sources": len(accepted),
        "samples": int(sum(len(part) for part in observations)),
        "pre_steps": args.pre_steps,
        "post_steps": args.post_steps,
        "phase_repeat": args.phase_repeat,
        "max_front_to_rear_steps": args.max_front_to_rear_steps,
        "max_sources": args.max_sources,
        "sources": rows,
    }
    (args.output / "front_retention_v1.json").write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(payload, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
