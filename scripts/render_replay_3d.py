#!/usr/bin/env python3
"""Render a lightweight segment replay into high-resolution MuJoCo PNG frames."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

# Software rendering must be selected before importing MuJoCo.
os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco
import numpy as np

from s10_perception.png import write_png

DEFAULT_OVERLAY_FONT = "/System/Library/Fonts/Supplemental/Arial.ttf"


def _intervals(values: np.ndarray, timing: np.ndarray):
    """Yield value/start/end runs on a timeline normalized to its first frame."""
    if len(values) != len(timing):
        raise ValueError("overlay values and timing must have equal lengths")
    if not len(values):
        return
    origin = float(timing[0])
    final_step = float(timing[-1] - timing[-2]) if len(timing) > 1 else 1.0 / 30.0
    start = 0
    for index in range(1, len(values) + 1):
        if index < len(values) and values[index] == values[start]:
            continue
        end_time = (
            float(timing[index] - origin)
            if index < len(timing)
            else float(timing[-1] - origin + final_step)
        )
        yield values[start], float(timing[start] - origin), end_time
        start = index


def _drawtext(
    text: str,
    x: int,
    y: int,
    start: float | None = None,
    end: float | None = None,
    font: str = DEFAULT_OVERLAY_FONT,
):
    safe = text.replace("\\", r"\\").replace("'", r"\'").replace(":", r"\:")
    safe_font = font.replace("\\", r"\\").replace("'", r"\'").replace(":", r"\:")
    item = (
        f"drawtext=fontfile='{safe_font}':text='{safe}':x={x}:y={y}:"
        "fontsize=34:fontcolor=white:"
        "box=1:boxcolor=black@0.60:boxborderw=10"
    )
    if start is not None and end is not None:
        item += f":enable='between(t\\,{start:.3f}\\,{end:.3f})'"
    return item


def _write_overlay_filter(trace, timing: np.ndarray, output: Path, font: str) -> None:
    required = ("target_waypoint", "active_policy", "joint_owner")
    missing = [key for key in required if key not in trace.files]
    if missing:
        raise ValueError(f"replay missing overlay metadata: {', '.join(missing)}")
    waypoints = np.asarray(trace["target_waypoint"])
    policies = np.asarray(trace["active_policy"]).astype(str)
    owners = np.asarray(trace["joint_owner"]).astype(str)
    if any(values.shape != timing.shape for values in (waypoints, policies, owners)):
        raise ValueError("replay overlay metadata is not aligned with frame timing")

    filters = [_drawtext("Elapsed %{pts:hms}", 24, 24, font=font)]
    for waypoint, start, end in _intervals(waypoints, timing):
        label = "Course complete" if int(waypoint) > 32 else f"Target WP{int(waypoint):02d}"
        filters.append(_drawtext(label, 24, 78, start, end, font))
    for policy, start, end in _intervals(policies, timing):
        filters.append(_drawtext(f"Policy {policy}", 24, 132, start, end, font))
    for owner, start, end in _intervals(owners, timing):
        filters.append(_drawtext(f"Joint owner {owner}", 24, 186, start, end, font))

    for index in range(1, len(waypoints)):
        previous, current = int(waypoints[index - 1]), int(waypoints[index])
        if current != previous + 1:
            continue
        event = float(timing[index] - timing[0])
        filters.append(_drawtext(f"WP{previous:02d} PASSED", 24, 250, event, event + 2.0, font))
    output.write_text(",\n".join(filters) + "\n")


def _write_timing_files(args, trace, timing: np.ndarray, qpos_count: int) -> None:
    indices = list(range(0, qpos_count, args.stride))
    concat = args.out / "frames.ffconcat"
    lines = ["ffconcat version 1.0"]
    for number, source_index in enumerate(indices):
        lines.append(f"file '{number:05d}.png'")
        if number + 1 < len(indices):
            duration = timing[indices[number + 1]] - timing[source_index]
        else:
            # There is no measured interval after the final sample. Show it for one output
            # frame; repeating the previous interval can add a long artificial tail when
            # shutdown delayed the last capture.
            duration = 1.0 / 30.0
        lines.append(f"duration {duration:.9f}")
    # The concat demuxer ignores the final duration unless the last frame is repeated.
    if indices:
        lines.append(f"file '{len(indices) - 1:05d}.png'")
    concat.write_text("\n".join(lines) + "\n")
    elapsed = 0.0 if not indices else timing[indices[-1]] - timing[indices[0]]
    print(f"wrote {concat} for {elapsed:.3f}s of {args.timing} time", flush=True)
    if args.timing != "none":
        overlay = args.out / "overlay_filters.txt"
        _write_overlay_filter(trace, timing, overlay, args.overlay_font)
        print(f"wrote {overlay}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path, help="replay.npz written by sim_node")
    parser.add_argument("--repo", type=Path, default=Path("/ws"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--distance", type=float, default=4.5)
    parser.add_argument("--elevation", type=float, default=-24.0)
    parser.add_argument("--azimuth", type=float, default=90.0)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--frame-start", type=int, default=0)
    parser.add_argument("--frame-stop", type=int, help="exclusive output-frame index")
    parser.add_argument(
        "--manifest-only",
        action="store_true",
        help="write timing/overlay files without rendering frames",
    )
    parser.add_argument(
        "--no-manifest",
        action="store_true",
        help="render a frame shard without writing shared timing/overlay files",
    )
    parser.add_argument(
        "--overlay-font",
        default=DEFAULT_OVERLAY_FONT,
        help="host font path embedded in the ffmpeg overlay filter",
    )
    parser.add_argument(
        "--timing",
        choices=("none", "wall", "simulation"),
        default="none",
        help="write frames.ffconcat using recorded wall or simulation timestamps",
    )
    args = parser.parse_args()

    if args.stride <= 0:
        parser.error("--stride must be positive")
    if args.frame_start < 0:
        parser.error("--frame-start must be non-negative")
    if args.manifest_only and args.no_manifest:
        parser.error("--manifest-only and --no-manifest are mutually exclusive")

    trace = np.load(args.replay)
    qpos = trace["qpos"]
    timing = None
    if args.timing != "none":
        key = "wall_time" if args.timing == "wall" else "time"
        if key not in trace.files:
            raise SystemExit(
                f"replay has no {key!r}; record it with the current sim_node or use "
                "--timing simulation"
            )
        timing = np.asarray(trace[key], dtype=float)
        if timing.shape != (len(qpos),) or not np.all(np.isfinite(timing)):
            raise SystemExit(f"invalid {key} shape or values: {timing.shape}")
        if len(timing) > 1 and np.any(np.diff(timing) <= 0.0):
            raise SystemExit(f"{key} must be strictly increasing")
    width = args.width or int(trace["width"])
    height = args.height or int(trace["height"])
    if width <= 0 or height <= 0:
        parser.error("width and height must be positive")

    args.out.mkdir(parents=True, exist_ok=True)
    all_indices = list(range(0, len(qpos), args.stride))
    frame_stop = len(all_indices) if args.frame_stop is None else args.frame_stop
    if not args.frame_start <= frame_stop <= len(all_indices):
        parser.error(
            f"frame range [{args.frame_start}, {frame_stop}) is outside [0, {len(all_indices)})"
        )
    if args.manifest_only:
        if timing is None:
            parser.error("--manifest-only requires --timing")
        _write_timing_files(args, trace, timing, len(qpos))
        return 0

    xml = (
        args.repo / "upstream/goai_embodied_future_material/src/S10_sdk_deploy/"
        "S10_description/s10_mjcf/mjcf/S10_track.xml"
    )
    if not xml.is_file():
        raise SystemExit(f"track model not found: {xml}")

    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    if qpos.ndim != 2 or qpos.shape[1] != model.nq:
        raise SystemExit(f"replay qpos shape {qpos.shape} is incompatible with model nq={model.nq}")

    base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base_body_id < 0:
        raise SystemExit("body 'base_link' not found in track model")

    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)
    renderer = mujoco.Renderer(model, height=height, width=width)
    camera = mujoco.MjvCamera()
    camera.distance = args.distance
    camera.elevation = args.elevation
    camera.azimuth = args.azimuth

    selected = [(number, all_indices[number]) for number in range(args.frame_start, frame_stop)]
    count = len(selected)
    for shard_number, (number, source_index) in enumerate(selected):
        data.qpos[:] = qpos[source_index]
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        camera.lookat[:] = data.xpos[base_body_id]
        camera.lookat[2] += 0.15
        renderer.update_scene(data, camera)
        write_png(
            args.out / f"{number:05d}.png",
            renderer.render(),
            compression=1,
        )
        if shard_number == 0 or (shard_number + 1) % 50 == 0 or shard_number + 1 == count:
            print(
                f"rendered shard {shard_number + 1}/{count} "
                f"(global frame {number + 1}/{len(all_indices)})",
                flush=True,
            )

    if timing is not None and not args.no_manifest:
        _write_timing_files(args, trace, timing, len(qpos))

    renderer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
