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
    parser.add_argument(
        "--timing",
        choices=("none", "wall", "simulation"),
        default="none",
        help="write frames.ffconcat using recorded wall or simulation timestamps",
    )
    args = parser.parse_args()

    if args.stride <= 0:
        parser.error("--stride must be positive")

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

    xml = (
        args.repo
        / "upstream/goai_embodied_future_material/src/S10_sdk_deploy/"
        "S10_description/s10_mjcf/mjcf/S10_track.xml"
    )
    if not xml.is_file():
        raise SystemExit(f"track model not found: {xml}")

    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    if qpos.ndim != 2 or qpos.shape[1] != model.nq:
        raise SystemExit(
            f"replay qpos shape {qpos.shape} is incompatible with model nq={model.nq}"
        )

    base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base_body_id < 0:
        raise SystemExit("body 'base_link' not found in track model")

    args.out.mkdir(parents=True, exist_ok=True)
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)
    renderer = mujoco.Renderer(model, height=height, width=width)
    camera = mujoco.MjvCamera()
    camera.distance = args.distance
    camera.elevation = args.elevation
    camera.azimuth = args.azimuth

    selected = range(0, len(qpos), args.stride)
    count = (len(qpos) + args.stride - 1) // args.stride
    for number, source_index in enumerate(selected):
        data.qpos[:] = qpos[source_index]
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        camera.lookat[:] = data.xpos[base_body_id]
        camera.lookat[2] += 0.15
        renderer.update_scene(data, camera)
        write_png(args.out / f"{number:05d}.png", renderer.render())
        if number == 0 or (number + 1) % 50 == 0 or number + 1 == count:
            print(f"rendered {number + 1}/{count}", flush=True)

    if timing is not None:
        indices = list(selected)
        concat = args.out / "frames.ffconcat"
        lines = ["ffconcat version 1.0"]
        for number, source_index in enumerate(indices):
            lines.append(f"file '{number:05d}.png'")
            if number + 1 < len(indices):
                duration = timing[indices[number + 1]] - timing[source_index]
            elif len(indices) > 1:
                duration = timing[indices[-1]] - timing[indices[-2]]
            else:
                duration = 1.0 / 30.0
            lines.append(f"duration {duration:.9f}")
        # The concat demuxer ignores the final duration unless the last frame is repeated.
        if indices:
            lines.append(f"file '{len(indices) - 1:05d}.png'")
        concat.write_text("\n".join(lines) + "\n")
        print(
            f"wrote {concat} for {timing[indices[-1]] - timing[indices[0]]:.3f}s "
            f"of {args.timing} time",
            flush=True,
        )

    renderer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
