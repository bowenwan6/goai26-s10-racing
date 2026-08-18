#!/usr/bin/env python3
"""Render synchronized MuJoCo qpos frames to a tracking-camera GIF."""

from __future__ import annotations

import argparse
import math
import subprocess
from pathlib import Path

import mujoco
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("qpos", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--xml-path", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    args = parser.parse_args()

    frames = np.load(args.qpos)
    if frames.ndim != 2 or frames.shape[1] < 23:
        raise ValueError(f"expected qpos array shaped (frames, >=23), got {frames.shape}")
    frames = frames[np.linalg.norm(frames[:, :2], axis=1) > 1.0]
    if len(frames) == 0:
        raise ValueError("recording contains no initialized robot poses")

    model = mujoco.MjModel.from_xml_path(str(args.xml_path))
    data = mujoco.MjData(model)
    base_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base_body < 0:
        raise ValueError("model has no base_link body")

    first_quat = frames[0, 3:7]
    yaw = math.atan2(
        2 * (first_quat[0] * first_quat[3] + first_quat[1] * first_quat[2]),
        1 - 2 * (first_quat[2] ** 2 + first_quat[3] ** 2),
    )
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = base_body
    camera.azimuth = math.degrees(yaw) + 90
    camera.elevation = -18
    camera.distance = 4.0

    option = mujoco.MjvOption()
    mujoco.mjv_defaultOption(option)
    option.geomgroup[1] = 0
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "ffmpeg",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pixel_format",
        "rgb24",
        "-video_size",
        f"{args.width}x{args.height}",
        "-framerate",
        str(args.fps),
        "-i",
        "-",
        "-vf",
        "split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse",
        "-loop",
        "0",
        str(args.output),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    assert process.stdin is not None
    try:
        for qpos in frames:
            data.qpos[:23] = qpos[:23]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=option)
            process.stdin.write(renderer.render().tobytes())
    finally:
        process.stdin.close()
        renderer.close()
    if process.wait() != 0:
        raise SystemExit("ffmpeg failed")
    print(f"saved {args.output} ({len(frames)} frames at {args.fps} fps)")


if __name__ == "__main__":
    main()
