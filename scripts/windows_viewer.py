"""Native Windows MuJoCo viewer for qpos frames streamed from WSL."""

from __future__ import annotations

import argparse
import socket
import struct
import time

import mujoco
import mujoco.viewer
import numpy as np

_QPOS_FRAME = struct.Struct("!23d")


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml-path", required=True)
    parser.add_argument("--port", type=int, default=18777)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    return args


def main() -> None:
    args = _parse_args()
    model = mujoco.MjModel.from_xml_path(args.xml_path)
    if model.nq < 23:
        raise ValueError(f"viewer model needs at least 23 qpos values, got {model.nq}")
    data = mujoco.MjData(model)
    model.vis.scale.contactwidth = 0.5
    model.vis.scale.contactheight = 0.15
    wheel_visuals = {}
    for name in ("fl_wheel", "fr_wheel", "hl_wheel", "hr_wheel"):
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        geoms = np.flatnonzero((model.geom_bodyid == body) & (model.geom_group == 2))
        model.geom_matid[geoms] = -1
        wheel_visuals[body] = geoms
    robot_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if robot_body < 0:
        raise ValueError("viewer model has no 'base_link' body to track")
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("0.0.0.0", args.port))
    receiver.setblocking(False)
    print(f"Windows native viewer listening on UDP {args.port}", flush=True)

    started = last_frame = time.monotonic()
    received = False
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            with viewer.lock():
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                viewer.cam.trackbodyid = robot_body
                viewer.cam.azimuth = 90
                viewer.cam.elevation = -25
                viewer.cam.distance = 4.0
                viewer.opt.geomgroup[1] = 0
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 1

            while viewer.is_running():
                latest = None
                while True:
                    try:
                        payload, _ = receiver.recvfrom(_QPOS_FRAME.size)
                    except BlockingIOError:
                        break
                    if len(payload) == _QPOS_FRAME.size:
                        candidate = np.asarray(_QPOS_FRAME.unpack(payload))
                        if np.isfinite(candidate).all():
                            latest = candidate

                now = time.monotonic()
                if latest is not None:
                    if not received:
                        print("Viewer stream connected", flush=True)
                    received = True
                    last_frame = now
                    with viewer.lock():
                        data.qpos[:23] = latest
                        mujoco.mj_forward(model, data)
                        touching = {
                            int(model.geom_bodyid[geom])
                            for contact in data.contact
                            for geom in (contact.geom1, contact.geom2)
                        }
                        for body, geoms in wheel_visuals.items():
                            model.geom_rgba[geoms] = (
                                (0.1, 1.0, 0.1, 1.0)
                                if body in touching
                                else (1.0, 0.1, 0.1, 1.0)
                            )
                    viewer.sync()
                elif (received and now - last_frame > 2.0) or (
                    not received and now - started > 30.0
                ):
                    print("Viewer stream stopped", flush=True)
                    break
                else:
                    time.sleep(0.005)
    finally:
        receiver.close()


if __name__ == "__main__":
    main()
