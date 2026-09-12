"""Native Windows MuJoCo viewer for qpos frames streamed from WSL."""

from __future__ import annotations

import argparse
import ctypes
from ctypes import wintypes
import os
import re
import socket
import struct
import time

import mujoco
import mujoco.viewer
import numpy as np

_QPOS_FRAME = struct.Struct("!23d")
_KEY_FRAME = struct.Struct("!4sB")
_KEY_MAGIC = b"S10K"
_CONTROL_KEYS = frozenset("0rzcxvmhplkwasdqe12345678[]")
_WM_KEYDOWN = 0x0100
_WM_KEYUP = 0x0101
_WM_SYSKEYDOWN = 0x0104
_WM_SYSKEYUP = 0x0105
_WAYPOINT_NAME = re.compile(r"track_waypoint_(\d+)_")
_WAYPOINT_LABEL_HEIGHT = 0.35


class _KeyboardEvent(ctypes.Structure):
    _fields_ = (
        ("vk_code", wintypes.DWORD),
        ("scan_code", wintypes.DWORD),
        ("flags", wintypes.DWORD),
        ("time", wintypes.DWORD),
        ("extra_info", wintypes.WPARAM),
    )


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml-path", required=True)
    parser.add_argument("--port", type=int, default=18777)
    parser.add_argument("--control-port", type=int, default=18778)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if not 1 <= args.control_port <= 65535:
        parser.error("--control-port must be between 1 and 65535")
    return args


def _control_key(vk_code: int, modified: bool = False) -> str | None:
    if modified:
        return None
    if ord("A") <= vk_code <= ord("Z"):
        key = chr(vk_code).lower()
    elif ord("0") <= vk_code <= ord("9"):
        key = chr(vk_code)
    else:
        key = {0xDB: "[", 0xDD: "]"}.get(vk_code)
    return key if key in _CONTROL_KEYS else None


def _encode_key(key: str) -> bytes | None:
    return _KEY_FRAME.pack(_KEY_MAGIC, ord(key)) if key in _CONTROL_KEYS else None


def _waypoint_labels(model, data):
    labels = []
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        match = _WAYPOINT_NAME.match(name)
        if match:
            labels.append((int(match.group(1)), data.geom_xpos[geom_id].copy()))
    return [
        (f"WP{index + 1}", position + (0.0, 0.0, _WAYPOINT_LABEL_HEIGHT))
        for index, position in sorted(labels)
    ]


def _add_waypoint_labels(scene, labels) -> None:
    if len(labels) > scene.maxgeom:
        raise ValueError("too many waypoint labels for the MuJoCo user scene")
    scene.ngeom = len(labels)
    for geom, (label, position) in zip(scene.geoms, labels):
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_LABEL,
            np.zeros(3),
            position,
            np.eye(3).ravel(),
            np.array((1.0, 0.9, 0.1, 1.0)),
        )
        geom.label = label


def _install_keyboard_hook(send_key):
    user32 = ctypes.windll.user32
    kernel32 = ctypes.windll.kernel32
    callback_type = ctypes.WINFUNCTYPE(
        ctypes.c_ssize_t, ctypes.c_int, wintypes.WPARAM, wintypes.LPARAM
    )
    user32.SetWindowsHookExW.argtypes = (
        ctypes.c_int,
        callback_type,
        wintypes.HINSTANCE,
        wintypes.DWORD,
    )
    user32.SetWindowsHookExW.restype = wintypes.HHOOK
    user32.CallNextHookEx.argtypes = (
        wintypes.HHOOK,
        ctypes.c_int,
        wintypes.WPARAM,
        wintypes.LPARAM,
    )
    user32.CallNextHookEx.restype = ctypes.c_ssize_t
    user32.GetForegroundWindow.restype = wintypes.HWND
    user32.GetAsyncKeyState.argtypes = (ctypes.c_int,)
    user32.GetAsyncKeyState.restype = ctypes.c_short
    kernel32.GetModuleHandleW.restype = wintypes.HMODULE
    process_id = os.getpid()

    def callback(code, message, event_address):
        if code >= 0 and message in (
            _WM_KEYDOWN,
            _WM_KEYUP,
            _WM_SYSKEYDOWN,
            _WM_SYSKEYUP,
        ):
            foreground_process = wintypes.DWORD()
            user32.GetWindowThreadProcessId(
                user32.GetForegroundWindow(), ctypes.byref(foreground_process)
            )
            event = ctypes.cast(
                event_address, ctypes.POINTER(_KeyboardEvent)
            ).contents
            modified = bool(
                user32.GetAsyncKeyState(0x11) & 0x8000
                or user32.GetAsyncKeyState(0x12) & 0x8000
            )
            key = _control_key(event.vk_code, modified)
            if foreground_process.value == process_id and key is not None:
                if message in (_WM_KEYDOWN, _WM_SYSKEYDOWN):
                    send_key(key)
                return 1
        return user32.CallNextHookEx(None, code, message, event_address)

    callback_ref = callback_type(callback)
    hook = user32.SetWindowsHookExW(
        13, callback_ref, kernel32.GetModuleHandleW(None), 0
    )
    if not hook:
        raise ctypes.WinError()
    return user32, hook, callback_ref


def _pump_keyboard_hook(user32) -> None:
    message = wintypes.MSG()
    while user32.PeekMessageW(ctypes.byref(message), None, 0, 0, 1):
        user32.TranslateMessage(ctypes.byref(message))
        user32.DispatchMessageW(ctypes.byref(message))


def main() -> None:
    args = _parse_args()
    model = mujoco.MjModel.from_xml_path(args.xml_path)
    if model.nq < 23:
        raise ValueError(f"viewer model needs at least 23 qpos values, got {model.nq}")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    waypoint_labels = _waypoint_labels(model, data)
    model.vis.scale.contactwidth = 0.03 / model.stat.meansize
    model.vis.scale.contactheight = 0.01 / model.stat.meansize
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
    controller_host = [None]

    def send_key(key: str) -> None:
        payload = _encode_key(key)
        if payload is not None and controller_host[0] is not None:
            receiver.sendto(payload, (controller_host[0], args.control_port))
            print(f"Forwarded robot key: {key}", flush=True)

    started = last_frame = time.monotonic()
    received = False
    try:
        with mujoco.viewer.launch_passive(
            model,
            data,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer:
            user32, keyboard_hook, callback_ref = _install_keyboard_hook(send_key)
            print("Focused-window robot keyboard capture active", flush=True)
            with viewer.lock():
                _add_waypoint_labels(viewer.user_scn, waypoint_labels)
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                viewer.cam.trackbodyid = robot_body
                viewer.cam.azimuth = 90
                viewer.cam.elevation = -25
                viewer.cam.distance = 4.0
                viewer.opt.geomgroup[1] = 0
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0

            while viewer.is_running():
                _pump_keyboard_hook(user32)
                latest = None
                while True:
                    try:
                        payload, source = receiver.recvfrom(_QPOS_FRAME.size)
                    except BlockingIOError:
                        break
                    if len(payload) == _QPOS_FRAME.size:
                        candidate = np.asarray(_QPOS_FRAME.unpack(payload))
                        if np.isfinite(candidate).all():
                            latest = candidate
                            controller_host[0] = source[0]

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
            user32.UnhookWindowsHookEx(keyboard_hook)
            del callback_ref
    finally:
        receiver.close()


if __name__ == "__main__":
    main()
