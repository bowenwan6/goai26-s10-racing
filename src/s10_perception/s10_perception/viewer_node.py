"""MuJoCo viewer decoupled from the simulation/control process."""

from __future__ import annotations

import argparse
import os
import socket
import struct
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from drdds.msg import JointsData
from nav_msgs.msg import Odometry
from rclpy.node import Node

from s10_perception.upstream import default_track_xml, load_simulator_module

_upstream = load_simulator_module()
_QPOS_FRAME = struct.Struct("!23d")


def _raw_joint_positions(published) -> np.ndarray:
    positions = np.fromiter(published, dtype=np.float64, count=16)
    return positions * _upstream.JOINT_DIR + _upstream.POS_OFFSET_RAD


class ViewerNode(Node):
    def __init__(self, xml_path: str | None = None) -> None:
        super().__init__("mujoco_viewer")
        self.model = mujoco.MjModel.from_xml_path(xml_path) if xml_path else None
        self.data = mujoco.MjData(self.model) if self.model else None
        self.qpos = self.data.qpos.copy() if self.data else np.zeros(23)
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_odom, 10)
        self.create_subscription(JointsData, "/JOINTS_DATA", self._on_joints, 10)

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.qpos[:3] = (pose.position.x, pose.position.y, pose.position.z)
        self.qpos[3:7] = (
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        )

    def _on_joints(self, msg: JointsData) -> None:
        joints = msg.data.joints_data
        if len(joints) == 16:
            self.qpos[7:23] = _raw_joint_positions(joint.position for joint in joints)

    def sync(self, viewer) -> None:
        assert self.model is not None and self.data is not None
        self.data.qpos[:] = self.qpos
        mujoco.mj_forward(self.model, self.data)
        viewer.sync()


def _stream_to_windows(node: ViewerNode, host: str, port: int, hz: float) -> None:
    destination = (host, port)
    period = 1.0 / hz
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        node.get_logger().info(f"Streaming viewer state to Windows at {host}:{port}")
        next_send = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=max(0.0, next_send - time.monotonic()))
            now = time.monotonic()
            if now >= next_send:
                sender.sendto(_QPOS_FRAME.pack(*node.qpos), destination)
                next_send = now + period


def _record_qpos(node: ViewerNode, path: str, hz: float, duration: float) -> None:
    period = 1.0 / hz
    started = next_sample = time.monotonic()
    samples = []
    while rclpy.ok() and time.monotonic() - started < duration:
        rclpy.spin_once(node, timeout_sec=max(0.0, next_sample - time.monotonic()))
        now = time.monotonic()
        if now >= next_sample:
            samples.append(node.qpos.copy())
            next_sample = now + period
    np.save(path, np.asarray(samples))
    node.get_logger().info(f"Saved {len(samples)} qpos frames to {path}")


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml-path", default=os.environ.get("S10_MUJOCO_XML"))
    parser.add_argument("--hz", type=float, default=float(os.environ.get("S10_VIEWER_HZ", "60")))
    parser.add_argument("--stream-host", help="Send qpos frames to a Windows viewer")
    parser.add_argument("--stream-port", type=int, default=18777)
    parser.add_argument("--record-path", help="Save synchronized qpos frames as a NumPy array")
    parser.add_argument("--duration", type=float, default=30.0)
    args, ros_args = parser.parse_known_args()
    args.xml_path = args.xml_path or str(default_track_xml())
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if not 1 <= args.stream_port <= 65535:
        parser.error("--stream-port must be between 1 and 65535")
    if args.duration <= 0:
        parser.error("--duration must be positive")
    return args, ros_args


def main() -> None:
    args, ros_args = _parse_args()
    rclpy.init(args=ros_args)
    node = ViewerNode(None if args.stream_host or args.record_path else args.xml_path)
    period = 1.0 / args.hz
    try:
        if args.record_path:
            _record_qpos(node, args.record_path, args.hz, args.duration)
            return
        if args.stream_host:
            _stream_to_windows(node, args.stream_host, args.stream_port, args.hz)
            return

        node.get_logger().info("Detached WSLg viewer ready; simulation remains headless")
        assert node.model is not None and node.data is not None
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            with viewer.lock():
                viewer.cam.lookat[:] = _upstream.TRACK_START_BASE_POS
                viewer.cam.azimuth = _upstream.CAMERA_AZIMUTH
                viewer.cam.elevation = _upstream.CAMERA_ELEVATION
                viewer.cam.distance = _upstream.CAMERA_DISTANCE
                viewer.opt.geomgroup[_upstream.COLLISION_GEOM_GROUP] = 0
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 1

            next_sync = time.monotonic()
            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(node, timeout_sec=max(0.0, next_sync - time.monotonic()))
                now = time.monotonic()
                if now >= next_sync:
                    node.sync(viewer)
                    next_sync = now + period
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
