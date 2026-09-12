"""Perception-augmented MuJoCo simulator node.

Subclasses the contest simulator instead of forking it: the physics loop, joint command
handling, waypoint timing and scoring all remain upstream's, and we only add publishers
for the sensors the contest expects us to build ourselves.

Published in addition to upstream's ``/IMU_DATA`` and ``/JOINTS_DATA``:

==============================  ==========================  ====================
Topic                           Type                        Contents
==============================  ==========================  ====================
``/ground_truth/odom``          ``nav_msgs/Odometry``       base pose and twist
``/scan``                       ``sensor_msgs/LaserScan``   horizontal lidar ring
``/perception/lidar``           ``std_msgs/Float32MultiArray``  full range image
``/perception/heightmap``       ``std_msgs/Float32MultiArray``  body-frame grid
==============================  ==========================  ====================

Reading the base pose from the simulator is explicitly permitted by the contest rules:
SLAM is not required in simulation. On hardware the same topic is fed by odometry, so
downstream nodes need no change.
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import time

import mujoco
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Float32MultiArray, MultiArrayDimension, UInt8

from s10_perception.heightmap import HeightmapConfig, HeightmapSampler
from s10_perception.lidar import LidarConfig, RayCastLidar
from s10_perception.upstream import load_simulator_module

_upstream = load_simulator_module()

#: Publish perception at 50 Hz to match the policy rate. The base loop runs at 1 kHz and
#: upstream publishes proprioception every 5 steps, so we decimate by 20.
PERCEPTION_DECIMATION = 20
_MANUAL_FRAME = struct.Struct("!4sB16f")
_MANUAL_MAGIC = b"S10C"
_KEY_FRAME = struct.Struct("!4sB")
_KEY_MAGIC = b"S10K"
_CONTROL_KEYS = frozenset(b"0rzcxvmhplkwasdqe12345678[]")
_WHEEL_JOINTS = np.array([3, 7, 11, 15])


def _decode_manual_control(payload: bytes):
    if len(payload) != _MANUAL_FRAME.size:
        return None
    magic, mode, *targets = _MANUAL_FRAME.unpack(payload)
    targets = np.asarray(targets, dtype=np.float32)
    if magic != _MANUAL_MAGIC or mode not in (0, 1, 2) or not np.isfinite(targets).all():
        return None
    return mode, targets


def _decode_key(payload: bytes) -> int | None:
    if len(payload) != _KEY_FRAME.size:
        return None
    magic, key = _KEY_FRAME.unpack(payload)
    return key if magic == _KEY_MAGIC and key in _CONTROL_KEYS else None


def _set_wheel_friction(model, friction: float) -> None:
    if not math.isfinite(friction) or friction <= 0:
        raise ValueError("S10_WHEEL_FRICTION must be positive and finite")
    for name in ("fl_wheel", "fr_wheel", "hl_wheel", "hr_wheel"):
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        geoms = (model.geom_bodyid == body) & (model.geom_group == 1)
        model.geom_friction[geoms, 0] = friction


class PerceptionSimulationNode(_upstream.MuJoCoSimulationNode):
    """Upstream simulator plus synthetic exteroception."""

    def __init__(
        self,
        model_key: str | None = None,
        xml_path: str | None = None,
        lidar_config: LidarConfig | None = None,
        heightmap_config: HeightmapConfig | None = None,
    ) -> None:
        # Batch evaluation runs many laps without a display; the viewer is a module-level
        # switch upstream, so it is overridden before the base class reads it.
        if os.environ.get("S10_USE_VIEWER", "1") == "0":
            _upstream.USE_VIEWER = False

        kwargs = {}
        if model_key is not None:
            kwargs["model_key"] = model_key
        if xml_path is not None:
            kwargs["xml_path"] = xml_path
        super().__init__(**kwargs)
        _set_wheel_friction(
            self.model, float(os.environ.get("S10_WHEEL_FRICTION", "2.0"))
        )

        self.base_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, _upstream.TRACK_BODY_NAME
        )
        if self.base_body_id < 0:
            raise RuntimeError(f"Body '{_upstream.TRACK_BODY_NAME}' not found in the model")

        self._set_start_waypoint()
        self._reset_qpos = self.data.qpos.copy()

        self.lidar = RayCastLidar(self.model, self.base_body_id, lidar_config)
        self.heightmap = HeightmapSampler(self.model, self.base_body_id, heightmap_config)
        self.use_perception = os.environ.get("S10_USE_PERCEPTION", "1") != "0"
        self.use_lidar = self.use_perception and os.environ.get("S10_USE_LIDAR", "1") != "0"
        self.use_heightmap = self.use_perception and os.environ.get("S10_USE_HEIGHTMAP", "1") != "0"

        self.odom_pub = self.create_publisher(Odometry, "/ground_truth/odom", 50)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.lidar_pub = self.create_publisher(Float32MultiArray, "/perception/lidar", 10)
        self.heightmap_pub = self.create_publisher(Float32MultiArray, "/perception/heightmap", 10)
        self.key_pub = self.create_publisher(UInt8, "/keyboard/key", 10)
        self.reset_sub = self.create_subscription(Empty, "/sim/reset", self._reset_sim, 1)

        manual_port = int(os.environ.get("S10_MANUAL_PORT", "18778"))
        if not 1 <= manual_port <= 65535:
            raise ValueError("S10_MANUAL_PORT must be between 1 and 65535")
        self._manual_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._manual_socket.bind(("0.0.0.0", manual_port))
        self._manual_socket.setblocking(False)
        self._manual_mode = 0
        self._manual_targets = np.zeros(16, dtype=np.float32)
        self._manual_last_packet = 0.0
        self.get_logger().info(f"[manual] joint control UDP listening on {manual_port}")

        if self.use_lidar or self.use_heightmap:
            sensors = []
            if self.use_lidar:
                sensors.append(
                    f"lidar {self.lidar.cfg.n_elevation}x{self.lidar.cfg.n_azimuth} rays"
                )
            if self.use_heightmap:
                sensors.append(f"heightmap {self.heightmap.cfg.n_x}x{self.heightmap.cfg.n_y} cells")
            self.get_logger().info(f"[perception] {', '.join(sensors)}")
        else:
            self.get_logger().info("[perception] lidar and heightmap disabled")

    def _set_start_waypoint(self) -> None:
        value = os.environ.get("S10_START_WAYPOINT")
        if value is None:
            return
        try:
            waypoint_number = int(value)
        except ValueError as exc:
            raise ValueError("S10_START_WAYPOINT must be an integer") from exc
        if not 1 <= waypoint_number < len(self.track_waypoint_positions):
            raise ValueError(
                f"S10_START_WAYPOINT must be between 1 and {len(self.track_waypoint_positions) - 1}"
            )

        start_index = waypoint_number - 1
        start = self.track_waypoint_positions[start_index]
        target = self.track_waypoint_positions[start_index + 1]
        path_yaw = math.atan2(target[1] - start[1], target[0] - start[0])
        yaw = math.radians(float(os.environ["S10_START_YAW_DEG"])) if os.environ.get(
            "S10_START_YAW_DEG"
        ) else path_yaw
        yaw += math.radians(float(os.environ.get("S10_START_YAW_OFFSET_DEG", "0")))
        forward_offset = float(os.environ.get("S10_START_FORWARD_OFFSET", "0"))
        lateral_offset = float(os.environ.get("S10_START_LATERAL_OFFSET", "0"))
        offset_xy = (
            forward_offset * np.array([math.cos(path_yaw), math.sin(path_yaw)])
            + lateral_offset * np.array([-math.sin(path_yaw), math.cos(path_yaw)])
        )
        self.data.qpos[:3] = start + np.array([offset_xy[0], offset_xy[1], 0.2])
        self.data.qpos[3:7] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.get_logger().info(
            f"[sim] starting at path waypoint {waypoint_number} "
            f"toward waypoint {waypoint_number + 1}: "
            f"x={start[0]:.3f}, y={start[1]:.3f}, yaw={math.degrees(yaw):.1f}deg"
        )

    def _reset_sim(self, _msg: Empty) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self._reset_qpos
        for command in (
            self.kp_cmd,
            self.kd_cmd,
            self.pos_cmd,
            self.vel_cmd,
            self.tau_ff,
            self.input_tq,
        ):
            command.fill(0.0)
        self.last_base_linvel.fill(0.0)
        mujoco.mj_forward(self.model, self.data)
        self.get_logger().info("[sim] reset to the test start pose")

    def _poll_manual_control(self) -> None:
        while True:
            try:
                payload = self._manual_socket.recv(_MANUAL_FRAME.size + 1)
            except BlockingIOError:
                return
            key = _decode_key(payload)
            if key is not None:
                self.key_pub.publish(UInt8(data=key))
                continue
            command = _decode_manual_control(payload)
            if command is None:
                continue
            mode, targets = command
            for index in range(16):
                if index in _WHEEL_JOINTS:
                    targets[index] = np.clip(targets[index], -20.0, 20.0)
                    continue
                joint = self.model.actuator_trnid[index, 0]
                if self.model.jnt_limited[joint]:
                    targets[index] = np.clip(targets[index], *self.model.jnt_range[joint])
            if mode != self._manual_mode:
                self.get_logger().info(f"[manual] mode {self._manual_mode} -> {mode}")
            self._manual_mode = mode
            self._manual_targets = targets
            self._manual_last_packet = time.monotonic()

    def _apply_joint_torque(self) -> None:
        self._poll_manual_control()
        if self._manual_mode == 0:
            super()._apply_joint_torque()
            return
        if self._manual_mode == 1 and time.monotonic() - self._manual_last_packet > 0.5:
            self._manual_mode = 2
            self.get_logger().warn("[manual] control heartbeat lost; entering damping")

        q = self.data.qpos[7:23].reshape(-1, 1)
        dq = self.data.qvel[6:22].reshape(-1, 1)
        if self._manual_mode == 2:
            self.input_tq = -2.0 * dq
        else:
            kp = np.full((16, 1), 80.0, dtype=np.float32)
            kp[_WHEEL_JOINTS] = 0.0
            pos = self._manual_targets.reshape(-1, 1).copy()
            pos[_WHEEL_JOINTS] = q[_WHEEL_JOINTS]
            vel = np.zeros((16, 1), dtype=np.float32)
            vel[_WHEEL_JOINTS] = self._manual_targets[_WHEEL_JOINTS, None]
            self.input_tq = kp * (pos - q) + 2.0 * (vel - dq)
        self.data.ctrl[:] = self.input_tq.ravel()

    def destroy_node(self):
        self._manual_socket.close()
        return super().destroy_node()

    def _publish_robot_state(self, step: int) -> None:
        """Extend upstream's state publication with our own sensors."""
        super()._publish_robot_state(step)
        if step % PERCEPTION_DECIMATION == 0:
            self._publish_perception()

    def _publish_perception(self) -> None:
        stamp = self.get_clock().now().to_msg()

        self._publish_odometry(stamp)
        if not self.use_perception:
            return

        if self.use_lidar:
            ranges = self.lidar.scan(self.data)
            self._publish_scan(stamp, self.lidar.horizontal_ring(ranges))
            self.lidar_pub.publish(_to_float_array(ranges, ("elevation", "azimuth")))

        if self.use_heightmap:
            grid = self.heightmap.sample(self.data)
            self.heightmap_pub.publish(_to_float_array(grid, ("x", "y")))

    def _publish_odometry(self, stamp) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        msg.child_frame_id = _upstream.TRACK_BODY_NAME

        pos = self.data.xpos[self.base_body_id]
        quat = self.data.xquat[self.base_body_id]  # MuJoCo order: (w, x, y, z)
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation.w = float(quat[0])
        msg.pose.pose.orientation.x = float(quat[1])
        msg.pose.pose.orientation.y = float(quat[2])
        msg.pose.pose.orientation.z = float(quat[3])

        # The free joint occupies qvel[0:6]: linear velocity in world frame, then angular.
        linvel = self.data.qvel[0:3]
        angvel = self.data.qvel[3:6]
        msg.twist.twist.linear.x = float(linvel[0])
        msg.twist.twist.linear.y = float(linvel[1])
        msg.twist.twist.linear.z = float(linvel[2])
        msg.twist.twist.angular.x = float(angvel[0])
        msg.twist.twist.angular.y = float(angvel[1])
        msg.twist.twist.angular.z = float(angvel[2])

        self.odom_pub.publish(msg)

    def _publish_scan(self, stamp, ring: np.ndarray) -> None:
        angles = self.lidar.azimuth_angles
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = "lidar_link"
        msg.angle_min = float(angles[0])
        msg.angle_max = float(angles[-1])
        msg.angle_increment = float(angles[1] - angles[0]) if len(angles) > 1 else 0.0
        msg.time_increment = 0.0
        msg.scan_time = PERCEPTION_DECIMATION * _upstream.DT
        msg.range_min = float(self.lidar.cfg.range_min)
        msg.range_max = float(self.lidar.cfg.range_max)
        msg.ranges = ring.astype(np.float32).tolist()
        self.scan_pub.publish(msg)


def _to_float_array(array: np.ndarray, labels: tuple[str, ...]) -> Float32MultiArray:
    """Wrap a 2D array in a Float32MultiArray with a fully described layout."""
    msg = Float32MultiArray()
    stride = int(array.size)
    for axis, label in enumerate(labels):
        dim = MultiArrayDimension()
        dim.label = label
        dim.size = int(array.shape[axis])
        dim.stride = stride
        stride //= int(array.shape[axis])
        msg.layout.dim.append(dim)
    msg.data = array.astype(np.float32).ravel().tolist()
    return msg


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Run the S10 MuJoCo simulation with synthetic perception."
    )
    parser.add_argument(
        "--xml-path",
        default=os.environ.get("S10_MUJOCO_XML"),
        help="Custom MJCF path. Defaults to S10_MUJOCO_XML, then the upstream track scene.",
    )
    parser.add_argument("--model-key", default=None, help="Robot key for the initial pose.")
    parser.add_argument("--lidar-azimuth", type=int, default=LidarConfig.n_azimuth)
    parser.add_argument("--lidar-elevation", type=int, default=LidarConfig.n_elevation)
    parser.add_argument("--lidar-range", type=float, default=LidarConfig.range_max)
    args, ros_args = parser.parse_known_args()
    # Launch substitutions resolve to an empty string when unset; treat that as "default".
    args.xml_path = args.xml_path or None
    args.model_key = args.model_key or None
    return args, ros_args


def main() -> None:
    args, ros_args = _parse_args()
    rclpy.init(args=ros_args)

    lidar_config = LidarConfig(
        n_azimuth=args.lidar_azimuth,
        n_elevation=args.lidar_elevation,
        range_max=args.lidar_range,
    )

    node = PerceptionSimulationNode(
        model_key=args.model_key,
        xml_path=args.xml_path,
        lidar_config=lidar_config,
    )
    try:
        node.start()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
