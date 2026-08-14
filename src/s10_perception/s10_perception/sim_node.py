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
import os

import mujoco
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from s10_perception.heightmap import HeightmapConfig, HeightmapSampler
from s10_perception.lidar import LidarConfig, RayCastLidar
from s10_perception.upstream import load_simulator_module

_upstream = load_simulator_module()

#: Publish perception at 50 Hz to match the policy rate. The base loop runs at 1 kHz and
#: upstream publishes proprioception every 5 steps, so we decimate by 20.
PERCEPTION_DECIMATION = 20


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

        self.base_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, _upstream.TRACK_BODY_NAME
        )
        if self.base_body_id < 0:
            raise RuntimeError(f"Body '{_upstream.TRACK_BODY_NAME}' not found in the model")

        self.lidar = RayCastLidar(self.model, self.base_body_id, lidar_config)
        self.heightmap = HeightmapSampler(self.model, self.base_body_id, heightmap_config)

        self.odom_pub = self.create_publisher(Odometry, "/ground_truth/odom", 50)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.lidar_pub = self.create_publisher(Float32MultiArray, "/perception/lidar", 10)
        self.heightmap_pub = self.create_publisher(Float32MultiArray, "/perception/heightmap", 10)

        self.get_logger().info(
            f"[perception] lidar {self.lidar.cfg.n_elevation}x{self.lidar.cfg.n_azimuth} rays, "
            f"heightmap {self.heightmap.cfg.n_x}x{self.heightmap.cfg.n_y} cells"
        )

    def _publish_robot_state(self, step: int) -> None:
        """Extend upstream's state publication with our own sensors."""
        super()._publish_robot_state(step)
        if step % PERCEPTION_DECIMATION == 0:
            self._publish_perception()

    def _publish_perception(self) -> None:
        stamp = self.get_clock().now().to_msg()

        self._publish_odometry(stamp)

        ranges = self.lidar.scan(self.data)
        self._publish_scan(stamp, self.lidar.horizontal_ring(ranges))
        self.lidar_pub.publish(_to_float_array(ranges, ("elevation", "azimuth")))

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
    except (KeyboardInterrupt, ExternalShutdownException):
        # See viz_node.main: SIGTERM has already closed the context by this point.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
