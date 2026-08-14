"""Renderable views of the perception the follower actually steers on.

The contest requires the perception-control fusion to be shown as scattered points or
rays, and neither perception topic can be drawn as published: ``/perception/lidar`` and
``/perception/heightmap`` are ``Float32MultiArray``, which RViz has no plugin for, and
nothing in the stack broadcasts TF, so even ``/scan`` has no frame to be drawn in.

This node closes both gaps without touching the simulator. It is deliberately a separate
process: the simulator carries the 1 kHz physics loop and the policy's deadline, and
turning range images into point clouds is display work that should not share that thread.
Not launching it costs nothing but the picture.

Published:

==================================  =========================  ======================
Topic                               Type                       Frame
==================================  =========================  ======================
``/perception/lidar_points``        ``sensor_msgs/PointCloud2``  ``lidar_link``
``/perception/heightmap_points``    ``sensor_msgs/PointCloud2``  ``base_yaw``
``/tf``, ``/tf_static``             ``tf2_msgs/TFMessage``     see below
==================================  =========================  ======================

Three frames, because the two sensors genuinely live in different ones:

``world`` -> ``base_link``
    Full base pose. The lidar rotates with the body, pitch and roll included, which is
    why a nose-down robot sees the floor ahead as an obstacle.
``base_link`` -> ``lidar_link``
    Static mount offset.
``world`` -> ``base_yaw``
    Base position and heading only. The height map is sampled on a yaw-aligned grid with
    heights measured from the base, so it is level by construction even when the body is
    not; drawing it in ``base_link`` would tilt it away from the terrain it describes.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from s10_perception.heightmap import HeightmapConfig, grid_points
from s10_perception.lidar import LidarConfig, ray_directions

WORLD_FRAME = "world"
BASE_FRAME = "base_link"
LIDAR_FRAME = "lidar_link"
#: Yaw-aligned, level frame at the base origin; see the module docstring.
BASE_YAW_FRAME = "base_yaw"


class PerceptionVizNode(Node):
    """Republishes the perception arrays as point clouds, and broadcasts their frames."""

    def __init__(
        self,
        lidar_config: LidarConfig | None = None,
        heightmap_config: HeightmapConfig | None = None,
    ) -> None:
        super().__init__("perception_viz")

        self.lidar_cfg = lidar_config or LidarConfig()
        self.heightmap_cfg = heightmap_config or HeightmapConfig()

        # The ray pattern and the grid are reconstructed from the same config objects the
        # simulator samples with, rather than re-derived here, so the picture cannot drift
        # away from the data if either geometry is retuned.
        self._ray_dirs = ray_directions(self.lidar_cfg)
        self._grid_xy = grid_points(self.heightmap_cfg)

        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(self._lidar_mount_transform())

        self.lidar_cloud_pub = self.create_publisher(PointCloud2, "/perception/lidar_points", 5)
        self.height_cloud_pub = self.create_publisher(
            PointCloud2, "/perception/heightmap_points", 5
        )

        self.create_subscription(Odometry, "/ground_truth/odom", self._odom_callback, 20)
        self.create_subscription(
            Float32MultiArray, "/perception/lidar", self._lidar_callback, 5
        )
        self.create_subscription(
            Float32MultiArray, "/perception/heightmap", self._heightmap_callback, 5
        )

        self.get_logger().info(
            "[viz] publishing /perception/lidar_points and /perception/heightmap_points"
        )

    def _lidar_mount_transform(self) -> TransformStamped:
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = BASE_FRAME
        tf.child_frame_id = LIDAR_FRAME
        offset = np.asarray(self.lidar_cfg.mount_offset, float)
        tf.transform.translation.x = float(offset[0])
        tf.transform.translation.y = float(offset[1])
        tf.transform.translation.z = float(offset[2])
        tf.transform.rotation.w = 1.0
        return tf

    def _odom_callback(self, msg: Odometry) -> None:
        stamp = msg.header.stamp
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        base = TransformStamped()
        base.header.stamp = stamp
        base.header.frame_id = WORLD_FRAME
        base.child_frame_id = BASE_FRAME
        base.transform.translation.x = p.x
        base.transform.translation.y = p.y
        base.transform.translation.z = p.z
        base.transform.rotation = q

        # Same origin, heading only: yaw survives, pitch and roll are dropped.
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        level = TransformStamped()
        level.header.stamp = stamp
        level.header.frame_id = WORLD_FRAME
        level.child_frame_id = BASE_YAW_FRAME
        level.transform.translation.x = p.x
        level.transform.translation.y = p.y
        level.transform.translation.z = p.z
        level.transform.rotation.z = math.sin(yaw / 2.0)
        level.transform.rotation.w = math.cos(yaw / 2.0)

        self.tf.sendTransform([base, level])

    def _lidar_callback(self, msg: Float32MultiArray) -> None:
        ranges = _as_array(msg)
        if ranges is None or ranges.size != self._ray_dirs.shape[0]:
            return

        # A ray that reached the cutoff hit nothing. Plotting it anyway would draw a shell
        # of phantom returns at 12 m and hide the geometry that is actually there.
        hit = ranges < self.lidar_cfg.range_max - 1e-3
        points = self._ray_dirs[hit] * ranges[hit, None]
        self.lidar_cloud_pub.publish(self._cloud(LIDAR_FRAME, points))

    def _heightmap_callback(self, msg: Float32MultiArray) -> None:
        heights = _as_array(msg)
        if heights is None or heights.size != self._grid_xy.shape[0]:
            return
        points = np.column_stack([self._grid_xy, heights])
        self.height_cloud_pub.publish(self._cloud(BASE_YAW_FRAME, points))

    def _cloud(self, frame_id: str, points: np.ndarray) -> PointCloud2:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return point_cloud2.create_cloud_xyz32(header, points.astype(np.float32).tolist())


def _as_array(msg: Float32MultiArray) -> np.ndarray | None:
    """Flat float view of a Float32MultiArray, or None if it is empty."""
    data = np.asarray(msg.data, dtype=np.float32)
    return data if data.size else None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionVizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGTERM tears the context down before the exception reaches here, so shutting it
        # down again below raises on a context that is already gone -- turning an ordinary
        # teardown into a traceback that reads like a crash. Every node in a stopped stack
        # printed one, which is noise in exactly the logs a failed run is read from.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
