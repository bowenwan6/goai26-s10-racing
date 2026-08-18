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
import json
import math
import os
import time
from pathlib import Path

import mujoco
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String

from s10_perception.heightmap import HeightmapConfig, HeightmapSampler
from s10_perception.lidar import LidarConfig, RayCastLidar
from s10_perception.png import write_png
from s10_perception.segment_spawn import SpawnOverride
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
        self.model_key = model_key or _upstream.MODEL_NAME
        self._actual_joint_owner = "unknown"
        self._active_policy = ""

        self.base_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, _upstream.TRACK_BODY_NAME
        )
        if self.base_body_id < 0:
            raise RuntimeError(f"Body '{_upstream.TRACK_BODY_NAME}' not found in the model")

        self._apply_segment_spawn()
        self._open_segment_video()

        self.lidar = RayCastLidar(self.model, self.base_body_id, lidar_config)
        self.heightmap = HeightmapSampler(self.model, self.base_body_id, heightmap_config)
        self.wheel_body_ids = np.asarray(
            [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                for name in ("fl_wheel", "fr_wheel", "hl_wheel", "hr_wheel")
            ],
            dtype=np.int32,
        )
        if np.any(self.wheel_body_ids < 0):
            raise RuntimeError("S10 wheel bodies are missing from the MuJoCo model")
        self._wheel_geoms = [self._descendant_geoms(int(body)) for body in self.wheel_body_ids]

        self.odom_pub = self.create_publisher(Odometry, "/ground_truth/odom", 50)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.lidar_pub = self.create_publisher(Float32MultiArray, "/perception/lidar", 10)
        self.heightmap_pub = self.create_publisher(Float32MultiArray, "/perception/heightmap", 10)
        self.wheel_state_pub = self.create_publisher(
            Float32MultiArray, "/perception/wheel_state", 10
        )
        self.create_subscription(String, "/joints/owner", self._on_joint_owner, 10)
        self.create_subscription(String, "/strategy/status", self._on_strategy_status, 10)

        self.get_logger().info(
            f"[perception] lidar {self.lidar.cfg.n_elevation}x{self.lidar.cfg.n_azimuth} rays, "
            f"heightmap {self.heightmap.cfg.n_x}x{self.heightmap.cfg.n_y} cells"
        )

    def _apply_segment_spawn(self) -> None:
        """Move the start pose, for the segment harness only.

        A scored run never gets here: :meth:`SpawnOverride.from_env` returns ``None`` unless
        ``S10_SPAWN_XY`` is set, and the whole method is then two comparisons and a return.
        The simulator's own waypoint counter is wound forward to match, so its ``[TRACK]``
        lines are about the segment under test rather than about a start line the robot is
        nowhere near -- it stays an independent check on our recorder, which is the reason
        for not simply ignoring it.
        """
        spawn = SpawnOverride.from_env()
        if spawn is None:
            return

        base = spawn.apply(self.model, self.data, _upstream.JOINT_INIT[self.model_key])
        if (
            spawn.waypoint_index is not None
            and spawn.waypoint_index > 0
            and getattr(self, "track_enabled", False)
        ):
            for index in range(min(spawn.waypoint_index + 1, len(self.track_waypoint_positions))):
                self._hide_track_point(index)
            self.track_next_index = spawn.waypoint_index + 1
        self.get_logger().warn(
            f"[segment] spawn overridden to ({base[0]:.3f}, {base[1]:.3f}, {base[2]:.3f}) "
            f"yaw={math.degrees(spawn.yaw):.1f} deg seed={spawn.seed}; this is a test run, "
            f"not a scored one"
        )

    def _open_segment_video(self) -> None:
        """Set up offscreen frame capture, for the segment harness only.

        Same rule as the spawn override: unset environment means this costs one dictionary
        lookup and nothing else. A headless renderer is expensive enough -- roughly a
        millisecond a frame at this size, against a 1 ms physics step -- that it must not be
        something a scored run can end up paying for by accident. ``replay`` mode records
        only qpos and renders after the run; that is the right choice for high-resolution
        software rendering, which otherwise blocks sensor publication long enough to change
        the navigation behavior being filmed.
        """
        self._frames_dir = None
        self._frame_renderer = None
        self._frame_replay = False
        self._frame_times: list[float] = []
        self._frame_wall_times: list[float] = []
        self._frame_qpos: list[np.ndarray] = []
        self._frame_waypoints: list[int] = []
        self._frame_owners: list[str] = []
        self._frame_policies: list[str] = []
        self._frame_wall_started = time.monotonic()
        directory = os.environ.get("S10_SEGMENT_VIDEO", "").strip()
        if not directory:
            return
        width = int(os.environ.get("S10_SEGMENT_VIDEO_WIDTH", "640"))
        height = int(os.environ.get("S10_SEGMENT_VIDEO_HEIGHT", "480"))
        if width <= 0 or height <= 0:
            raise ValueError("S10_SEGMENT_VIDEO_WIDTH/HEIGHT must be positive integers")
        self._frames_dir = Path(directory)
        self._frames_dir.mkdir(parents=True, exist_ok=True)
        self._frame_width = width
        self._frame_height = height
        capture_hz = float(os.environ.get("S10_SEGMENT_VIDEO_HZ", "10"))
        if capture_hz <= 0.0:
            raise ValueError("S10_SEGMENT_VIDEO_HZ must be positive")
        self._frame_period = 1.0 / capture_hz
        self._frame_next = 0.0
        self._frame_number = 0
        mode = os.environ.get("S10_SEGMENT_VIDEO_MODE", "frames").strip().lower()
        if mode not in {"frames", "replay"}:
            raise ValueError("S10_SEGMENT_VIDEO_MODE must be 'frames' or 'replay'")
        if mode == "replay":
            self._frame_replay = True
            self.get_logger().info(
                f"[segment] recording replay states at {capture_hz:.1f} Hz to "
                f"{self._frames_dir / 'replay.npz'}"
            )
            return

        # MuJoCo's offscreen framebuffer has its own size limit. Raise it before creating
        # the renderer so an explicitly requested 1080p capture is genuinely rendered at
        # 1080p rather than rejected or silently constrained by the model's default buffer.
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, width)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, height)
        self._frame_renderer = mujoco.Renderer(self.model, height=height, width=width)
        self._frame_camera = mujoco.MjvCamera()
        self._frame_camera.distance = float(
            os.environ.get("S10_SEGMENT_VIDEO_CAMERA_DISTANCE", "4.0")
        )
        self._frame_camera.elevation = float(
            os.environ.get("S10_SEGMENT_VIDEO_CAMERA_ELEVATION", "-20.0")
        )
        self._frame_camera.azimuth = float(
            os.environ.get("S10_SEGMENT_VIDEO_CAMERA_AZIMUTH", "90.0")
        )
        self.get_logger().info(
            f"[segment] recording {width}x{height} frames at "
            f"{capture_hz:.1f} Hz to {self._frames_dir}"
        )

    def _capture_frame(self) -> None:
        if self._frames_dir is None or self.timestamp < self._frame_next:
            return
        if self._frame_replay:
            self._frame_times.append(float(self.timestamp))
            self._frame_wall_times.append(time.monotonic() - self._frame_wall_started)
            self._frame_qpos.append(self.data.qpos.copy())
            self._frame_waypoints.append(int(getattr(self, "track_next_index", -1)))
            self._frame_owners.append(self._actual_joint_owner)
            displayed_policy = (
                "WP16" if self._active_policy == "climb_policy" else "official"
            )
            self._frame_policies.append(displayed_policy)
        else:
            self._frame_camera.lookat[:] = self.data.xpos[self.base_body_id]
            self._frame_renderer.update_scene(self.data, self._frame_camera)
            write_png(
                self._frames_dir / f"{self._frame_number:05d}.png",
                self._frame_renderer.render(),
            )
        self._frame_number += 1
        self._frame_next = self.timestamp + self._frame_period

    def destroy_node(self):
        """Flush a lightweight replay trace before ROS tears the simulator down."""
        if self._frame_replay and self._frame_times:
            replay = self._frames_dir / "replay.npz"
            np.savez_compressed(
                replay,
                time=np.asarray(self._frame_times),
                wall_time=np.asarray(self._frame_wall_times),
                qpos=np.asarray(self._frame_qpos),
                target_waypoint=np.asarray(self._frame_waypoints, dtype=np.int16),
                joint_owner=np.asarray(self._frame_owners, dtype="U16"),
                active_policy=np.asarray(self._frame_policies, dtype="U16"),
                width=self._frame_width,
                height=self._frame_height,
            )
            self.get_logger().info(
                f"[segment] wrote {len(self._frame_times)} replay states to {replay}"
            )
            self._frame_replay = False
        return super().destroy_node()

    def _on_joint_owner(self, msg: String) -> None:
        self._actual_joint_owner = msg.data

    def _on_strategy_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self._active_policy = str(status.get("active_policy", self._active_policy))

    def _publish_robot_state(self, step: int) -> None:
        """Extend upstream's state publication with our own sensors."""
        super()._publish_robot_state(step)
        if step % PERCEPTION_DECIMATION == 0:
            self._publish_perception()
            self._capture_frame()

    def _publish_perception(self) -> None:
        stamp = self.get_clock().now().to_msg()

        self._publish_odometry(stamp)

        ranges = self.lidar.scan(self.data)
        self._publish_scan(stamp, self.lidar.horizontal_ring(ranges))
        self.lidar_pub.publish(_to_float_array(ranges, ("elevation", "azimuth")))

        grid = self.heightmap.sample(self.data)
        self.heightmap_pub.publish(_to_float_array(grid, ("x", "y")))
        self.wheel_state_pub.publish(
            _to_float_array(self._wheel_state(), ("wheel", "x_y_z_contact"))
        )

    def _descendant_geoms(self, ancestor_body: int) -> set[int]:
        result: set[int] = set()
        for geom in range(self.model.ngeom):
            body = int(self.model.geom_bodyid[geom])
            while body > 0 and body != ancestor_body:
                body = int(self.model.body_parentid[body])
            if body == ancestor_body:
                result.add(geom)
        return result

    def _wheel_state(self) -> np.ndarray:
        """Four world-frame wheel centres plus a terrain-contact oracle."""
        contacts = np.zeros(4, dtype=np.float32)
        for index, geoms in enumerate(self._wheel_geoms):
            for contact_index in range(self.data.ncon):
                contact = self.data.contact[contact_index]
                if contact.dist > 0.005:
                    continue
                g1, g2 = int(contact.geom1), int(contact.geom2)
                other = g2 if g1 in geoms else g1 if g2 in geoms else -1
                if other >= 0 and int(self.model.geom_group[other]) == 0:
                    contacts[index] = 1.0
                    break
        return np.column_stack((self.data.xpos[self.wheel_body_ids], contacts)).astype(
            np.float32
        )

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
