"""Waypoint follower node.

Consumes ``/ground_truth/odom`` and emits ``/cmd_vel``. The locomotion policy receives that
command through the ROS command interface patched into the contest SDK, so this node is the
entire autonomy layer: swapping it out changes racing behaviour without retraining.

A stall watchdog is included because the dominant failure mode on the elevated sections is
not falling over but wedging against a ledge while the policy happily keeps walking in
place. Detecting that and backing off recovers a run that would otherwise never finish.
"""

from __future__ import annotations

import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32

from s10_auto_nav.pure_pursuit import (
    Command,
    PurePursuitController,
    PursuitGains,
    wrap_angle,
)
from s10_auto_nav.waypoints import Course

CONTROL_RATE_HZ = 50.0


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_follower")

        self.declare_parameter("course_file", "")
        self.declare_parameter("start_waypoint", int(os.environ.get("S10_START_WAYPOINT", "1")))
        self.declare_parameter("control_rate", CONTROL_RATE_HZ)
        self.declare_parameter("advance_radius", 0.35)
        self.declare_parameter("max_forward", PursuitGains.max_forward)
        self.declare_parameter("max_lateral", PursuitGains.max_lateral)
        self.declare_parameter("max_yaw_rate", PursuitGains.max_yaw_rate)
        self.declare_parameter("lookahead", PursuitGains.lookahead)
        self.declare_parameter("yaw_gain", PursuitGains.yaw_gain)
        self.declare_parameter("stall_speed", 0.08)
        self.declare_parameter("stall_timeout", 2.5)
        self.declare_parameter("recovery_duration", 1.0)

        course_file = self.get_parameter("course_file").value
        if not course_file:
            raise RuntimeError("Parameter 'course_file' is required")

        self.control_rate = float(self.get_parameter("control_rate").value)
        start_waypoint = int(self.get_parameter("start_waypoint").value)
        self.course = Course.from_yaml(
            course_file,
            advance_radius=float(self.get_parameter("advance_radius").value),
            start_index=start_waypoint - 1,
        )
        self.controller = PurePursuitController(
            PursuitGains(
                max_forward=float(self.get_parameter("max_forward").value),
                max_lateral=float(self.get_parameter("max_lateral").value),
                max_yaw_rate=float(self.get_parameter("max_yaw_rate").value),
                lookahead=float(self.get_parameter("lookahead").value),
                yaw_gain=float(self.get_parameter("yaw_gain").value),
            )
        )

        self.stall_speed = float(self.get_parameter("stall_speed").value)
        self.stall_timeout = float(self.get_parameter("stall_timeout").value)
        self.recovery_duration = float(self.get_parameter("recovery_duration").value)

        self._pose_xy: np.ndarray | None = None
        self._yaw = 0.0
        self._speed = 0.0
        self._stalled_for = 0.0
        self._recovering_for = 0.0

        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.progress_pub = self.create_publisher(Float32, "/nav/progress", latched)
        self.finished_pub = self.create_publisher(Bool, "/nav/finished", latched)
        self.create_subscription(Odometry, "/ground_truth/odom", self._odom_callback, 50)

        self.timer = self.create_timer(1.0 / self.control_rate, self._control_step)
        self.get_logger().info(
            f"Following {len(self.course)} waypoints from path waypoint {start_waypoint} "
            f"in {course_file} "
            f"at {self.control_rate:.0f} Hz"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        self._pose_xy = np.array([p.x, p.y])
        self._yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._speed = float(math.hypot(v.x, v.y))

    def _control_step(self) -> None:
        if self._pose_xy is None:
            return  # No pose yet; stay silent rather than command blind.

        dt = 1.0 / self.control_rate

        if self.course.update(self._pose_xy):
            reached = self.course.cursor - 1
            self.get_logger().info(
                f"Waypoint {reached} reached ({self.course.cursor}/{len(self.course)}), "
                f"{self.course.remaining_distance(self._pose_xy):.1f} m remaining"
            )
            self._publish_progress()

        if self.course.finished:
            self._publish_stop()
            return

        if self._recovering_for > 0.0:
            self._recovering_for -= dt
            self._publish(self._recovery_command())
            return

        self._update_stall_watchdog(dt)

        target = self.course.lookahead_point(self._pose_xy, self.controller.lookahead_distance())
        command = self.controller.compute(self._pose_xy, self._yaw, target, dt)
        self._publish(command)

    def _update_stall_watchdog(self, dt: float) -> None:
        """Trip into recovery when the robot is commanded to move but is not moving."""
        commanded = abs(self.controller.last_command.forward) > 0.1
        if commanded and self._speed < self.stall_speed:
            self._stalled_for += dt
            if self._stalled_for >= self.stall_timeout:
                self.get_logger().warn(
                    f"Stalled for {self._stalled_for:.1f}s at waypoint {self.course.cursor}; "
                    "backing off"
                )
                self._recovering_for = self.recovery_duration
                self._stalled_for = 0.0
                self.controller.reset()
        else:
            self._stalled_for = 0.0

    def _recovery_command(self) -> Command:
        """Reverse while yawing toward the target to unwedge from a ledge or wall."""
        target = self.course.target
        yaw_rate = 0.0
        if target is not None and self._pose_xy is not None:
            delta = target.xy - self._pose_xy
            yaw_rate = float(
                np.clip(wrap_angle(math.atan2(delta[1], delta[0]) - self._yaw), -0.8, 0.8)
            )
        return Command(forward=-0.4, lateral=0.0, yaw_rate=yaw_rate)

    def _publish(self, command: Command) -> None:
        msg = Twist()
        msg.linear.x = float(command.forward)
        msg.linear.y = float(command.lateral)
        msg.angular.z = float(command.yaw_rate)
        self.cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())
        if not getattr(self, "_announced_finish", False):
            self._announced_finish = True
            self.get_logger().info("Course complete; holding position")
            self.finished_pub.publish(Bool(data=True))
            self._publish_progress()

    def _publish_progress(self) -> None:
        self.progress_pub.publish(Float32(data=float(self.course.cursor) / len(self.course)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
