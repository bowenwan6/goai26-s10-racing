#!/usr/bin/env python3
"""Publish the fixed turn/stop/straight velocity profile without ROS CLI startup gaps."""

import time
import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main() -> None:
    rclpy.init()
    node = Node("velocity_profile_commands")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    phases = (
        ((1.0, 0.0, 0.0), (8.0, 0.8, 0.8), (2.0, 0.0, 0.0), (8.0, 1.2, -0.6), (2.0, 0.0, 0.0))
        if os.getenv("S10_PROFILE_MIXED")
        else ((1.0, 0.0, 0.0), (8.0, 0.0, 1.0), (2.0, 0.0, 0.0), (8.0, 1.2, 0.0), (2.0, 0.0, 0.0))
    )
    try:
        for duration, forward, yaw_rate in phases:
            command = Twist()
            command.linear.x = forward
            command.angular.z = yaw_rate
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
