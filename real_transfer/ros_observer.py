"""Bounded read-only ROS collector for later OFFLINE shadow replay.

Run on 106 only after reviewing topic/frame configuration. Does not launch any
controller or localization service. No publisher, service client or motion switch.
The collector intentionally does not run policy inference on the robot.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
from pathlib import Path

from real_transfer.geometry import decode_pointcloud2
from real_transfer.vendor import read_status


def header(msg, mono):
    return {
        "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        "received": mono,
        "frame": msg.header.frame_id,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=10)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 30:
        parser.error("collection must be bounded to 0 < seconds <= 30")
    config = json.loads(args.config.read_text())
    if config.get("mode") != "shadow_only":
        parser.error("only shadow_only mode exists")

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2

    rclpy.init(args=[])
    node = rclpy.create_node(
        "s10_readonly_transfer_collector", enable_rosout=False, start_parameter_services=False
    )
    latest = {}
    errors = {}

    def on_pose(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        latest["pose"] = dict(
            header(msg, time.monotonic()),
            child_frame=msg.child_frame_id,
            position=[p.x, p.y, p.z],
            orientation=[q.x, q.y, q.z, q.w],
        )

    def on_cloud(msg):
        try:
            points = decode_pointcloud2(msg)
            import numpy as np

            points = points[np.isfinite(points).all(axis=1)]
            latest["cloud"] = dict(header(msg, time.monotonic()), points=points.tolist())
            errors.pop("cloud", None)
        except (ValueError, TypeError) as exc:
            latest.pop("cloud", None)
            errors["cloud"] = str(exc)

    def on_scan(msg):
        ranges = [float(r) for r in msg.ranges]
        # JSON's null represents unknown, not free. Positive infinity is represented
        # by an explicit no-return mask and reconstructed only by the offline loader.
        latest["scan"] = dict(
            header(msg, time.monotonic()),
            angles=[msg.angle_min + i * msg.angle_increment for i in range(len(ranges))],
            ranges=[r if math.isfinite(r) else None for r in ranges],
            no_return=[r == math.inf for r in ranges],
            range_min=msg.range_min,
            range_max=msg.range_max,
        )

    topics = config["topics"]
    for key, message, callback in (
        ("pose", Odometry, on_pose),
        ("cloud", PointCloud2, on_cloud),
        ("scan", LaserScan, on_scan),
    ):
        if topics.get(key):
            node.create_subscription(message, topics[key], callback, qos_profile_sensor_data)
    started = time.monotonic()
    next_status = next_record = started
    try:
        with args.output.open("x") as out:
            while rclpy.ok() and time.monotonic() - started < args.seconds:
                rclpy.spin_once(node, timeout_sec=0.01)
                mono = time.monotonic()
                if mono >= next_status:
                    try:
                        status = read_status()
                        old = latest.get("localization")
                        # A repeated log line is not a new status measurement.
                        if old and (old["stamp"], old["session_id"], old["map_id"]) == (
                            status["stamp"],
                            status["session_id"],
                            status["map_id"],
                        ):
                            status["received"] = old["received"]
                        latest["localization"] = status
                        errors.pop("localization", None)
                    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
                        latest.pop("localization", None)
                        errors["localization"] = str(exc)
                    next_status = time.monotonic() + 0.5
                if mono >= next_record:
                    record = {
                        "wall_time": time.time(),
                        "monotonic_time": time.monotonic(),
                        "inputs": latest,
                        "collection_errors": dict(errors),
                    }
                    out.write(json.dumps(record, allow_nan=False) + "\n")
                    next_record = time.monotonic() + 0.05
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
