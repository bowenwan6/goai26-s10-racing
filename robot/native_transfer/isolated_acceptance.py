"""ROS wire test with synthetic sensors, guarded to localhost/domain 211.

This tests serialization, the real follower, router, acknowledgement and cancel
path. It is never evidence of physical policy response or terrain traversal.
"""

import io
import json
import math
import os
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "211" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("wire test requires ROS_DOMAIN_ID=211 and ROS_LOCALHOST_ONLY=1")
    import rclpy
    from drdds.msg import Gait, LocationStatus, MotionInfo, NavCmd, StdMsgInt32
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header, String
    from std_srvs.srv import Trigger

    from native_transfer.contracts import VERIFICATIONS
    from native_transfer.router import GAITS
    from native_transfer.runtime import NativeTest

    kind = os.environ.get("NATIVE_TEST_GAIT", "flat")
    if kind not in GAITS:
        raise ValueError("NATIVE_TEST_GAIT must be flat or stairs")

    rclpy.init(args=[])
    fixture = Node("synthetic_native_firmware", enable_rosout=False)
    publishers = {
        name: fixture.create_publisher(typ, name, 10)
        for name, typ in [
            ("/ODOM", Odometry),
            ("/MOTION_INFO", MotionInfo),
            ("/HES_STATUS", StdMsgInt32),
            ("/LOCATION_STATUS", LocationStatus),
            ("/synthetic_cloud", PointCloud2),
            ("/native_start_b/map_context", String),
        ]
    }
    gaits, commands = [], []
    fixture.create_subscription(Gait, "/GAIT", lambda m: gaits.append(m.data.gait), 10)
    fixture.create_subscription(
        NavCmd,
        "/NAV_CMD",
        lambda m: commands.append((m.data.x_vel, m.data.y_vel, m.data.yaw_vel)),
        10,
    )
    root = Path(tempfile.mkdtemp(prefix="native-wire-211-"))
    config = {
        "map_id": "0914_fr_v3-20260914-142008",
        "cloud_topic": "/synthetic_cloud",
        "frames": {"map": "map", "odom_child": "base", "cloud": "base"},
        "body_z_offset": 0.42,
        "odom_child_from_base": np.eye(4).tolist(),
        "base_from_cloud": np.eye(4).tolist(),
        "healthy_location_code": 0,
        "verified": dict.fromkeys(VERIFICATIONS, True),
        "policy_call_acceptance": "passed_on_048",
    }
    # Synthetic admission values above are confined by the mandatory domain guard.
    (root / "config.json").write_text(json.dumps(config))
    points = [
        {"index": 0, "position": [0, 0, 0], "kind": kind},
        {"index": 1, "position": [1, 0, 0], "kind": kind},
    ]
    (root / "route.json").write_text(json.dumps({"waypoints": points}))
    args = SimpleNamespace(
        config=str(root / "config.json"),
        route=str(root / "route.json"),
        output=str(root / "run.jsonl"),
        probe_gait=None,
        velocity_probe=False,
        enable_motion=True,
    )
    app_test = os.environ.get("NATIVE_TEST_APP_CONTROL") == "1"
    if app_test:
        args.app_control_file = str(root / "control.json")
        (root / "control.json").write_text(json.dumps({"command": "wait", "expires_monotonic": time.monotonic() + 6}))
    log = io.StringIO()
    node = NativeTest(args, log)
    grid = [
        [x + d, y, -0.42]
        for x in np.linspace(-0.6, 1.2, 13)
        for y in np.linspace(-0.6, 0.6, 9)
        for d in [-0.01, 0, 0.01]
    ]
    cloud_points = grid + [
        [3 * math.cos(a), 3 * math.sin(a), -0.42]
        for a in np.linspace(-math.pi, math.pi, 72, endpoint=False) + 0.02
    ]

    def run_for(duration):
        until = time.monotonic() + duration
        next_publish = 0
        while time.monotonic() < until:
            if time.monotonic() >= next_publish:
                stamp = fixture.get_clock().now().to_msg()
                odom = Odometry()
                odom.header = Header(stamp=stamp, frame_id="map")
                odom.child_frame_id = "base"
                odom.pose.pose.position.z = 0.42
                odom.pose.pose.orientation.w = 1.0
                publishers["/ODOM"].publish(odom)
                motion = MotionInfo()
                motion.header.stamp = stamp
                motion.data.motion_state.state = 17
                motion.data.gait_state.gait = gaits[-1] if gaits else 0x1001
                publishers["/MOTION_INFO"].publish(motion)
                publishers["/HES_STATUS"].publish(StdMsgInt32(value=0))
                location = LocationStatus()
                location.data.total_status = 0
                publishers["/LOCATION_STATUS"].publish(location)
                cloud = create_cloud_xyz32(Header(stamp=stamp, frame_id="base"), cloud_points)
                publishers["/synthetic_cloud"].publish(cloud)
                publishers["/native_start_b/map_context"].publish(
                    String(
                        data=json.dumps(
                            {
                                "wall_time": time.time(),
                                "map_id": config["map_id"],
                                "session_id": "synthetic",
                                "global_mode": True,
                            }
                        )
                    )
                )
                next_publish = time.monotonic() + 0.05
            rclpy.spin_once(fixture, timeout_sec=0.001)
            rclpy.spin_once(node, timeout_sec=0.001)

    try:
        run_for(2)
        assert node.follower is not None
        assert node.nav_pub is None and not commands and not gaits
        if app_test:
            (root / "control.json").write_text(json.dumps({"command": "arm", "expires_monotonic": time.monotonic() + 6}))
        else:
            reply = node.arm(Trigger.Request(), Trigger.Response())
            assert reply.success, (reply.message, node.input_faults, node.sensor_info)
        run_for(2)
        assert node.gate.state == "active", node.gate.reason
        assert gaits == [GAITS[kind]], gaits
        assert any(c[0] > 0 for c in commands), commands
        assert all(0 <= c[0] <= 0.20 + 1e-6 for c in commands)
        if app_test:
            (root / "control.json").write_text(json.dumps({"command": "arm", "expires_monotonic": time.monotonic() - 1}))
            run_for(0.2)
        else:
            node.cancel(Trigger.Request(), Trigger.Response())
        mark = len(commands)
        run_for(0.5)
        assert commands[mark:] and all(c == (0, 0, 0) for c in commands[mark:])
        print(
            json.dumps(
                {
                    "test": "isolated_real_follower_router_wire",
                    "passed": True,
                    "domain": 211,
                    "app_lease_test": app_test,
                    "robot_actuated": False,
                    "gait_requests": gaits,
                    "native_velocity_messages": len(commands),
                }
            )
        )
    finally:
        node.publish_zero()
        node.follower.destroy_node()
        node.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
