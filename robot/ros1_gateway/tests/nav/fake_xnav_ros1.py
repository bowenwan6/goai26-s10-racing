#!/usr/bin/env python3
"""Fake x_nav + fake s10_ros1_control for the ROS 1 nav node test (no robot).

Publishes an Odometry pose at 10 Hz integrated from /rl_nav/cmd_vel, a flat-ground PointCloud2
at 10 Hz (lidar frame, using the same extrinsics as nav.yaml), /s10_control/gait echoing
/rl_nav/gait_request after 1 s at a standstill, and /s10_control/state. Writes what it saw to
--log (JSON lines) so the test can check progress and speeds.
"""
import argparse
import json
import math
import sys
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


def cloud_msg(points, frame, stamp):
    m = PointCloud2()
    m.header.stamp, m.header.frame_id = stamp, frame
    m.height, m.width = 1, len(points)
    m.fields = [PointField("x", 0, PointField.FLOAT32, 1), PointField("y", 4, PointField.FLOAT32, 1),
                PointField("z", 8, PointField.FLOAT32, 1), PointField("intensity", 12, PointField.FLOAT32, 1)]
    m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 16, 16 * len(points), True
    arr = np.zeros((len(points), 4), np.float32)
    arr[:, :3] = points
    m.data = arr.tobytes()
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--x", type=float, default=0.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--z", type=float, default=0.27, help="body height above ground")
    ap.add_argument("--lidar-t", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    ap.add_argument("--lidar-pitch-deg", type=float, default=0.0)
    a = ap.parse_args(rospy.myargv(sys.argv)[1:])
    rospy.init_node("fake_xnav")
    st = dict(x=a.x, y=a.y, yaw=a.yaw, v=0.0, w=0.0, vy=0.0, cmd=(0.0, 0.0, 0.0), cmd_t=-1e9, gait="flat", pending=None, req_t=0.0,
              n_cmd=0, max_v=0.0)
    lock = threading.Lock()
    log = open(a.log, "a")

    def on_cmd(m):
        with lock:
            st["cmd"], st["cmd_t"], st["n_cmd"] = (m.linear.x, m.linear.y, m.angular.z), time.monotonic(), st["n_cmd"] + 1
            st["max_v"] = max(st["max_v"], abs(m.linear.x))

    def on_req(m):
        with lock:
            if m.data in ("flat", "stairs") and m.data != st["gait"] and st["pending"] != m.data:
                st["pending"], st["req_t"] = m.data, time.monotonic()

    rospy.Subscriber("/rl_nav/cmd_vel", Twist, on_cmd, queue_size=1)
    rospy.Subscriber("/rl_nav/gait_request", String, on_req, queue_size=1)
    odom_pub = rospy.Publisher("/base_link/odom", Odometry, queue_size=1)
    cloud_pub = rospy.Publisher("/LIDAR/POINTS", PointCloud2, queue_size=1)
    gait_pub = rospy.Publisher("/s10_control/gait", String, queue_size=1)
    state_pub = rospy.Publisher("/s10_control/state", String, queue_size=1)
    # flat ground sampled in the lidar frame: ground plane z = -(z_body + lidar z) in the body frame,
    # rotated back into the pitched lidar frame.
    gx, gy = np.meshgrid(np.arange(-1.5, 4.0, 0.06), np.arange(-2.0, 2.0, 0.06))
    body = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, -a.z)])
    body = body - np.array(a.lidar_t)
    p = math.radians(a.lidar_pitch_deg)
    r_pitch = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    lidar_pts = body @ r_pitch  # inverse rotation: lidar = R^T body
    rate = rospy.Rate(10)
    last = time.monotonic()
    while not rospy.is_shutdown():
        now = time.monotonic()
        dt, last = now - last, now
        with lock:
            vx, vy, wz = st["cmd"] if now - st["cmd_t"] < 0.5 else (0.0, 0.0, 0.0)
            if st["pending"]:
                vx, vy, wz = 0.0, 0.0, 0.0
                if now - st["req_t"] >= 1.0 and abs(st["v"]) < 0.02:
                    st["gait"], st["pending"] = st["pending"], None
                    log.write(json.dumps(dict(t=now, event="gait", gait=st["gait"])) + "\n")
            st["v"] += (vx - st["v"]) * min(1.0, 4.0 * dt)
            st["w"] += (wz - st["w"]) * min(1.0, 4.0 * dt)
            st["x"] += (st["v"] * math.cos(st["yaw"]) - vy * math.sin(st["yaw"])) * dt
            st["y"] += (st["v"] * math.sin(st["yaw"]) + vy * math.cos(st["yaw"])) * dt
            st["yaw"] = math.atan2(math.sin(st["yaw"] + st["w"] * dt), math.cos(st["yaw"] + st["w"] * dt))
            x, y, yaw, v, w = st["x"], st["y"], st["yaw"], st["v"], st["w"]
            gait = "switching" if st["pending"] else st["gait"]
            n_cmd, max_v = st["n_cmd"], st["max_v"]
        stamp = rospy.Time.now()
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "map", "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, a.z
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
        odom_pub.publish(o)
        cloud_pub.publish(cloud_msg(lidar_pts, "lidar_link", stamp))
        gait_pub.publish(String(gait))
        state_pub.publish(String(json.dumps(dict(fault="", latched_stop=False, gait=gait, cmd_source="rl_nav"))))
        log.write(json.dumps(dict(t=now, x=round(x, 3), y=round(y, 3), yaw=round(yaw, 3), v=round(v, 3), gait=gait, n_cmd=n_cmd, max_v=round(max_v, 3))) + "\n")
        log.flush()
        rate.sleep()


if __name__ == "__main__":
    main()
