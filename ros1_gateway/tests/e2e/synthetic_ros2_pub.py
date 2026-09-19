#!/usr/bin/env python3
"""Publish S10-shaped synthetic ROS 2 sensor data and log exactly what was sent.

Only for isolated tests (docker --network none, private ROS_DOMAIN_ID). Mirrors
the real 48 interface: /LIDAR/POINTS point_step 26 with ring(uint16@16) and
timestamp(float64@18) in lidar_link, /IMU with an empty frame_id at 200 Hz,
/ODOM frame map with an empty child_frame_id at 10 Hz.
"""
import argparse
import hashlib
import json
import os
import random
import struct
import time

import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField


def stamp(ns):
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def cloud(ns, rng):
    width = rng.randint(48000, 101000)
    msg = PointCloud2()
    msg.header.stamp = stamp(ns)
    msg.header.frame_id = 'lidar_link'
    msg.height, msg.width = 1, width
    msg.fields = [PointField(name='x', offset=0, datatype=7, count=1),
                  PointField(name='y', offset=4, datatype=7, count=1),
                  PointField(name='z', offset=8, datatype=7, count=1),
                  PointField(name='intensity', offset=12, datatype=7, count=1),
                  PointField(name='ring', offset=16, datatype=4, count=1),
                  PointField(name='timestamp', offset=18, datatype=8, count=1)]
    msg.is_bigendian = False
    msg.point_step = 26
    msg.row_step = 26 * width
    data = bytearray(os.urandom(26 * width))
    base = ns / 1e9
    for i in (0, width // 2, width - 1):  # plausible values at the sampled points
        struct.pack_into('<ffffHd', data, i * 26, 1.5, -2.25, 0.125, 17.0, i % 192, base + i * 1e-6)
    msg.data = bytes(data)
    msg.is_dense = True
    return msg, dict(width=width, height=1, point_step=26, row_step=26 * width, is_dense=True,
                     is_bigendian=False, data_sha256=hashlib.sha256(msg.data).hexdigest(),
                     fields=[[f.name, f.offset, f.datatype, f.count] for f in msg.fields])


def imu(ns, rng):
    msg = Imu()
    msg.header.stamp = stamp(ns)
    msg.header.frame_id = ''
    vals = [rng.uniform(-1, 1) for _ in range(37)]
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = vals[0:4]
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = vals[4:7]
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = vals[7:10]
    msg.orientation_covariance = vals[10:19]
    msg.angular_velocity_covariance = vals[19:28]
    msg.linear_acceleration_covariance = vals[28:37]
    return msg, dict(values=vals)


def odom(ns, rng):
    msg = Odometry()
    msg.header.stamp = stamp(ns)
    msg.header.frame_id = 'map'
    msg.child_frame_id = ''
    v = [rng.uniform(-5, 5) for _ in range(13)]
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    p.x, p.y, p.z, q.x, q.y, q.z, q.w = v[0:7]
    t = msg.twist.twist
    t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z = v[7:13]
    pc = [rng.uniform(0, 1) for _ in range(36)]
    tc = [rng.uniform(0, 1) for _ in range(36)]
    msg.pose.covariance, msg.twist.covariance = pc, tc
    return msg, dict(values=v, pose_cov=pc, twist_cov=tc, child_frame_id='')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=20)
    ap.add_argument('--log', required=True)
    ap.add_argument('--start-ns', type=int, default=1789635885950001240)  # same epoch region as the real bag
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('synthetic_s10_sensors')
    be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
    rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
    pubs = {'/LIDAR/POINTS': node.create_publisher(PointCloud2, '/LIDAR/POINTS', be),
            '/IMU': node.create_publisher(Imu, '/IMU', be),
            '/ODOM': node.create_publisher(Odometry, '/ODOM', rel)}
    rng = random.Random(48)
    time.sleep(3)  # discovery
    t0 = time.monotonic()
    tick = 0  # 5 ms ticks
    with open(args.log, 'w') as log:
        while time.monotonic() - t0 < args.seconds:
            ns = args.start_ns + tick * 5_000_000
            items = [('/IMU', *imu(ns, rng))]
            if tick % 20 == 0:
                items.append(('/LIDAR/POINTS', *cloud(ns, rng)))
            if tick % 20 == 10:
                items.append(('/ODOM', *odom(ns, rng)))
            for topic, msg, rec in items:
                pubs[topic].publish(msg)
                log.write(json.dumps(dict(topic=topic, stamp_ns=ns, frame_id=msg.header.frame_id, **rec)) + '\n')
            tick += 1
            delay = t0 + tick * 0.005 - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
