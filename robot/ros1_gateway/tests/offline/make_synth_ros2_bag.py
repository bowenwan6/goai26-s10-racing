#!/usr/bin/env python3
"""Write a deterministic S10-shaped ROS 2 MCAP bag with the official rosbag2_py writer.

Same message shapes as tests/e2e/synthetic_ros2_pub.py (point_step 26 with
ring/timestamp, IMU with empty frame_id, ODOM map/''), no DDS involved.
  make_synth_ros2_bag.py <out_bag_dir> --seconds 10
"""
import argparse
import os
import random
import sys

import rosbag2_py
from rclpy.serialization import serialize_message

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'e2e'))
from synthetic_ros2_pub import cloud, imu, odom

TYPES = {'/LIDAR/POINTS': 'sensor_msgs/msg/PointCloud2', '/IMU': 'sensor_msgs/msg/Imu',
         '/ODOM': 'nav_msgs/msg/Odometry'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--seconds', type=float, default=10)
    ap.add_argument('--start-ns', type=int, default=1789635885950001240)
    args = ap.parse_args()
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.out, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for i, (topic, type_name) in enumerate(TYPES.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(id=i, name=topic, type=type_name, serialization_format='cdr'))
    rng = random.Random(1789)
    for tick in range(int(args.seconds * 200)):
        ns = args.start_ns + tick * 5_000_000
        items = [('/IMU', imu(ns, rng)[0])]
        if tick % 20 == 0:
            items.append(('/LIDAR/POINTS', cloud(ns, rng)[0]))
        if tick % 20 == 10:
            items.append(('/ODOM', odom(ns, rng)[0]))
        for topic, msg in items:
            # bag receive time: realistic transport delays after the header stamp
            delay = {'/IMU': 9_500_000, '/LIDAR/POINTS': 118_500_000, '/ODOM': 38_500_000}[topic]
            writer.write(topic, serialize_message(msg), ns + delay)
    del writer
    print(f'SYNTH_BAG_DONE {args.out}')


if __name__ == '__main__':
    main()
