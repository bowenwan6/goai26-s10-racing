#!/usr/bin/env python3
"""ROS 2 side digests (official rclpy/rosbag2_py decoding), one JSON line per message.

  bag : read a rosbag2 directory (MCAP/sqlite3) read-only
        dump_ros2_digest.py bag <bag_dir> <out.jsonl>
  live: subscribe (best effort, read-only) for N seconds, e.g. on 106 during a
        live acceptance run; never publishes
        dump_ros2_digest.py live <out.jsonl> --seconds 60
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import msg_digest  # noqa: E402

TOPICS = {'/LIDAR/POINTS': 'sensor_msgs/msg/PointCloud2', '/IMU': 'sensor_msgs/msg/Imu',
          '/ODOM': 'nav_msgs/msg/Odometry'}


def from_bag(bag_dir, out):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    storage = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(bag_dir)) else 'sqlite3'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {t: get_message(types[t]) for t in TOPICS if t in types}
    n = 0
    with open(out, 'x') as f:
        while reader.has_next():
            topic, raw, t_ns = reader.read_next()
            if topic not in classes:
                continue
            msg = deserialize_message(raw, classes[topic])
            f.write(json.dumps(msg_digest.digest(topic, types[topic], msg, bag_time_ns=t_ns)) + '\n')
            n += 1
            if n % 5000 == 0:
                print(f'{n} messages', flush=True)
    print(f'DIGEST_DONE {n} messages -> {out}')


def live(out, seconds):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    rclpy.init()
    node = rclpy.create_node('s10_digest_reader', enable_rosout=False, start_parameter_services=False)
    f = open(out, 'x')
    counts = {t: 0 for t in TOPICS}

    def make(topic, type_name):
        def cb(msg):
            f.write(json.dumps(msg_digest.digest(topic, type_name, msg, recv_time_ns=time.time_ns())) + '\n')
            counts[topic] += 1
        return cb
    for topic, type_name in TOPICS.items():
        node.create_subscription(get_message(type_name), topic, make(topic, type_name), qos_profile_sensor_data)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    f.close()
    node.destroy_node()
    rclpy.shutdown()
    print(f'DIGEST_DONE {counts} -> {out}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='mode', required=True)
    b = sub.add_parser('bag')
    b.add_argument('bag_dir')
    b.add_argument('out')
    lv = sub.add_parser('live')
    lv.add_argument('out')
    lv.add_argument('--seconds', type=float, default=60)
    args = ap.parse_args()
    if args.mode == 'bag':
        from_bag(args.bag_dir, args.out)
    else:
        live(args.out, args.seconds)


if __name__ == '__main__':
    main()
