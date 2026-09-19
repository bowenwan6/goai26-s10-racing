#!/usr/bin/env python3
"""Compare a ROS 1 bag recorded behind the gateway with the synthetic publisher log.

Runs under ROS 1 (rosbag + genpy). Every ROS 1 message must match the ROS 2
message with the same header stamp exactly: frame ids, PointCloud2 layout and
SHA256 of the data blob, IMU/Odometry values and covariances bit for bit.
"""
import argparse
import hashlib
import json
import struct
import sys

import rosbag

TOPICS = {'/lidar_points': '/LIDAR/POINTS', '/imu/data': '/IMU', '/odom': '/ODOM'}
MD5 = {'sensor_msgs/PointCloud2': '1158d486dd51d683ce2f1be655c3c181',
       'sensor_msgs/Imu': '6a62c6daae103f4ff57a132d6f95cec2',
       'nav_msgs/Odometry': 'cd5e73d190d741a2f92e81eda573aca7'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('log')
    ap.add_argument('--min-coverage', type=float, default=0.95)
    args = ap.parse_args()
    sent = {}
    for line in open(args.log):
        r = json.loads(line)
        sent[(r['topic'], r['stamp_ns'])] = r
    problems = []
    got = {t: 0 for t in TOPICS.values()}
    last = {}
    with rosbag.Bag(args.bag) as bag:
        for conn in bag._connections.values():
            if conn.topic in TOPICS and MD5.get(conn.datatype) != conn.md5sum:
                problems.append(f'{conn.topic}: md5 {conn.md5sum} for {conn.datatype}')
        for topic, msg, _ in bag.read_messages(topics=list(TOPICS)):
            src = TOPICS[topic]
            ns = msg.header.stamp.secs * 1_000_000_000 + msg.header.stamp.nsecs
            if ns <= last.get(topic, -1):
                problems.append(f'{topic}: stamp not increasing at {ns}')
            last[topic] = ns
            ref = sent.get((src, ns))
            if ref is None:
                problems.append(f'{topic}: stamp {ns} was never published')
                continue
            got[src] += 1
            if msg.header.frame_id != ref['frame_id']:
                problems.append(f'{topic}@{ns}: frame_id {msg.header.frame_id!r} != {ref["frame_id"]!r}')
            if src == '/LIDAR/POINTS':
                fields = [[f.name, f.offset, f.datatype, f.count] for f in msg.fields]
                layout = dict(width=msg.width, height=msg.height, point_step=msg.point_step,
                              row_step=msg.row_step, is_dense=bool(msg.is_dense),
                              is_bigendian=bool(msg.is_bigendian), fields=fields,
                              data_sha256=hashlib.sha256(msg.data).hexdigest())
                for k, v in layout.items():
                    if ref[k] != v:
                        problems.append(f'{topic}@{ns}: {k} differs')
                i = ref['width'] // 2
                x, y, z, it, ring, ts = struct.unpack_from('<ffffHd', msg.data, i * 26)
                if ring != i % 192 or abs(ts - (ns / 1e9 + i * 1e-6)) > 1e-9:
                    problems.append(f'{topic}@{ns}: sampled point ring/timestamp wrong')
            elif src == '/IMU':
                v = ([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
                     + list(msg.orientation_covariance) + list(msg.angular_velocity_covariance)
                     + list(msg.linear_acceleration_covariance))
                if v != ref['values']:
                    problems.append(f'{topic}@{ns}: values differ')
            else:
                p, q, tw = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
                v = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, tw.linear.x, tw.linear.y, tw.linear.z,
                     tw.angular.x, tw.angular.y, tw.angular.z]
                if (v != ref['values'] or list(msg.pose.covariance) != ref['pose_cov']
                        or list(msg.twist.covariance) != ref['twist_cov']
                        or msg.child_frame_id != ref['child_frame_id']):
                    problems.append(f'{topic}@{ns}: pose/twist/covariance/child_frame_id differ')
    sent_counts = {}
    for (topic, _), _r in sent.items():
        sent_counts[topic] = sent_counts.get(topic, 0) + 1
    coverage = {t: got[t] / sent_counts.get(t, 1) for t in got}
    result = dict(received=got, sent=sent_counts, coverage=coverage, problems=problems[:50],
                  problem_count=len(problems))
    print(json.dumps(result, indent=2))
    ok = not problems and all(c >= args.min_coverage for c in coverage.values())
    print('E2E_OK' if ok else 'E2E_FAILED')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
