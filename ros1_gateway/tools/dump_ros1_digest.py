#!/usr/bin/env python3
"""ROS 1 side digests from a .bag (official rosbag/genpy decoding), one JSON line per message.

ROS 1 topic names are mapped back to the ROS 2 names with the same topics YAML
the gateway/converter used, so both digest files share topic keys.

  dump_ros1_digest.py <file.bag> <out.jsonl> --topics config/topics.yaml
"""
import argparse
import json
import os
import sys

import rosbag
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import msg_digest


def _text(v):
    return v.decode() if isinstance(v, bytes) else v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--topics', required=True)
    args = ap.parse_args()
    mapping = {row['ros1_topic']: row['ros2_topic'] for row in yaml.safe_load(open(args.topics))['topics']}
    n = 0
    with rosbag.Bag(args.bag) as bag, open(args.out, 'x') as f:
        connections = [dict(topic=c.topic, datatype=c.datatype, md5sum=c.md5sum, callerid=_text(c.header.get('callerid')))
                       for c in bag._connections.values()]
        f.write(json.dumps(dict(meta='connections', connections=connections)) + '\n')
        for topic, msg, t in bag.read_messages(topics=list(mapping)):
            row = msg_digest.digest(mapping[topic], msg._type, msg, bag_time_ns=t.to_nsec())
            row['ros1_topic'] = topic
            f.write(json.dumps(row) + '\n')
            n += 1
    print(f'DIGEST_DONE {n} messages -> {args.out}')


if __name__ == '__main__':
    main()
