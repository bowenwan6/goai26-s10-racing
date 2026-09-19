#!/usr/bin/env python3
"""Convert a ROS 2 rosbag2 (MCAP or sqlite3, CDR) into a ROS 1 Noetic .bag.

Conversion happens at serialization level: CDR bytes are rewritten into ROS 1
wire format by rosbags' generated converters (PointCloud2.data is copied as one
block, never decoded point by point). Header stamps, frame ids, covariances and
the PointCloud2 field layout are carried over byte for byte. ROS 1 adds
Header.seq, which ROS 2 does not have; it is written as 0.

The ROS 1 bag message time is the ROS 2 bag receive time (unchanged). Nothing
is synthesized: no TF, no frame ids, no seq counters.

The source bag is only read. The output path must not exist and must not be
inside the source directory.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import platform
import sys
import time
from importlib.metadata import version
from pathlib import Path

from rosbags.rosbag1 import Writer
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from ruamel.yaml import YAML

# ROS 1 short type -> rosbags type name.
SUPPORTED = {
    'sensor_msgs/PointCloud2': 'sensor_msgs/msg/PointCloud2',
    'sensor_msgs/Imu': 'sensor_msgs/msg/Imu',
    'nav_msgs/Odometry': 'nav_msgs/msg/Odometry',
}
CALLERID = '/s10_mcap_to_ros1'


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def load_topic_map(path: Path) -> list[dict]:
    data = YAML(typ='safe').load(path.read_text(encoding='utf-8'))
    rows = data.get('topics') if isinstance(data, dict) else None
    if not rows:
        raise SystemExit(f'{path}: no "topics" list')
    seen_ros1 = set()
    for row in rows:
        for key in ('ros2_topic', 'ros1_topic', 'type'):
            if not isinstance(row.get(key), str) or not row[key]:
                raise SystemExit(f'{path}: every topic needs a non-empty {key}')
        if row['type'] not in SUPPORTED:
            raise SystemExit(f'{path}: unsupported type {row["type"]}; supported: {sorted(SUPPORTED)}')
        if not row['ros1_topic'].startswith('/'):
            raise SystemExit(f'{path}: ros1_topic must be absolute: {row["ros1_topic"]}')
        if row['ros1_topic'] in seen_ros1:
            raise SystemExit(f'{path}: duplicate ros1_topic {row["ros1_topic"]}')
        seen_ros1.add(row['ros1_topic'])
    return rows


def source_files(bag_dir: Path) -> list[Path]:
    return sorted(p for p in bag_dir.iterdir() if p.is_file() and p.suffix in ('.yaml', '.mcap', '.db3'))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', type=Path, help='ROS 2 bag directory (contains metadata.yaml)')
    parser.add_argument('output', type=Path, help='new ROS 1 .bag file (must not exist)')
    parser.add_argument('--topics', type=Path, default=Path(__file__).resolve().parent.parent / 'config/topics.yaml',
                        help='topic map YAML (default: config/topics.yaml)')
    parser.add_argument('--compression', choices=('none', 'lz4', 'bz2'), default='none',
                        help='ROS 1 chunk compression (default none: fastest, widest compatibility)')
    parser.add_argument('--source-manifest', type=Path,
                        help='optional MANIFEST.json with expected source SHA256 (keys like raw_bag/<file>)')
    parser.add_argument('--report', type=Path, help='conversion report JSON (default: <output>.conversion.json)')
    args = parser.parse_args()

    bag_dir = args.input.resolve()
    out = args.output.resolve()
    report_path = (args.report or out.with_suffix(out.suffix + '.conversion.json')).resolve()
    if not (bag_dir / 'metadata.yaml').is_file():
        raise SystemExit(f'{bag_dir}: metadata.yaml not found')
    if out.exists() or report_path.exists():
        raise SystemExit(f'refusing to overwrite existing output: {out if out.exists() else report_path}')
    if bag_dir == out.parent or bag_dir in out.parents:
        raise SystemExit('output must not be written inside the source bag directory')
    if out.suffix != '.bag':
        raise SystemExit('output must end with .bag')
    out.parent.mkdir(parents=True, exist_ok=True)
    topic_rows = load_topic_map(args.topics)

    started = time.time()
    print(f'hashing source files in {bag_dir} ...', flush=True)
    files_before = {p.name: dict(bytes=p.stat().st_size, sha256=sha256(p), mtime_ns=p.stat().st_mtime_ns)
                    for p in source_files(bag_dir)}
    manifest_check = None
    if args.source_manifest:
        expected = json.loads(args.source_manifest.read_text())['files']
        mismatches = {}
        for name, info in files_before.items():
            key = next((k for k in expected if k.endswith('/' + name) or k == name), None)
            if key is None or expected[key]['sha256'] != info['sha256']:
                mismatches[name] = dict(expected=expected[key]['sha256'] if key else None, actual=info['sha256'])
        manifest_check = dict(manifest=str(args.source_manifest.resolve()), all_match=not mismatches,
                              mismatches=mismatches)
        if mismatches:
            raise SystemExit(f'source SHA256 does not match manifest: {mismatches}')

    ros2_store = get_typestore(Stores.ROS2_JAZZY)
    ros1_store = get_typestore(Stores.ROS1_NOETIC)
    stats: dict[str, dict] = {}
    with Reader(bag_dir) as reader:
        by_topic = {c.topic: c for c in reader.connections}
        wanted = []
        for row in topic_rows:
            conn = by_topic.get(row['ros2_topic'])
            if conn is None:
                raise SystemExit(f'topic {row["ros2_topic"]} not in bag; bag has {sorted(by_topic)}')
            if conn.msgtype != SUPPORTED[row['type']]:
                raise SystemExit(f'{row["ros2_topic"]} is {conn.msgtype}, map says {row["type"]}')
            wanted.append((conn, row))
        skipped = sorted(set(by_topic) - {r['ros2_topic'] for r in topic_rows})
        writer = Writer(out)
        if args.compression != 'none':
            writer.set_compression(getattr(Writer.CompressionFormat, args.compression.upper()))
        writer.open()
        try:
            out_conn = {}
            for conn, row in wanted:
                msgtype = SUPPORTED[row['type']]
                out_conn[conn.id] = (writer.add_connection(row['ros1_topic'], msgtype, typestore=ros1_store,
                                                           callerid=CALLERID),
                                     msgtype, row)
                stats[row['ros2_topic']] = dict(ros1_topic=row['ros1_topic'], ros1_type=row['type'],
                                                ros1_md5=ros1_store.generate_msgdef(msgtype)[1],
                                                ros2_type=conn.msgtype, messages=0, ros2_bytes=0, ros1_bytes=0,
                                                first_bag_time_ns=None, last_bag_time_ns=None,
                                                bag_time_backwards=0)
            total = sum(c.msgcount for c, _ in wanted)
            done = 0
            last_print = time.time()
            for conn, timestamp, raw in reader.messages(connections=[c for c, _ in wanted]):
                wconn, msgtype, row = out_conn[conn.id]
                data = ros2_store.cdr_to_ros1(raw, msgtype)
                writer.write(wconn, timestamp, data)
                s = stats[row['ros2_topic']]
                if s['last_bag_time_ns'] is not None and timestamp < s['last_bag_time_ns']:
                    s['bag_time_backwards'] += 1
                s['first_bag_time_ns'] = timestamp if s['first_bag_time_ns'] is None else s['first_bag_time_ns']
                s['last_bag_time_ns'] = timestamp
                s['messages'] += 1
                s['ros2_bytes'] += len(raw)
                s['ros1_bytes'] += len(data)
                done += 1
                if time.time() - last_print > 10:
                    print(f'  {done}/{total} messages ({100*done/total:.1f}%)', flush=True)
                    last_print = time.time()
            ros2_meta_counts = {c.topic: c.msgcount for c, _ in wanted}
        finally:
            writer.close()

    files_after = {p.name: dict(bytes=p.stat().st_size, sha256=sha256(p), mtime_ns=p.stat().st_mtime_ns)
                   for p in source_files(bag_dir)}
    for topic, s in stats.items():
        s['ros2_metadata_count'] = ros2_meta_counts[topic]
        s['count_matches_ros2_metadata'] = s['messages'] == ros2_meta_counts[topic]
    report = dict(
        tool='s10 ros1_gateway tools/mcap_to_ros1_bag.py',
        created_at=dt.datetime.now(dt.timezone.utc).isoformat(),
        command=[sys.executable, *sys.argv],
        host=dict(platform=platform.platform(), python=platform.python_version()),
        versions=dict(rosbags=version('rosbags'), numpy=version('numpy')),
        source=dict(path=str(bag_dir), files=files_before, unchanged_after_conversion=files_before == files_after,
                    manifest_check=manifest_check, skipped_topics=skipped),
        topic_map=str(args.topics.resolve()),
        output=dict(path=str(out), bytes=out.stat().st_size, sha256=sha256(out), compression=args.compression,
                    callerid=CALLERID, header_seq='0 (ROS 2 has no seq)'),
        topics=stats,
        elapsed_s=round(time.time() - started, 1),
    )
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('output', 'topics', 'elapsed_s')}, indent=2))
    ok = report['source']['unchanged_after_conversion'] and all(
        s['count_matches_ros2_metadata'] and s['bag_time_backwards'] == 0 for s in stats.values())
    print('CONVERSION_OK' if ok else 'CONVERSION_CHECK_FAILED', f'report: {report_path}')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
