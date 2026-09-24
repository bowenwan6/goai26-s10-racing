#!/usr/bin/env python3
"""Compare ROS 2 and ROS 1 digest files (dump_ros2_digest.py / dump_ros1_digest.py).

  compare_digests.py ros2.jsonl ros1.jsonl --mode offline --report report.json
  compare_digests.py ros2.jsonl ros1.jsonl --mode live    --report report.json

Messages are paired by (topic, header stamp). Every digest field must be equal:
frame ids, PointCloud2 layout and SHA256 of the data blob, sampled point values,
IMU/Odometry values and covariances.
offline: counts must match exactly and ROS 1 bag time must equal ROS 2 bag time.
live   : coverage is measured inside the time window both recordings share.
"""
import argparse
import json
import statistics
import sys

MD5 = {'sensor_msgs/PointCloud2': '1158d486dd51d683ce2f1be655c3c181',
       'sensor_msgs/Imu': '6a62c6daae103f4ff57a132d6f95cec2',
       'nav_msgs/Odometry': 'cd5e73d190d741a2f92e81eda573aca7'}
IGNORE = {'bag_time_ns', 'recv_time_ns', 'ros1_topic'}


def load(path):
    rows, meta = {}, []
    for line in open(path):
        r = json.loads(line)
        if 'meta' in r:
            meta.append(r)
            continue
        rows.setdefault(r['topic'], []).append(r)
    return rows, meta


def pct(values, q):
    values = sorted(values)
    return values[min(len(values) - 1, int(q * (len(values) - 1)))] if values else None


def stream_stats(rows):
    stamps = [r['stamp_ns'] for r in rows]
    steps = [b - a for a, b in zip(stamps, stamps[1:])]
    span = (stamps[-1] - stamps[0]) / 1e9 if len(stamps) > 1 else 0
    return dict(count=len(rows), first_stamp_ns=stamps[0] if stamps else None,
                last_stamp_ns=stamps[-1] if stamps else None, span_s=span,
                rate_hz_from_stamps=(len(stamps) - 1) / span if span > 0 else None,
                backwards=sum(s < 0 for s in steps), duplicates=sum(s == 0 for s in steps),
                max_gap_s=max(steps) / 1e9 if steps else None,
                median_gap_s=statistics.median(steps) / 1e9 if steps else None,
                frames=sorted({r['frame_id'] for r in rows}))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ros2')
    ap.add_argument('ros1')
    ap.add_argument('--mode', choices=('offline', 'live'), required=True)
    ap.add_argument('--report', required=True)
    ap.add_argument('--min-coverage', type=float, default=0.99)
    args = ap.parse_args()
    r2, _ = load(args.ros2)
    r1, meta1 = load(args.ros1)
    report = dict(mode=args.mode, ros2=args.ros2, ros1=args.ros1, topics={}, connections=[], problems=[])
    for m in meta1:
        for c in m.get('connections', []):
            ok = MD5.get(c['datatype']) == c['md5sum']
            report['connections'].append(dict(c, md5_matches_noetic=ok))
            if c['datatype'] in MD5 and not ok:
                report['problems'].append(f"{c['topic']}: md5 {c['md5sum']} is not Noetic's for {c['datatype']}")
    for topic in sorted(set(r2) | set(r1)):
        a, b = r2.get(topic, []), r1.get(topic, [])
        by2 = {r['stamp_ns']: r for r in a}
        by1 = {r['stamp_ns']: r for r in b}
        t = dict(ros2=stream_stats(a), ros1=stream_stats(b))
        mismatch = []
        matched = 0
        bag_time_diff = 0
        for s, x in by1.items():
            y = by2.get(s)
            if y is None:
                continue
            matched += 1
            bad = sorted(k for k in set(x) | set(y) if k not in IGNORE and x.get(k) != y.get(k))
            if bad:
                mismatch.append(dict(stamp_ns=s, fields=bad))
            if args.mode == 'offline' and x.get('bag_time_ns') != y.get('bag_time_ns'):
                bag_time_diff += 1
        t['matched'] = matched
        t['field_mismatches'] = len(mismatch)
        t['mismatch_examples'] = mismatch[:10]
        t['in_ros1_not_ros2'] = len(set(by1) - set(by2))
        if args.mode == 'offline':
            t['in_ros2_not_ros1'] = len(set(by2) - set(by1))
            t['bag_time_differences'] = bag_time_diff
            ok = (len(a) == len(b) == matched and not mismatch and bag_time_diff == 0
                  and t['ros1']['backwards'] == t['ros2']['backwards'])
        else:
            lo = max(min(by1, default=0), min(by2, default=0))
            hi = min(max(by1, default=0), max(by2, default=0))
            window = [s for s in by2 if lo <= s <= hi]
            got = sum(1 for s in window if s in by1)
            t['shared_window_s'] = (hi - lo) / 1e9 if hi > lo else 0
            t['ros2_in_window'] = len(window)
            t['coverage'] = got / len(window) if window else 0.0
            arrive = [(r['bag_time_ns'] - r['stamp_ns']) / 1e9 for r in b if 'bag_time_ns' in r]
            t['ros1_arrival_minus_stamp_s'] = dict(min=min(arrive, default=None), median=pct(arrive, 0.5),
                                                   p95=pct(arrive, 0.95), max=max(arrive, default=None))
            ok = not mismatch and t['coverage'] >= args.min_coverage and t['ros1']['backwards'] == 0 \
                and t['in_ros1_not_ros2'] == 0
        t['ok'] = ok
        if not ok:
            report['problems'].append(f'{topic}: check failed')
        report['topics'][topic] = t
    report['ok'] = not report['problems']
    with open(args.report, 'w') as f:
        json.dump(report, f, indent=2)
    for topic, t in report['topics'].items():
        extra = (f"coverage {t['coverage']:.4f} in {t['shared_window_s']:.1f}s" if args.mode == 'live'
                 else f"missing {t['in_ros2_not_ros1']}, bag-time diffs {t['bag_time_differences']}")
        print(f"{topic:15} ros2 {t['ros2']['count']:6} ros1 {t['ros1']['count']:6} matched {t['matched']:6} "
              f"field mismatches {t['field_mismatches']}  {extra}  "
              f"ros1 rate {t['ros1']['rate_hz_from_stamps'] or 0:.3f} Hz backwards {t['ros1']['backwards']}  "
              f"{'OK' if t['ok'] else 'FAIL'}")
    print('AUDIT_OK' if report['ok'] else f"AUDIT_FAILED {report['problems']}")
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
