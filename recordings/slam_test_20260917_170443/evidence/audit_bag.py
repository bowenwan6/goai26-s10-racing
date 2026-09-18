"""Read-only native ROS bag audit. Does not initialise ROS or replay messages."""
import collections
import datetime as dt
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024**2), b''):
            h.update(block)
    return h.hexdigest()


def summary(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return {'min': float(a.min()), 'median': float(np.median(a)),
            'p95': float(np.percentile(a, 95)), 'max': float(a.max())} if a.size else None


def iso(ns):
    return dt.datetime.fromtimestamp(ns / 1e9, dt.timezone(dt.timedelta(hours=8))).isoformat()


def main(root):
    paths = sorted(root.iterdir())
    before = {p.name: (p.stat().st_size, p.stat().st_mtime_ns) for p in paths if p.is_file()}
    info = rosbag2_py.Info().read_metadata(str(root), 'mcap')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(root), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {t: get_message(v) for t, v in types.items()}
    samples = collections.defaultdict(list)
    frames = collections.defaultdict(collections.Counter)
    schemas = collections.Counter()
    cloud_counts, point_ranges, point_offsets, point_spans = [], [], [], []
    bad_xyz = bad_point_times = bad_imu = 0
    rings = set()
    cloud_example = None
    imu_values, positions = [], []
    field_formats = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    while reader.has_next():
        topic, raw, received = reader.read_next()
        m = deserialize_message(raw, classes[topic])
        stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        samples[topic].append((stamp, received, len(raw)))
        frames[topic][m.header.frame_id] += 1
        if types[topic] == 'sensor_msgs/msg/Imu':
            a, w = m.linear_acceleration, m.angular_velocity
            v = [w.x, w.y, w.z, a.x, a.y, a.z]
            bad_imu += not np.isfinite(v).all()
            imu_values.append(v)
        elif types[topic] == 'nav_msgs/msg/Odometry':
            p = m.pose.pose.position
            positions.append([p.x, p.y, p.z])
        elif types[topic] == 'sensor_msgs/msg/PointCloud2':
            schema = [(f.name, f.offset, f.datatype, f.count) for f in m.fields]
            schemas[json.dumps(schema)] += 1
            count = m.height * m.width
            cloud_counts.append(count)
            if len(m.data) < m.row_step * m.height:
                raise ValueError('Truncated PointCloud2 data')
            columns = {}
            for f in m.fields:
                if f.name not in ('x', 'y', 'z', 'ring', 'timestamp'):
                    continue
                if f.count != 1:
                    raise ValueError('Unexpected vector field')
                dtype = np.dtype(('>' if m.is_bigendian else '<') + field_formats[f.datatype])
                columns[f.name] = np.ndarray((m.height, m.width), dtype=dtype,
                    buffer=m.data, offset=f.offset, strides=(m.row_step, m.point_step)).reshape(-1)
            if count:
                bad_xyz += int((~(np.isfinite(columns['x']) & np.isfinite(columns['y']) & np.isfinite(columns['z']))).sum())
                times = columns.get('timestamp')
                if times is not None:
                    bad_point_times += int((~np.isfinite(times)).sum())
                    valid = times[np.isfinite(times)]
                    lo, hi = float(valid.min()), float(valid.max())
                    point_ranges.append((lo, hi))
                    point_offsets.append(lo - stamp / 1e9)
                    point_spans.append(hi - lo)
                if 'ring' in columns:
                    rings.update(int(x) for x in np.unique(columns['ring']))
                if cloud_example is None:
                    ids = [0, count // 2, count - 1]
                    cloud_example = dict(header_stamp_ns=stamp, received_ns=received,
                        frame_id=m.header.frame_id, width=m.width, height=m.height,
                        point_step=m.point_step, row_step=m.row_step, is_bigendian=m.is_bigendian,
                        fields=schema, sample_points={k: v[ids].tolist() for k, v in columns.items()})
    del reader
    expected = {x.topic_metadata.name: x.message_count for x in info.topics_with_message_count}
    results = {}
    for topic, rows in samples.items():
        a = np.asarray(rows, dtype=np.int64)
        source_diffs = np.diff(a[:, 0]) / 1e9
        received_diffs = np.diff(a[:, 1]) / 1e9
        span = float((a[-1, 0] - a[0, 0]) / 1e9)
        results[topic] = dict(type=types[topic], count=len(rows), metadata_count=expected.get(topic),
            count_matches_metadata=len(rows) == expected.get(topic), frames=dict(frames[topic]),
            source_start=iso(int(a[0, 0])), source_end=iso(int(a[-1, 0])), source_span_s=span,
            source_hz=(len(rows) - 1) / span if span else None,
            source_duplicate_timestamps=int((source_diffs == 0).sum()),
            source_backward_timestamps=int((source_diffs < 0).sum()),
            source_interval_s=summary(source_diffs), receive_interval_s=summary(received_diffs),
            received_minus_source_s=summary((a[:, 1] - a[:, 0]) / 1e9),
            serialized_bytes=int(a[:, 2].sum()))
    imu = np.asarray(imu_values)
    pos = np.asarray(positions)
    imu_times = np.asarray(samples['/IMU'], dtype=np.int64)[:, 0] / 1e9
    pr = np.asarray(point_ranges)
    source_files = {p.name: dict(bytes=p.stat().st_size, sha256=digest(p)) for p in paths if p.is_file()}
    unchanged = all(before[p.name] == (p.stat().st_size, p.stat().st_mtime_ns) for p in paths if p.is_file())
    result = dict(audited_at=dt.datetime.now(dt.timezone.utc).isoformat(), source_path=str(root),
        all_messages_deserialized=True, message_count=sum(len(v) for v in samples.values()),
        metadata_message_count=info.message_count, topics=results, source_files=source_files,
        source_sizes_and_mtimes_unchanged=unchanged,
        lidar=dict(example=cloud_example, schemas={k: v for k, v in schemas.items()},
            total_points=sum(cloud_counts), points_per_scan=summary(cloud_counts),
            nonfinite_xyz_points=bad_xyz, nonfinite_point_timestamps=bad_point_times,
            observed_ring_values=sorted(rings), scan_span_s=summary(point_spans),
            first_point_minus_header_s=summary(point_offsets),
            scans_outside_imu_time_coverage=int(((pr[:, 0] < imu_times[0]) | (pr[:, 1] > imu_times[-1])).sum()),
            point_time_start=iso(int(pr[:, 0].min() * 1e9)), point_time_end=iso(int(pr[:, 1].max() * 1e9))),
        imu=dict(nonfinite_messages=int(bad_imu), gyro_norm=summary(np.linalg.norm(imu[:, :3], axis=1)),
            acceleration_norm=summary(np.linalg.norm(imu[:, 3:], axis=1)),
            first_30s_gyro_norm=summary(np.linalg.norm(imu[imu_times < imu_times[0] + 30, :3], axis=1))),
        odom=dict(first_xyz=pos[0].tolist(), last_xyz=pos[-1].tolist(),
            bbox_min=pos.min(axis=0).tolist(), bbox_max=pos.max(axis=0).tolist(),
            sampled_path_length_m=float(np.linalg.norm(np.diff(pos, axis=0), axis=1).sum()),
            note='Existing localization output; not independent ground truth.'),
        notes=['No rclpy.init, publishers, bag play, sensor changes or motion commands.',
            'Recorded driver output is not UDP packets; deskew and multi-LiDAR semantics require confirmation.',
            'Configuration snapshots collected after recording are not a frozen start-of-recording configuration.',
            'Continuous timestamps do not prove absence of every packet loss or correct calibration.'])
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main(Path(sys.argv[1]))
