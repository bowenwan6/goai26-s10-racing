"""Read-only S10 SLAM inventory (106) or DDS sampling (AGX/106). No publishers."""
import argparse
import datetime as dt
import json
import math
import socket
import struct
import subprocess
import time
from pathlib import Path

SLAM_ROOT = Path('/opt/robot/share/slam')


def inventory():
    commands = {
        'packages': ['dpkg-query', '-W', '-f=${db:Status-Abbrev}\t${Package}\t${Version}\n',
                     'slam', 'slam-common-lib', '*gtsam*'],
        'clock': ['timedatectl', 'show', '-p', 'Timezone,NTPSynchronized,TimeUSec'],
        'services': ['systemctl', 'show', 'mapping.service', 'localization.service',
                     '-p', 'Id,LoadState,ActiveState,SubState,UnitFileState,FragmentPath'],
        'mapping_unit': ['systemctl', 'cat', 'mapping.service'],
        'mapping_cli_status': ['/usr/local/bin/drmap', '--format', 'json', 'mapping', 'status'],
        'processes': ['ps', '-eo', 'pid,comm'],
    }
    result = {}
    for name, argv in commands.items():
        try:
            run = subprocess.run(argv, capture_output=True, text=True, timeout=10)
            result[name] = dict(returncode=run.returncode, stdout=run.stdout, stderr=run.stderr)
        except (OSError, subprocess.TimeoutExpired) as exc:
            result[name] = dict(error=str(exc))
    paths = [SLAM_ROOT / name for name in (
        'README.md', 'change_log.md', 'bin/drmap', 'scripts/config.sh',
        'scripts/mapping.sh', 'scripts/mapping_stop.sh', 'scripts/start_dds.sh',
        'scripts/pre_program.sh', 'conf/params.yaml',
        'include/dr_lio/lio.h', 'include/dr_lio/use-ikfom.h', 'include/dr_lio/imu_processing.h')]
    paths += [Path('/var/opt/robot/conf/slam/params.yaml'), Path('/opt/robot/fastdds.xml')]
    paths += list((SLAM_ROOT / 'conf').glob('params.yaml.C*'))
    for name in ('cli/main.py', 'services/mapping.py'):
        paths += list(SLAM_ROOT.glob('lib/venv/lib/python*/site-packages/map_manager/' + name))
    result['files'] = {}
    for path in paths:
        try:
            result['files'][str(path)] = path.read_text()
        except OSError as exc:
            result['files'][str(path)] = dict(error=str(exc))
    result['slam_binary_present'] = (SLAM_ROOT / 'bin/slam_ddsnode').is_file()
    return result


def summary(samples, now):
    if not samples:
        return dict(messages=0, hz=None, stamp_age_s=None, fresh=False)
    elapsed = samples[-1]['received'] - samples[0]['received']
    last = samples[-1]
    ages = [r['stamp_age_s'] for r in samples]
    steps = [b['stamp']-a['stamp'] for a, b in zip(samples, samples[1:])]
    jumps = [abs(step-(b['received']-a['received']))
             for step, a, b in zip(steps, samples, samples[1:])]
    return dict(messages=len(samples), hz=(len(samples)-1)/elapsed if elapsed > 0 else None,
                stamp_age_s=last['stamp_age_s'],
                stamp_age_range_s=[min(ages), max(ages)],
                backwards=sum(step < -1e-6 for step in steps),
                max_clock_jump_s=max(jumps, default=0),
                fresh=now-last['received'] < 1 and min(ages) >= -.25 and max(ages) <= 1
                      and min(steps, default=0) >= -1e-6 and max(jumps, default=0) < .5,
                frame_id=last['frame_id'], child_frame_id=last.get('child_frame_id'),
                position_m=last.get('position_m'), orientation_xyzw=last.get('orientation_xyzw'),
                fields=last.get('fields'), point_stamp_samples=last.get('point_stamp_samples'),
                dds_header_offset_s=last.get('dds_header_offset_s'))


def sample(seconds):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2

    topics = {'/LIDAR/POINTS': PointCloud2, '/LIDAR/POINTS_MERGED': PointCloud2,
              '/IMU': Imu, '/ODOM': Odometry, '/LIO_ODOM': Odometry, '/SLAM_ODOM': Odometry}
    received = {topic: [] for topic in topics}
    rclpy.init()
    node = rclpy.create_node('s10_slam_read_only_check', enable_rosout=False,
                            start_parameter_services=False)

    def callback(topic, msg, info):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        row = dict(received=time.monotonic(), stamp=stamp, stamp_age_s=time.time()-stamp,
                   dds_header_offset_s=info['source_timestamp']/1e9-stamp,
                   frame_id=msg.header.frame_id)
        if isinstance(msg, Odometry):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            row.update(child_frame_id=msg.child_frame_id, position_m=[p.x, p.y, p.z],
                       orientation_xyzw=[q.x, q.y, q.z, q.w])
        elif isinstance(msg, PointCloud2):
            row['fields'] = [f.name for f in msg.fields]
            field = next((f for f in msg.fields if f.name == 'timestamp' and f.datatype in (7, 8)), None)
            if field and msg.width and msg.height:
                reader = struct.Struct(('>' if msg.is_bigendian else '<')+('d' if field.datatype == 8 else 'f'))
                count = msg.width*msg.height
                row['point_stamp_samples'] = [reader.unpack_from(msg.data,
                    (i//msg.width)*msg.row_step+(i%msg.width)*msg.point_step+field.offset)[0]
                    for i in (0, count//2, count-1)]
        received[topic].append(row)

    def receiver(topic):
        def receive(msg, info):
            callback(topic, msg, info)
        return receive

    try:
        for topic, kind in topics.items():
            node.create_subscription(kind, topic, receiver(topic),
                                     qos_profile_sensor_data)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        now = time.monotonic()
        result = {}
        for topic, rows in received.items():
            publishers = node.get_publishers_info_by_topic(topic)
            result[topic] = dict(summary(rows, now), publishers=[
                dict(node=p.node_name, type=p.topic_type, gid=bytes(p.endpoint_gid).hex(),
                     reliability=str(p.qos_profile.reliability)) for p in publishers])
        return result
    finally:
        node.destroy_node()
        rclpy.shutdown()


def check():
    assert not summary([], 10)['fresh']
    rows = [dict(received=9.8, stamp=100., stamp_age_s=.01, frame_id='odom'),
            dict(received=9.9, stamp=100.1, stamp_age_s=.02, frame_id='odom')]
    result = summary(rows, 10)
    assert result['fresh'] and math.isclose(result['hz'], 10)
    assert not summary(rows, 12)['fresh']
    rows[-1]['stamp_age_s'] = -5
    assert not summary(rows, 10)['fresh']
    rows[-1]['stamp_age_s'] = .02
    rows[0]['stamp'] -= 15638400
    assert not summary(rows, 10)['fresh'], 'A recovered final stamp must not hide a clock jump'
    print('SLAM_CHECK_OK')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inventory', action='store_true', help='Read installed vendor files on 106')
    parser.add_argument('--seconds', type=float, default=6, help='DDS sample window, 1..60 seconds')
    parser.add_argument('--output', type=Path, help='New JSON file; existing files are not overwritten')
    parser.add_argument('--check', action='store_true', help='Offline self-check; no ROS needed')
    parser.add_argument('--require-ready', action='store_true', help='Exit 2 if lidar/IMU timestamps are not ready')
    args = parser.parse_args()
    if args.check:
        check()
    else:
        if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 60:
            parser.error('--seconds must be finite and between 1 and 60')
        result = dict(host=socket.gethostname(), collected_at=dt.datetime.now(dt.UTC).isoformat(),
                      data=inventory() if args.inventory else sample(args.seconds))
        text = json.dumps(result, ensure_ascii=False, indent=2)
        if args.output:
            with args.output.open('x', encoding='utf-8') as stream:
                stream.write(text + '\n')
        print(text)
        if args.require_ready and (args.inventory or any(not result['data'][t]['fresh'] or
                result['data'][t]['messages'] < 2 for t in ('/LIDAR/POINTS', '/IMU'))):
            raise SystemExit(2)
