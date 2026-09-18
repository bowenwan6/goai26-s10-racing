"""Phone UI for the installed vendor SLAM. No motion publishers; no new SLAM algorithm."""
import os
import json
from contextlib import redirect_stdout
import math
from pathlib import Path
import re
import struct
import subprocess
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
SLAM = Path('/opt/robot/share/slam')
# Reuse vendor CLI modules after the system ROS/Numpy packages.
sys.path.append(str(SLAM/'lib/venv/lib/python3.12/site-packages'))
MAPS = Path('/var/opt/robot/data/maps')
latest, errors = {}, []
data_lock = threading.Lock()
stopping = threading.Event()


def measurement_time(stamp, received, wall_time, previous=None):
    age = wall_time-stamp
    row = dict(stamp=stamp, stamp_age_s=round(age, 4))
    if not math.isfinite(age) or not -.25 <= age <= 1:
        row['error'] = f'测量时间与定位板相差 {age:.3f} 秒，请先完成授时并重新检查传感器'
    elif previous and abs((stamp-previous['stamp'])-(received-previous['received'])) > .5:
        row['error'] = '检测到测量时间跳变；本次建图需要检查，请停稳后保存并重新开始'
    return row


def cloud_points(msg, limit=900):
    """Sample organized/unorganized PointCloud2 without copying the full cloud."""
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields and fields[k].datatype in (7, 8) for k in ('x', 'y', 'z')):
        raise ValueError('Point cloud requires float x/y/z')
    readers = [struct.Struct(('>' if msg.is_bigendian else '<') +
                             ('f' if fields[k].datatype == 7 else 'd')) for k in ('x', 'y', 'z')]
    offsets = [fields[k].offset for k in ('x', 'y', 'z')]
    if any(o < 0 or o+r.size > msg.point_step for o, r in zip(offsets, readers)):
        raise ValueError('Invalid point field offsets')
    count = msg.width * msg.height
    if not count or msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.row_step * msg.height:
        raise ValueError('Invalid point cloud buffer')
    data = memoryview(msg.data)
    points = []
    for i in range(0, count, max(1, math.ceil(count / limit))):
        base = (i // msg.width) * msg.row_step + (i % msg.width) * msg.point_step
        p = [r.unpack_from(data, base+o)[0] for o, r in zip(offsets, readers)]
        if all(math.isfinite(v) for v in p) and any(abs(v) > .001 for v in p):
            points.append([round(v, 3) for v in p])
    return points


def sense():
    node = None
    try:
        # Reuse the board's ROS/Numpy packages without modifying the vendor venv.
        if '/usr/lib/python3/dist-packages' not in sys.path:
            sys.path.append('/usr/lib/python3/dist-packages')
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2, Imu
        from nav_msgs.msg import Odometry
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('s10_phone_mapping_preview', enable_rosout=False,
                                start_parameter_services=False)

        def receive(key, serialized, kind):
            now = time.monotonic()
            with data_lock:
                if key in latest and now-latest[key]['received'] < 1:
                    return
            # Skip before decoding large clouds or 200 Hz IMU messages.
            msg = deserialize_message(serialized, kind)
            row = dict(received=now, frame=msg.header.frame_id,
                       **measurement_time(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                                          now, time.time(), latest.get(key)))
            if isinstance(msg, PointCloud2):
                try:
                    row['points'] = cloud_points(msg)
                except ValueError as exc:
                    row['error'] = str(exc)
                    row['points'] = []
            elif isinstance(msg, Odometry):
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                if not all(math.isfinite(v) for v in values):
                    return
                row.update(position=[p.x, p.y], yaw=math.atan2(2*(q.w*q.z+q.x*q.y),
                                                             1-2*(q.y*q.y+q.z*q.z)))
                row['xyz'] = [p.x, p.y, p.z]
            with data_lock:
                latest[key] = row

        for topic, kind, key in (('/LIDAR/POINTS', PointCloud2, 'raw'),
                                 ('/SLAM_ALIGNED_POINTS', PointCloud2, 'map'),
                                 ('/SLAM_ODOM', Odometry, 'pose'), ('/IMU', Imu, 'imu'),
                                 ('/ODOM', Odometry, 'localization_pose')):
            node.create_subscription(kind, topic, lambda msg, k=key, t=kind: receive(k, msg, t),
                                     qos_profile_sensor_data, raw=True)
        while not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except Exception as exc:
        errors.append(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()


def run(argv, timeout=15):
    env = os.environ.copy()
    if argv[0] == 'drsec':
        env['LD_LIBRARY_PATH'] = '/usr/local/lib:/usr/local/lib/aarch64-linux-gnu:'+str(SLAM/'lib')+':'+env.get('LD_LIBRARY_PATH', '')
    result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout, env=env)
    if result.returncode:
        raise RuntimeError((result.stderr or result.stdout).strip() or f'Command failed: {argv[0]}')
    return result.stdout


def service(name='mapping'):
    text = run(['systemctl', 'show', name+'.service', '-p',
                'ActiveState,InvocationID,MainPID,ExecMainStartTimestampMonotonic'])
    return dict(line.split('=', 1) for line in text.splitlines() if '=' in line)


def parse_localization_status(text, now, started_at=0):
    """Read the last monitor status, never treating old log output as live health."""
    pattern = (r'^\[([\d-]{10} [\d:]{8})\.(\d{3})\.(\d{3})\].*\[monitor\].*'
               r'上报状态=(\d+)\(([^)]+)\).*运行状态=(全局|局部)')
    for line in reversed(text.splitlines()):
        match = re.search(pattern, line)
        if match:
            stamp = time.mktime(time.strptime(match[1], '%Y-%m-%d %H:%M:%S'))
            stamp += int(match[2])/1000+int(match[3])/1000000
            age = now-stamp
            if stamp < started_at or not -.25 <= age <= 5:
                return dict(fresh=False, label='定位状态已过期')
            return dict(fresh=True, code=int(match[4]), label=match[5], mode=match[6],
                        stamp=stamp, age=round(age, 3))
    return dict(fresh=False, label='等待定位状态')


def localization_status():
    info = service('localization')
    result = dict(service=info.get('ActiveState'), invocation=info.get('InvocationID'),
                  active_map=None, status=dict(fresh=False, label='定位未运行'))
    try:
        path = (MAPS/'active').resolve(strict=True)
        if path.parent == MAPS.resolve() and path.is_dir():
            result['active_map'] = path.name
        if info.get('ActiveState') == 'active':
            now = time.time()
            started_at = now-(time.monotonic()-int(info['ExecMainStartTimestampMonotonic'])/1e6)
            result['started_at'] = started_at
            day = time.strftime('%Y_%m%d')
            log = Path('/var/opt/robot/log')/day/f'localization.{day}.log'
            # The vendor log is large; a bounded tail includes its frequent status lines.
            with log.open('rb') as f:
                f.seek(max(0, f.seek(0, 2)-65536))
                text = f.read().decode('utf-8', errors='replace')
            result['status'] = parse_localization_status(text, now, started_at)
    except (OSError, ValueError, KeyError) as exc:
        result['status'] = dict(fresh=False, label='无法读取定位状态', error=str(exc))
    return result


def map_directory():
    try:
        values = dict(line.split('=', 1) for line in Path('/tmp/mapping_path.env').read_text().splitlines() if '=' in line)
        path = Path(values['directory']).resolve(strict=True)
        if path.parent != MAPS.resolve() or not path.is_dir():
            raise ValueError('Unexpected mapping directory')
        return path
    except (OSError, KeyError, ValueError):
        return None


def saved_files(path):
    return path is not None and all((path/n).is_file() and (path/n).stat().st_size > 0
                                   for n in ('full_cloud.pcd', 'occ_grid.yaml', 'occ_grid.pgm'))


def save_mapping():
    """Keep the mapping process alive until save and postprocessing are confirmed."""
    info, path = service(), map_directory()
    invocation = info.get('InvocationID', '')
    owner = json.loads((HERE/'session.json').read_text())
    if invocation != owner['invocation'] or path is None or path.name != owner['map_name']:
        raise ValueError('当前建图不属于这个手机会话')
    if info.get('ActiveState') != 'active' or path is None or not re.fullmatch('[0-9a-f]{32}', invocation):
        raise ValueError('没有可确认归属的建图会话；未发送保存或停止命令')

    def completed():
        logs = run(['journalctl', '_SYSTEMD_INVOCATION_ID='+invocation, '-n', '200', '--no-pager'])
        return saved_files(path) and '地图后处理完成' in logs

    if not completed():
        run(['drsec', 'exec', str(SLAM/'bin/slam_command')], timeout=120)
        deadline = time.monotonic()+60
        while not completed():
            if time.monotonic() >= deadline:
                raise RuntimeError('尚未确认地图后处理完成，建图进程和文件已保留；可重试保存并查看日志')
            time.sleep(1)
    if service().get('InvocationID') != invocation:
        raise RuntimeError('建图会话已变化，拒绝停止其他会话')
    run(['systemctl', 'stop', 'mapping.service'], timeout=30)
    run(['systemctl', 'restart', 'localization.service'], timeout=30)
    (HERE/'session.json').unlink(missing_ok=True)
    return dict(message='地图已保存，定位服务已恢复', map_name=path.name)



def snapshot():
    from map_manager.services.mapping import MappingService
    info, path = service(), map_directory()
    try:
        owner = json.loads((HERE/'session.json').read_text())
    except (OSError, ValueError):
        owner = {}
    active = info.get('ActiveState') == 'active'
    return dict(active=active, service=info.get('ActiveState'),
                invocation=info.get('InvocationID'),
                owned=active and owner.get('invocation') == info.get('InvocationID'),
                status=MappingService._get_status() if active else dict(phase='idle'),
                map_name=path.name if path else None, saved=saved_files(path),
                localization=localization_status())


def dispatch(request):
    action = request.get('action')
    if action == 'status':
        return snapshot()
    if action == 'start':
        from map_manager.services.mapping import MappingService
        name, mode = request.get('name', ''), request.get('mode', '')
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,48}', name) or mode not in ('indoor', 'outdoor'):
            raise ValueError('无效地图名称或场景')
        if service().get('ActiveState') not in ('inactive', 'failed'):
            raise ValueError('已有建图服务，请先保存')
        existing = subprocess.run(['pgrep', '-x', 'slam_ddsnode'], capture_output=True)
        if existing.returncode != 1:
            raise ValueError('存在其他 SLAM 进程或无法检查，未启动新会话')
        # Check on 106 immediately before the vendor CLI stops localization.
        # Reuse the read-only diagnostic; wall clocks on the phone/AGX are irrelevant.
        try:
            run([sys.executable, str(Path.home()/'check_s10_slam.py'),
                 '--seconds', '3', '--require-ready'], timeout=15)
        except RuntimeError as exc:
            raise ValueError('雷达或 IMU 测量时间未通过检查，未启动建图；请先处理授时或输入异常') from exc
        MappingService.start_mapping(name, mode, activate=False, enable_rviz=False)
        info, path = service(), map_directory()
        if info.get('ActiveState') != 'active' or path is None:
            raise RuntimeError('厂商建图未启动，请检查日志')
        (HERE/'session.json').write_text(json.dumps(dict(invocation=info['InvocationID'], map_name=path.name)))
        return dict(message='建图已启动，等待实时地图')
    if action == 'save':
        return save_mapping()
    if action == 'maps':
        return dict(maps=[dict(name=p.name, saved=saved_files(p)) for p in sorted(MAPS.iterdir(), reverse=True)
                          if p.is_dir() and not p.is_symlink()])
    raise ValueError('不支持的操作')


def field_pending():
    """Legacy writes share the worker lock and respect its durable reservation."""
    from field_core import DEFAULT_ROOT
    import sqlite3
    path = DEFAULT_ROOT/'field.sqlite3'
    if not path.exists():
        return False  # Backward compatible before explicit field deployment.
    with sqlite3.connect(f'file:{path}?mode=ro', uri=True, timeout=3) as db:
        return db.execute("SELECT 1 FROM jobs WHERE state IN ('QUEUED','RUNNING')").fetchone() is not None


def main():
    import fcntl
    request = json.loads(sys.stdin.readline(4096))
    if request.get('action') == 'imu_diag_stream':
        from field_worker import rpc_call
        cursor = 0
        epoch = None
        try:
            while True:
                row = rpc_call(dict(action='imu_diag', request=dict(action='live', cursor=cursor)))
                if epoch is not None and epoch != row['epoch']:
                    row = rpc_call(dict(action='imu_diag', request=dict(action='live', cursor=0)))
                epoch, cursor = row['epoch'], row['seq']
                print('S10_RESULT '+json.dumps(row, ensure_ascii=False, allow_nan=False), flush=True)
                time.sleep(.2)
        except (BrokenPipeError, KeyboardInterrupt):
            pass
        return
    if request.get('action') == 'field':
        from field_worker import rpc_call
        try:
            response = dict(ok=True, result=rpc_call(request.get('request', {})))
        except Exception as exc:
            response = dict(ok=False, error=str(exc), code=getattr(exc, 'code', 'unavailable'))
        print('S10_RESULT '+json.dumps(response, ensure_ascii=False), flush=True)
        return
    if request.get('action') == 'heightmap_stream':
        from heightmap import stream
        stream()
        return
    if request.get('action') == 'stream':
        thread = threading.Thread(target=sense, daemon=True)
        thread.start()
        try:
            info = {}
            tick = 0
            while True:
                if tick % 3 == 0:
                    info = snapshot()
                tick += 1
                with data_lock:
                    now = time.monotonic()
                    data = {k: dict(v, age=round(now-v['received'], 2)) for k, v in latest.items()}
                print('S10_RESULT '+json.dumps(dict(**info, streams=data, board_time=time.time(),
                                                   error=errors[-1] if errors else None)), flush=True)
                time.sleep(1)
        except (BrokenPipeError, KeyboardInterrupt):
            stopping.set()
            thread.join(3)
        return
    # Same file lock for every command, following xwy's SSH backend pattern.
    try:
        with (HERE/'operation.lock').open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if request.get('action') in ('start', 'save') and field_pending():
                raise ValueError('已有持久现场任务，请在现场助手查询；未启动建图/保存')
            with redirect_stdout(sys.stderr):
                result = dispatch(request)
        response = dict(ok=True, result=result)
    except Exception as exc:
        response = dict(ok=False, error=str(exc))
    print('S10_RESULT '+json.dumps(response, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
