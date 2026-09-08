"""Read-only ROS 2 gait recorder with a phone UI; never publishes robot commands."""
import argparse
import copy
import hashlib
import hmac
import json
import math
import os
from pathlib import Path
import re
import secrets
import shutil
import signal
import threading
import time
import zipfile
from collections import deque

from flask import Flask, jsonify, request, session, send_file
from werkzeug.security import check_password_hash

TOPICS = {
    '/IMU': '惯性状态', '/JOINTS_DATA': '关节反馈',
    '/JOINTS_DATA_10HZ': '厂商关节反馈备用通道（频率实测为准）',
    '/JOINTS_CMD': '外部关节控制命令（未证实为原厂动作反馈）', '/MOTION_INFO': '当前运动状态与步态反馈',
    '/GAIT': '步态切换命令（不是当前反馈）', '/cmd_vel': '速度指令（来源待确认）',
    '/STEER': '厂商轴指令', '/REAL_STEER': '厂商真实轴指令',
    '/HANDLE_STEER': '厂商手柄轴指令',
    '/SLAM_ODOM': '建图里程计', '/LOCATION_STATUS': '定位状态',
    '/rslidar_front/points': '前雷达', '/rslidar_rear/points': '后雷达',
    '/tf': '动态坐标变换', '/tf_static': '静态坐标变换',
}
REQUIRED = ('/IMU', '/JOINTS_DATA', '/JOINTS_CMD')
TERRAINS = {'basic', 'stairs', 'ledge', 'gravel', 'mixed', 'unlabeled'}
SID = re.compile(r'gait_\d{8}_\d{6}_[a-f0-9]{12}\Z')
GAITS = {0x1001: '基础（标准运动模式）', 0x1003: '楼梯（标准运动模式）',
         0x3002: '平地（敏捷运动模式）',
         0x3003: '楼梯（敏捷运动模式）'}
MOTION_STATES = {0: '空闲', 1: '站立', 2: '关节阻尼 / 软急停',
                 3: '开机阻尼', 4: '趴下', 17: 'RL 控制'}


def decode_motion_info(fields):
    """Resolve unique named numeric leaves; the guide's example repeats data names.

    Actual paths come from the installed ROS type. Ambiguous fields stay unknown.
    """
    found = {'gait': [], 'state': []}
    def walk(value, path=''):
        if isinstance(value, dict):
            for key, child in value.items():
                here = path+'.'+key if path else key
                if key in found and not isinstance(child, dict):
                    found[key].append((here, child))
                walk(child, here)
    walk(fields)
    result = {'gait_code': None, 'gait_label': '未知步态', 'gait_known': False,
              'state_code': None, 'state_label': '未知运动状态', 'field_paths': {}}
    for key in ('gait', 'state'):
        entries = found[key]
        if len(entries) == 1 and type(entries[0][1]) is int:
            path, value = entries[0]
            if (key == 'gait' and 0 <= value <= 0xffffffff) or (key == 'state' and -(2**31) <= value < 2**31):
                result[key+'_code'] = value
                result['field_paths'][key] = path
    code = result['gait_code']
    result['gait_hex'] = f'0x{code:04X}' if code is not None else None
    result['gait_known'] = code in GAITS
    result['gait_label'] = GAITS.get(code, '未映射步态' if code is not None else '未知步态')
    result['state_label'] = MOTION_STATES.get(result['state_code'], '未知运动状态')
    result['hint'] = ('0x1002 在指南状态协议表中定义为高台；ROS 表未列出，待实机确认'
                      if code == 0x1002 else '')
    return result


def atomic_json(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    temp.replace(path)


class Recorder:
    def __init__(self, root, demo=False):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.demo = demo
        self.lock = threading.RLock()
        self.stats = {}
        self.discovered_topics = {}
        self.active = None
        self.writer = None
        self.types = {}
        self.rosbag = None
        self.error = '' if demo else '尚未连接 ROS 2'
        self.quit = threading.Event()
        self.requests = {}
        self.static_messages = {}
        self.motion = None
        self.motion_seen = None
        for path in self.root.glob('gait_*/manifest.json'):
            item = json.loads(path.read_text(encoding='utf-8'))
            if item['status'] == 'recording':
                item.update(status='interrupted', error='后台退出，记录不完整；原始数据已保留')
                atomic_json(path, item)

    def start_worker(self):
        self.thread = threading.Thread(target=self._demo if self.demo else self._ros, daemon=True)
        self.thread.start()

    def _demo(self):
        while not self.quit.wait(.02):
            for topic in REQUIRED:
                self.ingest(topic, 'demo/Synthetic', b'', time.time_ns())
            self.observe_motion({'data': {'gait': 0x1001, 'state': 17}}, time.time_ns())
            self.tick()

    def _ros(self):
        node = None
        try:
            import rclpy
            import rosbag2_py
            from rclpy.serialization import serialize_message
            from rclpy.signals import SignalHandlerOptions
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from rosidl_runtime_py.utilities import get_message
            from rosidl_runtime_py.convert import message_to_ordereddict
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node('s10_gait_readonly_capture')
            self.rosbag = rosbag2_py
            self.error = ''
            subscriptions = {}
            discovered = 0
            while not self.quit.is_set():
                if time.monotonic() - discovered > 2:
                    discovered = time.monotonic()
                    for topic, types in node.get_topic_names_and_types():
                        if topic not in TOPICS or len(types) != 1:
                            continue
                        publishers = node.get_publishers_info_by_topic(topic)
                        with self.lock:
                            self.discovered_topics[topic] = {'type': types[0], 'publisher_count': len(publishers)}
                        if topic in subscriptions or not publishers:
                            continue
                        try:
                            cls = get_message(types[0])
                            reliability = (ReliabilityPolicy.RELIABLE if all(
                                p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in publishers)
                                else ReliabilityPolicy.BEST_EFFORT)
                            qos = QoSProfile(depth=100, reliability=reliability,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL if topic == '/tf_static' else DurabilityPolicy.VOLATILE)
                            def callback(msg, topic=topic, kind=types[0]):
                                header = getattr(msg, 'header', None)
                                stamp = getattr(header, 'stamp', None) or getattr(header, 'timestamp', None)
                                source = (stamp.sec * 10**9 + getattr(stamp, 'nanosec', getattr(stamp, 'nsec', 0))) if stamp else None
                                self.ingest(topic, kind, serialize_message(msg), source,
                                            node.get_clock().now().nanoseconds)
                                if topic == '/MOTION_INFO':
                                    self.observe_motion(message_to_ordereddict(msg), source)
                            subscriptions[topic] = node.create_subscription(cls, topic, callback, qos)
                        except (ImportError, AttributeError, ModuleNotFoundError) as exc:
                            self.error = f'{topic} 消息类型不可用: {exc}'
                rclpy.spin_once(node, timeout_sec=.1)
                self.tick()
        except Exception as exc:
            with self.lock:
                self.error = f'ROS 采集不可用: {exc}'
                if self.active:
                    self._stop('failed', self.error)
        finally:
            if node:
                node.destroy_node()

    def observe_motion(self, fields, source_ns=None):
        with self.lock:
            decoded = decode_motion_info(fields)
            now = time.monotonic()
            changed = (self.motion is None or self.motion_seen is None or now-self.motion_seen > .5 or
                       any(decoded[key] != self.motion[key] for key in ('gait_code', 'state_code', 'field_paths')))
            self.motion = {**decoded, 'source_topic': '/MOTION_INFO', 'source_stamp_ns': source_ns,
                           'receive_wall_ns': time.time_ns(), 'synthetic': self.demo}
            self.motion_seen = now
            if changed and self.active:
                self.active['motion_feedback_events'].append({**self.motion,
                    'elapsed_s': now-self.started_mono, 'event': 'feedback_update'})
                atomic_json(self.root/self.active['id']/'manifest.json', self.active)

    def motion_snapshot(self):
        age = time.monotonic()-self.motion_seen if self.motion_seen is not None else None
        fresh = age is not None and age <= .5
        last = copy.deepcopy(self.motion)
        return {'fresh': fresh, 'age_s': age, 'last_feedback': last,
                'current': last if fresh else None}

    def ingest(self, topic, kind, raw, source_ns=None, receive_ns=None):
        with self.lock:
            now = time.monotonic()
            stat = self.stats.setdefault(topic, {'count': 0, 'times': deque(maxlen=200), 'type': kind})
            stat['count'] += 1
            stat['times'].append(now)
            stat['type'] = kind
            if topic == '/tf_static':
                self.static_messages[hashlib.sha256(raw).hexdigest()] = (topic, kind, raw, source_ns, receive_ns)
            if not self.active:
                return
            try:
                if not self.demo:
                    if topic not in self.types:
                        meta = self.rosbag.TopicMetadata(id=len(self.types)+1, name=topic,
                            type=kind, serialization_format='cdr')
                        self.writer.create_topic(meta)
                        self.types[topic] = kind
                    self.writer.write(topic, raw, receive_ns if receive_ns is not None else time.time_ns())
                item = self.active['topics'].setdefault(topic, {'count': 0, 'type': kind,
                    'source_stamp_missing': 0, 'source_nonincreasing': 0, 'max_receive_gap_s': 0})
                item['count'] += 1
                if 'last_receive_monotonic' in item:
                    item['max_receive_gap_s'] = max(item['max_receive_gap_s'], now-item['last_receive_monotonic'])
                item['last_receive_monotonic'] = now
                if source_ns is None or source_ns <= 0:
                    item['source_stamp_missing'] += 1
                else:
                    if source_ns <= item.get('last_source_ns', -1):
                        item['source_nonincreasing'] += 1
                    item['last_source_ns'] = source_ns
                if self.demo:
                    self.writer.write(json.dumps({'topic': topic, 'synthetic': True, 'receive_ns': time.time_ns()})+'\n')
            except Exception as exc:
                self._stop('failed', f'写入失败: {exc}')

    def tick(self):
        with self.lock:
            if self.active and (time.monotonic() >= self.deadline or shutil.disk_usage(self.root).free < 256*1024**2):
                self._stop('stopped', '达到录制时限或磁盘剩余空间下限')

    def health(self):
        now = time.monotonic()
        result = {}
        for topic, label in TOPICS.items():
            stat = self.stats.get(topic)
            times = stat['times'] if stat else []
            age = now-times[-1] if times else None
            result[topic] = {'label': label, 'fresh': age is not None and age < 2,
                'age_s': age, 'count': stat['count'] if stat else 0,
                'hz': (len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0,
                'type': stat['type'] if stat else self.discovered_topics.get(topic,{}).get('type'),
                'publisher_count': self.discovered_topics.get(topic,{}).get('publisher_count')}
        return result

    def snapshot(self):
        with self.lock:
            history = []
            for path in sorted(self.root.glob('gait_*/manifest.json'), reverse=True)[:100]:
                history.append(json.loads(path.read_text(encoding='utf-8')))
            return {'mode': 'demo' if self.demo else 'ros', 'error': self.error,
                'active': copy.deepcopy(self.active), 'health': self.health(), 'history': history,
                'motion_feedback': self.motion_snapshot(),
                'free_gb': round(shutil.disk_usage(self.root).free/1024**3, 2)}

    def action(self, payload):
        if not isinstance(payload, dict):
            raise ValueError('请求必须是 JSON 对象')
        key = payload.get('request_id', '')
        if not isinstance(key, str) or not re.fullmatch(r'[a-zA-Z0-9_-]{8,100}', key):
            raise ValueError('请求编号无效')
        signature = json.dumps(payload, sort_keys=True)
        with self.lock:
            if key in self.requests:
                previous, result = self.requests[key]
                if previous != signature:
                    raise ValueError('请求编号已用于其他操作')
                return result
            action = payload.get('action')
            if action == 'start':
                result = self._start(payload)
            elif action in ('mark', 'stop'):
                if not self.active or payload.get('session_id') != self.active['id']:
                    raise RuntimeError('录制会话已变化，请刷新状态')
                if action == 'mark':
                    label = payload.get('label')
                    if label not in ('obstacle_enter', 'obstacle_exit', 'slip', 'intervention', 'failure'):
                        raise ValueError('事件类型无效')
                    event = {'label': label, 'receive_wall_ns': time.time_ns(),
                        'elapsed_s': time.monotonic()-self.started_mono,
                        'time_basis': 'server receipt; not sensor-aligned'}
                    self.active['events'].append(event)
                    atomic_json(self.root/self.active['id']/'manifest.json', self.active)
                    result = event
                else:
                    outcome = payload.get('outcome', 'unlabeled')
                    if outcome not in ('success', 'failure', 'intervention', 'unlabeled'):
                        raise ValueError('结果类型无效')
                    self.active['outcome'] = outcome
                    result = self._stop('stopped', '')
            else:
                raise ValueError('操作无效')
            self.requests[key] = (signature, result)
            if len(self.requests)>512:
                del self.requests[next(iter(self.requests))]
            return result

    def _start(self, p):
        if self.active:
            raise RuntimeError('已有录制进行中')
        if not self.demo and self.rosbag is None:
            raise RuntimeError(self.error or 'ROS 2 尚未就绪')
        terrain = p.get('terrain')
        if terrain not in TERRAINS:
            raise ValueError('请选择基础、台阶、高台或碎石')
        duration = p.get('duration_s', 120)
        if type(duration) not in (int, float) or not math.isfinite(duration) or not 5 <= duration <= 1800:
            raise ValueError('录制时限应为 5–1800 秒')
        params = p.get('parameters', {})
        if not isinstance(params, dict) or set(params)-{'height_cm','depth_cm','slope_deg','gravel_cm'}:
            raise ValueError('地形参数无效')
        for value in params.values():
            if type(value) not in (int,float) or not math.isfinite(value) or not 0 <= value <= 1000:
                raise ValueError('地形参数须为 0–1000 的有限数值')
        notes = p.get('notes', '')
        teacher = p.get('teacher', '')
        if not isinstance(notes, str) or len(notes)>2000 or not isinstance(teacher,str) or len(teacher)>200:
            raise ValueError('备注或教师版本无效')
        missing = [name for name in REQUIRED if not self.health()[name]['fresh']]
        if missing and p.get('allow_partial') is not True:
            raise RuntimeError('关键数据未收到：'+', '.join(missing)+'；可显式选择仅采集诊断数据')
        if shutil.disk_usage(self.root).free < 512*1024**2:
            raise RuntimeError('磁盘剩余不足 512 MB')
        sid = 'gait_'+time.strftime('%Y%m%d_%H%M%S')+'_'+secrets.token_hex(6)
        folder = self.root/sid
        folder.mkdir()
        self.types = {}
        if self.demo:
            self.writer = (folder/'synthetic.jsonl').open('w', encoding='utf-8')
        else:
            self.writer = self.rosbag.SequentialWriter()
            self.writer.open(self.rosbag.StorageOptions(uri=str(folder/'bag'), storage_id='sqlite3'),
                self.rosbag.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
        self.started_mono = time.monotonic()
        self.deadline = self.started_mono+duration
        self.active = {'schema_version': 1, 'id': sid, 'terrain': terrain, 'parameters': params,
            'teacher': teacher, 'notes': notes, 'duration_limit_s': duration,
            'started_wall_ns': time.time_ns(), 'status': 'recording', 'outcome': 'unlabeled',
            'mode': 'demo' if self.demo else 'ros', 'missing_at_start': missing,
            'topics': {}, 'events': [], 'training_ready': False,
            'motion_feedback_at_start': self.motion_snapshot(), 'motion_feedback_events': [],
            'qualification': '演示数据，禁止用于训练' if self.demo else '原始记录；动作来源、单位、关节顺序、同步与闭环效果待验收'}
        atomic_json(folder/'manifest.json', self.active)
        for message in list(self.static_messages.values()):
            self.ingest(*message)
        return {'session_id': sid}

    def _stop(self, status, reason):
        record = self.active
        if self.demo and self.writer:
            self.writer.close()
        self.writer = None  # rosbag2 closes and writes metadata when released.
        self.active = None
        record.update(status=status, error=reason, stopped_wall_ns=time.time_ns(),
            duration_s=time.monotonic()-self.started_mono)
        record['missing_topics'] = [t for t in REQUIRED if not record['topics'].get(t, {}).get('count')]
        folder = self.root/record['id']
        atomic_json(folder/'manifest.json', record)
        return {'session_id': record['id'], 'status': status}

    def archive(self, sid):
        if not SID.fullmatch(sid):
            raise ValueError('会话编号无效')
        with self.lock:
            folder = self.root/sid
            if not folder.is_dir():
                raise ValueError('会话不存在')
            if self.active and self.active['id'] == sid:
                raise RuntimeError('请先停止录制再下载')
            if self.active:
                raise RuntimeError('请先结束当前录制，再打包下载，避免影响采集')
            dest = self.root/(sid+'.zip')
            if not dest.exists():
                hashes = {}
                for path in sorted(folder.rglob('*')):
                    if path.is_file() and path.name != 'SHA256SUMS.json':
                        with path.open('rb') as stream:
                            hashes[path.relative_to(folder).as_posix()] = hashlib.file_digest(stream, 'sha256').hexdigest()
                atomic_json(folder/'SHA256SUMS.json', hashes)
                temp = dest.with_suffix('.zip.tmp')
                with zipfile.ZipFile(temp, 'w', compression=zipfile.ZIP_STORED, allowZip64=True) as output:
                    for path in folder.rglob('*'):
                        if path.is_file():
                            output.write(path, sid+'/'+path.relative_to(folder).as_posix())
                temp.replace(dest)
            return dest


def create_app(recorder, token=None, secret=None, password_hash=None):
    app = Flask(__name__, static_folder=str(Path(__file__).parent/'web'), static_url_path='/static')
    app.config.update(SECRET_KEY=secret or secrets.token_hex(32), MAX_CONTENT_LENGTH=16384,
        SESSION_COOKIE_HTTPONLY=True, SESSION_COOKIE_SAMESITE='Strict', SESSION_COOKIE_NAME='s10_gait_session')

    @app.before_request
    def auth():
        if request.method == 'POST':
            origin = request.headers.get('Origin')
            if origin and origin.rstrip('/') != request.host_url.rstrip('/'):
                return jsonify(error='来源校验失败'), 403
        if request.path.startswith('/api/') and request.path != '/api/login':
            if not session.get('authenticated'):
                return jsonify(error='请先登录'), 401
            if request.method == 'POST' and not hmac.compare_digest(request.headers.get('X-CSRF-Token', ''), session.get('csrf', '!')):
                return jsonify(error='页面凭证已失效，请重新登录'), 403

    @app.get('/')
    def index():
        return app.send_static_file('index.html')

    @app.post('/api/login')
    def login():
        data = request.get_json()
        supplied = data.get('token') if isinstance(data, dict) else None
        valid = (isinstance(supplied, str) and (check_password_hash(password_hash, supplied)
                 if password_hash else bool(token) and hmac.compare_digest(supplied.encode('utf-8'), token.encode('utf-8'))))
        if not valid:
            return jsonify(error='访问口令不正确'), 401
        session.clear()
        session.update(authenticated=True, csrf=secrets.token_hex(24))
        return jsonify(csrf=session['csrf'])

    @app.get('/api/state')
    def state():
        return jsonify(**recorder.snapshot(), csrf=session['csrf'])

    @app.post('/api/action')
    def action():
        return jsonify(recorder.action(request.get_json()))

    @app.get('/api/download/<sid>')
    def download(sid):
        return send_file(recorder.archive(sid), as_attachment=True)

    @app.errorhandler(ValueError)
    def invalid(exc):
        return jsonify(error=str(exc)), 400

    @app.errorhandler(RuntimeError)
    def conflict(exc):
        return jsonify(error=str(exc)), 409

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--demo', action='store_true', help='Synthetic data only; never training data')
    parser.add_argument('--output', type=Path, default=Path.home()/'s10_gait_data')
    parser.add_argument('--auth-config', type=Path, help='Reuse existing mapping password hash')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    # Process-lifetime exclusive lock prevents competing servers/recovery on one dataset.
    lockfile = (args.output/'.recorder.lock').open('a+b')
    if os.name == 'nt':
        import msvcrt
        lockfile.seek(0); lockfile.write(b'0'); lockfile.flush(); lockfile.seek(0)
        msvcrt.locking(lockfile.fileno(), msvcrt.LK_NBLCK, 1)
    else:
        import fcntl
        fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
    token_file = args.output/'.access-token'
    password_hash = None
    token = None
    if args.auth_config:
        password_hash = json.loads(args.auth_config.read_text())['password_hash']
        if not isinstance(password_hash, str) or not password_hash:
            raise ValueError('Missing mapping password hash')
    else:
        if not token_file.exists():
            token_file.write_text(secrets.token_urlsafe(24), encoding='utf-8')
            token_file.chmod(0o600)
        token = token_file.read_text(encoding='utf-8').strip()
    recorder = Recorder(args.output, args.demo)
    def terminate(signum, frame):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, terminate)
    recorder.start_worker()
    print(f'Local page: http://127.0.0.1:{args.port}; access token file: {token_file}', flush=True)
    from waitress import serve
    try:
        serve(create_app(recorder, token, password_hash=password_hash), host=args.host, port=args.port, threads=6)
    finally:
        recorder.quit.set()
        with recorder.lock:
            if recorder.active:
                recorder._stop('interrupted', '后台关闭')
        lockfile.close()


if __name__ == '__main__':
    main()
