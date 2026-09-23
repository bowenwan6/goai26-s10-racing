"""Durable field jobs and evidence gates. No ROS or motion interfaces here."""
import hashlib
import json
import math
import re
import sqlite3
import time
import uuid
from contextlib import contextmanager
from pathlib import Path

ACTIVE = ('QUEUED', 'RUNNING')
ACTIONS = {'selfcheck', 'load_map', 'localization_check', 'confirm_overlay',
           'record', 'waypoint', 'waypoint_revisit', 'finish'}
MAP_RE = re.compile(r'[A-Za-z0-9_-]{1,100}')
ID_RE = re.compile(r'[a-f0-9]{32}')
DEFAULT_ROOT = Path('/var/opt/robot/data/s10_field_assistant')


class FieldError(ValueError):
    def __init__(self, message, code='invalid', details=None):
        super().__init__(message)
        self.code = code
        self.details = details


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False)


def ident(value):
    if not isinstance(value, str) or not ID_RE.fullmatch(value):
        raise FieldError('无效记录编号')
    return value


def text(value, limit=80):
    if not isinstance(value, str) or not 1 <= len(value) <= limit or any(ord(c) < 32 for c in value):
        raise FieldError('名称或备注长度/字符无效')
    return value


def validate_request(request):
    if not isinstance(request, dict) or set(request) - {'action', 'key', 'session_id', 'params'}:
        raise FieldError('任务格式无效')
    action = request.get('action')
    if action not in ACTIONS:
        raise FieldError('不支持的现场操作')
    key = request.get('key')
    if not isinstance(key, str) or not re.fullmatch(r'[A-Za-z0-9_-]{16,100}', key):
        raise FieldError('需要有效幂等键')
    params = request.get('params', {})
    if not isinstance(params, dict):
        raise FieldError('参数必须为对象')
    allowed = {
        'selfcheck': {'target_map'}, 'load_map': {'stationary', 'remote_ready'},
        'localization_check': {'stationary'}, 'confirm_overlay': {'check_id', 'confirmed'},
        'record': {'kind', 'seconds', 'remote_ready'},
        'waypoint': {'name', 'floor', 'segment', 'stationary', 'draft', 'marker_note'}, 'finish': set(),
        'waypoint_revisit': {'waypoint_id', 'stationary', 'remote_ready', 'physical_confirmed', 'reference_note', 'ruler_offset_cm', 'alignment_mode'},
    }[action]
    if set(params) - allowed:
        raise FieldError('存在不支持的参数')
    session = request.get('session_id')
    if action == 'selfcheck' and session is None:
        target = params.get('target_map')
        if not isinstance(target, str) or not MAP_RE.fullmatch(target):
            raise FieldError('目标地图名称无效')
    else:
        ident(session)
    if action in ('load_map', 'localization_check', 'waypoint', 'waypoint_revisit') and params.get('stationary') is not True:
        raise FieldError('需要现场人员确认机器人已停稳')
    if action in ('load_map', 'record', 'waypoint_revisit') and params.get('remote_ready') is not True:
        raise FieldError('需要确认手柄接管/急停可用；网页不控制停车')
    if action == 'confirm_overlay':
        ident(params.get('check_id'))
        if params.get('confirmed') is not True:
            raise FieldError('需要人眼核对地标、方向和实时叠合')
    if action == 'record':
        if params.get('kind') not in ('stationary', 'straight', 'turn'):
            raise FieldError('请选择静止/直行/转向')
        seconds = params.get('seconds', 10)
        if type(seconds) is not int or not 10 <= seconds <= 30:
            raise FieldError('录制时长必须为 10–30 秒整数')
    if action == 'waypoint':
        text(params.get('name'))
        text(params.get('floor', '未填写'), 40)
        if params.get('segment', 'flat') != 'flat':
            raise FieldError('首版仅标记平地短段；楼梯不生成自动连线')
        if type(params.get('draft', True)) is not bool:
            raise FieldError('草稿标志必须为布尔值')
        if params.get('marker_note') is not None:
            text(params['marker_note'], 200)
    if action == 'waypoint_revisit':
        ident(params.get('waypoint_id'))
        if params.get('physical_confirmed') is not True:
            raise FieldError('请确认已到达所选实体标记附近并停稳；不要求位置或朝向完全一致')
        if params.get('reference_note') is not None:
            text(params['reference_note'], 200)
        if params.get('alignment_mode', 'nearby') not in ('nearby', 'careful'):
            raise FieldError('请选择附近快速对照或尽量对准标记')
        measured = params.get('ruler_offset_cm')
        if measured is not None and (type(measured) not in (int, float) or not math.isfinite(measured) or not 0 <= measured <= 1000):
            raise FieldError('尺量对点残差必须是0到1000厘米的有限数；未测量请留空')
    canonical(request)
    return action, key, session, params


def finite_list(value, n):
    return isinstance(value, (list, tuple)) and len(value) == n and all(
        type(v) in (int, float) and math.isfinite(v) for v in value)


def binding(snapshot):
    return {k: snapshot.get(k) for k in ('robot_id', 'boot_id', 'map_identity', 'invocation')}


def localization_start_reasons(snap, expected=None):
    """Read-only inspection can start with bad sensor/localization evidence."""
    reasons = []
    if expected is not None and binding(snap) != expected:
        reasons.append('地图内容、设备、开机会话或定位服务会话已变化')
    if not all(snap.get(k) for k in ('robot_id', 'boot_id', 'map_identity', 'invocation')):
        reasons.append('地图/定位会话身份证据缺失')
    if snap.get('mapping_active') is not False:
        reasons.append('建图正在运行或状态未知')
    if snap.get('localization_active') is not True:
        reasons.append('定位服务未运行')
    return reasons


def localization_reasons(snap, expected=None, require_global=True):
    reasons = localization_start_reasons(snap, expected)
    status = snap.get('status', {})
    if require_global:
        if status.get('fresh') is not True:
            reasons.append('厂商定位状态消息过期/未知；与位姿消息新鲜度是两项检查')
        if type(status.get('code')) is not int or status.get('code') != 0:
            reasons.append('厂商定位状态非正常：Code'+str(status.get('code', '未知'))+'（'+str(status.get('label', '未给出标签'))+'）；数据更新不代表定位成功')
        if status.get('mode') != '全局':
            reasons.append('尚未完成全局定位，当前模式：'+str(status.get('mode', '未知')))
    pose = snap.get('pose', {})
    if pose.get('frame') != 'map':
        reasons.append('位姿不在 map 帧')
    if not finite_list(pose.get('xyz'), 3) or not finite_list(pose.get('quaternion'), 4):
        reasons.append('位姿非有限数或不完整')
    elif not .99 <= sum(v*v for v in pose['quaternion']) <= 1.01:
        reasons.append('四元数未归一化')
    for k in ('age', 'stamp_age_s'):
        v = pose.get(k)
        if type(v) not in (int, float) or not math.isfinite(v) or not (-.05 <= v <= .5):
            reasons.append('位姿接收或源时间过期/异常')
            break
    if pose.get('error'):
        reasons.append('位姿源时间跳变或解码异常')
    stamp, started = pose.get('stamp'), snap.get('started_at')
    if (type(stamp) not in (float, int) or type(started) not in (float, int)
            or not math.isfinite(stamp) or not math.isfinite(started) or stamp < started):
        reasons.append('位姿不属于当前定位服务启动后的样本')
    return reasons


def pose_summary(samples, duration, expected, min_hz=2, require_global=True):
    """Only distinct advancing ROS source stamps count as observations."""
    reasons, unique, last_stamp, reference = [], [], None, None
    for snap in samples:
        for reason in localization_reasons(snap, expected, require_global=require_global):
            if reason not in reasons:
                reasons.append(reason)
        p = snap.get('pose', {})
        frames = (p.get('frame'), p.get('child_frame'))
        if reference is None:
            reference = frames
        elif frames != reference:
            reasons.append('采样窗口中的参考帧/子帧变化，不能混合平均')
        stamp = p.get('stamp')
        if not isinstance(stamp, (float, int)) or not math.isfinite(stamp):
            continue
        if last_stamp is not None and stamp < last_stamp:
            reasons.append('采样中位姿源时间倒退')
        if last_stamp is None or stamp > last_stamp:
            if finite_list(p.get('xyz'), 3) and finite_list(p.get('quaternion'), 4):
                unique.append(p)
            last_stamp = stamp
    if len(unique) < max(3, int(duration*min_hz)):
        reasons.append('独立新位姿样本不足')
    if len(unique) > 1:
        if unique[-1]['stamp']-unique[0]['stamp'] < duration*.8:
            reasons.append('有效源时间覆盖不足')
        if max(b['stamp']-a['stamp'] for a, b in zip(unique, unique[1:])) > .75:
            reasons.append('位姿源数据间断超过 0.75 秒')
    summary = dict(passed=False, reasons=list(dict.fromkeys(reasons)), samples=len(unique), duration_s=duration)
    if not unique:
        return summary
    mean = [sum(p['xyz'][i] for p in unique)/len(unique) for i in range(3)]
    spread = max(math.dist(p['xyz'], mean) for p in unique)
    q0 = unique[0]['quaternion']
    qs = [[v * (1 if sum(a*b for a, b in zip(p['quaternion'], q0)) >= 0 else -1)
           for v in p['quaternion']] for p in unique]
    q = [sum(p[i] for p in qs)/len(qs) for i in range(4)]
    norm = math.sqrt(sum(v*v for v in q))
    if norm < .9:
        summary['reasons'].append('姿态变化过大')
        q = q0
    else:
        q = [v/norm for v in q]
    angle = max(2*math.acos(min(1., abs(sum(a*b for a, b in zip(p, q))))) for p in qs)
    if spread > .10:
        summary['reasons'].append('静止位置离散超过 0.10 米（暂定数据门限，非精度证明）')
    if angle > math.radians(5):
        summary['reasons'].append('静止姿态离散超过 5 度')
    summary.update(xyz=mean, quaternion=q, yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),
                   1-2*(q[1]*q[1]+q[2]*q[2])), spread_m=spread, angle_rad=angle,
                   first_stamp=unique[0]['stamp'], last_stamp=unique[-1]['stamp'],
                   frame=unique[-1]['frame'], child_frame=unique[-1].get('child_frame', ''),
                   reference_verified=False)
    summary['passed'] = not summary['reasons']
    return summary


class Store:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.root.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.path = self.root/'field.sqlite3'
        with self.connect() as db:
            db.executescript('''
                CREATE TABLE IF NOT EXISTS sessions(id TEXT PRIMARY KEY, target TEXT NOT NULL,
                    created REAL NOT NULL, data TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS jobs(id TEXT PRIMARY KEY, key TEXT UNIQUE NOT NULL,
                    request TEXT NOT NULL, action TEXT NOT NULL, session_id TEXT NOT NULL,
                    state TEXT NOT NULL, stage TEXT NOT NULL, created REAL NOT NULL,
                    updated REAL NOT NULL, result TEXT, error TEXT);
                CREATE UNIQUE INDEX IF NOT EXISTS one_active ON jobs((1))
                    WHERE state IN ('QUEUED','RUNNING');
                CREATE TABLE IF NOT EXISTS events(id INTEGER PRIMARY KEY, job_id TEXT,
                    at REAL NOT NULL, stage TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS artifacts(id TEXT PRIMARY KEY, job_id TEXT NOT NULL,
                    name TEXT NOT NULL, relative_path TEXT NOT NULL, size INTEGER NOT NULL,
                    sha256 TEXT NOT NULL, fingerprint TEXT);
            ''')
            if 'fingerprint' not in {r[1] for r in db.execute('PRAGMA table_info(artifacts)')}:
                db.execute('ALTER TABLE artifacts ADD COLUMN fingerprint TEXT')

    @contextmanager
    def connect(self):
        db = sqlite3.connect(self.path, timeout=5)
        db.row_factory = sqlite3.Row
        db.execute('PRAGMA synchronous=FULL')
        db.execute('PRAGMA foreign_keys=ON')
        try:
            with db:
                yield db
        finally:
            db.close()

    def submit(self, request):
        action, key, session_id, params = validate_request(request)
        payload = canonical(request)
        now = time.time()
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            row = db.execute('SELECT * FROM jobs WHERE key=?', (key,)).fetchone()
            if row:
                if row['request'] != payload:
                    raise FieldError('同一幂等键不能用于不同请求', 'conflict')
                return self.decode_job(row)
            if db.execute("SELECT 1 FROM jobs WHERE state IN ('QUEUED','RUNNING')").fetchone():
                raise FieldError('已有现场任务，先查看其结果；不能重复启动', 'busy')
            if session_id is None:
                session_id = uuid.uuid4().hex
                db.execute('INSERT INTO sessions VALUES(?,?,?,?)', (session_id, params['target_map'], now, '{}'))
            else:
                row = db.execute('SELECT * FROM sessions WHERE id=?', (session_id,)).fetchone()
                if not row:
                    raise FieldError('现场会话不存在')
                if params.get('target_map', row['target']) != row['target']:
                    raise FieldError('不能更改既有会话目标地图；请新建会话')
            job_id = uuid.uuid4().hex
            db.execute('INSERT INTO jobs VALUES(?,?,?,?,?,?,?,?,?,?,?)',
                       (job_id, key, payload, action, session_id, 'QUEUED', '已持久登记，等待执行', now, now, None, None))
        return self.job(job_id)

    @staticmethod
    def decode_job(row):
        d = dict(row)
        d['request'] = json.loads(d['request'])
        d['result'] = json.loads(d['result']) if d['result'] else None
        return d

    def job(self, job_id):
        ident(job_id)
        with self.connect() as db:
            row = db.execute('SELECT * FROM jobs WHERE id=?', (job_id,)).fetchone()
            if not row:
                raise FieldError('任务不存在', 'not_found')
            result = self.decode_job(row)
            result['events'] = [dict(r) for r in db.execute('SELECT at,stage FROM events WHERE job_id=? ORDER BY id', (job_id,))]
            result['artifacts'] = [dict(r) for r in db.execute('SELECT id,name,size,sha256 FROM artifacts WHERE job_id=?', (job_id,))]
            return result

    def jobs(self, session_id=None, limit=100):
        if session_id:
            ident(session_id)
        if limit is not None and (type(limit) is not int or limit < 1):
            raise FieldError('无效任务查询上限')
        with self.connect() as db:
            rows = db.execute('SELECT * FROM jobs '+('WHERE session_id=? ' if session_id else '')+
                              'ORDER BY created DESC'+(' LIMIT ?' if limit is not None else ''),
                              ((session_id,) if session_id else ()) + ((limit,) if limit is not None else ()))
            return [self.decode_job(r) for r in rows]

    def session_headers(self):
        with self.connect() as db:
            return [dict(r) for r in db.execute('SELECT id,target,created FROM sessions ORDER BY created DESC')]

    def active_job(self):
        with self.connect() as db:
            row = db.execute("SELECT id,action,session_id,state,stage,request FROM jobs WHERE state IN ('QUEUED','RUNNING') ORDER BY created LIMIT 1").fetchone()
        if not row:
            return None
        job = dict(row)
        params = json.loads(job.pop('request')).get('params', {})
        job['duration_seconds'] = {'localization_check': 30, 'waypoint': 3, 'waypoint_revisit': 5}.get(job['action'])
        if job['action'] == 'record':
            job['duration_seconds'] = params.get('seconds', 10)
        return job

    def session(self, session_id):
        ident(session_id)
        with self.connect() as db:
            row = db.execute('SELECT * FROM sessions WHERE id=?', (session_id,)).fetchone()
            if not row:
                raise FieldError('会话不存在', 'not_found')
            return dict(id=row['id'], target=row['target'], created=row['created'], **json.loads(row['data']))

    def update_session(self, session_id, **updates):
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            row = db.execute('SELECT data FROM sessions WHERE id=?', (session_id,)).fetchone()
            data = json.loads(row[0]); data.update(updates)
            db.execute('UPDATE sessions SET data=? WHERE id=?', (canonical(data), session_id))

    def revoke_checks(self, session_id):
        # Invalidate the underlying static check as well as its human approval.
        # Keeping only approved_check=None would allow re-confirming old evidence.
        self.update_session(session_id, eligible_check=None, approved_check=None, approved_at=None)

    def approve_eligible_check(self, session_id, check_id, calibration_identity):
        # The health monitor may revoke evidence while preview/ROS I/O runs.
        # Compare and grant in one DB transaction, never from a cached session.
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            row = db.execute('SELECT data FROM sessions WHERE id=?', (session_id,)).fetchone()
            data = json.loads(row[0])
            if data.get('eligible_check') != check_id:
                raise FieldError('静止检查已失效或被新检查取代；请重新检查30秒并核对叠合')
            data.update(approved_check=check_id, approved_at=time.time(),
                        approved_calibration=calibration_identity)
            db.execute('UPDATE sessions SET data=? WHERE id=?', (canonical(data), session_id))

    def update(self, job_id, state=None, stage=None, result=None, error=None):
        job = self.job(job_id)
        with self.connect() as db:
            db.execute('UPDATE jobs SET state=?,stage=?,updated=?,result=?,error=? WHERE id=?',
                       (state or job['state'], stage or job['stage'], time.time(),
                        canonical(result) if result is not None else canonical(job['result']), error, job_id))
            if stage:
                db.execute('INSERT INTO events(job_id,at,stage) VALUES(?,?,?)', (job_id, time.time(), stage))

    def interrupt_pending(self):
        with self.connect() as db:
            db.execute("UPDATE jobs SET state='INTERRUPTED',stage='worker 重启：原任务未自动重试',"
                       "error='需要核对真实地图/进程及部分文件；未声称跨重启完成',updated=? WHERE state IN ('QUEUED','RUNNING')", (time.time(),))

    def artifact(self, job_id, path, name=None):
        ident(job_id)
        path = Path(path)
        real = path.resolve(strict=True)
        base = self.root/'jobs'/job_id
        if (path.is_symlink() or not real.is_relative_to(base) or not real.is_relative_to(self.root)
                or not real.is_file() or any(p.is_symlink() for p in path.parents if p.is_relative_to(self.root))):
            raise FieldError('产物必须是本任务目录中的普通已完成文件')
        before = real.stat()
        h = hashlib.sha256()
        with real.open('rb') as f:
            for block in iter(lambda: f.read(1024*1024), b''):
                h.update(block)
        after = real.stat()
        if (before.st_ino, before.st_size, before.st_mtime_ns) != (after.st_ino, after.st_size, after.st_mtime_ns):
            raise FieldError('文件仍在变化，不能登记下载')
        aid = uuid.uuid4().hex
        with self.connect() as db:
            db.execute('INSERT INTO artifacts VALUES(?,?,?,?,?,?,?)',
                       (aid, job_id, text(name or real.name, 150), str(real.relative_to(self.root)), after.st_size,
                        h.hexdigest(), canonical([after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns])))
        return aid

    def artifact_info(self, artifact_id):
        ident(artifact_id)
        with self.connect() as db:
            row = db.execute('SELECT * FROM artifacts WHERE id=?', (artifact_id,)).fetchone()
        if not row:
            raise FieldError('产物不存在', 'not_found')
        path = self.root/row['relative_path']
        if (path.is_symlink() or not path.resolve(strict=True).is_relative_to(self.root/'jobs'/row['job_id'])
                or any(p.is_symlink() for p in path.parents if p.is_relative_to(self.root))):
            raise FieldError('产物路径校验失败')
        stat = path.stat()
        fingerprint = canonical([stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns])
        if stat.st_size != row['size'] or row['fingerprint'] != fingerprint:
            raise FieldError('产物内容已改变，不能继续下载')
        return dict(row), path


@contextmanager
def exclusive(path):
    import fcntl
    with Path(path).open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise FieldError('已有建图/现场操作，不能并发执行', 'busy') from exc
        yield
