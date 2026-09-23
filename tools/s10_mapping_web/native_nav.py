"""Bounded native-policy app jobs on 106. No motion on service/page startup.

Uses the field worker's operation lock, a persistent request ledger, and a local
runtime lease. The browser cannot edit engineering verification or ROS arguments.
"""
import copy
import json
import os
import re
import signal
import sqlite3
import subprocess
import threading
import time
import uuid
from pathlib import Path

from field_core import DEFAULT_ROOT, FieldError, canonical, exclusive
from field_worker import OPERATION_LOCK, RPCHandler, RPCServer, rpc_call

ROOT = DEFAULT_ROOT / 'native_navigation'
SOCKET = ROOT / 'worker.sock'
CODE = Path('/home/user/goai_native_start_b_20260917/code')
KINDS = {
    'observe': ('实机检查（不运动）', 'start_b', 15),
    'flat': ('原地切换普通模式', 'start', 12),
    'stairs': ('原地切换楼梯模式', 'start', 12),
    'start': ('Start 平地段', 'start', 180),
    'b': ('Area B 楼梯段', 'b', 300),
    'start_b': ('Start → Area B', 'start_b', 600),
}
VERIFY_TEXT = {
    'map_and_global_status': '确认 v3 地图与全局定位',
    'odom_reference': '核对定位坐标与机身参考点',
    'cloud_frame_deskew_self_filter': '核对点云坐标与机身过滤',
    'perception_projection': '验收障碍物与地形输入',
    'route_and_staging_points': '现场复核路线与切换模式的位置',
    'native_command_timeout_and_stop': '验收原厂指令超时停车',
}
TERMINAL = {'completed', 'cancelled', 'failed', 'interrupted'}


def token(value):
    if not isinstance(value, str) or not re.fullmatch('[a-f0-9]{32}', value):
        raise FieldError('请求编号无效')
    return value


def atomic(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(canonical(value))
    temp.replace(path)


def last_record(path):
    if not path.exists():
        return None
    with path.open('rb') as f:
        f.seek(max(0, path.stat().st_size - 65536))
        lines = f.read().splitlines()
    for line in reversed(lines):
        try:
            value = json.loads(line)
            if isinstance(value, dict) and 'wall_time' in value:
                return value
        except (ValueError, UnicodeDecodeError):
            pass  # A partially written final line is never a fresh snapshot.
    return None


class Engine:
    def __init__(self, root=ROOT, code=CODE, lock_path=OPERATION_LOCK, field=rpc_call):
        self.root, self.code, self.lock_path, self.field = Path(root), Path(code), Path(lock_path), field
        self.root.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.guard = threading.RLock()
        self.active = None
        self.lease = 0.
        self.stop_requested = False
        with self.db() as db:
            db.execute('CREATE TABLE IF NOT EXISTS runs (id TEXT PRIMARY KEY, key TEXT UNIQUE, request TEXT, data TEXT)')
            rows = db.execute('SELECT id,data FROM runs').fetchall()
            for rid, raw in rows:
                row = json.loads(raw)
                if row['state'] not in TERMINAL:
                    row.update(state='interrupted', message='服务曾中断；未自动重启测试，请核对机器人。')
                    db.execute('UPDATE runs SET data=? WHERE id=?', (canonical(row), rid))

    def db(self):
        return sqlite3.connect(self.root / 'runs.sqlite3', timeout=3)

    def row(self, rid):
        token(rid)
        with self.db() as db:
            result = db.execute('SELECT data FROM runs WHERE id=?', (rid,)).fetchone()
        if not result:
            raise FieldError('没有这次测试记录', 'not_found')
        return json.loads(result[0])

    def update(self, rid, **values):
        with self.guard:
            row = self.row(rid)
            row.update(values)
            with self.db() as db:
                db.execute('UPDATE runs SET data=? WHERE id=?', (canonical(row), rid))
            return row

    def config(self):
        accepted = self.code / 'native_transfer/config/field.accepted.json'
        path = accepted if accepted.is_file() else self.code / 'native_transfer/config/field.pending.json'
        return json.loads(path.read_text())

    def routes(self):
        values = []
        for kind in ('start', 'b', 'start_b'):
            raw = json.loads((self.code / f'native_transfer/config/{kind}.draft.json').read_text())
            pts = raw['waypoints']
            modes = [p['kind'] for i, p in enumerate(pts) if i == 0 or p['kind'] != pts[i-1]['kind']]
            values.append(dict(id=kind, label=KINDS[kind][0], points=pts, modes=modes,
                               start=pts[0]['position'], end=pts[-1]['position']))
        return values

    def blockers(self, kind, snapshot, config):
        if kind == 'observe':
            return []
        reasons = []
        if not snapshot or not 0 <= time.time() - snapshot.get('wall_time', 0) < 60:
            reasons.append('先运行一次实机检查；参考读数有效期 60 秒，启动时仍会重新核对。')
        else:
            count = snapshot.get('nav_cmd_publishers')
            if count != 0:
                reasons.append(f'当前有 {count if count is not None else "未知数量的"} 个其他速度指令来源；需先确认并释放控制权。')
            if snapshot.get('nav_cmd_subscribers', 0) < 1 or snapshot.get('gait_subscribers', 0) < 1:
                reasons.append('尚未发现完整的原厂速度／模式接收端。')
            if snapshot.get('motion_state') != 17:
                reasons.append('机器人尚未处于原厂 RL 行走状态；本页不会自动站立。')
            if snapshot.get('hes') != 0:
                reasons.append('急停状态未解除或尚未读到。')
            ages = snapshot.get('stream_ages', {})
            if any(ages.get(k, 999) > .35 for k in ('pose', 'motion', 'hes')):
                reasons.append('定位／运动／急停反馈缺失或过期。')
            faults = snapshot.get('input_faults', {})
            relevant = [k for k in faults if k in ('motion', 'operator', 'motion_status')]
            if relevant:
                reasons.append('运动反馈存在异常，请展开诊断详情核对。')
        if kind not in ('flat', 'stairs'):
            reasons += [v for k, v in VERIFY_TEXT.items() if config.get('verified', {}).get(k) is not True]
            if config.get('policy_call_acceptance') != 'passed_on_048':
                reasons.append('原厂 policy 调用与实际速度响应尚未完成实机验收。')
            if config.get('body_z_offset') is None or any(config.get(k) is None for k in ('odom_child_from_base', 'base_from_cloud', 'healthy_location_code')):
                reasons.append('机身高度、坐标变换或定位状态定义尚未校准。')
            if not ((snapshot or {}).get('map_context') or {}).get('global_mode'):
                reasons.append('尚未确认全局定位；把机器人放到 Start 后，在实时位置页核对。')
        return reasons

    def status(self):
        # A slow field-worker query must not hold the stop/heartbeat mutex.
        try:
            field_busy = self.field(dict(action='list')).get('active_job') if not self.active else None
            field_error = None
        except Exception:
            field_busy, field_error = None, '现场助手状态暂不可用；恢复连接后才能开始测试。'
        with self.guard:
            with self.db() as db:
                rows = [json.loads(r[0]) for r in db.execute('SELECT data FROM runs ORDER BY rowid DESC LIMIT 20')]
            for row in rows[:1]:
                snap = last_record(self.root / row['id'] / 'telemetry.jsonl')
                if snap:
                    row['snapshot'] = snap
            current = rows[0] if rows else None
            snap = current.get('snapshot') if current else None
            config = self.config()
            available = {k: self.blockers(k, snap, config) for k in KINDS}
            return dict(online=True, active_id=self.active, current=current, history=rows,
                        server_time=time.time(),
                        blockers=available, field_busy=bool(field_busy), field_error=field_error,
                        routes=self.routes(), map_id=config.get('map_id'),
                        policy_accepted=config.get('policy_call_acceptance') == 'passed_on_048')

    def rpc(self, request):
        if not isinstance(request, dict):
            raise FieldError('导航请求格式无效')
        action = request.get('action')
        if action == 'status' and set(request) == {'action'}:
            return self.status()
        with self.guard:
            if action == 'report' and set(request) == {'action', 'id'}:
                row = self.row(request['id'])
                row['snapshot'] = last_record(self.root / row['id'] / 'telemetry.jsonl')
                return dict(scope='native_navigation_test', physical_stop_verified=False, run=row)
            if action in ('cancel', 'heartbeat') and set(request) <= {'action', 'id', 'owner'}:
                row = self.row(request.get('id'))
                if row['id'] != self.active:
                    return row
                if action == 'heartbeat':
                    with self.db() as db:
                        raw = db.execute('SELECT request FROM runs WHERE id=?', (row['id'],)).fetchone()[0]
                    if token(request.get('owner')) != json.loads(raw).get('owner'):
                        raise FieldError('只有发起测试的页面可以保持测试运行', 'conflict')
                    self.lease = time.monotonic() + 4
                else:
                    self.stop_requested = True
                    self.update(row['id'], state='stopping', message='已收到停止请求；等待控制程序回执，现场请用遥控器确认停稳。')
                return self.row(row['id'])
            if action == 'submit':
                return self.submit(request)
        raise FieldError('不支持的导航操作')

    def submit(self, request):
        if set(request) != {'action', 'kind', 'key', 'owner', 'onsite'} or request['kind'] not in KINDS:
            raise FieldError('测试参数无效；不能从网页传入配置、文件路径或 ROS 参数')
        token(request['key']); token(request['owner'])
        with self.db() as db:
            found = db.execute('SELECT id,request FROM runs WHERE key=?', (request['key'],)).fetchone()
        if found:
            if found[1] != canonical(request):
                raise FieldError('同一请求编号不能用于不同测试', 'conflict')
            return self.row(found[0])
        if self.active:
            raise FieldError('已有测试正在运行，先停止或等待结束', 'busy')
        status = self.status()
        if status['field_busy'] or status['field_error']:
            raise FieldError(status['field_error'] or '现场助手正在执行任务，请等待完成', 'busy')
        kind = request['kind']
        if kind != 'observe':
            if request['onsite'] != dict(supervisor=True, clear=True, position=True):
                raise FieldError('请确认有人持遥控器、路线净空和起点位置')
            if status['blockers'][kind]:
                raise FieldError('；'.join(status['blockers'][kind]), 'conflict')
        elif request['onsite'] != {}:
            raise FieldError('实机检查不需要现场运动确认')
        reservation = exclusive(self.lock_path)
        reservation.__enter__()
        try:
            if self.field(dict(action='list')).get('active_job'):
                raise FieldError('现场助手已有任务，请等待完成', 'busy')
            rid = uuid.uuid4().hex
            (self.root / rid).mkdir(mode=0o700)
            row = dict(id=rid, key=request['key'], kind=kind, label=KINDS[kind][0], state='starting',
                       created=time.time(), message='正在连接原厂状态；尚未发送运动指令。', snapshot=None)
            with self.db() as db:
                db.execute('INSERT INTO runs VALUES (?,?,?,?)', (rid, request['key'], canonical(request), canonical(row)))
            self.active, self.stop_requested, self.lease = rid, False, time.monotonic() + 4
            threading.Thread(target=self.run, args=(rid, reservation), daemon=True).start()
            return row
        except BaseException:
            reservation.__exit__(None, None, None)
            raise

    def run(self, rid, reservation):
        processes, files = [], []
        try:
            row = self.row(rid)
            kind = row['kind']; _, route, duration = KINDS[kind]
            directory = self.root / rid
            config = copy.deepcopy(self.config())
            config.setdefault('verified', {})['operator_on_site'] = kind != 'observe'
            atomic(directory / 'config.json', config)
            control = directory / 'control.json'
            config_dir = self.code / 'native_transfer/config'
            args = ['/bin/bash', str(self.code / 'native_transfer/run_on_106.sh'),
                    '--config', str(directory / 'config.json'), '--route', str(config_dir / f'{route}.draft.json'),
                    '--output', str(directory / 'telemetry.jsonl'), '--duration', str(duration)]
            if kind in ('flat', 'stairs', 'observe'):
                # Observer uses raw pose for status even while transforms remain unverified.
                args += ['--probe-gait', 'stairs' if kind == 'stairs' else 'flat']
            if kind != 'observe':
                args += ['--enable-motion', '--app-control-file', str(control)]
                atomic(control, dict(command='wait', expires_monotonic=self.lease))
            env = dict(os.environ)
            env['PYTHONPATH'] = str(self.code) + ':' + str(self.code / 'src/s10_auto_nav') + ':' + env.get('PYTHONPATH', '')
            for filename, command in [('context.log', ['/usr/bin/python3', '-m', 'native_transfer.map_context', '--duration', str(duration+8)]), ('runtime.log', args)]:
                f = (directory / filename).open('xb'); files.append(f)
                processes.append(subprocess.Popen(command, env=env, stdout=f, stderr=subprocess.STDOUT))
            proc = processes[-1]
            started = time.monotonic(); ending = None; failed = None; cancelled = False
            while proc.poll() is None:
                now = time.monotonic()
                snap = last_record(directory / 'telemetry.jsonl')
                with self.guard:
                    stop = self.stop_requested or (kind != 'observe' and now >= self.lease)
                    if stop and ending is None:
                        ending, cancelled = now, True
                    if kind != 'observe':
                        command = 'cancel' if ending is not None else ('arm' if now-started >= 3 and snap else 'wait')
                        atomic(control, dict(command=command, expires_monotonic=self.lease))
                    if snap:
                        event = snap.get('app_event') or {}
                        if snap.get('state') == 'fault' or (event.get('action') == 'arm' and event.get('success') is False):
                            failed = event.get('message') or snap.get('reason') or '控制程序拒绝继续'
                            ending = ending or now
                        if snap.get('state') in ('done', 'stopped'):
                            ending = ending or now
                        self.update(rid, snapshot=snap, state='stopping' if ending is not None else ('observing' if kind == 'observe' else 'running'),
                                    message='正在只读采集；不会创建原厂运动发布器。' if kind == 'observe' else '以控制程序实时状态为准；可随时停止本次测试。')
                if now-started > duration+15:
                    failed, ending = '控制程序超时；请使用遥控器核对停稳。', ending or now
                if ending is not None and now-ending > .7:
                    proc.send_signal(signal.SIGINT)
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill(); failed = '控制程序未正常退出，已终止；请立即用遥控器接管。'
                    break
                time.sleep(.15)
            proc.wait(timeout=3)
            snap = last_record(directory / 'telemetry.jsonl')
            if failed or (proc.returncode != 0 and not cancelled and ending is None):
                state, message = 'failed', failed or '控制程序启动或运行失败；未通过验收。'
            elif cancelled or (snap or {}).get('state') == 'stopped':
                state, message = 'cancelled', '本次测试已结束；软件回执不等于实机停稳，请以遥控器和现场观察为准。'
            elif kind == 'observe' and snap:
                state, message = 'completed', '实机检查已完成；这不是导航通过结论。读数在下方显示。'
            elif kind in ('flat', 'stairs') and (snap or {}).get('accepted_gaits'):
                state, message = 'completed', '已收到原厂模式反馈；尚未验证行走速度响应。'
            elif (snap or {}).get('state') == 'done':
                state, message = 'completed', '控制程序报告到达终点；仍需核对现场到达位置与停稳。'
            else:
                state, message = 'failed', '测试到时或缺少完成回执；未通过验收。'
            self.update(rid, state=state, message=message, snapshot=snap, ended=time.time())
        except Exception as exc:
            self.update(rid, state='failed', message='测试未完成：' + str(exc), ended=time.time())
        finally:
            for proc in reversed(processes):
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill(); proc.wait(timeout=3)
            for f in files:
                f.close()
            reservation.__exit__(None, None, None)
            with self.guard:
                self.active = None


def main():
    os.umask(0o077)
    if not os.path.ismount('/var/opt/robot/data'):
        raise RuntimeError('机器人数据分区未挂载；拒绝启动导航测试服务')
    ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
    with exclusive(ROOT / 'service.lock'):
        if SOCKET.is_symlink() or (SOCKET.exists() and not SOCKET.is_socket()):
            raise RuntimeError('导航 socket 路径异常')
        if SOCKET.exists():
            SOCKET.unlink()
        with RPCServer(str(SOCKET), RPCHandler) as server:
            os.chmod(SOCKET, 0o600)
            server.engine = Engine()
            server.serve_forever()


if __name__ == '__main__':
    main()
