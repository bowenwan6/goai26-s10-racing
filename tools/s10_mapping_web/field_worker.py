"""106 field worker: persists accepted jobs; never publishes robot motion."""
import argparse
import base64
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import socketserver
import stat
import threading
import time

from field_core import (DEFAULT_ROOT, FieldError, Store, binding, canonical, exclusive,
                        finite_list, ident, localization_reasons, localization_start_reasons, pose_summary)
from waypoint_review import REVISIT_SECONDS, REFERENCE_LIMITS, source_compatible, strict_stationary, compare, comparison_svg

# The robot's overlay environment cannot reliably create systemd RuntimeDirectory.
# Use the existing private data root, with the same default for server and RPC.
SOCKET = DEFAULT_ROOT/'worker.sock'
OPERATION_LOCK = Path(__file__).resolve().parent/'operation.lock'


def rpc_call(request, socket_path=SOCKET):
    raw = canonical(request).encode()+b'\n'
    if len(raw) > 16384:
        raise FieldError('RPC 请求过大')
    with socket.socket(socket.AF_UNIX) as sock:
        sock.settimeout(15)
        sock.connect(str(socket_path))
        sock.sendall(raw)
        reader = sock.makefile('rb')
        response = reader.readline(4*1024*1024+1)
        if len(response) > 4*1024*1024 or not response.endswith(b'\n'):
            raise FieldError('worker 响应超过上限或未完整收到', 'unavailable')
    value = json.loads(response)
    if not value.get('ok'):
        raise FieldError(value.get('error', 'worker 未返回结果'), value.get('code', 'unavailable'))
    return value['result']


class Engine:
    def __init__(self, store, adapter, lock_path=OPERATION_LOCK):
        self.store, self.adapter, self.lock_path = store, adapter, Path(lock_path)

    def rpc(self, request):
        action = request.get('action')
        if action == 'imu_diag':
            diagnostic = getattr(self.adapter, 'imu_diag', None)
            if diagnostic is None:
                raise FieldError(getattr(self.adapter, 'imu_diag_error', None) or
                                 '此worker尚未安装真实IMU诊断；不会自动使用模拟值', 'unavailable')
            payload = request.get('request')
            if not isinstance(payload, dict):
                raise FieldError('诊断请求格式无效')
            return diagnostic.rpc(payload)
        if action == 'dashboard':
            # One SSH/RPC round trip, rather than ageing live data behind several queries.
            listing = self.rpc(dict(action='list', session_id=request.get('session_id')))
            try:
                live, error = self.adapter.snapshot(), None
            except Exception as exc:
                live, error = None, str(exc)
            return dict(health=self.rpc(dict(action='health')), live=live, live_error=error, listing=listing)
        if action == 'submit':
            # Serialize reservation with legacy mapping start/save. A retry of an
            # already reserved key is still discoverable while that job runs.
            payload = request.get('request')
            from field_core import validate_request
            _, key, _, _ = validate_request(payload)
            with self.store.connect() as db:
                row = db.execute('SELECT request,id FROM jobs WHERE key=?', (key,)).fetchone()
            if row:
                if row['request'] != canonical(payload):
                    raise FieldError('幂等键已用于不同请求', 'conflict')
                return self.store.job(row['id'])
            with exclusive(self.lock_path):
                return self.store.submit(payload)
        if action == 'list':
            sid = request.get('session_id')
            jobs = self.store.jobs(sid)
            eligible = {sid: self.store.session(sid).get('eligible_check')
                        for sid in {job['session_id'] for job in jobs}}
            return dict(jobs=jobs, eligible_checks=eligible, demo=self.adapter.demo,
                        active_job=self.store.active_job(),
                        sessions=self.store.session_headers(),
                        session=self.store.session(sid) if sid else None,
                        overview=self.overview(sid) if sid else None)
        if action == 'job':
            return self.store.job(request.get('job_id'))
        if action == 'session':
            return self.store.session(request.get('session_id'))
        if action == 'health':
            return dict(worker=True, demo=self.adapter.demo, ui_contract=2, features=['waypoint_revisit_v1','field_workflow_v2'], storage=str(self.store.root),
                        warning='演示模式：所有数据是模拟的，不可用于现场验收' if self.adapter.demo else '不控制行走或停车')
        if action == 'preview':
            session = self.store.session(request.get('session_id'))
            return self.adapter.preview(session['target'])
        if action == 'live':
            return self.adapter.snapshot()
        if action in ('artifact_info', 'artifact_read'):
            info, path = self.store.artifact_info(request.get('artifact_id'))
            if action == 'artifact_info':
                return {k: info[k] for k in ('id', 'name', 'size', 'sha256')}
            offset, length = request.get('offset'), request.get('length')
            if type(offset) is not int or type(length) is not int or offset < 0 or not 1 <= length <= 262144 or offset > info['size']:
                raise FieldError('下载范围无效')
            # O_NOFOLLOW plus root containment above rejects a replaced symlink.
            fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
            with os.fdopen(fd, 'rb') as f:
                stat = os.fstat(f.fileno())
                if canonical([stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns]) != info['fingerprint']:
                    raise FieldError('下载打开时文件身份变化，拒绝读取')
                f.seek(offset)
                raw = f.read(min(length, info['size']-offset))
            return dict(offset=offset, data=base64.b64encode(raw).decode(), eof=offset+len(raw) >= info['size'])
        raise FieldError('不支持的 worker RPC')

    def overview(self, session_id):
        # History on screen is bounded; counts and reports must cover the whole session.
        jobs = self.store.jobs(session_id, limit=None)
        points = []
        for job in reversed(jobs):
            result = job.get('result') or {}
            if job['action'] == 'waypoint' and job['state'] == 'SUCCEEDED' and result.get('pose', {}).get('passed'):
                points.append(dict(job_id=job['id'], created=job['created'], name=result['name'],
                                   floor=result.get('floor'), xyz=result['pose']['xyz'],
                                   yaw=result['pose'].get('yaw'), marker_note=result.get('marker_note', ''),
                                   draft=result.get('draft', True), binding=result.get('binding')))
        def latest(action):
            row = next((j for j in jobs if j['action'] == action), None)
            if row:
                row = dict(row)
                row['result'] = {k: v for k, v in (row.get('result') or {}).items() if k != 'snapshot'}
            return row
        session = self.store.session(session_id)
        eligible = session.get('eligible_check')
        report = latest('finish')
        names = [p['name'] for p in points]
        revisits = [dict(job_id=j['id'], created=j['created'], **{k:(j['result'] or {}).get(k) for k in
                    ('source_waypoint_id','source_name','metrics','within_reference','reference_verified','binding')})
                    for j in reversed(jobs) if j['action']=='waypoint_revisit' and j['state']=='SUCCEEDED']
        return dict(total_jobs=len(jobs), saved_points=points, saved_waypoint_count=len(points),
                    revisits=revisits, revisit_count=len(revisits), latest_revisit=latest('waypoint_revisit'),
                    duplicate_names=sorted({name for name in names if names.count(name) > 1}),
                    selfcheck=latest('selfcheck'), report=report,
                    eligible_job=self.store.job(eligible) if eligible else None,
                    report_outdated=bool(report and any(j['action'] != 'finish' and j['created'] > report['created'] for j in jobs)))

    def stage(self, job, message):
        self.store.update(job['id'], stage=message)

    def evidence(self, job, name, value):
        root = self.store.root/'jobs'/job['id']
        root.mkdir(parents=True, mode=0o700, exist_ok=True)
        final, partial = root/name, root/(name+'.partial')
        with partial.open('x', encoding='utf-8') as f:
            f.write(canonical(value)+'\n'); f.flush(); os.fsync(f.fileno())
        partial.replace(final)
        return self.store.artifact(job['id'], final)

    def current(self, session, require_good=True):
        try:
            snap = self.adapter.snapshot()
        except Exception:
            self.store.revoke_checks(session['id'])
            raise
        if snap.get('map_name') != session['target']:
            self.store.revoke_checks(session['id'])
            raise FieldError('当前图不是本次目标图')
        if require_good:
            reasons = localization_reasons(snap, session.get('binding'))
            if not session.get('binding'):
                reasons.append('尚未保存当前目标地图与定位会话绑定')
            if reasons:
                self.store.revoke_checks(session['id'])
                raise FieldError('；'.join(reasons))
        return snap

    def approved(self, session):
        snap = self.current(session)
        check_id = session.get('approved_check')
        if not check_id or self.store.session(session['id']).get('eligible_check') != check_id:
            raise FieldError('先完成静止检查并人工核对实时叠合')
        check = self.store.job(check_id)
        result = check.get('result') or {}
        if check['session_id'] != session['id'] or check['state'] != 'SUCCEEDED' or not result.get('passed'):
            raise FieldError('人工确认所关联的静止检查无效')
        if result.get('binding') != binding(snap):
            raise FieldError('确认后地图/定位会话已改变；请重新静止检查')
        if session.get('approved_calibration') != self.calibration_identity():
            self.store.revoke_checks(session['id'])
            raise FieldError('标定/参考点配置已变化，需要重新检查和确认')
        # Human confirmation is bounded; new observations still gate every action.
        age = time.time()-session.get('approved_at', 0)
        if not 0 <= age <= 1800:
            raise FieldError('人工确认超过 30 分钟或时钟异常，请重新核对')
        return snap

    def calibration_identity(self):
        return hashlib.sha256(canonical(self.adapter.calibration()).encode()).hexdigest()

    def invalidate_on_health(self):
        """Latch any observed loss; reconnect/recovery never restores approval."""
        with self.store.connect() as db:
            ids = [r[0] for r in db.execute('SELECT id FROM sessions')]
        for session_id in ids:
            session = self.store.session(session_id)
            if session.get('approved_check') or session.get('eligible_check'):
                try:
                    self.current(session)
                except Exception:
                    self.store.revoke_checks(session_id)

    def execute(self, job_id):
        job = self.store.job(job_id)
        if job['state'] != 'QUEUED':
            return job
        try:
            with exclusive(self.lock_path):
                self.store.update(job_id, state='RUNNING', stage='开始执行（手机断线不取消任务）')
                result = self.perform(job)
                result['demo'] = self.adapter.demo
                self.store.update(job_id, stage='检查结束，正在保存结果文件；请等待最终结果')
                self.evidence(job, 'result.json', result)
                self.store.update(job_id, state='SUCCEEDED', stage='任务完成；请阅读质量结论', result=result)
        except Exception as exc:
            if getattr(exc, 'details', None) is not None:
                try:
                    self.evidence(job, 'failure-diagnostics.json', exc.details)
                except Exception:
                    # A failed evidence write must never convert the failed operation into success.
                    pass
            self.store.update(job_id, state='FAILED', stage='检查/操作失败，未自动重试', error=str(exc))
        return self.store.job(job_id)

    def perform(self, job):
        action, params = job['action'], job['request'].get('params', {})
        session = self.store.session(job['session_id'])
        progress = lambda message: self.stage(job, message)
        if action == 'selfcheck':
            progress('核对设备、内部时间、地图、传感器及本地存储')
            result = self.adapter.selfcheck(session['target'])
            snap = self.adapter.snapshot()
            updates = dict(last_selfcheck=job['id'])
            if session.get('binding') != binding(snap):
                updates.update(binding=binding(snap), eligible_check=None, approved_check=None, approved_at=None)
            self.store.update_session(session['id'], **updates)
            return result
        if action == 'load_map':
            # Clearing approval before any external operation prevents a response
            # loss or partial vendor failure from leaving an old green light.
            self.store.revoke_checks(session['id'])
            result = self.adapter.load_map(session['target'], progress)
            snap = self.adapter.snapshot()
            if snap.get('map_name') != session['target']:
                raise FieldError('厂商切图后 active 地图不符合目标，未自动回退')
            self.store.update_session(session['id'], binding=binding(snap))
            result.update(binding=binding(snap), localization_verified=False,
                          next='加载不代表定位成功；继续 30 秒静止检查和人工叠合确认')
            return result
        if action == 'localization_check':
            # A new attempt supersedes old evidence even if it fails or raises.
            self.store.revoke_checks(session['id'])
            snap = self.current(session, require_good=False)
            setup = localization_start_reasons(snap, session.get('binding'))
            if not session.get('binding'):
                setup.append('先重新自检绑定当前地图/设备/定位会话')
            if setup:
                raise FieldError('；'.join(setup))
            progress('连续诊断30秒；允许异常时检查，检查不等于修复定位，也不自动通过')
            samples = self.adapter.sample(30, progress)
            result = pose_summary(samples, 30, binding(snap))
            after = self.adapter.snapshot()
            result['reasons'] = list(dict.fromkeys(result['reasons']+localization_reasons(after, binding(snap))))
            result['passed'] = not result['reasons']
            result.update(binding=binding(snap), human_confirmed=False,
                          inspection_only=True, before_status=snap.get('status'), after_status=after.get('status'),
                          limitation='静止稳定不是全场精度或碰撞安全证明')
            self.evidence(job, 'localization-samples.json', samples)
            if result['passed']:
                self.store.update_session(session['id'], eligible_check=job['id'])
            return result
        if action == 'confirm_overlay':
            snap = self.current(session)
            check = self.store.job(params['check_id'])
            r = check.get('result') or {}
            age = time.time()-check['updated']
            if check['action'] != 'localization_check' or check['session_id'] != session['id'] or check['state'] != 'SUCCEEDED' or not r.get('passed'):
                raise FieldError('只能确认本会话已通过的静止检查')
            if self.store.session(session['id']).get('eligible_check') != check['id']:
                raise FieldError('静止检查已失效或被新检查取代；请重新检查30秒并核对叠合')
            if r.get('binding') != binding(snap) or not 0 <= age <= 300:
                raise FieldError('检查已过期（5 分钟）或地图/会话变化，需要重新检查')
            # Require current same-frame live scan and an identity-bound map
            # preview before claiming a visual overlay was actually possible.
            preview = self.adapter.preview(session['target'])
            cloud = snap.get('aligned_cloud', {})
            age, stamp = cloud.get('age'), cloud.get('stamp')
            valid_time = (type(age) in (int, float) and type(stamp) in (int, float)
                          and math.isfinite(age) and math.isfinite(stamp) and 0 <= age <= 1
                          and abs(stamp-snap['pose']['stamp']) <= .5)
            source_age = cloud.get('stamp_age_s', snap.get('board_time', time.time())-stamp if type(stamp) in (int, float) else None)
            valid_time = valid_time and type(source_age) in (int, float) and math.isfinite(source_age) and -.05 <= source_age <= .5
            points = cloud.get('points')
            valid_points = isinstance(points, list) and bool(points) and all(finite_list(p, 3) for p in points)
            if preview.get('map_identity') != snap['map_identity'] or cloud.get('frame') != 'map' or cloud.get('error') or not valid_points or not valid_time:
                raise FieldError('当前地图/扫描叠合不可用或不同步，不能确认')
            self.store.approve_eligible_check(session['id'], check['id'], self.calibration_identity())
            return dict(confirmed=True, check_id=check['id'], binding=binding(snap), expires_s=1800)
        if action == 'record':
            snap = self.approved(session)
            progress('检查录制依赖、话题类型与磁盘预算')
            root = self.store.root/'jobs'/job['id']
            root.mkdir(mode=0o700, parents=True, exist_ok=True)
            result = self.adapter.record(root, params['kind'], params.get('seconds', 10), binding(snap), progress)
            result['binding'] = binding(snap)
            # Adapter finalizes all files before registration. Partial recordings
            # remain on disk but are never advertised as complete raw data.
            for p in result.pop('files', []):
                self.store.artifact(job['id'], p)
            return result
        if action == 'waypoint':
            draft = params.get('draft', True)
            snap = self.current(session) if draft else self.approved(session)
            calibration_identity = self.calibration_identity()
            progress('重新采样停稳位姿 3 秒；不使用浏览器缓存')
            samples = self.adapter.sample(3, progress)
            quality = pose_summary(samples, 3, binding(snap))
            calibration = self.adapter.calibration()
            calibrated = (calibration.get('verified') is True and bool(calibration.get('reference_frame'))
                          and calibration.get('reference_frame') == quality.get('child_frame')
                          and bool(calibration.get('version')) and bool(calibration.get('evidence')))
            if calibration_identity != self.calibration_identity():
                raise FieldError('采样期间标定配置变化，拒绝保存混合语义航点')
            if not quality['passed']:
                raise FieldError('标点采样未通过：'+'；'.join(quality['reasons']))
            if not draft and not calibrated:
                raise FieldError('参考点/外参未核实：仅能保存位置草稿，不能生成有效导航航点')
            return dict(name=params['name'], floor=params.get('floor', '未填写'), segment='flat',
                        marker_note=params.get('marker_note', ''),
                        navigation_valid=not draft and calibrated, draft=draft,
                        binding=binding(snap), pose=quality, calibration=calibration,
                        calibration_identity=calibration_identity,
                        note='Z 是机器人参考点高度，不是地面；不同楼层不自动连线')
        if action == 'waypoint_revisit':
            snap = self.approved(session)
            source_job = self.store.job(params['waypoint_id'])
            if (source_job['session_id'] != session['id'] or source_job['action'] != 'waypoint'
                    or source_job['state'] != 'SUCCEEDED'):
                raise FieldError('只能复测本会话成功保存的原航点；请恢复该点所在会话，不能仅凭同名匹配')
            source = source_job['result'] or {}
            calibration_id = self.calibration_identity()
            source_compatible(source, snap, calibration_id)
            progress('在所选实体标记附近停稳，连续采样5秒；无需与原位置/朝向完全一致，不控制行走')
            samples = self.adapter.sample(REVISIT_SECONDS, progress)
            self.evidence(job, 'revisit-samples.json', dict(source_waypoint_id=source_job['id'],
                          before=snap, samples=samples, human_confirmation=params))
            quality = pose_summary(samples, REVISIT_SECONDS, binding(snap))
            strict_stationary(quality, samples)
            if any(quality.get(k) != source['pose'].get(k) for k in ('frame', 'child_frame')):
                raise FieldError('复测采样的定位参考帧与原航点不同，不能比较')
            after = self.approved(session)
            if binding(after) != binding(snap) or calibration_id != self.calibration_identity():
                raise FieldError('采样期间地图、定位会话或参考点配置改变，复测不可比')
            source_compatible(source, after, calibration_id)
            result = compare(source, quality, binding(snap), params.get('reference_note', '未填写'), params.get('ruler_offset_cm'), params.get('alignment_mode', 'nearby'))
            result.update(source_waypoint_id=source_job['id'], source_name=source['name'],
                          source_created=source_job['created'], source_floor=source.get('floor'),
                          source_marker_note=source.get('marker_note', ''), calibration_identity=calibration_id, demo=self.adapter.demo)
            self.evidence(job, 'waypoint-revisit.json', result)
            root = self.store.root/'jobs'/job['id']
            drawing = root/'revisit-xy.svg'
            with drawing.open('x', encoding='utf-8') as f:
                f.write(comparison_svg(result)); f.flush(); os.fsync(f.fileno())
            self.store.artifact(job['id'], drawing)
            return result
        if action == 'finish':
            progress('汇总实际结果、缺项与产物校验；不发送停车命令')
            return self.finish(job, session)
        raise FieldError('无效任务')

    def finish(self, job, session):
        jobs = self.store.jobs(session['id'], limit=None)
        try:
            snap = self.adapter.snapshot()
        except Exception as exc:
            snap = dict(error='结束时无法取得现场状态：'+str(exc))
        current_binding = binding(snap)
        reasons = localization_reasons(snap, session.get('binding'))
        try:
            self.approved(session)
        except Exception as exc:
            reasons.append(str(exc))
        points = []
        drafts = []
        saved_count = 0
        kinds = set()
        for item in reversed(jobs):
            result = item.get('result') or {}
            if item['action'] == 'waypoint' and item['state'] == 'SUCCEEDED' and result.get('pose', {}).get('passed'):
                saved_count += 1
                if result.get('draft'):
                    drafts.append(dict(result, source_job_id=item['id']))
            if item['state'] != 'SUCCEEDED' or result.get('binding') != current_binding:
                continue
            if (item['action'] == 'waypoint' and result.get('navigation_valid')
                    and result.get('calibration_identity') == self.calibration_identity()):
                points.append(result)
            if item['action'] == 'record' and result.get('passed'):
                kinds.add(result.get('kind'))
        missing = sorted({'stationary', 'straight', 'turn'}-kinds)
        if missing:
            reasons.append('缺少合格录制：'+', '.join(missing))
        if len(points) < 3:
            reasons.append('本会话有效平地航点少于 3 个')
        if len({p['floor'] for p in points}) > 1 or any(p['floor'] == '未填写' for p in points):
            reasons.append('楼层未明确或跨楼层；不生成自动连接路线')
        passed = not reasons and not self.adapter.demo
        revisits = [dict(job_id=item['id'], created=item['created'], state=item['state'],
                         source_waypoint_id=item['request']['params']['waypoint_id'],
                         error=item.get('error'), result=item.get('result'))
                    for item in reversed(jobs) if item['action']=='waypoint_revisit']
        report = dict(passed=passed, demo=self.adapter.demo, session=session, binding=current_binding,
                      reasons=reasons, valid_waypoint_count=len(points), recordings=sorted(kinds),
                      saved_waypoint_count=saved_count, draft_waypoint_count=len(drafts),
                      revisit_count=sum(r['state']=='SUCCEEDED' for r in revisits),
                      report_scope='整个会话；历史/其他绑定的草稿保留原身份，不自动拼接为导航路线',
                      jobs=jobs, no_motion_commands=True,
                      next='先做离线影子回放与人工路线审阅，尚未授权自主运动')
        self.evidence(job, 'field-report.json', report)
        # Route is deliberately not an executable controller configuration.
        self.evidence(job, 'waypoints-review.json', dict(navigation_ready=False, field_checks_passed=passed,
                      requires_human_route_review=True, map_binding=current_binding, waypoints=points,
                      draft_waypoints=drafts))
        self.evidence(job, 'waypoint-revisits.json', dict(absolute_accuracy_verified=False,navigation_ready=False,demo=self.adapter.demo,
                      reference_limits=REFERENCE_LIMITS, trials=revisits,
                      note='原WP未覆盖；跨会话记录保留各自身份，不合并为导航授权'))
        return {k: v for k, v in report.items() if k not in ('jobs', 'session')}


class RPCHandler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            raw = self.rfile.readline(16385)
            if len(raw) > 16384 or not raw.endswith(b'\n'):
                raise FieldError('RPC 请求大小无效')
            request = json.loads(raw)
            if not isinstance(request, dict):
                raise FieldError('RPC 格式无效')
            result = self.server.engine.rpc(request)
            reply = dict(ok=True, result=result)
        except Exception as exc:
            reply = dict(ok=False, error=str(exc), code=getattr(exc, 'code', 'unavailable'))
        raw_reply = (canonical(reply)+'\n').encode()
        if len(raw_reply) > 4*1024*1024:
            raw_reply = (canonical(dict(ok=False, error='worker 响应超过4MiB上限', code='invalid'))+'\n').encode()
        self.wfile.write(raw_reply)


class RPCServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True


def run(root, socket_path, adapter, lock_path=OPERATION_LOCK):
    os.umask(0o077)
    if not adapter.demo and (not Path(root).resolve().is_relative_to(Path('/var/opt/robot/data'))
                            or not os.path.ismount('/var/opt/robot/data')):
        raise FieldError('生产worker必须使用已挂载的106数据分区；不静默回退其他路径')
    store = Store(root)
    with exclusive(store.root/'worker.lock'):
        store.interrupt_pending()
        with store.connect() as db:
            sessions = [r[0] for r in db.execute('SELECT id FROM sessions')]
        for session_id in sessions:
            store.revoke_checks(session_id)
        socket_path = Path(socket_path)
        socket_path.parent.mkdir(parents=True, mode=0o700, exist_ok=True)
        parent_stat = socket_path.parent.stat()
        if stat.S_IMODE(parent_stat.st_mode) != 0o700 or parent_stat.st_uid != os.geteuid():
            raise FieldError('socket 父目录必须属于运行用户且权限为0700；未自动改权限或回退路径')
        if socket_path.is_symlink():
            raise FieldError('socket 位置存在符号链接，拒绝覆盖')
        if socket_path.exists():
            if not socket_path.is_socket():
                raise FieldError('socket 位置存在非 socket 文件，拒绝覆盖')
            socket_path.unlink()
        engine = Engine(store, adapter, lock_path)
        with RPCServer(str(socket_path), RPCHandler) as server:
            server.engine = engine
            os.chmod(socket_path, 0o600)
            threading.Thread(target=server.serve_forever, daemon=True).start()
            def monitor():
                while True:
                    try:
                        engine.invalidate_on_health()
                    except Exception:
                        # DB/IO failure cannot be advertised as a live health
                        # loop. A service restart will clear all approvals.
                        os._exit(1)
                    time.sleep(1)
            threading.Thread(target=monitor, daemon=True).start()
            while True:
                queued = [j for j in store.jobs() if j['state'] == 'QUEUED']
                if queued:
                    engine.execute(queued[-1]['id'])
                time.sleep(.1)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    p.add_argument('--socket', type=Path, default=SOCKET)
    p.add_argument('--config', type=Path, default=Path('/etc/s10-field/config.json'))
    args = p.parse_args()
    from field_robot import RobotAdapter
    run(args.root, args.socket, RobotAdapter(args.root, args.config))
