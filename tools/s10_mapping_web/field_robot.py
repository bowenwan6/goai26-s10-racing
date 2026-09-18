"""Read-only ROS evidence + explicit vendor map activation and bounded rosbag.

No publishers, navigation goals or replay. Missing calibration stays unverified.
"""
from collections import deque
import importlib.util
import json
import math
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time

from field_core import FieldError, MAP_RE, binding, canonical, finite_list, localization_reasons, pose_summary
import robot_backend as vendor

# Display decoding only; independent rosbag recording keeps original messages.
# At ~0.13 s measured LiDAR source latency, a 0.5 s display interval could
# randomly exceed the unchanged 0.5 s evidence gate. Use 5 Hz cloud previews.
PREVIEW_INTERVALS_S = dict(pose=.1, imu=.1, cloud=.2, aligned_cloud=.2)

# Map-loading evidence only, NOT a motion-controller deadband or IMU calibration.
# 2026-09-16 hand-controller move/stop capture: standing feedback Y ~= -0.018
# while relative position stayed within 3.7 mm over the confirmed stop segment.
# Feedback tolerance is paired with a longer, tighter relative-pose check below.
MAP_STOP_SECONDS = 5
MOTION_RESIDUAL_LIMIT = .02
COMMAND_ZERO_LIMIT = .0005  # Existing planner log resolution/check is unchanged.

PROFILE = {
    '/ODOM': ('nav_msgs/msg/Odometry', True),
    '/IMU': ('sensor_msgs/msg/Imu', True),
    '/LIDAR/POINTS_MERGED': ('sensor_msgs/msg/PointCloud2', True),
    '/ALIGNED_POINTS': ('sensor_msgs/msg/PointCloud2', True),
    '/tf': ('tf2_msgs/msg/TFMessage', False),
    '/tf_static': ('tf2_msgs/msg/TFMessage', False),
    '/LOCATION_STATUS/MATCHING_ERROR': ('std_msgs/msg/Float64MultiArray', False),
}


def parse_navigation(text, now, started_at, invocation):
    """Parse the latest complete Planning Monitor block, never mix blocks."""
    pattern = r'^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)\.(\d{3})\.(\d{3})\]'
    markers = list(re.finditer(pattern, text, re.MULTILINE))
    for i in reversed(range(len(markers))):
        timestamp = markers[i]
        block = text[timestamp.end():markers[i+1].start() if i+1 < len(markers) else len(text)]
        if 'Planning Monitor' not in block:
            continue
        stamp = time.mktime(time.strptime(timestamp[1], '%Y-%m-%d %H:%M:%S'))+int(timestamp[2])/1000+int(timestamp[3])/1e6
        code = re.search(r'Planner Status\s*:\s*Code\s*=\s*(\d+)\b', block)
        values, velocity = [], {}
        complete = bool(re.search(r'Goal\[NO\]', block)) and len(re.findall(r'=+\s*Planning Monitor\s*=+', block)) == 2
        for label in ('Cmd Velocity', 'Motion Vel'):
            match = re.search(re.escape(label)+r'\s*:\s*x_vel\s*=\s*([^|\s]+)\s*\|\s*y_vel\s*=\s*([^|\s]+)\s*\|\s*yaw_vel\s*=\s*([^|\s]+)', block)
            try:
                row = [float(match[j]) for j in (1, 2, 3)] if match else []
            except ValueError:
                row = []
            complete = complete and finite_list(row, 3)
            values.extend(row)
            velocity[label] = row if finite_list(row, 3) else None
        fresh = bool(invocation) and all(type(v) in (int, float) and math.isfinite(v) for v in (now, started_at)) and started_at <= stamp and -.05 <= now-stamp <= 2
        command_zero = bool(velocity.get('Cmd Velocity') is not None and
                            all(abs(v) < COMMAND_ZERO_LIMIT for v in velocity['Cmd Velocity']))
        feedback_within_tolerance = bool(velocity.get('Motion Vel') is not None and
                                        all(abs(v) <= MOTION_RESIDUAL_LIMIT for v in velocity['Motion Vel']))
        idle = bool(fresh and complete and code and int(code[1]) == 999 and command_zero and feedback_within_tolerance)
        reasons = []
        if not fresh:
            reasons.append('导航监视数据过期、时间异常或服务会话未知')
        if not complete:
            reasons.append('最新监视块不完整、有导航目标或速度字段无效')
        if not code or int(code[1]) != 999:
            reasons.append('规划器不是空闲状态 Code999')
        if velocity.get('Cmd Velocity') and not command_zero:
            reasons.append('规划器仍有非零运动命令')
        if velocity.get('Motion Vel') and not feedback_within_tolerance:
            reasons.append('运动反馈超过 ±0.02 容差；先停稳，不把真实运动清零')
        return dict(fresh=bool(fresh and complete), idle=idle, invocation=invocation, stamp=stamp,
                    planner_code=int(code[1]) if code else None, goal_none=bool(re.search(r'Goal\[NO\]', block)),
                    command=velocity.get('Cmd Velocity'), motion=velocity.get('Motion Vel'),
                    zero_limit=COMMAND_ZERO_LIMIT, feedback_limit=MOTION_RESIDUAL_LIMIT,
                    stop_sample_seconds=MAP_STOP_SECONDS, velocity_units='厂商日志原值，单位未独立核实',
                    reason='无导航目标、规划命令为零，运动反馈在 ±0.02 容差内；这不等于已停稳，加载前还要检查5秒位姿/IMU，仍须确认其他策略退出' if idle else '；'.join(reasons))
    return dict(fresh=False, idle=False, invocation=invocation, stamp=None, reason='没有可解析的完整导航monitor')


def navigation_status():
    try:
        info = vendor.service('planner')
        if info.get('ActiveState') != 'active':
            raise FieldError('planner 非活动，无法确认命令归属/空闲状态')
        now = time.time()
        started = now-(time.monotonic()-int(info['ExecMainStartTimestampMonotonic'])/1e6)
        day = time.strftime('%Y_%m%d')
        path = Path('/var/opt/robot/log')/day/f'planner.{day}.log'
        with path.open('rb') as f:
            f.seek(max(0, f.seek(0, 2)-65536)); text = f.read().decode(errors='replace')
        return parse_navigation(text, now, started, info.get('InvocationID'))
    except Exception as exc:
        return dict(fresh=False, idle=False, reason=str(exc))


class RobotAdapter:
    demo = False

    def __init__(self, root, config_path):
        self.root = Path(root)
        self.root.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.config = json.loads(Path(config_path).read_text()) if Path(config_path).is_file() else {}
        # Config absence deliberately does not guess the robot ID or reference.
        self.lock = threading.Lock()
        self.latest, self.counts, self.error = {}, {}, None
        self.node = None
        self.imu_diag, self.imu_diag_error = None, None
        try:
            from imu_diag import Diagnostic
            self.imu_diag = Diagnostic(self.root/'imu_diagnostics', nav=navigation_status)
        except Exception as exc:
            self.imu_diag_error = '诊断初始化失败：'+str(exc)
        self.map_cache, self.status_cache = {}, None
        self.status_at = 0
        self.started = time.monotonic()
        threading.Thread(target=self.sense, daemon=True).start()

    def diagnostic_wants(self):
        try:
            return self.imu_diag is not None and self.imu_diag.wants()
        except Exception as exc:
            self.imu_diag_error = '诊断采集失败：'+str(exc)
            return False

    def offer_diagnostic(self, key, raw, msg, stamp, now):
        # This optional observer must never stop the original ROS evidence loop.
        try:
            import base64
            evidence = dict(topic='imu' if key == 'imu' else 'odom', received=now,
                            received_wall=time.time(), stamp=stamp, frame=msg.header.frame_id)
            if key == 'imu':
                w, a, q = msg.angular_velocity, msg.linear_acceleration, msg.orientation
                evidence.update(values=[w.x,w.y,w.z,a.x,a.y,a.z], quaternion=[q.x,q.y,q.z,q.w],
                    orientation_covariance=list(msg.orientation_covariance),
                    angular_velocity_covariance=list(msg.angular_velocity_covariance),
                    linear_acceleration_covariance=list(msg.linear_acceleration_covariance))
                if msg.angular_velocity_covariance[0] == -1:
                    evidence['values'][:3] = [None]*3
                if msg.linear_acceleration_covariance[0] == -1:
                    evidence['values'][3:] = [None]*3
            else:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                v, w = msg.twist.twist.linear, msg.twist.twist.angular
                evidence.update(values=[p.x,p.y,p.z], quaternion=[q.x,q.y,q.z,q.w],
                    child_frame=msg.child_frame_id, twist=[v.x,v.y,v.z,w.x,w.y,w.z],
                    pose_covariance=list(msg.pose.covariance),twist_covariance=list(msg.twist.covariance))
            if self.imu_diag.active:
                evidence['cdr_b64'] = base64.b64encode(raw).decode()
            self.imu_diag.ingest(evidence)
        except Exception as exc:
            self.imu_diag_error = '诊断采集失败：'+str(exc)
            if self.imu_diag is not None:
                self.imu_diag.error = self.imu_diag_error
                self.imu_diag.dropped += 1

    def calibration(self):
        c = self.config.get('calibration', {})
        return dict(verified=c.get('verified') is True,
                    reference_frame=c.get('reference_frame'), version=c.get('version'),
                    evidence=c.get('evidence'),
                    note='ODOM 的空 child_frame 和零协方差均不是标定证明')

    def sense(self):
        try:
            import rclpy
            from rclpy.signals import SignalHandlerOptions
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.serialization import deserialize_message
            from sensor_msgs.msg import PointCloud2, Imu
            from nav_msgs.msg import Odometry
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            self.node = rclpy.create_node('s10_field_evidence', enable_rosout=False, start_parameter_services=False)

            def receive(key, raw, kind):
                now = time.monotonic()
                diagnostic = key in ('pose', 'imu') and self.diagnostic_wants()
                with self.lock:
                    old = self.latest.get(key)
                    self.counts[key] = self.counts.get(key, 0)+1
                    if not diagnostic and old and now-old['received'] < PREVIEW_INTERVALS_S[key]:
                        return
                msg = deserialize_message(raw, kind)
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                if diagnostic:
                    self.offer_diagnostic(key, raw, msg, stamp, now)
                    if old and now-old['received'] < PREVIEW_INTERVALS_S[key]:
                        return  # Existing field preview gates and rate remain unchanged.
                row = dict(frame=msg.header.frame_id, received=now, stamp=stamp,
                           **{k: v for k, v in vendor.measurement_time(stamp, now, time.time(), old).items() if k != 'stamp'})
                if kind is Odometry:
                    p, q = msg.pose.pose.position, msg.pose.pose.orientation
                    row.update(xyz=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w],
                               child_frame=msg.child_frame_id, covariance=list(msg.pose.covariance))
                    if not finite_list(row['xyz'], 3) or not finite_list(row['quaternion'], 4):
                        row.pop('xyz'); row.pop('quaternion')
                        row['error'] = '位姿含非有限数，拒绝作为定位/标点证据'
                    row['covariance'] = [v if math.isfinite(v) else None for v in row['covariance']]
                elif kind is PointCloud2:
                    row.update(point_count=msg.width*msg.height, points=vendor.cloud_points(msg, 900))
                else:
                    row.update(angular_velocity=[msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                    if not finite_list(row['angular_velocity'], 3):
                        row['angular_velocity'] = None
                        row['error'] = 'IMU角速度含非有限数'
                with self.lock:
                    self.latest[key] = row

            for topic, cls, key in (('/ODOM', Odometry, 'pose'), ('/IMU', Imu, 'imu'),
                                     ('/LIDAR/POINTS_MERGED', PointCloud2, 'cloud'),
                                     ('/ALIGNED_POINTS', PointCloud2, 'aligned_cloud')):
                self.node.create_subscription(cls, topic, lambda raw, k=key, c=cls: receive(k, raw, c),
                                              qos_profile_sensor_data, raw=True)
            # Optional diagnostic-only body feedback. Never feeds snapshot(),
            # navigation_status(), waypoint gates or any control publisher.
            try:
                from drdds.msg import MotionInfo
                def receive_body(raw):
                    if not self.diagnostic_wants():return
                    try:
                        import base64
                        msg=deserialize_message(raw,MotionInfo);d=msg.data;now=time.monotonic()
                        row=dict(topic='body_motion',received=now,received_wall=time.time(),
                                 stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                                 frame_id=msg.header.frame_id,motion_state=d.motion_state.state,
                                 values=[d.vel_x,d.vel_y,d.vel_yaw])
                        if self.imu_diag.active:row['cdr_b64']=base64.b64encode(raw).decode()
                        self.imu_diag.ingest(row)
                    except Exception as exc:
                        self.imu_diag_error='本体速度诊断读取失败：'+str(exc)
                self.node.create_subscription(MotionInfo,'/MOTION_INFO',receive_body,qos_profile_sensor_data,raw=True)
            except Exception as exc:
                self.imu_diag_error='本体速度诊断未接入：'+str(exc)
            while True:
                rclpy.spin_once(self.node, timeout_sec=.1)
        except Exception as exc:
            self.error = str(exc)

    def map_identity(self, name):
        import hashlib
        if not isinstance(name, str) or not MAP_RE.fullmatch(name):
            raise FieldError('地图名无效')
        path = vendor.MAPS/name
        if path.is_symlink() or not path.is_dir() or path.resolve().parent != vendor.MAPS.resolve():
            raise FieldError('地图目录无效或不是受信普通目录')
        files = [path/n for n in ('full_cloud.pcd', 'occ_grid.yaml', 'occ_grid.pgm')]
        if any(p.is_symlink() or not p.is_file() or p.stat().st_size == 0 for p in files):
            raise FieldError('地图所需文件缺失、为空或为符号链接')
        fingerprint = [(p.name, p.stat().st_size, p.stat().st_mtime_ns, p.stat().st_ctime_ns) for p in files]
        cache = self.map_cache.get(name)
        if cache and cache[0] == fingerprint:
            return cache[1]
        hashes = []
        for p in files:
            h = hashlib.sha256()
            with p.open('rb') as f:
                for block in iter(lambda: f.read(1024*1024), b''):
                    h.update(block)
            hashes.append([p.name, h.hexdigest()])
        if fingerprint != [(p.name, p.stat().st_size, p.stat().st_mtime_ns, p.stat().st_ctime_ns) for p in files]:
            raise FieldError('地图在校验期间被修改')
        value = name+':sha256:'+hashlib.sha256(canonical(hashes).encode()).hexdigest()
        self.map_cache[name] = (fingerprint, value)
        return value

    def snapshot(self):
        schedule_now = time.monotonic()
        if self.status_cache is None or schedule_now-self.status_at > .5:
            loc = vendor.localization_status()
            self.status_cache = dict(loc=loc, mapping=vendor.service().get('ActiveState'), navigation=navigation_status())
            self.status_at = schedule_now
        info = self.status_cache
        loc = info['loc']
        identity, identity_error = None, None
        try:
            identity = self.map_identity(loc.get('active_map'))
        except Exception as exc:
            identity_error = str(exc)
        with self.lock:
            # Vendor service/log/map reads above may be slow while ROS callbacks
            # continue receiving. Sample clocks only once we hold the data lock,
            # not before that IO; do not hide genuinely future data with a clamp.
            snapshot_now = time.monotonic()
            snapshot_wall = time.time()
            streams = {k: dict(v, age=snapshot_now-v['received'], stamp_age_s=snapshot_wall-v['stamp'])
                       for k, v in self.latest.items()}
        status = dict(loc.get('status', {}))
        status['fresh'] = (status.get('fresh') is True and -.05 <= snapshot_wall-status.get('stamp', 0) <= 5)
        import hashlib
        machine_hash = hashlib.sha256(Path('/etc/machine-id').read_bytes().strip()).hexdigest()
        identity_verified = (self.config.get('expected_machine_id_sha256') == machine_hash and bool(self.config.get('robot_id')))
        return dict(robot_id=self.config.get('robot_id') if identity_verified else None,
                    identity_verified=identity_verified, identity_note='配置必须与106实际machine-id哈希匹配，不能仅自报48号',
                    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                    map_name=loc.get('active_map'), map_identity=identity, map_error=identity_error,
                    invocation=loc.get('invocation'), started_at=loc.get('started_at'),
                    mapping_active=info['mapping'] not in ('inactive', 'failed'),
                    localization_active=loc.get('service') == 'active', status=status,
                    error=self.error, board_time=snapshot_wall, monotonic=snapshot_now, calibration=self.calibration(),
                    navigation=info.get('navigation', {}),
                    demo=False, **streams)

    def topics(self):
        if self.node is None:
            raise FieldError('ROS 观察节点未就绪：'+str(self.error))
        result = []
        for name, (kind, required) in PROFILE.items():
            endpoints = self.node.get_publishers_info_by_topic(name)
            valid = any(e.topic_type == kind for e in endpoints)
            result.append(dict(name=name, type=kind, required=required, available=valid,
                               publishers=len(endpoints), reliability=[str(e.qos_profile.reliability) for e in endpoints],
                               durability=[str(e.qos_profile.durability) for e in endpoints]))
        return result

    def selfcheck(self, target):
        # Brief warmup is part of the persisted job, never a browser timer.
        time.sleep(max(0, 2-(time.monotonic()-self.started)))
        snap = self.snapshot()
        checks = []
        def add(name, good, detail):
            checks.append(dict(name=name, state='pass' if good else 'fail', detail=detail))
        add('设备身份', bool(snap['robot_id']), snap['robot_id'] or '部署配置尚未匹配真实106机器身份')
        try:
            target_id = self.map_identity(target)
            add('候选地图', True, target_id)
        except Exception as exc:
            add('候选地图', False, str(exc))
        free = shutil.disk_usage(self.root).free
        add('106 持久数据分区', os.path.ismount('/var/opt/robot/data') and self.root.resolve().is_relative_to(Path('/var/opt/robot/data')), str(self.root))
        add('存储余量', free >= 2*1024**3, f'{free/1024**3:.2f} GiB；保留至少 2 GiB')
        for key in ('pose', 'imu', 'cloud'):
            s = snap.get(key, {})
            good = s.get('age', 999) <= .5 and -.05 <= s.get('stamp_age_s', 999) <= .5 and not s.get('error')
            add(key+' 时序', good, '源时间与106时间比较；不依赖公网NTP' if good else '输入过期/时间异常或尚无消息')
        modules = ['rosbag2_py', 'rclpy']
        add('ROS 录制依赖', all(importlib.util.find_spec(m) for m in modules) and shutil.which('ros2') is not None, 'rosbag2_py / rclpy / ros2 CLI')
        topics = self.topics()
        add('必需录制话题', all(t['available'] for t in topics if t['required']), '缺少静态TF不阻断原始诊断录制，但不能因此推断外参')
        checks.append(dict(name='参考点/外参', state='pass' if self.calibration()['verified'] else 'unknown',
                           detail='未验证只能保存位置草稿；不妨碍原始诊断采集'))
        checks.append(dict(name='互联网', state='not_required', detail='户外无WAN是正常条件，使用机器人内部Wi-Fi/有线网'))
        return dict(passed=all(c['state'] != 'fail' for c in checks), checks=checks, snapshot=snap,
                    topics=topics, binding=binding(snap), target_map=target,
                    current_localization_reasons=localization_reasons(snap))

    def sample(self, seconds, progress):
        deadline = time.monotonic()+seconds
        samples = []
        last_tick = -1
        progress_interval = 1 if seconds <= 5 else 5
        while time.monotonic() < deadline:
            samples.append(self.snapshot())
            tick = int(seconds-(deadline-time.monotonic()))
            if tick//progress_interval != last_tick:
                progress(f'采样 {tick}/{seconds} 秒（不是定位精度百分比）')
                last_tick = tick//progress_interval
            time.sleep(.1)
        progress('采样结束，正在检查数据与保存结果；请继续等待')
        return samples

    def load_map(self, target, progress):
        target_id = self.map_identity(target)
        before = self.snapshot()
        if before['mapping_active']:
            raise FieldError('建图服务活动/未知，禁止切图')
        # Human stationary/remote confirmations are checked at request validation.
        # IMU angular movement adds evidence, never a safety-certified stop check.
        imu = before.get('imu', {})
        if (imu.get('error') or any(type(imu.get(k)) not in (int, float) or not math.isfinite(imu[k])
                or not -.05 <= imu[k] <= .5 for k in ('age', 'stamp_age_s'))):
            raise FieldError('IMU 证据不新鲜，未切图')
        if not finite_list(imu.get('angular_velocity'), 3) or any(abs(v) > .08 for v in imu['angular_velocity']):
            raise FieldError('检测到转动，请用手柄停稳再操作')
        if before['map_name'] == target:
            return dict(loaded=True, changed=False, map_identity=target_id, message='目标图已经启用；没有重复重启定位')
        progress('采样5秒停稳证据：允许小反馈残差，但位置/姿态必须稳定，规划命令必须为零')
        samples = self.sample(MAP_STOP_SECONDS, progress)
        stationary = pose_summary([before, *samples], MAP_STOP_SECONDS, binding(before), require_global=False)
        pose0 = before.get('pose', {})
        relative_distance, relative_angle = math.inf, math.inf
        if (finite_list(pose0.get('xyz'), 3) and finite_list(pose0.get('quaternion'), 4)
                and samples and all(finite_list(s.get('pose', {}).get('xyz'), 3)
                                    and finite_list(s.get('pose', {}).get('quaternion'), 4) for s in samples)):
            relative_distance = max(math.dist(pose0['xyz'], s['pose']['xyz']) for s in samples)
            q0 = pose0['quaternion']
            def angle_from_start(q):
                norm = math.sqrt(sum(v*v for v in q0)*sum(v*v for v in q))
                return 2*math.acos(min(1., abs(sum(a*b for a,b in zip(q0,q)))/norm)) if norm else math.inf
            relative_angle = max(angle_from_start(s['pose']['quaternion']) for s in samples)
        stationary.update(distance_from_start_m=relative_distance, angle_from_start_rad=relative_angle)
        if relative_distance > .01:
            stationary['reasons'].append('5秒内相对起点位置变化超过1厘米，或位置证据无效')
        if relative_angle > math.radians(1):
            stationary['reasons'].append('5秒内相对起点姿态变化超过1度，或姿态证据无效')
        if (not stationary['passed'] or stationary.get('spread_m', 999) > .03
                or stationary.get('angle_rad', 999) > math.radians(2)
                or relative_distance > .01 or relative_angle > math.radians(1)):
            raise FieldError('未通过连续新鲜位姿的静止证据；未切图：'+'；'.join(stationary['reasons']))
        for snap in [before, *samples]:
            sample_imu = snap.get('imu', {})
            if (sample_imu.get('error') or not finite_list(sample_imu.get('angular_velocity'), 3)
                    or any(type(sample_imu.get(k)) not in (int, float) or not math.isfinite(sample_imu[k])
                           or not -.05 <= sample_imu[k] <= .5 for k in ('age', 'stamp_age_s'))
                    or any(abs(v) > .08 for v in sample_imu['angular_velocity'])):
                raise FieldError('静止采样期间 IMU 过期、异常或检测到转动；未切图')
            nav = snap.get('navigation', {})
            stamp = nav.get('stamp')
            if (nav.get('fresh') is not True or nav.get('idle') is not True or not nav.get('invocation')
                    or type(stamp) not in (int, float) or not math.isfinite(stamp)
                    or not -.05 <= snap.get('board_time', time.time())-stamp <= 2):
                detail = dict(target=target, current_map=before['map_name'], gate='navigation_idle',
                              navigation=nav, board_time=snap.get('board_time'), stationary_summary=stationary,
                              samples=[dict(board_time=s.get('board_time'), navigation=s.get('navigation'),
                                            pose={k: v for k, v in s.get('pose', {}).items() if k != 'covariance'},
                                            imu=s.get('imu')) for s in [before, *samples]],
                              vendor_activation_called=False)
                raise FieldError('未加载目标图，实际地图仍是 '+before['map_name']+'。原因：'+
                                 nav.get('reason', '导航空闲/零命令证据未知')+
                                 '。请停止所有 policy/导航程序，用手柄停稳后查看②的实时诊断；'
                                 '若仍不通过，下载本次诊断交给维护者，不要继续录制或标点。', details=detail)
        if any(s.get('navigation', {}).get('invocation') != before.get('navigation', {}).get('invocation') for s in samples):
            raise FieldError('静止采样期间导航服务会话变化，未切图')
        progress('调用官方 drmap 激活；若响应丢失，不自动重试')
        vendor.run(['drmap', '--format', 'json', 'map', 'activate', target], timeout=120)
        deadline = time.monotonic()+360
        progress('等待地图预处理（厂商没有可信百分比）')
        while time.monotonic() < deadline:
            self.status_at = 0
            snap = self.snapshot()
            if snap['mapping_active']:
                raise FieldError('外部建图已启动；停止本次验收，不自动回退地图')
            if snap['map_name'] != target:
                raise FieldError('active 地图与请求不一致，不自动再切换')
            if snap['invocation'] and snap['invocation'] != before['invocation']:
                # Do not invent a preprocessing completion marker. Global output
                # is usable evidence; otherwise expose timeout as unverified.
                if snap['status'].get('fresh'):
                    progress('目标图已启用，定位进程开始上报；等待现场全局定位验证')
                    return dict(loaded=True, changed=True, map_identity=target_id,
                                global_now=snap['status'].get('code') == 0 and snap['status'].get('mode') == '全局')
            time.sleep(1)
        raise FieldError('目标图可能已切换，但 360 秒内未确认新定位会话状态；请查看诊断，未自动重试/回退')

    def preview(self, target):
        from field_preview import read_pcd_preview
        identity = self.map_identity(target)
        cache = self.root/'preview'
        cache.mkdir(mode=0o700, exist_ok=True)
        import hashlib
        path = cache/(hashlib.sha256(identity.encode()).hexdigest()+'.json')
        if not path.exists():
            value = dict(map_name=target, map_identity=identity, frame='map',
                         points=read_pcd_preview(vendor.MAPS/target/'full_cloud.pcd'),
                         note='仅显示抽样；不是完整地图或碰撞几何', demo=False)
            partial = path.with_suffix('.partial')
            partial.write_text(canonical(value)); partial.replace(path)
        return json.loads(path.read_text())

    def record(self, root, kind, seconds, expected, progress):
        topics = self.topics()
        missing = [t['name'] for t in topics if t['required'] and not t['available']]
        if missing:
            raise FieldError('必需录制话题无正确类型发布者：'+', '.join(missing))
        if shutil.disk_usage(root).free < 2*1024**3+512*1024**2:
            raise FieldError('空间不足：需要保留2GiB及本段512MiB预算')
        selected = [t['name'] for t in topics if t['available']]
        qos = root/'qos.yaml'
        # Best effort accepts both reliable and sensor-data publishers. Static TF
        # requests transient-local so a latched sample can be captured if present.
        qos.write_text('\n'.join(t+':\n  reliability: best_effort\n  durability: '+
                      ('transient_local' if t == '/tf_static' else 'volatile')+'\n  history: keep_last\n  depth: 20'
                      for t in selected)+'\n')
        bag = root/'raw_bag'
        argv = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag), '--disable-keyboard-controls',
                '--max-cache-size', str(16*1024**2), '--qos-profile-overrides-path', str(qos), '--topics', *selected]
        # The guardian has its own deadline and parent-death pipe, so an HTTP
        # disconnect/worker crash cannot leave an unbounded recorder behind.
        read_fd, write_fd = os.pipe()
        from pathlib import Path as P
        guardian = [sys.executable, str(P(__file__).with_name('field_recorder.py')), '--seconds', str(seconds),
                    '--root', str(root), '--parent-fd', str(read_fd), '--', *argv]
        evidence = []
        try:
            with (root/'recorder.log').open('wb') as log:
                process = subprocess.Popen(guardian, stdout=log, stderr=subprocess.STDOUT,
                                           stdin=subprocess.DEVNULL, pass_fds=(read_fd,))
                os.close(read_fd); read_fd = None
                progress(f'正在106本地录制{seconds}秒；停止录制不等于停车')
                while process.poll() is None:
                    evidence.append(self.snapshot())
                    time.sleep(.2)
                code = process.returncode
        finally:
            os.close(write_fd)
            if read_fd is not None:
                os.close(read_fd)
        monitor_path = root/'recorder-result.json'
        monitor = json.loads(monitor_path.read_text()) if monitor_path.exists() else {'reason': '监护进程未写完结果'}
        progress('录制已停止，正在检查原始文件、逐话题数量与时段')
        result = validate_bag(bag, topics, seconds)
        quality_reasons = []
        for s in evidence:
            for reason in localization_reasons(s, expected):
                if reason not in quality_reasons:
                    quality_reasons.append(reason)
        if code != 0 or monitor.get('reason') != 'duration_complete':
            quality_reasons.append('录制未正常按时长收尾：'+str(monitor))
        result['reasons'].extend(quality_reasons)
        result.update(passed=not result['reasons'], kind=kind, requested_seconds=seconds,
                      profile=topics, guardian=monitor, reference_verified=self.calibration()['verified'],
                      note='原始数据可读不代表无丢包/外参已确认；未播放录制，未发运动指令')
        result['files'] = [p for p in root.rglob('*') if p.is_file() and not p.is_symlink()]
        return result


def validate_bag(path, profile, seconds):
    """Deserialize recorded headers to audit source time, not just bag existence."""
    result = dict(passed=False, topics=[], reasons=[])
    reader = None
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        metadata = Path(path)/'metadata.yaml'
        if not metadata.is_file() or metadata.is_symlink() or metadata.stat().st_size == 0:
            raise FieldError('录制元数据尚未正常生成')
        # The storage API validates the native metadata rather than parsing a
        # localized `ros2 bag info` text output or accepting a present filename.
        rosbag2_py.Info().read_metadata(str(path), 'mcap')
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        for topic in profile:
            if topic['required'] and types.get(topic['name']) != topic['type']:
                result['reasons'].append(topic['name']+' 原始 bag 话题类型与白名单不符')
        stats = {}
        while reader.has_next():
            topic, raw, received = reader.read_next()
            row = stats.setdefault(topic, dict(count=0, first=received, last=received, max_gap=0,
                                              source_first=None, source_last=None, source_max_gap=0, source_errors=0,
                                              header_count=0, frames=[]))
            row['count'] += 1
            row['max_gap'] = max(row['max_gap'], (received-row['last'])/1e9)
            if received < row['last']:
                row['source_errors'] += 1
            row['last'] = received
            msg = deserialize_message(raw, get_message(types[topic]))
            if hasattr(msg, 'header'):
                row['header_count'] += 1
                frame = msg.header.frame_id
                if frame not in row['frames']:
                    row['frames'].append(frame)
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                old = row['source_last']
                if old is not None:
                    row['source_max_gap'] = max(row['source_max_gap'], stamp-old)
                    if stamp <= old:
                        row['source_errors'] += 1
                if abs(received/1e9-stamp) > .5:
                    row['source_errors'] += 1
                if row['source_first'] is None:
                    row['source_first'] = stamp
                row['source_last'] = stamp
        for topic in profile:
            row = dict(topic, **stats.get(topic['name'], dict(count=0)))
            if row['count']:
                row['span_s'] = (row['last']-row['first'])/1e9
            if topic['required'] and (row['count'] < 2 or row.get('span_s', 0) < seconds*.65
                                      or row.get('max_gap', 999) > .75 or row.get('source_max_gap', 999) > .75
                                      or row.get('source_errors', 1) or row.get('header_count', 0) != row['count']
                                      or len(row.get('frames', [])) != 1):
                result['reasons'].append(topic['name']+' 实际消息数/时段/源时间/缺口检查失败')
            result['topics'].append(row)
    except Exception as exc:
        result['reasons'].append('原始 bag 无法完整读取：'+str(exc))
    finally:
        if reader is not None:
            try:
                reader.close()
            except Exception as exc:
                result['reasons'].append('原始 bag 关闭异常：'+str(exc))
    result['passed'] = not result['reasons']
    return result
