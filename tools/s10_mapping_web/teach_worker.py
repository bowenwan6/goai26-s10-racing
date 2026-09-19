#!/usr/bin/env python3
"""S10 teach worker (新 SLAM 采集助手后台) on the AGX, ROS 1 side.

Reads the new SLAM's pose (x_nav, ROS 1), watches the lidar/IMU rates, records
rosbags for three jobs and takes 3 s still samples for marks:

  mapping  建图采集  /LIDAR/POINTS /IMU pose /teach/mark  (large; loop-closure helper)
  survey   标点      pose /IMU /teach/mark                (WP01-WP30, SWIN/SWOUT)
  path     示教路径  pose /IMU /teach/mark                (continuous shortest manual path)

It never publishes anything except the /teach/mark std_msgs/String annotations and
never commands the robot. HTTP JSON API on 127.0.0.1 only; tools/s10_mapping_web/
server.py adds login/CSRF and serves the phone page (/teach).

  python3 teach_worker.py [--pose-topic /base_link/odom] [--port 8091] [--data-dir ~/teach]
  python3 teach_worker.py --fake          # demo robot, no ROS (UI tests, training)
Exit code 3 = the ROS master restarted; the wrapper script restarts the worker.
"""
import argparse
import collections
import json
import math
import os
import random
import shutil
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import teach_core as core

VERSION = 's10_teach_worker 1.0.0'


# ---------------------------------------------------------------- recorders
class BagRecorder:
    """`rosbag record` in its own process group; SIGINT lets it close the bag cleanly."""

    def __init__(self):
        self.proc = None

    def start(self, prefix, topics, split_mb=None):
        cmd = ['rosbag', 'record', '-O', str(prefix) + '.bag']
        if split_mb:
            cmd += ['--split', '--size=%d' % split_mb]
        cmd += topics
        log = open(str(prefix) + '.record.log', 'w')
        self.proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                     cwd=str(Path(prefix).parent), start_new_session=True)
        return ' '.join(cmd)

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self, timeout=45):
        if self.proc is None:
            return None
        for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
            if self.proc.poll() is not None:
                break
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                self.proc.wait(wait)
            except subprocess.TimeoutExpired:
                continue
        code = self.proc.returncode
        self.proc = None
        return code


class FakeRecorder:
    """Demo only: writes the pose stream as text so sizes grow. Never a real bag."""

    def __init__(self, worker):
        self.worker, self.file, self.thread, self.running = worker, None, None, False

    def start(self, prefix, topics, split_mb=None):
        self.file = open(str(prefix) + '.FAKE.txt', 'w')
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return 'FAKE recorder ' + ' '.join(topics)

    def _run(self):
        while self.running:
            p = self.worker.latest
            if p:
                self.file.write('%.3f %.4f %.4f %.4f %.4f\n' % (time.time(), p['x'], p['y'], p['z'], p['yaw']))
                self.file.flush()
            time.sleep(0.1)

    def alive(self):
        return self.running

    def stop(self, timeout=5):
        self.running = False
        if self.thread:
            self.thread.join(2)
        if self.file:
            self.file.close()
        return 0


# ---------------------------------------------------------------- pose sources
class RosSource:
    TYPES = ('nav_msgs/Odometry', 'geometry_msgs/PoseStamped', 'geometry_msgs/PoseWithCovarianceStamped')

    def __init__(self, worker, args):
        self.worker, self.args = worker, args
        self.pub = None
        self.run_id = None
        threading.Thread(target=self._connect, daemon=True).start()

    def _connect(self):
        import rosgraph
        while not rosgraph.is_master_online():
            self.worker.set_ros(False, 'ROS 1 master 未运行：先在 AGX 启动网关（start_gateway.sh）')
            time.sleep(2)
        import rospy
        from std_msgs.msg import String
        rospy.init_node('s10_teach_worker', anonymous=False, disable_signals=True)
        self.run_id = rospy.get_param('/run_id', None)
        self.pub = rospy.Publisher('/teach/mark', String, queue_size=20)
        rospy.Subscriber(self.args.pose_topic, rospy.AnyMsg, self._on_pose, queue_size=50)
        rospy.Subscriber(self.args.lidar_topic, rospy.AnyMsg, lambda m: self.worker.tick('lidar'), queue_size=2)
        rospy.Subscriber(self.args.imu_topic, rospy.AnyMsg, lambda m: self.worker.tick('imu'), queue_size=200)
        self.worker.set_ros(True, '')
        while True:  # a restarted master forgets this node: exit and let the wrapper restart us
            time.sleep(5)
            try:
                if rospy.get_param('/run_id', None) != self.run_id:
                    raise RuntimeError('run_id changed')
            except Exception:
                self.worker.set_ros(False, 'ROS master 已重启，采集助手正在重启')
                os._exit(3)

    def _on_pose(self, raw):
        kind = raw._connection_header.get('type', '')
        try:
            if kind == 'nav_msgs/Odometry':
                from nav_msgs.msg import Odometry
                m = Odometry(); m.deserialize(raw._buff)
                pose, tw = m.pose.pose, m.twist.twist
                speed = math.hypot(tw.linear.x, tw.linear.y)
            elif kind == 'geometry_msgs/PoseStamped':
                from geometry_msgs.msg import PoseStamped
                m = PoseStamped(); m.deserialize(raw._buff)
                pose, speed = m.pose, None
            elif kind == 'geometry_msgs/PoseWithCovarianceStamped':
                from geometry_msgs.msg import PoseWithCovarianceStamped
                m = PoseWithCovarianceStamped(); m.deserialize(raw._buff)
                pose, speed = m.pose.pose, None
            else:
                self.worker.set_pose_type_error('位姿话题类型 %s 不支持（需要 %s）' % (kind, ' / '.join(self.TYPES)))
                return
        except Exception as exc:
            self.worker.set_pose_type_error('位姿解析失败：%s' % exc)
            return
        q = pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.worker.on_pose(pose.position.x, pose.position.y, pose.position.z,
                            core.yaw_from_quat(q.x, q.y, q.z, q.w), roll, pitch, speed, kind,
                            getattr(m.header, 'frame_id', ''))

    def publish(self, text):
        if self.pub is not None:
            from std_msgs.msg import String
            self.pub.publish(String(data=text))


class FakeSource:
    """Demo robot: walks a small course, stops 8 s at every corner (good time to mark)."""

    COURSE = [(0, 0), (6, 0), (6, 4), (12, 4), (12, 10), (4, 12), (0, 6)]

    def __init__(self, worker, args):
        self.worker = worker
        worker.set_ros(True, '')
        threading.Thread(target=self._pose_loop, daemon=True).start()
        threading.Thread(target=self._sensor_loop, daemon=True).start()

    def _pose_loop(self):
        rng = random.Random(10)
        pts = self.COURSE + [self.COURSE[0]]
        i, x, y, yaw = 0, 0.0, 0.0, 0.0
        while True:
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            seg = math.hypot(bx - ax, by - ay)
            yaw = math.atan2(by - ay, bx - ax)
            for k in range(int(seg / 0.02)):
                s = k * 0.02
                x, y = ax + (bx - ax) * s / seg, ay + (by - ay) * s / seg
                self.worker.on_pose(x + rng.gauss(0, .003), y + rng.gauss(0, .003), 0.42 + 0.02 * math.sin(x),
                                    yaw + rng.gauss(0, .002), 0, 0, 0.4, 'fake', 'map')
                time.sleep(0.05)
            for _ in range(160):  # 8 s still
                self.worker.on_pose(bx + rng.gauss(0, .002), by + rng.gauss(0, .002), 0.42,
                                    yaw + rng.gauss(0, .001), 0, 0, 0.0, 'fake', 'map')
                time.sleep(0.05)
            i = (i + 1) % (len(pts) - 1)

    def _sensor_loop(self):
        n = 0
        while True:
            self.worker.tick('imu')
            if n % 20 == 0:
                self.worker.tick('lidar')
            n += 1
            time.sleep(0.005)

    def publish(self, text):
        pass


# ---------------------------------------------------------------- worker state
class Worker:
    def __init__(self, args):
        self.args = args
        self.fake = args.fake
        self.data = Path(args.data_dir).expanduser() / 'sessions'
        self.data.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.rx = {k: collections.deque(maxlen=2000) for k in ('lidar', 'imu', 'pose')}
        self.ros_ok, self.ros_error, self.pose_error = False, '正在连接 ROS', ''
        self.pose_type, self.pose_frame = '', ''
        self.latest = None
        self.trail = []
        self.session = None
        self.marks = []
        self.seq = 0
        self.job = None
        self.last_result = None
        self.recording = None
        self.loop_start = None
        self.path_len, self.path_last = 0.0, None
        self.recorder = FakeRecorder(self) if self.fake else BagRecorder()
        self.source = FakeSource(self, args) if self.fake else RosSource(self, args)
        threading.Thread(target=self._housekeeping, daemon=True).start()

    # ---- inputs
    def set_ros(self, ok, error):
        with self.lock:
            self.ros_ok, self.ros_error = ok, error

    def set_pose_type_error(self, text):
        with self.lock:
            self.pose_error = text

    def tick(self, name):
        self.rx[name].append(time.monotonic())

    def on_pose(self, x, y, z, yaw, roll, pitch, speed, kind, frame):
        t = time.monotonic()
        with self.lock:
            self.rx['pose'].append(t)
            self.pose_error, self.pose_type, self.pose_frame = '', kind, frame
            self.latest = dict(t=t, x=x, y=y, z=z, yaw=yaw, roll=roll, pitch=pitch, speed=speed)
            if self.job is not None:
                self.job['samples'].append(dict(t=t, x=x, y=y, z=z, yaw=yaw))
            last = self.trail[-1] if self.trail else None
            if last is None or math.hypot(x - last[0], y - last[1]) >= core.TRAIL_STEP_M:
                self.trail.append((round(x, 3), round(y, 3)))
                if len(self.trail) > 40000:
                    del self.trail[:10000]
                if self.recording and self.recording['mode'] == 'path':
                    if self.path_last is not None:
                        self.path_len += math.hypot(x - self.path_last[0], y - self.path_last[1])
                    self.path_last = (x, y)
            if self.recording and self.recording['mode'] == 'mapping' and self.loop_start is None:
                self.loop_start = dict(x=x, y=y, yaw=yaw)
            if self.recording and self.recording.get('trail_file'):
                self.recording['trail_file'].write('%.3f,%.4f,%.4f,%.4f,%.5f\n' % (time.time(), x, y, z, yaw))

    # ---- helpers
    def _hz(self, name):
        q = self.rx[name]
        now = time.monotonic()
        recent = [t for t in list(q) if now - t <= 5.0]
        age = now - q[-1] if q else None
        hz = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) >= 2 and recent[-1] > recent[0] else 0.0
        return dict(hz=round(hz, 1), age=None if age is None else round(age, 2))

    def _free_gb(self):
        return shutil.disk_usage(str(self.data)).free / 1e9

    def _session_dir(self):
        if self.session is None:
            raise core.TeachError('请先新建会话')
        return self.data / self.session['id']

    def _append(self, name, row):
        with open(self._session_dir() / name, 'a') as f:
            f.write(json.dumps(row, ensure_ascii=False) + '\n')

    def _publish(self, row):
        try:
            self.source.publish(json.dumps(row, ensure_ascii=False))
        except Exception:
            pass  # annotations are also in marks.jsonl

    # ---- actions
    def session_new(self, label, map_name):
        with self.lock:
            if self.recording:
                raise core.TeachError('正在录制，先结束录制再新建会话')
            if map_name is not None and (not isinstance(map_name, str) or len(map_name) > 80):
                raise core.TeachError('地图名最多 80 字')
            sid = core.session_id(label)
            d = self.data / sid
            if d.exists():
                raise core.TeachError('会话已存在，请稍后再试')
            d.mkdir(parents=True)
            self.session = dict(id=sid, label=label or '', map_name=map_name or '', created=time.time(),
                                pose_topic=self.args.pose_topic, fake=self.fake, worker=VERSION)
            (d / 'session.json').write_text(json.dumps(self.session, ensure_ascii=False, indent=2))
            self.marks, self.seq, self.last_result, self.trail = [], 0, None, []
            self.loop_start, self.path_len, self.path_last = None, 0.0, None
            return dict(session=self.session)

    def session_open(self, sid):
        with self.lock:
            if self.recording:
                raise core.TeachError('正在录制，先结束录制')
            if not isinstance(sid, str) or not core.SESSION_RE.match(sid) or not (self.data / sid / 'session.json').is_file():
                raise core.TeachError('会话不存在')
            self.session = json.loads((self.data / sid / 'session.json').read_text())
            self.marks, self.seq = [], 0
            f = self.data / sid / 'marks.jsonl'
            if f.is_file():
                for line in f.read_text().splitlines():
                    r = json.loads(line)
                    if r.get('kind') == 'VOID':
                        for m in self.marks:
                            if m['seq'] == r['target']:
                                m['void'] = True
                    else:
                        self.marks.append(r)
                    self.seq = max(self.seq, r.get('seq', 0))
            self.trail, self.last_result, self.loop_start = [], None, None
            return dict(session=self.session)

    def record_start(self, mode):
        with self.lock:
            if mode not in core.MODES:
                raise core.TeachError('未知录制类型')
            d = self._session_dir()
            if self.recording:
                raise core.TeachError('已在录制「%s」' % core.MODES[self.recording['mode']])
            if not self.ros_ok:
                raise core.TeachError(self.ros_error or 'ROS 未连接')
            free = self._free_gb()
            need = self.args.min_free_gb_mapping if mode == 'mapping' else 1.0
            if free < need:
                raise core.TeachError('磁盘剩余 %.1f GB，不足 %.0f GB，不能开始%s' % (free, need, core.MODES[mode]))
            pose = self._hz('pose')
            if mode != 'mapping' and (pose['age'] is None or pose['age'] > 1.0):
                raise core.TeachError('没有收到新 SLAM 位姿（%s）：先确认 x_nav 在定位模式' % self.args.pose_topic)
            if mode == 'mapping':
                for name in ('lidar', 'imu'):
                    s = self._hz(name)
                    if s['age'] is None or s['age'] > 1.0:
                        raise core.TeachError('%s 没有数据：先确认网关在运行' % ('点云' if name == 'lidar' else 'IMU'))
            stamp = time.strftime('%Y%m%d_%H%M%S')
            name = '%s_%s' % (mode, stamp)
            topics = [self.args.pose_topic, self.args.imu_topic, '/teach/mark']
            if mode == 'mapping':
                topics = [self.args.lidar_topic] + topics
            cmd = self.recorder.start(d / name, topics, split_mb=2048 if mode == 'mapping' else None)
            trail_file = open(d / (name + '.trail.csv'), 'w')
            trail_file.write('wall_time,x,y,z,yaw\n')
            self.recording = dict(mode=mode, name=name, started=time.time(), started_mono=time.monotonic(),
                                  cmd=cmd, trail_file=trail_file, free_gb_at_start=round(free, 1))
            self.trail = []
            self.loop_start = None
            if mode == 'path':
                self.path_len, self.path_last = 0.0, None
            row = dict(event='start', mode=mode, name=name, wall=time.time(), cmd=cmd, topics=topics)
            self._append('recordings.jsonl', row)
            if mode == 'path':
                self._instant_mark('PATH_START', None, None)
            return dict(recording=self._rec_status())

    def record_stop(self, reason='operator'):
        with self.lock:
            rec = self.recording
            if not rec:
                raise core.TeachError('当前没有在录制')
            if rec['mode'] == 'path':
                self._instant_mark('PATH_END', None, None)
            rec['state'] = 'stopping'
        code = self.recorder.stop()
        with self.lock:
            rec['trail_file'].close()
            files = self._rec_files(rec['name'])
            summary = dict(event='stop', mode=rec['mode'], name=rec['name'], wall=time.time(), reason=reason,
                           exit_code=code, duration_s=round(time.monotonic() - rec['started_mono'], 1), files=files)
            if rec['mode'] == 'mapping' and self.loop_start and self.latest:
                summary['loop'] = self._loop_status()
            if rec['mode'] == 'path':
                summary['path_length_m'] = round(self.path_len, 2)
            self._append('recordings.jsonl', summary)
            self.recording = None
            return summary

    def _rec_files(self, name):
        d = self._session_dir()
        return [dict(name=p.name, bytes=p.stat().st_size) for p in sorted(d.glob(name + '*')) if p.is_file()]

    def mark(self, kind, wp_id=None, note=None):
        core.validate_mark(kind, wp_id, note)
        with self.lock:
            self._session_dir()
            if kind in core.SAMPLED_KINDS:
                if self.job is not None:
                    raise core.TeachError('正在采样，请等 3 秒结束')
                if self.latest is None or time.monotonic() - self.latest['t'] > 1.0:
                    raise core.TeachError('没有新 SLAM 位姿，不能标点')
                self.job = dict(kind=kind, wp_id=wp_id, note=note, t0=time.monotonic(), wall=time.time(), samples=[])
                return dict(job=self._job_status())
            if kind != 'NOTE' and (self.latest is None or time.monotonic() - self.latest['t'] > 1.0):
                raise core.TeachError('没有新 SLAM 位姿，不能标点')
            return dict(mark=self._instant_mark(kind, wp_id, note))

    def _instant_mark(self, kind, wp_id, note):
        self.seq += 1
        p = self.latest if kind != 'NOTE' else None
        row = dict(seq=self.seq, kind=kind, wp_id=wp_id, note=note, wall=time.time(),
                   mode=self.recording['mode'] if self.recording else None,
                   pose=None if p is None else [round(p['x'], 4), round(p['y'], 4), round(p['z'], 4), round(p['yaw'], 5)],
                   result=None)
        self.marks.append(row)
        self._append('marks.jsonl', row)
        self._publish(row)
        return row

    def _finish_job(self):
        job, self.job = self.job, None
        res = core.still_stats(job['samples'])
        self.seq += 1
        row = dict(seq=self.seq, kind=job['kind'], wp_id=job['wp_id'], note=job['note'], wall=job['wall'],
                   mode=self.recording['mode'] if self.recording else None,
                   pose=(res['mean'] + [res['yaw']]) if res.get('mean') else None, result=res,
                   pose_topic=self.args.pose_topic, pose_type=self.pose_type, frame=self.pose_frame)
        self.marks.append(row)
        self._append('marks.jsonl', row)
        self._publish(row)
        self.last_result = row

    def redo(self):
        with self.lock:
            target = next((m for m in reversed(self.marks) if not m.get('void') and m['kind'] not in ('PATH_START', 'PATH_END')), None)
            if target is None:
                raise core.TeachError('没有可以作废的标记')
            target['void'] = True
            self.seq += 1
            row = dict(seq=self.seq, kind='VOID', target=target['seq'], wall=time.time())
            self._append('marks.jsonl', row)
            self._publish(row)
            self.last_result = None
            return dict(voided=target['seq'])

    def trail_clear(self):
        with self.lock:
            self.trail = []
            return {}

    # ---- periodic
    def _housekeeping(self):
        while True:
            time.sleep(0.1)
            with self.lock:
                if self.job is not None and time.monotonic() - self.job['t0'] >= core.SAMPLE_S:
                    self._finish_job()
                rec = self.recording
                stop_reason = None
                if rec and rec.get('state') != 'stopping':
                    if not self.recorder.alive():
                        stop_reason = 'recorder_exited'
                    elif self._free_gb() < self.args.stop_free_gb:
                        stop_reason = 'disk_low'
            if stop_reason:
                try:
                    self.record_stop(stop_reason)
                except core.TeachError:
                    pass

    # ---- status
    def _job_status(self):
        if self.job is None:
            return None
        return dict(kind=self.job['kind'], wp_id=self.job['wp_id'],
                    progress=min(1.0, (time.monotonic() - self.job['t0']) / core.SAMPLE_S), samples=len(self.job['samples']))

    def _rec_status(self):
        rec = self.recording
        if not rec:
            return None
        files = self._rec_files(rec['name'])
        return dict(mode=rec['mode'], name=rec['name'], state=rec.get('state', 'recording'),
                    elapsed_s=round(time.monotonic() - rec['started_mono'], 1),
                    bytes=sum(f['bytes'] for f in files), files=len(files))

    def _loop_status(self):
        if not self.loop_start or not self.latest:
            return None
        o = core.loop_offset(self.loop_start, self.latest)
        return dict(start=[round(self.loop_start['x'], 3), round(self.loop_start['y'], 3), round(self.loop_start['yaw'], 4)],
                    dist=round(o['dist'], 3), heading_deg=round(o['heading_deg'], 1))

    def status(self):
        with self.lock:
            p = self.latest
            trail = self.trail
            if len(trail) > 3000:
                step = len(trail) // 3000 + 1
                trail = trail[::step] + [trail[-1]]
            live = [m for m in self.marks if not m.get('void')]
            return dict(
                version=VERSION, fake=self.fake, now=time.time(),
                ros=dict(ok=self.ros_ok, error=self.ros_error),
                config=dict(pose_topic=self.args.pose_topic, lidar_topic=self.args.lidar_topic,
                            imu_topic=self.args.imu_topic, data_dir=str(self.data)),
                topics=dict(lidar=self._hz('lidar'), imu=self._hz('imu'), pose=self._hz('pose')),
                pose_error=self.pose_error, pose_type=self.pose_type, pose_frame=self.pose_frame,
                pose=None if p is None else dict(x=round(p['x'], 3), y=round(p['y'], 3), z=round(p['z'], 3),
                                                 yaw_deg=round(math.degrees(p['yaw']), 1),
                                                 roll_deg=round(math.degrees(p['roll']), 1),
                                                 pitch_deg=round(math.degrees(p['pitch']), 1),
                                                 speed=None if p['speed'] is None else round(p['speed'], 2)),
                disk=dict(free_gb=round(self._free_gb(), 1), mapping_min_gb=self.args.min_free_gb_mapping),
                session=self.session, recording=self._rec_status(), job=self._job_status(),
                last_result=self.last_result, marks=live[-200:], board=core.wp_board(live),
                pairs=core.switch_pairs(live), trail=trail,
                loop=self._loop_status() if self.recording and self.recording['mode'] == 'mapping' else None,
                path=dict(length_m=round(self.path_len, 2),
                          elapsed_s=round(time.monotonic() - self.recording['started_mono'], 1))
                if self.recording and self.recording['mode'] == 'path' else None)

    def sessions(self):
        rows = []
        for d in sorted(self.data.iterdir(), reverse=True):
            meta = d / 'session.json'
            if d.is_dir() and core.SESSION_RE.match(d.name) and meta.is_file():
                s = json.loads(meta.read_text())
                files = [dict(name=p.name, bytes=p.stat().st_size) for p in sorted(d.iterdir()) if p.is_file()]
                rows.append(dict(id=d.name, label=s.get('label', ''), map_name=s.get('map_name', ''),
                                 created=s.get('created'), fake=s.get('fake', False), files=files,
                                 bytes=sum(f['bytes'] for f in files)))
        return dict(sessions=rows[:50], data_dir=str(self.data))


# ---------------------------------------------------------------- HTTP (loopback)
def make_handler(worker):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, data):
            raw = json.dumps(data, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            u = urlsplit(self.path)
            if u.path == '/status':
                return self.reply(200, worker.status())
            if u.path == '/sessions':
                return self.reply(200, worker.sessions())
            self.reply(404, dict(detail='没有这个接口'))

        def do_POST(self):
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 4096:
                    raise core.TeachError('请求大小无效')
                body = json.loads(self.rfile.read(size))
                action = body.get('action')
                if action == 'session_new':
                    return self.reply(200, worker.session_new(body.get('label', ''), body.get('map_name', '')))
                if action == 'session_open':
                    return self.reply(200, worker.session_open(body.get('session_id')))
                if action == 'record_start':
                    return self.reply(200, worker.record_start(body.get('mode')))
                if action == 'record_stop':
                    return self.reply(200, worker.record_stop())
                if action == 'mark':
                    return self.reply(200, worker.mark(body.get('kind'), body.get('wp_id'), body.get('note')))
                if action == 'redo':
                    return self.reply(200, worker.redo())
                if action == 'trail_clear':
                    return self.reply(200, worker.trail_clear())
                raise core.TeachError('未知操作')
            except core.TeachError as exc:
                return self.reply(400, dict(detail=str(exc)))
            except Exception as exc:
                return self.reply(500, dict(detail='后台错误：%s' % exc))

        def log_message(self, fmt, *args):
            pass
    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pose-topic', default=os.environ.get('S10_TEACH_POSE_TOPIC', '/base_link/odom'))
    ap.add_argument('--lidar-topic', default='/LIDAR/POINTS')
    ap.add_argument('--imu-topic', default='/IMU')
    ap.add_argument('--data-dir', default='~/teach')
    ap.add_argument('--port', type=int, default=8091)
    ap.add_argument('--min-free-gb-mapping', type=float, default=20.0)
    ap.add_argument('--stop-free-gb', type=float, default=3.0)
    ap.add_argument('--fake', action='store_true', help='demo robot without ROS')
    args = ap.parse_args()
    worker = Worker(args)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(worker))

    def shutdown(*_):
        if worker.recording:
            try:
                worker.record_stop('worker_shutdown')
            except Exception:
                pass
        os._exit(0)
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    print('%s on 127.0.0.1:%d, pose %s%s' % (VERSION, args.port, args.pose_topic, ' (FAKE)' if args.fake else ''), flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
