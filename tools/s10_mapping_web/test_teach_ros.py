#!/usr/bin/env python3
"""Real-ROS test of teach_worker.py (ROS 1 / ROS-O, isolated, no robot).

Run inside the ros1 gateway builder image (roscore, rospy, rosbag available):
  docker run --rm --network none -v "$PWD":/app:ro s10-ros1-gateway-test \
    bash -c 'source /opt/ros/one/setup.bash && python3 /app/test_teach_ros.py'
Publishes a fake x_nav pose (nav_msgs/Odometry), /IMU and a small /LIDAR/POINTS,
drives the worker's HTTP API and checks the recorded bags with the rosbag API.
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

APP = os.path.dirname(os.path.abspath(__file__))
failures = []


def check(name, cond, detail=''):
    print(('PASS ' if cond else 'FAIL ') + name + (('  ' + str(detail)) if detail else ''), flush=True)
    if not cond:
        failures.append(name)


def api(path, body=None):
    data = None if body is None else json.dumps(body).encode()
    r = urllib.request.Request('http://127.0.0.1:8091' + path, data=data, method='POST' if data else 'GET',
                               headers={'Content-Type': 'application/json'} if data else {})
    try:
        with urllib.request.urlopen(r, timeout=70) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def main():
    home = tempfile.mkdtemp(prefix='teachros-')
    env = dict(os.environ, HOME=home, ROS_MASTER_URI='http://127.0.0.1:11311', ROS_IP='127.0.0.1')
    procs = [subprocess.Popen(['roscore', '-p', '11311'], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)]
    time.sleep(4)
    procs.append(subprocess.Popen([sys.executable, os.path.join(APP, 'teach_worker.py'), '--port', '8091',
                                   '--data-dir', os.path.join(home, 'teach'), '--min-free-gb-mapping', '0.5'],
                                  env=env, cwd=APP, stdout=open(os.path.join(home, 'worker.log'), 'w'),
                                  stderr=subprocess.STDOUT))
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    os.environ.update(env)
    rospy.init_node('teach_test_publishers', disable_signals=True)
    pose_pub = rospy.Publisher('/base_link/odom', Odometry, queue_size=10)
    imu_pub = rospy.Publisher('/IMU', Imu, queue_size=50)
    cloud_pub = rospy.Publisher('/LIDAR/POINTS', PointCloud2, queue_size=2)
    motion = dict(v=0.0, x=2.0, y=1.0, yaw=0.5)
    stop = threading.Event()

    def pose_loop():
        while not stop.is_set():
            motion['x'] += motion['v'] * 0.05 * math.cos(motion['yaw'])
            motion['y'] += motion['v'] * 0.05 * math.sin(motion['yaw'])
            m = Odometry()
            m.header.stamp = rospy.Time.now(); m.header.frame_id = 'map'
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = motion['x'], motion['y'], 0.42
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(motion['yaw'] / 2), math.cos(motion['yaw'] / 2)
            m.twist.twist.linear.x = motion['v']
            pose_pub.publish(m)
            time.sleep(0.05)

    def fixed_rate(period, fn):
        def loop():
            nxt = time.monotonic()
            while not stop.is_set():
                fn()
                nxt += period
                time.sleep(max(0.0, nxt - time.monotonic()))
        threading.Thread(target=loop, daemon=True).start()

    cloud = PointCloud2(height=1, width=100, point_step=26, row_step=2600, is_dense=True,
                        fields=[PointField('x', 0, 7, 1), PointField('y', 4, 7, 1), PointField('z', 8, 7, 1),
                                PointField('intensity', 12, 7, 1), PointField('ring', 16, 4, 1),
                                PointField('timestamp', 18, 8, 1)], data=bytes(2600))

    def pub_imu():
        imu = Imu(); imu.header.stamp = rospy.Time.now(); imu_pub.publish(imu)

    def pub_cloud():
        cloud.header.stamp = rospy.Time.now(); cloud.header.frame_id = 'lidar_link'; cloud_pub.publish(cloud)

    threading.Thread(target=pose_loop, daemon=True).start()
    fixed_rate(0.005, pub_imu)
    fixed_rate(0.1, pub_cloud)
    try:
        ok = False
        for _ in range(60):
            code, s = api('/status')
            if code == 200 and s['ros']['ok'] and s['topics']['pose']['age'] is not None and s['topics']['pose']['age'] < 1:
                ok = True
                break
            time.sleep(0.5)
        check('worker connected to ROS and sees the pose', ok, s.get('ros') if 'ros' in s else s)
        check('pose type recognised', s.get('pose_type') == 'nav_msgs/Odometry', s.get('pose_type'))
        time.sleep(3)
        s = api('/status')[1]
        check('rates: pose ~20 Hz, IMU > 100 Hz, lidar ~10 Hz',
              15 < s['topics']['pose']['hz'] < 25 and s['topics']['imu']['hz'] > 100 and 7 < s['topics']['lidar']['hz'] < 13,
              s['topics'])
        code, r = api('/action', dict(action='session_new', label='rostest', map_name='synthetic'))
        check('session', code == 200, r)
        sid = r['session']['id']
        sdir = os.path.join(home, 'teach', 'sessions', sid)
        code, r = api('/action', dict(action='record_start', mode='survey'))
        check('survey recording (rosbag) started', code == 200, r)
        time.sleep(2)
        code, r = api('/action', dict(action='mark', kind='WP', wp_id='WP01'))
        time.sleep(3.6)
        lr = api('/status')[1]['last_result']
        check('still WP01 passes with the real pose stream', lr and lr['result']['passed'] and lr['result']['n'] >= 50, lr and lr['result'])
        check('WP01 position is the published pose', lr and abs(lr['pose'][0] - motion['x']) < 1e-3 and abs(lr['pose'][3] - 0.5) < 1e-3, lr and lr['pose'])
        motion['v'] = 0.5
        time.sleep(0.5)
        api('/action', dict(action='mark', kind='SWIN'))
        time.sleep(3.6)
        lr = api('/status')[1]['last_result']
        check('SWIN while moving fails', lr['kind'] == 'SWIN' and not lr['result']['passed'], lr['result']['reasons'])
        motion['v'] = 0.0
        code, r = api('/action', dict(action='record_stop'))
        bags = [f['name'] for f in r.get('files', []) if f['name'].endswith('.bag')]
        check('survey bag closed', code == 200 and r['exit_code'] == 0 and bags, r)
        import rosbag
        with rosbag.Bag(os.path.join(sdir, bags[0])) as bag:
            info = bag.get_type_and_topic_info().topics
            marks = [json.loads(m.data) for _, m, _ in bag.read_messages(topics=['/teach/mark'])]
        check('survey bag has pose, IMU and marks', {'/base_link/odom', '/IMU', '/teach/mark'} <= set(info), sorted(info))
        check('marks in bag carry the 3 s result', any(m.get('wp_id') == 'WP01' and m['result']['passed'] for m in marks), marks[:1])
        code, r = api('/action', dict(action='record_start', mode='path'))
        check('path recording started', code == 200, r)
        motion['v'] = 0.5
        time.sleep(4.0)
        motion['v'] = 0.0
        time.sleep(0.5)
        s = api('/status')[1]
        check('path length ~2 m (0.5 m/s x 4 s)', 1.6 < s['path']['length_m'] < 2.4, s['path'])
        code, r = api('/action', dict(action='record_stop'))
        check('path bag closed with length', code == 200 and r['exit_code'] == 0 and 'path_length_m' in r, r)
        code, r = api('/action', dict(action='record_start', mode='mapping'))
        check('mapping recording started', code == 200, r)
        time.sleep(3.0)
        s = api('/status')[1]
        check('loop helper reports distance to start', s['loop'] is not None and s['loop']['dist'] < 0.05, s['loop'])
        code, r = api('/action', dict(action='record_stop'))
        mbags = [f['name'] for f in r.get('files', []) if f['name'].endswith('.bag')]
        check('mapping bag closed', code == 200 and r['exit_code'] == 0 and mbags, r)
        with rosbag.Bag(os.path.join(sdir, mbags[0])) as bag:
            t = bag.get_type_and_topic_info().topics
        check('mapping bag has lidar, IMU and pose', {'/LIDAR/POINTS', '/IMU', '/base_link/odom'} <= set(t),
              {k: v.message_count for k, v in t.items()})
        lidar = t['/LIDAR/POINTS'].message_count if '/LIDAR/POINTS' in t else 0
        check('mapping bag lidar rate ~10 Hz over ~3 s', 20 <= lidar <= 40, lidar)
        with open(os.path.join(sdir, 'recordings.jsonl')) as f:
            rec = [json.loads(line) for line in f]
        check('recordings.jsonl logs 3 start/stop pairs', sum(1 for x in rec if x['event'] == 'stop') == 3)
    finally:
        stop.set()
        for p in reversed(procs):
            p.terminate()
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                p.kill()
    print('TEACH_ROS_TESTS_OK' if not failures else 'TEACH_ROS_TESTS_FAILED %s' % failures)
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
