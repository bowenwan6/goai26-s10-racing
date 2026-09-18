"""Bounded SDK stand/lateral trial; dry mode isolates every joint output from motors."""
import json
import copy
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from collections import deque

MANUAL_MAX_INPUT = 1.
# Extended fore/aft probe range: runner multiplies x by 1.5 m/s. Not a motor rating.
MANUAL_MAX_FORWARD_INPUT = 2.
MANUAL_LEASE = .35
# Diagnostic ceilings, not certified motor ratings. Wheel speed uses the body fault rule;
# torque ceilings stay below the local S10 model's leg/wheel 50/14 Nm parameters.
LEG_SPEED_LIMIT = 25.76
WHEEL_SPEED_LIMIT = 30.
LEG_TORQUE_LIMIT = 45.
WHEEL_TORQUE_LIMIT = 12.


def record_joint_peaks(peaks, phase, rows):
    if len(rows) != 16:
        return
    for field, column in (('velocity', 1), ('torque', 2), ('temperature', 3)):
        key = phase + '/' + field
        previous = peaks.get(key, [0.] * 16)
        peaks[key] = [max(a, abs(b[column])) if math.isfinite(b[column]) else a
                      for a, b in zip(previous, rows)]


def manual_request(line):
    packet = json.loads(line)
    if not isinstance(packet, dict) or packet.get('action') not in ('move', 'zero', 'lie'):
        raise ValueError('Invalid manual command')
    axes = packet.get('axes') if packet['action'] == 'move' else [0., 0., 0.]
    if not isinstance(axes, list) or len(axes) != 3 or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
            and abs(v) <= (MANUAL_MAX_FORWARD_INPUT if i == 0 else MANUAL_MAX_INPUT)
            for i, v in enumerate(axes)):
        raise ValueError('Manual axes require finite values: abs(x)<=2, abs(y/yaw)<=1')
    return packet['action'], tuple(axes)


def leased_axes(axes, updated, now, ready):
    return axes if ready and now - updated <= MANUAL_LEASE else (0., 0., 0.)


def handset_request(line):
    packet = json.loads(line)
    if isinstance(packet, dict) and packet.get('action') == 'handset':
        limit, enabled = packet.get('limit'), packet.get('enabled')
        if (isinstance(limit, bool) or not isinstance(limit, (int, float))
                or not math.isfinite(limit) or not .01 <= limit <= 1 or not isinstance(enabled, bool)):
            raise ValueError('Handset limit must be 0.01-1 and enabled must be boolean')
        return limit, enabled
    action, _ = manual_request(line)
    if action not in ('zero', 'lie'):
        raise ValueError('Use the handset for direction commands')
    return action, False


class HandsetInput:
    """Lease moving inputs; this handset stops publishing after returning to center."""
    def __init__(self):
        self.axes, self.updated = (0., 0., 0.), 0.
        self.limit, self.enabled, self.needs_center = .2, False, True
        self.last_c = 0.

    def configure(self, limit, enabled):
        if enabled and not self.enabled:
            self.needs_center = True
        self.limit, self.enabled = limit, enabled

    def receive(self, axes, now):
        if len(axes) != 3 or any(not math.isfinite(v) or abs(v) > 1.001 for v in axes):
            raise ValueError('Invalid handset axes: expected normalized values in [-1,1]')
        self.axes = tuple(0. if abs(v) <= .03 else max(-1., min(1., v)) for v in axes)
        self.updated = now
        if not any(self.axes):
            self.needs_center = False

    def centered(self, now):
        # Called for a freshly received posture key: an old zero is still safe.
        return self.enabled and not self.needs_center and self.updated > 0 and not any(self.axes)

    def output(self, now, ready):
        if any(self.axes) and now - self.updated > MANUAL_LEASE:
            self.needs_center = True
        if not ready or not self.enabled or self.needs_center:
            return (0., 0., 0.)
        return tuple(v * self.limit for v in self.axes)

    def posture_key(self, phase, state, now):
        previous, self.last_c = self.last_c, now
        if now - previous < .5:
            return None  # Duplicate messages / held-key repeats must not toggle twice.
        if phase == 'handset_wait' and state in ('unknown', 'liedown_state') and self.centered(now):
            return 'stand'
        if phase in ('rl_zero', 'manual_ready') and state == 'rl_control':
            return 'lie'
        return None


def validate_feedback(data, now):
    for key, age in (('imu', .3), ('joints', .3), ('hes', .5), ('battery', 3.)):
        if key not in data or now - data[key]['updated'] > age:
            return key + ' feedback stale'
    imu, joints = data['imu'], data['joints']
    if not all(math.isfinite(v) for v in imu['values']):
        return 'nonfinite IMU'
    if max(abs(imu['values'][0]), abs(imu['values'][1])) > .35:
        return 'roll/pitch exceeds 20-degree test bound'
    if max(abs(v) for v in imu['values'][3:6]) > 2.:
        return 'angular velocity exceeds test bound'
    if len(joints['values']) != 16:
        return 'wrong joint count'
    for i, (position, velocity, torque, temperature, status) in enumerate(joints['values']):
        name = ('fl', 'fr', 'hl', 'hr')[i // 4] + '_' + ('hipx', 'hipy', 'knee', 'wheel')[i % 4]
        label = f'joint[{i}] {name}'
        speed_limit = WHEEL_SPEED_LIMIT if i % 4 == 3 else LEG_SPEED_LIMIT
        torque_limit = WHEEL_TORQUE_LIMIT if i % 4 == 3 else LEG_TORQUE_LIMIT
        if not all(math.isfinite(v) for v in (position, velocity, torque, temperature)):
            return label + ' nonfinite feedback'
        if abs(velocity) > speed_limit:
            return f'{label} velocity={velocity:.6f} rad/s exceeds diagnostic limit={speed_limit:g}'
        if abs(torque) > torque_limit:
            return f'{label} torque={torque:.6f} Nm exceeds diagnostic limit={torque_limit:g}'
        if temperature > 80:
            return f'{label} temperature={temperature:.2f} C exceeds limit=80'
        if status != 1:
            return f'{label} status={status} expected=1'
    if data['hes']['value'] != 0:
        return 'hard emergency active'
    if data['battery']['level'] < 20 or data['battery']['protected']:
        return 'battery low or protected'
    return None


def lateral_input(phase, now, deadline, direction, strength=.1):
    if direction not in (-1, 0, 1):
        raise ValueError('Direction must be -1, 0, or 1')
    if not math.isfinite(strength) or not 0 < strength <= 1:
        raise ValueError('Strength must be > 0 and <= 1')
    return strength * direction if phase == 'lateral' and now < deadline else 0.


def legs_responded(before, after):
    """Reject a completed software transition with stationary physical legs."""
    return any(abs(a[0] - b[0]) > .03 for i, (a, b) in enumerate(zip(before, after)) if i % 4 != 3)


def main(root, dry, direction=0, strength=.1, seconds=2, manual=False, handset=False):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from drdds.msg import ImuData, JointsData, JointsDataCmd, BatteryData, StdMsgInt32, Steer
    from std_msgs.msg import String

    output_dir = Path(os.environ.get('S10_TRIAL_OUTPUT_DIR', str(root)))
    output_dir.mkdir(parents=True, exist_ok=True)

    stop, done = threading.Event(), threading.Event()
    heartbeat = [time.monotonic()]
    motion = [((0., 0., 0.), 0.)]
    remote = HandsetInput()
    handset_error = [None]
    stand_requested, end_requested = threading.Event(), threading.Event()
    lie_requested = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())

    def receive():
        for line in sys.stdin:
            line = line.strip()
            if line == 'ping':
                heartbeat[0] = time.monotonic()
                continue
            if not manual or line == 'stop':
                break
            try:
                if handset:
                    setting, enabled = handset_request(line)
                    if isinstance(setting, str):
                        remote.configure(remote.limit, False)
                        if setting == 'lie':
                            lie_requested.set()
                            end_requested.set()
                    else:
                        remote.configure(setting, enabled)
                    heartbeat[0] = time.monotonic()
                    continue
                action, axes = manual_request(line)
            except (ValueError, TypeError):
                break
            motion[0] = (axes, time.monotonic())
            if action == 'lie':
                lie_requested.set()
            heartbeat[0] = time.monotonic()
        stop.set()
    threading.Thread(target=receive, daemon=True).start()

    rclpy.init()
    node = rclpy.create_node('s10_sdk_trial_supervisor')
    data, samples, events = {}, [], []
    joint_peaks = {}
    if manual:
        # ponytail: retain the last minute; stream samples to disk if full sessions are needed.
        samples, events = deque(maxlen=600), deque(maxlen=600)
    phase = 'preflight'
    proc = None
    state = ['unknown']
    cpp_rpy = []
    error = None
    failure = None
    completed = False
    next_zero = 0.
    next_sample = 0.
    lateral_deadline = 0.
    command_y = 0.
    command_axes = (0., 0., 0.)
    next_status = 0.
    private = '/s10_sdk_test'

    def stamp(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def save(key, message, values):
        previous = data.get(key)
        fresh = not previous or previous['stamp'] != stamp(message)
        data[key] = dict(values, stamp=stamp(message),
                         updated=time.monotonic() if fresh else previous['updated'])

    def imu(msg):
        d = msg.data
        save('imu', msg, dict(values=[d.roll, d.pitch, d.yaw, d.omega_x, d.omega_y, d.omega_z]))

    def joints(msg):
        save('joints', msg, dict(values=[[d.position, d.velocity, d.torque, d.motion_temp,
                                        d.status_word] for d in msg.data.joints_data]))
        record_joint_peaks(joint_peaks, phase, data['joints']['values'])

    def joint_commands(msg):
        save('joint_commands', msg, dict(values=[[d.position, d.velocity, d.torque, d.kp, d.kd]
                                               for d in msg.data.joints_data]))

    def battery(msg):
        if msg.data:
            data['battery'] = dict(updated=time.monotonic(),
                level=min(d.battery_level for d in msg.data),
                protected=max(d.protected_state for d in msg.data))

    subscriptions = [node.create_subscription(ImuData, '/IMU_DATA', imu, qos_profile_sensor_data),
        node.create_subscription(JointsData, '/JOINTS_DATA', joints, qos_profile_sensor_data),
        node.create_subscription(JointsDataCmd, private + '/DRY_JOINTS_CMD' if dry else '/JOINTS_CMD',
                                 joint_commands, qos_profile_sensor_data),
        node.create_subscription(BatteryData, '/BATTERY_DATA', battery, qos_profile_sensor_data),
        node.create_subscription(StdMsgInt32, '/HES_STATUS',
            lambda m: data.update(hes=dict(updated=time.monotonic(), value=m.value)),
            qos_profile_sensor_data)]
    keys = node.create_publisher(String, private + '/GAMEPAD_KEY', 10)
    steer = node.create_publisher(Steer, private + '/STEER', 10)

    def event(text):
        events.append(dict(time=time.monotonic(), phase=phase, text=text))
        try:
            print(text, flush=True)
        except OSError:
            stop.set()  # Keep cleanup running even if the SSH log stream disappears.

    def key(name):
        msg = String()
        msg.data = name
        keys.publish(msg)
        event('KEY ' + name)

    def handset_key(msg):
        name = {'G12_KEY_C': 'G20_KEY_L1', 'G12_KEY_A': 'G20_KEY_L2',
                'G12_KEY_B': 'G20_KEY_R1', 'G12_KEY_D': 'G20_KEY_R2'}.get(msg.data, msg.data)
        event('HANDSET_KEY ' + name)
        if name == 'G20_KEY_R2':
            handset_error[0] = 'handset requested damping (R2)'
            stop.set()
        elif name == 'G20_KEY_R1' and phase in ('stand', 'rl_zero', 'manual_ready'):
            lie_requested.set()
        elif name == 'G20_KEY_L1':
            action = remote.posture_key(phase, state[0], time.monotonic())
            if action == 'stand':
                stand_requested.set()
            elif action == 'lie':
                lie_requested.set()

    if handset:
        def handset_axes(msg):
            try:
                remote.receive((msg.data.x, msg.data.y, msg.data.yaw), time.monotonic())
            except ValueError as exc:
                handset_error[0] = str(exc)
                stop.set()

        # Dry checks inject only private inputs; never publish synthetic factory keys.
        handset_prefix = private + '/handset' if dry else ''
        subscriptions.extend([
            node.create_subscription(Steer, handset_prefix + '/STEER', handset_axes, qos_profile_sensor_data),
            node.create_subscription(String, handset_prefix + '/GAMEPAD_KEY', handset_key, qos_profile_sensor_data)])

    def spin(guard=True):
        nonlocal next_zero, next_sample, command_y, command_axes, next_status
        rclpy.spin_once(node, timeout_sec=.01)
        now = time.monotonic()
        if guard:
            if stop.is_set() or now - heartbeat[0] > 1:
                raise RuntimeError(handset_error[0] or 'local heartbeat lost / cancelled')
            err = validate_feedback(data, now)
            if err:
                raise RuntimeError(err)
            if proc is not None and proc.poll() is not None:
                raise RuntimeError('deployment process exited')
            if state[0] == 'joint_damping':
                raise RuntimeError('deployment entered damping unexpectedly')
            if phase in ('lateral', 'after_lateral', 'manual_ready') and state[0] != 'rl_control':
                raise RuntimeError('left RL state during lateral trial')
        if now >= next_zero:
            msg = Steer()
            value = lateral_input(phase, now, lateral_deadline, direction, strength) if guard else 0.
            if manual:
                ready = guard and phase == 'manual_ready' and not lie_requested.is_set()
                axes = (remote.output(now, ready) if handset else
                        leased_axes(*motion[0], now, ready))
                if axes != command_axes:
                    event('AXES ' + json.dumps(axes))
                command_axes = axes
                msg.data.x, value, msg.data.yaw = axes
            if value != command_y:
                event('AXIS_Y ' + str(value))
            command_y = msg.data.y = value
            msg.header.stamp = node.get_clock().now().to_msg()
            steer.publish(msg)
            next_zero = now + (.02 if handset else .1)
        if now >= next_sample:
            samples.append(dict(time=now, phase=phase, state=state[0], axis_y=command_y,
                                axes=command_axes, data=dict(data)))
            next_sample = now + .1
        if manual and now >= next_status and 'imu' in data and 'battery' in data:
            event('STATUS ' + json.dumps(dict(phase=phase, state=state[0],
                battery=data['battery']['level'], rpy=data['imu']['values'][:3], axes=command_axes,
                handset_age=now-remote.updated if remote.updated else None,
                handset_center_required=remote.needs_center if handset else None)))
            next_status = now + 1.

    def observe(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            spin()

    def wait_state(expected, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            spin()
            if state[0] == expected:
                return
        raise RuntimeError('no transition to ' + expected)

    def output_reader():
        with (output_dir / ('deploy-dry.log' if dry else 'deploy-live.log')).open('w') as log:
            for line in proc.stdout:
                log.write(line)
                log.flush()
                if ' ------------> ' in line:
                    state[0] = line.split(' ------------> ')[-1].strip()
                    event('TRANSITION ' + state[0])
                if line.startswith('rpy: '):
                    try:
                        cpp_rpy.append([float(x) for x in line[5:].split()])
                    except ValueError:
                        pass

    try:
        end = time.monotonic() + 8
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if stop.is_set() or time.monotonic() - heartbeat[0] > 1:
                raise RuntimeError('cancelled before start')
            if validate_feedback(data, time.monotonic()) is None:
                break
        else:
            raise RuntimeError('preflight: ' + str(validate_feedback(data, time.monotonic())))
        observe(1)
        if subprocess.run(['pgrep', '-x', 'rl_deploy'], stdout=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError('another AGX deployment process is already running')
        executable = root / 'install/s10_sdk_deploy/lib/s10_sdk_deploy/rl_deploy'
        argv = [str(executable), '--ros-args', '-r', '__node:=s10_sdk_isolated_deploy',
                '-r', '/STEER:=' + private + '/STEER',
                '-r', '/GAMEPAD_KEY:=' + private + '/GAMEPAD_KEY']
        if dry:
            argv += ['-r', '/JOINTS_CMD:=' + private + '/DRY_JOINTS_CMD']
        phase = 'startup'
        proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, bufsize=1, start_new_session=True)
        threading.Thread(target=output_reader, daemon=True).start()
        event('START ' + ('DRY' if dry else 'LIVE') + ' pid=' + str(proc.pid))
        end = time.monotonic() + 12
        while time.monotonic() < end:
            spin()
            if cpp_rpy and keys.get_subscription_count() and steer.get_subscription_count():
                break
        else:
            raise RuntimeError('deployment did not finish sensor initialization')
        if len(cpp_rpy[-1]) != 3 or any(abs(a-b) > .02 for a,b in zip(cpp_rpy[-1], data['imu']['values'][:3])):
            raise RuntimeError('deployment IMU differs from raw radians')
        event('CPP_IMU_VERIFIED ' + str(cpp_rpy[-1]))
        while True:
            if handset:
                if end_requested.is_set():
                    break
                stand_requested.clear()
                lie_requested.clear()
                phase = 'handset_wait'
                event('HANDSET_CONNECTED')
                while not (stand_requested.is_set() and remote.centered(time.monotonic())):
                    stand_requested.clear()
                    spin()
                    if end_requested.is_set():
                        completed = True
                        return
            if dry and not manual:
                phase = 'dry_verified'
                observe(2)
            else:
                phase = 'stand'
                if handset:
                    event('HANDSET_STANDING')
                before_stand = data['joints']['values']
                key('G20_KEY_L1')
                wait_state('standup_state', 2)
                observe(4.5)  # Official stand has two 2-second stages.
                if not dry and not legs_responded(before_stand, data['joints']['values']):
                    raise RuntimeError('Stand command produced no leg response; verify SDK control takeover on the robot')
                event('DRY_OUTPUT_ISOLATED' if dry else 'LEG_RESPONSE_DETECTED')
                if not lie_requested.is_set():
                    phase = 'rl_zero'
                    key('G20_KEY_L2')
                    wait_state('rl_control', 2)
                    observe(1 if manual else 5)
                    if manual:
                        phase = 'manual_ready'
                        if handset:
                            remote.needs_center = remote.needs_center or any(remote.axes)
                        event('MANUAL_READY')
                        while not lie_requested.is_set():
                            spin()
                if direction:
                    phase = 'lateral'
                    lateral_deadline = time.monotonic() + seconds
                    next_zero = 0.
                    event('LATERAL target_y=' + str(.5 * strength * direction) + ' m/s, ' + str(seconds) + ' seconds')
                    observe(seconds)
                    phase = 'after_lateral'
                    next_zero = 0.
                    observe(2)
                phase = 'lie'
                if handset:
                    event('HANDSET_LYING')
                next_zero = 0.
                spin()  # Publish zero before requesting lie-down.
                key('G20_KEY_R1')
                wait_state('liedown_state', 2)
                observe(4.5)
            if not handset or end_requested.is_set():
                break
        completed = True
    except Exception as exc:
        error = str(exc)
        # Capture before cleanup spins overwrite the feedback that actually tripped the guard.
        failure = dict(time=time.monotonic(), wall_time=time.time(), phase=phase,
                       state=state[0], axes=command_axes, data=copy.deepcopy(data), error=error)
        event('TRIAL_ERROR ' + error)
    finally:
        if proc is not None and proc.poll() is None:
            if not completed and not dry:
                phase = 'abort_damping'
                next_zero = 0.
                for _ in range(5):
                    key('G20_KEY_R2')
                    end = time.monotonic() + .1
                    while time.monotonic() < end:
                        spin(guard=False)
            phase = 'exit'
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=3)
        result = dict(dry=dry, manual=manual, handset=handset, direction=direction, strength=strength, seconds=seconds,
            completed=completed, error=error, final_state=state[0],
            cpp_rpy=cpp_rpy, final=data, failure=failure, events=list(events), samples=list(samples),
            joint_peaks=joint_peaks,
            diagnostic_limits=dict(leg_speed=LEG_SPEED_LIMIT, wheel_speed=WHEEL_SPEED_LIMIT,
                                   leg_torque=LEG_TORQUE_LIMIT, wheel_torque=WHEEL_TORQUE_LIMIT),
            command_scale=[1.5, .5, .6],
            pid=proc.pid if proc else None, returncode=proc.returncode if proc else None)
        (output_dir / ('result-dry.json' if dry else 'result-live.json')).write_text(json.dumps(result, indent=2))
        event('RESULT ' + json.dumps({k:v for k,v in result.items() if k not in ('samples','events','final','failure','joint_peaks')}))
        done.set()
        node.destroy_node()
        rclpy.shutdown()


def check():
    remote = HandsetInput()
    assert handset_request('{"action":"handset","limit":0.7,"enabled":true}') == (.7, True)
    assert handset_request('{"action":"zero"}') == ('zero', False)
    for packet in ('{}', '{"action":"handset","limit":NaN,"enabled":true}',
                   '{"action":"handset","limit":1.01,"enabled":true}',
                   '{"action":"handset","limit":true,"enabled":true}',
                   '{"action":"handset","limit":0.2,"enabled":1}',
                   '{"action":"move","axes":[0,0,0]}'):
        try:
            handset_request(packet)
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid handset setting accepted')
    remote.configure(.7, True)
    remote.receive((1, 0, 0), 10)
    assert remote.output(10.1, True) == (0, 0, 0)  # Must center before arming.
    remote.receive((.02, 0, 0), 10.2)
    assert remote.centered(10.3)
    remote.receive((1, -.5, -1), 10.3)
    assert remote.output(10.4, True) == (.7, -.35, -.7)
    assert remote.output(10.4, False) == (0, 0, 0)
    assert remote.output(10.7, True) == (0, 0, 0)  # Stale input never persists.
    remote.receive((1, 0, 0), 10.8)
    assert remote.output(10.8, True) == (0, 0, 0)  # Reconnect also requires center.
    remote.configure(.2, False)
    remote.receive((0, 0, 0), 11)
    remote.configure(.2, True)
    assert not remote.centered(11.1)  # Need a new centered sample after resume.
    remote.receive((0, 0, 0), 11.2)
    assert remote.centered(11.2)
    assert remote.output(30, True) == (0, 0, 0) and remote.centered(30)
    remote.configure(1., True)
    remote.receive((-1., .5, 1.), 31)
    assert remote.output(31.1, True) == (-1., .5, 1.)  # Full-range mapping preserves direction.
    assert remote.posture_key('handset_wait', 'unknown', 32) is None  # Deflected stick blocks stand.
    remote.receive((0, 0, 0), 33)
    assert remote.posture_key('handset_wait', 'unknown', 34) == 'stand'
    assert remote.posture_key('manual_ready', 'rl_control', 34.1) is None  # Duplicate C.
    assert remote.posture_key('stand', 'standup_state', 35) is None  # Transition in progress.
    assert remote.posture_key('manual_ready', 'rl_control', 40) == 'lie'
    assert remote.posture_key('lie', 'liedown_state', 41) is None
    assert remote.posture_key('handset_wait', 'liedown_state', 46) == 'stand'
    assert remote.posture_key('manual_ready', 'joint_damping', 50) is None
    for axes in ((2, 0, 0), (float('nan'), 0, 0), (0, 0)):
        try:
            remote.receive(axes, 12)
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid raw handset input accepted')
    assert manual_request('{"action":"move","axes":[0.1,0,-0.2]}') == ('move', (.1, 0, -.2))
    assert manual_request('{"action":"move","axes":[1,-1,1]}') == ('move', (1, -1, 1))
    assert manual_request('{"action":"move","axes":[-2,0,0]}') == ('move', (-2, 0, 0))
    assert manual_request('{"action":"lie"}') == ('lie', (0, 0, 0))
    for packet in ('[]', '{}', '{"action":"stand"}', '{"action":"move","axes":[0,0]}',
                   '{"action":"move","axes":[2.01,0,0]}', '{"action":"move","axes":[0,1.01,0]}',
                   '{"action":"move","axes":[0,0,1.01]}', '{"action":"move","axes":[true,0,0]}',
                   '{"action":"move","axes":[NaN,0,0]}'):
        try:
            manual_request(packet)
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid manual command accepted')
    assert leased_axes((.1,0,0), 10, 10.2, True) == (.1,0,0)
    assert leased_axes((.1,0,0), 10, 10.4, True) == (0,0,0)
    assert leased_axes((.1,0,0), 10, 10.2, False) == (0,0,0)
    still = [[0, 0, 0, 30, 1] for _ in range(16)]
    assert not legs_responded(still, [[.0002, 0, 0, 30, 1] for _ in range(16)])
    wheel_only = [row[:] for row in still]
    wheel_only[3][0] = 1
    assert not legs_responded(still, wheel_only)
    moved = [row[:] for row in still]
    moved[1][0] = .1
    assert legs_responded(still, moved)
    assert lateral_input('lateral', 10, 12, 1) == .1
    assert lateral_input('lateral', 10, 12, -1) == -.1
    assert lateral_input('lateral', 10, 11, -1, 1) == -1
    assert lateral_input('lateral', 11, 11, -1, 1) == 0
    assert lateral_input('lateral', 12, 12, 1) == 0
    assert lateral_input('after_lateral', 11, 12, 1) == 0
    assert lateral_input('abort_damping', 11, 12, 1) == 0
    data = dict(imu=dict(updated=10, values=[.09,0,0,0,0,0]),
                joints=dict(updated=10, values=[[0,0,0,40,1] for _ in range(16)]),
                hes=dict(updated=10, value=0), battery=dict(updated=10, level=80, protected=0))
    assert validate_feedback(data, 10.1) is None
    assert validate_feedback(data, 10.4) == 'imu feedback stale'
    assert validate_feedback(dict(data, imu=dict(updated=10, values=[.4,0,0,0,0,0])), 10.1)
    assert validate_feedback(dict(data, battery=dict(updated=10, level=19, protected=0)), 10.1)
    assert validate_feedback(dict(data, joints=dict(updated=10, values=[[0,0,float('nan'),40,1]]*16)), 10.1)
    for column, value, expected in ((1, -30.1, 'velocity='), (2, 12.1, 'torque='),
                                    (3, 81, 'temperature='), (4, 2, 'status=')):
        rows = copy.deepcopy(data['joints']['values'])
        rows[15][column] = value
        reason = validate_feedback(dict(data, joints=dict(updated=10, values=rows)), 10.1)
        assert reason.startswith('joint[15] hr_wheel ') and expected in reason, reason
    rows = copy.deepcopy(data['joints']['values'])
    rows[7][1] = 12.254774
    assert validate_feedback(dict(data, joints=dict(updated=10, values=rows)), 10.1) is None
    rows[0][1] = 25.77
    assert 'joint[0] fl_hipx velocity=' in validate_feedback(dict(data, joints=dict(updated=10, values=rows)), 10.1)
    peaks = {}
    record_joint_peaks(peaks, 'manual_ready', rows)
    rows[7][1] = -20
    record_joint_peaks(peaks, 'manual_ready', rows)
    rows[7][1] = 1
    record_joint_peaks(peaks, 'manual_ready', rows)
    assert peaks['manual_ready/velocity'][7] == 20
    print('SDK_SUPERVISOR_GUARD_CHECK_OK')


if __name__ == '__main__':
    if sys.argv[1:] == ['--check']:
        check()
    elif len(sys.argv) == 3 and sys.argv[2] == '--right-full':
        main(Path(sys.argv[1]), False, -1, 1., 1.)
    elif len(sys.argv) == 3 and sys.argv[2] in ('--manual', '--manual-dry'):
        main(Path(sys.argv[1]), sys.argv[2] == '--manual-dry', manual=True)
    elif len(sys.argv) == 3 and sys.argv[2] in ('--handset', '--handset-dry'):
        main(Path(sys.argv[1]), sys.argv[2] == '--handset-dry', manual=True, handset=True)
    elif len(sys.argv) == 3 and sys.argv[2] in ('--dry', '--run', '--left', '--right'):
        main(Path(sys.argv[1]), sys.argv[2] == '--dry', {'--left': 1, '--right': -1}.get(sys.argv[2], 0))
    else:
        raise SystemExit('Use --check or ROOT --dry / --run / --left / --right / --right-full / --manual / --manual-dry / --handset / --handset-dry')
