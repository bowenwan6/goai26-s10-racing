"""Bounded SDK stand/lateral trial; dry mode isolates every joint output from motors."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


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
    for position, velocity, torque, temperature, status in joints['values']:
        if not all(math.isfinite(v) for v in (position, velocity, torque, temperature)):
            return 'nonfinite joint data'
        if abs(velocity) > 12 or abs(torque) > 45 or temperature > 80 or status != 1:
            return 'joint speed/torque/temperature/status outside test bounds'
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


def main(root, dry, direction=0, strength=.1, seconds=2):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from drdds.msg import ImuData, JointsData, BatteryData, StdMsgInt32, Steer
    from std_msgs.msg import String

    output_dir = Path(os.environ.get('S10_TRIAL_OUTPUT_DIR', str(root)))
    output_dir.mkdir(parents=True, exist_ok=True)

    stop, done = threading.Event(), threading.Event()
    heartbeat = [time.monotonic()]
    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())

    def receive():
        for line in sys.stdin:
            if line.strip() != 'ping':
                break
            heartbeat[0] = time.monotonic()
        stop.set()
    threading.Thread(target=receive, daemon=True).start()

    rclpy.init()
    node = rclpy.create_node('s10_sdk_trial_supervisor')
    data, samples, events = {}, [], []
    phase = 'preflight'
    proc = None
    state = ['unknown']
    cpp_rpy = []
    error = None
    completed = False
    next_zero = 0.
    next_sample = 0.
    lateral_deadline = 0.
    command_y = 0.
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

    def battery(msg):
        if msg.data:
            data['battery'] = dict(updated=time.monotonic(),
                level=min(d.battery_level for d in msg.data),
                protected=max(d.protected_state for d in msg.data))

    subscriptions = [node.create_subscription(ImuData, '/IMU_DATA', imu, qos_profile_sensor_data),
        node.create_subscription(JointsData, '/JOINTS_DATA', joints, qos_profile_sensor_data),
        node.create_subscription(BatteryData, '/BATTERY_DATA', battery, qos_profile_sensor_data),
        node.create_subscription(StdMsgInt32, '/HES_STATUS',
            lambda m: data.update(hes=dict(updated=time.monotonic(), value=m.value)),
            qos_profile_sensor_data)]
    keys = node.create_publisher(String, private + '/GAMEPAD_KEY', 10)
    steer = node.create_publisher(Steer, private + '/STEER', 10)

    def event(text):
        events.append(dict(time=time.monotonic(), phase=phase, text=text))
        print(text, flush=True)  # Runner redirects output to a file on the AGX.

    def key(name):
        msg = String()
        msg.data = name
        keys.publish(msg)
        event('KEY ' + name)

    def spin(guard=True):
        nonlocal next_zero, next_sample, command_y
        rclpy.spin_once(node, timeout_sec=.01)
        now = time.monotonic()
        if guard:
            if stop.is_set() or now - heartbeat[0] > 1:
                raise RuntimeError('local heartbeat lost / cancelled')
            err = validate_feedback(data, now)
            if err:
                raise RuntimeError(err)
            if proc is not None and proc.poll() is not None:
                raise RuntimeError('deployment process exited')
            if state[0] == 'joint_damping':
                raise RuntimeError('deployment entered damping unexpectedly')
            if phase in ('lateral', 'after_lateral') and state[0] != 'rl_control':
                raise RuntimeError('left RL state during lateral trial')
        if now >= next_zero:
            msg = Steer()
            value = lateral_input(phase, now, lateral_deadline, direction, strength) if guard else 0.
            if value != command_y:
                event('AXIS_Y ' + str(value))
            command_y = msg.data.y = value
            msg.header.stamp = node.get_clock().now().to_msg()
            steer.publish(msg)
            next_zero = now + .1
        if now >= next_sample:
            samples.append(dict(time=now, phase=phase, state=state[0], axis_y=command_y, data=dict(data)))
            next_sample = now + .1

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
        if dry:
            phase = 'dry_verified'
            observe(2)
        else:
            phase = 'stand'
            key('G20_KEY_L1')
            wait_state('standup_state', 2)
            observe(4.5)  # Official stand has two 2-second stages.
            phase = 'rl_zero'
            key('G20_KEY_L2')
            wait_state('rl_control', 2)
            observe(5)
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
            key('G20_KEY_R1')
            wait_state('liedown_state', 2)
            observe(4.5)
        completed = True
    except Exception as exc:
        error = str(exc)
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
        result = dict(dry=dry, direction=direction, strength=strength, seconds=seconds,
            completed=completed, error=error, final_state=state[0],
            cpp_rpy=cpp_rpy, final=data, events=events, samples=samples,
            pid=proc.pid if proc else None, returncode=proc.returncode if proc else None)
        (output_dir / ('result-dry.json' if dry else 'result-live.json')).write_text(json.dumps(result, indent=2))
        event('RESULT ' + json.dumps({k:v for k,v in result.items() if k not in ('samples','events','final')}))
        done.set()
        node.destroy_node()
        rclpy.shutdown()


def check():
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
    print('SDK_SUPERVISOR_GUARD_CHECK_OK')


if __name__ == '__main__':
    if sys.argv[1:] == ['--check']:
        check()
    elif len(sys.argv) == 3 and sys.argv[2] == '--right-full':
        main(Path(sys.argv[1]), False, -1, 1., 1.)
    elif len(sys.argv) == 3 and sys.argv[2] in ('--dry', '--run', '--left', '--right'):
        main(Path(sys.argv[1]), sys.argv[2] == '--dry', {'--left': 1, '--right': -1}.get(sys.argv[2], 0))
    else:
        raise SystemExit('Use --check or ROOT --dry / --run / --left / --right / --right-full')
