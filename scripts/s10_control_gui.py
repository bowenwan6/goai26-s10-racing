"""Windows S10 bench controls; --check is offline, --worker runs only on the AGX."""
import json
import math
import os
from pathlib import Path
import queue
import shlex
import sys
import threading
import time


ACTIONS = {
    'forward': ('前进 ↑', 'x', 1), 'back': ('后退 ↓', 'x', -1),
    'left': ('左移 ←', 'y', 1), 'right': ('右移 →', 'y', -1),
    'ccw': ('逆时针 ↶', 'yaw', 1), 'cw': ('顺时针 ↷', 'yaw', -1),
}
MAX_INPUT = 1.0
MAX_DURATION = 5
LINEAR_LIMIT = .40
ANGULAR_LIMIT = .50
SDK_MODES = {
    'sdk_dry': ('连接检查（不驱动电机）', 'dry'),
    'sdk_stand': ('起身 → 支撑 5 秒 → 趴下', 'stand'),
    'sdk_left': ('左移测试 · 0.05 m/s · 2 秒', 'left'),
    'sdk_right': ('右移测试 · 0.05 m/s · 2 秒', 'right'),
}


def validate(action, strength, duration):
    if action not in {*ACTIONS, 'read', 'stand', 'lie', 'zero'}:
        raise ValueError('Unknown action')
    if not math.isfinite(strength) or not 0 < strength <= MAX_INPUT:
        raise ValueError('输入比例须大于 0 且不超过 100%')
    if not math.isfinite(duration) or not 0 < duration <= MAX_DURATION:
        raise ValueError('持续时间须大于 0 且不超过 5 秒')


def feedback_error(s, now, states, standing=False):
    if not s or now - s['received'] > .5:
        return 'Motion feedback stale'
    if s['state'] not in states:
        return 'Unexpected motion state'
    if not all(math.isfinite(s[k]) for k in ('height', 'vx', 'vy', 'yaw')):
        return 'Nonfinite feedback'
    if standing and not .30 <= s['height'] <= .55:
        return 'Standing height outside test range'
    if standing and (math.hypot(s['vx'], s['vy']) > LINEAR_LIMIT or abs(s['yaw']) > ANGULAR_LIMIT):
        return '速度反馈超过测试阈值（平移 0.40 m/s 或旋转 0.50 rad/s），结束本轮'
    return None


def worker(action, strength, duration):
    """Bounded ROS operation. EOF, missing GUI heartbeat, and STOP all cancel it."""
    import fcntl
    import signal
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from drdds.msg import BatteryData, MotionInfo, MotionState, StdMsgInt32, Steer

    validate(action, strength, duration)
    lock = open('/tmp/s10-gui-control.lock', 'a')
    if action != 'read':
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    latest, own, rows = {}, set(), []
    stopped = threading.Event()
    signals = (signal.SIGTERM, signal.SIGHUP, signal.SIGINT)
    previous_signals = {s: signal.signal(s, lambda *_: stopped.set()) for s in signals}
    heartbeat = [time.monotonic()]
    takeover = [False]
    sent_axes = False
    last_report = 0.
    phase = 'initial'
    frame = time.time_ns() % (2**63)

    def receive():
        for line in sys.stdin:
            if line.strip() == 'ping':
                heartbeat[0] = time.monotonic()
            else:
                break
        stopped.set()

    threading.Thread(target=receive, daemon=True).start()
    rclpy.init()
    node = rclpy.create_node('s10_gui_bounded_control')

    def motion(msg):
        d = msg.data
        latest['motion'] = dict(received=time.monotonic(), state=d.motion_state.state,
                                height=d.height, vx=d.vel_x, vy=d.vel_y, yaw=d.vel_yaw,
                                phase=phase)
        rows.append(latest['motion'])

    def battery(msg):
        if msg.data:
            latest['battery'] = (time.monotonic(), min(d.battery_level for d in msg.data),
                                 max(d.protected_state for d in msg.data))

    def axes(msg):
        if msg.header.frame_id not in own:
            values = [getattr(msg.data, k) for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')]
            if any(not math.isfinite(v) or abs(v) > .001 for v in values):
                takeover[0] = True

    _subscriptions = [
        node.create_subscription(MotionInfo, '/MOTION_INFO', motion, qos_profile_sensor_data),
        node.create_subscription(BatteryData, '/BATTERY_DATA', battery, qos_profile_sensor_data),
        node.create_subscription(StdMsgInt32, '/HES_STATUS',
            lambda m: latest.update(hes=(time.monotonic(), m.value)), qos_profile_sensor_data),
        node.create_subscription(Steer, '/STEER', axes, qos_profile_sensor_data),
    ]
    steer = node.create_publisher(Steer, '/STEER', 10)
    modes = node.create_publisher(MotionState, '/MOTION_STATE', 10)

    def emit(kind, **data):
        try:
            print(json.dumps(dict(kind=kind, **data)), flush=True)
        except (BrokenPipeError, OSError):
            stopped.set()  # Logging failure must not prevent the zero-input path.

    def spin():
        nonlocal last_report
        rclpy.spin_once(node, timeout_sec=.02)
        now = time.monotonic()
        if now - last_report > .25:
            emit('status', motion=latest.get('motion'), battery=latest.get('battery'),
                 hes=latest.get('hes'))
            last_report = now

    def guard(states, standing=False):
        now = time.monotonic()
        if stopped.is_set() or now - heartbeat[0] > 1.:
            raise RuntimeError('Stopped / GUI heartbeat lost')
        if takeover[0]:
            raise RuntimeError('External controller input; yielding control')
        err = feedback_error(latest.get('motion'), now, states, standing)
        ht, hes = latest.get('hes', (0, None))
        bt, level, protection = latest.get('battery', (0, 0, 0))
        if err or now - ht > .5 or hes != 0:
            raise RuntimeError(err or 'Emergency feedback stale or active')
        if now - bt > 3 or level < 20 or protection:
            raise RuntimeError('Battery low, protected, or feedback stale')

    def publish(pub, msg):
        nonlocal frame
        frame += 1
        own.add(frame)
        msg.header.frame_id = frame
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    def observe(seconds, states, standing=False):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            spin()
            guard(states, standing)

    def mode(state):
        msg = MotionState()
        msg.data.state = state
        publish(modes, msg)
        emit('event', text=f'State request: {state}')

    error = None
    try:
        end = time.monotonic() + 8
        while time.monotonic() < end:
            spin()
            if stopped.is_set() or time.monotonic() - heartbeat[0] > 1:
                raise RuntimeError('Cancelled before command')
            if all(k in latest for k in ('motion', 'hes', 'battery')):
                if action == 'read' or (steer.get_subscription_count() and modes.get_subscription_count()):
                    break
        else:
            raise RuntimeError('Required feedback / subscribers unavailable')
        if action == 'read':
            emit('status', motion=latest.get('motion'), battery=latest.get('battery'),
                 hes=latest.get('hes'))
        elif action == 'zero':
            sent_axes = True  # Explicit zero input uses the same finally block.
        else:
            guard({0, 1, 3, 4, 17})
            if max(abs(latest['motion'][k]) for k in ('vx', 'vy', 'yaw')) > .04:
                raise RuntimeError('Robot is not initially stationary')
            if action == 'stand':
                if latest['motion']['state'] in {0, 3, 4}:
                    mode(1)
                    observe(2.5, {0, 1, 3, 4})
                    guard({1})
                if latest['motion']['state'] == 1:
                    mode(17)
                    observe(2.5, {1, 17})
                observe(2, {17}, True)
            elif action == 'lie':
                guard({1, 17})
                mode(4)
                observe(4, {0, 1, 4, 17})
                guard({0, 4})
            else:
                phase = 'baseline'
                observe(.5, {17}, True)
                axis, direction = ACTIONS[action][1:]
                phase = 'move'
                emit('event', text=f'{ACTIONS[action][0]}：输入 {strength * 100:g}%，持续 {duration:g} 秒')
                end, next_send = time.monotonic() + duration, time.monotonic()
                while time.monotonic() < end:
                    spin()
                    guard({17}, True)
                    if time.monotonic() >= end:
                        break
                    if time.monotonic() >= next_send:
                        msg = Steer()
                        setattr(msg.data, axis, direction * strength)
                        sent_axes = True
                        publish(steer, msg)
                        next_send = time.monotonic() + .1
    except Exception as exc:
        error = str(exc)
        emit('error', text=error)
    finally:
        if sent_axes and not takeover[0]:
            phase = 'zero'
            emit('event', text='正在回零：本轮运动输入已结束，不代表此前移动已生效')
            for _ in range(10):
                publish(steer, Steer())
                end = time.monotonic() + .1
                while time.monotonic() < end:
                    spin()
                if takeover[0]:
                    break
            # Observe without the cancelled heartbeat; zero has already been sent.
            phase = 'after'
            end = time.monotonic() + 1
            while time.monotonic() < end:
                spin()
        if action in ACTIONS:
            key = {'x': 'vx', 'y': 'vy', 'yaw': 'yaw'}[ACTIONS[action][1]]
            baseline = [s[key] for s in rows if s['phase'] == 'baseline']
            movement = [s[key] for s in rows if s['phase'] == 'move']
            if baseline and movement:
                delta = sum(movement) / len(movement) - sum(baseline) / len(baseline)
                unit = 'rad/s' if key == 'yaw' else 'm/s'
                small = abs(delta) < (.03 if key == 'yaw' else .015)
                emit('event', text=f'相对起始反馈的速度变化：{delta:+.3f} {unit}。'
                     + ('响应较小，尚不能确认动作生效。' if small else '请结合现场观察确认动作。'))
        emit('result', action=action, error=error, motion=latest.get('motion'),
             takeover=takeover[0], zero_sent=sent_axes and not takeover[0], samples=rows)
        node.destroy_node()
        rclpy.shutdown()
        lock.close()
        for sig, handler in previous_signals.items():
            signal.signal(sig, handler)


def run_remote(host, user, password, setup, action, strength, duration, cancel, alive, report):
    """One SSH job; the robot-side worker owns the timer and stop logic."""
    import paramiko
    validate(action, strength, duration)
    client = paramiko.SSHClient()
    known = paramiko.HostKeys(str(Path.home() / '.ssh/known_hosts'))
    config = paramiko.SSHConfig()
    config_path = Path.home() / '.ssh/config'
    if config_path.is_file():
        with config_path.open(encoding='utf-8') as stream:
            config.parse(stream)
    target = config.lookup(host)
    hostname = target.get('hostname', host)
    alias = target.get('hostkeyalias', hostname)
    trusted = known.lookup(alias)  # Trust only this robot's alias, not the reused IP's old key.
    if not trusted:
        raise ValueError(f'请先用 ssh {host} 核验该设备的主机密钥（{alias}）')
    port = int(target.get('port', 22))
    key_host = hostname if port == 22 else f'[{hostname}]:{port}'
    for kind, key in list(trusted.items()):
        client.get_host_keys().add(key_host, kind, key)
    done = threading.Event()
    log = []
    try:
        client.connect(hostname, port=port, username=user or target.get('user'),
                       password=password or None, key_filename=target.get('identityfile'), timeout=6,
                       auth_timeout=6, banner_timeout=6)
        if cancel.is_set():
            return
        remote_file = '/tmp/s10-gui-' + str(time.time_ns()) + '.py'
        command = 'umask 077; set -C; cat > ' + shlex.quote(remote_file)
        stdin, stdout, stderr = client.exec_command(command)
        stdin.write(Path(__file__).read_text(encoding='utf-8'))
        stdin.channel.shutdown_write()
        if stdout.channel.recv_exit_status():
            raise RuntimeError(stderr.read().decode())
        if cancel.is_set():
            return
        command = ('source /opt/ros/jazzy/setup.bash && source ' + shlex.quote(setup)
                   + ' && export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp && '
                   + 'if [ -f "$HOME/.ros/fastdds_ethernet.xml" ]; then '
                   + 'export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_ethernet.xml"; fi && '
                   + f'exec python3 {shlex.quote(remote_file)} --worker {action} {strength} {duration}')
        stdin, stdout, stderr = client.exec_command('bash -lc ' + shlex.quote(command))

        def heartbeat():
            try:
                while not done.is_set():
                    if cancel.is_set() or time.monotonic() - alive[0] > 1:
                        stdin.write('stop\n')
                        stdin.flush()
                        return
                    stdin.write('ping\n')
                    stdin.flush()
                    done.wait(.2)
            except (OSError, EOFError, paramiko.SSHException):
                pass  # Remote heartbeat timeout stops the operation.

        threading.Thread(target=heartbeat, daemon=True).start()
        received_result = False
        for line in stdout:
            log.append(line)
            try:
                value = json.loads(line)
            except ValueError:
                continue
            received_result |= value.get('kind') == 'result'
            report(value)
        err = stderr.read().decode()
        if stdout.channel.recv_exit_status() or not received_result:
            raise RuntimeError(err or 'Connection ended without final feedback; use remote controller')
    finally:
        done.set()
        client.close()
        if log:
            folder = Path(__file__).resolve().parents[1] / 'artifacts/s10-control/gui'
            folder.mkdir(parents=True, exist_ok=True)
            (folder / f'{time.time_ns()}-{action}.jsonl').write_text(''.join(log), encoding='utf-8')


def gui(sdk=False):
    import tkinter as tk
    from tkinter import ttk
    root = tk.Tk()
    root.title('S10 · 48号 · 自定义 ONNX speedturn2000' if sdk else 'S10 · 48号机器人控制 · 100% / 5秒')
    root.geometry('700x640')
    root.minsize(680, 620)
    frame = ttk.Frame(root, padding=20)
    frame.pack(fill='both', expand=True)
    ttk.Label(frame, text='S10 · speedturn2000 测试' if sdk else 'S10 低速控制', font=('Microsoft YaHei UI', 20)).pack(anchor='w')
    ttk.Label(frame, text='自定义 ONNX · 先连接检查，再站立测试 · 启动窗口不会驱动电机' if sdk else
              '保持手柄可接管 · 按机身方向移动 · 每次点击仅执行一轮').pack(anchor='w', pady=(4, 14))
    fields = ttk.Frame(frame)
    fields.pack(fill='x')
    host = tk.StringVar(value=os.environ.get('S10_SSH_HOST', 's10-48-golai'))
    user = tk.StringVar(value=os.environ.get('S10_SSH_USER', 'golai'))
    password = tk.StringVar(value=os.environ.pop('S10_SSH_PASSWORD', ''))
    for i, (label, variable, width) in enumerate((('AGX', host, 18), ('用户', user, 8), ('密码', password, 12))):
        if sdk and label == '密码':
            continue  # SDK launcher uses the configured OpenSSH key.
        ttk.Label(fields, text=label).grid(row=0, column=i*2, padx=(0, 5))
        ttk.Entry(fields, textvariable=variable, width=width, show='*' if i == 2 else '').grid(row=0, column=i*2+1, padx=(0, 12))
    setup = os.environ.get('S10_GUI_ROS_SETUP', '/home/golai/s10_control_ws/install/setup.bash')
    trial_root = os.environ.get('S10_SDK_TRIAL_ROOT', '/home/golai/s10-sdk-isolated-xly21vwi')
    status = tk.StringVar(value='尚未读取状态；启动 GUI 不会自动发运动指令。')
    ttk.Label(frame, textvariable=status, wraplength=640, font=('Microsoft YaHei UI', 11)).pack(anchor='w', pady=16)
    controls = ttk.Frame(frame)
    if not sdk:
        controls.pack(fill='x')
    strength, duration = tk.StringVar(value='3'), tk.StringVar(value='1')
    ttk.Label(controls, text='输入比例（%）').pack(side='left')
    ttk.Combobox(controls, textvariable=strength, values=('1', '2', '3', '5', '8', '10', '15', '20', '30', '40', '50', '60', '70', '80', '90', '100'), state='readonly', width=5).pack(side='left', padx=8)
    ttk.Label(controls, text='持续（秒）').pack(side='left', padx=(12, 0))
    ttk.Combobox(controls, textvariable=duration, values=('1', '2', '3', '4', '5'), state='readonly', width=5).pack(side='left', padx=8)
    ttk.Label(frame, text='每个运动测试都从趴稳开始，自动起身、测试、趴下；移动前先确认站立稳定。' if sdk else
              '100% 为满量程；反馈超过平移 0.40 m/s / 旋转 0.50 rad/s 会提前回零。', wraplength=640).pack(anchor='w', pady=(8, 10))
    pad = ttk.Frame(frame)
    pad.pack(fill='x')
    buttons, inbox = [], queue.Queue()
    busy, closing = [False], [False]
    cancel, alive = threading.Event(), [time.monotonic()]
    output = tk.Text(frame, height=7, wrap='word', state='disabled')

    def note(text):
        output.configure(state='normal')
        output.insert('end', time.strftime('%H:%M:%S ') + text + '\n')
        output.see('end')
        output.configure(state='disabled')

    def start(action):
        if busy[0] or closing[0]:
            return
        try:
            amount, seconds = float(strength.get()) / 100, float(duration.get())
            if action not in SDK_MODES:
                validate(action, amount, seconds)
        except ValueError as exc:
            note(str(exc))
            return
        credentials = (host.get().strip(), user.get().strip(), password.get())
        busy[0] = True
        cancel.clear()
        for button in buttons:
            button.configure(state='disabled')
        note('读取并检查反馈…' if action != 'read' else '读取状态…')

        def job():
            try:
                if action in SDK_MODES:
                    from argparse import Namespace
                    from run_s10_sdk_trial import run
                    args = Namespace(host=credentials[0], user=credentials[1], host_alias=None,
                                     source_install=str(Path(setup).parent).replace('\\', '/'),
                                     trial_root=trial_root, mode=SDK_MODES[action][1])
                    run(args, cancel, alive, lambda text: inbox.put(dict(kind='event', text=text)))
                    inbox.put(dict(kind='result', error=None))
                else:
                    run_remote(*credentials, setup, action, amount, seconds, cancel, alive, inbox.put)
            except Exception as exc:
                message = str(exc).replace('Stand command produced no leg response; verify SDK control takeover on the robot',
                                          '起身指令已发出，但腿没有响应：请检查本体 SDK 接管状态；本轮已中止，未进入 RL。')
                inbox.put(dict(kind='error', text=message))
            finally:
                inbox.put(dict(kind='done'))
        threading.Thread(target=job, daemon=True).start()

    def stop():
        if busy[0]:
            cancel.set()
            note('正在中止；SDK 测试将请求阻尼并退出，请用手柄接管。' if sdk else '正在中止；等待机器人端回零反馈。')
        elif sdk:
            note('本窗口没有正在运行的测试；需要停止本体时请用现场手柄。')
        else:
            start('zero')

    positions = {'ccw': (0, 0), 'forward': (0, 1), 'cw': (0, 2),
                 'left': (1, 0), 'back': (1, 1), 'right': (1, 2)}
    actions = {key: value[0] for key, value in (SDK_MODES if sdk else ACTIONS).items()}
    if sdk:
        positions = {key: divmod(i, 2) for i, key in enumerate(SDK_MODES)}
    for action, label in actions.items():
        button = ttk.Button(pad, text=label, command=lambda a=action: start(a))
        button.grid(row=positions[action][0], column=positions[action][1], sticky='ew', padx=4, pady=5, ipady=9)
        buttons.append(button)
    for col in range(2 if sdk else 3):
        pad.columnconfigure(col, weight=1)
    modes = ttk.Frame(frame)
    modes.pack(fill='x', pady=8)
    mode_buttons = (('读取本体状态 / 电量', 'read'),) if sdk else (('刷新状态', 'read'), ('站立 → RL 支撑', 'stand'), ('趴下', 'lie'))
    for label, action in mode_buttons:
        button = ttk.Button(modes, text=label, command=lambda a=action: start(a))
        button.pack(side='left', expand=True, fill='x', padx=4)
        buttons.append(button)
    tk.Button(frame, text='中止本轮 SDK 测试 / 请求阻尼  [空格]' if sdk else '停止本轮运动 / 速度回零  [空格]', bg='#a92121', fg='white',
              activebackground='#831818', activeforeground='white', font=('Microsoft YaHei UI', 12),
              command=stop).pack(fill='x', pady=(4, 8), ipady=6)
    ttk.Label(frame, text='手柄保持可接管；阻尼不等于趴下或硬急停。\n此窗口运行 AGX 自定义模型，SDK 开关仍由现场操作；退出测试不会关闭 SDK。' if sdk else
              '回零不等于硬急停；模式切换已生效时不会自动撤销。\nSDK 授权尚未核实；本界面不切换 SDK，也不启动 AGX 比赛策略。', wraplength=640).pack(anchor='w')
    output.pack(fill='both', expand=True, pady=(10, 0))

    def tick():
        alive[0] = time.monotonic()
        while not inbox.empty():
            item = inbox.get()
            kind = item['kind']
            if kind == 'status' and item.get('motion'):
                m, b = item['motion'], item.get('battery')
                state_name = {0: '空闲', 1: '起身', 2: '阻尼', 3: '上电阻尼', 4: '趴下', 17: 'RL'}.get(m['state'], '未知')
                level = str(b[1]) + '%' if b else '未知'
                status.set(('本体接口反馈（不代表自定义 ONNX 状态）\n' if sdk else '') + f"状态 {m['state']} · {state_name}    电量 {level}    高度 {m['height']*100:.1f} cm\n"
                           f"前向 {m['vx']:.3f} / 横向 {m['vy']:.3f} m/s    转向 {m['yaw']:.3f} rad/s\n"
                           + time.strftime('采样于 %H:%M:%S（非持续监控）'))
            elif kind in ('error', 'event'):
                note(item['text'])
            elif kind == 'result':
                note('本轮结束：' + (item.get('error') or '已完成；请结合现场观察'))
            elif kind == 'done':
                busy[0] = False
                for button in buttons:
                    button.configure(state='normal')
        if closing[0] and not busy[0]:
            root.destroy()
            return
        root.after(100, tick)

    def close():
        closing[0] = True
        if busy[0]:
            cancel.set()
            note('正在结束本轮，等待远端退出后关闭窗口。')
        else:
            root.destroy()
    root.bind('<space>', lambda e: stop() if root.focus_get() is None or root.focus_get().winfo_class() != 'TEntry' else None)
    root.protocol('WM_DELETE_WINDOW', close)
    tick()
    if '--ui-check' in sys.argv:
        root.update()
        assert all(b.winfo_width() > 50 for b in buttons)
        assert output.winfo_height() >= 70
        root.after(300, root.destroy)
    elif password.get() and not sdk:
        root.after(500, lambda: start('read'))
    root.mainloop()


def check():
    validate('forward', .03, 3)
    validate('left', MAX_INPUT, MAX_DURATION)
    for values in [('forward', 1.01, 1), ('left', 0, 1), ('left', .03, 6), ('cw', float('nan'), 1), ('sdk', .03, 1)]:
        try:
            validate(*values)
        except ValueError:
            pass
        else:
            raise AssertionError(values)
    s = dict(received=10, state=17, height=.4, vx=0., vy=0., yaw=0.)
    assert feedback_error(s, 10.1, {17}, True) is None
    for changed in (dict(state=4), dict(height=.2), dict(vx=.41), dict(vx=.3, vy=.3),
                    dict(yaw=.51), dict(yaw=float('nan')), dict(received=9)):
        assert feedback_error(dict(s, **changed), 10.1, {17}, True)
    assert ACTIONS['left'][1:] == ('y', 1) and ACTIONS['cw'][1:] == ('yaw', -1)
    print('GUI_CHECK_OK')


if __name__ == '__main__':
    if sys.argv[1:2] == ['--worker']:
        worker(sys.argv[2], float(sys.argv[3]), float(sys.argv[4]))
    elif sys.argv[1:] == ['--check']:
        check()
    elif '--sdk' in sys.argv:
        from s10_sdk_gui import gui as sdk_gui
        sdk_gui()
    else:
        gui()
