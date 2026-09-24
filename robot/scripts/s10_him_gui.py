"""Manual ONNX controls: stand once, hold directions, lie down explicitly."""
import json
import math
import os
import queue
import sys
import threading
import time
import tkinter as tk
from argparse import Namespace
from tkinter import ttk

from run_s10_him_trial import run

ACTIONS = {
    'forward': ('前进 ↑', 'x', 1), 'back': ('后退 ↓', 'x', -1),
    'left': ('左移 ←', 'y', 1), 'right': ('右移 →', 'y', -1),
    'ccw': ('逆时针 ↶', 'yaw', 1), 'cw': ('顺时针 ↷', 'yaw', -1),
}

MODELS = {'HIM 1500 (aligned)': '/home/golai/s10-sdk-isolated-vbyhquwi'}


def gui():
    model_name = os.environ.get('S10_MODEL_NAME', 'HIM 1500 (aligned)')
    models = dict(MODELS)
    models[model_name] = os.environ.get('S10_SDK_TRIAL_ROOT') or models.get(model_name, MODELS['HIM 1500 (aligned)'])
    handset = os.environ.get('S10_CONTROL_SOURCE', 'handset') == 'handset'
    root = tk.Tk()
    root.title('S10 · 48号 · ' + model_name)
    root.geometry('640x760' if handset else '640x720')
    root.minsize(620, 740 if handset else 700)
    frame = ttk.Frame(root, padding=22)
    frame.pack(fill='both', expand=True)
    ttk.Label(frame, text='S10 · ' + ('遥控器控制' if handset else '手动控制'), font=('Microsoft YaHei UI', 22)).pack(anchor='w')
    description = ' · C 起身/趴下 · 起身后自动进入模型 · D 阻尼' if handset else ' · 起身后持续支撑 · 方向按住移动，松开回零'
    subtitle = tk.StringVar(value=model_name + description)
    ttk.Label(frame, textvariable=subtitle,
              wraplength=570).pack(anchor='w', pady=(6, 16))
    model_choice = tk.StringVar(value=model_name)
    model_picker = None
    if handset:
        model_row = ttk.Frame(frame)
        model_row.pack(fill='x', pady=(0, 12))
        ttk.Label(model_row, text='模型').pack(side='left', padx=(0, 8))
        model_picker = ttk.Combobox(model_row, textvariable=model_choice, values=list(models), state='readonly', width=38)
        model_picker.pack(side='left', fill='x', expand=True)

    def select_model(event=None):
        nonlocal model_name
        if busy:
            model_choice.set(model_name)
            return
        model_name = model_choice.get()
        root.title('S10 · 48号 · ' + model_name)
        subtitle.set(model_name + description)
        status.set('已选择模型 · 点击连接遥控器，回中后按 C 起身')

    if model_picker:
        model_picker.bind('<<ComboboxSelected>>', select_model)
    status = tk.StringVar(value='待连接 · 请保持遥控器 SDK 模式开启' if handset else '待起身 · 请保持遥控器 SDK 模式开启')
    telemetry = tk.StringVar(value='尚未连接')
    ttk.Label(frame, textvariable=status, font=('Microsoft YaHei UI', 13), wraplength=540).pack(anchor='w')
    ttk.Label(frame, textvariable=telemetry).pack(anchor='w', pady=(6, 12))
    inbox = queue.Queue()
    cancel, alive = threading.Event(), [time.monotonic()]
    busy = ready = closing = False
    packet = {'action': 'zero'}
    motion_buttons = {}
    speed = tk.StringVar(value='20')

    def refresh():
        if model_picker:
            model_picker.configure(state='disabled' if busy or closing else 'readonly')
        resumable = handset and busy and packet['action'] == 'zero' and not cancel.is_set()
        stand.configure(state='normal' if not closing and (not busy or resumable) else 'disabled',
                        text=('恢复摇杆' if busy else '连接遥控器') if handset else '起身')
        lie.configure(state='normal' if busy and not closing and packet['action'] != 'lie' else 'disabled')
        for button in motion_buttons.values():
            button.configure(state='normal' if ready and not closing and not handset else 'disabled')

    def zero():
        nonlocal packet
        if packet['action'] != 'lie':
            packet = {'action': 'zero'}
        if handset and busy and packet['action'] != 'lie':
            status.set('摇杆已暂停 · 点击「恢复摇杆」后先回中')
            refresh()
        elif ready:
            status.set('支撑中 · 速度输入已回零')

    def input_amount():
        try:
            amount = float(speed.get()) / 100
            if not math.isfinite(amount) or not .01 <= amount <= 1:
                raise ValueError
        except ValueError:
            zero()
            status.set('输入比例请填写 1–100 之间的数字')
            return None
        return amount

    def move(action):
        nonlocal packet
        if not ready or closing or handset:
            return
        amount = input_amount()
        if amount is None:
            return
        axis, direction = ACTIONS[action][1:]
        axes = [0., 0., 0.]
        axes[('x', 'y', 'yaw').index(axis)] = direction * amount
        packet = {'action': 'move', 'axes': axes}
        status.set(ACTIONS[action][0] + ' · 松开停止移动')

    def lie_down():
        nonlocal packet, ready
        if not busy:
            return
        packet = {'action': 'lie'}
        ready = False
        status.set('正在趴下…')
        refresh()

    def abort():
        nonlocal ready
        zero()
        ready = False
        if busy:
            cancel.set()
            status.set('正在请求阻尼并退出…')
        refresh()

    def start():
        nonlocal busy, ready, packet
        if handset and not closing:
            amount = input_amount()
            if amount is None:
                return
            if busy:
                if packet['action'] == 'zero' and not cancel.is_set():
                    packet = {'action': 'handset', 'limit': amount, 'enabled': True}
                    status.set('摇杆已恢复 · 请先回中')
                    refresh()
                return
        if busy or closing:
            return
        busy, ready = True, False
        packet = {'action': 'handset', 'limit': amount, 'enabled': True} if handset else {'action': 'zero'}
        cancel.clear()
        alive[0] = time.monotonic()
        status.set('正在连接遥控器 · 等待按键，不自动起身' if handset else '正在连接并起身…')
        refresh()
        args = Namespace(host=os.environ.get('S10_SSH_HOST', 's10-48-golai'),
            user=os.environ.get('S10_SSH_USER') or None, host_alias=None,
            source_install='/home/golai/s10_control_ws/install',
            trial_root=models[model_name],
            mode='handset' if handset else 'manual')

        def job():
            try:
                result = run(args, cancel, alive, lambda text: inbox.put(('log', text)),
                             command_source=lambda: packet)
                inbox.put(('finished', ('会话已结束 · 点击连接遥控器可再起身' if handset else '已趴下 · 可再次起身') if result else '已取消'))
            except Exception as exc:
                inbox.put(('finished', '控制已结束：' + str(exc)))
            finally:
                inbox.put(('done', ''))
        threading.Thread(target=job, daemon=True).start()

    posture = ttk.Frame(frame)
    posture.pack(fill='x', pady=(0, 12))
    stand = ttk.Button(posture, text='起身', command=start)
    lie = ttk.Button(posture, text='趴下并断开 / 换模型' if handset else '趴下', command=lie_down)
    for button in (stand, lie):
        button.pack(side='left', fill='x', expand=True, padx=4, ipady=12)
    pad = ttk.Frame(frame)
    pad.pack(fill='x')
    positions = {'ccw': (0, 0), 'forward': (0, 1), 'cw': (0, 2),
                 'left': (1, 0), 'back': (1, 1), 'right': (1, 2)}
    labels = {'ccw': '左转 ↶', 'cw': '右转 ↷'}
    for action, (row, col) in positions.items():
        button = ttk.Button(pad, text=labels.get(action, ACTIONS[action][0]))
        button.grid(row=row, column=col, sticky='ew', padx=4, pady=5, ipady=15)
        if not handset:
            button.bind('<ButtonPress-1>', lambda e, a=action: move(a))
            button.bind('<ButtonRelease-1>', lambda e: zero())
            button.bind('<Leave>', lambda e: zero())
            # Keyboard users can hold Return on the focused direction button.
            button.bind('<KeyPress-Return>', lambda e, a=action: move(a))
            button.bind('<KeyRelease-Return>', lambda e: zero())
            button.bind('<FocusOut>', lambda e: zero())
        motion_buttons[action] = button
    for col in range(3):
        pad.columnconfigure(col, weight=1)
    options = ttk.Frame(frame)
    options.pack(fill='x', pady=12)
    ttk.Label(options, text='摇杆上限' if handset else '输入比例').pack(side='left')
    ttk.Spinbox(options, textvariable=speed, from_=1, to=100,
                increment=5, width=6).pack(side='left', padx=8)
    ttk.Label(options, text='% · 回中 0%，半程 50%，满程 100%（上限100时）' if handset else '%（1–100，可直接填写） · 按狗自身方向移动',
              wraplength=400).pack(side='left')
    ttk.Button(frame, text='暂停摇杆 / 保持支撑  [空格]' if handset else '停止移动 / 保持支撑  [空格]', command=zero).pack(fill='x', ipady=7)
    tk.Button(frame, text='中止控制 / 阻尼', command=abort, bg='#a92121', fg='white',
              activebackground='#831818', activeforeground='white').pack(fill='x', pady=(8, 7), ipady=4)
    ttk.Label(frame, text=('C 可反复起身/趴下；换模型先点「趴下并断开」。保持电脑和窗口运行，电脑或反馈断线退出到阻尼。' if handset else '正常结束请点「趴下」。阻尼或断线退出不能保证站立，手柄保持可接管。'),
              wraplength=540).pack(anchor='w')
    output = tk.Text(frame, height=5, wrap='word', state='disabled')
    output.pack(fill='both', expand=True, pady=(12, 0))

    def note(text):
        output.configure(state='normal')
        output.insert('end', time.strftime('%H:%M:%S ') + text + '\n')
        if int(output.index('end-1c').split('.')[0]) > 160:
            output.delete('1.0', '2.0')
        output.see('end')
        output.configure(state='disabled')

    def tick():
        nonlocal busy, ready, packet
        alive[0] = time.monotonic()
        if handset and busy and packet['action'] == 'handset':
            amount = input_amount()
            if amount is not None:
                packet = dict(packet, limit=amount)
        while not inbox.empty():
            kind, text = inbox.get()
            if kind == 'done':
                busy, ready, packet = False, False, {'action': 'zero'}
                refresh()
            elif kind == 'finished':
                status.set(text)
                note(text)
            elif text.startswith('STATUS '):
                value = json.loads(text[7:])
                telemetry.set(f"电量 {value['battery']}% · 侧倾 {value['rpy'][0]*57.2958:.1f}° · 俯仰 {value['rpy'][1]*57.2958:.1f}°")
                if handset:
                    age = value.get('handset_age')
                    telemetry.set(telemetry.get() + (' · 遥控信号未收到' if age is None else f' · 摇杆消息 {age:.1f}s 前'))
            elif text == 'HANDSET_CONNECTED':
                ready = False
                status.set('等待 C 起身 · 先将摇杆回中 · 起身后自动进入模型')
                refresh()
            elif text in ('HANDSET_STANDING', 'HANDSET_LYING'):
                ready = False
                status.set('正在起身 · 完成后自动进入模型' if text == 'HANDSET_STANDING' else '正在趴下 · 完成后可再按 C 起身')
                refresh()
            elif text == 'MANUAL_READY':
                ready = not cancel.is_set() and packet['action'] != 'lie' and not closing
                if ready:
                    status.set('模型已就绪 · 摇杆控制 · 再按 C 趴下，D 阻尼' if handset else '支撑中 · 按住方向按钮移动')
                refresh()
            else:
                note(text)
        if closing and not busy:
            root.destroy()
        else:
            root.after(50, tick)

    def focus_lost():
        if not handset and root.focus_displayof() is None:
            zero()

    def close():
        nonlocal closing
        closing = True
        abort()
        if not busy:
            root.destroy()

    if not handset:
        root.bind('<ButtonRelease-1>', lambda e: zero())
    # Handle Space before Tk's button bindings, which would otherwise invoke a focused button.
    def space_stop(event):
        zero()
        return 'break'

    def bind_space(widget):
        widget.bindtags(('S10Stop', *widget.bindtags()))
        for child in widget.winfo_children():
            bind_space(child)

    root.bind_class('S10Stop', '<KeyPress-space>', space_stop)
    root.bind_class('S10Stop', '<KeyRelease-space>', space_stop)
    bind_space(root)
    root.bind('<FocusOut>', lambda e: root.after_idle(focus_lost))
    root.protocol('WM_DELETE_WINDOW', close)
    refresh()
    tick()
    if '--ui-check' in sys.argv:
        root.update()
        assert all(b.instate(['disabled']) and b.winfo_width() > 100 for b in motion_buttons.values())
        if handset:
            for name in models:
                model_choice.set(name)
                select_model()
                assert name in root.title() and models[model_name] == models[name]
            busy = ready = True  # No SSH or motor commands in this UI check.
            start()
            assert model_picker.instate(['disabled'])
            active_model = model_name
            model_choice.set('Turn Stop 2000 (research)')
            select_model()
            assert model_name == active_model and model_choice.get() == active_model
            assert packet == {'action': 'handset', 'limit': .2, 'enabled': True}
            move('forward')
            assert packet['action'] == 'handset'
            zero()
            assert packet == {'action': 'zero'} and stand.instate(['!disabled'])
            speed.set('70')
            start()
            assert packet['limit'] == .7
            speed.set('NaN')
            tick()
            assert packet == {'action': 'zero'}
            lie_down()
            zero()
            start()
            assert packet == {'action': 'lie'}
            busy = False
            root.destroy()
            print('HANDSET_GUI_CHECK_OK')
            return
        move('forward')
        assert packet == {'action': 'zero'}  # No movement before a session is ready.
        busy = ready = True  # UI-only check: never start SSH or the robot.
        refresh()
        invoked = []
        stand.configure(command=lambda: invoked.append(True), state='normal')
        stand.focus_force()
        root.update()
        stand.event_generate('<KeyPress-space>')
        stand.event_generate('<KeyRelease-space>')
        assert not invoked  # Global stop must never trigger the focused posture button.
        refresh()
        for action, button in motion_buttons.items():
            button.event_generate('<ButtonPress-1>')
            assert packet['action'] == 'move' and max(map(abs, packet['axes'])) == .2
            button.event_generate('<ButtonRelease-1>')
            assert packet == {'action': 'zero'}
        move('left')
        motion_buttons['left'].event_generate('<Leave>')
        assert packet == {'action': 'zero'}
        speed.set('100')
        move('forward')
        assert packet['axes'] == [1., 0., 0.]
        for invalid in ('', 'abc', 'NaN', 'inf', '0', '101'):
            speed.set(invalid)
            move('forward')
            assert packet == {'action': 'zero'}
        speed.set('20')
        lie_down()
        zero()
        move('forward')
        assert packet == {'action': 'lie'} and not ready
        assert output.winfo_height() >= 70
        busy = False
        print('MANUAL_GUI_CHECK_OK')
        root.after(100, close)
    elif '--connect' in sys.argv and handset:
        root.after(300, start)
    root.mainloop()


if __name__ == '__main__':
    gui()
