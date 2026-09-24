"""Offline checks of actual worker publication/stop paths; no robot connection."""
import contextlib
import io
import json
import sys
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, mock_open, patch

from s10_control_gui import check, run_remote, worker


def remote_login():
    """Exercise the SSH-alias/key route and make old-account dependencies fail offline."""
    client = MagicMock()
    keys = MagicMock()
    keys.lookup.return_value = {'ssh-ed25519': 'verified-48-key'}
    config = MagicMock()
    config.lookup.return_value = dict(hostname='10.21.33.102', hostkeyalias='s10-48-agx',
                                     identityfile=['team-key'])
    upload, output, error = MagicMock(), MagicMock(), MagicMock()
    upload.channel.recv_exit_status.return_value = 0
    output.channel.recv_exit_status.return_value = 0
    error.read.return_value = b''
    # Empty SSH output must fail, without creating an unrelated test log file.
    output.__iter__.return_value = iter(())
    client.exec_command.side_effect = [(MagicMock(), upload, error), (MagicMock(), output, error)]
    module = NS(SSHClient=lambda: client, SSHConfig=lambda: config, HostKeys=lambda _: keys,
                SSHException=OSError)
    with patch.dict(sys.modules, {'paramiko': module}):
        try:
            run_remote('s10-48-golai', 'golai', '', '/home/golai/s10_control_ws/install/setup.bash',
                       'read', .03, 1, threading.Event(), [time.monotonic()], lambda _: None)
        except RuntimeError as exc:
            assert 'without final feedback' in str(exc)
        else:
            raise AssertionError('Missing SSH result must fail')
    keys.lookup.assert_called_once_with('s10-48-agx')
    client.get_host_keys().add.assert_called_once_with('10.21.33.102', 'ssh-ed25519', 'verified-48-key')
    assert client.connect.call_args.args == ('10.21.33.102',)
    assert client.connect.call_args.kwargs['username'] == 'golai'
    assert client.connect.call_args.kwargs['key_filename'] == ['team-key']
    commands = [call.args[0] for call in client.exec_command.call_args_list]
    assert all('xwy' not in command and 'sudo' not in command for command in commands)
    assert 'source /home/golai/s10_control_ws/install/setup.bash &&' in commands[1]
    client.close.assert_called_once()


def trial(heartbeat_mode, action='forward', duration=3, strength=.03):
    sent, callbacks = [], {}
    release = threading.Event()

    class Input:
        def __iter__(self):
            self.first = True
            return self

        def __next__(self):
            if self.first:
                self.first = False
                return 'ping\n'
            if heartbeat_mode == 'disconnect':
                release.wait(.8)
                raise StopIteration
            if heartbeat_mode == 'lost':
                release.wait(10)
                raise StopIteration
            release.wait(.1)
            if release.is_set():
                raise StopIteration
            return 'ping\n'

    class Message:
        def __init__(self):
            self.header = NS(frame_id=0, stamp=None)
            self.data = NS(x=0., y=0., z=0., roll=0., pitch=0., yaw=0., state=0)

    class Node:
        def create_subscription(self, typ, topic, callback, qos):
            callbacks[topic] = callback

        def create_publisher(self, typ, topic, qos):
            return NS(get_subscription_count=lambda: 1,
                      publish=lambda msg: sent.append((time.monotonic(), topic, msg)))

        def get_clock(self):
            return NS(now=lambda: NS(to_msg=lambda: None))

        def destroy_node(self):
            pass

    def spin_once(node, timeout_sec):
        time.sleep(.005)
        callbacks['/MOTION_INFO'](NS(data=NS(motion_state=NS(state=17), height=.4,
                                            vel_x=0., vel_y=0., vel_yaw=0.)))
        callbacks['/HES_STATUS'](NS(value=0))
        callbacks['/BATTERY_DATA'](NS(data=[NS(battery_level=80, protected_state=0)]))

    ros = NS(init=lambda: None, shutdown=lambda: None, create_node=lambda name: Node(),
             spin_once=spin_once)
    msg = NS(**dict.fromkeys(('BatteryData', 'MotionInfo', 'MotionState', 'StdMsgInt32', 'Steer'), Message))
    modules = {'fcntl': NS(LOCK_EX=1, LOCK_NB=2, flock=lambda *args: None),
               'rclpy': ros, 'rclpy.qos': NS(qos_profile_sensor_data=None), 'drdds.msg': msg}
    class Output(io.StringIO):
        def write(self, value):
            if heartbeat_mode == 'broken-output' and time.monotonic() - started > .65:
                raise BrokenPipeError('SSH stdout disconnected')
            return super().write(value)
    output = Output()
    started = time.monotonic()
    try:
        with patch.dict(sys.modules, modules), patch('sys.stdin', Input()), \
                patch('signal.SIGHUP', 1, create=True), patch('signal.signal'), \
                patch('builtins.open', mock_open()), contextlib.redirect_stdout(output):
            worker(action, strength, duration)
    finally:
        release.set()
    messages = [m for _, topic, m in sent if topic == '/STEER']
    assert messages and messages[-1].data.x == messages[-1].data.y == messages[-1].data.yaw == 0
    moving = [(t, m) for t, topic, m in sent if topic == '/STEER'
              and any(getattr(m.data, axis) for axis in ('x', 'y', 'yaw'))]
    assert moving, 'Test must exercise movement before stop'
    assert moving[-1][0] - moving[0][0] <= duration
    if heartbeat_mode == 'broken-output':
        assert moving[-1][0] - moving[0][0] < 1
        return
    result = json.loads(output.getvalue().splitlines()[-1])
    assert result['zero_sent']
    if heartbeat_mode in ('lost', 'disconnect'):
        assert 'heartbeat' in result['error']
        assert moving[-1][0] - moving[0][0] < 1
    else:
        assert result['error'] is None
        assert all(m.data.y == strength and m.data.x == 0 for _, m in moving)


if __name__ == '__main__':
    check()
    remote_login()
    trial('disconnect')
    trial('lost')
    trial('broken-output')
    trial('healthy', 'left', .2, strength=1.0)
    print('GUI_WORKER_STOP_CHECK_OK: disconnect, heartbeat timeout, timed lateral pulse')
