"""Offline guards; --robot-dry also exercises the isolated AGX runner."""
from argparse import Namespace
import json
import subprocess
import sys
import threading
import time

from s10_sdk_trial import check, MANUAL_MAX_FORWARD_INPUT
from run_s10_sdk_trial import run


def robot_dry():
    started = [None]
    cancel = threading.Event()
    directions = [(0.1, 0., 0.), (-0.1, 0., 0.), (0., 0.1, 0.),
                  (0., -0.1, 0.), (0., 0., 0.1), (0., 0., -0.1)]

    def report(line):
        print(line, flush=True)
        if line == 'MANUAL_READY':
            # Verify the actual running process has its joint output remapped.
            actual = subprocess.check_output(['ssh', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                's10-48-golai', 'ps -C rl_deploy -o args='], text=True)
            if '/JOINTS_CMD:=/s10_sdk_test/DRY_JOINTS_CMD' not in actual:
                cancel.set()
                raise AssertionError('Dry output remap missing')
            started[0] = time.monotonic()

    def command():
        if started[0] is None:
            return {'action': 'zero'}
        elapsed = time.monotonic() - started[0]
        index = int(elapsed)
        if index >= len(directions):
            return {'action': 'lie'}
        return {'action': 'move', 'axes': directions[index]} if elapsed - index < .5 else {'action': 'zero'}

    args = Namespace(host='s10-48-golai', user=None, host_alias=None,
        source_install='/home/golai/s10_control_ws/install',
        trial_root='/home/golai/s10-sdk-isolated-xly21vwi', mode='manual-dry')
    result = run(args, cancel=cancel, report=report, command_source=command)
    seen = [tuple(json.loads(e['text'][5:])) for e in result['events'] if e['text'].startswith('AXES ')]
    expected = [value for direction in directions for value in (direction, (0., 0., 0.))]
    assert seen == expected, (seen, expected)
    assert result['dry'] and result['manual'] and result['completed'] and result['error'] is None
    assert result['final_state'] == 'liedown_state'
    print('MANUAL_ROBOT_DRY_CHECK_OK: six axes, release-to-zero, explicit lie, isolated motor output')


def fore_aft_command(elapsed, strength=.1, seconds=.3):
    """Bounded fore/aft inputs; one-second probes ramp up/down over 0.2 seconds."""
    if not .01 <= strength <= MANUAL_MAX_FORWARD_INPUT or not .1 <= seconds <= 1:
        raise ValueError('Fore/aft probe requires x input 0.01-2 (target 0.015-3 m/s) and 0.1-1 second duration')
    for start, direction in ((2., 1), (4.+seconds, -1)):
        if start <= elapsed < start + seconds:
            ramp = min(1., (elapsed-start)/.2, (start+seconds-elapsed)/.2) if seconds >= .8 else 1.
            return {'action': 'move', 'axes': [direction*strength*ramp, 0., 0.]}
    return {'action': 'lie' if elapsed >= 6.+2*seconds else 'zero'}


def robot_fore_aft(dry=True, strength=.1, seconds=.3):
    fore_aft_command(0, strength, seconds)  # Reject invalid parameters before connecting.
    print(f'FORE_AFT_TARGET_MPS={1.5*strength:g} PULSE_SECONDS={seconds:g}', flush=True)
    started = [None]
    def report(line):
        print(line, flush=True)
        if line == 'MANUAL_READY':
            started[0] = time.monotonic()

    def command():
        return {'action': 'zero'} if started[0] is None else fore_aft_command(time.monotonic()-started[0], strength, seconds)

    args = Namespace(host='s10-48-golai', user=None, host_alias=None,
        source_install='/home/golai/s10_control_ws/install',
        trial_root='/home/golai/s10-sdk-isolated-xly21vwi', mode='manual-dry' if dry else 'manual')
    result = run(args, report=report, command_source=command)
    assert result['completed'] and result['final_state'] == 'liedown_state'
    assert any('joint_commands' in s['data'] for s in result['samples'])
    print('FORE_AFT_SEQUENCE_COMPLETED dry=' + str(dry))


if __name__ == '__main__':
    check()
    for t, action in ((0,'zero'),(2,'move'),(2.3,'zero'),(4.3,'move'),(4.6,'zero'),(6.6,'lie')):
        assert fore_aft_command(t)['action'] == action
    assert fore_aft_command(2)['axes'][0] == .1
    assert fore_aft_command(4.3)['axes'][0] == -.1
    assert fore_aft_command(4.3, .3)['axes'][0] == -.3
    assert fore_aft_command(4.3, .5)['axes'][0] == -.5
    assert fore_aft_command(4.3, 1.)['axes'][0] == -1
    assert fore_aft_command(2, 1, 1)['axes'][0] == 0
    assert abs(fore_aft_command(2.1, 1, 1)['axes'][0] - .5) < 1e-9
    assert fore_aft_command(2.5, 1, 1)['axes'][0] == 1
    assert fore_aft_command(5.5, 1, 1)['axes'][0] == -1
    assert fore_aft_command(8, 1, 1)['action'] == 'lie'
    assert fore_aft_command(2.5, 2, 1)['axes'][0] == 2
    assert fore_aft_command(5.5, 2, 1)['axes'][0] == -2
    for bad in (0., 2.01, float('nan')):
        try:
            fore_aft_command(0, bad)
        except ValueError:
            pass
        else:
            raise AssertionError('Unsafe automatic probe strength accepted')
    try:
        fore_aft_command(0, 1, 1.1)
    except ValueError:
        pass
    else:
        raise AssertionError('Excessive automatic probe duration accepted')
    if sys.argv[1:] == ['--robot-dry']:
        robot_dry()
    elif len(sys.argv) in (3, 4) and sys.argv[1] in ('--fore-aft-mps-dry', '--fore-aft-mps-live'):
        robot_fore_aft(dry=sys.argv[1] == '--fore-aft-mps-dry',
                      strength=float(sys.argv[2])/1.5,
                      seconds=float(sys.argv[3]) if len(sys.argv) == 4 else 1.)
    elif len(sys.argv) in (2, 3, 4) and sys.argv[1] in ('--fore-aft-dry', '--fore-aft-live'):
        robot_fore_aft(dry=sys.argv[1] == '--fore-aft-dry',
                      strength=float(sys.argv[2]) if len(sys.argv) >= 3 else .1,
                      seconds=float(sys.argv[3]) if len(sys.argv) == 4 else .3)
