#!/usr/bin/env python3
"""ROS 1 side of the isolated control test: plays the navigation stack
(/cmd_vel, /web_cmd) and checks what the mock robot received.

Usage: drive_ros1.py <mock_log.jsonl> <scenario: enabled|dry_run>
"""
import json
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

LOG = sys.argv[1]
SCENARIO = sys.argv[2]
state = {}
failures = []


def on_state(msg):
    state.update(json.loads(msg.data))


def mock_since(t0, kind=None):
    out = []
    for line in open(LOG):
        r = json.loads(line)
        if r['t'] >= t0 and (kind is None or r['kind'] == kind):
            out.append(r)
    return out


def check(name, cond, detail=''):
    print(('PASS ' if cond else 'FAIL ') + name + (f'  ({detail})' if detail else ''), flush=True)
    if not cond:
        failures.append(name)


def wait_for(pred, timeout):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


class Cmd:
    """Publishes /cmd_vel at 20 Hz while active."""

    def __init__(self, pub):
        self.pub, self.v, self.on = pub, (0.0, 0.0, 0.0), False
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        while not rospy.is_shutdown():
            if self.on:
                t = Twist()
                t.linear.x, t.linear.y, t.angular.z = self.v
                self.pub.publish(t)
            time.sleep(0.05)


def main():
    rospy.init_node('control_test_driver', anonymous=True)
    rospy.Subscriber('/s10_control/state', String, on_state)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    web = rospy.Publisher('/web_cmd', String, queue_size=10)
    cmd = Cmd(cmd_pub)
    time.sleep(2)
    check('state topic alive', wait_for(lambda: 'feedback' in state, 10))
    check('feedback fresh', wait_for(lambda: state['feedback']['fresh'], 10), str(state.get('feedback')))

    if SCENARIO == 'dry_run':
        t0 = time.monotonic()
        web.publish('cmd4')
        cmd.v, cmd.on = (0.2, 0, 0), True
        time.sleep(3)
        cmd.on = False
        check('dry run: no publishers created', state['publishers_created'] is False)
        check('dry run: robot received nothing', not mock_since(t0), str(mock_since(t0)[:2]))
        check('dry run: robot still lying', state['feedback']['state'] == 4)
        return

    # 1. velocity while lying is not forwarded
    t0 = time.monotonic()
    cmd.v, cmd.on = (0.2, 0.0, 0.0), True
    time.sleep(1.5)
    cmd.on = False
    moving = [r for r in mock_since(t0, 'NAV_CMD') if any(abs(x) > 0 for x in r['v'])]
    check('no velocity while lying', not moving, f'{len(moving)} non-zero NAV_CMD')

    # 2. stand sequence
    t0 = time.monotonic()
    web.publish('cmd4')
    ok = wait_for(lambda: state['feedback']['state'] == 17 and state['feedback']['gait'] == 0x3002
                  and state['sequence'] == 'none', 20)
    check('stand -> RL -> navigation gait', ok, json.dumps(state['feedback']))
    order = [(r['kind'], r.get('state', r.get('gait'))) for r in mock_since(t0) if r['kind'] != 'NAV_CMD']
    check('stand order 1,17,0x3002', order[:3] == [('MOTION_STATE', 1), ('MOTION_STATE', 17), ('GAIT', 0x3002)],
          str(order))
    stamps = [r['stamp'] for r in mock_since(t0, 'MOTION_STATE')]
    check('command stamps follow robot clock', stamps and abs(stamps[0] - (time.time() + 14.0)) < 30, str(stamps[:1]))

    # 2b. cmd1 while already in the navigation gait is a no-op; cmd2 is ignored
    t0 = time.monotonic()
    web.publish('cmd1')
    web.publish('cmd2')
    time.sleep(1.5)
    sent = [r for r in mock_since(t0) if r['kind'] in ('GAIT', 'MOTION_STATE')]
    check('cmd1 in nav gait / cmd2: robot receives no mode commands', not sent, str(sent[:2]))
    check('still RL + navigation gait', state['feedback']['state'] == 17 and state['feedback']['gait'] == 0x3002)

    # 3. clamped forwarding at ~10 Hz
    t0 = time.monotonic()
    cmd.v, cmd.on = (0.8, 0.5, 2.0), True
    time.sleep(3)
    got = mock_since(t0, 'NAV_CMD')
    nz = [r for r in got if any(abs(x) > 0 for x in r['v'])]
    check('velocity forwarded', len(nz) > 20, f'{len(nz)} msgs')
    check('velocity clamped to limits', all(abs(r['v'][0] - 0.3) < 1e-6 and abs(r['v'][1] - 0.1) < 1e-6
                                          and abs(r['v'][2] - 0.5) < 1e-6 for r in nz), str(nz[:1]))
    rate = (len(got) - 1) / (got[-1]['t'] - got[0]['t']) if len(got) > 1 else 0
    check('rate about 10 Hz', 8.5 <= rate <= 11.5, f'{rate:.2f} Hz')

    # 4. /cmd_vel timeout -> zeros for zero_hold, then silence
    cmd.on = False
    t_stop = time.monotonic()
    time.sleep(3)
    after = mock_since(t_stop, 'NAV_CMD')
    late_nz = [r for r in after if r['t'] > t_stop + 0.7 and any(abs(x) > 0 for x in r['v'])]
    check('timeout: no motion after 0.7 s', not late_nz, str(late_nz[:1]))
    zeros = [r for r in after if not any(abs(x) > 0 for x in r['v'])]
    check('timeout: zeros sent', len(zeros) >= 5, f'{len(zeros)} zeros')
    check('then quiet (no NAV_CMD after 2.5 s)', not [r for r in after if r['t'] > t_stop + 2.5])

    # 5. Nav stop latches, Nav continue releases
    cmd.v, cmd.on = (0.1, 0.0, 0.0), True
    time.sleep(1)
    web.publish('Nav stop')
    t_latch = time.monotonic()
    time.sleep(2.5)
    nz = [r for r in mock_since(t_latch + 0.3, 'NAV_CMD') if any(abs(x) > 0 for x in r['v'])]
    check('Nav stop: no motion while latched', not nz and state['latched_stop'], f'{len(nz)} non-zero')
    web.publish('Nav continue')
    t_cont = time.monotonic()
    time.sleep(1.5)
    nz = [r for r in mock_since(t_cont, 'NAV_CMD') if any(abs(x) > 0 for x in r['v'])]
    check('Nav continue: motion resumes', len(nz) > 5, f'{len(nz)} non-zero')

    # 6. lie down while commanded to move: zero first, wait still, then 4
    t_lie = time.monotonic()
    web.publish('cmd3')
    ok = wait_for(lambda: state['feedback']['state'] == 4 and state['sequence'] == 'none', 15)
    cmd.on = False
    check('lie down completes', ok, json.dumps(state['feedback']))
    evts = mock_since(t_lie)
    lie_t = next((r['t'] for r in evts if r['kind'] == 'MOTION_STATE' and r['state'] == 4), None)
    nz_after = [r for r in evts if r['kind'] == 'NAV_CMD' and r['t'] > t_lie + 0.3 and any(abs(x) > 0 for x in r['v'])]
    check('lie: no motion after request', not nz_after, f'{len(nz_after)} non-zero')
    check('lie: sent only after >=1 s still', lie_t is not None and lie_t - t_lie >= 1.0, f'{lie_t and lie_t - t_lie:.2f}s')

    # 7. a second /NAV_CMD publisher latches a fault
    print('STARTING_SECOND_PUBLISHER', flush=True)
    check('fault on second /NAV_CMD publisher', wait_for(lambda: state.get('fault', '') != '', 15), state.get('fault'))
    web.publish('Nav continue')
    time.sleep(0.5)
    check('continue refused after fault', state['latched_stop'] or state['fault'] != '')


if __name__ == '__main__':
    try:
        main()
    finally:
        print('CONTROL_TEST_OK' if not failures else f'CONTROL_TEST_FAILED {failures}', flush=True)
