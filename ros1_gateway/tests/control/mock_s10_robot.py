#!/usr/bin/env python3
"""Minimal S10 motion state machine for isolated tests (never on a robot network).

Publishes /MOTION_INFO at 20 Hz and reacts to /MOTION_STATE, /GAIT and /NAV_CMD
the way the developer guide describes: stand (1) -> standing, 17 -> RL,
navigation gait only in RL, velocity only in RL + navigation gait, lie (4).
A gait switch takes --gait-delay s and is refused while the robot moves.
Every received command is appended to --log as JSON lines.

--silent-nav-publisher: also creates a /NAV_CMD publisher that never publishes
(the shared dog's idle native handler/localPlanner), to check exclusive_mode messages.
"""
import argparse
import json
import threading
import time

import rclpy
from drdds.msg import Gait, MotionInfo, MotionState, NavCmd
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', required=True)
    ap.add_argument('--initial-state', type=int, default=4)
    ap.add_argument('--clock-offset', type=float, default=14.0, help='robot clock ahead of local clock (s)')
    ap.add_argument('--gait-delay', type=float, default=1.0)
    ap.add_argument('--silent-nav-publisher', action='store_true')
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('mock_s10_robot')
    rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
    pub = node.create_publisher(MotionInfo, '/MOTION_INFO', QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5))
    silent = node.create_publisher(NavCmd, '/NAV_CMD', rel) if args.silent_nav_publisher else None
    lock = threading.Lock()
    sim = dict(state=args.initial_state, gait=0x1001, v=[0.0, 0.0, 0.0], v_t=0.0, pending=None, pending_gait=None)
    log = open(args.log, 'a')

    def record(kind, **kw):
        log.write(json.dumps(dict(t=time.monotonic(), kind=kind, **kw)) + '\n')
        log.flush()

    def moving():
        return any(abs(x) > 1e-6 for x in sim['v'])

    def on_state(m):
        with lock:
            record('MOTION_STATE', state=m.data.state, stamp=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            s = m.data.state
            if s == 1 and sim['state'] in (0, 3, 4):
                sim['pending'] = (time.monotonic() + 1.0, 1)
            elif s == 17 and sim['state'] == 1:
                sim['pending'] = (time.monotonic() + 0.5, 17)
            elif s == 4 and sim['state'] in (1, 17):
                sim['pending'] = (time.monotonic() + 1.0, 4)

    def on_gait(m):
        with lock:
            record('GAIT', gait=m.data.gait, moving=moving())
            if sim['state'] == 17 and m.data.gait in (0x3002, 0x3003) and not moving():
                sim['pending_gait'] = (time.monotonic() + args.gait_delay, m.data.gait)

    def on_nav(m):
        with lock:
            v = [m.data.x_vel, m.data.y_vel, m.data.yaw_vel]
            record('NAV_CMD', v=v, stamp=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            if sim['state'] == 17 and sim['gait'] in (0x3002, 0x3003):
                sim['v'], sim['v_t'] = v, time.monotonic()

    node.create_subscription(MotionState, '/MOTION_STATE', on_state, rel)
    node.create_subscription(Gait, '/GAIT', on_gait, rel)
    node.create_subscription(NavCmd, '/NAV_CMD', on_nav, rel)

    def tick():
        with lock:
            now = time.monotonic()
            if sim['pending'] and now >= sim['pending'][0]:
                sim['state'] = sim['pending'][1]
                sim['pending'] = None
                if sim['state'] != 17:
                    sim['gait'] = 0x1001
            if sim['pending_gait'] and now >= sim['pending_gait'][0]:
                if sim['state'] == 17:
                    sim['gait'] = sim['pending_gait'][1]
                sim['pending_gait'] = None
            if now - sim['v_t'] > 0.5 or sim['state'] != 17:
                sim['v'] = [0.0, 0.0, 0.0]  # the robot's own command timeout
            m = MotionInfo()
            t = time.time() + args.clock_offset
            m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
            m.data.vel_x, m.data.vel_y, m.data.vel_yaw = sim['v']
            m.data.height = 0.40 if sim['state'] == 17 else 0.19 if sim['state'] == 1 else 0.07
            m.data.motion_state.state = sim['state']
            m.data.gait_state.gait = sim['gait']
        pub.publish(m)

    node.create_timer(0.05, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _ = silent


if __name__ == '__main__':
    main()
