#!/usr/bin/env python3
"""Read-only lidar tap for the S10 106 board.

106 publishes /LIDAR/POINTS on its own host only (vendor design), so the 102
gateway cannot subscribe to it over DDS. This process runs on 106 as the normal
`user`, subscribes locally and forwards the raw serialized (CDR) message bytes
to the 102 gateway over one TCP connection. It never publishes anything into
ROS 2, never decodes points and never touches vendor files or services.

The DDS subscription exists only while the gateway has accepted the tap, so an
idle tap puts no load on the lidar driver. If the gateway is slower than the
lidar, the oldest queued scan is dropped (counted, never buffered without bound).

Uses only the Python standard library and 106's own /opt/ros/jazzy rclpy.
Run it through tap/run_tap_106.sh (nice 19, CPU cores 0-3).
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import signal
import socket
import struct
import threading
import time

PREFIX = struct.Struct('<4sHHII')
MAGIC = b'S10L'
HELLO, MESSAGE, HEARTBEAT, WELCOME = 1, 2, 3, 4

stop = threading.Event()


def frame(kind: int, header: dict, payload: bytes = b'') -> list:
    head = json.dumps(header, separators=(',', ':')).encode()
    return [PREFIX.pack(MAGIC, 1, kind, len(head), len(payload)), head, payload]


def recv_exact(sock: socket.socket, n: int) -> bytes:
    data = bytearray()
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError('gateway closed the connection')
        data += chunk
    return bytes(data)


class Tap:
    def __init__(self, args):
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        self.args = args
        self.rclpy = rclpy
        self.qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                              depth=args.depth, durability=DurabilityPolicy.VOLATILE)
        from rclpy.signals import SignalHandlerOptions
        # SIGINT/SIGTERM are handled here (stop event), not by rclpy.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('s10_lidar_tap', enable_rosout=False, start_parameter_services=False)
        from rosidl_runtime_py.utilities import get_message
        self.msg_class = get_message(args.type)
        self.queue: collections.deque = collections.deque(maxlen=args.queue)
        self.cv = threading.Condition()
        self.stats = dict(received=0, sent=0, dropped=0, bytes_sent=0, connections=0, last_error='')
        self.sub = None
        self.tap_seq = 0

    # rclpy passes (msg, info) because the callback takes two arguments.
    def on_message(self, raw: bytes, info) -> None:
        now_ns = time.time_ns()
        with self.cv:
            if len(self.queue) == self.queue.maxlen:
                self.stats['dropped'] += 1
            self.stats['received'] += 1
            self.queue.append((raw, now_ns, info))
            self.cv.notify()

    def subscribe(self) -> None:
        if self.sub is None:
            self.sub = self.node.create_subscription(self.msg_class, self.args.topic, self.on_message, self.qos,
                                                     raw=True)

    def unsubscribe(self) -> None:
        if self.sub is not None:
            self.node.destroy_subscription(self.sub)
            self.sub = None
        with self.cv:
            self.queue.clear()

    def spin(self) -> None:
        while not stop.is_set():
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def session(self, sock: socket.socket) -> None:
        hello = dict(tap='s10_lidar_tap', version=1, host=socket.gethostname(), pid=os.getpid(),
                     topics=[dict(topic=self.args.topic, type=self.args.type)])
        sock.sendall(b''.join(frame(HELLO, hello)))
        magic, version, kind, hlen, plen = PREFIX.unpack(recv_exact(sock, PREFIX.size))
        if magic != MAGIC or version != 1 or kind != WELCOME:
            raise ConnectionError('unexpected reply from gateway')
        welcome = json.loads(recv_exact(sock, hlen))
        recv_exact(sock, plen)
        if self.args.topic not in welcome.get('accepted', []):
            raise ConnectionError(f'gateway rejected {self.args.topic}: {welcome}')
        print(f'accepted by {welcome.get("gateway")}; subscribing {self.args.topic}', flush=True)
        self.subscribe()
        next_beat = time.monotonic()
        try:
            while not stop.is_set():
                item = None
                with self.cv:
                    if not self.queue:
                        self.cv.wait(timeout=0.2)
                    if self.queue:
                        item = self.queue.popleft()
                if item is not None:
                    raw, recv_ns, info = item
                    self.tap_seq += 1
                    header = dict(topic=self.args.topic, type=self.args.type, recv_ns=recv_ns,
                                  src_ns=int(info.get('source_timestamp', 0)),
                                  pub_seq=int(info.get('publication_sequence_number', -1)),
                                  tap_seq=self.tap_seq, tap_dropped=self.stats['dropped'])
                    sock.sendall(b''.join(frame(MESSAGE, header, raw)))
                    self.stats['sent'] += 1
                    self.stats['bytes_sent'] += len(raw)
                if time.monotonic() >= next_beat:
                    next_beat = time.monotonic() + 1.0
                    beat = dict(self.stats, subscribed=self.sub is not None, queue=len(self.queue),
                                wall_ns=time.time_ns())
                    sock.sendall(b''.join(frame(HEARTBEAT, beat)))
        finally:
            self.unsubscribe()

    def run(self) -> None:
        spinner = threading.Thread(target=self.spin, daemon=True)
        spinner.start()
        deadline = time.monotonic() + self.args.duration if self.args.duration else None
        backoff = 1.0
        last_log = 0.0
        while not stop.is_set() and (deadline is None or time.monotonic() < deadline):
            try:
                with socket.create_connection((self.args.gateway_host, self.args.gateway_port), timeout=3) as sock:
                    sock.settimeout(10)
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8 << 20)
                    self.stats['connections'] += 1
                    backoff = 1.0
                    if deadline is not None:
                        threading.Timer(max(0.0, deadline - time.monotonic()), stop.set).start()
                    self.session(sock)
            except (OSError, ConnectionError, ValueError) as exc:
                self.stats['last_error'] = str(exc)
                if time.monotonic() - last_log > 30:
                    print(f'gateway not reachable/closed ({exc}); retrying', flush=True)
                    last_log = time.monotonic()
            if not stop.is_set():
                stop.wait(backoff)
                backoff = min(backoff * 2, 10.0)
        print('stopping: ' + json.dumps(self.stats), flush=True)
        stop.set()
        spinner.join(timeout=2.0)
        self.unsubscribe()
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--gateway', default='10.21.33.102:47631', help='gateway host:port')
    parser.add_argument('--topic', default='/LIDAR/POINTS')
    parser.add_argument('--type', default='sensor_msgs/msg/PointCloud2')
    parser.add_argument('--depth', type=int, default=2, help='DDS reader history depth')
    parser.add_argument('--queue', type=int, default=2, help='scans buffered while sending (oldest dropped)')
    parser.add_argument('--duration', type=float, default=0, help='stop after N seconds (0 = run until stopped)')
    parser.add_argument('--cpus', default='0-3',
                        help='CPU cores to run on (vendor drivers use 4-7); "" keeps the default')
    args = parser.parse_args()
    host, _, port = args.gateway.rpartition(':')
    args.gateway_host, args.gateway_port = host, int(port)
    if not 1 <= args.depth <= 10 or not 1 <= args.queue <= 10:
        parser.error('--depth and --queue must be within 1..10')
    if args.cpus:
        # Set affinity here instead of with taskset: on 106 /usr/bin/taskset has file
        # capabilities, so glibc strips LD_LIBRARY_PATH and rclpy cannot load.
        lo, _, hi = args.cpus.partition('-')
        os.sched_setaffinity(0, set(range(int(lo), int(hi or lo) + 1)))
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    Tap(args).run()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
