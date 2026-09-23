#!/usr/bin/env python3
"""S10 ASDU client (developer guide V1.0.1, plain UDP 30004): status reporting and the
使用模式切换 (常规 0 / 导航 1 / 辅助 2) that hands the robot between the remote and navigation.

  python3 tools/asdu_mode.py status [--seconds 5]         # heartbeat, print BasicStatus (read-only)
  python3 tools/asdu_mode.py set-mode 1 --i-am-on-site     # switch use mode, verify from the report
  python3 tools/asdu_mode.py remote-gait --i-am-on-site    # after navigation: give the robot its remote gait back (0x1001)

Only the heartbeat (0x00100064/0x00000005) and the mode switch (0x00100002/0x00500002) are
implemented; no axis, gait or motion-state commands. The robot reports BasicStatus
(MotionState, Gait, HES, ControlUsageMode) at 2 Hz to whoever sends heartbeats. The port is
DTLS by default; the vendor's example says encryption is disabled on this robot. If nothing
comes back within --seconds, the port is encrypted or filtered: stop and report, do not retry
blindly.
"""
from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import threading
import time
from datetime import datetime

SYNC = b"\xeb\x91\xeb\x90"
MODE_NAMES = {0: "常规(遥控)", 1: "导航", 2: "辅助"}
MOTION_NAMES = {-2: "软急停", 0: "未上报", 1: "站立", 2: "关节阻尼", 4: "趴下", 5: "标零", 17: "RL控制", 0x1001: "阻尼趴下"}
GAIT_NAMES = {0: "无", 0x1001: "基础(常规)", 0x1003: "楼梯(常规)", 0x3002: "平地(导航)", 0x3003: "楼梯(导航)"}


class Asdu:
    def __init__(self, host="10.21.33.103", port=30004):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        self.msg_id = 0
        self.pkt_id = 0
        self.lock = threading.Lock()
        self.status = {}
        self.status_t = 0.0
        self.reports = 0
        self.running = False

    def apdu(self, body):
        with self.lock:
            b = json.dumps(body, separators=(",", ":")).encode()
            h = struct.pack("<4sHHBBB5s", SYNC, len(b), self.msg_id, 0x01, self.pkt_id & 0xFF, 0x01, b"\x00" * 5)
            self.msg_id = (self.msg_id + 1) % 0x10000
            self.pkt_id = (self.pkt_id + 1) % 256
        return h + b

    @staticmethod
    def body(type_, command, items):
        return {"PatrolDevice": {"Type": type_, "Command": command,
                                 "Time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"), "Items": items}}

    def send(self, type_, command, items=None):
        self.sock.sendto(self.apdu(self.body(type_, command, items or {})), self.addr)

    def heartbeat(self):
        self.send(0x00100064, 0x00000005)

    def set_mode(self, mode):
        self.send(0x00100002, 0x00500002, {"Mode": int(mode)})

    def set_gait(self, gait):
        """运动步态切换 (developer guide 1.2.4): Type 0x00100001, Command 0x00300002, Items.GaitParam."""
        self.send(0x00100001, 0x00300002, {"GaitParam": int(gait)})

    def start(self):
        self.running = True
        threading.Thread(target=self._hb, daemon=True).start()
        threading.Thread(target=self._rx, daemon=True).start()

    def stop(self):
        self.running = False

    def _hb(self):
        while self.running:
            try:
                self.heartbeat()
            except OSError:
                pass
            time.sleep(1.0)

    def _rx(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(65535)
            except TimeoutError:
                continue
            except OSError:
                break
            if len(data) < 16 or data[:4] != SYNC:
                continue
            n = struct.unpack("<H", data[4:6])[0]
            try:
                body = json.loads(data[16:16 + n].decode("utf-8", "replace"))
            except ValueError:
                continue
            items = body.get("PatrolDevice", {}).get("Items", {})
            bs = items.get("BasicStatus")
            if bs:
                self.status, self.status_t = bs, time.monotonic()
                self.reports += 1

    def wait_status(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.status:
                return True
            time.sleep(0.1)
        return False


def fmt(bs):
    return (f"MotionState={bs.get('MotionState')}({MOTION_NAMES.get(bs.get('MotionState'), '?')}) "
            f"Gait={bs.get('Gait')}({GAIT_NAMES.get(bs.get('Gait'), '?')}) HES={bs.get('HES')} "
            f"ControlUsageMode={bs.get('ControlUsageMode')}({MODE_NAMES.get(bs.get('ControlUsageMode'), '?')}) "
            f"Model={bs.get('Model')} Charge={bs.get('Charge')}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["status", "set-mode", "remote-gait"])
    ap.add_argument("mode", nargs="?", type=int, choices=[0, 1, 2])
    ap.add_argument("--host", default="10.21.33.103")
    ap.add_argument("--port", type=int, default=30004)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--i-am-on-site", action="store_true", help="required for set-mode: an operator holds the remote")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    c = Asdu(a.host, a.port)
    c.start()
    try:
        if not c.wait_status(a.seconds):
            print(f"NO_STATUS: no BasicStatus from {a.host}:{a.port} in {a.seconds:.0f} s "
                  "(port encrypted/filtered, or the robot is off). Stop here.", file=sys.stderr)
            return 3
        if a.cmd == "status":
            t0 = time.monotonic()
            last = None
            while time.monotonic() - t0 < a.seconds:
                if c.status != last:
                    last = dict(c.status)
                    print(json.dumps(last) if a.json else fmt(last), flush=True)
                time.sleep(0.2)
            print(f"reports: {c.reports} in {a.seconds:.0f} s (~{c.reports / a.seconds:.1f} Hz)")
            return 0
        if a.cmd == "remote-gait":
            # A navigation run leaves the robot standing in 0x3002 / 0x3003 (or 0x1002). In use mode 0 the remote page only
            # drives in 0x1001 / 0x1002 / 0x1003, so the hand-back is not complete until the gait is a remote one again.
            if not a.i_am_on_site:
                print("remote-gait refused: pass --i-am-on-site only when an operator holds the remote", file=sys.stderr)
                return 2
            bs = dict(c.status)
            if bs.get("MotionState") != 17 or bs.get("Gait") in (0x1001, 0x1003):
                print("gait left alone: " + fmt(bs))
                return 0
            for i in range(3):
                c.set_gait(0x1001)
                end = time.monotonic() + 2.5
                while time.monotonic() < end:
                    if c.status.get("Gait") == 0x1001:
                        print("GAIT_OK 0x1001 after %d request(s): %s" % (i + 1, fmt(c.status)))
                        return 0
                    time.sleep(0.1)
            print("GAIT_NOT_CONFIRMED: " + fmt(c.status), file=sys.stderr)
            return 4
        if a.mode is None:
            ap.error("set-mode needs a mode (0 常规/遥控, 1 导航, 2 辅助)")
        if not a.i_am_on_site:
            print("set-mode refused: pass --i-am-on-site only when an operator holds the remote", file=sys.stderr)
            return 2
        before = dict(c.status)
        print("before: " + fmt(before))
        if before.get("ControlUsageMode") == a.mode:
            print(f"already in mode {a.mode}")
            return 0
        for i in range(3):
            c.set_mode(a.mode)
            end = time.monotonic() + 2.0
            while time.monotonic() < end:
                if c.status.get("ControlUsageMode") == a.mode:
                    print("after:  " + fmt(c.status))
                    print(f"MODE_OK {a.mode} ({MODE_NAMES[a.mode]}) after {i + 1} request(s)")
                    return 0
                time.sleep(0.1)
        print("MODE_NOT_CONFIRMED: " + fmt(c.status), file=sys.stderr)
        return 4
    finally:
        c.stop()


if __name__ == "__main__":
    sys.exit(main())
