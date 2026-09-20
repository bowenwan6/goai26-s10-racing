#!/usr/bin/env python3
"""Kinematic plumbing test of nav_core (no ROS, no lidar): a point robot integrates the runner's
commands along a route_v2 file on flat, fully known synthetic ground; the gait "robot" switches
only at a standstill after a delay, as s10_ros1_control does.

  python3 tests/nav/test_nav_core.py [route_v2.json] [--stairs-zones] [--max-s 1500] [--first N --last M]

Checks: the runner reaches waypoints in order, requests the stairs gait inside every zone (with
--stairs-zones, zones are synthesised from the route's stairs segments), never commands more than
the configured speeds, and finishes.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "nav"))
import nav_core as core  # noqa: E402
from s10_auto_nav.rl_nav.maneuvers import Maneuver, save as save_maneuvers  # noqa: E402
from s10_auto_nav.route_v2 import RouteV2  # noqa: E402


def stairs_zones(route, path, pre=0.7, post=0.7):
    zones = []
    for i, seg in enumerate(route.segments):
        if seg.gait != "stairs":
            continue
        s0, s1 = float(path.waypoint_s[i]), float(path.waypoint_s[i + 1])
        if zones and zones[-1].s1 >= s0 - pre - 1e-6:
            zones[-1].s1 = s1 + post
            zones[-1].s_last = s1
            continue
        zones.append(Maneuver(id=f"Z{len(zones):02d}", s0=max(0.0, s0 - pre), s1=s1 + post, s_first=s0, s_last=s1,
                              policy="stairs", kinds=["synthetic"]))
    return zones


class GaitRobot:
    """Switches gait only when still, 1 s after the request (s10_ros1_control semantics)."""

    def __init__(self):
        self.gait, self.pending, self.t_req = "flat", None, 0.0

    def update(self, t, request, speed):
        if request and request != self.gait and self.pending != request:
            self.pending, self.t_req = request, t
        if self.pending and speed < 0.05 and t - self.t_req >= 1.0:
            self.gait, self.pending = self.pending, None
        return self.gait if self.pending is None else None  # "switching" -> None

    @property
    def blocking(self):
        return self.pending is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("route", nargs="?", default=os.path.join(HERE, "data", "route_v2_v3draft.json"))
    ap.add_argument("--stairs-zones", action="store_true")
    ap.add_argument("--max-s", type=float, default=1500.0)
    ap.add_argument("--first", type=int, default=1)
    ap.add_argument("--last", type=int, default=0)
    ap.add_argument("--rate", type=float, default=20.0)
    args = ap.parse_args()

    cfg = core.load_yaml(os.path.join(HERE, "..", "..", "config", "nav.yaml"))
    cfg["control_rate"] = args.rate
    route = RouteV2.load(args.route)
    if args.first > 1 or args.last:
        raw = dict(route.raw)
        last = args.last or len(raw["waypoints"])
        raw["waypoints"] = raw["waypoints"][args.first - 1:last]
        raw["segments"] = raw["segments"][args.first - 1:last - 1]
        sub = os.path.join(HERE, "data", "_sub.json")
        json.dump(raw, open(sub, "w"))
        route_path = sub
        route = RouteV2.load(sub)
    else:
        route_path = args.route
    man_path = None
    if args.stairs_zones:
        from s10_auto_nav.route_v2 import RoutePath
        zones = stairs_zones(route, RoutePath(route))
        man_path = os.path.join(HERE, "data", "_zones.json")
        save_maneuvers(man_path, zones, route_id="synthetic", source="test_nav_core")
        print(f"{len(zones)} synthetic stairs zones: " + ", ".join(f"{z.id} s={z.s0:.1f}-{z.s1:.1f}" for z in zones))
    nav = core.NavCore(core.RouteBundle(route_path, man_path), cfg)
    path = nav.follower.path
    zoff = float(cfg["body_z_offset"])
    obs = core.flat_observation(zoff)
    w0 = route.waypoints[0]
    x, y, yaw = w0.position[0], w0.position[1], (w0.yaw if w0.yaw is not None else 0.0)
    robot = GaitRobot()
    dt = 1.0 / args.rate
    t, v_fwd, yaw_rate = 0.0, 0.0, 0.0
    max_cmd = [0.0, 0.0, 0.0]
    modes, reached_t, stairs_in_zone, gait_switches = {}, [], set(), 0
    last_gait = "flat"
    failures = []
    wall = time.time()
    while t < args.max_s:
        s_now = float(nav.follower.last_output.s) if nav.follower.last_output and math.isfinite(nav.follower.last_output.s) else 0.0
        z = float(path.point_at(min(max(s_now, 0.0), path.length))[2]) + zoff
        gait = robot.update(t, nav.owner_requested and core.OWNER_TO_GAIT.get(nav.owner_requested), abs(v_fwd) + abs(yaw_rate))
        if gait and gait != last_gait:
            gait_switches += 1
            last_gait = gait
        r = nav.step(t, (x, y, z, 0.0, 0.0, yaw, v_fwd, yaw_rate), obs, gait)
        modes[r.mode] = modes.get(r.mode, 0) + dt
        for k in range(3):
            max_cmd[k] = max(max_cmd[k], abs(r.command[k]))
        if r.reached:
            reached_t.append((r.reached, round(t, 1)))
        zone = nav.runner._zone()
        if zone and zone.s0 <= s_now <= zone.s1 and r.gait_request == "stairs":
            stairs_in_zone.add(zone.id)
        vx, vy, wz = (0.0, 0.0, 0.0) if robot.blocking else r.command
        # first-order response, then integrate (world frame)
        v_fwd += (vx - v_fwd) * min(1.0, 4.0 * dt)
        yaw_rate += (wz - yaw_rate) * min(1.0, 4.0 * dt)
        x += (v_fwd * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (v_fwd * math.sin(yaw) + vy * math.cos(yaw)) * dt
        yaw = math.atan2(math.sin(yaw + yaw_rate * dt), math.cos(yaw + yaw_rate * dt))
        t += dt
        if r.finished:
            break
        if r.mode == "HOLD" and modes.get("HOLD", 0) > 30:
            failures.append(f"HOLD for 30 s: {r.reason} {r.status.get('follower_reason')}")
            break
    print(f"sim {t:.0f} s in {time.time() - wall:.1f} s wall; reached {nav.reached}/{nav.n_wp}; gait switches {gait_switches}")
    print("modes (s): " + ", ".join(f"{k} {v:.0f}" for k, v in sorted(modes.items(), key=lambda kv: -kv[1])))
    print("reached: " + ", ".join(f"{ids[0]}@{tt}" for ids, tt in reached_t))
    print(f"max |cmd| vx {max_cmd[0]:.2f} vy {max_cmd[1]:.2f} wz {max_cmd[2]:.2f}")
    rp = nav.runner.p if hasattr(nav.runner, "p") else None
    if nav.reached != nav.n_wp:
        failures.append(f"reached {nav.reached}/{nav.n_wp}")
    if max_cmd[0] > 0.3 + 1e-6:
        failures.append(f"forward command {max_cmd[0]:.2f} > 0.30")
    if args.stairs_zones:
        missing = [z.id for z in nav.maneuvers if z.id not in stairs_in_zone]
        if missing:
            failures.append(f"no stairs request inside zones {missing}")
        if gait_switches == 0:
            failures.append("no gait switch happened")
    print("NAV_CORE_TEST_OK" if not failures else "NAV_CORE_TEST_FAILED " + "; ".join(failures))
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
