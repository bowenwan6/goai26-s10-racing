#!/usr/bin/env python3
"""Straight 5 m route, real-robot-like disturbances: 10 Hz pose with 0.15 s latency and noise
(xy 1.5 cm, yaw 1 deg), first-order velocity response (0.25 s), a slow sideways drift, and a height
grid with 2 cm noise and a blind zone. Compares the old tracking settings with config/nav.yaml.
  python3 tests/nav/sim_tracking_noise.py <route_dir> [speed]"""
import copy
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "nav"))
import nav_core as core

OLD = dict(gains=dict(max_forward=0.6, max_lateral=0.2, max_yaw_rate=0.6, lookahead=1.0, lookahead_speed_gain=0.0,
                      pivot_threshold_deg=35.0, align_falloff_deg=60.0, brake_distance=0.05),
           planner={}, lane=dict(enabled=False), smooth=dict(enabled=False), fuse_frames=3)


def run(route, cfg, speed, seed=0, T=60.0):
    rng = np.random.default_rng(seed)
    cfg = copy.deepcopy(cfg)
    cfg["runner_params"]["walk_v"] = speed
    cfg["gains"]["max_forward"] = max(cfg["gains"]["max_forward"], speed)
    cfg["gains"]["max_yaw_rate"] = 0.8
    cfg["flat_speed_override"] = speed
    nc = core.NavCore(core.RouteBundle.from_dir(route), cfg)
    x, y, yaw = 0.0, 0.06, math.radians(4)
    v = np.zeros(3); hist = []; dt = 0.05; pose_meas = (x, y, yaw); flips = 0; last_sign = 0; zero_fwd = 0; wz_abs = []
    for k in range(int(T / dt)):
        t = k * dt
        hist.append((x, y, yaw))
        if k % 2 == 0:                                   # 10 Hz pose, 0.15 s old, noisy
            px, py, pyaw = hist[max(0, len(hist) - 4)]
            pose_meas = (px + rng.normal(0, 0.015), py + rng.normal(0, 0.015), pyaw + rng.normal(0, math.radians(1.0)))
        grid = np.full((13, 9), -0.41) + rng.normal(0, 0.02, (13, 9))
        valid = np.ones((13, 9), bool); valid[2:7, :] = rng.random((5, 9)) > 0.9   # blind zone
        per = core.Perception(np.eye(4)); per.blind_radius = 1.1
        grid, valid = per.fill_blind(grid, valid)
        obs = dict(grid=grid, valid=valid, scan=np.full(72, 6.0) + rng.normal(0, 0.05, 72), points_yaw=None)
        r = nc.step(t, (pose_meas[0], pose_meas[1], 0.46, 0, 0, pose_meas[2], v[0], v[2]), obs, "flat", 0.0)
        cmd = np.array(r.command, float)
        v += (cmd - v) * (dt / 0.25)                       # gait response
        x += (v[0] * math.cos(yaw) - v[1] * math.sin(yaw)) * dt
        y += (v[0] * math.sin(yaw) + v[1] * math.cos(yaw)) * dt + 0.01 * dt   # 1 cm/s sideways drift
        yaw += v[2] * dt + rng.normal(0, math.radians(0.15))
        if r.mode == "WALK":
            sgn = 0 if abs(cmd[2]) < 0.15 else int(math.copysign(1, cmd[2]))
            if sgn and last_sign and sgn != last_sign: flips += 1
            if sgn: last_sign = sgn
            zero_fwd += cmd[0] < 0.02; wz_abs.append(abs(cmd[2]))
        if r.finished:
            return dict(time=round(t, 1), flips=flips, zero_forward_ticks=int(zero_fwd), mean_abs_wz=round(float(np.mean(wz_abs)), 3),
                        end_d=round(y, 3), mode=r.mode)
    return dict(time=None, flips=flips, zero_forward_ticks=int(zero_fwd), mean_abs_wz=round(float(np.mean(wz_abs or [0])), 3), end_d=round(y, 3), mode=r.mode,
                s=r.status.get("s"), follower=r.status.get("follower"), reason=r.status.get("follower_reason"))


if __name__ == "__main__":
    route = sys.argv[1]; speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.7
    new = core.load_yaml(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "config", "nav.yaml"))
    old = copy.deepcopy(new); old.update(copy.deepcopy(OLD))
    for name, cfg in (("old", old), ("new", new)):
        res = [run(route, cfg, speed, seed) for seed in range(4)]
        print(name, "speed", speed)
        for r in res: print("   ", r)
