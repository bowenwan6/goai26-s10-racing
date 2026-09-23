#!/usr/bin/env python3
"""Kinematic run of a whole route with gait switching emulated (s10_ros1_control: still >= 1 s, then the gait
changes). Flat synthetic perception: checks the route, the zones and the follower, not the terrain.
  python3 tests/nav/sim_full_route.py <route_dir> [speed] [stairs speed]"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "nav"))
import nav_core as core

route = sys.argv[1]; v = float(sys.argv[2]) if len(sys.argv) > 2 else 0.8; vs = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
cfg = core.load_yaml(os.environ.get("NAV_CONFIG") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "config", "nav.yaml"))
cfg.update(flat_speed_override=v, stairs_speed_override=vs); cfg["runner_params"]["walk_v"] = v; cfg["gains"]["max_forward"] = max(v, cfg["gains"]["max_forward"])
nc = core.NavCore(core.RouteBundle.from_dir(route), cfg)
w0 = nc.route.waypoints[0]; x, y, yaw = float(w0.xy[0]), float(w0.xy[1]), float(w0.yaw); obs = core.flat_observation(float(cfg["body_z_offset"]))
gait, since, switches, wz, vxs, modes, zero = "flat", None, [], [], [], set(), 0
for k in range(20 * 1500):
    t = k * 0.05
    r = nc.step(t, (x, y, 0.0, 0, 0, yaw, 0, 0), obs, gait, 0.0); vx, vy, w = r.command
    if r.gait_request and r.gait_request != gait:
        since = t if since is None else since
        if t - since > 1.5: gait = r.gait_request; switches.append((round(t, 1), gait, r.status.get("s"))); since = None
    x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * 0.05; y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * 0.05; yaw += w * 0.05
    wz.append(abs(w)); vxs.append(vx); modes.add(r.mode); zero += vx < 0.02
    if r.finished or r.mode == "HOLD": break
print("route %s: %.0f s, reached %d/%d, end %s, modes %s" % (os.path.basename(route.rstrip("/")), t, nc.reached, nc.n_wp, r.mode, sorted(modes)))
print("mean vx %.2f m/s, mean |wz| %.3f rad/s, standing still %.0f s, gait switches %s" % (sum(vxs) / len(vxs), sum(wz) / len(wz), zero * 0.05, switches))
if r.mode == "HOLD": print("HOLD:", r.status.get("reason"), r.status.get("follower"), r.status.get("follower_reason"), "s=", r.status.get("s"), r.status.get("blocked"))
print("SIM_FULL_ROUTE_OK" if r.finished else "SIM_FULL_ROUTE_FAILED")
