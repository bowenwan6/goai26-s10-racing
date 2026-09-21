#!/usr/bin/env python3
"""The dog must ARRIVE slowly at every stairs / platform zone and stop for the gait switch BEFORE the recorded zone
start, whatever the run speed (2026-09-21: at 1.67 m/s it ran 1.1 m past the start onto the steps and the robot never
confirmed the stairs gait).  python3 tests/nav/test_zone_entry.py <route_dir> [speed]"""
import json, math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "nav"))
import nav_core as core
route = sys.argv[1]; v = float(sys.argv[2]) if len(sys.argv) > 2 else 1.67
cfg = core.load_yaml(os.environ.get("NAV_CONFIG") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "config", "nav.yaml"))
cfg.update(flat_speed_override=v, stairs_speed_override=v); cfg["runner_params"]["walk_v"] = v; cfg["gains"]["max_forward"] = max(v, cfg["gains"]["max_forward"])
if os.environ.get("NO_ENTRY"): cfg["zone_entry"] = dict(lead_m=0.0, entry_speed=9.0, decel=0.0)
recorded = [(a, b) for a, b, n in json.load(open(os.path.join(route, "terrain.json")))["gaits"] if n in ("stairs", "platform")]
nc = core.NavCore(core.RouteBundle.from_dir(route), cfg)
w0 = nc.route.waypoints[0]; x, y, yaw = float(w0.xy[0]), float(w0.xy[1]), float(w0.yaw); obs = core.flat_observation(float(cfg["body_z_offset"]))
gait, since, res, lag = "flat", None, [], []
for k in range(20 * 1500):
    t = k * 0.05
    r = nc.step(t, (x, y, 0.0, 0, 0, yaw, 0, 0), obs, gait, 0.0); vx, vy, w = r.command
    lag.append((vx, vy, w)); vx, vy, w = lag[-7] if len(lag) > 7 else (0, 0, 0)          # the real dog answers ~0.3 s late
    s = r.status.get("s")
    if r.gait_request and r.gait_request != gait:
        if since is None:
            since = t; z = min(recorded, key=lambda q: abs(q[0] - (s or 0)))
            if r.gait_request != "flat": res.append(dict(zone=z[0], s_request=s, v_request=math.hypot(vx, vy)))
        if t - since > 1.5:
            if r.gait_request != "flat": res[-1]["s_switched"] = s
            gait = r.gait_request; since = None
    x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * 0.05; y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * 0.05; yaw += w * 0.05
    if r.finished or r.mode == "HOLD": break
bad = 0
for q in res:
    over = q.get("s_switched", q["s_request"]) - q["zone"]
    follows = any(abs(b_ - q["zone"]) < 0.15 for _a, b_ in recorded)      # zone right after another zone (stairs -> platform): no lead
    ok = (over <= 0.30 and q["v_request"] <= 0.5) if follows else (over <= -0.10 and q["v_request"] <= 1.0); bad += not ok      # possible, so it must ARRIVE slowly; the platform zone starts >= 1.5 m before its ledge
    print("zone at %6.1f m: gait asked at s=%.2f going %.2f m/s, standing for the switch at s=%.2f (%+.2f m from the recorded start) %s" % (q["zone"], q["s_request"], q["v_request"], q.get("s_switched", float("nan")), over, "ok" if ok else "TOO FAST / TOO FAR"))
print("speed %.2f: %d zone entries, %d bad, run %s" % (v, len(res), bad, "finished" if r.finished else r.mode))
print("ZONE_ENTRY_OK" if not bad and r.finished and len(res) == len(recorded) else "ZONE_ENTRY_FAILED"); sys.exit(0 if not bad and r.finished else 1)
