#!/usr/bin/env python3
"""Ledges climbed in the platform gait (terrain.json `jumps`). With a dog that answers 0.3 s late:
  1. it is in the platform gait and at <= 0.45 m/s from 0.8 m before every ledge (also when the zone before is stairs);
  2. it crosses every ledge within tol_lateral of the lane and tol_yaw of square;
  3. pushed 0.20 m sideways just before the check point, it backs up 0.5 m, comes again and crosses inside the tolerance;
  4. pushed 0.35 m (beyond hold_beyond_m) with retries = 0: it does NOT take off: HOLD, reason "jump lane".
python3 tests/nav/test_jump_lane.py <route_dir> [speed]"""
import json, math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "nav"))
import nav_core as core
route = sys.argv[1]; v = float(sys.argv[2]) if len(sys.argv) > 2 else 1.67
jumps = json.load(open(os.path.join(route, "terrain.json"))).get("jumps", [])
if not jumps: print("JUMP_LANE_SKIPPED: no jumps in this route"); sys.exit(0)


def run(push=None, retries=None):
    cfg = core.load_yaml(os.environ.get("NAV_CONFIG") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "config", "nav.yaml"))
    cfg.update(flat_speed_override=v, stairs_speed_override=v); cfg["runner_params"]["walk_v"] = v; cfg["gains"]["max_forward"] = max(v, cfg["gains"]["max_forward"])
    if retries is not None: cfg.setdefault("jump", {})["retries"] = retries
    nc = core.NavCore(core.RouteBundle.from_dir(route), cfg); J = nc.jumps; tol = nc.jump_cfg
    w0 = nc.route.waypoints[0]; x, y, yaw = float(w0.xy[0]), float(w0.xy[1]), float(w0.yaw)
    obs = core.flat_observation(float(cfg["body_z_offset"])); gait, since, lag, pushed = "flat", None, [], False
    seen = [dict(v_near=0.0, gait_near=set(), gait_at_ledge=None, cross=None, phases=set()) for _ in J]
    for k in range(20 * 1500):
        t = k * 0.05
        r = nc.step(t, (x, y, 0.0, 0, 0, yaw, 0, 0), obs, gait, 0.0); lag.append(r.command); vx, vy, w = lag[-7] if len(lag) > 7 else (0, 0, 0)
        if r.gait_request and r.gait_request != gait:
            since = t if since is None else since
            if t - since > 1.5: gait, since = r.gait_request, None
        for q, j in zip(seen, J):
            su = (x - j["xy"][0]) * math.cos(j["yaw"]) + (y - j["xy"][1]) * math.sin(j["yaw"]); sv = -(x - j["xy"][0]) * math.sin(j["yaw"]) + (y - j["xy"][1]) * math.cos(j["yaw"])
            if abs(sv) < 1.5 and -0.8 <= su <= 0.0: q["v_near"] = max(q["v_near"], vx); q["gait_near"].add(gait)
            if abs(sv) < 1.5 and q["cross"] is None and su >= 0.0 and su < 0.3: q["gait_at_ledge"] = gait; q["cross"] = (sv, math.degrees(math.atan2(math.sin(yaw - j["yaw"]), math.cos(yaw - j["yaw"]))))
            if r.status.get("jump") and abs(su) < 4 and abs(sv) < 1.5: q["phases"].add(r.status["jump"])
            if push and not pushed and j is J[0] and -0.80 <= su <= -0.72 and abs(sv) < 0.3:
                x += -math.sin(j["yaw"]) * push; y += math.cos(j["yaw"]) * push; pushed = True
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * 0.05; y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * 0.05; yaw += w * 0.05
        if r.finished or r.mode == "HOLD" or (push and seen[0]["cross"] is not None): break
    return r, seen, tol


bad = 0
r, seen, tol = run()
for k, q in enumerate(seen):
    ok = q["v_near"] <= 0.45 and "platform" in q["gait_near"] and q["gait_at_ledge"] == "platform" and q["cross"] is not None and abs(q["cross"][0]) <= tol["tol_lateral"] and abs(q["cross"][1]) <= tol["tol_yaw_deg"]
    bad += not ok
    print("ledge %d: fastest %.2f m/s in the last 0.8 m, gait there %s, crossed %s, check phases %s: %s" % (k + 1, q["v_near"], sorted(q["gait_near"]), "%+.3f m / %+.1f deg" % q["cross"] if q["cross"] else "NOT", sorted(q["phases"]), "ok" if ok else "BAD"))
print("undisturbed run:", "finished" if r.finished else r.mode); bad += not r.finished
r, seen, _ = run(push=0.20); q = seen[0]
ok = "back" in q["phases"] and q["cross"] is not None and abs(q["cross"][0]) <= tol["tol_lateral"]; bad += not ok
print("pushed 0.20 m: phases %s, crossed %s: %s" % (sorted(q["phases"]), "%+.3f m / %+.1f deg" % q["cross"] if q["cross"] else "NOT", "ok" if ok else "BAD"))
r, seen, _ = run(push=0.35, retries=0); q = seen[0]
ok = r.mode == "HOLD" and q["cross"] is None and "jump lane" in str(r.status.get("why") or r.status.get("reason") or r.reason); bad += not ok
print("pushed 0.35 m, no retries: end %s (%s), crossed %s: %s" % (r.mode, str(r.status.get("why") or r.reason)[:90], q["cross"], "ok" if ok else "BAD"))
print("JUMP_LANE_OK" if not bad else "JUMP_LANE_FAILED"); sys.exit(1 if bad else 0)
