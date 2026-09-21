#!/usr/bin/env python3
"""READJUST instead of HOLD. The dog is put beside the taught line (as it ended up in the 2026-09-21 field runs) and must
walk back onto it by itself; only far from the line it must HOLD.  python3 tests/nav/test_readjust.py <route_dir>
The simulated dog IGNORES side-step commands (what the real one did in the stairs gait on steps)."""
import json, math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "nav"))
import numpy as np, shutil, tempfile, types
for m_, names_ in (("rospy", []), ("geometry_msgs.msg", ["PoseWithCovarianceStamped"]), ("nav_msgs.msg", ["Odometry"]), ("std_msgs.msg", ["String"])):
    parts_ = m_.split(".")
    for i_ in range(1, len(parts_) + 1): sys.modules.setdefault(".".join(parts_[:i_]), types.ModuleType(".".join(parts_[:i_])))
    for n_ in names_: setattr(sys.modules[m_], n_, object)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "scripts"))
import nav_core as core
import nav_run
route = sys.argv[1]
base = core.load_yaml(os.environ.get("NAV_CONFIG") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "config", "nav.yaml"))
doc = json.load(open(os.path.join(route, "route_v2.json")))
line = np.vstack([np.array(s["centerline"])[:, :2] for s in doc["segments"]]); line = line[np.r_[True, np.linalg.norm(np.diff(line, axis=0), axis=1) > 1e-6]]
S = np.r_[0, np.cumsum(np.linalg.norm(np.diff(line, axis=0), axis=1))]


def case(name, s0, offset, yaw_err_deg, gait, expect, v=1.67, seconds=40, enabled=True):
    cfg = json.loads(json.dumps(base)); cfg.update(flat_speed_override=v, stairs_speed_override=v); cfg["runner_params"]["walk_v"] = v
    cfg["gains"]["max_forward"] = max(v, cfg["gains"]["max_forward"]); cfg.setdefault("readjust", {})["enabled"] = enabled
    i = int(np.searchsorted(S, s0)); th = math.atan2(line[i + 5][1] - line[i][1], line[i + 5][0] - line[i][0])
    # the run is resumed ON the line here (the real --from here), then the dog is put beside it
    tmp = tempfile.mkdtemp(); work = os.path.join(tmp, "r"); shutil.copytree(route, work)
    acc, k0 = 0.0, 0
    for k_, g_ in enumerate(doc["segments"]):
        c_ = np.array(g_["centerline"])[:, :2]; L_ = float(np.sum(np.linalg.norm(np.diff(c_, axis=0), axis=1)))
        if acc + L_ > s0: k0 = k_; break
        acc += L_
    nc = core.NavCore(core.RouteBundle.from_dir(nav_run.start_here(work, float(line[i][0]), float(line[i][1]), th, k0)), cfg)
    x, y, yaw = line[i][0] - offset * math.sin(th), line[i][1] + offset * math.cos(th), th + math.radians(yaw_err_deg)
    # walk the projection up to s0 first (the follower projects forward from its last s)
    nc.follower.reset() if hasattr(nc.follower, "reset") else None
    obs = core.flat_observation(float(cfg["body_z_offset"])); lag = []; tmin = None; dmax = 0.0; r = None
    for k in range(int(seconds * 20)):
        t = k * 0.05
        r = nc.step(t, (x, y, 0.0, 0, 0, yaw, 0, 0), obs, gait if r is None or not r.gait_request else r.gait_request, 0.0)
        lag.append(r.command); vx, vy, w = lag[-7] if len(lag) > 7 else (0, 0, 0)
        x += vx * math.cos(yaw) * 0.05; y += vx * math.sin(yaw) * 0.05; yaw += w * 0.05        # vy ignored on purpose
        d = r.status.get("d")
        if d is not None and k > 20:
            dmax = max(dmax, abs(d))
            if abs(d) < 0.15 and r.mode == "WALK" and tmin is None: tmin = t
        if r.mode == "HOLD" or r.finished: break
    got = "HOLD" if r.mode == "HOLD" else ("BACK_ON_LINE" if tmin is not None else "STILL_OFF")
    ok = got == expect
    print("%-58s -> %-12s %s (back on the line after %s s, largest offset %.2f m, end mode %s%s)" % (
        name, got, "ok" if ok else "EXPECTED " + expect, "%.1f" % tmin if tmin is not None else "-", dmax, r.mode, ": " + str(r.status.get("why") or r.reason)[:60] if r.mode == "HOLD" else ""))
    return ok


zones = [(a, b) for a, b, n in json.load(open(os.path.join(route, "terrain.json")))["gaits"]]
z = zones[1]; flat_s = (zones[2][1] + 25.0)
res = [
    case("stairs zone, 0.70 m beside the line (18:31 run)", z[0] + 8.0, -0.70, 0, "stairs", "BACK_ON_LINE"),
    case("flat, 1.10 m beside the line, heading 25 deg off (16:57 run)", flat_s, -1.10, 25, "flat", "BACK_ON_LINE"),
    case("flat, 0.90 m beside the line, other side", flat_s + 20, 0.90, -20, "flat", "BACK_ON_LINE"),
    case("flat, 1.80 m beside the line: too far, must HOLD", flat_s, 1.80, 0, "flat", "HOLD"),
]
# information only: in this kinematic model the follower often steers back by itself even with readjust off; on the robot
# (2026-09-21 18:31) the old TRACK mode stood still for 8 s (forward speed walk_floor = 0, side-step ignored) and held.
case("(info) the first case with readjust switched OFF", z[0] + 8.0, -0.70, 0, "stairs", "HOLD", enabled=False)
print("READJUST_TEST_OK" if all(res) else "READJUST_TEST_FAILED"); sys.exit(0 if all(res) else 1)
