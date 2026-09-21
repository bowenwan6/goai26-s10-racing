#!/usr/bin/env python3
"""Audit a /teach session: marks, taught paths, and the gait the operator actually used.

  python3 tools/audit_teach_session.py <session_dir> [--control-logs <dir with control-events-*.jsonl>]
        [--map <global_map_downsize.pcd>] [--out <dir>]

Prints one block per WP/switch mark and per path recording, writes audit.json and (with matplotlib)
audit.png: the map from above, WPs, switch marks, every taught path coloured by gait.
The gait comes from s10_ros1_control's event log (gait_change from /MOTION_INFO), matched by wall time.
"""
import argparse, csv, glob, json, math, os, sys
import numpy as np

GAIT = {0x1001: "basic", 0x1003: "stairs", 0x3002: "nav_flat", 0x3003: "nav_stairs", 0: "none"}


def read_marks(p):
    rows = [json.loads(l) for l in open(p) if l.strip()]
    void = {r.get("target") for r in rows if r.get("kind") == "VOID"}
    return rows, [r for r in rows if r.get("kind") != "VOID" and r.get("seq") not in void]


def read_trail(p):
    a = np.array([[float(r["wall_time"]), float(r["x"]), float(r["y"]), float(r["z"]), float(r["yaw"])] for r in csv.DictReader(open(p))])
    return a if len(a) else np.zeros((0, 5))


def gait_timeline(logdir):
    ev = []
    for f in sorted(glob.glob(os.path.join(logdir, "control-events-*.jsonl"))):
        for l in open(f):
            try: e = json.loads(l)
            except ValueError: continue
            if e.get("event") == "gait_change": ev.append((e["wall_ns"] / 1e9, int(e["detail"]["to"])))
            if e.get("event") == "state_change": ev.append((e["wall_ns"] / 1e9, -int(e["detail"]["to"])))   # negative = motion state
    return sorted(ev)


def gait_at(timeline, t):
    g = None
    for tt, v in timeline:
        if tt > t: break
        if v >= 0: g = v
    return g


def turn_stats(xy, step=0.25):
    """Resample at `step`, heading change between consecutive chords: how zig-zag the operator drove."""
    if len(xy) < 3: return dict(length=0.0)
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    s = np.arange(0, d[-1], step)
    p = np.c_[np.interp(s, d, xy[:, 0]), np.interp(s, d, xy[:, 1])]
    h = np.arctan2(*np.diff(p, axis=0).T[::-1])
    dh = np.abs(np.arctan2(np.sin(np.diff(h)), np.cos(np.diff(h))))
    straight = float(np.mean(dh < math.radians(3)))
    return dict(length=round(float(d[-1]), 1), kinks_over_15deg=int((dh > math.radians(15)).sum()),
                kinks_over_30deg=int((dh > math.radians(30)).sum()), straight_fraction=round(straight, 2),
                total_turning_deg=int(math.degrees(dh.sum())))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session"); ap.add_argument("--control-logs", default=""); ap.add_argument("--map", default=""); ap.add_argument("--out", default="")
    a = ap.parse_args()
    out = a.out or a.session
    rows, live = read_marks(os.path.join(a.session, "marks.jsonl"))
    tl = gait_timeline(a.control_logs) if a.control_logs else []
    wps, sw = {}, []
    for r in live:
        ok = (r.get("result") or {}).get("passed")
        if r["kind"] == "WP" and ok: wps[r["wp_id"]] = r          # last passing mark wins
        if r["kind"] in ("SWIN", "SWOUT") and ok: sw.append(r)
    report = dict(session=a.session, marks_total=len(rows), marks_live=len(live), waypoints={}, switch_marks=[], paths={}, findings=[])
    ids = sorted(wps)
    missing = [f"WP{i:02d}" for i in range(1, int(ids[-1][2:]) + 1) if f"WP{i:02d}" not in wps] if ids else []
    if missing: report["findings"].append(f"missing WPs in the sequence: {missing}")
    print(f"== marks: {len(rows)} rows, {len(live)} live, {len(wps)} WPs {ids[0]}..{ids[-1]}, {len(sw)} switch marks; missing {missing or 'none'}")
    prev = None
    for w in ids:
        p = wps[w]["pose"]; n_try = sum(1 for r in rows if r.get("wp_id") == w)
        leg = math.hypot(p[0] - prev[0], p[1] - prev[1]) if prev else 0.0
        report["waypoints"][w] = dict(pose=[round(v, 3) for v in p], tries=n_try, leg_from_prev_m=round(leg, 2), dz_from_prev=round(p[2] - prev[2], 2) if prev else 0.0,
                                      std_xy=(wps[w]["result"] or {}).get("std_xy"))
        prev = p
    for r in sw:
        report["switch_marks"].append(dict(seq=r["seq"], kind=r["kind"], pose=[round(v, 2) for v in r["pose"]]))
    trails = sorted(glob.glob(os.path.join(a.session, "path_*.trail.csv")))
    for f in trails:
        name = os.path.basename(f)[:-10]; t = read_trail(f)
        if len(t) < 2:
            report["paths"][name] = dict(points=len(t)); continue
        xy = t[:, 1:3]; st = turn_stats(xy)
        passed = []
        for w in ids:
            dmin = np.min(np.linalg.norm(xy - np.array(wps[w]["pose"][:2]), axis=1)); i = int(np.argmin(np.linalg.norm(xy - np.array(wps[w]["pose"][:2]), axis=1)))
            if dmin < 0.6: passed.append((i, w, round(float(dmin), 2)))
        passed.sort()
        order = [w for _, w, _ in passed]
        gaits = []
        if tl:
            g_prev = None
            for k in range(len(t)):
                g = gait_at(tl, t[k, 0])
                if g != g_prev:
                    gaits.append(dict(t=round(t[k, 0] - t[0, 0], 1), gait=GAIT.get(g, hex(g) if g is not None else "?"), xy=[round(t[k, 1], 2), round(t[k, 2], 2)]))
                    g_prev = g
        stops = [v for tt, v in tl if t[0, 0] <= tt <= t[-1, 0] and v in (-2, -0, -4)] if tl else []
        jumps = int((np.linalg.norm(np.diff(xy, axis=0), axis=1) > 0.5).sum())
        report["paths"][name] = dict(points=len(t), duration_s=round(t[-1, 0] - t[0, 0], 1), **st, start=[round(v, 2) for v in t[0, 1:4]], end=[round(v, 2) for v in t[-1, 1:4]],
                                     z_range=[round(float(t[:, 3].min()), 2), round(float(t[:, 3].max()), 2)], wps_passed=order, wps_in_order=order == sorted(order),
                                     worst_wp_miss=max([d for _, _, d in passed], default=None), pose_jumps_over_0p5m=jumps, gait_segments=gaits, robot_fell_or_damped=bool(stops))
    json.dump(report, open(os.path.join(out, "audit.json"), "w"), indent=1, ensure_ascii=False)
    for n, p in report["paths"].items():
        if p.get("points", 0) < 2: print(f"-- {n}: empty"); continue
        print(f"-- {n}: {p['duration_s']} s, {p['length']} m, WPs {p['wps_passed'][0] if p['wps_passed'] else '-'}..{p['wps_passed'][-1] if p['wps_passed'] else '-'} ({len(p['wps_passed'])}), in order {p['wps_in_order']}, "
              f"kinks>15deg {p['kinks_over_15deg']}, >30deg {p['kinks_over_30deg']}, straight {p['straight_fraction']}, jumps {p['pose_jumps_over_0p5m']}, fell/damped {p['robot_fell_or_damped']}, gait changes {max(0, len(p['gait_segments']) - 1)}")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, ax = plt.subplots(figsize=(16, 11))
    if a.map and os.path.exists(a.map):
        b = open(a.map, "rb").read(); j = b.find(b"\n", b.find(b"DATA")) + 1
        H = {l.split()[0]: l.split()[1:] for l in b[:j].decode(errors="ignore").splitlines() if l and not l.startswith("#")}
        n = int(H["POINTS"][0]); step = sum(int(s) * int(c) for s, c in zip(H["SIZE"], H["COUNT"]))
        m = np.frombuffer(b[j:j + n * step], dtype=np.uint8).reshape(n, step)[:, :12].copy().view(np.float32).reshape(n, 3)
        ax.scatter(m[::3, 0], m[::3, 1], s=0.05, c="#bbbbbb", linewidths=0)
    col = {"basic": "#1f77b4", "stairs": "#d62728", None: "#555555"}
    for f in trails:
        name = os.path.basename(f)[:-10]; t = read_trail(f)
        if len(t) < 2: continue
        g = np.array([gait_at(tl, tt) if tl else None for tt in t[:, 0]], dtype=object)
        for gv in set(g.tolist()):
            msk = g == gv
            ax.scatter(t[msk, 1], t[msk, 2], s=1.5, c=col.get(GAIT.get(gv), "#555555"), linewidths=0)
        ax.annotate(name[-6:], t[0, 1:3], fontsize=7, color="#333333")
    for w in ids:
        p = wps[w]["pose"]; ax.plot(p[0], p[1], "o", ms=6, mfc="none", mec="k"); ax.annotate(w[2:], (p[0] + .4, p[1] + .4), fontsize=9, weight="bold")
    for r in sw:
        ax.plot(r["pose"][0], r["pose"][1], "^" if r["kind"] == "SWIN" else "v", ms=9, c="#2ca02c")
    ax.set_aspect("equal"); ax.set_title("taught paths by operator gait (blue basic 0x1001, red stairs 0x1003); o WP, green ^ SWIN v SWOUT"); ax.grid(alpha=.3)
    fig.savefig(os.path.join(out, "audit.png"), dpi=110, bbox_inches="tight")


if __name__ == "__main__":
    main()
