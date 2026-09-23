#!/usr/bin/env python3
"""Demo video: the operator's taught trail against the straightened teach line, with a simulated run.

  python3 tools/make_demo_video.py <session_dir> <teach_line_dir> <route_dir> --map <global_map_downsize.pcd> --out demo.mp4

Left: whole course (map, taught trails, new line coloured by gait, waypoints with their touch circle, the
simulated robot). Right: a 12 m window that follows the robot, where the difference between the zig-zag the
operator drove and the line the robot follows is visible. The run is the kinematic simulation of
tests/nav/sim_full_route.py (real NavCore, synthetic perception): it shows the PLAN, not a field result."""
import argparse  # noqa: I001  (matplotlib Agg must be selected before pyplot imports)
import json
import math
import os
import sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "nav")); sys.path.insert(0, HERE)
import nav_core as core
from audit_teach_session import read_trail


def pcd_xy(path, every=4):
    b = open(path, "rb").read(); j = b.find(b"\n", b.find(b"DATA")) + 1
    H = {l.split()[0]: l.split()[1:] for l in b[:j].decode(errors="ignore").splitlines() if l and not l.startswith("#")}
    n = int(H["POINTS"][0]); step = sum(int(s) * int(c) for s, c in zip(H["SIZE"], H["COUNT"]))
    return np.frombuffer(b[j:j + n * step], dtype=np.uint8).reshape(n, step)[:, :12].copy().view(np.float32).reshape(n, 3)[::every, :2]


def simulate(route, v, vs):
    cfg = core.load_yaml(os.path.join(HERE, "..", "config", "nav.yaml"))
    cfg.update(flat_speed_override=v, stairs_speed_override=vs); cfg["runner_params"]["walk_v"] = v; cfg["gains"]["max_forward"] = max(v, cfg["gains"]["max_forward"])
    nc = core.NavCore(core.RouteBundle.from_dir(route), cfg)
    w0 = nc.route.waypoints[0]; x, y, yaw = float(w0.xy[0]), float(w0.xy[1]), float(w0.yaw); obs = core.flat_observation(float(cfg["body_z_offset"]))
    gait, since, log = "flat", None, []
    for k in range(20 * 1500):
        t = k * 0.05
        r = nc.step(t, (x, y, 0.0, 0, 0, yaw, 0, 0), obs, gait, 0.0); vx, vy, w = r.command
        if r.gait_request and r.gait_request != gait:
            since = t if since is None else since
            if t - since > 1.5: gait, since = r.gait_request, None
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * 0.05; y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * 0.05; yaw += w * 0.05
        if k % 10 == 0 or r.finished or r.mode == "HOLD": log.append((t, x, y, yaw, vx, gait, nc.reached, r.mode))
        if r.finished or r.mode == "HOLD": break
    return nc, log


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("session"); ap.add_argument("teach_line"); ap.add_argument("route")
    ap.add_argument("--map", required=True); ap.add_argument("--out", required=True); ap.add_argument("--speed", type=float, default=0.8); ap.add_argument("--stairs-speed", type=float, default=0.5)
    ap.add_argument("--stride", type=int, default=2, help="one video frame per this many 0.5 s simulation samples")
    a = ap.parse_args()
    rep = json.load(open(os.path.join(a.teach_line, "teach_line.json")))
    demos = {d: read_trail(os.path.join(a.session, d + ".trail.csv")) for d in rep["demos"]}
    doc = json.load(open(os.path.join(a.route, "route_v2.json")))
    line = np.vstack([np.array(s["centerline"])[:, :2] for s in doc["segments"]]); stairs = np.concatenate([[s["gait"] == "stairs"] * len(s["centerline"]) for s in doc["segments"]])
    marks = [json.loads(l) for l in open(os.path.join(a.teach_line, "derived_session", "marks.jsonl"))]
    wps = [(m["wp_id"], m.get("wp_true", m["pose"])[:2], m["pose"][:2]) for m in marks if m["kind"] == "WP"]
    cloud = pcd_xy(a.map)
    nc, log = simulate(a.route, a.speed, a.stairs_speed)
    print("simulated: %d WPs of %d, %.0f s, end %s" % (log[-1][6], nc.n_wp, log[-1][0], log[-1][7]))
    fig, (axL, axR) = plt.subplots(1, 2, figsize=(16, 8), gridspec_kw=dict(width_ratios=[1.35, 1])); fig.patch.set_facecolor("white")
    for ax in (axL, axR):
        ax.scatter(cloud[:, 0], cloud[:, 1], s=0.05, c="#c8c8c8", linewidths=0, zorder=0)
        for d, t in demos.items(): ax.plot(t[:, 1], t[:, 2], lw=0.8, c="#f08a24", alpha=0.75, zorder=1)
        ax.scatter(line[~stairs, 0], line[~stairs, 1], s=2.5, c="#0a58ca", linewidths=0, zorder=2); ax.scatter(line[stairs, 0], line[stairs, 1], s=2.5, c="#d62728", linewidths=0, zorder=2)
        for wid, true, gate in wps:
            ax.add_patch(plt.Circle(true, rep["params"]["reach"], fill=False, ec="#2ca02c", lw=0.8, zorder=3)); ax.plot(*true, ".", c="k", ms=3, zorder=3)
        ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    for wid, true, gate in wps: axL.annotate(wid[2:], (true[0] + 0.8, true[1] + 0.8), fontsize=7)
    pad = 6; axL.set_xlim(line[:, 0].min() - pad, line[:, 0].max() + pad); axL.set_ylim(line[:, 1].min() - pad, line[:, 1].max() + pad)
    axL.plot([], [], c="#f08a24", label="operator's taught trails (%d runs)" % len(demos)); axL.plot([], [], c="#0a58ca", label="straightened line, flat gait"); axL.plot([], [], c="#d62728", label="straightened line, stairs gait")
    axL.plot([], [], "o", mfc="none", mec="#2ca02c", label="waypoint touch circle (%.2f m)" % rep["params"]["reach"]); axL.legend(loc="upper left", fontsize=8, framealpha=0.9)
    (dotL,) = axL.plot([], [], "o", ms=9, c="k", zorder=5); (trailL,) = axL.plot([], [], lw=2.0, c="k", alpha=0.6, zorder=4)
    body = plt.Polygon(np.zeros((4, 2)), closed=True, fc="#222222", ec="w", lw=0.8, zorder=6); axR.add_patch(body); (trailR,) = axR.plot([], [], lw=1.6, c="k", alpha=0.7, zorder=5)
    title = fig.suptitle("", fontsize=12)
    b0 = rep["before"]; bf = list(b0.values())
    sub = "zig-zag removed: straight share %.0f-%.0f %% -> %.0f %%, total turning %d-%d deg -> %d deg, %d of %d waypoints touched in the simulation" % (
        100 * min(v["straight_fraction"] for v in bf), 100 * max(v["straight_fraction"] for v in bf), 100 * rep["after"]["straight_fraction"],
        min(v["total_turning_deg"] for v in bf), max(v["total_turning_deg"] for v in bf), rep["after"]["total_turning_deg"], log[-1][6], nc.n_wp)
    fig.text(0.5, 0.02, sub + "   (kinematic simulation of the plan, not a field run)", ha="center", fontsize=9)
    hl, hw = 0.45, 0.25; corners = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    xs, ys = [], []
    writer = FFMpegWriter(fps=25, bitrate=2400)
    with writer.saving(fig, a.out, dpi=90):
        for i, (t, x, y, yaw, vx, gait, reached, mode) in enumerate(log):
            xs.append(x); ys.append(y)
            if i % a.stride: continue
            dotL.set_data([x], [y]); trailL.set_data(xs, ys); trailR.set_data(xs, ys)
            R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]); body.set_xy(corners @ R.T + [x, y])
            axR.set_xlim(x - 6, x + 6); axR.set_ylim(y - 6, y + 6)
            title.set_text("S10 teach-and-repeat plan   t = %3.0f s   %.2f m/s   gait: %s   waypoints %d / %d" % (t, vx, gait, reached, nc.n_wp))
            writer.grab_frame()
    print("wrote", a.out)


if __name__ == "__main__":
    main()
