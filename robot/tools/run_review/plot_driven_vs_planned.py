#!/usr/bin/env python3
"""Draw a field run's driven path against the planned line and the offline map.

The ROS 1 runner logs, once per control tick, its distance along the planned route
and its lateral deviation from it:

    run WALK cmd [...] s=12.34 d=-0.07 follower=DETOUR/ ... gait=flat lane=return

That is enough to put the robot back on the map: the driven point is the planned
line at arc length ``s``, pushed sideways by ``d`` along the line's normal. No pose
topic is recorded during a run, so this reconstruction is the only way to see where
the robot actually went.

Usage:
    python plot_driven_vs_planned.py --log nav-YYYYmmdd-HHMMSS.log \
        --trail path_00000000_teachline.trail.csv --route route_v2.json \
        [--poses poses_base.txt] [--start-wp WP10] --out driven_vs_planned.png
"""
from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import numpy as np

RUN_RE = re.compile(
    r"\[INFO\] \[(?P<t>\d+\.\d+)\]: run (?P<mode>\w+) cmd \[[^\]]*\] "
    r"s=(?P<s>-?[\d.]+) d=(?P<d>-?[\d.]+) follower=(?P<foll>\S+) "
    r"seg_limit=(?P<seg>[\d.]+) plan_scale=(?P<ps>[\d.]+) gait=(?P<gait>\S+)"
    r"(?: lane=(?P<lane>\S+))?"
)
WP_RE = re.compile(r"\[INFO\] \[(?P<t>\d+\.\d+)\]: (?P<wp>WP\d+) reached \((?P<i>\d+)/(?P<n>\d+)\)")
STATE_COLOUR = {"RUNNING": "#1a73e8", "DETOUR": "#e8710a", "GATE": "#188038", "BLOCKED": "#d93025"}


def parse_log(path: Path):
    header, samples, reached = None, [], []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if header is None and "rl_nav ros1: {" in line:
            body = line.split("rl_nav ros1: ", 1)[1].split("}; shadow")[0] + "}"
            header = json.loads(body)
        m = RUN_RE.search(line)
        if m:
            g = m.groupdict()
            state = g["foll"].split("/")[0]
            samples.append(dict(t=float(g["t"]), s=float(g["s"]), d=float(g["d"]), state=state,
                                gait=g["gait"], lane=g["lane"] or "", mode=g["mode"]))
            continue
        m = WP_RE.search(line)
        if m:
            reached.append(dict(t=float(m["t"]), wp=m["wp"]))
    if not samples:
        raise SystemExit(f"no 'run ... s=... d=...' lines in {path}")
    return header or {}, samples, reached


def load_trail(path: Path):
    xy = np.array([[float(r["x"]), float(r["y"])] for r in csv.DictReader(path.open())])
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    return xy, seg, np.concatenate([[0.0], np.cumsum(seg)])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", type=Path, required=True)
    ap.add_argument("--trail", type=Path, required=True)
    ap.add_argument("--route", type=Path, required=True)
    ap.add_argument("--poses", type=Path, help="SLAM mapping poses, for map context")
    ap.add_argument("--start-wp", default=None, help="waypoint where s=0 (default: first reached)")
    ap.add_argument("--title", default=None)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()

    import matplotlib  # noqa: I001  Agg must be selected before pyplot imports
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    header, samples, reached = parse_log(a.log)
    xy, seg, arc = load_trail(a.trail)
    wp_xy = {w["id"]: np.array(w["position"][:2])
             for w in json.loads(a.route.read_text(encoding="utf-8"))["waypoints"]}

    start = a.start_wp or (reached[0]["wp"] if reached else None)
    i0 = int(np.argmin(np.linalg.norm(xy - wp_xy[start], axis=1))) if start in wp_xy else 0
    s0 = arc[i0]

    def on_line(s):
        i = int(np.clip(np.searchsorted(arc, s0 + s) - 1, 0, len(xy) - 2))
        f = (s0 + s - arc[i]) / max(seg[i], 1e-9)
        p = xy[i] + f * (xy[i + 1] - xy[i])
        t = (xy[i + 1] - xy[i]) / max(seg[i], 1e-9)
        return p, np.array([-t[1], t[0]])

    S = np.array([x["s"] for x in samples])
    D = np.array([x["d"] for x in samples])
    state = [x["state"] for x in samples]
    gait = [x["gait"] for x in samples]
    driven = np.array([on_line(s)[0] + d * on_line(s)[1] for s, d in zip(S, D)])
    dur = samples[-1]["t"] - samples[0]["t"]

    fig = plt.figure(figsize=(15, 10.5))
    gs = fig.add_gridspec(2, 1, height_ratios=[2.15, 1], hspace=0.22)

    ax = fig.add_subplot(gs[0])
    if a.poses and a.poses.exists():
        pz = np.loadtxt(a.poses, usecols=(1, 2))
        ax.scatter(pz[:, 0], pz[:, 1], s=1.5, c="#d5d5d5", zorder=1,
                   label="offline map (SLAM mapping trajectory)")
    ax.plot(xy[i0:, 0], xy[i0:, 1], "-", c="#5f6368", lw=2.4, zorder=2,
            label="planned line (taught → MuJoCo-checked)")
    for st in ("RUNNING", "DETOUR", "GATE", "BLOCKED"):
        m = [i for i, v in enumerate(state) if v == st]
        if m:
            ax.scatter(driven[m, 0], driven[m, 1], s=13, c=STATE_COLOUR[st], zorder=4,
                       label=f"driven · {st.lower()} ({len(m)} ticks)")
    for r in reached:
        p = wp_xy.get(r["wp"])
        if p is not None:
            ax.plot(*p, "o", mfc="none", mec="#202124", ms=9, mew=1.4, zorder=5)
    for r in ([reached[0], reached[-1]] if reached else []):
        p = wp_xy.get(r["wp"])
        if p is not None:
            ax.annotate(r["wp"], p, textcoords="offset points", xytext=(8, 8),
                        fontsize=9.5, weight="bold", zorder=6)
    ax.set_aspect("equal"); ax.grid(alpha=.25)
    ax.set_xlabel("x (m, map frame)"); ax.set_ylabel("y (m)")
    ax.set_title(a.title or (
        f"Driven path vs planned line — {a.log.stem}, {len(reached)} waypoints, "
        f"{S.max():.1f} m in {dur:.0f} s\n"
        "Driven path reconstructed from the runner's along-route distance and lateral deviation"),
        fontsize=12.5, weight="bold", loc="left")
    ax.legend(loc="best", fontsize=9, framealpha=.93, ncol=2)

    ax2 = fig.add_subplot(gs[1])
    runs, cur = [], 0
    for i in range(1, len(gait)):
        if gait[i] != gait[i - 1]:
            runs.append((cur, i, gait[i - 1])); cur = i
    runs.append((cur, len(gait) - 1, gait[-1]))
    for s_i, e_i, g in runs:
        if g == "stairs":
            ax2.axvspan(S[s_i], S[e_i], color="#fce8b2", zorder=0)
    ax2.axhline(0, c="#5f6368", lw=1.6, zorder=2)
    for th, col, lab in ((0.8, "#d93025", "off-route 0.80 m → recovery"),
                         (0.25, "#188038", "back-on-line 0.25 m")):
        for sgn in (1, -1):
            ax2.axhline(sgn * th, c=col, ls="--", lw=1.0, alpha=.75, zorder=2)
        ax2.text(S.max(), th, lab + " ", va="bottom", ha="right", fontsize=8.5, color=col)
    for st in ("RUNNING", "DETOUR", "GATE", "BLOCKED"):
        m = [i for i, v in enumerate(state) if v == st]
        if m:
            ax2.scatter(S[m], D[m], s=11, c=STATE_COLOUR[st], zorder=3)
    stairs_pct = sum(1 for g in gait if g == "stairs") * 100 // max(len(gait), 1)
    ax2.set_ylim(-0.95, 0.95); ax2.set_xlim(0, max(S.max(), 1e-6))
    ax2.grid(alpha=.25)
    ax2.set_xlabel("distance along the planned route (m)")
    ax2.set_ylabel("lateral deviation (m)")
    ax2.set_title(f"Lateral deviation stayed within {np.abs(D).max():.2f} m "
                  f"(mean {np.abs(D).mean():.3f} m); shaded = stairs gait ({stairs_pct} % of ticks)",
                  fontsize=11, loc="left")
    ax2.legend(handles=[Line2D([], [], marker="o", ls="", color=c, label=s.lower())
                        for s, c in STATE_COLOUR.items()],
               loc="lower right", fontsize=8.5, ncol=4, framealpha=.93)

    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=125, bbox_inches="tight", facecolor="white")
    print(f"{a.out}  |  {len(reached)} wp, {S.max():.1f} m, {dur:.0f} s, "
          f"max|d| {np.abs(D).max():.2f} m, detour {sum(1 for v in state if v=='DETOUR')}/{len(state)} ticks")


if __name__ == "__main__":
    main()
