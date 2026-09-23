"""Review sheets for the photo-matched waypoints: photo | local map | height profile.

Run after match_wps.py:
  python3 tools/wp_match/render_review.py --out tools/wp_match/out
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import match_wps as m
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
from PIL import Image
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
PHOTO_DIR = HERE.parents[1] / "waypoint-photos-20260914" / "preview-jpg"
GAIT_COLOR = {"flat": "#1f77b4", "stairs": "#d62728"}


def preview_for(photo: str) -> Path:
    stem = photo.replace(".HEIC", "").replace(" ", "-")
    hits = sorted(PHOTO_DIR.glob(f"*__{stem}.jpg"))
    return hits[0]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=HERE / "out")
    ap.add_argument("--raw", type=Path, default=m.DEFAULT_RAW)
    args = ap.parse_args()
    route = json.loads((args.out / "route_v2.json").read_text())
    wps = json.loads((args.out / "wp30_v3_matched.json").read_text())
    segs = route["segments"]

    cloud = m.load_pcd_xyz(args.raw / "full_cloud.pcd")
    path_all = np.vstack([np.array(s["centerline"]) for s in segs])
    d, _ = cKDTree(path_all[:, :2]).query(cloud[:, :2], distance_upper_bound=12.0)
    near = cloud[np.isfinite(d)]
    # Drop canopy / building overhang for the plan view: keep points within 1.2 m above the route.
    dz_ref = cKDTree(path_all[:, :2]).query(near[:, :2])[1]
    near = near[near[:, 2] < path_all[dz_ref, 2] + 1.2]

    # Overview.
    fig, ax = plt.subplots(figsize=(16, 11), dpi=110)
    sel = near[:: max(1, len(near) // 600000)]
    ax.scatter(sel[:, 0], sel[:, 1], c=sel[:, 2], s=0.2, cmap="terrain", vmin=-1, vmax=7, linewidths=0)
    for s in segs:
        c = np.array(s["centerline"])
        ax.plot(c[:, 0], c[:, 1], color=GAIT_COLOR[s["gait"]], lw=2.2)
    for w in route["waypoints"]:
        x, y, _ = w["position"]
        ax.add_patch(Circle((x, y), w["uncertainty_radius_m"], fill=False, ec="k", lw=0.8, ls="--"))
        ax.plot(x, y, "o", ms=6, mfc="yellow", mec="k")
        ax.annotate(w["id"][2:], (x, y), xytext=(4, 4), textcoords="offset points", fontsize=9,
                    weight="bold", bbox=dict(boxstyle="round,pad=0.15", fc="white", alpha=0.8))
    ax.plot([], [], color=GAIT_COLOR["flat"], lw=2.2, label="flat gait (0x3002)")
    ax.plot([], [], color=GAIT_COLOR["stairs"], lw=2.2, label="stairs gait (0x3003)")
    ax.legend(loc="lower right")
    ax.set_aspect("equal")
    ax.set_title("v3 map: 30 photo-matched WPs (official order, WP01 = door Start). "
                 "Dashed circle = position uncertainty. Colour = ground height (m)")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    fig.tight_layout()
    fig.savefig(args.out / "overview_route_v2.png")
    plt.close(fig)

    # Per-WP sheets, 5 WPs per page.
    tree = cKDTree(near[:, :2])
    incoming = {s["to"]: s for s in segs}
    outgoing = {s["from"]: s for s in segs}
    for page in range(6):
        fig, axes = plt.subplots(5, 3, figsize=(15, 24), dpi=90,
                                 gridspec_kw=dict(width_ratios=[0.8, 1.0, 1.1]))
        for row, w in enumerate(wps[page * 5:(page + 1) * 5]):
            a0, a1, a2 = axes[row]
            a0.imshow(Image.open(preview_for(w["photo"])))
            a0.set_title(f"{w['id']}  {w['photo']}  ({w['confidence']}, ±{w['uncertainty_radius_m']} m)",
                         fontsize=10)
            a0.axis("off")

            x, y, gz = w["position"]
            idx = tree.query_ball_point([x, y], 6.0)
            loc = near[idx]
            loc = loc[np.abs(loc[:, 2] - gz) < 1.5]
            a1.scatter(loc[:, 0], loc[:, 1], c=loc[:, 2] - gz, s=1, cmap="RdYlBu_r", vmin=-0.6,
                       vmax=0.6, linewidths=0)
            for s in (incoming.get(w["id"]), outgoing.get(w["id"])):
                if s:
                    c = np.array(s["centerline"])
                    a1.plot(c[:, 0], c[:, 1], color=GAIT_COLOR[s["gait"]], lw=2)
            rx, ry, _ = w["robot_pose_xyz"]
            h = w["robot_heading_in_mapping_run"]
            a1.arrow(rx, ry, 0.8 * np.cos(h), 0.8 * np.sin(h), width=0.08, color="k")
            a1.add_patch(Circle((x, y), w["uncertainty_radius_m"], fill=False, ec="m", lw=1.5, ls="--"))
            a1.plot(x, y, "*", ms=14, mfc="yellow", mec="k")
            a1.set_xlim(x - 6, x + 6)
            a1.set_ylim(y - 6, y + 6)
            a1.set_aspect("equal")
            a1.set_title(f"map ({x:.1f}, {y:.1f}) ground z={gz:.2f}  terrain={w['terrain']}\n"
                         "colour = height rel. WP ground (±0.6 m); arrow = robot at photo time",
                         fontsize=9)

            for s, sign in ((incoming.get(w["id"]), -1), (outgoing.get(w["id"]), 1)):
                if not s:
                    continue
                c = np.array(s["centerline"])
                ds = np.r_[0, np.cumsum(np.linalg.norm(np.diff(c[:, :2], axis=0), axis=1))]
                sx = ds - ds[-1] if sign < 0 else ds
                a2.plot(sx, c[:, 2], color=GAIT_COLOR[s["gait"]], lw=2,
                        label=f"{s['id']} {s['gait']} step={s['max_step_m']} grade={s['max_grade']}")
            a2.axvline(0, color="k", lw=0.8)
            a2.set_xlabel("distance along route from this WP (m)")
            a2.set_ylabel("ground z (m)")
            a2.legend(fontsize=8, loc="best")
            a2.grid(alpha=0.3)
        for rest in range(len(wps[page * 5:(page + 1) * 5]), 5):
            for a in axes[rest]:
                a.axis("off")
        fig.tight_layout()
        fig.savefig(args.out / f"review_WP{page * 5 + 1:02d}-WP{page * 5 + 5:02d}.jpg",
                    pil_kwargs=dict(quality=80))
        plt.close(fig)
    print("review sheets written to", args.out)


if __name__ == "__main__":
    main()
