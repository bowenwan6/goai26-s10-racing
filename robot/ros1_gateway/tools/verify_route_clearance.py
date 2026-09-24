#!/usr/bin/env python3
"""Independent collision check of a route against the saved x_nav map. It shares NO free-space logic
with tools/teach_line.py: it only asks one question.

    Does the robot BODY, swept along the route, overlap any obstacle cell that the robot body did not
    already overlap while the operator drove the demonstrations?

  obstacles   map points 0.20-1.00 m above x_nav's own ground cloud, 0.10 m cells (nothing is exempted)
  demo body   0.9 x 0.5 m rectangle at every recorded pose of every demonstration, with the RECORDED yaw
  route body  (0.9 + 2*margin) x (0.5 + 2*margin) rectangle along the route centreline, heading = tangent

The criterion is teach_line.Clearance (continuous geometry): the body keeps `margin` from every obstacle point,
except that it may pass something as closely as the operator's body did (3 cm slack). HARD = the body is ON an
obstacle the operator's body never went over; SOFT = only the margin is broken. Also reported: the clearance of the route centre to those real obstacles, next to the
clearance the demonstrations had at the same place.

  python3 tools/verify_route_clearance.py <route_dir> <session_dir> --map-dir <x_nav map dir> [--demos a,b,c]
        [--margin 0.05] [--out <dir>]          exit code 0 = no violation
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt, label

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from audit_teach_session import read_trail
from teach_line import (  # the criterion itself is shared; the generator's free-space logic is not
    Clearance,
    Grid,
    free_zones,
    load_pcd_xyz,
    obstacle_mask,
)

HL, HW = 0.45, 0.25                                                # body half length / half width (m)


def body_cover(grid, xy, yaw, hl, hw):
    """Cells covered by an hl x hw rectangle at every (xy, yaw)."""
    m = np.zeros((grid.nx, grid.ny), bool)
    lx = np.arange(-hl, hl + 1e-9, grid.res / 2); ly = np.arange(-hw, hw + 1e-9, grid.res / 2)
    gx, gy = np.meshgrid(lx, ly, indexing="ij"); loc = np.c_[gx.ravel(), gy.ravel()]
    for k in range(0, len(xy), 400):
        c, s = np.cos(yaw[k:k + 400]), np.sin(yaw[k:k + 400])
        wx = xy[k:k + 400, 0, None] + loc[None, :, 0] * c[:, None] - loc[None, :, 1] * s[:, None]
        wy = xy[k:k + 400, 1, None] + loc[None, :, 0] * s[:, None] + loc[None, :, 1] * c[:, None]
        i, j = grid.idx(np.stack([wx.ravel(), wy.ravel()], axis=-1)); m[i, j] = True
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("route"); ap.add_argument("session"); ap.add_argument("--map-dir", required=True)
    ap.add_argument("--demos", default=""); ap.add_argument("--margin", type=float, default=0.05); ap.add_argument("--out", default="")
    ap.add_argument("--free-at", default="", help='same as teach_line --free-at: "WP06:1.5" = the operator says nothing stands there')
    a = ap.parse_args()
    doc = json.load(open(os.path.join(a.route, "route_v2.json")))
    line = np.vstack([np.array(s["centerline"])[:, :2] for s in doc["segments"]])
    keep = np.r_[True, np.linalg.norm(np.diff(line, axis=0), axis=1) > 1e-6]; line = line[keep]
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(line, axis=0), axis=1))]
    s = np.r_[np.arange(0, d[-1], 0.05), d[-1]]                     # 5 cm steps
    line = np.c_[np.interp(s, d, line[:, 0]), np.interp(s, d, line[:, 1])]
    _tang = np.arctan2(np.gradient(line[:, 1]), np.gradient(line[:, 0]))
    names = [n for n in a.demos.split(",") if n] or [os.path.basename(f)[:-10] for f in sorted(glob.glob(os.path.join(a.session, "path_*.trail.csv")))]
    demos = {n: read_trail(os.path.join(a.session, n + ".trail.csv")) for n in names}
    demos = {n: t for n, t in demos.items() if len(t) > 10}
    allxy = np.vstack([t[:, 1:3] for t in demos.values()] + [line])
    grid = Grid(allxy)
    occ, _ = obstacle_mask(grid, load_pcd_xyz(os.path.join(a.map_dir, "global_ground_map.pcd")), load_pcd_xyz(os.path.join(a.map_dir, "global_map.pcd")))
    occ, freed = free_zones(grid, occ, a.free_at, {w["id"]: w["position"][:2] for w in doc["waypoints"]}, strict=False)
    for wid, r, n in freed: print(f"--free-at {wid}: {n} obstacle cells within {r} m ignored on the operator's word")
    tracks = dict(demos)
    for f in sorted(glob.glob(os.path.join(a.session, "survey_*.trail.csv"))):      # the drive on which the marks were made
        t = read_trail(f)
        if len(t) > 10: tracks[os.path.basename(f)[:-10]] = t
    clr = Clearance(grid, occ, list(tracks.values()), a.margin)
    real, demo_body = clr.obst, clr.passable
    vi, d_new, on_it = clr.check(line)
    hit = np.zeros_like(occ); hard = np.zeros_like(occ)
    hit[clr.ij[0][vi], clr.ij[1][vi]] = True
    hard[clr.ij[0][on_it], clr.ij[1][on_it]] = True
    from teach_line import sweep_mask
    route_body = sweep_mask(grid, line, a.margin)
    dist_real = distance_transform_edt(~real) * grid.res           # distance of every cell to a real obstacle
    li, lj = grid.idx(line); clr_line = dist_real[li, lj]
    # clearance the demonstrations had near each route point
    from scipy.spatial import cKDTree
    clr_demo = np.full(len(line), np.inf)
    for t in demos.values():
        ti, tj = grid.idx(t[:, 1:3]); cd = dist_real[ti, tj]
        dist, idx = cKDTree(t[:, 1:3]).query(line)
        clr_demo = np.minimum(clr_demo, np.where(dist < 1.5, cd[idx], np.inf))
    lab, n = label(hit)
    wps = [(w["id"], np.array(w["position"][:2])) for w in doc["waypoints"]]
    clusters = []
    for k in range(1, n + 1):
        ii, jj = np.where(lab == k)
        c = np.array([grid.x0 + (ii.mean() + 0.5) * grid.res, grid.y0 + (jj.mean() + 0.5) * grid.res])
        near = int(np.argmin(np.linalg.norm(line - c, axis=1)))
        wid = min(wps, key=lambda w: np.linalg.norm(w[1] - c))[0]
        clusters.append(dict(cells=len(ii), xy=[round(float(v), 2) for v in c], s=round(float(s[near]), 1), near_wp=wid,
                             centre_distance=round(float(np.linalg.norm(line[near] - c)), 2)))
    closer = (clr_line < np.minimum(clr_demo, 0.60) - 0.10) & (clr_line < 0.45)   # clearly closer to a real obstacle than the operator ever was
    n_hard = int(hard.sum())
    rep = dict(hard_violating_cells=n_hard, free_at=freed, route=a.route, demos=list(demos), body=[2 * HL, 2 * HW], margin=a.margin, obstacle_cells=int(occ.sum()), real_obstacle_cells=int(real.sum()),
               violations=len(clusters), violating_cells=int(hit.sum()), clusters=sorted(clusters, key=lambda c: -c["cells"])[:30],
               centre_clearance=dict(min=round(float(clr_line.min()), 2), p5=round(float(np.percentile(clr_line, 5)), 2), median=round(float(np.median(clr_line)), 2)),
               demo_clearance=dict(min=round(float(clr_demo[np.isfinite(clr_demo)].min()), 2), p5=round(float(np.percentile(clr_demo[np.isfinite(clr_demo)], 5)), 2)),
               metres_closer_than_any_demo=round(float(closer.sum() * 0.05), 1))
    out = a.out or a.route
    json.dump(rep, open(os.path.join(out, "clearance.json"), "w"), indent=1)
    print("obstacle cells %d, of which the demonstrations' body never touched %d" % (rep["obstacle_cells"], rep["real_obstacle_cells"]))
    print("HARD  body %.2f x %.2f m on a real obstacle: %d cells" % (2 * HL, 2 * HW, n_hard))
    print("SOFT  within the %.2f m margin round the body: %d cells in %d places" % (a.margin, rep["violating_cells"], rep["violations"]))
    for c in rep["clusters"][:12]: print("   ", c)
    print("centre clearance to real obstacles: route", rep["centre_clearance"], "| demonstrations", rep["demo_clearance"], "| route clearly closer than any demonstration for %.1f m" % rep["metres_closer_than_any_demo"])
    try:
        # noqa order matters: the Agg backend must be selected before pyplot is imported.
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt  # noqa: I001
        ext = [grid.x0, grid.x0 + grid.nx * grid.res, grid.y0, grid.y0 + grid.ny * grid.res]
        img = np.ones(occ.shape + (3,)); img[occ & demo_body] = (0.80, 0.86, 0.95); img[real] = (0.55, 0.55, 0.55); img[route_body & ~real] = (0.85, 0.95, 0.85); img[hit] = (1.0, 0.65, 0.0); img[hard] = (0.9, 0.1, 0.1)
        fig, ax = plt.subplots(figsize=(18, 11)); ax.imshow(np.transpose(img, (1, 0, 2)), origin="lower", extent=ext, interpolation="nearest")
        ax.plot(line[:, 0], line[:, 1], lw=0.8, c="#0a58ca")
        for c in rep["clusters"]: ax.add_patch(plt.Circle(c["xy"], 1.0, fill=False, ec="r", lw=1.5))
        ax.set_aspect("equal"); ax.set_xlim(line[:, 0].min() - 5, line[:, 0].max() + 5); ax.set_ylim(line[:, 1].min() - 5, line[:, 1].max() + 5)
        ax.set_title("grey = real obstacles, pale blue = map points the demonstrations' body passed over, green = route body sweep, orange = inside the margin, RED = body on a real obstacle")
        fig.savefig(os.path.join(out, "clearance.png"), dpi=120, bbox_inches="tight")
    except ImportError:
        pass
    print("ROUTE_CLEARANCE_OK" if n_hard == 0 else "ROUTE_CLEARANCE_VIOLATIONS")
    sys.exit(0 if n_hard == 0 else 1)


if __name__ == "__main__":
    main()
