"""Static check of a route_v2 against the course heightfield (no simulation).

For each segment: footprint (0.9 x 0.5 m, heading along the centerline) collisions with the
obstacle layer, unknown cells under the footprint, max footprint step, and the
unknown/obstacle share of the allowed corridor band.

    python -m sim_full_course.route_check [--route route_v2.json] [--json out.json]
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from sim_full_course.harness import TERRAIN_NPZ, default_route_path
from sim_full_course.robot import KinematicRobot
from sim_full_course.route import Route
from sim_full_course.terrain import Terrain


def check(route: Route, terrain: Terrain, ds: float = 0.1) -> list[dict]:
    bot = KinematicRobot(terrain, *route.point_at(0)[0][:2], 0.0)
    out = []
    for i, seg in enumerate(route.segments):
        a, b = route.seg_s0[i], route.wp_s[i + 1]
        bot.state.gait = seg["gait"]
        coll_s, max_step, unk, sv = [], 0.0, 0.0, 0
        band_unknown, band_obst, n_band = 0, 0, 0
        hw = seg["corridor_half_width"]
        for s in np.arange(a, b + 1e-9, ds):
            (x, y, _), h = route.point_at(s)
            c = bot.check(x, y, h)
            if c["collision"]:
                coll_s.append(round(float(s), 1))
            max_step = max(max_step, c["step"])
            unk = max(unk, c["unknown_frac"])
            sv += c["step_violation"]
            for off in np.arange(-hw, hw + 1e-9, 0.1):
                px, py = x - off * math.sin(h), y + off * math.cos(h)
                n_band += 1
                band_unknown += not bool(terrain.known_at(px, py))
                band_obst += bool(terrain.static_obstacle_at(px, py))
        out.append({
            "segment": seg["id"], "gait": seg["gait"], "s": [round(float(a), 1), round(float(b), 1)],
            "footprint_collision_samples": len(coll_s),
            "collision_s": _ranges(coll_s),
            "max_footprint_step_m": round(max_step, 3),
            "step_limit_violations": int(sv),
            "max_footprint_unknown_frac": round(unk, 3),
            "corridor_unknown_frac": round(band_unknown / max(n_band, 1), 3),
            "corridor_obstacle_frac": round(band_obst / max(n_band, 1), 3),
        })
    return out


def _ranges(vals, gap=0.25):
    rs = []
    for v in vals:
        if rs and v - rs[-1][1] <= gap:
            rs[-1][1] = v
        else:
            rs.append([v, v])
    return rs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", type=Path, default=default_route_path())
    ap.add_argument("--terrain", type=Path, default=TERRAIN_NPZ)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    rep = check(Route.load(a.route), Terrain.load(a.terrain))
    for r in rep:
        flag = " <-- footprint hits obstacle layer" if r["footprint_collision_samples"] else ""
        print(f"{r['segment']:10s} {r['gait']:6s} s={r['s']} step={r['max_footprint_step_m']:.2f} "
              f"stepviol={r['step_limit_violations']:3d} unk={r['max_footprint_unknown_frac']:.2f} "
              f"corr_unk={r['corridor_unknown_frac']:.2f} corr_obst={r['corridor_obstacle_frac']:.2f}"
              f" coll={r['collision_s']}{flag}")
    if a.json:
        a.json.write_text(json.dumps({"route": str(a.route), "segments": rep}, indent=1))


if __name__ == "__main__":
    main()
