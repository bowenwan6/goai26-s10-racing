"""route_v2 loader/geometry (contract: scratchpad route_v2_contract.md) and a PLACEHOLDER
route generated from the mapping trajectory until the real route_v2.json exists.

Route geometry: the segment centerlines are concatenated (duplicate joints dropped) into one
polyline with arc length s. `project` is windowed around a hint so self-crossings and
switchbacks don't make s jump.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from sim_full_course.io_utils import ARTIFACTS, MAP_ID, PKG

PLACEHOLDER_PATH = PKG / "routes" / "placeholder_route_v2.json"
GAIT_LIMITS = {"flat": 0.20, "stairs": 0.15}


class Route:
    def __init__(self, data: dict):
        if data.get("schema") != "s10_route_v2":
            raise ValueError("not an s10_route_v2 document")
        self.data = data
        self.waypoints = data["waypoints"]
        self.segments = data["segments"]
        ids = [w["id"] for w in self.waypoints]
        for a, b in zip(self.segments[:-1], self.segments[1:], strict=True):
            if a["to"] != b["from"]:
                raise ValueError(f"segment chain broken at {a['id']} -> {b['id']}")
        pts, seg_idx = [], []
        for i, seg in enumerate(self.segments):
            c = np.asarray(seg["centerline"], float)
            if pts and np.linalg.norm(c[0, :2] - pts[-1][-1, :2]) < 1e-6:
                c = c[1:]
            pts.append(c)
            seg_idx.append(np.full(len(c), i))
        self.xyz = np.vstack(pts)
        self.seg_of_pt = np.concatenate(seg_idx)
        d = np.linalg.norm(np.diff(self.xyz[:, :2], axis=0), axis=1)
        keep = np.r_[True, d > 1e-9]
        self.xyz, self.seg_of_pt = self.xyz[keep], self.seg_of_pt[keep]
        self.s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.xyz[:, :2], axis=0), axis=1))]
        self.length = float(self.s[-1])
        # Arc length of each waypoint = projection onto the chain (in order).
        self.wp_s = []
        hint = 0.0
        wp_by_id = {w["id"]: w for w in self.waypoints}
        for wid in [self.segments[0]["from"]] + [s["to"] for s in self.segments]:
            p = wp_by_id[wid]["position"]
            s_wp, _, _ = self.project(p[:2], hint, back=0.5, ahead=1e9)
            self.wp_s.append(s_wp)
            hint = s_wp
        self.wp_order = [self.segments[0]["from"]] + [s["to"] for s in self.segments]
        if ids != self.wp_order:
            raise ValueError("waypoint list order differs from the segment chain")
        # Segment start arc lengths (at their `from` WP).
        self.seg_s0 = np.array(self.wp_s[:-1])

    @classmethod
    def load(cls, path: Path | str) -> Route:
        return cls(json.loads(Path(path).read_text()))

    # ---------- geometry ----------
    def point_at(self, s: float):
        s = float(np.clip(s, 0, self.length))
        i = int(np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.s) - 2))
        f = (s - self.s[i]) / max(self.s[i + 1] - self.s[i], 1e-9)
        p = self.xyz[i] + f * (self.xyz[i + 1] - self.xyz[i])
        d = self.xyz[i + 1] - self.xyz[i]
        return p, math.atan2(d[1], d[0])

    def project(self, xy, s_hint: float | None = None, back: float = 1.0, ahead: float = 3.0):
        """Closest point on the chain -> (s, signed lateral (+left), vertex index)."""
        xy = np.asarray(xy, float)[:2]
        a, b = self.xyz[:-1, :2], self.xyz[1:, :2]
        if s_hint is None:
            lo, hi = 0, len(a)
        else:
            lo = max(int(np.searchsorted(self.s, s_hint - back)) - 1, 0)
            hi = min(int(np.searchsorted(self.s, s_hint + ahead)) + 1, len(a))
        a, b = a[lo:hi], b[lo:hi]
        ab = b - a
        L2 = np.maximum((ab**2).sum(1), 1e-12)
        f = np.clip(((xy - a) * ab).sum(1) / L2, 0, 1)
        q = a + f[:, None] * ab
        d = np.linalg.norm(xy - q, axis=1)
        k = int(np.argmin(d))
        i = lo + k
        s = self.s[i] + f[k] * (self.s[i + 1] - self.s[i])
        cross = ab[k, 0] * (xy[1] - a[k, 1]) - ab[k, 1] * (xy[0] - a[k, 0])
        lat = float(np.sign(cross) * d[k]) if d[k] > 0 else 0.0
        return float(s), lat, i

    def segment_index_at(self, s: float) -> int:
        return int(np.clip(np.searchsorted(self.seg_s0, s, side="right") - 1, 0,
                           len(self.segments) - 1))

    def segment_at(self, s: float) -> dict:
        return self.segments[self.segment_index_at(s)]

    def gait_boundaries(self):
        """[(s, from_wp_id, new_gait)] where the incoming gait changes (switch while stopped)."""
        out = []
        prev = None
        for i, seg in enumerate(self.segments):
            if prev is not None and seg["gait"] != prev:
                out.append((float(self.seg_s0[i]), seg["from"], seg["gait"]))
            prev = seg["gait"]
        return out


# ---------------------------------------------------------------------------------------
# Placeholder route generation
# ---------------------------------------------------------------------------------------


def _resample(xyz: np.ndarray, step: float) -> np.ndarray:
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1))]
    n = max(2, int(math.ceil(d[-1] / step)) + 1)
    s = np.linspace(0, d[-1], n)
    return np.column_stack([np.interp(s, d, xyz[:, k]) for k in range(xyz.shape[1])]), s


def _intervals(mask: np.ndarray, s: np.ndarray, min_len: float, merge_gap: float, pad: float):
    runs, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        if (not m or i == len(mask) - 1) and start is not None:
            end = i if m else i - 1
            runs.append([float(s[start]), float(s[end])])
            start = None
    merged = []
    for r in runs:
        if merged and r[0] - merged[-1][1] <= merge_gap:
            merged[-1][1] = r[1]
        else:
            merged.append(r)
    out = [[max(0.0, a - pad), min(s[-1], b + pad)] for a, b in merged if b - a >= min_len]
    fused = []
    for r in out:
        if fused and r[0] <= fused[-1][1]:
            fused[-1][1] = max(fused[-1][1], r[1])
        else:
            fused.append(r)
    return fused


def make_placeholder(npz: Path | str, wp_spacing: float = 9.0, slope_thresh: float = 0.15,
                     step_thresh: float = 0.12) -> dict:
    from sim_full_course.terrain import Terrain

    t = np.load(npz)
    terrain = Terrain.load(npz)
    course = t["course_trajectory"]  # [t, x, y, z_base], reverse time = official order
    xyz = course[:, 1:4].copy()
    # Drop keyframe jitter < 5 cm and resample at 0.20 m (<= 0.25 m contract spacing).
    keep = np.r_[True, np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1) > 0.05]
    xyz = xyz[keep]
    line, s = _resample(xyz, 0.20)
    ground = terrain.ground_at(line[:, 0], line[:, 1])
    # Stairs/hard-terrain label: 1 m ground slope, or a local step under a 0.5 m footprint.
    k = 5
    zs = np.convolve(np.pad(ground, k, mode="edge"), np.ones(2 * k + 1) / (2 * k + 1), "valid")
    slope = np.abs(np.gradient(zs, s))
    step = np.zeros_like(s)
    h = np.arctan2(np.gradient(line[:, 1]), np.gradient(line[:, 0]))
    for off in np.linspace(-0.25, 0.25, 5):
        px = line[:, 0] - off * np.sin(h)
        py = line[:, 1] + off * np.cos(h)
        step = np.maximum(step, terrain.step_at(px, py))
    stairs_mask = (slope > slope_thresh) | (step > step_thresh)
    stairs = _intervals(stairs_mask, s, min_len=1.0, merge_gap=3.0, pad=0.6)

    # Waypoints: gait boundaries + every ~wp_spacing m inside each homogeneous run.
    cuts = sorted({0.0, float(s[-1])} | {b for iv in stairs for b in iv})
    wp_s = []
    for a, b in zip(cuts[:-1], cuts[1:], strict=True):
        n = max(1, int(math.ceil((b - a) / wp_spacing)))
        wp_s.extend(np.linspace(a, b, n + 1)[:-1].tolist())
    wp_s.append(float(s[-1]))
    wp_s = sorted(set(round(v, 4) for v in wp_s))

    def gait_of(a, b):
        mid = 0.5 * (a + b)
        return "stairs" if any(lo <= mid <= hi for lo, hi in stairs) else "flat"

    def at(sv):
        return np.array([np.interp(sv, s, line[:, 0]), np.interp(sv, s, line[:, 1]),
                         float(terrain.ground_at(np.interp(sv, s, line[:, 0]),
                                                 np.interp(sv, s, line[:, 1])))])

    wps = []
    for i, sv in enumerate(wp_s):
        p = at(sv)
        q = at(min(sv + 0.5, s[-1])) if sv < s[-1] else p
        prev = at(max(sv - 0.5, 0))
        yaw = math.atan2(q[1] - p[1], q[0] - p[0]) if sv < s[-1] else math.atan2(
            p[1] - prev[1], p[0] - prev[0])
        wps.append({
            "id": f"WP{i + 1:02d}",
            "position": [round(float(v), 4) for v in p],
            "yaw": round(yaw, 4),
            "radius_xy": 0.20,
            "tol_z": 0.20,
            "terrain": "placeholder_" + (gait_of(sv, wp_s[min(i + 1, len(wp_s) - 1)])
                                         if i + 1 < len(wp_s) else "end"),
            "confidence": "low",
            "s_m": round(float(sv), 3),
        })
    segs = []
    for i in range(len(wps) - 1):
        a, b = wp_s[i], wp_s[i + 1]
        m = (s > a) & (s < b)
        cl = np.vstack([at(a), np.column_stack([line[m, :2], ground[m]]), at(b)])
        g = gait_of(a, b)
        segs.append({
            "id": f"{wps[i]['id']}-{wps[i + 1]['id']}",
            "from": wps[i]["id"],
            "to": wps[i + 1]["id"],
            "gait": g,
            "speed_limit": GAIT_LIMITS[g],
            "allow_detour": g == "flat",
            "corridor_half_width": 0.8 if g == "flat" else 0.25,
            "centerline": [[round(float(v), 4) for v in row] for row in cl],
        })
    return {
        "schema": "s10_route_v2",
        "map_id": MAP_ID,
        "frame": "map",
        "z_reference": "ground",
        "status": "PLACEHOLDER - generated by sim_full_course/route.py from the mapping "
                  "keyframes (reverse time 1789368238..1789368895); NOT the matched route_v2; "
                  "waypoints are evenly spaced, not photo WPs; stairs labels are heuristic",
        "placeholder_meta": {
            "stairs_intervals_s": [[round(a, 2), round(b, 2)] for a, b in stairs],
            "slope_thresh": slope_thresh,
            "step_thresh": step_thresh,
            "wp_spacing_m": wp_spacing,
            "length_m": round(float(s[-1]), 2),
        },
        "waypoints": wps,
        "segments": segs,
    }


def main():
    ap = argparse.ArgumentParser(description="Generate the PLACEHOLDER route from the mapping run")
    ap.add_argument("--npz", type=Path, default=ARTIFACTS / "terrain" / "course_terrain.npz")
    ap.add_argument("--out", type=Path, default=PLACEHOLDER_PATH)
    a = ap.parse_args()
    data = make_placeholder(a.npz)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(data, indent=1))
    r = Route(data)
    print(f"{a.out}: {len(r.waypoints)} WPs, {len(r.segments)} segments, length {r.length:.1f} m")
    print("stairs intervals (s):", data["placeholder_meta"]["stairs_intervals_s"])
    for b in r.gait_boundaries():
        print("gait switch at s=%.1f %s -> %s" % b)


if __name__ == "__main__":
    main()


def subroute(route: Route, s0: float, s1: float) -> Route:
    """Clip a route to arc-length [s0, s1]: WPs strictly inside are kept (same ids, same
    segment gaits/limits); synthetic end WPs `S0`/`S1` are added at the clip points."""
    s0, s1 = float(max(0.0, s0)), float(min(route.length, s1))
    if s1 - s0 < 0.5:
        raise ValueError("subroute too short")
    by_id = {w["id"]: w for w in route.waypoints}
    ids = route.wp_order
    inner = [(sv, wid) for sv, wid in zip(route.wp_s, ids, strict=True) if s0 + 0.05 < sv < s1 - 0.05]

    def wp(wid, sv, src=None):
        p, h = route.point_at(sv)
        d = dict(src) if src else {"radius_xy": 0.20, "tol_z": 0.20, "terrain": "clip",
                                   "confidence": "low"}
        d.update({"id": wid, "position": [float(v) for v in p], "yaw": float(h)})
        d["s_m"] = float(sv - s0)
        return d

    wps = [wp("S0", s0)] + [wp(w, sv, by_id[w]) for sv, w in inner] + [wp("S1", s1)]
    cuts = [s0] + [sv for sv, _ in inner] + [s1]
    segs = []
    for i in range(len(cuts) - 1):
        a, b = cuts[i], cuts[i + 1]
        src = route.segment_at(0.5 * (a + b))
        m = (route.s > a) & (route.s < b)
        cl = np.vstack([route.point_at(a)[0], route.xyz[m], route.point_at(b)[0]])
        seg = {k: v for k, v in src.items() if k not in ("centerline", "id", "from", "to")}
        seg.update({"id": f"{wps[i]['id']}-{wps[i + 1]['id']}", "from": wps[i]["id"],
                    "to": wps[i + 1]["id"], "centerline": cl.tolist()})
        segs.append(seg)
    data = {k: v for k, v in route.data.items() if k not in ("waypoints", "segments")}
    data["status"] = f"SUBROUTE [{s0:.1f}, {s1:.1f}] of: " + str(route.data.get("status", ""))
    data["waypoints"], data["segments"] = wps, segs
    return Route(data)
