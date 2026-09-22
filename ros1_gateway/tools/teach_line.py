#!/usr/bin/env python3
"""Best teach line from several demonstrations of the same course.

The operator drives with discrete stick inputs, so a taught trail is a zig-zag. Following it makes the
robot correct all the time. This tool straightens it WITHOUT leaving ground the operator showed to be
walkable and WITHOUT cutting through anything in the map:

  1. obstacle grid (0.10 m) from the saved x_nav map: cells with map points 0.20-1.00 m above the
     local ground (x_nav's own ground cloud). Cells the demonstrations walked over are free by
     definition (stairs risers, kerbs the dog climbed).
  2. corridor = everything within `corridor` metres of ANY demonstration (flat 0.5 m, stairs 0.25 m):
     the line may move sideways only where some demonstration has been. A detour round an obstacle
     stays a detour; a wobble on open ground becomes a straight.
  3. waypoints are TOUCH DISCS, not points to stand on. The rule is "any part of the body within 0.20 m
     of the waypoint"; with a 0.9 x 0.5 m body that is "body centre within 0.45 m" whatever the heading
     (--reach 0.45). Several waypoints sit on the edge of platforms the
     robot cannot climb: the line passes close by and carries on, it never has to reach the mark.
     One reference demonstration for the whole course (the calmest one that touches the most
     waypoints), greedy line-of-sight shortcutting inside (corridor minus inflated obstacles) where
     every skipped stretch must still pass within --touch of its waypoints; corners rounded with the
     largest fillet that keeps those touches. The route's gate for each waypoint is the PASS-BY POINT on
     the line (ground the operator walked, so the planner never finds it blocked) with radius
     reach - offset, which still guarantees the body rule at the true waypoint.
  4. gait zones = where the operator really switched gait (s10_ros1_control event log), consensus of
     the chosen demonstrations; the /teach switch marks are only a fallback.

Output: a derived session directory (marks.jsonl + one path_*.trail.csv) that the unchanged
tools/teach_to_route.py turns into route_v2.json / maneuvers.json, plus teach_line.json (metrics) and
teach_line.png (before / after).

  python3 tools/teach_line.py <session_dir> --map-dir <x_nav map dir> --control-logs <dir> --out <dir>
        [--demos path_a,path_b] [--gait-demos path_b,path_c] [--corridor 0.5] [--corridor-stairs 0.25]
"""
import argparse, csv, glob, json, math, os, sys
import numpy as np
from scipy.ndimage import distance_transform_edt, binary_dilation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scipy.spatial import cKDTree as cKDTree_  # noqa: E402
from audit_teach_session import read_marks, read_trail, gait_timeline, gait_at, turn_stats  # noqa: E402

STAIRS_GAITS = (0x1003, 0x3003)


def load_pcd_xyz(path):
    b = open(path, "rb").read(); j = b.find(b"\n", b.find(b"DATA")) + 1
    H = {l.split()[0]: l.split()[1:] for l in b[:j].decode(errors="ignore").splitlines() if l and not l.startswith("#")}
    n = int(H["POINTS"][0]); step = sum(int(s) * int(c) for s, c in zip(H["SIZE"], H["COUNT"]))
    return np.frombuffer(b[j:j + n * step], dtype=np.uint8).reshape(n, step)[:, :12].copy().view(np.float32).reshape(n, 3).astype(float)


class Grid:
    def __init__(self, xy_all, res=0.10, margin=4.0):
        self.res = res
        # origin snapped to the cell size: teach_line and verify_route_clearance bin the map identically whatever
        # recordings each of them loads (2026-09-22: a 2-point obstacle was a cell in one grid and not in the other)
        self.x0, self.y0 = math.floor((xy_all[:, 0].min() - margin) / res) * res, math.floor((xy_all[:, 1].min() - margin) / res) * res
        self.nx = int(math.ceil((xy_all[:, 0].max() + margin - self.x0) / res)) + 1
        self.ny = int(math.ceil((xy_all[:, 1].max() + margin - self.y0) / res)) + 1

    def idx(self, xy):
        i = np.floor((np.asarray(xy)[..., 0] - self.x0) / self.res).astype(int)
        j = np.floor((np.asarray(xy)[..., 1] - self.y0) / self.res).astype(int)
        return np.clip(i, 0, self.nx - 1), np.clip(j, 0, self.ny - 1)

    def inside(self, pts):
        return (pts[:, 0] >= self.x0) & (pts[:, 0] < self.x0 + self.nx * self.res) & (pts[:, 1] >= self.y0) & (pts[:, 1] < self.y0 + self.ny * self.res)

    def mask_near(self, xy, radius):
        m = np.ones((self.nx, self.ny), bool)
        i, j = self.idx(xy); m[i, j] = False
        return distance_transform_edt(m) * self.res <= radius


def obstacle_mask(grid, ground, allpts, band=(0.20, 1.00), min_pts=2):
    g = ground[grid.inside(ground)]; a = allpts[grid.inside(allpts)]
    gi, gj = grid.idx(g); flat = gi * grid.ny + gj
    cnt = np.bincount(flat, minlength=grid.nx * grid.ny); zs = np.bincount(flat, weights=g[:, 2], minlength=grid.nx * grid.ny)
    have = cnt > 0
    gz = np.where(have, zs / np.maximum(cnt, 1), np.nan).reshape(grid.nx, grid.ny)
    known = ~np.isnan(gz)
    _, (ii, jj) = distance_transform_edt(~known, return_indices=True)           # ground height: nearest known cell
    gz = gz[ii, jj]
    ai, aj = grid.idx(a); h = a[:, 2] - gz[ai, aj]
    sel = (h >= band[0]) & (h <= band[1])
    occ = np.bincount(ai[sel] * grid.ny + aj[sel], minlength=grid.nx * grid.ny).reshape(grid.nx, grid.ny) >= min_pts
    return occ, gz


HL, HW = 0.45, 0.25          # robot body half length / half width (m): the team planner's 0.9 x 0.5 m footprint


def body_cover(grid, xy, yaw, hl=HL, hw=HW):
    """Cells covered by an (2 hl) x (2 hw) rectangle placed at every (xy, yaw)."""
    m = np.zeros((grid.nx, grid.ny), bool)
    lx = np.arange(-hl, hl + 1e-9, grid.res / 2); ly = np.arange(-hw, hw + 1e-9, grid.res / 2)
    gx, gy = np.meshgrid(lx, ly, indexing="ij"); loc = np.c_[gx.ravel(), gy.ravel()]
    xy, yaw = np.asarray(xy, float), np.asarray(yaw, float)
    for k in range(0, len(xy), 400):
        c, s_ = np.cos(yaw[k:k + 400]), np.sin(yaw[k:k + 400])
        wx = xy[k:k + 400, 0, None] + loc[None, :, 0] * c[:, None] - loc[None, :, 1] * s_[:, None]
        wy = xy[k:k + 400, 1, None] + loc[None, :, 0] * s_[:, None] + loc[None, :, 1] * c[:, None]
        i, j = grid.idx(np.stack([wx.ravel(), wy.ravel()], axis=-1)); m[i, j] = True
    return m


def rect_dist(centre, yaw, w, hl=HL, hw=HW):
    """Distance from point w to the body rectangle at (centre, yaw); 0 when w is under the body."""
    d = np.asarray(w, float) - np.asarray(centre, float); c, s_ = math.cos(yaw), math.sin(yaw)
    lx, ly = d[0] * c + d[1] * s_, -d[0] * s_ + d[1] * c
    return math.hypot(max(abs(lx) - hl, 0.0), max(abs(ly) - hw, 0.0))


def body_touch(line, w, window=None):
    """(smallest body-to-point distance along the line, index): heading = direction of travel."""
    tang = np.arctan2(np.gradient(line[:, 1]), np.gradient(line[:, 0]))
    ks = range(len(line)) if window is None else range(max(0, window[0]), min(len(line), window[1]))
    best = min(((rect_dist(line[k], tang[k], w), k) for k in ks), default=(float("inf"), 0))
    return best


def sweep_samples(line):
    """5 cm interpolation of a polyline with three headings per sample (smoothed tangent, chord out, chord in) and
    the index of the polyline vertex each sample belongs to. ONE definition of "the body swept along a line",
    used for the operator's tracks (what is exempt) and for new lines (what is tested), so both are judged alike."""
    line = np.asarray(line, float)
    keep = np.r_[True, np.linalg.norm(np.diff(line, axis=0), axis=1) > 1e-6]
    idx0 = np.where(keep)[0]; pts = line[keep]
    if len(pts) < 2:
        return pts, np.zeros((len(pts), 3)), idx0
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    s5 = np.r_[np.arange(0, d[-1], 0.05), d[-1]]
    p5 = np.c_[np.interp(s5, d, pts[:, 0]), np.interp(s5, d, pts[:, 1])]
    owner = idx0[np.clip(np.searchsorted(d, s5, side="right") - 1, 0, len(pts) - 1)]
    g = np.arctan2(np.gradient(p5[:, 1]), np.gradient(p5[:, 0]))
    ch = np.arctan2(np.diff(p5[:, 1], append=p5[-1, 1]), np.diff(p5[:, 0], append=p5[-1, 0])); ch[-1] = ch[-2]
    return p5, np.c_[g, ch, np.r_[ch[0], ch[:-1]]], owner


def sweep_mask(grid, line, margin):
    p5, heads, _ = sweep_samples(line)
    m = np.zeros((grid.nx, grid.ny), bool)
    for c in range(heads.shape[1] if len(p5) else 0):
        m |= body_cover(grid, p5, heads[:, c], HL + margin, HW + margin)
    return m


def _rect_dist_many(c, yaw, pts):
    d = pts - c; co, si = math.cos(yaw), math.sin(yaw)
    lx, ly = d[:, 0] * co + d[:, 1] * si, -d[:, 0] * si + d[:, 1] * co
    return np.hypot(np.maximum(np.abs(lx) - HL, 0.0), np.maximum(np.abs(ly) - HW, 0.0))


def free_zones(grid, occ, spec, wp_xy, strict=True):
    """Operator statement "there is nothing at this waypoint": removes the map's obstacle cells within a radius of the
    named marks (ghosts of people standing at the mark while it was made, etc.). spec = "WP06:1.5,WP11" (default
    radius 1.5 m). Returns (occ, [[id, radius, cells removed], ...])."""
    occ = occ.copy(); done = []
    for item in [v for v in (spec or "").split(",") if v]:
        wid, _, r = item.partition(":"); r = float(r) if r else 1.5
        if wid not in wp_xy:
            if strict: sys.exit("--free-at: unknown waypoint " + wid)
            continue                                              # a shortened route without that waypoint
        m = grid.mask_near(np.array([wp_xy[wid]], float), r) & occ
        done.append([wid, r, int(m.sum())]); occ &= ~m
    return occ, done


class Clearance:
    """The safety criterion, used by the generator and by tools/verify_route_clearance.py alike. Continuous
    geometry (exact distance from each obstacle point to the 0.9 x 0.5 m body rectangle), so it does not depend on
    how a line happens to be sampled or on grid rounding.

        obstacle point = centre of a 0.1 m map cell with points 0.20-1.00 m above the ground
        d_op(o)        = how close the operator's body came to o in any demonstration (0 = the body was over it)
        d_new(o)       = how close the body comes to o along the new line
        violation      = d_new < margin  and  d_new < d_op - slack

    i.e. the body keeps `margin` from everything, except that it may pass something as closely as the operator's
    body did (3 cm slack). What the operator's body went over (stair risers, kerbs, grass, ghosts) has d_op = 0 and
    can never be a violation; an obstacle the operator never touched is a violation as soon as the body is on it."""

    def __init__(self, grid, occ, tracks, margin=0.05, slack=0.05):
        from scipy.spatial import cKDTree
        self.grid, self.margin, self.slack = grid, margin, slack
        ii, jj = np.where(occ)
        self.P = np.c_[grid.x0 + (ii + 0.5) * grid.res, grid.y0 + (jj + 0.5) * grid.res]
        self.ij = (ii, jj)
        self.tree = cKDTree(self.P)
        self.R = math.hypot(HL, HW) + margin + 0.35
        self.d_op = np.full(len(self.P), np.inf)
        for t in tracks:                                          # t: N x 5 (time, x, y, z, yaw)
            xy = t[:, 1:3]
            self._accumulate(self.d_op, xy, t[:, 4][:, None])     # heading as recorded
            mv = xy[np.r_[True, np.linalg.norm(np.diff(xy, axis=0), axis=1) > 0.02]]
            if len(mv) > 2:
                p5, heads, _ = sweep_samples(mv)
                self._accumulate(self.d_op, p5, heads)            # heading along the direction of travel
        self.passable = np.zeros_like(occ); self.obst = np.zeros_like(occ)
        under = self.d_op <= 0.0
        self.passable[ii[under], jj[under]] = True
        self.obst[ii[~under], jj[~under]] = True

    def _accumulate(self, dmin, pts, heads):
        for k, nb in enumerate(self.tree.query_ball_point(pts, self.R)):
            if not nb: continue
            nb = np.asarray(nb); q = self.P[nb]
            d = np.min([_rect_dist_many(pts[k], th, q) for th in heads[k]], axis=0)
            dmin[nb] = np.minimum(dmin[nb], d)

    def check(self, line):
        """(indices of violating obstacle points, their d_new, of which the body is ON the obstacle)"""
        p5, heads, _ = sweep_samples(line)
        d_new = np.full(len(self.P), np.inf)
        self._accumulate(d_new, p5, heads)
        v = (d_new < self.margin) & (d_new < self.d_op - self.slack)
        return np.where(v)[0], d_new, v & (d_new <= 0.0)

    def bad_vertices(self, line, reach=0.75):
        from scipy.spatial import cKDTree
        vi, _, _ = self.check(line)
        bad = np.zeros(len(line), bool)
        if len(vi):
            for hits in cKDTree(line).query_ball_point(self.P[vi], reach):
                bad[hits] = True
        return bad


def strip_backward(t, v_min=-0.10, n_min=4):
    """Cut out what was driven BACKWARDS (forward speed along the recorded yaw < v_min for n_min samples) together with
    the ground covered again afterwards, so the trail only ever advances. Returns (trail, metres removed)."""
    if len(t) < 10: return t, 0.0
    dt = np.maximum(np.diff(t[:, 0]), 1e-3); v = np.diff(t[:, 1:3], axis=0) / dt[:, None]
    vf = np.convolve(v[:, 0] * np.cos(t[:-1, 4]) + v[:, 1] * np.sin(t[:-1, 4]), np.ones(5) / 5, "same")
    keep, k, removed = np.ones(len(t), bool), 0, 0.0
    while k < len(vf):
        if vf[k] >= v_min: k += 1; continue
        k1 = k
        while k1 + 1 < len(vf) and vf[k1 + 1] < v_min: k1 += 1
        if k1 - k + 1 < n_min: k = k1 + 1; continue
        h = np.array([math.cos(t[k, 4]), math.sin(t[k, 4])]); j = k1 + 1
        while j < len(t) - 1 and (t[j, 1:3] - t[k, 1:3]) @ h < 0.0: j += 1
        keep[k:j] = False; removed += float(np.linalg.norm(t[k1, 1:3] - t[k, 1:3])); k = j
    return t[keep], removed


def cut_leg(trail, a_xy, b_xy, cursor):
    d = np.linalg.norm(trail[cursor:, 1:3] - a_xy, axis=1); i0 = cursor + int(np.argmin(d))
    d = np.linalg.norm(trail[i0:, 1:3] - b_xy, axis=1)
    # first local approach to b after i0 (not a later pass of a loop)
    near = np.where(d < max(0.8, d.min() + 0.3))[0]
    i1 = i0 + int(near[np.argmin(d[near[near <= near[0] + 400]])]) if len(near) else i0 + int(np.argmin(d))
    return i0, max(i1, i0 + 1)


def resample(xy, step):
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    if d[-1] < 1e-6: return xy[:1].copy()
    s = np.r_[np.arange(0, d[-1], step), d[-1]]
    return np.c_[np.interp(s, d, xy[:, 0]), np.interp(s, d, xy[:, 1])]


def seg_free(grid, free, p, q):
    n = max(2, int(np.linalg.norm(q - p) / (grid.res * 0.5)) + 1)
    pts = p + (q - p) * np.linspace(0, 1, n)[:, None]
    i, j = grid.idx(pts)
    return bool(free[i, j].all())


def seg_dist(p, q, c):
    d = q - p; L2 = float(d @ d)
    t = 0.0 if L2 < 1e-12 else float(np.clip((c - p) @ d / L2, 0, 1))
    return float(np.linalg.norm(p + t * d - c))


def poly_dist(poly, c):
    return min(seg_dist(poly[k], poly[k + 1], c) for k in range(len(poly) - 1)) if len(poly) > 1 else float(np.linalg.norm(poly[0] - c))


def shortcut(grid, free, pts, discs=(), max_span=200):
    """Greedy farthest-visible vertex. discs = [(anchor index on pts, centre xy, radius)]: a chord that skips
    an anchor must still pass within the radius of its centre."""
    out, idx, i, n = [pts[0]], [0], 0, len(pts)
    while i < n - 1:
        j = min(n - 1, i + max_span)
        while j > i + 1:
            if seg_free(grid, free, pts[i], pts[j]) and all(seg_dist(pts[i], pts[j], c) <= r for a, c, r in discs if i < a < j):
                break
            j -= 1
        out.append(pts[j]); idx.append(j); i = j
    return np.array(out), idx


def fillet(grid, free, poly, r_max, keep_within=None, accept=None):
    """Round interior corners. keep_within: {vertex index: max distance the rounded line may pass from it}.
    accept(k, arc) -> bool: extra test for the arc that replaces vertex k (e.g. waypoint touches kept)."""
    out = [poly[0]]
    for k in range(1, len(poly) - 1):
        P, V, N = np.array(out[-1]), poly[k], poly[k + 1]
        u, w = P - V, N - V; lu, lw = np.linalg.norm(u), np.linalg.norm(w)
        if lu < 1e-6 or lw < 1e-6: continue
        u, w = u / lu, w / lw
        phi = math.acos(float(np.clip(u @ w, -1, 1))); theta = math.pi - phi
        if theta < math.radians(8) or phi < math.radians(20):
            out.append(V); continue
        r = r_max
        if keep_within and k in keep_within:
            r = min(r, keep_within[k] / max(1.0 / math.sin(phi / 2) - 1.0, 1e-6))
        r = min(r, 0.45 * min(lu, lw) * math.tan(phi / 2))
        done = False
        for _ in range(4):
            if r < 0.15: break
            t = r / math.tan(phi / 2); bis = (u + w) / np.linalg.norm(u + w); C = V + bis * (r / math.sin(phi / 2))
            a0 = math.atan2(*(V + u * t - C)[::-1]); a1 = math.atan2(*(V + w * t - C)[::-1])
            da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
            arc = np.array([C + r * np.array([math.cos(a0 + da * s), math.sin(a0 + da * s)]) for s in np.linspace(0, 1, max(4, int(abs(da) * r / 0.1) + 2))])
            i, j = grid.idx(arc)
            if free[i, j].all() and seg_free(grid, free, np.array(out[-1]), arc[0]) and (accept is None or accept(k, arc)):
                out.extend(list(arc)); done = True; break
            r *= 0.6
        if not done: out.append(V)
    out.append(poly[-1])
    return np.array(out)


def hairpins(line, grid, free, e0=0.20, taper=2.5, step=0.10):
    """Dead-end waypoints: the robot goes in and comes back the same way. Two coincident lines make pure
    pursuit aim at the robot's own position at the tip (it spins both ways) and make the projection ambiguous.
    Keep right in both directions (offset tapering from 0 to e over `taper` m) and join the two sides with a
    half circle through the tip: the way in and the way out are e*2 apart and the turn has one direction."""
    out, notes, i, n, k3 = line.copy(), [], 3, len(line), 3
    tips = []
    while i < n - k3:
        d1, d2 = line[i] - line[i - k3], line[i + k3] - line[i]
        l1, l2 = np.linalg.norm(d1), np.linalg.norm(d2)
        if l1 > 1e-6 and l2 > 1e-6 and (d1 @ d2) / (l1 * l2) < math.cos(math.radians(120)):
            j = i
            while j + 1 < n - k3:                                   # sharpest point of this reversal
                a1, a2 = line[j + 1] - line[j + 1 - k3], line[j + 1 + k3] - line[j + 1]
                if (a1 @ a2) / (np.linalg.norm(a1) * np.linalg.norm(a2) + 1e-9) < (d1 @ d2) / (l1 * l2):
                    j += 1; d1, d2, l1, l2 = a1, a2, np.linalg.norm(a1), np.linalg.norm(a2)
                else:
                    break
            tips.append(j); i = j + int(1.0 / step)
        else:
            i += 1
    pieces, last = [], 0
    for t in tips:
        T = line[t]; m = int(taper / step)
        lo, hi = max(last, t - m), min(n - 1, t + m)
        u = T - line[max(lo, t - 5)]; u = u / (np.linalg.norm(u) + 1e-9); nl = np.array([-u[1], u[0]])
        done = False
        for e in (e0, 0.15, 0.10):
            cut = max(1, int(round(e / step)))
            def side(idx_from, idx_to):
                pts = []
                for j in range(idx_from, idx_to + 1):
                    a = line[min(n - 1, j + 1)] - line[max(0, j - 1)]; a = a / (np.linalg.norm(a) + 1e-9)
                    w = 0.5 * (1 - math.cos(math.pi * min(1.0, 1.0 - abs(j - t) / max(m, 1))))   # 0 far away .. 1 at the tip
                    pts.append(line[j] + e * w * np.array([a[1], -a[0]]))                        # to the RIGHT of travel
                return pts
            way_in, way_out = side(lo, t - cut), side(t + cut, hi)
            C = T - e * u
            arc = [C + e * (-nl * math.cos(th) + u * math.sin(th)) for th in np.linspace(0, math.pi, 9)]
            cand = np.array(way_in + arc + way_out)
            ii, jj = grid.idx(cand)
            if free[ii, jj].all():
                pieces.append((lo, hi, cand)); notes.append(dict(at=[round(float(v), 2) for v in T], separation=round(2 * e, 2))); done = True; break
        if not done:
            notes.append(dict(at=[round(float(v), 2) for v in T], separation=0.0, note="no room for a hairpin: left as a reversal"))
        last = hi
    if not pieces:
        return line, notes
    res, cur = [], 0
    for lo, hi, cand in pieces:
        res.extend(list(line[cur:lo])); res.extend(list(cand)); cur = hi + 1
    res.extend(list(line[cur:]))
    return resample(np.array(res), step), notes


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session"); ap.add_argument("--map-dir", required=True); ap.add_argument("--control-logs", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--demos", default="", help="path recordings to use for the line (default: every recording that passes >= 90 %% of the WPs)")
    ap.add_argument("--gait-demos", default="", help="recordings whose operator gait defines the zones (default: the demos with >= 2 gait changes)")
    ap.add_argument("--corridor", type=float, default=0.5); ap.add_argument("--corridor-stairs", type=float, default=0.25)
    ap.add_argument("--margin", type=float, default=0.05, help="margin added round the body in the sweep test, m")
    ap.add_argument("--clearance", type=float, default=0.10, help="free space kept between the body side and any real obstacle, m (centre keep-out = 0.25 + this)")
    ap.add_argument("--fillet", type=float, default=1.2)
    ap.add_argument("--reach", type=float, default=0.45, help="body centre this close to a waypoint = some part of the body within 0.20 m, whatever the heading (0.9 x 0.5 m body: 0.20 + half width 0.25)")
    ap.add_argument("--touch", type=float, default=0.20, help="the line passes every waypoint within this distance")
    ap.add_argument("--body-touch", type=float, default=0.05, help="a waypoint counts as touched when the body sweep comes this close to its mark (rule: 0.20; the rest is margin)")
    ap.add_argument("--body-z-offset", type=float, default=0.41)
    ap.add_argument("--splice", default="", help="partial recordings (a few waypoints each) that REPLACE the reference where they run, newest way of driving that stretch; two over the same stretch: the calmer one is the reference, both widen the corridor")
    ap.add_argument("--drop", default="", help="waypoints that are no longer part of the course: WP25,WP31")
    ap.add_argument("--contact", default="", help="waypoints on a wall / platform edge the dog must drive INTO to touch: the line goes to the mark itself, the wall round it is not an obstacle (WP17,WP24)")
    ap.add_argument("--contact-radius", type=float, default=1.5)
    ap.add_argument("--terrain-zones", type=int, default=1, help="1 = stairs gait only where the operator used it AND the ground map shows steps/slope (pad 1.5 m, gaps < 6 m merged)")
    ap.add_argument("--fast-gait", type=int, default=0, help="1 = also plan 'fast' (0xF002) stretches. OFF: on the robot (2026-09-21) that gait ran ~2.7 m/s on a 0.9 m/s command and left the line")
    ap.add_argument("--gait-plan", type=int, default=1, help="1 = write the four-gait plan (walk / fast / stairs / platform) into terrain.json")
    ap.add_argument("--fast-turn-deg", type=float, default=25.0, help="fast gait only where the line turns less than this over 2 m")
    ap.add_argument("--fast-clearance", type=float, default=0.0, help="... and the body side keeps this distance from real obstacles, m")
    ap.add_argument("--fast-min-run", type=float, default=8.0, help="... for at least this many metres")
    ap.add_argument("--strip-backward", default="", help="recordings in which stretches driven BACKWARDS (a slip after a jump, an operator correction) are cut out before use: 193500,194417")
    ap.add_argument("--jump-lane", type=int, default=1, help="1 = where a --splice recording climbs a ledge in the platform gait (0x1002), the line is a STRAIGHT lane through the operators' crossing point, square to it (narrow gaps between objects on the edge)")
    ap.add_argument("--jump-before", type=float, default=2.0, help="straight lane this far before the ledge, m (shortened automatically where a waypoint or an obstacle needs it)")
    ap.add_argument("--jump-after", type=float, default=1.0, help="... and this far after it, m")
    ap.add_argument("--jump-max-shift", type=float, default=1.5, help="a lane may move the line sideways by at most this, m; every moved point must also lie inside the corridor of the recordings (one of them drove there) and pass the body sweep")
    ap.add_argument("--platform-before", type=float, default=0.8, help="platform gait from this far before a ledge (body centre; the switch is made standing) ...")
    ap.add_argument("--platform-before-at", default="", help='other value at single ledges, by the nearest waypoint: "WP24:0.5,WP26:0.5" (0.5 = nose against the ledge: no run-up needed there, operator 2026-09-22)')
    ap.add_argument("--platform-after", type=float, default=1.2, help="... to this far after it (hind legs up), then straight back to the operator's gait. Operator 2026-09-21: platform gait ONLY for the jump itself")
    ap.add_argument("--platform-absorb", type=float, default=4.0, help="a stairs-gait stretch shorter than this that touches a platform zone is walked in the platform gait (saves two gait switches, ~2 s standing each); 0 = off")
    ap.add_argument("--free-at", default="", help='the operator says there is NO obstacle at these waypoints: "WP06:1.5,WP11" (radius m, default 1.5); map obstacle cells there are ignored')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    rows, live = read_marks(os.path.join(a.session, "marks.jsonl"))
    wps = {}
    for r in live:
        if r["kind"] == "WP" and (r.get("result") or {}).get("passed"): wps[r["wp_id"]] = r
    for w in [v.upper() for v in a.drop.split(",") if v]:
        if wps.pop(w, None) is not None: print("dropped from the course:", w)
    ids = sorted(wps); W = np.array([wps[w]["pose"][:2] for w in ids])
    contact = [v.upper() for v in a.contact.split(",") if v]
    tl = gait_timeline(a.control_logs) if a.control_logs else []

    trails = {os.path.basename(f)[:-10]: read_trail(f) for f in sorted(glob.glob(os.path.join(a.session, "path_*.trail.csv")))}
    for d in [v for v in a.strip_backward.split(",") if v]:
        if d not in trails: sys.exit("--strip-backward: no recording " + d)
        trails[d], cut_m = strip_backward(trails[d])
        print(f"   {d}: {cut_m:.2f} m driven backwards cut out")
    def coverage(t): return np.mean([np.min(np.linalg.norm(t[:, 1:3] - w, axis=1)) < 0.8 for w in W]) if len(t) > 1 else 0.0
    patches = [d for d in a.splice.split(",") if d]
    for d in patches:
        if d not in trails: sys.exit("--splice: no recording " + d)
    demos = [d for d in a.demos.split(",") if d] or [n for n, t in trails.items() if coverage(t) >= 0.85 and n not in patches]
    if not demos: sys.exit("no demonstration covers the waypoints")
    def n_gait_changes(t):
        g = [gait_at(tl, tt) for tt in t[::20, 0]]; return sum(1 for x, y in zip(g, g[1:]) if x != y)
    gait_demos = [d for d in a.gait_demos.split(",") if d] or [d for d in demos if tl and n_gait_changes(trails[d]) >= 2]
    print("demonstrations:", demos, "| gait from:", gait_demos or "switch marks")

    allxy = np.vstack([trails[d][:, 1:3] for d in demos + patches])
    grid = Grid(allxy)
    ground_pts = load_pcd_xyz(os.path.join(a.map_dir, "global_ground_map.pcd"))
    occ, gz = obstacle_mask(grid, ground_pts, load_pcd_xyz(os.path.join(a.map_dir, "global_map.pcd")))
    spec = ",".join([v for v in [a.free_at] + [f"{w}:{a.contact_radius}" for w in contact if w in wps] if v])
    occ, freed = free_zones(grid, occ, spec, {w: wps[w]["pose"][:2] for w in ids})
    for wid, r, n in freed: print(f"--free-at {wid}: {n} obstacle cells within {r} m ignored on the operator's word")
    # The survey run (the drive on which the waypoints were MARKED) passes through every mark: it shows how
    # the dog got to each mark. Only its stretches within 2.5 m of a mark are used.
    survey = [read_trail(f) for f in sorted(glob.glob(os.path.join(a.session, "survey_*.trail.csv")))]
    survey = [t[np.min(np.linalg.norm(t[:, None, 1:3] - W[None, :, :], axis=2), axis=1) < 2.5] for t in survey if len(t) > 10]
    # the trimmed survey is PIECES round the marks: keep them separate tracks. As one track, the body sweep along the
    # direction of travel bridged the gaps with straight chords and exempted whatever stood on them (2026-09-22: a thin
    # object between WP21 and WP22 ended up under the line; verify_route_clearance caught it).
    survey = [p_ for t in survey for p_ in np.split(t, np.where(np.linalg.norm(np.diff(t[:, 1:3], axis=0), axis=1) > 0.5)[0] + 1)]
    survey = [t for t in survey if len(t) > 10]
    # An obstacle is a map cell the robot BODY never overlapped in any demonstration (recorded yaw). What the body
    # did overlap (stair risers, kerbs, grass, the operator's ghost in the map) is not an obstacle for this robot.
    # The exemption uses the same body + margin as the final sweep, with the recorded heading AND the direction of
    # travel, so the operator's own line passes its own test: the new line may come as close to something as the
    # operator's body did, never closer.
    clr = Clearance(grid, occ, [trails[d] for d in demos + patches] + survey, a.margin)
    real, demo_body = clr.obst, clr.passable
    # The body CENTRE keeps half a body width + clearance from every real obstacle, except exactly on a demonstrated
    # track (narrow places: the operator's own line is the only one allowed there).
    blocked = binary_dilation(real, iterations=int(math.ceil((HW + a.clearance) / grid.res)))
    demo_tracks = np.vstack([trails[d][:, 1:3] for d in demos + patches] + [t[:, 1:3] for t in survey])
    centre_ok = ~blocked | grid.mask_near(demo_tracks, 0.06)
    print(f"grid {grid.nx}x{grid.ny} @ {grid.res} m, obstacle cells {int(occ.sum())}, real (body never touched) {int(real.sum())}")

    # operator gait along each gait demo -> stairs flag per point
    def stairs_flags(t): return np.array([gait_at(tl, tt) in STAIRS_GAITS for tt in t[:, 0]]) if tl else np.zeros(len(t), bool)
    flags = {d: stairs_flags(trails[d]) for d in demos + patches}
    PLATFORM_GAITS = (0x1002,)
    pflags = {d: (np.array([gait_at(tl, tt) in PLATFORM_GAITS for tt in trails[d][:, 0]]) if tl else np.zeros(len(trails[d]), bool)) for d in demos + patches}

    # ---- ledges climbed in the platform gait: crossing point and direction, agreed between the recordings
    jumps = []
    if a.jump_lane and tl:
        cross = []
        for d in patches:
            t, pf = trails[d], pflags[d]
            dcum = np.r_[0, np.cumsum(np.linalg.norm(np.diff(t[:, 1:3], axis=0), axis=1))]
            idx = np.where(pf)[0]
            for run in (np.split(idx, np.where(np.diff(idx) > 1)[0] + 1) if len(idx) else []):
                if len(run) < 8: continue
                zr = np.convolve(np.pad(t[run, 3], 2, mode="edge"), np.ones(5) / 5, "valid"); z0, z1 = float(np.median(zr[:5])), float(np.median(zr[-5:]))
                if z1 - z0 < 0.12: continue                       # platform gait, but no ledge climbed in this run
                kc = int(run[int(np.argmax(zr > 0.5 * (z0 + z1)))])               # body centre half way up
                ka, kb = int(np.searchsorted(dcum, dcum[kc] - 1.0)), min(len(t) - 1, int(np.searchsorted(dcum, dcum[kc] + 1.0)))
                u_ = t[kb, 1:3] - t[ka, 1:3]
                cross.append(dict(rec=d, xy=t[kc, 1:3].copy(), u=u_ / max(np.linalg.norm(u_), 1e-6), rise=z1 - z0))
        while cross:
            grp = [c_ for c_ in cross if np.linalg.norm(c_["xy"] - cross[0]["xy"]) < 1.5]; cross = [c_ for c_ in cross if not any(c_ is g_ for g_ in grp)]
            u_ = np.mean([g_["u"] for g_ in grp], axis=0); u_ /= np.linalg.norm(u_); E_ = np.mean([g_["xy"] for g_ in grp], axis=0)
            # the ledge itself: where x_nav's ground cloud steps up along the lane (the body-height estimate is +-0.2 m)
            us_ = np.arange(-1.0, 1.01, 0.05); prof = np.full(len(us_), np.nan); gt_ = cKDTree_(ground_pts[:, :2])
            for q_, uu in enumerate(us_):
                nb = gt_.query_ball_point(E_ + uu * u_, 0.15)
                if len(nb) >= 3: prof[q_] = np.percentile(ground_pts[nb, 2], 50)
            lo_z, hi_z = np.nanmedian(prof[us_ < -0.5]) if np.isfinite(prof[us_ < -0.5]).any() else np.nan, np.nanmedian(prof[us_ > 0.5]) if np.isfinite(prof[us_ > 0.5]).any() else np.nan
            shift_ = 0.0
            if np.isfinite(lo_z) and np.isfinite(hi_z) and hi_z - lo_z > 0.10:
                up_ = np.where(np.isfinite(prof) & (prof > 0.5 * (lo_z + hi_z)))[0]
                if len(up_): shift_ = float(us_[up_[0]])
            nrm_ = np.array([-u_[1], u_[0]]); lat = [float((g_["xy"] - E_) @ nrm_) for g_ in grp]
            jumps.append(dict(xy=E_ + shift_ * u_, u=u_, yaw=float(math.atan2(u_[1], u_[0])), rise=float(np.mean([g_["rise"] for g_ in grp])),
                              recordings=[g_["rec"] for g_ in grp], spread=float(max(lat) - min(lat)), edge_from_map=bool(shift_ != 0.0)))
        for J in jumps:
            kw = int(np.argmin(np.linalg.norm(W - J["xy"], axis=1)))
            print("   ledge %.2f m near %s (%.2f m away): %d recordings cross within %.2f m of each other, direction %.0f deg, ledge position %s" % (
                J["rise"], ids[kw], np.linalg.norm(W[kw] - J["xy"]), len(J["recordings"]), J["spread"], math.degrees(J["yaw"]), "from the ground cloud" if J["edge_from_map"] else "from the body height"))

    # ---- one reference for the whole course: the calmest demonstration among those touching the most waypoints
    def touched(t): return [float(np.min(np.linalg.norm(t[:, 1:3] - w, axis=1))) for w in W]
    score = {d: (sum(v <= a.touch for v in touched(trails[d])), -turn_stats(trails[d][:, 1:3]).get("total_turning_deg", 0)) for d in demos}
    ref_name = max(score, key=lambda d: score[d]); ref = trails[ref_name]
    def anchors_on(t):
        """index on trail t of its pass at every waypoint, walked in order; -1 = t does not go there"""
        cursor, out = 0, []
        for w in W:
            d = np.linalg.norm(t[cursor:, 1:3] - w, axis=1)
            if d.min() > 1.2: out.append(-1); continue
            near = np.where(d < max(1.0, d.min() + 0.3))[0]; near = near[near <= near[0] + 600]
            i = cursor + int(near[np.argmin(d[near])]); out.append(i); cursor = i
        return out
    # partial recordings replace the reference where they run (grouped by the waypoints they join; calmest of a group)
    groups, spliced = {}, []
    for d in patches:
        an = anchors_on(trails[d]); ks = [k for k, i in enumerate(an) if i >= 0]
        if len(ks) < 2: print("   splice", d, "touches fewer than 2 waypoints: corridor only"); continue
        groups.setdefault((ks[0], ks[-1]), []).append((turn_stats(trails[d][an[ks[0]]:an[ks[-1]] + 1, 1:3]).get("total_turning_deg", 0), d, an))
    for (ka, kb), cands in sorted(groups.items()):
        _, d, an = min(cands); main = anchors_on(ref)
        if main[ka] < 0: sys.exit(f"--splice {d}: the reference does not reach {ids[ka]}")
        piece = trails[d][an[ka]:an[kb] + 1]
        ref = np.vstack([ref[:main[ka]], piece] + ([ref[main[kb] + 1:]] if main[kb] >= 0 else []))
        spliced.append(dict(recording=d, first=ids[ka], last=ids[kb], k=(ka, kb)))
        print(f"   reference {ids[ka]}..{ids[kb]} now from {d}" + ("" if main[kb] >= 0 else " (extends the course)"))
    anchors_raw = anchors_on(ref)
    if min(anchors_raw) < 0: sys.exit("no recording reaches " + ", ".join(ids[k] for k, i in enumerate(anchors_raw) if i < 0))
    ref = ref[anchors_raw[0]:anchors_raw[-1] + 1]; anchors_raw = [i - anchors_raw[0] for i in anchors_raw]
    base, b_anchor = [], []
    seg_pts = ref[:, 1:3]
    d_ref = np.r_[0, np.cumsum(np.linalg.norm(np.diff(seg_pts, axis=0), axis=1))]
    s_new = np.r_[np.arange(0, d_ref[-1], 0.25), d_ref[-1]]
    base = np.c_[np.interp(s_new, d_ref, seg_pts[:, 0]), np.interp(s_new, d_ref, seg_pts[:, 1])]
    b_anchor = [int(np.argmin(np.abs(s_new - d_ref[i]))) for i in anchors_raw]
    # a waypoint the reference does not touch: bend the reference towards the mark (the dog stood there when it
    # was marked) with a smooth 2 m bulge. Inserting the mark as a vertex would make a spike the follower stops at.
    inserted = []
    s_base = np.r_[0, np.cumsum(np.linalg.norm(np.diff(base, axis=0), axis=1))]
    for k in range(len(W)):
        i = b_anchor[k]; off = W[k] - base[i]; dist = float(np.linalg.norm(off))
        reach_now, _ = body_touch(base, W[k], (i - 12, i + 13))
        is_contact = ids[k] in contact
        if (reach_now > a.body_touch and not is_contact) or (is_contact and dist > 0.10):   # the body on the reference does not reach this mark
            ds = np.abs(s_base - s_base[i]); done = False
            for frac in (1.0, 0.8, 0.6, 0.4, 0.2):                # as far towards the mark as stays clear of obstacles
                for width in (2.0, 1.2, 0.8):                     # a narrow bulge where a wide one would brush something further along
                    wgt = np.where(ds < width, 0.5 * (1 + np.cos(np.pi * ds / width)), 0.0)
                    shift = off * (1.0 - 0.06 / dist) * frac
                    cand = base + wgt[:, None] * shift[None, :]
                    moved = np.where(wgt > 0)[0]; lo_, hi_ = max(0, moved[0] - 4), min(len(cand), moved[-1] + 5)
                    ci, cj = grid.idx(cand[moved])
                    if centre_ok[ci, cj].all() and not clr.bad_vertices(resample(cand[lo_:hi_], 0.10)).any():
                        base = cand; inserted.append(f"{ids[k]}:{dist:.2f}->{dist * (1 - frac) + 0.06 * frac:.2f}"); done = True; break
                if done: break
            else:
                inserted.append(f"{ids[k]}:{dist:.2f} (no safe way closer)")
    discs = [(b_anchor[k], W[k], 0.08 if ids[k] in contact else a.touch) for k in range(len(W))]

    # per-point stairs flag (corridor width) and the free mask
    stairs_base = np.zeros(len(base), bool)
    if gait_demos:
        v = np.zeros(len(base))
        for d in gait_demos:
            from scipy.spatial import cKDTree as _T
            idx = _T(trails[d][:, 1:3]).query(base)[1]; v += flags[d][idx]
        stairs_base = v / len(gait_demos) >= 0.5
    corridor = np.zeros((grid.nx, grid.ny), bool)
    for d in demos + patches:
        t = trails[d]; f = flags[d] if tl else np.zeros(len(t), bool)
        if (~f).any(): corridor |= grid.mask_near(t[~f, 1:3], a.corridor)
        if f.any(): corridor |= grid.mask_near(t[f, 1:3], a.corridor_stairs)
    for t in survey:
        corridor |= grid.mask_near(t[:, 1:3], 0.30)
    free = corridor & centre_ok
    sc, sc_idx = shortcut(grid, free, base, discs)

    def accept(k, arc):                                           # a rounded corner must not lose a waypoint touch
        lo, hi = sc_idx[max(0, k - 1)], sc_idx[min(len(sc_idx) - 1, k + 1)]
        piece = np.vstack([sc[max(0, k - 1)], arc, sc[min(len(sc) - 1, k + 1)]])
        return all(poly_dist(piece, c) <= r for a_, c, r in discs if lo <= a_ <= hi)
    line = resample(fillet(grid, free, sc, a.fillet, accept=accept), 0.10)
    line, hairpin_notes = hairpins(line, grid, free)
    print("dead-end turn-arounds:", hairpin_notes or "none")

    # ---- last gate: sweep the BODY along the line. Where it would overlap a real obstacle, go back to the line
    # the operator drove there (its body sweep is obstacle-free by definition) and blend in and out over 1 m.
    ref_track = ref[:, 1:3]

    def monotone_map(pts):
        """Index on the reference track for every line point, never going backwards (out-and-back stretches of
        the course lie on top of each other, so a plain nearest-point search would jump between the two passes)."""
        out, r, n_ref = np.zeros(len(pts), int), 0, len(ref_track)
        for k, p in enumerate(pts):
            guess = int(k / max(len(pts) - 1, 1) * (n_ref - 1))   # same fraction of the way round the course
            hi = min(n_ref, max(r + 80, guess + 300))             # never behind the last match, up to 15 m ahead of the guess
            lo = max(r, min(guess - 300, hi - 1)) if guess - 300 > r else r
            r = lo + int(np.argmin(np.linalg.norm(ref_track[lo:hi] - p, axis=1)))
            out[k] = r
        return out

    clear = distance_transform_edt(~real) * grid.res             # distance of every cell to a real obstacle
    repaired = []

    def repair(line, attempts=6):
        for attempt in range(6):
            bad = clr.bad_vertices(line)
            if not bad.any():
                break
            idx = np.where(bad)[0]; groups = np.split(idx, np.where(np.diff(idx) > 5)[0] + 1)
            if os.environ.get("TEACH_LINE_DEBUG"):
                vi_, dn_, _ = clr.check(line)
                for g in groups:
                    near_ = vi_[np.min(np.linalg.norm(clr.P[vi_][:, None, :] - line[g][None, :, :], axis=2), axis=1) < 0.8]
                    c_ = clr.P[near_].mean(axis=0) if len(near_) else line[g[0]]
                    kw = int(np.argmin(np.linalg.norm(W - c_, axis=1)))
                    print(f"   attempt {attempt}: s {g[0] * 0.1:.1f}-{g[-1] * 0.1:.1f} m, {len(near_)} obstacle points round {np.round(c_, 2)}, {np.linalg.norm(W[kw] - c_):.2f} m from {ids[kw]}, d_new min {dn_[near_].min() if len(near_) else 0:.2f}, d_op there {clr.d_op[near_].min() if len(near_) else 0:.2f}")
            rmap = monotone_map(line)
            pieces, cur = [], 0
            for g in groups:
                pad = 10 + 10 * attempt                                # 1 m each side, wider on every attempt
                lo, hi = max(cur, g[0] - pad), min(len(line) - 1, g[-1] + pad)

                def seamless(k):                                       # both lines coincide here, or there is room to cross over
                    ci, cj = grid.idx(line[k])
                    return np.linalg.norm(line[k] - ref_track[rmap[k]]) < 0.06 or clear[ci, cj] > 0.75
                n_ = 0
                while lo > cur and not seamless(lo) and n_ < 150: lo -= 1; n_ += 1
                n_ = 0
                while hi < len(line) - 1 and not seamless(hi) and n_ < 150: hi += 1; n_ += 1
                if hi <= lo or rmap[hi] <= rmap[lo]:
                    print(f"   repair skipped at {[round(float(v), 2) for v in line[g[0]]]}: no monotone stretch of the reference (lo {lo} hi {hi} ref {rmap[lo]}..{rmap[hi]})")
                    continue
                pieces.append(line[cur:lo]); pieces.append(ref_track[rmap[lo]:rmap[hi] + 1]); cur = hi + 1
                repaired.append(dict(s=round(float(g[0] * 0.1), 1), length=round(float((hi - lo) * 0.1), 1), attempt=attempt))
            pieces.append(line[cur:])
            line = resample(np.vstack([p_ for p_ in pieces if len(p_)]), 0.10)
        return line
    line = repair(line)
    # Still not clean (narrow channels, stretches of the course that lie on top of each other): give the whole leg
    # between two waypoints back to the operator's own line. Leg by leg this cannot fail: the operator's line is
    # never closer to anything than the operator was.
    def wp_indices(pts):
        out, cur = [], 0
        for w in W:
            d_ = np.linalg.norm(pts[cur:] - w, axis=1); near_ = np.where(d_ < max(0.9, d_.min() + 0.2))[0]; near_ = near_[near_ <= near_[0] + 400]
            cur = cur + int(near_[np.argmin(d_[near_])]); out.append(cur)
        return out
    legs_given_back = set()
    for round_ in range(6):
        bad = clr.bad_vertices(line)
        if not bad.any():
            break
        li_ = wp_indices(line); legs = set()
        for k_ in np.where(bad)[0]:
            leg = int(np.clip(np.searchsorted(li_, k_, side="right") - 1, 0, len(W) - 2))
            legs.update(range(max(0, leg - min(round_, 1)), min(len(W) - 1, leg + min(round_, 1) + 1)))      # neighbours too, from the 2nd round
        legs_given_back |= legs
        pieces = []
        for leg in range(len(W) - 1):
            pieces.append(ref_track[anchors_raw[leg]:anchors_raw[leg + 1] + 1] if leg in legs_given_back else line[li_[leg]:li_[leg + 1] + 1])
        line = resample(np.vstack([line[:li_[0]]] + pieces + [line[li_[-1] + 1:]]), 0.10)
    if legs_given_back:
        print("legs given back to the operator's line (no safe straightening found):", [f"{ids[k]}-{ids[k + 1]}" for k in sorted(legs_given_back)])
    bad = clr.bad_vertices(line)
    print("body sweep: %d stretches moved back to the operator's line; %d line points still on an obstacle" % (len(repaired), int(bad.sum())))
    if bad.any():
        where = [[round(float(v), 2) for v in line[k]] for k in np.where(bad)[0][::5][:10]]
        print("TEACH_LINE_UNSAFE: the body would touch a real obstacle at", where)

    # A waypoint the repaired line no longer reaches: bend the FINAL line towards the mark again, narrow bulges first
    # allowed, every candidate checked with the body sweep (the first bend was on the reference, before the repair).
    cur = 0
    for k, w in enumerate(W):
        d = np.linalg.norm(line[cur:] - w, axis=1); near = np.where(d < max(0.6, d.min() + 0.2))[0]; near = near[near <= near[0] + 300]
        i = cur + int(near[np.argmin(d[near])]); cur = i
        d_body, _ = body_touch(line, w, (i - 15, i + 16)); off = w - line[i]; dist = float(np.linalg.norm(off))
        if d_body <= a.body_touch or dist < 0.10: continue
        sl = np.r_[0, np.cumsum(np.linalg.norm(np.diff(line, axis=0), axis=1))]; ds = np.abs(sl - sl[i]); done = False
        for frac in (1.0, 0.8, 0.6, 0.4):
            for width in (2.0, 1.2, 0.8):
                wgt = np.where(ds < width, 0.5 * (1 + np.cos(np.pi * ds / width)), 0.0)
                cand = line + wgt[:, None] * (off * (1.0 - 0.06 / dist) * frac)[None, :]
                moved = np.where(wgt > 0)[0]; lo_, hi_ = max(0, moved[0] - 8), min(len(cand), moved[-1] + 9)
                ci, cj = grid.idx(cand[moved])
                if centre_ok[ci, cj].all() and not clr.bad_vertices(cand[lo_:hi_]).any():
                    nb, _ = body_touch(cand, w, (i - 15, i + 16))
                    if nb < d_body - 0.02:
                        line = cand; inserted.append(f"{ids[k]}: final line bent, body {d_body:.2f}->{nb:.2f}"); done = True; break
            if done: break
    line = resample(line, 0.10)

    # ---- jump lanes: the last metres before a ledge and the first after it are ONE straight, square to the ledge, through
    # the operators' crossing point. Nothing above may round or shortcut it; blended into the line on both sides. Every
    # candidate passes the body sweep and keeps the waypoint touches, else a shorter lane is tried.
    for J in jumps:
        E_, u_ = J["xy"], J["u"]; nrm_ = np.array([-u_[1], u_[0]]); J["lane"] = None
        i0 = int(np.argmin(np.linalg.norm(line - E_, axis=1)))
        if np.linalg.norm(line[i0] - E_) > 1.0:
            print("   JUMP LANE NOT APPLIED: the line does not pass the ledge at", np.round(E_, 2)); continue
        for before, blend in [(b_, l_) for b_ in (a.jump_before, 1.5, 1.0, 0.7) for l_ in (1.5, 1.0, 0.6)]:
            su, sv = (line - E_) @ u_, (line - E_) @ nrm_
            lo_ = i0
            while lo_ > 0 and su[lo_ - 1] > -(before + blend) and su[lo_ - 1] < su[lo_] + 0.05: lo_ -= 1
            hi_ = i0
            while hi_ < len(line) - 1 and su[hi_ + 1] < a.jump_after + blend and su[hi_ + 1] > su[hi_] - 0.05: hi_ += 1
            wgt = np.zeros(len(line)); k_ = np.arange(lo_, hi_ + 1); q_ = su[k_]
            wgt[k_] = np.where(q_ < -before, 0.5 * (1 + np.cos(np.pi * np.clip((-before - q_) / blend, 0, 1))), np.where(q_ > a.jump_after, 0.5 * (1 + np.cos(np.pi * np.clip((q_ - a.jump_after) / blend, 0, 1))), 1.0))
            cand = line - (wgt * sv)[:, None] * nrm_[None, :]
            ci, cj = grid.idx(cand[k_]); w_lo, w_hi = max(0, lo_ - 8), min(len(cand), hi_ + 9)
            touch_ok = all(body_touch(cand, W[kk], (w_lo, w_hi))[0] <= max(a.body_touch, body_touch(line, W[kk], (w_lo, w_hi))[0] + 0.01)
                           for kk in range(len(W)) if np.min(np.linalg.norm(line[w_lo:w_hi] - W[kk], axis=1)) < a.reach)
            if os.environ.get("TEACH_LINE_DEBUG"):
                print("      lane %.1f/%.1f: shift %.2f corridor %s touch %s sweep %s" % (before, blend, np.abs(wgt * sv).max(), bool(free[ci, cj].all()), touch_ok, not clr.bad_vertices(cand[w_lo:w_hi]).any()))
            if float(np.abs(wgt * sv).max()) <= a.jump_max_shift and free[ci, cj].all() and touch_ok and not clr.bad_vertices(cand[w_lo:w_hi]).any():
                line = cand; J["lane"] = dict(before=before, after=a.jump_after, blend=blend, moved=round(float(np.abs(wgt * sv).max()), 2)); break
        print("   jump lane at %s: %s" % (np.round(E_, 2), J["lane"] or "NOT APPLIED (no straight lane passes the body sweep and keeps the waypoints): the operator's own line stays"))
    line = resample(line, 0.10)
    # LAST check of the finished line (2026-09-22: a thin object beside the WP21-WP22 marking drive ended up under the
    # line after the steps above; verify_route_clearance refused the route). Repair again; a lane that gets cut is reported.
    n_final = int(clr.bad_vertices(line).sum())
    if os.environ.get("TEACH_LINE_DEBUG"):
        vi_, dn_, _ = clr.check(line); c_ = np.array([-10.85, -22.55]); k_ = np.argsort(np.linalg.norm(clr.P - c_, axis=1))[:4]
        print("DEBUG final: violations", len(vi_), [(np.round(clr.P[q], 2).tolist(), round(float(clr.d_op[q]), 2), round(float(dn_[q]), 2)) for q in k_], "line near:", round(float(np.linalg.norm(line - c_, axis=1).min()), 2))
    if n_final:
        line = resample(repair(line), 0.10)
        print("final check: %d line points on an obstacle after the waypoint bends / jump lanes -> repaired, %d left" % (n_final, int(clr.bad_vertices(line).sum())))
    bad = clr.bad_vertices(line)

    # pass-by point of every waypoint = its gate on the route (on walked ground), radius = reach - offset
    gates, cur = [], 0
    for k, w in enumerate(W):
        d = np.linalg.norm(line[cur:] - w, axis=1); near = np.where(d < max(0.6, d.min() + 0.2))[0]; near = near[near <= near[0] + 300]
        i = cur + int(near[np.argmin(d[near])]); cur = i
        off = float(np.linalg.norm(line[i] - w))
        d_body, i_body = body_touch(line, w, (i - 15, i + 16))
        # centre within r of the gate => body within 0.20 m of the mark, by either argument:
        r_centre = a.reach - off                                  # centre within 0.45 m of the mark, any heading
        r_body = 0.20 - d_body                                    # the body already reaches to d_body at the gate; moving r costs at most r
        if r_body > r_centre: i = i_body
        r = max(r_centre, r_body)
        if ids[k] in contact:
            i = cur; r = 0.15                                      # drive to the mark itself
        gates.append(dict(id=ids[k], i=i, off=float(np.linalg.norm(line[i] - w)), body=round(d_body, 3), radius=round(min(0.40, max(0.15, r)), 2), guaranteed=bool(r >= 0.15), contact=ids[k] in contact))
    b0, b1 = turn_stats(base), turn_stats(line)
    report_legs = [dict(leg="course", ref=ref_name[-6:], vertices=len(sc), reference_bent_to=inserted, length=[b0["length"], b1["length"]],
                        kinks15=[b0.get("kinks_over_15deg"), b1.get("kinks_over_15deg")], straight=[b0.get("straight_fraction"), b1.get("straight_fraction")],
                        turning_deg=[b0.get("total_turning_deg"), b1.get("total_turning_deg")])]
    weak = [(g["id"], round(g["off"], 2), g["body"]) for g in gates if not g["guaranteed"]]
    print("waypoints NOT guaranteed by the 0.20 m body rule on a safe line (id, centre distance, body distance):", weak or "none")
    print("reference:", ref_name, "| reference bent towards these marks (id:distance before):", inserted or "none")

    # Height: x_nav's z differs between passes of the same spot (decimetres), so one demonstration per leg
    # would put a step into the route at every waypoint. Median over all demonstrations, then smoothed.
    from scipy.spatial import cKDTree
    zs = []
    for d in demos + patches:
        t = trails[d]; dist, idx = cKDTree(t[:, 1:3]).query(line)
        zs.append(np.where(dist < 1.0, t[idx, 3], np.nan))
    z = np.nanmedian(np.array(zs), axis=0)
    z = np.interp(np.arange(len(z)), np.where(~np.isnan(z))[0], z[~np.isnan(z)])
    k = np.ones(21) / 21.0
    z = np.convolve(np.pad(z, 10, mode="edge"), k, mode="valid")
    spread = np.nanmax(np.array(zs), axis=0) - np.nanmin(np.array(zs), axis=0)
    print("x_nav height disagreement between demonstrations: median %.2f m, worst %.2f m" % (np.nanmedian(spread), np.nanmax(spread)))
    yaw = np.arctan2(np.gradient(line[:, 1]), np.gradient(line[:, 0]))

    # gait zones: consensus of the gait demos, projected on the new line
    tree = cKDTree(line); sline = np.r_[0, np.cumsum(np.linalg.norm(np.diff(line, axis=0), axis=1))]
    votes = np.zeros(len(line)); n_votes = 0
    for d in gait_demos:
        t = trails[d]; idx = tree.query(t[:, 1:3])[1]
        f = np.zeros(len(line)); c = np.zeros(len(line))
        np.add.at(f, idx, flags[d].astype(float)); np.add.at(c, idx, 1.0)
        known = c > 0
        v = np.interp(np.arange(len(line)), np.where(known)[0], (f[known] / c[known])) if known.any() else np.zeros(len(line))
        votes += v; n_votes += 1
    stairs_line = (votes / max(n_votes, 1)) >= 0.5 if n_votes else np.zeros(len(line), bool)
    gate_i = [g["i"] for g in gates]
    keep_op = np.zeros(len(line), bool)
    for sp in spliced:                                            # where a partial recording is the reference, its gait counts
        t = trails[sp["recording"]]; lo_, hi_ = gate_i[sp["k"][0]], gate_i[sp["k"][1]]
        dist, idx = cKDTree(t[:, 1:3]).query(line[lo_:hi_ + 1])
        if tl: stairs_line[lo_:hi_ + 1] = np.where(dist < 0.8, flags[sp["recording"]][idx], stairs_line[lo_:hi_ + 1])
        if tl and pflags[sp["recording"]].any():                 # a platform-gait teaching: the operator's gait as driven, from its first
            keep_op[lo_:hi_ + 1] = True                           # waypoint (2026-09-22: stairs gait from WP22, no switch between WP22 and WP23)
    # terrain: height of x_nav's ground cloud under the line; steep = steps or slope. The stairs gait is slow, so it is
    # only kept where the operator used it AND the ground needs it.
    gtree = cKDTree(ground_pts[:, :2]); zg = np.full(len(line), np.nan)
    for i_, nb in enumerate(gtree.query_ball_point(line, 0.25)):
        if len(nb) >= 3: zg[i_] = np.percentile(ground_pts[nb, 2], 20)
    okz = ~np.isnan(zg); zg = np.interp(np.arange(len(line)), np.where(okz)[0], zg[okz]) if okz.any() else np.zeros(len(line))
    n_ = 10                                                       # 1 m each side at 0.1 m spacing
    slope = np.abs(np.r_[np.zeros(n_), zg[2 * n_:] - zg[:-2 * n_], np.zeros(n_)]) / (2 * n_ * 0.1)
    step = np.array([np.ptp(zg[max(0, i_ - 5):i_ + 6]) for i_ in range(len(line))])
    steep = (slope > 0.15) | (step > 0.15)
    from scipy.ndimage import binary_dilation as _dil, binary_closing as _close
    if a.terrain_zones and steep.any():
        need = _dil(steep, iterations=15)                         # 1.5 m before and after
        stairs_line = (stairs_line & need) | (stairs_line & keep_op)
        stairs_line = _close(np.pad(stairs_line, 60), iterations=30)[60:-60]    # gaps < 6 m are not worth two gait switches
    def runs(mask, min_len):
        out, k_ = [], 0
        while k_ < len(mask):
            if mask[k_]:
                k1_ = k_
                while k1_ + 1 < len(mask) and mask[k1_ + 1]: k1_ += 1
                if sline[k1_] - sline[k_] >= min_len: out.append((k_, k1_))
                k_ = k1_ + 1
            else:
                k_ += 1
        return out
    steep_runs = [(round(float(sline[k0]), 1), round(float(sline[k1]), 1)) for k0, k1 in runs(_dil(steep, iterations=8) if steep.any() else steep, 0.5)]
    # ---- four-gait plan: platform (0x1002) where an operator recording used it; stairs as above; FAST (0xF002) on open,
    # straight, flat ground; the rest = walk (0x3002, slow). A switch costs ~3 s standing, so short runs are absorbed.
    platform_line = np.zeros(len(line), bool)
    plat_refs = [sp["recording"] for sp in spliced if pflags[sp["recording"]].any()]
    for J in jumps:                                               # only the jump itself; the switch is made standing
        sJ = sline[int(np.argmin(np.linalg.norm(line - J["xy"], axis=1)))]
        pb_at = {q.split(":")[0].upper(): float(q.split(":")[1]) for q in a.platform_before_at.split(",") if ":" in q}
        J["before_gait"] = pb_at.get(ids[int(np.argmin(np.linalg.norm(W - J["xy"], axis=1)))], a.platform_before)
        platform_line |= (sline >= sJ - J["before_gait"]) & (sline <= sJ + a.platform_after)
    if not jumps:                                                 # no ledge found: wherever an operator used the gait
        for d in (plat_refs or demos + patches):
            if pflags[d].any():
                dist, idx = cKDTree(line).query(trails[d][pflags[d], 1:3])
                platform_line[idx[dist < 0.8]] = True
        if platform_line.any():
            platform_line = _close(np.pad(_dil(platform_line, iterations=5), 30), iterations=15)[30:-30]   # +-0.5 m, gaps < 3 m
    if platform_line.any():
        stairs_line = stairs_line & ~platform_line
    hd = np.unwrap(np.arctan2(np.gradient(line[:, 1]), np.gradient(line[:, 0])))
    w_ = 10
    turn = np.abs(np.r_[np.zeros(w_), hd[2 * w_:] - hd[:-2 * w_], np.zeros(w_)])                     # heading change over 2 m
    li_, lj_ = grid.idx(line); narrow = clear[li_, lj_] < (HW + a.fast_clearance)
    fast_line = ~stairs_line & ~platform_line & ~_dil(steep, iterations=10) & ~_dil(turn > math.radians(a.fast_turn_deg), iterations=8) & ~_dil(narrow, iterations=5)
    fast_line = _close(np.pad(fast_line, 50), iterations=20)[50:-50] & ~stairs_line & ~platform_line   # walk gaps < 4 m absorbed
    keep = np.zeros(len(line), bool)
    for k0, k1 in runs(fast_line, a.fast_min_run): keep[k0:k1 + 1] = True
    fast_line = keep if a.fast_gait else np.zeros(len(line), bool)
    gait_plan = sorted([(round(float(sline[k0]), 1), round(float(sline[k1]), 1), name) for mask, name, ml in ((platform_line, "platform", 0.5), (stairs_line, "stairs", 1.0), (fast_line, "fast", 1.0)) for k0, k1 in runs(mask, ml)])
    gait_plan = [list(g) for g in gait_plan]
    if a.platform_absorb > 0:                                     # short stairs stretch touching a platform zone -> platform
        for k_, g in enumerate(gait_plan):
            nb_ = [q for q in (gait_plan[k_ - 1] if k_ else None, gait_plan[k_ + 1] if k_ + 1 < len(gait_plan) else None) if q is not None]
            if g[2] == "stairs" and g[1] - g[0] < a.platform_absorb and any(q[2] == "platform" and (abs(q[0] - g[1]) < 0.15 or abs(g[0] - q[1]) < 0.15) for q in nb_):
                g[2] = "platform"
        merged = []
        for g in gait_plan:
            if merged and merged[-1][2] == g[2] and g[0] - merged[-1][1] < 0.15: merged[-1][1] = g[1]
            else: merged.append(g)
        gait_plan = merged
    for ga, gb in zip(gait_plan, gait_plan[1:]):                  # a walk gap < 3 m between two zones = two extra switches: close it
        if 0.0 < gb[0] - ga[1] < 3.0:
            if ga[2] == "fast": ga[1] = gb[0]
            elif gb[2] == "fast": gb[0] = ga[1]
            else: ga[1] = gb[0]
    gait_plan = [tuple(g) for g in gait_plan]
    tot = {n: round(sum(b - a_ for a_, b, m in gait_plan if m == n), 1) for n in ("fast", "stairs", "platform")}
    print("gait plan (m): fast %.0f, stairs %.0f, platform %.0f, walk %.0f; %d switches" % (tot["fast"], tot["stairs"], tot["platform"], sline[-1] - sum(tot.values()), 2 * len(gait_plan)))
    zones, k = [], 0
    while k < len(line):
        if stairs_line[k]:
            k1 = k
            while k1 + 1 < len(line) and stairs_line[k1 + 1]: k1 += 1
            if sline[k1] - sline[k] >= 1.0: zones.append((k, k1))
            k = k1 + 1
        else:
            k += 1

    # derived session for the unchanged teach_to_route.py
    ds = os.path.join(a.out, "derived_session"); os.makedirs(ds, exist_ok=True)
    with open(os.path.join(ds, "path_00000000_teachline.trail.csv"), "w") as f:
        f.write("wall_time,x,y,z,yaw\n")
        for i in range(len(line)): f.write("%.3f,%.4f,%.4f,%.4f,%.5f\n" % (i * 0.1, line[i, 0], line[i, 1], z[i], yaw[i]))
    seq = 0
    with open(os.path.join(ds, "marks.jsonl"), "w") as f:
        for g in gates:
            seq += 1; r = dict(wps[g["id"]]); r["seq"] = seq
            r["wp_true"] = list(r["pose"]); r["radius_xy"] = g["radius"]
            r["pose"] = [float(line[g["i"], 0]), float(line[g["i"], 1]), float(z[g["i"]]), float(yaw[g["i"]])]
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
        src = zones if n_votes else []
        for (k0, k1) in src:
            for kind, kk in (("SWIN", k0), ("SWOUT", k1)):
                seq += 1
                f.write(json.dumps(dict(seq=seq, kind=kind, wp_id=None, note="operator gait (teach_line)", pose=[float(line[kk, 0]), float(line[kk, 1]), float(z[kk]), float(yaw[kk])],
                                        result=dict(passed=True))) + "\n")
        if not n_votes:                                          # fallback: the operator's own switch marks
            for r in live:
                if r["kind"] in ("SWIN", "SWOUT") and (r.get("result") or {}).get("passed"):
                    seq += 1; r = dict(r); r["seq"] = seq; f.write(json.dumps(r, ensure_ascii=False) + "\n")

    # for the run time: where the ground is steep (slower inside a stairs zone) and which waypoints are driven INTO
    json.dump(dict(schema="s10_route_terrain_v1", steep=steep_runs, gaits=[list(g) for g in gait_plan] if a.gait_plan else [],
                   jumps=[dict(xy=[round(float(v), 3) for v in J["xy"]], yaw=round(J["yaw"], 4), rise=round(J["rise"], 2), before=J["lane"]["before"], after=J["lane"]["after"]) for J in jumps if J.get("lane")],
                   contact=[dict(id=g["id"], xy=[float(v) for v in wps[g["id"]]["pose"][:2]], radius=a.contact_radius) for g in gates if g.get("contact")]),
              open(os.path.join(ds, "terrain.json"), "w"), indent=1)
    before = {d: turn_stats(trails[d][:, 1:3]) for d in demos}; after = turn_stats(line)
    miss = {g["id"]: round(g["off"], 3) for g in gates}
    rep = dict(session=a.session, demos=demos, gait_demos=gait_demos, before=before, after=after, worst_wp_distance=max(miss.values()),
               wp_distance=miss, zones=[dict(s0=round(float(sline[k0]), 1), s1=round(float(sline[k1]), 1), xy0=[round(float(v), 2) for v in line[k0]], xy1=[round(float(v), 2) for v in line[k1]]) for k0, k1 in zones],
               legs=report_legs, gait_plan=[list(g) for g in gait_plan], spliced=[dict(recording=x['recording'], first=x['first'], last=x['last']) for x in spliced], steep=steep_runs, params=dict(contact=contact, dropped=a.drop, corridor=a.corridor, corridor_stairs=a.corridor_stairs, clearance=a.clearance, fillet=a.fillet, reach=a.reach, touch=a.touch, free_at=freed), hairpins=hairpin_notes, body_sweep=dict(repaired=repaired, legs_given_back=[f"{ids[k]}-{ids[k + 1]}" for k in sorted(legs_given_back)], points_on_obstacle=int(bad.sum())), gates=[dict(id=g["id"], offset=round(g["off"], 3), body_distance=g["body"], radius=g["radius"], guaranteed=g["guaranteed"]) for g in gates])
    json.dump(rep, open(os.path.join(a.out, "teach_line.json"), "w"), indent=1, ensure_ascii=False)
    print("before:", {d[-6:]: (v["length"], v["kinks_over_15deg"], v["straight_fraction"], v["total_turning_deg"]) for d, v in before.items()})
    print("after :", (after["length"], after["kinks_over_15deg"], after["straight_fraction"], after["total_turning_deg"]), "(length m, kinks > 15 deg, straight fraction, total turning deg)")
    print("worst WP distance %.3f m; stairs zones (s0-s1 m): %s" % (rep["worst_wp_distance"], [(z_["s0"], z_["s1"]) for z_ in rep["zones"]]))

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    except ImportError:
        return
    ext = [grid.x0, grid.x0 + grid.nx * grid.res, grid.y0, grid.y0 + grid.ny * grid.res]
    def draw(ax, box=None, lw=1.6):
        ax.imshow(np.where(real, 0.3, np.where(blocked, 0.78, np.where(occ & demo_body, 0.93, 1.0))).T, origin="lower", extent=ext, cmap="gray", vmin=0, vmax=1, interpolation="nearest")
        for d in demos: ax.plot(trails[d][:, 1], trails[d][:, 2], lw=0.7, alpha=0.8, label="taught " + d[-6:])
        ax.plot(line[~stairs_line, 0], line[~stairs_line, 1], ".", ms=lw * 1.6, c="#0a58ca", label="new line, flat gait")
        ax.plot(line[stairs_line, 0], line[stairs_line, 1], ".", ms=lw * 1.6, c="#d62728", label="new line, stairs gait")
        for i, w in enumerate(ids):
            ax.plot(*W[i], "o", ms=7, mfc="none", mec="k"); ax.annotate(w[2:], W[i] + 0.3, fontsize=8, weight="bold")
        ax.set_aspect("equal")
        if box: ax.set_xlim(box[0], box[1]); ax.set_ylim(box[2], box[3])
    fig, ax = plt.subplots(figsize=(18, 11)); draw(ax); ax.legend(loc="upper left", fontsize=8)
    ax.set_title("teach line: thin = operator's demonstrations, dots = straightened line (blue flat, red stairs); dark = real obstacles, grey = centre keep-out (0.35 m)")
    fig.savefig(os.path.join(a.out, "teach_line.png"), dpi=120, bbox_inches="tight"); plt.close(fig)
    far = sorted(gates, key=lambda g: -g["off"])[:4]              # the waypoints the line passes furthest from
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    for ax, g in zip(axes.ravel(), far):
        k = ids.index(g["id"]); c = W[k]; h = 3.0
        draw(ax, (c[0] - h, c[0] + h, c[1] - h, c[1] + h), lw=3)
        ax.add_patch(plt.Circle(c, a.reach, fill=False, ec="#2ca02c", lw=1.5)); ax.plot(*line[g["i"]], "s", ms=8, mfc="none", mec="#2ca02c")
        ax.set_title(f"{g['id']}: line passes {g['off']:.2f} m from the mark\ncircle = touch range {a.reach} m, square = gate (radius {g['radius']} m)", fontsize=10)
    fig.savefig(os.path.join(a.out, "teach_line_zoom.png"), dpi=110, bbox_inches="tight")
    if bad.any():
        sys.exit(3)


if __name__ == "__main__":
    main()
