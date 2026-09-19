"""A path on the map from where the robot is back onto the route: the global planner of the recovery
layer.

The follower's own planner works on its 8 x 8 m local grid, which reads rough ground -- a slope, the
rocks of a creek bed, a terrace edge -- as blocked or unknown; on this course it refuses more than
half the time between some waypoints (the first version's logs). The map knows that ground. This
plans on the map raster instead (the ``MapSurface`` route preparation used), for the walking actor:

* a cell is a hazard if the map does not know it, a static obstacle stands on it, a jump taller than
  ``cliff`` borders it, or anything the robot has seen that the map does not have (``extra_xy``,
  from ``map_check``) lies in it;
* cells closer than ``half_width + margin`` to a hazard are not traversable, and cells with a
  step over ``max_step`` or a slope over ``max_slope_deg`` are not either (the walking actor takes
  neither on a rejoin path; the route's own climbs are the stairs actor's);
* each cell costs its length, more within ``soft`` of a hazard, and a little per metre away from
  the route, so the path rejoins rather than wandering;
* goals are route points between two arc lengths; the cheapest reachable one wins, preferring the
  one nearest ``s_target``.

Dijkstra (``scipy.sparse.csgraph``) over an 8-connected grid at ``res`` (0.1 m) cropped around the
robot and the goals, then shortcut by line of sight. Sub-10 ms on a 10 x 10 m window.
"""

from __future__ import annotations

import math

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra


class MapPlanner:
    def __init__(
        self,
        surface,
        res=0.1,
        half_width=0.3,
        margin=0.1,
        soft=0.4,
        cliff=0.3,
        obstacle=0.05,
        max_step=0.1,
        max_slope_deg=15.0,
    ):
        self.surface = surface
        self.res = float(res)
        self.half_width, self.margin, self.soft = half_width, margin, soft
        self.cliff, self.obstacle = cliff, obstacle
        self.max_step, self.max_slope = max_step, math.radians(max_slope_deg)
        k = max(1, round(self.res / surface.res))
        self.k = k
        g, kn = surface.ground, surface.known
        obst = surface.surface - surface.ground
        ny, nx = (g.shape[0] // k) * k, (g.shape[1] // k) * k
        blocks = (ny // k, k, nx // k, k)
        kn_b = kn[:ny, :nx].reshape(blocks)
        n_known = kn_b.sum(axis=(1, 3))
        g_sum = np.where(kn[:ny, :nx], g[:ny, :nx], 0.0).reshape(blocks).sum(axis=(1, 3))
        self.ground = np.where(n_known > 0, g_sum / np.maximum(n_known, 1), np.nan)
        self.known = n_known >= (k * k + 1) // 2
        self.obst = np.nan_to_num(obst[:ny, :nx]).reshape(blocks).max(axis=(1, 3))
        self.origin = surface.origin
        self.hazard, self.rough = self._hazards()
        self.clear = ndimage.distance_transform_edt(~self.hazard) * self.res

    def _hazards(self):
        g = np.where(self.known, self.ground, np.nan)
        haz = ~self.known | (self.obst > self.obstacle)
        rough = np.zeros_like(haz)
        for ax in (0, 1):
            dz = np.abs(np.diff(g, axis=ax))
            jump = np.nan_to_num(dz, nan=0.0) > self.cliff
            step = np.nan_to_num(dz, nan=0.0) > self.max_step
            sl = [slice(None), slice(None)]
            for side in (slice(None, -1), slice(1, None)):
                sl[ax] = side
                haz[tuple(sl)] |= jump
                rough[tuple(sl)] |= step
        # Slope over 0.3 m (the body's half length): a riser is a step, not a slope.
        span = max(1, round(0.3 / self.res))
        gf = np.where(self.known, self.ground, np.nanmedian(self.ground))
        gy, gx = np.gradient(ndimage.uniform_filter(gf, size=span), self.res)
        rough |= np.hypot(gx, gy) > math.tan(self.max_slope)
        return haz, rough

    # ------------------------------------------------------------------ lookups
    def cell(self, xy):
        xy = np.atleast_2d(np.asarray(xy, float))
        ix = np.floor((xy[:, 0] - self.origin[0]) / self.res).astype(int)
        iy = np.floor((xy[:, 1] - self.origin[1]) / self.res).astype(int)
        ny, nx = self.hazard.shape
        inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        return np.clip(ix, 0, nx - 1), np.clip(iy, 0, ny - 1), inside

    def clearance_at(self, xy):
        """Distance (m) to the nearest map hazard; 0 outside the map."""
        ix, iy, inside = self.cell(xy)
        return np.where(inside, self.clear[iy, ix], 0.0)

    def rough_at(self, xy):
        ix, iy, inside = self.cell(xy)
        return np.where(inside, self.rough[iy, ix], True)

    def line_clear(self, a, b, need=None, step=0.1):
        """Is the straight way from a to b clear of map hazards by ``need`` (half width default)?"""
        need = self.half_width if need is None else need
        a, b = np.asarray(a, float)[:2], np.asarray(b, float)[:2]
        n = max(2, math.ceil(float(np.hypot(*(b - a))) / step) + 1)
        pts = a + np.linspace(0.0, 1.0, n)[:, None] * (b - a)
        return bool(np.all(self.clearance_at(pts) >= need))

    # ------------------------------------------------------------------ planning
    def plan(
        self,
        start,
        path,
        s_goal=(1.0, 3.0),
        s_target=None,
        extra_xy=None,
        window=3.0,
        w_route=0.3,
        w_goal=0.5,
    ):
        """(N, 2) map-frame path from ``start`` to a route point with arc length in ``s_goal``, or
        None. ``extra_xy``: points the map does not have (blocked, with the usual clearance)."""
        s_lo, s_hi = max(0.0, s_goal[0]), min(path.length, s_goal[1])
        if s_hi <= s_lo:
            return None
        s_target = s_lo if s_target is None else s_target
        s_g = np.arange(s_lo, s_hi + 1e-9, self.res)
        goals_xy = np.array([np.asarray(path.point_at(s))[:2] for s in s_g])
        start = np.asarray(start, float)[:2]
        pts = np.vstack([goals_xy, start[None, :]])
        lo = pts.min(axis=0) - window
        hi = pts.max(axis=0) + window
        ix0, iy0, _ = self.cell(lo)
        ix1, iy1, _ = self.cell(hi)
        ix0, iy0, ix1, iy1 = int(ix0[0]), int(iy0[0]), int(ix1[0]) + 1, int(iy1[0]) + 1
        haz = self.hazard[iy0:iy1, ix0:ix1].copy()
        rough = self.rough[iy0:iy1, ix0:ix1]
        if extra_xy is not None and len(extra_xy):
            ex, ey, inside = self.cell(extra_xy)
            ex, ey = ex[inside] - ix0, ey[inside] - iy0
            ok = (ex >= 0) & (ex < haz.shape[1]) & (ey >= 0) & (ey < haz.shape[0])
            haz[ey[ok], ex[ok]] = True
        clear = ndimage.distance_transform_edt(~haz) * self.res
        need = self.half_width + self.margin
        blocked = (clear < need) | rough
        h, w = haz.shape
        yy, xx = np.mgrid[0:h, 0:w]
        cx = self.origin[0] + (xx + ix0 + 0.5) * self.res
        cy = self.origin[1] + (yy + iy0 + 0.5) * self.res
        # The robot stands where it stands: free the cells around it unless they are hazards.
        near_start = np.hypot(cx - start[0], cy - start[1]) <= self.half_width + 0.05
        blocked[near_start & ~haz] = False
        # Distance to the route (the stretch in the window).
        s_w = np.arange(max(0.0, s_lo - 8.0), min(path.length, s_hi + 1.0), self.res)
        route_xy = np.array([np.asarray(path.point_at(s))[:2] for s in s_w])
        route_mask = np.zeros_like(haz)
        rx = np.floor((route_xy[:, 0] - self.origin[0]) / self.res).astype(int) - ix0
        ry = np.floor((route_xy[:, 1] - self.origin[1]) / self.res).astype(int) - iy0
        ok = (rx >= 0) & (rx < w) & (ry >= 0) & (ry < h)
        route_mask[ry[ok], rx[ok]] = True
        d_route = ndimage.distance_transform_edt(~route_mask) * self.res
        cost = 1.0 + np.clip((self.soft + need - clear) / self.soft, 0.0, 1.0) * 2.0
        cost += w_route * np.minimum(d_route, 2.0)

        idx = np.arange(h * w).reshape(h, w)
        rows, cols, wts = [], [], []
        free = ~blocked
        for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
            fa, fb = _pair(free, dy, dx)
            ia, ib = _pair(idx, dy, dx)
            ca, cb = _pair(cost, dy, dx)
            both = fa & fb
            wgt = 0.5 * (ca + cb)[both] * self.res * math.hypot(dx, dy)
            rows += [ia[both], ib[both]]
            cols += [ib[both], ia[both]]
            wts += [wgt, wgt]
        if not rows:
            return None
        graph = coo_matrix(
            (np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))), shape=(h * w,) * 2
        ).tocsr()
        sx, sy, _ = self.cell(start)
        s_idx = int((sy[0] - iy0) * w + (sx[0] - ix0))
        if not (0 <= s_idx < h * w) or blocked.flat[s_idx]:
            return None
        dist, pred = dijkstra(graph, indices=s_idx, return_predecessors=True)
        gx, gy, g_in = self.cell(goals_xy)
        gx, gy = gx - ix0, gy - iy0
        best, best_score = None, math.inf
        for k in range(len(goals_xy)):
            if not g_in[k] or not (0 <= gx[k] < w and 0 <= gy[k] < h):
                continue
            j = int(gy[k] * w + gx[k])
            if not np.isfinite(dist[j]):
                continue
            score = dist[j] + w_goal * abs(s_g[k] - s_target)
            if score < best_score:
                best, best_score = (j, k), score
        if best is None:
            return None
        j, k = best
        chain = [j]
        while chain[-1] != s_idx:
            chain.append(int(pred[chain[-1]]))
            if chain[-1] < 0:
                return None
        chain.reverse()
        cells = np.array([[cx.flat[c], cy.flat[c]] for c in chain])
        cells[0] = start
        cells[-1] = goals_xy[k]
        return self._shortcut(cells, blocked, ix0, iy0)

    def _shortcut(self, cells, blocked, ix0, iy0):
        """Line-of-sight shortcut of the cell chain (a path that turns only where it has to)."""
        h, w = blocked.shape

        def seen(a, b):
            n = max(2, math.ceil(float(np.hypot(*(b - a))) / (0.5 * self.res)) + 1)
            p = a + np.linspace(0.0, 1.0, n)[:, None] * (b - a)
            px = np.floor((p[:, 0] - self.origin[0]) / self.res).astype(int) - ix0
            py = np.floor((p[:, 1] - self.origin[1]) / self.res).astype(int) - iy0
            ok = (px >= 0) & (px < w) & (py >= 0) & (py < h)
            if not ok.all():
                return False
            # The first 0.35 m may be in the start's freed disc (checked as the planner did).
            return not blocked[py[1:], px[1:]].any()

        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not seen(cells[i], cells[j]):
                j -= 1
            out.append(cells[j])
            i = j
        return np.array(out)


def _pair(arr, dy, dx):
    """The grid and itself shifted by (dy, dx), dy >= 0: every neighbour pair once."""
    h, w = arr.shape[:2]
    return (
        arr[0 : h - dy, max(0, -dx) : w - max(0, dx)],
        arr[dy:h, max(0, dx) : w - max(0, -dx)],
    )
