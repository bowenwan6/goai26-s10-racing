"""A* on the follower's local grid, from where the robot is back onto the route ahead.

Used when the route follower refuses to drive -- off its corridor after a slide down a side slope,
or blocked -- instead of driving the taught line blind. The grid is the follower's own ``LocalGrid``
(0.1 m, world aligned, UNKNOWN / FREE / SCAN_CLEAR / OBSTACLE), so the path respects exactly the
evidence the follower has:

* OBSTACLE and UNKNOWN are not traversable, dilated by the body's half width plus a margin; the
  cells under the robot's own footprint are exempt (occluded, not unknown ground);
* FREE costs 1 per cell, SCAN_CLEAR (no body-height return, ground not verified) 1.5;
* each cell also pays for its distance from the route centreline, so the path rejoins rather than
  wandering;
* goals are centreline points between ``goal_ahead`` metres ahead of the robot's projection, the
  first reachable one wins (the nearest on the route that the robot can actually get to).

This is the grid-A* counterpart of Nav2's cost-aware Smac planners, sized to the 8 m local window.
"""

from __future__ import annotations

import heapq
import math

import numpy as np
from scipy import ndimage

from s10_auto_nav.route_planner import OBSTACLE, SCAN_CLEAR, UNKNOWN


def plan(
    grid,
    pose,
    path,
    s_now,
    goal_ahead=(0.8, 3.0),
    half_width=0.3,
    margin=0.15,
    w_route=2.0,
    max_expansions=40000,
):
    """Returns (N, 2) waypoints in map frame from the robot to the route, or None."""
    res = grid.resolution
    state = grid.state
    blocked = (state == OBSTACLE) | (state == UNKNOWN)
    r = math.ceil((half_width + margin) / res)
    yy, xx = np.mgrid[-r : r + 1, -r : r + 1]
    blocked = ndimage.binary_dilation(blocked, structure=(xx * xx + yy * yy) <= r * r)
    x, y, yaw = pose
    ix, iy, inside = grid.index(np.array([[x, y]]))
    if not inside[0]:
        return None
    start = (int(ix[0]), int(iy[0]))
    # Exempt the footprint (0.9 x 0.5 m) around the robot.
    c = grid.cell_centers()
    rel = c - np.array([x, y])
    lon = rel[..., 0] * math.cos(yaw) + rel[..., 1] * math.sin(yaw)
    lat = -rel[..., 0] * math.sin(yaw) + rel[..., 1] * math.cos(yaw)
    blocked[(np.abs(lon) <= 0.5) & (np.abs(lat) <= 0.3)] = False

    # Distance of every cell to the route centreline (only the stretch near the robot matters).
    s_lo, s_hi = max(0.0, s_now - 2.0), min(path.length, s_now + goal_ahead[1] + 2.0)
    ss = np.arange(s_lo, s_hi, 0.1)
    line = np.array([np.asarray(path.point_at(v))[:2] for v in ss])
    d_route = np.full(state.shape, 5.0)
    if len(line):
        flat = c.reshape(-1, 2)
        dmin = np.full(len(flat), np.inf)
        for p in line:
            dmin = np.minimum(dmin, np.hypot(flat[:, 0] - p[0], flat[:, 1] - p[1]))
        d_route = np.minimum(dmin.reshape(state.shape), 5.0)
    step_cost = np.where(state == SCAN_CLEAR, 1.5, 1.0) + w_route * d_route

    goals = {}
    for v in np.arange(s_now + goal_ahead[0], min(path.length, s_now + goal_ahead[1]) + 1e-9, 0.2):
        gx, gy, gin = grid.index(np.array([np.asarray(path.point_at(v))[:2]]))
        if gin[0] and not blocked[gx[0], gy[0]]:
            goals[(int(gx[0]), int(gy[0]))] = v
    if not goals:
        return None
    gpts = np.array(list(goals.keys()), float)

    def h(cell):
        return float(np.min(np.hypot(gpts[:, 0] - cell[0], gpts[:, 1] - cell[1])))

    nbrs = [
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)),
        (-1, -1, math.sqrt(2)),
    ]
    g = {start: 0.0}
    came = {}
    heap = [(h(start), 0.0, start)]
    seen = 0
    nx, ny = state.shape
    while heap and seen < max_expansions:
        _, gc, cell = heapq.heappop(heap)
        if gc > g.get(cell, math.inf) + 1e-9:
            continue
        seen += 1
        if cell in goals:
            out = [cell]
            while out[-1] in came:
                out.append(came[out[-1]])
            out.reverse()
            xy = grid.origin + (np.array(out, float) + 0.5) * res
            return xy
        for dx, dy, w in nbrs:
            n = (cell[0] + dx, cell[1] + dy)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny) or blocked[n]:
                continue
            ng = gc + w * step_cost[n]
            if ng < g.get(n, math.inf) - 1e-9:
                g[n] = ng
                came[n] = cell
                heapq.heappush(heap, (ng + h(n), ng, n))
    return None


def pursue_path(xy_path, pose, speed, max_yaw_rate=0.6, lookahead=0.45):
    """Pure pursuit along an A* path: (vx, vy, wz)."""
    x, y, yaw = pose
    d = np.hypot(xy_path[:, 0] - x, xy_path[:, 1] - y)
    k = int(np.argmin(d))
    ahead = np.flatnonzero((np.arange(len(d)) >= k) & (d >= lookahead))
    tgt = xy_path[ahead[0]] if len(ahead) else xy_path[-1]
    err = math.atan2(tgt[1] - y, tgt[0] - x) - yaw
    err = (err + math.pi) % (2 * math.pi) - math.pi
    if abs(err) > math.radians(35):
        return 0.0, 0.0, float(np.clip(1.5 * err, -max_yaw_rate, max_yaw_rate))
    wz = float(np.clip(1.5 * err, -max_yaw_rate, max_yaw_rate))
    return speed * max(0.3, 1 - abs(err) / math.radians(60)), 0.0, wz
