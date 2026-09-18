"""Route-relative local planner: Frenet lateral-offset sampling + bounded A* fallback.

The legacy ``LocalPlanner`` scores headings by how far they deviate from the bearing to the
next gate. Once it has steered round something it has no notion of "the route", so the
robot cuts straight to the next gate. Here every candidate is expressed relative to the
taught centreline (:class:`~s10_auto_nav.route_v2.RoutePath`):

* A candidate is a smooth lateral transition from the current offset ``d0`` to a target
  offset ``d_t`` in ``[-W, +W]`` along the next 2-4 m of arc length (quintic in ``s``;
  Werling et al., ICRA 2010). ``W`` is the segment's ``corridor_half_width``. With
  ``allow_detour = false`` the only candidate converges to ``d = 0``. The chosen
  transition stays anchored where it was first chosen (plan continuity) and is tracked by
  pure pursuit; switching target needs a cost advantage (``switch_margin``).
* Each candidate's robot footprint (rectangle, swept at the heading pure pursuit will
  actually take, plus the in-place pivot at the start) is checked against a robot-centred,
  world-aligned local grid built from the 13x9 height grid (+ valid mask) and the 72-bin
  conservative scan. UNKNOWN cells are never free; step limits depend on the gait.
* ``cost = w_obs*proximity + w_free*(1 - free/H) + w_lat*|d_t| + w_smooth*|d_t - prev|
  - w_progress*free/H (+ side-switch penalty)``. With no obstacle every candidate is fully
  free and ``w_lat > w_smooth`` makes ``d_t = 0`` win, so the robot tracks the taught line
  and rejoins it after a detour (lateral-weighted idea of VT&R3 "Along Similar Lines" /
  "Off the Beaten Track").
* All candidates blocked and detours allowed -> 8-connected grid A* inside a ~6x6 m
  window, cells outside the corridor forbidden, lateral offset added to the cell cost,
  goal = centreline 5..3 m ahead, else the reachable KNOWN cell with the most route
  progress (never into UNKNOWN). The A* path is re-checked with the rectangular footprint.
  Failure -> ``BLOCKED`` with a reason. Nothing here invents free space or plans a reverse.

Output (:class:`PlanResult`) is a short world-frame path, a carrot for the existing
``PurePursuitController.compute`` and a speed scale. Pure numpy; no ROS.
"""

from __future__ import annotations

import heapq
import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from s10_auto_nav.route_v2 import Projection, RoutePath, RouteSegment

UNKNOWN, FREE, SCAN_CLEAR, OBSTACLE = 0, 1, 2, 3
STATE_NAMES = {UNKNOWN: "unknown", FREE: "free", SCAN_CLEAR: "scan_clear", OBSTACLE: "obstacle"}

#: Height grid produced by ``real_transfer.geometry.height_grid``: 13 x 9, X-major,
#: cell centres X -0.6..1.2, Y -0.6..0.6 (robot yaw frame, x forward, y left), 0.15 m.
HEIGHT_XS = np.linspace(-0.6, 1.2, 13)
HEIGHT_YS = np.linspace(-0.6, 0.6, 9)
HEIGHT_CELL = 0.15


# ------------------------------------------------------------------- config


@dataclass
class FootprintConfig:
    length: float = 0.9
    width: float = 0.5
    margin: float = 0.05
    #: Spacing of the points used to sample the footprint rectangle, metres. Finer than
    #: the 0.1 m planning cells; edges and corners are always sampled.
    spacing: float = 0.08

    def body_points(self) -> np.ndarray:
        hl = self.length / 2 + self.margin
        hw = self.width / 2 + self.margin
        xs = np.linspace(-hl, hl, max(2, int(math.ceil(2 * hl / self.spacing)) + 1))
        ys = np.linspace(-hw, hw, max(2, int(math.ceil(2 * hw / self.spacing)) + 1))
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        return np.column_stack([gx.ravel(), gy.ravel()])

    @property
    def half_width(self) -> float:
        return self.width / 2 + self.margin

    @property
    def half_length(self) -> float:
        return self.length / 2 + self.margin


@dataclass
class LocalGridConfig:
    resolution: float = 0.1
    size: float = 8.0
    #: 1 = single frame (default). >1 fuses the last N observations using their poses.
    fuse_frames: int = 1
    #: Observations older than this are dropped from the fusion, seconds.
    max_age: float = 2.0
    #: A base z change larger than this clears the fusion (different storey / relocalised).
    max_z_jump: float = 0.5
    #: Sub-rays per scan bin (bins are 5 degrees wide).
    scan_sub_rays: int = 5
    #: Range used for a bin reporting +inf ("no return up to range_max").
    scan_inf_range: float = 4.0
    #: A scan end point closer than this is marked OBSTACLE (conservative: an end point is
    #: either a body-height return or the end of observed ground; both stop free space).
    scan_obstacle_range: float = 6.0
    #: Ignore scan end points inside the robot's own footprint (self returns).
    scan_self_filter: bool = True
    #: Height step between neighbouring 0.15 m cells that makes both cells an obstacle.
    max_step_flat: float = 0.12
    max_step_stairs: float = 0.22
    #: On stairs segments the risers sit in the scan's body-height band; only height-map
    #: evidence is used there (and cells beyond the height grid remain UNKNOWN).
    ignore_scan_on_stairs: bool = True
    #: Fill an invalid height cell from its neighbours when at least this many of its 8
    #: neighbours are valid and agree within the gait's step limit. Real single frames have
    #: scattered sparse/mixed cells (~5 %); treating each as UNKNOWN blocks every candidate.
    #: Larger gaps (possible holes/drops) stay UNKNOWN. 0 disables.
    fill_hole_min_neighbours: int = 6


@dataclass
class RoutePlannerConfig:
    horizon: float = 3.5
    min_horizon: float = 0.3
    ds: float = 0.1
    n_offsets: int = 9
    #: Corridor half-width used when ``allow_detour`` is false (centreline-only anyway).
    narrow_corridor: float = 0.25
    #: Quintic blend length = max(min_blend, |d_t - d0| / blend_slope), capped to the horizon.
    blend_slope: float = 0.5
    min_blend: float = 1.0
    #: Length over which the offset returns to zero before an unscored gate.
    return_length: float = 1.0
    #: A candidate is admissible if its first ``commit_length`` metres are collision free.
    commit_length_flat: float = 1.0
    commit_length_stairs: float = 0.6
    #: Footprint samples within this arc must lie on height-verified ground (FREE); beyond
    #: it SCAN_CLEAR (no body-height return, ground not verified) is admissible. Default 0:
    #: the single-frame height ROI (1.8 x 1.2 m) cannot contain a footprint that moves
    #: laterally while the body is yawed, so this blocks every detour. Inside the ROI the
    #: height map is authoritative anyway (invalid -> UNKNOWN -> blocked). Consider > 0 only
    #: together with LocalGridConfig.fuse_frames > 1.
    require_ground_within: float = 0.0
    w_obs: float = 1.0
    #: Penalty for the part of the horizon that is not collision free. Large, so that a
    #: candidate running into an obstacle loses to a detour early, while there is still
    #: room to blend sideways.
    w_free: float = 5.0
    w_lat: float = 1.0
    #: Must stay below ``w_lat`` so that, with nothing in the way, d = 0 beats holding the
    #: previous offset and the robot rejoins the taught line.
    w_smooth: float = 0.3
    w_progress: float = 0.5
    #: Clearance (beyond the footprint half-width) at which proximity cost reaches zero.
    inflation: float = 0.3
    #: Extra cost for choosing a detour on the opposite side from the current one.
    w_side_switch: float = 1.0
    #: Keep the previous target while it stays admissible and within this cost of the
    #: best one (prevents pivot-induced flip-flopping between two targets).
    switch_margin: float = 0.3
    #: A* frontier fallback: if no centreline goal is reachable (typically because it lies
    #: in the scan shadow of the obstacle), drive to the reachable KNOWN cell with the most
    #: route progress, if it gains at least this much s.
    astar_min_frontier_progress: float = 0.8
    full_speed_free_length: float = 2.0
    min_speed_fraction: float = 0.3
    astar_enabled: bool = True
    astar_goal_ahead: tuple[float, ...] = (5.0, 4.5, 4.0, 3.5, 3.0)
    astar_window: float = 6.0
    astar_lateral_weight: float = 2.0
    astar_proximity_weight: float = 2.0
    astar_speed_fraction: float = 0.5
    footprint: FootprintConfig = field(default_factory=FootprintConfig)


# -------------------------------------------------------------- local grid


@dataclass
class Observation:
    t: float
    pose: tuple[float, float, float, float]  # x, y, z(base), yaw
    height: np.ndarray | None = None  # (13, 9) relative to base z, robot yaw frame
    mask: np.ndarray | None = None  # (13, 9) bool, True = valid
    scan_ranges: np.ndarray | None = None  # NaN = unknown, inf = no return
    scan_angles: np.ndarray | None = None  # robot frame; default = 72 bin centres
    #: Rasterised layers keyed by (gait, resolution, size); filled lazily by the builder.
    cache: dict = field(default_factory=dict, repr=False, compare=False)


def default_scan_angles(n: int) -> np.ndarray:
    """Bin centres matching ``native_transfer.contracts.conservative_scan``."""
    return -math.pi + (np.arange(n) + 0.5) * (2 * math.pi / n)


class LocalGrid:
    """World-aligned grid centred near the robot. ``state[ix, iy]``."""

    def __init__(self, origin: np.ndarray, resolution: float, shape: tuple[int, int]) -> None:
        self.origin = np.asarray(origin, float)
        self.resolution = float(resolution)
        self.state = np.full(shape, UNKNOWN, np.int8)
        self.height = np.full(shape, np.nan)

    @classmethod
    def centred(cls, center_xy, size: float, resolution: float, fill: int = UNKNOWN) -> LocalGrid:
        n = int(round(size / resolution))
        # Snap to the world lattice so rasterised observation layers can be reused.
        origin = np.floor((np.asarray(center_xy, float)[:2] - n * resolution / 2) / resolution)
        grid = cls(origin * resolution, resolution, (n, n))
        grid.state[:] = fill
        return grid

    @property
    def shape(self) -> tuple[int, int]:
        return self.state.shape

    def cell_centers(self) -> np.ndarray:
        nx, ny = self.shape
        xs = self.origin[0] + (np.arange(nx) + 0.5) * self.resolution
        ys = self.origin[1] + (np.arange(ny) + 0.5) * self.resolution
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        return np.stack([gx, gy], axis=-1)

    def index(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        xy = np.asarray(xy, float).reshape(-1, 2)
        ij = np.floor((xy - self.origin) / self.resolution).astype(int)
        inside = (ij[:, 0] >= 0) & (ij[:, 0] < self.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < self.shape[1])
        return ij[:, 0], ij[:, 1], inside

    def lookup(self, xy: np.ndarray) -> np.ndarray:
        """State at world points; outside the grid is UNKNOWN."""
        ix, iy, inside = self.index(xy)
        out = np.full(len(ix), UNKNOWN, np.int8)
        out[inside] = self.state[ix[inside], iy[inside]]
        return out

    def fill_rect(self, x0, x1, y0, y1, state: int, height: float | None = None) -> None:
        c = self.cell_centers()
        sel = (c[..., 0] >= x0) & (c[..., 0] <= x1) & (c[..., 1] >= y0) & (c[..., 1] <= y1)
        self.state[sel] = state
        if height is not None:
            self.height[sel] = height

    def obstacle_points(self) -> np.ndarray:
        return self.cell_centers()[self.state == OBSTACLE]


def _rot(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def step_obstacles(
    height: np.ndarray, mask: np.ndarray, max_step: float, reference: float | None = None
) -> np.ndarray:
    """Cells on the far side (from ``reference``) of a neighbour jump above ``max_step``.

    For each 8-neighbour pair whose heights differ by more than ``max_step`` the cell whose
    height is farther from the robot's own ground level ``reference`` is marked: the top of
    a box for a rise, the low side for a drop. With no reference both cells are marked.
    """
    h = np.where(mask, height, np.nan)
    flag = np.zeros(h.shape, bool)
    nx, ny = h.shape
    padded = np.full((nx + 2, ny + 2), np.nan)
    padded[1:-1, 1:-1] = h
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == dy == 0:
                continue
            nb = padded[1 + dx : 1 + dx + nx, 1 + dy : 1 + dy + ny]
            with np.errstate(invalid="ignore"):
                jump = np.abs(h - nb) > max_step
                if reference is None:
                    flag |= jump
                else:
                    flag |= jump & (np.abs(h - reference) >= np.abs(nb - reference))
    return flag & mask


def ground_reference(height: np.ndarray, mask: np.ndarray, footprint: FootprintConfig) -> float | None:
    """Median height of valid cells under the robot's footprint (robot yaw frame)."""
    gx, gy = np.meshgrid(HEIGHT_XS, HEIGHT_YS, indexing="ij")
    under = (np.abs(gx) <= footprint.length / 2) & (np.abs(gy) <= footprint.width / 2)
    if height.shape != under.shape:
        return None
    sel = under & mask & np.isfinite(height)
    return float(np.median(height[sel])) if sel.any() else None


def fill_isolated_holes(
    height: np.ndarray, mask: np.ndarray, limit: float, min_neighbours: int = 6
) -> tuple[np.ndarray, np.ndarray]:
    """Fill single invalid cells surrounded by consistent valid ground (one pass).

    Only cells with ``>= min_neighbours`` valid 8-neighbours whose heights span ``<= limit``
    are filled (with the neighbour median). Cells on the ROI border have at most 5
    neighbours, so with the default 6 they are never filled.
    """
    h = np.asarray(height, float)
    m = np.asarray(mask, bool) & np.isfinite(h)
    out_h, out_m = h.copy(), m.copy()
    nx, ny = h.shape
    for i, j in zip(*np.nonzero(~m), strict=True):
        vals = [
            h[a, b]
            for a in range(max(i - 1, 0), min(i + 2, nx))
            for b in range(max(j - 1, 0), min(j + 2, ny))
            if (a, b) != (i, j) and m[a, b]
        ]
        if len(vals) >= min_neighbours and max(vals) - min(vals) <= limit:
            out_h[i, j] = float(np.median(vals))
            out_m[i, j] = True
    return out_h, out_m


class LocalGridBuilder:
    """Builds the planning grid from the last ``fuse_frames`` observations.

    Newer observations overwrite older ones for every cell they KNOW; a newer UNKNOWN
    (e.g. self-occlusion under the body) keeps an older height-verified state. Scan-clear
    evidence only upgrades UNKNOWN cells, because a clear body-height scan says nothing
    about low obstacles or holes the height map may have seen.
    """

    def __init__(
        self,
        config: LocalGridConfig | None = None,
        footprint: FootprintConfig | None = None,
    ) -> None:
        self.config = config or LocalGridConfig()
        self.footprint = footprint or FootprintConfig()
        self.frames: deque[Observation] = deque(maxlen=max(1, int(self.config.fuse_frames)))

    def reset(self) -> None:
        self.frames.clear()

    def add(self, obs: Observation) -> None:
        c = self.config
        if self.frames:
            last = self.frames[-1]
            if abs(obs.pose[2] - last.pose[2]) > c.max_z_jump or obs.t < last.t:
                self.frames.clear()
        self.frames.append(obs)

    def build(self, center_xy, gait: str = "flat", now: float | None = None) -> LocalGrid:
        c = self.config
        grid = LocalGrid.centred(center_xy, c.size, c.resolution)
        if not self.frames:
            return grid
        now = self.frames[-1].t if now is None else now
        for obs in self.frames:
            if now - obs.t > c.max_age:
                continue
            self._apply(grid, obs, gait)
        return grid

    def _apply(self, grid: LocalGrid, obs: Observation, gait: str) -> None:
        layer = self._layer(obs, gait, grid.resolution)
        frame, heights = layer.state, layer.height
        # Overlap of the cached layer with the build grid, in lattice indices.
        go = np.round(grid.origin / grid.resolution).astype(int)
        lo = np.round(layer.origin / grid.resolution).astype(int)
        a0 = np.maximum(go, lo)
        a1 = np.minimum(go + np.array(grid.shape), lo + np.array(layer.shape))
        if np.any(a1 <= a0):
            return
        gs = (slice(a0[0] - go[0], a1[0] - go[0]), slice(a0[1] - go[1], a1[1] - go[1]))
        ls = (slice(a0[0] - lo[0], a1[0] - lo[0]), slice(a0[1] - lo[1], a1[1] - lo[1]))
        f, h = frame[ls], heights[ls]
        g, gh = grid.state[gs], grid.height[gs]
        known = (f == FREE) | (f == OBSTACLE)
        g[known] = f[known]
        g[(f == SCAN_CLEAR) & (g == UNKNOWN)] = SCAN_CLEAR
        fin = np.isfinite(h)
        gh[fin] = h[fin]

    def _layer(self, obs: Observation, gait: str, resolution: float) -> LocalGrid:
        """One observation rasterised on the world lattice around its own pose (cached)."""
        c = self.config
        key = (gait, resolution, c.size)
        cached = obs.cache.get(key)
        if cached is not None:
            return cached
        x, y, z, yaw = obs.pose
        grid = LocalGrid.centred((x, y), c.size, resolution)
        frame = grid.state  # starts UNKNOWN
        heights = grid.height
        rot = _rot(yaw)
        use_scan = obs.scan_ranges is not None and not (
            gait == "stairs" and c.ignore_scan_on_stairs
        )
        obstacle_from_scan = np.zeros(grid.shape, bool)
        if use_scan:
            ranges = np.asarray(obs.scan_ranges, float)
            angles = (
                default_scan_angles(len(ranges))
                if obs.scan_angles is None
                else np.asarray(obs.scan_angles, float)
            )
            keep = ~np.isnan(ranges) & (ranges > 0)  # NaN bin: unknown stays unknown
            ranges, angles = ranges[keep], angles[keep]
            width = 2 * math.pi / max(len(keep), 1)
            subs = np.linspace(-0.5, 0.5, c.scan_sub_rays + 2)[1:-1] * width
            th = (yaw + angles[:, None] + subs[None, :]).ravel()
            r = np.repeat(ranges, len(subs))
            reach = np.where(np.isinf(r), c.scan_inf_range, r)
            step = resolution / 2
            dists = np.arange(step, min(float(reach.max(initial=0.0)), c.size), step)
            if dists.size and th.size:
                ux, uy = np.cos(th), np.sin(th)
                px = x + dists[None, :] * ux[:, None]
                py = y + dists[None, :] * uy[:, None]
                inside_reach = dists[None, :] < reach[:, None]
                pts = np.column_stack([px[inside_reach], py[inside_reach]])
                ix, iy, inside = grid.index(pts)
                frame[ix[inside], iy[inside]] = SCAN_CLEAR
            end_ok = np.isfinite(r) & (r <= c.scan_obstacle_range)
            if end_ok.any():
                ends = np.column_stack([x + r * np.cos(th), y + r * np.sin(th)])[end_ok]
                if c.scan_self_filter:
                    body = (ends - np.array([x, y])) @ rot
                    fp = self.footprint
                    ends = ends[
                        ~((np.abs(body[:, 0]) <= fp.half_length) & (np.abs(body[:, 1]) <= fp.half_width))
                    ]
                ix, iy, inside = grid.index(ends)
                obstacle_from_scan[ix[inside], iy[inside]] = True
        if obs.height is not None:
            h = np.asarray(obs.height, float)
            m = np.isfinite(h) if obs.mask is None else (np.asarray(obs.mask, bool) & np.isfinite(h))
            limit = c.max_step_stairs if gait == "stairs" else c.max_step_flat
            if c.fill_hole_min_neighbours > 0:
                h, m = fill_isolated_holes(h, m, limit, c.fill_hole_min_neighbours)
            steps = step_obstacles(h, m, limit, ground_reference(h, m, self.footprint))
            centers = grid.cell_centers().reshape(-1, 2)
            body = (centers - np.array([x, y])) @ rot  # world -> yaw frame
            hx = np.floor((body[:, 0] - (HEIGHT_XS[0] - HEIGHT_CELL / 2)) / HEIGHT_CELL).astype(int)
            hy = np.floor((body[:, 1] - (HEIGHT_YS[0] - HEIGHT_CELL / 2)) / HEIGHT_CELL).astype(int)
            inside = (hx >= 0) & (hx < h.shape[0]) & (hy >= 0) & (hy < h.shape[1])
            gi, gj = np.unravel_index(np.nonzero(inside)[0], grid.shape)
            cx, cy = hx[inside], hy[inside]
            valid = m[cx, cy]
            # Inside the height ROI the height map is authoritative: an invalid cell is
            # UNKNOWN even if a body-height scan ray passed over it (holes, drops).
            frame[gi, gj] = np.where(valid, np.where(steps[cx, cy], OBSTACLE, FREE), UNKNOWN)
            heights[gi, gj] = np.where(valid, h[cx, cy] + z, np.nan)
        frame[obstacle_from_scan & (frame != FREE)] = OBSTACLE
        obs.cache[key] = grid
        return grid


# ------------------------------------------------------------------ planner


@dataclass(frozen=True)
class Candidate:
    d_target: float
    valid: bool
    free_length: float
    horizon: float
    proximity: float
    cost: float
    reason: str = ""


@dataclass(frozen=True)
class PlanResult:
    status: str  # TRACK | DETOUR | GATE | ASTAR | BLOCKED
    reason: str
    path: np.ndarray  # (M, 2) world xy, starts at the robot
    carrot: np.ndarray | None
    speed_scale: float
    d_target: float = 0.0
    free_length: float = 0.0
    candidates: tuple[Candidate, ...] = ()

    @property
    def blocked(self) -> bool:
        return self.status == "BLOCKED"


def quintic_offset(s_rel: np.ndarray, d0: float, slope0: float, d_t: float, length: float) -> np.ndarray:
    """Quintic d(s): d(0)=d0, d'(0)=slope0, d''(0)=0, d(L)=d_t, d'(L)=d''(L)=0; flat after L.

    Re-planned every tick from the robot's current offset AND lateral slope (Werling et
    al. 2010). Dropping the slope term makes every re-anchored blend start tangent to the
    centreline and the robot never builds up lateral motion.
    """
    L = max(float(length), 1e-6)
    v = slope0 * L
    rhs = np.array([d_t - d0 - v, -v, 0.0])
    a = np.linalg.solve(np.array([[1.0, 1.0, 1.0], [3.0, 4.0, 5.0], [6.0, 12.0, 20.0]]), rhs)
    u = np.clip(np.asarray(s_rel, float) / L, 0.0, 1.0)
    return d0 + v * u + a[0] * u**3 + a[1] * u**4 + a[2] * u**5


def smoothstep5(u: np.ndarray) -> np.ndarray:
    u = np.clip(u, 0.0, 1.0)
    return u * u * u * (10 - 15 * u + 6 * u * u)


def _headings(xy: np.ndarray, fallback: float) -> np.ndarray:
    if len(xy) < 2:
        return np.full(len(xy), fallback)
    diff = np.diff(xy, axis=0)
    yaw = np.arctan2(diff[:, 1], diff[:, 0])
    small = np.linalg.norm(diff, axis=1) < 1e-9
    yaw[small] = fallback
    return np.concatenate([yaw, yaw[-1:]])


def pursuit_headings(xy: np.ndarray, lookahead: float, fallback: float) -> np.ndarray:
    """Heading the body will actually take at each sample under pure pursuit: the bearing
    to the path point ``lookahead`` further along (not the local tangent, which
    under-estimates how far a long body's rear swings while rejoining)."""
    if len(xy) < 2:
        return np.full(len(xy), fallback)
    arc = _arc(xy)
    ahead = np.column_stack(
        [np.interp(arc + lookahead, arc, xy[:, 0]), np.interp(arc + lookahead, arc, xy[:, 1])]
    )
    delta = ahead - xy
    yaw = np.arctan2(delta[:, 1], delta[:, 0])
    tangent = _headings(xy, fallback)
    short = np.linalg.norm(delta, axis=1) < 0.25 * max(lookahead, 1e-6)
    yaw[short] = tangent[short]
    return yaw


def _arc(xy: np.ndarray) -> np.ndarray:
    if len(xy) < 2:
        return np.zeros(len(xy))
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])


def point_along(xy: np.ndarray, distance: float) -> np.ndarray:
    arc = _arc(xy)
    if len(xy) == 1 or distance >= arc[-1]:
        return xy[-1].copy()
    k = int(np.searchsorted(arc, distance, side="right") - 1)
    t = (distance - arc[k]) / max(arc[k + 1] - arc[k], 1e-12)
    return xy[k] + t * (xy[k + 1] - xy[k])


class RoutePlanner:
    def __init__(self, config: RoutePlannerConfig | None = None) -> None:
        self.config = config or RoutePlannerConfig()
        self._body = self.config.footprint.body_points()
        self._last_obstacle_hits = np.zeros(0, bool)
        #: Committed transition (d_target, s_start, d_start, length). The chosen candidate
        #: stays anchored in space where it was first chosen and the robot tracks it with
        #: pure pursuit. Re-anchoring the transition at the robot every tick (a flat-start
        #: quintic) never builds lateral motion; re-anchoring with the measured heading or
        #: the old plan's slope feeds heading error back into the path.
        self._anchor: tuple[float, float, float, float] | None = None

    def reset(self) -> None:
        self._anchor = None

    # ------------------------------------------------------------ collision

    def sweep(
        self,
        grid: LocalGrid,
        xy: np.ndarray,
        yaw: np.ndarray,
        robot_pose: tuple[float, float, float],
        ground_within: float,
    ) -> np.ndarray:
        """Per-sample blocked flag for footprint poses along a path.

        Sample 0 is the robot itself (actual yaw). If the path starts at a heading that
        needs an in-place pivot, the rotating footprint is swept at the robot's position
        and any collision there blocks sample 0.
        """
        xy = np.asarray(xy, float)
        yaw = np.asarray(yaw, float).copy()
        rx, ry, ryaw = robot_pose
        pivot_block = False
        if len(xy) and np.linalg.norm(xy[0] - np.array([rx, ry])) < 1e-6:
            turn = math.atan2(math.sin(yaw[0] - ryaw), math.cos(yaw[0] - ryaw))
            yaw[0] = ryaw
            if abs(turn) > math.radians(5.0):
                steps = np.linspace(0.0, turn, max(2, int(abs(turn) / math.radians(5.0)) + 1))[1:]
                pivot = self._sweep_raw(
                    grid, np.repeat(xy[:1], len(steps), axis=0), ryaw + steps, robot_pose,
                    exempt_current=True,
                )
                pivot_block = bool(pivot.any())
        bad = self._sweep_raw(grid, xy, yaw, robot_pose, exempt_current=True, ground_within=ground_within)
        if pivot_block and len(bad):
            bad[0] = True
            self._last_obstacle_hits[0] = True
        return bad

    def _sweep_raw(self, grid, xy, yaw, robot_pose, *, exempt_current, ground_within=0.0):
        fp = self.config.footprint
        n = len(xy)
        c, s = np.cos(yaw), np.sin(yaw)
        bx, by = self._body[:, 0], self._body[:, 1]
        wx = xy[:, 0:1] + c[:, None] * bx[None, :] - s[:, None] * by[None, :]
        wy = xy[:, 1:2] + s[:, None] * bx[None, :] + c[:, None] * by[None, :]
        pts = np.stack([wx.ravel(), wy.ravel()], axis=1)
        states = grid.lookup(pts).reshape(n, -1)
        # The robot is standing on its current footprint: exempt it (often unobserved).
        rx, ry, ryaw = robot_pose
        rel = pts - np.array([rx, ry])
        cr, sr = math.cos(ryaw), math.sin(ryaw)
        along = rel[:, 0] * cr + rel[:, 1] * sr
        across = -rel[:, 0] * sr + rel[:, 1] * cr
        exempt = ((np.abs(along) <= fp.half_length) & (np.abs(across) <= fp.half_width)).reshape(n, -1)
        if not exempt_current:
            exempt[:] = False
        arc = _arc(xy)
        bad = (states == OBSTACLE) | (states == UNKNOWN)
        near = arc <= ground_within
        bad[near] |= states[near] == SCAN_CLEAR
        bad &= ~exempt
        self._last_obstacle_hits = ((states == OBSTACLE) & ~exempt).any(axis=1)
        return bad.any(axis=1)

    def proximity(self, grid: LocalGrid, xy: np.ndarray, yaw: np.ndarray | None = None) -> float:
        """0..1: how close hazards (OBSTACLE or UNKNOWN) come to the footprint rectangle.

        Measured to the rectangle at each sample pose, not to the path centre, so a 0.9 m
        body whose REAR would swing into a corner while turning back is penalised.
        """
        cfg = self.config
        fp = cfg.footprint
        if len(xy) == 0:
            return 0.0
        xy = np.asarray(xy, float)
        yaw = _headings(xy, 0.0) if yaw is None else np.asarray(yaw, float)
        reach = math.hypot(fp.half_length, fp.half_width) + cfg.inflation
        lo, hi = xy.min(axis=0) - reach, xy.max(axis=0) + reach
        centers = grid.cell_centers()
        sel = (grid.state == OBSTACLE) | (grid.state == UNKNOWN)
        sel &= (centers[..., 0] >= lo[0]) & (centers[..., 0] <= hi[0])
        sel &= (centers[..., 1] >= lo[1]) & (centers[..., 1] <= hi[1])
        hazards = centers[sel]
        if hazards.size == 0:
            return 0.0
        step = max(1, len(xy) // 20)
        pts, yaws = xy[::step], yaw[::step]
        rel = hazards[None, :, :] - pts[:, None, :]
        c, s_ = np.cos(yaws)[:, None], np.sin(yaws)[:, None]
        bx = rel[..., 0] * c + rel[..., 1] * s_
        by = -rel[..., 0] * s_ + rel[..., 1] * c
        # Cell centres stand for 0.1 m cells: subtract half a cell.
        dx = np.maximum(np.abs(bx) - fp.half_length, 0.0)
        dy = np.maximum(np.abs(by) - fp.half_width, 0.0)
        dist = float(np.hypot(dx, dy).min()) - grid.resolution / 2
        return float(np.clip(1.0 - dist / cfg.inflation, 0.0, 1.0))

    # ------------------------------------------------------------ candidates

    def lateral_targets(self, segment: RouteSegment) -> np.ndarray:
        if not segment.allow_detour:
            return np.array([0.0])
        w = segment.corridor_half_width
        n = max(3, self.config.n_offsets | 1)
        return np.linspace(-w, w, n)

    def plan(
        self,
        path: RoutePath,
        projection: Projection,
        pose: tuple[float, float, float],
        grid: LocalGrid,
        *,
        segment: RouteSegment,
        s_gate: float,
        gate_xy: np.ndarray,
        prev_choice: float = 0.0,
        lookahead: float = 0.5,
    ) -> PlanResult:
        cfg = self.config
        x, y, yaw = pose
        robot = np.array([x, y])
        s0, d0 = projection.s, projection.d
        commit = cfg.commit_length_stairs if segment.gait == "stairs" else cfg.commit_length_flat
        to_gate = s_gate - s0
        gate_xy = np.asarray(gate_xy, float)[:2]

        if to_gate < cfg.min_horizon:
            self._anchor = None
            # Final approach (or overshoot): straight at the gate itself.
            line = np.linspace(robot, gate_xy, max(2, int(np.linalg.norm(gate_xy - robot) / cfg.ds) + 2))
            heading = math.atan2(*(gate_xy - robot)[::-1]) if np.linalg.norm(gate_xy - robot) > 1e-6 else yaw
            blocked = self.sweep(grid, line, np.full(len(line), heading), pose, cfg.require_ground_within)
            if blocked.any():
                return PlanResult("BLOCKED", "gate_approach_blocked", line, None, 0.0)
            return PlanResult("GATE", "", line, gate_xy.copy(), 1.0, 0.0, float(_arc(line)[-1]))

        gate_limited = to_gate <= cfg.horizon
        horizon = min(cfg.horizon, to_gate)
        commit = min(commit, horizon)
        n = max(2, int(math.ceil(horizon / cfg.ds)) + 1)
        s_rel = np.linspace(0.0, horizon, n)
        s_abs = s0 + s_rel
        profiles: list[np.ndarray] = []
        obstacle_free: list[float] = []
        anchors: list[tuple[float, float, float, float]] = []
        anchor = self._anchor
        if anchor is not None and not anchor[1] - 1e-6 <= s0 <= anchor[1] + anchor[3] + cfg.horizon:
            anchor = None
        candidates: list[Candidate] = []
        paths: list[np.ndarray] = []
        for d_t in self.lateral_targets(segment):
            if anchor is not None and abs(anchor[0] - d_t) < 1e-9:
                _, s_start, d_start, blend = anchor
                d = quintic_offset(s_abs - s_start, d_start, 0.0, d_t, blend)
                this_anchor = anchor
            else:
                blend = max(cfg.min_blend, abs(d_t - d0) / cfg.blend_slope)
                blend = min(blend, horizon if not gate_limited else max(horizon / 2, 1e-3))
                d = quintic_offset(s_rel, d0, 0.0, d_t, blend)
                this_anchor = (float(d_t), s0, d0, blend)
            if gate_limited:
                ret = min(cfg.return_length, horizon / 2)
                d = d * (1.0 - smoothstep5((s_rel - (horizon - ret)) / max(ret, 1e-3)))
            profiles.append(d)
            anchors.append(this_anchor)
            xy = path.offset_points(s_abs, d)
            xy[0] = robot
            if gate_limited:
                xy[-1] = gate_xy
            heads = pursuit_headings(xy, lookahead, yaw)
            blocked = self.sweep(grid, xy, heads, pose, cfg.require_ground_within)
            hits = self._last_obstacle_hits
            arc = _arc(xy)
            if blocked.any():
                k = int(np.argmax(blocked))
                free = float(arc[k - 1]) if k > 0 else 0.0
            else:
                free = float(arc[-1])
            if hits.any():
                k = int(np.argmax(hits))
                clear_of_obstacles = float(arc[k - 1]) if k > 0 else 0.0
            else:
                clear_of_obstacles = math.inf
            obstacle_free.append(clear_of_obstacles)
            total = float(arc[-1])
            valid = free >= commit - 1e-9 and total > 0
            inside = arc <= free + 1e-9
            prox = self.proximity(grid, xy[inside][1:], heads[inside][1:]) if valid else 1.0
            frac = free / max(total, 1e-6)
            cost = (
                cfg.w_obs * prox
                + cfg.w_free * (1.0 - frac)
                + cfg.w_lat * abs(d_t)
                + cfg.w_smooth * abs(d_t - prev_choice)
                - cfg.w_progress * frac
                + (cfg.w_side_switch if d_t * prev_choice < -1e-9 else 0.0)
            )
            candidates.append(
                Candidate(float(d_t), bool(valid), free, total, prox, float(cost),
                          "" if valid else f"blocked at {free:.2f}m")
            )
            paths.append(xy)

        valid_idx = [i for i, c in enumerate(candidates) if c.valid]
        if valid_idx:
            best = min(
                valid_idx,
                key=lambda i: (
                    round(candidates[i].cost, 9),
                    abs(candidates[i].d_target),
                    abs(candidates[i].d_target - prev_choice),
                    -candidates[i].d_target,
                ),
            )
            if self._anchor is not None:
                held = [i for i in valid_idx if abs(candidates[i].d_target - self._anchor[0]) < 1e-9]
                if held and candidates[held[0]].cost <= candidates[best].cost + cfg.switch_margin:
                    best = held[0]
            chosen = candidates[best]
            xy = paths[best]
            self._anchor = anchors[best]
            arc = _arc(xy)
            usable = xy[arc <= chosen.free_length + 1e-9]
            carrot = point_along(usable, lookahead)
            # Speed tapers with distance to the first real OBSTACLE on the path. Unknown
            # space ahead only has to stay beyond the commit length (checked above): the
            # robot keeps seeing further as it moves and stops if that ever fails.
            span = max(cfg.full_speed_free_length - commit, 1e-6)
            openness = float(np.clip((obstacle_free[best] - commit) / span, 0.0, 1.0))
            scale = cfg.min_speed_fraction + (1 - cfg.min_speed_fraction) * openness
            scale *= 1.0 - 0.5 * chosen.proximity
            status = "TRACK" if abs(chosen.d_target) < 1e-9 and abs(d0) < 0.05 else "DETOUR"
            return PlanResult(
                status, "", usable, carrot, float(scale), chosen.d_target,
                chosen.free_length, tuple(candidates),
            )

        self._anchor = None
        if not segment.allow_detour:
            return PlanResult(
                "BLOCKED", "centerline_blocked_detour_forbidden", paths[0][:1], None, 0.0,
                0.0, 0.0, tuple(candidates),
            )
        if not cfg.astar_enabled:
            return PlanResult("BLOCKED", "all_candidates_blocked", paths[0][:1], None, 0.0,
                              0.0, 0.0, tuple(candidates))
        result = self.astar(path, projection, pose, grid, segment=segment, s_gate=s_gate,
                            gate_xy=gate_xy, lookahead=lookahead, commit=commit)
        return PlanResult(result.status, result.reason, result.path, result.carrot,
                          result.speed_scale, result.d_target, result.free_length,
                          tuple(candidates))

    # ------------------------------------------------------------------ A*

    def astar(
        self,
        path: RoutePath,
        projection: Projection,
        pose: tuple[float, float, float],
        grid: LocalGrid,
        *,
        segment: RouteSegment,
        s_gate: float,
        gate_xy: np.ndarray,
        lookahead: float,
        commit: float,
    ) -> PlanResult:
        cfg = self.config
        fp = cfg.footprint
        x, y, yaw = pose
        robot = np.array([x, y])
        s0 = projection.s
        width = segment.corridor_half_width
        res = grid.resolution

        # Goal candidates: centreline 5..3 m ahead (never past the unscored gate).
        goals = []
        for ahead in cfg.astar_goal_ahead:
            s_goal = s0 + ahead
            if s_goal >= s_gate:
                goals.append((s_gate, np.asarray(gate_xy, float)[:2]))
            else:
                goals.append((s_goal, path.point_at(s_goal)[:2]))
        # Search window.
        mid = (robot + goals[0][1]) / 2
        half = cfg.astar_window / 2
        ix0, iy0, _ = grid.index(np.array([mid - half]))
        ix1, iy1, _ = grid.index(np.array([mid + half]))
        ix0, iy0 = max(0, ix0[0]), max(0, iy0[0])
        ix1, iy1 = min(grid.shape[0] - 1, ix1[0]), min(grid.shape[1] - 1, iy1[0])
        if ix1 <= ix0 or iy1 <= iy0:
            return PlanResult("BLOCKED", "astar_window_outside_grid", robot[None, :], None, 0.0)
        state = grid.state[ix0 : ix1 + 1, iy0 : iy1 + 1].copy()
        centers = grid.cell_centers()[ix0 : ix1 + 1, iy0 : iy1 + 1]
        # Treat the cells under the robot as ground (they are, the robot stands on them).
        rel = centers - robot
        along = rel[..., 0] * math.cos(yaw) + rel[..., 1] * math.sin(yaw)
        across = -rel[..., 0] * math.sin(yaw) + rel[..., 1] * math.cos(yaw)
        under = (np.abs(along) <= fp.half_length) & (np.abs(across) <= fp.half_width)
        state[under & (state == UNKNOWN)] = FREE
        hazard = (state == OBSTACLE) | (state == UNKNOWN)
        radius = fp.half_width
        r_cells = int(math.ceil(radius / res))
        blocked = np.zeros_like(hazard)
        nx, ny = hazard.shape
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                if (dx * dx + dy * dy) * res * res > (radius + res / 2) ** 2:
                    continue
                shifted = np.zeros_like(hazard)
                xs = slice(max(0, dx), nx + min(0, dx))
                xd = slice(max(0, -dx), nx + min(0, -dx))
                ys = slice(max(0, dy), ny + min(0, dy))
                yd = slice(max(0, -dy), ny + min(0, -dy))
                shifted[xd, yd] = hazard[xs, ys]
                blocked |= shifted
        # Corridor limit and lateral cost.
        flat_centers = centers.reshape(-1, 2)
        s_lo = max(0.0, s0 - 1.5)
        s_hi = min(path.length, max(s0 + max(cfg.astar_goal_ahead) + 0.5, s_gate))
        lateral = path.distance_to_path(flat_centers, s_lo, s_hi).reshape(nx, ny)
        blocked |= lateral > width
        # Proximity to obstacles for cost.
        obstacles = centers[state == OBSTACLE]
        if obstacles.size:
            dist = np.linalg.norm(flat_centers[:, None, :] - obstacles[None, :, :], axis=2).min(axis=1)
            prox = np.clip(1.0 - (dist - radius) / cfg.inflation, 0.0, 1.0).reshape(nx, ny)
        else:
            prox = np.zeros((nx, ny))
        cell_cost = 1.0 + cfg.astar_lateral_weight * lateral / max(width, 1e-6) + cfg.astar_proximity_weight * prox

        def to_local(p):
            ix, iy, inside = grid.index(np.asarray(p, float)[None, :])
            i, j = ix[0] - ix0, iy[0] - iy0
            return (i, j) if inside[0] and 0 <= i < nx and 0 <= j < ny else None

        start = to_local(robot)
        if start is None:
            return PlanResult("BLOCKED", "astar_start_outside_window", robot[None, :], None, 0.0)
        blocked[start] = False
        goal = None
        goal_s = None
        for s_goal, gxy in goals:
            cell = to_local(gxy)
            if cell is not None and not blocked[cell]:
                goal, goal_s = cell, s_goal
                break

        parent, g, found = _astar_search(start, goal, blocked, cell_cost, res)
        kind = "centreline"
        if not found:
            # Frontier fallback: best reachable KNOWN cell by route progress. The goal on
            # the centreline is usually in the obstacle's scan shadow (UNKNOWN), which must
            # not be treated as free; driving to the edge of known space and re-planning is.
            reached = np.array(list(g.keys()))
            if reached.size == 0:
                return PlanResult("BLOCKED", "astar_no_path", robot[None, :], None, 0.0)
            pts = centers[reached[:, 0], reached[:, 1]]
            s_cells, lat = path.project_many(pts, s_lo, s_hi)
            s_cells = np.minimum(s_cells, s_gate)
            score = s_cells - 0.5 * lat
            best = int(np.argmax(score))
            if s_cells[best] - s0 < cfg.astar_min_frontier_progress:
                reason = "astar_no_path" if goal is not None else "astar_no_free_goal"
                return PlanResult("BLOCKED", reason, robot[None, :], None, 0.0)
            goal = (int(reached[best, 0]), int(reached[best, 1]))
            goal_s = float(s_cells[best])
            kind = "frontier"
        cells = [goal]
        while cells[-1] != start:
            cells.append(parent[cells[-1]])
        cells.reverse()
        xy = np.array([centers[i, j] for i, j in cells])
        xy[0] = robot
        # Light smoothing (keep endpoints).
        if len(xy) > 4:
            sm = xy.copy()
            sm[1:-1] = (xy[:-2] + xy[1:-1] + xy[2:]) / 3
            xy = sm
        # Validate with the real rectangular footprint over the committed part.
        heads = pursuit_headings(xy, lookahead, yaw)
        arc = _arc(xy)
        bad = self.sweep(grid, xy, heads, pose, cfg.require_ground_within)
        if bad.any():
            k = int(np.argmax(bad))
            free = float(arc[k - 1]) if k > 0 else 0.0
        else:
            free = float(arc[-1])
        if free < min(commit, arc[-1]) - 1e-9:
            return PlanResult("BLOCKED", "astar_path_fails_footprint_check", xy, None, 0.0)
        usable = xy[arc <= free + 1e-9]
        end_d = path.project(xy[-1], s_hint=goal_s, window=(0.5, 0.5)).d
        return PlanResult(
            "ASTAR", f"grid A* ({kind}) to s={goal_s:.1f}", usable, point_along(usable, lookahead),
            cfg.astar_speed_fraction, float(end_d), free,
        )


def _astar_search(start, goal, blocked, cell_cost, res):
    """8-connected A* (Dijkstra when ``goal`` is None). Returns (parent, g, found)."""
    nx, ny = blocked.shape
    moves = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    g = {start: 0.0}
    parent: dict = {}
    heap = [(0.0, start)]
    closed = set()
    while heap:
        _, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        if goal is not None and cur == goal:
            return parent, g, True
        closed.add(cur)
        ci, cj = cur
        for di, dj in moves:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < nx and 0 <= nj < ny) or blocked[ni, nj]:
                continue
            step = math.hypot(di, dj) * res
            cand = g[cur] + step * 0.5 * (cell_cost[ci, cj] + cell_cost[ni, nj])
            if cand < g.get((ni, nj), math.inf):
                g[(ni, nj)] = cand
                parent[(ni, nj)] = cur
                h = 0.0 if goal is None else math.hypot(ni - goal[0], nj - goal[1]) * res
                heapq.heappush(heap, (cand + h, (ni, nj)))
    return parent, g, False
