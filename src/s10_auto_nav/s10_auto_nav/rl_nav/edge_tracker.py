"""Step edges from the 13x9 height grid, and where the robot is relative to them.

The grid is the ``real_transfer.geometry.height_grid`` contract the follower already consumes: robot
yaw frame (gravity aligned, x forward, y left), 0.15 m cells centred at x = -0.6 .. 1.2 (13 rows)
and y = -0.6 .. 0.6 (9 columns), heights relative to the base, unknown cells -1 / masked.

``detect_edge`` finds, in every column, the first rise or drop ahead of the robot, then fits a
straight line through those points with RANSAC (the usual way stair trackers and plane-based
stairway mappers recover a riser from a height map). A riser inside one cell makes that cell fail
the contract's spread check, so an edge is also accepted across one unknown cell between two valid
ones.

``EdgeTracker`` keeps the edge in the map frame across frames, so it survives the robot turning and
the edge leaving the grid under the body, and gates each new fix against the map prior of the
manoeuvre: a fix is used only if it lies within ``gate_distance`` of the prior line and
``gate_angle`` of its normal. Without a fix the prior stands in, flagged, and the caller decides
what that is allowed to do.

``side_margins`` reports how far the body's sides are from a drop, for centring on a narrow
structure.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import numpy as np

CELL = 0.15
X0, Y0 = -0.6, -0.6
NX, NY = 13, 9
XS = X0 + CELL * np.arange(NX)
YS = Y0 + CELL * np.arange(NY)


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class EdgeFix:
    """One detection, in the robot's yaw frame."""

    distance: float  # m from the base origin to the edge line, along its normal (positive ahead)
    angle: float  # rad, yaw of the edge normal in the robot frame (0 = square on; turn by this)
    height: float  # m, signed: + rise, - drop
    inliers: int
    rms: float


def _grid(values, valid):
    g = np.asarray(values, float).reshape(NX, NY)
    v = (
        np.asarray(valid, bool).reshape(NX, NY)
        if valid is not None
        else np.isfinite(g) & (g > -1.0)
    )
    return g, v & np.isfinite(g)


def edge_points(values, valid, min_rise=0.05, x_min=0.15, x_max=1.25, want="any"):
    """First edge ahead in each column: list of (x, y, dz)."""
    g, v = _grid(values, valid)
    pts = []
    i0 = int(np.searchsorted(XS, x_min - 1e-9))
    for j in range(NY):
        for i in range(max(i0 - 1, 0), NX - 1):
            if XS[i] > x_max:
                break
            if not v[i, j]:
                continue
            k = i + 1 if v[i + 1, j] else (i + 2 if i + 2 < NX and v[i + 2, j] else None)
            if k is None:
                continue
            dz = g[k, j] - g[i, j]
            if abs(dz) < min_rise or (want == "up" and dz < 0) or (want == "down" and dz > 0):
                continue
            x = 0.5 * (XS[i] + XS[k])
            if x < x_min:
                continue
            pts.append((x, YS[j], dz))
            break
    return pts


def fit_line(pts, tol=0.08, min_inliers=4):
    """RANSAC fit of x = a + b y through (x, y) points; returns (a, b, inlier mask) or None."""
    if len(pts) < min_inliers:
        return None
    p = np.asarray([(x, y) for x, y, _ in pts], float)
    best = None
    for i, k in itertools.combinations(range(len(p)), 2):
        if abs(p[k, 1] - p[i, 1]) < 1e-6:
            continue
        b = (p[k, 0] - p[i, 0]) / (p[k, 1] - p[i, 1])
        if abs(b) > 1.2:  # more than ~50 deg off square: not a crossing
            continue
        a = p[i, 0] - b * p[i, 1]
        res = np.abs(p[:, 0] - (a + b * p[:, 1])) / math.sqrt(1 + b * b)
        inl = res < tol
        score = (int(inl.sum()), -float(res[inl].sum()))
        if best is None or score > best[0]:
            best = (score, inl)
    if best is None or best[0][0] < min_inliers:
        return None
    inl = best[1]
    A = np.column_stack([np.ones(int(inl.sum())), p[inl, 1]])
    (a, b), *_ = np.linalg.lstsq(A, p[inl, 0], rcond=None)
    return float(a), float(b), inl


def detect_edge(
    values, valid, min_rise=0.05, x_min=0.15, x_max=1.25, want="any", tol=0.08, min_inliers=4
) -> EdgeFix | None:
    pts = edge_points(values, valid, min_rise, x_min, x_max, want)
    fit = fit_line(pts, tol, min_inliers)
    if fit is None:
        return None
    a, b, inl = fit
    norm = math.sqrt(1 + b * b)
    dz = np.asarray([d for _, _, d in pts])[inl]
    p = np.asarray([(x, y) for x, y, _ in pts])[inl]
    rms = float(np.sqrt(np.mean(((p[:, 0] - (a + b * p[:, 1])) / norm) ** 2)))
    return EdgeFix(
        distance=a / norm,
        angle=math.atan2(-b, 1.0),
        height=float(np.median(dz)),
        inliers=int(inl.sum()),
        rms=rms,
    )


def side_margins(values, valid, drop=0.3, x_range=(-0.3, 0.9), body_half_width=0.25):
    """Distance (m) from each side of the body to the nearest drop or unknown gap beside it, or None
    when the grid shows ground all the way to its edge on that side."""
    g, v = _grid(values, valid)
    rows = [i for i in range(NX) if x_range[0] <= XS[i] <= x_range[1]]
    centre = [int(np.argmin(np.abs(YS)))]
    ref_cells = g[rows][:, centre][v[rows][:, centre]]
    if ref_cells.size == 0:
        return None, None
    ref = float(np.median(ref_cells))
    out = []
    for side in (+1, -1):  # left (+y), right (-y)
        cols = [j for j in range(NY) if side * YS[j] > 0]
        cols.sort(key=lambda j: abs(YS[j]))
        margin = None
        for j in cols:
            col_ok = v[rows, j]
            low = (~col_ok) | (g[rows, j] < ref - drop)
            if low.mean() >= 0.5:
                margin = max(0.0, abs(YS[j]) - CELL / 2 - body_half_width)
                break
        out.append(margin)
    return out[0], out[1]


@dataclass
class EdgeEstimate:
    point: np.ndarray  # a point on the edge line, map frame
    normal_yaw: float  # map-frame yaw of the edge normal, pointing the way the route crosses it
    height: float
    measured: bool  # False: this is the map prior, no fix accepted yet
    age: float  # s since the last accepted fix (inf for the prior)
    fixes: int


class EdgeTracker:
    """Map-frame edge estimate for one manoeuvre: prior, gated fixes, exponential smoothing."""

    def __init__(
        self,
        prior_point,
        prior_normal_yaw,
        prior_height,
        gate_distance=0.5,
        gate_angle=math.radians(20.0),
        smoothing=0.4,
        want="up",
    ):
        self.prior = EdgeEstimate(
            np.asarray(prior_point, float)[:2].copy(),
            float(prior_normal_yaw),
            float(prior_height),
            False,
            math.inf,
            0,
        )
        self.gate_distance, self.gate_angle, self.smoothing = gate_distance, gate_angle, smoothing
        self.want = want
        self.est = self.prior
        self._last_t = None
        self.rejected = 0

    def _line_offset(self, point, normal_yaw, ref: EdgeEstimate) -> float:
        n = np.array([math.cos(ref.normal_yaw), math.sin(ref.normal_yaw)])
        return float(np.dot(point - ref.point, n))

    def update(self, t, pose_xy, yaw, values, valid) -> EdgeEstimate:
        if self._last_t is not None and self.est.measured:
            self.est.age = t - self._last_fix_t
        self._last_t = t
        fix = detect_edge(values, valid, want=self.want)
        if fix is None:
            return self.est
        normal = wrap(yaw + fix.angle)
        n = np.array([math.cos(normal), math.sin(normal)])
        point = np.asarray(pose_xy, float)[:2] + fix.distance * n
        # Gate against the prior (not against the running estimate, so one bad fix cannot drag the
        # gate along with it).
        if (
            abs(wrap(normal - self.prior.normal_yaw)) > self.gate_angle
            or abs(self._line_offset(point, normal, self.prior)) > self.gate_distance
            or (self.want == "up" and fix.height < 0)
            or (self.want == "down" and fix.height > 0)
        ):
            self.rejected += 1
            return self.est
        if not self.est.measured:
            self.est = EdgeEstimate(point, normal, fix.height, True, 0.0, 1)
        else:
            a = self.smoothing
            # Move the stored point to the foot of the new line nearest to it before blending, so
            # the blend is across the line and not along it.
            e = self.est
            en = np.array([math.cos(normal), math.sin(normal)])
            foot = e.point - np.dot(e.point - point, en) * en
            yaw_mix = math.atan2(
                (1 - a) * math.sin(e.normal_yaw) + a * math.sin(normal),
                (1 - a) * math.cos(e.normal_yaw) + a * math.cos(normal),
            )
            self.est = EdgeEstimate(
                (1 - a) * e.point + a * foot,
                yaw_mix,
                (1 - a) * e.height + a * fix.height,
                True,
                0.0,
                e.fixes + 1,
            )
        self._last_fix_t = t
        return self.est

    @staticmethod
    def coordinates(est: EdgeEstimate, pose_xy, yaw):
        """(distance to the edge along its normal, lateral offset along it, heading error) of the
        base."""
        n = np.array([math.cos(est.normal_yaw), math.sin(est.normal_yaw)])
        tvec = np.array([-n[1], n[0]])
        d = est.point - np.asarray(pose_xy, float)[:2]
        return float(np.dot(d, n)), float(-np.dot(d, tvec)), wrap(yaw - est.normal_yaw)
