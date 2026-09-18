"""Route v2: taught centreline paths with per-segment gait, speed and corridor.

``route_v2.json`` (schema ``s10_route_v2``) replaces "a list of gates joined by straight
lines" with "a list of gates joined by the path that was actually driven while mapping".
This module is pure Python + numpy (no ROS) and provides:

* :class:`RouteV2` -- load/validate the file, convert to the legacy ordered ``Course`` /
  course YAML / native point list so the old follower and native runtime still work.
* :class:`RoutePath` -- the concatenated centreline with horizontal arc length ``s`` and a
  windowed, z-gated projection ``(s, d, segment, tangent_yaw)``.
* :class:`RouteTracker` -- stateful projection that only searches a window of ``s`` around
  the previous answer, so a parallel leg of a switchback or another storey that happens to
  be close in XY is never snapped to.
* :class:`TerrainCrossCheck` -- compares live pitch / height map against the segment's
  declared gait. It only warns or holds; it never selects a gait.

Conventions: map frame, metres, radians. Route ``z`` is GROUND height; a robot base pose
is ``ground + body_z_offset`` and callers subtract the offset before projecting.
``d`` is positive to the LEFT of the direction of travel.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

SCHEMA = "s10_route_v2"
GAIT_CODES = {"flat": 0x3002, "stairs": 0x3003}
CONFIDENCE = ("high", "medium", "low")


class RouteValidationError(ValueError):
    """The route file violates the route_v2 contract."""


@dataclass(frozen=True)
class RouteWaypoint:
    id: str
    position: np.ndarray  # (3,) map frame, ground z
    yaw: float | None
    radius_xy: float
    tol_z: float
    terrain: str = ""
    confidence: str = "high"

    @property
    def xy(self) -> np.ndarray:
        return self.position[:2]


@dataclass(frozen=True)
class RouteSegment:
    id: str
    from_id: str
    to_id: str
    gait: str
    speed_limit: float
    allow_detour: bool
    corridor_half_width: float
    centerline: np.ndarray  # (N, 3)

    @property
    def gait_code(self) -> int:
        return GAIT_CODES[self.gait]


def _finite_positive(value, name) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RouteValidationError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise RouteValidationError(f"{name} must be finite and positive")
    return result


def _xyz(value, name) -> np.ndarray:
    try:
        result = np.asarray(value, float)
    except (TypeError, ValueError) as exc:
        raise RouteValidationError(f"{name} must be [x, y, z]") from exc
    if result.shape != (3,) or not np.isfinite(result).all():
        raise RouteValidationError(f"{name} must be a finite [x, y, z]")
    return result


@dataclass
class RouteV2:
    map_id: str
    frame: str
    z_reference: str
    status: str
    waypoints: tuple[RouteWaypoint, ...]
    segments: tuple[RouteSegment, ...]
    raw: dict = field(default_factory=dict, repr=False)

    # ------------------------------------------------------------------ loading

    @classmethod
    def load(cls, path: str | Path, **kwargs) -> RouteV2:
        return cls.from_dict(json.loads(Path(path).read_text()), **kwargs)

    @classmethod
    def from_dict(
        cls,
        data: dict,
        *,
        max_spacing: float = 0.30,
        endpoint_tolerance: float = 0.5,
        endpoint_z_tolerance: float = 0.5,
        expected_map_id: str | None = None,
    ) -> RouteV2:
        """Validate and build; unknown extra fields are ignored.

        ``max_spacing`` (horizontal) is the contract's 0.25 m plus 5 cm slack.
        """
        if not isinstance(data, dict) or data.get("schema") != SCHEMA:
            raise RouteValidationError(f"schema must be {SCHEMA!r}")
        if data.get("frame") != "map":
            raise RouteValidationError("frame must be 'map'")
        if data.get("z_reference") != "ground":
            raise RouteValidationError("z_reference must be 'ground'")
        map_id = data.get("map_id")
        if not isinstance(map_id, str) or not map_id:
            raise RouteValidationError("map_id required")
        if expected_map_id is not None and map_id != expected_map_id:
            raise RouteValidationError(f"map_id {map_id!r} != expected {expected_map_id!r}")

        raw_wps = data.get("waypoints")
        if not isinstance(raw_wps, list) or len(raw_wps) < 2:
            raise RouteValidationError("at least two waypoints required")
        waypoints = []
        for i, w in enumerate(raw_wps):
            if not isinstance(w, dict):
                raise RouteValidationError(f"waypoint {i} must be an object")
            wid = w.get("id")
            if not isinstance(wid, str) or not wid:
                raise RouteValidationError(f"waypoint {i} needs a string id")
            yaw = w.get("yaw")
            if yaw is not None:
                if isinstance(yaw, bool) or not isinstance(yaw, (int, float)):
                    raise RouteValidationError(f"{wid}.yaw must be a number or null")
                if not math.isfinite(yaw):
                    raise RouteValidationError(f"{wid}.yaw must be finite")
                yaw = float(yaw)
            confidence = w.get("confidence", "high")
            if confidence not in CONFIDENCE:
                raise RouteValidationError(f"{wid}.confidence must be one of {CONFIDENCE}")
            waypoints.append(
                RouteWaypoint(
                    id=wid,
                    position=_xyz(w.get("position"), f"{wid}.position"),
                    yaw=yaw,
                    radius_xy=_finite_positive(w.get("radius_xy"), f"{wid}.radius_xy"),
                    tol_z=_finite_positive(w.get("tol_z"), f"{wid}.tol_z"),
                    terrain=str(w.get("terrain", "")),
                    confidence=confidence,
                )
            )
        ids = [w.id for w in waypoints]
        if len(set(ids)) != len(ids):
            raise RouteValidationError("waypoint ids must be unique")

        raw_segs = data.get("segments")
        if not isinstance(raw_segs, list) or len(raw_segs) != len(waypoints) - 1:
            raise RouteValidationError("need exactly one segment between consecutive waypoints")
        segments = []
        for i, sg in enumerate(raw_segs):
            if not isinstance(sg, dict):
                raise RouteValidationError(f"segment {i} must be an object")
            sid = str(sg.get("id", f"segment{i}"))
            if sg.get("from") != ids[i] or sg.get("to") != ids[i + 1]:
                raise RouteValidationError(
                    f"segment {sid} must go {ids[i]} -> {ids[i + 1]} (ordered chain)"
                )
            gait = sg.get("gait")
            if gait not in GAIT_CODES:
                raise RouteValidationError(f"{sid}.gait must be one of {sorted(GAIT_CODES)}")
            allow = sg.get("allow_detour")
            if not isinstance(allow, bool):
                raise RouteValidationError(f"{sid}.allow_detour must be a boolean")
            try:
                line = np.asarray(sg.get("centerline"), float)
            except (TypeError, ValueError) as exc:
                raise RouteValidationError(f"{sid}.centerline must be [[x,y,z],...]") from exc
            if line.ndim != 2 or line.shape[1] != 3 or len(line) < 2:
                raise RouteValidationError(f"{sid}.centerline needs >=2 [x,y,z] points")
            if not np.isfinite(line).all():
                raise RouteValidationError(f"{sid}.centerline must be finite")
            # Horizontal spacing: arc length s is horizontal, and on stairs the vertical
            # component legitimately makes 3-D spacing larger than the contract's 0.25 m.
            spacing = np.linalg.norm(np.diff(line[:, :2], axis=0), axis=1)
            if spacing.max() > max_spacing + 1e-9:
                raise RouteValidationError(
                    f"{sid}.centerline XY spacing {spacing.max():.3f} m exceeds {max_spacing} m"
                )
            for end, wp in ((line[0], waypoints[i]), (line[-1], waypoints[i + 1])):
                if (
                    np.linalg.norm(end[:2] - wp.xy) > endpoint_tolerance
                    or abs(end[2] - wp.position[2]) > endpoint_z_tolerance
                ):
                    raise RouteValidationError(f"{sid}.centerline does not end near {wp.id}")
            segments.append(
                RouteSegment(
                    id=sid,
                    from_id=ids[i],
                    to_id=ids[i + 1],
                    gait=gait,
                    speed_limit=_finite_positive(sg.get("speed_limit"), f"{sid}.speed_limit"),
                    allow_detour=allow,
                    corridor_half_width=_finite_positive(
                        sg.get("corridor_half_width"), f"{sid}.corridor_half_width"
                    ),
                    centerline=line,
                )
            )
        return cls(
            map_id=map_id,
            frame="map",
            z_reference="ground",
            status=str(data.get("status", "")),
            waypoints=tuple(waypoints),
            segments=tuple(segments),
            raw=data,
        )

    # ------------------------------------------------------------ conversions

    def incoming_segment(self, waypoint_index: int) -> RouteSegment:
        """Segment driven INTO ``waypoint_index``; WP0 (start) uses the first segment."""
        if not 0 <= waypoint_index < len(self.waypoints):
            raise IndexError(waypoint_index)
        return self.segments[max(0, waypoint_index - 1)]

    def gait_kinds(self) -> list[str]:
        """``kind`` per target, exactly the NativeGaitRouter/start_b.draft semantics."""
        return [self.incoming_segment(i).gait for i in range(len(self.waypoints))]

    def native_points(self) -> list[dict]:
        """Route as the native ``waypoints`` list (ground z), legacy fields first."""
        points = []
        for i, wp in enumerate(self.waypoints):
            seg = self.incoming_segment(i)
            points.append(
                {
                    "index": i,
                    "id": wp.id,
                    "position": [float(v) for v in wp.position],
                    "kind": seg.gait,
                    "radius_xy": wp.radius_xy,
                    "tol_z": wp.tol_z,
                    "segment_id": seg.id,
                    "speed_limit": seg.speed_limit,
                    "allow_detour": seg.allow_detour,
                    "corridor_half_width": seg.corridor_half_width,
                }
            )
        return points

    def course_yaml_dict(self, body_z_offset: float = 0.0) -> dict:
        """Ordered gates for ``Course.from_yaml``; z converted from ground to base."""
        return {
            "waypoints": [
                {
                    "index": i,
                    "id": wp.id,
                    "position": [
                        float(wp.position[0]),
                        float(wp.position[1]),
                        float(wp.position[2] + body_z_offset),
                    ],
                    "radius_xy": wp.radius_xy,
                    "tol_z": wp.tol_z,
                }
                for i, wp in enumerate(self.waypoints)
            ]
        }

    def to_course(self, body_z_offset: float = 0.0, **kwargs):
        """Legacy ``Course`` with per-waypoint radius/height tolerance."""
        from s10_auto_nav.waypoints import Course, Waypoint

        waypoints = [
            Waypoint(
                index=i,
                position=np.array([*wp.position[:2], wp.position[2] + body_z_offset]),
                radius=wp.radius_xy,
                height_tolerance=wp.tol_z,
            )
            for i, wp in enumerate(self.waypoints)
        ]
        kwargs.setdefault("score_radius", min(wp.radius_xy for wp in self.waypoints))
        kwargs.setdefault("advance_radius", kwargs["score_radius"])
        return Course(waypoints, **kwargs)

    def path(self) -> RoutePath:
        return RoutePath(self)


# ---------------------------------------------------------------------- path


@dataclass(frozen=True)
class Projection:
    s: float
    d: float  # signed lateral offset, + left of travel
    segment_index: int
    tangent_yaw: float
    distance: float  # horizontal distance to the path
    point: np.ndarray  # (2,) foot point
    z_path: float
    dz: float  # |z - z_path| (nan if z not given)
    z_mismatch: bool = False  # no candidate within the z gate; best XY used
    relocked: bool = False


class RoutePath:
    """Concatenated centreline with horizontal arc length."""

    def __init__(self, route: RouteV2) -> None:
        self.route = route
        pts: list[np.ndarray] = []
        seg_of_edge: list[int] = []
        seg_start_vertex: list[int] = []
        for k, seg in enumerate(route.segments):
            line = seg.centerline
            if pts and np.linalg.norm(line[0, :2] - pts[-1][:2]) < 1e-6:
                line = line[1:]
            elif pts:
                # Bridge the (small, validated) gap between consecutive centrelines; the
                # bridge edge belongs to the new segment.
                pass
            seg_start_vertex.append(max(0, len(pts) - 1))
            for p in line:
                if pts and np.linalg.norm(p[:2] - pts[-1][:2]) < 1e-6:
                    continue
                if pts:
                    seg_of_edge.append(k)
                pts.append(np.asarray(p, float))
        self.points = np.asarray(pts)  # (M, 3)
        if len(self.points) < 2:
            raise RouteValidationError("route centreline has no horizontal length")
        self.edge_segment = np.asarray(seg_of_edge, int)  # (M-1,)
        a = self.points[:-1, :2]
        b = self.points[1:, :2]
        self._a, self._b = a, b
        self._ab = b - a
        self.edge_length = np.linalg.norm(self._ab, axis=1)
        self.edge_tangent = self._ab / self.edge_length[:, None]
        self.edge_yaw = np.arctan2(self.edge_tangent[:, 1], self.edge_tangent[:, 0])
        self.s = np.concatenate([[0.0], np.cumsum(self.edge_length)])
        self.length = float(self.s[-1])
        # Segment s ranges.
        n = len(route.segments)
        self.segment_s = np.zeros((n, 2))
        for k in range(n):
            idx = np.nonzero(self.edge_segment == k)[0]
            if idx.size == 0:
                raise RouteValidationError(f"segment {route.segments[k].id} has no length")
            self.segment_s[k] = (self.s[idx[0]], self.s[idx[-1] + 1])
        # Gate positions along s: WP0 at 0, WP k at end of segment k-1.
        self.waypoint_s = np.concatenate([[0.0], self.segment_s[:, 1]])

    # -------------------------------------------------------------- queries

    def _edge_at(self, s: float) -> int:
        s = float(np.clip(s, 0.0, self.length))
        return int(np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.edge_length) - 1))

    def point_at(self, s: float | np.ndarray) -> np.ndarray:
        """(x, y, z) at arc length ``s`` (clamped). Vectorised over ``s``."""
        s_arr = np.clip(np.atleast_1d(np.asarray(s, float)), 0.0, self.length)
        e = np.clip(np.searchsorted(self.s, s_arr, side="right") - 1, 0, len(self.edge_length) - 1)
        t = (s_arr - self.s[e]) / self.edge_length[e]
        out = self.points[e] + t[:, None] * (self.points[e + 1] - self.points[e])
        return out[0] if np.ndim(s) == 0 else out

    def tangent_yaw_at(self, s: float | np.ndarray) -> float | np.ndarray:
        s_arr = np.clip(np.atleast_1d(np.asarray(s, float)), 0.0, self.length)
        e = np.clip(np.searchsorted(self.s, s_arr, side="right") - 1, 0, len(self.edge_length) - 1)
        yaw = self.edge_yaw[e]
        return float(yaw[0]) if np.ndim(s) == 0 else yaw

    def normal_at(self, s: float | np.ndarray) -> np.ndarray:
        yaw = np.atleast_1d(self.tangent_yaw_at(s))
        n = np.stack([-np.sin(yaw), np.cos(yaw)], axis=-1)
        return n[0] if np.ndim(s) == 0 else n

    def offset_points(self, s: np.ndarray, d: np.ndarray) -> np.ndarray:
        """XY of the points laterally offset by ``d`` from the centreline at ``s``."""
        base = self.point_at(np.asarray(s, float))[:, :2]
        return base + np.asarray(d, float)[:, None] * self.normal_at(np.asarray(s, float))

    def segment_index_at(self, s: float) -> int:
        return int(self.edge_segment[self._edge_at(s)])

    def segment_at(self, s: float) -> RouteSegment:
        return self.route.segments[self.segment_index_at(s)]

    def gait_at(self, s: float) -> str:
        return self.segment_at(s).gait

    def speed_limit_at(self, s: float) -> float:
        return self.segment_at(s).speed_limit

    def corridor_at(self, s: float) -> float:
        return self.segment_at(s).corridor_half_width

    def allow_detour_at(self, s: float) -> bool:
        return self.segment_at(s).allow_detour

    # ----------------------------------------------------------- projection

    def project(
        self,
        xy,
        z: float | None = None,
        *,
        s_hint: float | None = None,
        window: tuple[float, float] | None = None,
        z_gate: float = 0.6,
        s_tie: float = 0.03,
    ) -> Projection:
        """Closest point on the centreline, restricted to ``[s_hint-back, s_hint+fwd]``.

        ``z`` (ground height) removes candidates more than ``z_gate`` away vertically, which
        is what keeps a landing above the start from being matched to the start. Candidates
        within ``s_tie`` metres of the best distance are resolved by the smallest
        ``|s - s_hint|`` (continuity at corners / switchbacks).
        """
        p = np.asarray(xy, float)[:2]
        lo, hi = 0.0, self.length
        if s_hint is not None and window is not None:
            lo = max(0.0, s_hint - window[0])
            hi = min(self.length, s_hint + window[1])
            if hi <= lo:
                lo, hi = max(0.0, hi - 1e-3), min(self.length, lo + 1e-3)
        edges = np.nonzero((self.s[1:] >= lo) & (self.s[:-1] <= hi))[0]
        a, ab, length = self._a[edges], self._ab[edges], self.edge_length[edges]
        t = np.einsum("ij,ij->i", p - a, ab) / (length * length)
        # Keep the foot inside the window.
        t_lo = (lo - self.s[edges]) / length
        t_hi = (hi - self.s[edges]) / length
        t = np.clip(t, np.maximum(0.0, t_lo), np.minimum(1.0, t_hi))
        foot = a + t[:, None] * ab
        dist = np.linalg.norm(p - foot, axis=1)
        s_foot = self.s[edges] + t * length
        z_foot = self.points[edges, 2] + t * (self.points[edges + 1, 2] - self.points[edges, 2])
        mismatch = False
        if z is not None:
            dz = np.abs(z_foot - z)
            ok = dz <= z_gate
            if ok.any():
                score = np.where(ok, dist, np.inf)
            else:
                mismatch = True
                score = dist + dz  # best effort; flagged
        else:
            dz = np.full(len(edges), np.nan)
            score = dist.copy()
        j = int(np.argmin(score))
        if s_hint is not None and np.isfinite(score[j]):
            # Continuity only breaks near-ties (e.g. two legs of a switchback equally far);
            # it must never drag the foot off the geometrically closest edge.
            # Only DISTINCT local minima of distance along the (consecutive) edge sequence
            # compete. Adjacent edges' clamped end points are not minima and can never win
            # a "tie"; two legs of a switchback or the two sides of an inside corner can.
            left = np.concatenate([[np.inf], score[:-1]])
            right = np.concatenate([score[1:], [np.inf]])
            minima = np.nonzero((score <= left) & (score <= right) & np.isfinite(score))[0]
            near = minima[score[minima] <= score[j] + s_tie]
            if near.size:
                j = int(near[np.argmin(np.abs(s_foot[near] - s_hint))])
        e = int(edges[j])
        tangent = self.edge_tangent[e]
        rel = p - foot[j]
        cross = tangent[0] * rel[1] - tangent[1] * rel[0]
        d = math.copysign(float(dist[j]), cross) if dist[j] > 1e-9 else 0.0
        return Projection(
            s=float(s_foot[j]),
            d=d,
            segment_index=int(self.edge_segment[e]),
            tangent_yaw=float(self.edge_yaw[e]),
            distance=float(dist[j]),
            point=foot[j].copy(),
            z_path=float(z_foot[j]),
            dz=float(dz[j]),
            z_mismatch=mismatch,
        )

    def project_many(self, points: np.ndarray, s_lo: float, s_hi: float) -> tuple[np.ndarray, np.ndarray]:
        """(s, unsigned distance) of many points against the path section [s_lo, s_hi]."""
        pts = np.asarray(points, float)[:, :2]
        edges = np.nonzero((self.s[1:] >= s_lo) & (self.s[:-1] <= s_hi))[0]
        if edges.size == 0:
            edges = np.array([self._edge_at(s_lo)])
        a, ab, length = self._a[edges], self._ab[edges], self.edge_length[edges]
        rel = pts[:, None, :] - a[None, :, :]
        t = np.clip(np.einsum("pei,ei->pe", rel, ab) / (length * length)[None, :], 0.0, 1.0)
        foot = a[None, :, :] + t[..., None] * ab[None, :, :]
        dist = np.linalg.norm(pts[:, None, :] - foot, axis=2)
        k = dist.argmin(axis=1)
        rows = np.arange(len(pts))
        s = self.s[edges][k] + t[rows, k] * length[k]
        return s, dist[rows, k]

    def distance_to_path(self, points: np.ndarray, s_lo: float, s_hi: float) -> np.ndarray:
        """Unsigned horizontal distance of many points to the path section [s_lo, s_hi]."""
        pts = np.asarray(points, float)[:, :2]
        edges = np.nonzero((self.s[1:] >= s_lo) & (self.s[:-1] <= s_hi))[0]
        if edges.size == 0:
            edges = np.array([self._edge_at(s_lo)])
        a, ab, length = self._a[edges], self._ab[edges], self.edge_length[edges]
        rel = pts[:, None, :] - a[None, :, :]
        t = np.clip(np.einsum("pei,ei->pe", rel, ab) / (length * length)[None, :], 0.0, 1.0)
        foot = a[None, :, :] + t[..., None] * ab[None, :, :]
        return np.linalg.norm(pts[:, None, :] - foot, axis=2).min(axis=1)


class RouteTracker:
    """Windowed projection that follows the robot along ``s``.

    The first call (or a lost track) searches the whole path with the z gate; afterwards
    only ``[s_prev - back, s_prev + forward]`` is searched. ``s_max`` (normally the current
    unscored gate plus a little slack) stops the projection running past a gate the robot
    has not been credited with.
    """

    def __init__(
        self,
        path: RoutePath,
        *,
        back: float = 1.0,
        forward: float = 2.5,
        relock_distance: float = 1.5,
        z_gate: float = 0.6,
    ) -> None:
        self.path = path
        self.back = back
        self.forward = forward
        self.relock_distance = relock_distance
        self.z_gate = z_gate
        self.s: float | None = None

    def reset(self, s: float | None = None) -> None:
        self.s = s

    def update(self, xy, z_ground: float | None = None, *, s_max: float | None = None) -> Projection:
        path = self.path
        if self.s is None:
            proj = path.project(xy, z_ground, z_gate=self.z_gate)
            if s_max is not None and proj.s > s_max:
                proj = path.project(
                    xy, z_ground, s_hint=0.0, window=(0.0, s_max), z_gate=self.z_gate
                )
            proj = _with(proj, relocked=True)
        else:
            fwd = self.forward
            if s_max is not None:
                fwd = max(0.0, min(fwd, s_max - self.s))
            proj = path.project(
                xy, z_ground, s_hint=self.s, window=(self.back, fwd), z_gate=self.z_gate
            )
            if proj.distance > self.relock_distance or proj.z_mismatch:
                hi = path.length if s_max is None else s_max
                wide = path.project(
                    xy, z_ground, s_hint=self.s, window=(self.s, max(0.0, hi - self.s)),
                    z_gate=self.z_gate,
                )
                if wide.distance + 1e-6 < proj.distance and not wide.z_mismatch:
                    proj = _with(wide, relocked=True)
        self.s = proj.s
        return proj


def _with(proj: Projection, **changes) -> Projection:
    values = {k: getattr(proj, k) for k in proj.__dataclass_fields__}
    values.update(changes)
    return Projection(**values)


# ---------------------------------------------------------- terrain cross-check


@dataclass
class CrossCheckConfig:
    #: Sustained body pitch at/above this on a FLAT segment means the robot is on steps
    #: with the flat gait -> HOLD (zero command) until it clears. Degrees.
    stairs_pitch_deg: float = 12.0
    #: Height-map relief that looks like a riser across the corridor -> WARN only (a wall
    #: or box reads the same way; the planner handles those).
    stairs_rise: float = 0.12
    stairs_span: float = 0.8
    stairs_slope_deg: float = 12.0
    #: On a STAIRS segment, pitch below this and relief below ``flat_relief`` -> WARN.
    flat_pitch_deg: float = 4.0
    flat_relief: float = 0.05
    #: Evidence must persist this long, seconds.
    dwell: float = 0.5
    #: Minimum fraction of valid cells in the wheel corridor to judge the height map.
    min_valid_fraction: float = 0.8


@dataclass(frozen=True)
class CrossCheckResult:
    level: str  # "ok" | "warn" | "hold"
    reason: str

    @property
    def hold(self) -> bool:
        return self.level == "hold"


class TerrainCrossCheck:
    """Warn/hold when live terrain disagrees with the segment's declared gait.

    Never requests a gait: the route file is the only gait authority and a switch only
    happens stopped at a waypoint, through the router's settle -> switch -> confirm.
    """

    def __init__(self, config: CrossCheckConfig | None = None) -> None:
        self.config = config or CrossCheckConfig()
        self._pitch_for = 0.0
        self._relief_for = 0.0
        self._flat_for = 0.0

    def reset(self) -> None:
        self._pitch_for = self._relief_for = self._flat_for = 0.0

    def update(
        self,
        gait: str,
        dt: float,
        *,
        pitch: float | None = None,
        height: np.ndarray | None = None,
        mask: np.ndarray | None = None,
    ) -> CrossCheckResult:
        c = self.config
        pitch_deg = None if pitch is None or not math.isfinite(pitch) else abs(math.degrees(pitch))
        relief = _masked_relief(height, mask, c.min_valid_fraction)
        stairs_pitch = pitch_deg is not None and pitch_deg >= c.stairs_pitch_deg
        stairs_relief = relief is not None and (
            (relief.rise >= c.stairs_rise and relief.rise_fraction >= c.stairs_span)
            or abs(relief.slope_deg) >= c.stairs_slope_deg
        )
        flat_like = (
            pitch_deg is not None
            and pitch_deg < c.flat_pitch_deg
            and relief is not None
            and relief.rise < c.flat_relief
            and abs(relief.slope_deg) < c.flat_pitch_deg
        )
        self._pitch_for = self._pitch_for + dt if stairs_pitch else 0.0
        self._relief_for = self._relief_for + dt if stairs_relief else 0.0
        self._flat_for = self._flat_for + dt if flat_like else 0.0
        if gait == "flat":
            if self._pitch_for >= c.dwell:
                return CrossCheckResult(
                    "hold", f"pitch {pitch_deg:.0f}deg on flat-gait segment: stairs evidence"
                )
            if self._relief_for >= c.dwell:
                return CrossCheckResult(
                    "warn",
                    f"height map rise {relief.rise:.2f}m across {relief.rise_fraction:.0%}, "
                    f"slope {relief.slope_deg:+.0f}deg on flat-gait segment",
                )
        elif gait == "stairs" and self._flat_for >= c.dwell:
            return CrossCheckResult("warn", "flat terrain on stairs-gait segment")
        if relief is None and height is not None:
            return CrossCheckResult("ok", "insufficient height evidence")
        return CrossCheckResult("ok", "")


def _masked_relief(height, mask, min_valid_fraction):
    if height is None:
        return None
    from s10_auto_nav.local_planner import _wheel_corridor, terrain_relief

    grid = np.asarray(height, float)
    valid = np.isfinite(grid) if mask is None else (np.asarray(mask, bool) & np.isfinite(grid))
    if grid.ndim != 2 or valid.shape != grid.shape:
        return None
    corridor = _wheel_corridor(valid)
    if corridor.size == 0 or corridor.mean() < min_valid_fraction:
        return None
    # Warning-only evidence: fill unknown cells with the valid median so the plane fit
    # runs. Never used for planning / free-space decisions.
    filled = np.where(valid, grid, float(np.median(grid[valid])))
    relief = terrain_relief(filled)
    return relief if relief.valid else None
