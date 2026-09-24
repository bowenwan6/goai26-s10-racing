"""route_v2 support for the native runtime, kept free of ROS so it can be tested.

* :func:`load_route` accepts either a legacy draft (``{"waypoints": [... kind ...]}``) or a
  ``s10_route_v2`` file and returns the native point list the runtime already consumes
  (gait ``kind`` per target = gait of the INCOMING segment) plus the parsed route.
* :class:`CorridorMonitor` replaces the fixed 0.35 m straight-line corridor fault with the
  segment's ``corridor_half_width`` (+ margin) measured against the taught centreline.
  Legacy routes keep the exact old 0.35 m straight-segment check.
* :func:`native_router_for` builds a ``NativeGaitRouter`` whose gait per target and speed
  cap per target come from the route. The settle -> switch -> confirm sequence and every
  other safety gate in the router are untouched.
* :class:`TerrainMonitor` wraps ``route_v2.TerrainCrossCheck``: warn / hold only.
"""

from __future__ import annotations

import numpy as np

from native_transfer.contracts import validate_route
from native_transfer.router import Limits, NativeGaitRouter
from s10_auto_nav.route_v2 import (
    SCHEMA,
    CrossCheckConfig,
    CrossCheckResult,
    RoutePath,
    RouteTracker,
    RouteV2,
    TerrainCrossCheck,
)

LEGACY_CORRIDOR = 0.35
ROUTE_V2_CORRIDOR_MARGIN = 0.10


def is_route_v2(data) -> bool:
    return isinstance(data, dict) and data.get("schema") == SCHEMA


def load_route(data, *, expected_map_id: str | None = None):
    """Return ``(points, route_or_None)``; ``points`` passes ``validate_route``."""
    if is_route_v2(data):
        route = RouteV2.from_dict(data, expected_map_id=expected_map_id)
        points = route.native_points()
        validate_route({"waypoints": points})
        return points, route
    return validate_route(data), None


def native_router_for(points, route: RouteV2 | None = None, limits: Limits | None = None, **kw):
    kinds = [p["kind"] for p in points]
    caps = None if route is None else [p["speed_limit"] for p in points]
    return NativeGaitRouter(kinds, limits, speed_caps=caps, **kw)


class CorridorMonitor:
    """Is the robot inside the allowed corridor of the segment into ``cursor``?"""

    def __init__(self, points, route: RouteV2 | None = None, *, margin=ROUTE_V2_CORRIDOR_MARGIN):
        self.points = points
        self.route = route
        self.margin = float(margin)
        self.path = None if route is None else RoutePath(route)
        self.tracker = None if self.path is None else RouteTracker(self.path)

    def check(self, cursor: int, xyz_ground) -> tuple[bool, str, float, dict]:
        """``(ok, reason, along, info)``; ``along`` feeds the runtime's progress counter.

        ``xyz_ground`` is the base position with ``body_z_offset`` already removed (route
        z is ground height).
        """
        xyz = np.asarray(xyz_ground, float)
        if self.route is None:
            return self._legacy(cursor, xyz)
        path = self.path
        seg_in = self.route.incoming_segment(cursor)
        s_gate = float(path.waypoint_s[cursor])
        proj = self.tracker.update(xyz[:2], xyz[2], s_max=s_gate + 1.0)
        if cursor == 0:
            distance = float(np.linalg.norm(xyz[:2] - self.route.waypoints[0].xy))
            allowed = seg_in.corridor_half_width + self.margin
            info = {"s": proj.s, "d": distance, "allowed": allowed, "segment": seg_in.id}
            ok = distance <= allowed
            return ok, "" if ok else "route_corridor_exceeded", 0.0, info
        seg_here = self.route.segments[proj.segment_index]
        # Near a boundary the robot may be on the previous segment; allow the wider of the
        # two, never more than the route file says.
        width = max(seg_here.corridor_half_width, seg_in.corridor_half_width)
        allowed = width + self.margin
        along = max(0.0, proj.s - float(path.waypoint_s[cursor - 1]))
        info = {
            "s": proj.s,
            "d": proj.d,
            "allowed": allowed,
            "segment": seg_here.id,
            "z_mismatch": proj.z_mismatch,
        }
        if proj.z_mismatch:
            return False, "route_level_mismatch", along, info
        ok = abs(proj.d) <= allowed
        return ok, "" if ok else "route_corridor_exceeded", along, info

    def _legacy(self, cursor, xyz):
        # Byte-for-byte the pre-route_v2 runtime rule.
        target = np.array(self.points[cursor]["position"][:2])
        start = np.array(self.points[max(0, cursor - 1)]["position"][:2])
        line = target - start
        length = np.linalg.norm(line)
        along = 0.0 if length < 1e-6 else float(np.dot(xyz[:2] - start, line / length))
        closest = start if length < 1e-6 else start + np.clip(along, 0, length) * line / length
        distance = float(np.linalg.norm(xyz[:2] - closest))
        ok = distance <= LEGACY_CORRIDOR
        info = {"d": distance, "allowed": LEGACY_CORRIDOR}
        return ok, "" if ok else "route_corridor_exceeded", along, info


class TerrainMonitor:
    """Warn/hold only. The gait authority stays with the route + router."""

    def __init__(self, route: RouteV2 | None, config: CrossCheckConfig | None = None):
        self.route = route
        self.check = TerrainCrossCheck(config)
        self.last = CrossCheckResult("ok", "")

    def update(self, cursor, dt, *, pitch=None, height=None, mask=None) -> CrossCheckResult:
        if self.route is None or cursor >= len(self.route.waypoints):
            self.last = CrossCheckResult("ok", "")
            return self.last
        gait = self.route.incoming_segment(cursor).gait
        self.last = self.check.update(gait, dt, pitch=pitch, height=height, mask=mask)
        return self.last
