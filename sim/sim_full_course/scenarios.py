"""Named obstacle scenarios: f(route, s_override) -> list[Obstacle].

Default placement: middle of the first flat, detour-allowed stretch >= 8 m long (so the
planner has room to leave and rejoin the centerline before the next WP / gait switch).
"""

from __future__ import annotations

from sim_full_course.obstacles import Cylinder, blocked_corridor, place_on_route
from sim_full_course.route import Route


def default_s(route: Route, min_len: float = 8.0) -> float:
    for i, seg in enumerate(route.segments):
        a = route.seg_s0[i]
        b = route.wp_s[i + 1]
        if seg["gait"] == "flat" and seg["allow_detour"] and b - a >= min_len:
            return 0.5 * (a + b)
    return 0.5 * route.length


def nominal(route, s=None):
    return []


def detour_box(route, s=None):
    """0.5 x 0.5 x 0.5 m box centred on the centerline: must detour and rejoin."""
    s = default_s(route) if s is None else s
    return [place_on_route(route, s, 0.0, "box", size_x=0.5, size_y=0.5, height=0.5)]


def offset_cylinder(route, s=None):
    """r = 0.15 m post 0.30 m left of the centerline: intrudes the 0.5 m footprint."""
    s = default_s(route) if s is None else s
    return [place_on_route(route, s, 0.30, "cylinder", radius=0.15, height=0.8)]


def slalom(route, s=None):
    """Two boxes, alternating sides, 2.5 m apart: detour, rejoin, detour."""
    s = default_s(route) if s is None else s
    return [
        place_on_route(route, s - 1.25, 0.20, "box", size_x=0.4, size_y=0.5, height=0.5),
        place_on_route(route, s + 1.25, -0.20, "box", size_x=0.4, size_y=0.5, height=0.5),
    ]


def blocked(route, s=None):
    """Wall across the whole allowed corridor: correct behaviour is stop + report, no contact."""
    s = default_s(route) if s is None else s
    return [blocked_corridor(route, s)]


def low_curb(route, s=None):
    """0.12 m high kerb across the path (in the obstacle band, but steppable in stairs gait)."""
    s = default_s(route) if s is None else s
    seg = route.segment_at(s)
    from sim_full_course.obstacles import Box

    (x, y, _), h = route.point_at(s)
    return [Box(x, y, 0.12, name=f"curb@s={s:.1f}", size_x=0.15,
                size_y=2 * seg["corridor_half_width"] + 1.0, yaw=h)]


SCENARIOS = {
    "nominal": nominal,
    "detour_box": detour_box,
    "offset_cylinder": offset_cylinder,
    "slalom": slalom,
    "blocked_corridor": blocked,
    "low_curb": low_curb,
}
__all__ = ["SCENARIOS", "Cylinder", "default_s"]
