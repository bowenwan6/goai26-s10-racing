"""Injected obstacles (boxes / cylinders) and route-relative placement helpers.

Placement is by route arc length `s` (m along the concatenated centerline) and a signed
lateral offset (m, + = left of the direction of travel). Obstacles stand on the ground at
their centre (`base_z` filled in by Terrain.inject) and extend `height` upward.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np


@dataclass
class Obstacle:
    x: float
    y: float
    height: float
    base_z: float | None = None
    name: str = ""

    def contains(self, x, y):  # pragma: no cover - abstract
        raise NotImplementedError

    def distance(self, x, y):  # pragma: no cover - abstract
        """Signed XY distance to the footprint boundary (negative inside)."""
        raise NotImplementedError

    def to_dict(self):
        d = asdict(self)
        d["type"] = type(self).__name__.lower()
        return d


@dataclass
class Box(Obstacle):
    size_x: float = 0.4  # along its own yaw
    size_y: float = 0.4
    yaw: float = 0.0

    def _local(self, x, y):
        dx, dy = np.asarray(x) - self.x, np.asarray(y) - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def contains(self, x, y):
        u, v = self._local(x, y)
        return (np.abs(u) <= self.size_x / 2) & (np.abs(v) <= self.size_y / 2)

    def distance(self, x, y):
        u, v = self._local(x, y)
        qu, qv = np.abs(u) - self.size_x / 2, np.abs(v) - self.size_y / 2
        outside = np.hypot(np.maximum(qu, 0), np.maximum(qv, 0))
        inside = np.minimum(np.maximum(qu, qv), 0)
        return outside + inside

    def corners(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hx, hy = self.size_x / 2, self.size_y / 2
        pts = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
        return np.array([(self.x + c * u - s * v, self.y + s * u + c * v) for u, v in pts])


@dataclass
class Cylinder(Obstacle):
    radius: float = 0.2

    def contains(self, x, y):
        return np.hypot(np.asarray(x) - self.x, np.asarray(y) - self.y) <= self.radius

    def distance(self, x, y):
        return np.hypot(np.asarray(x) - self.x, np.asarray(y) - self.y) - self.radius


def place_on_route(route, s: float, lateral: float, kind: str = "box", **kw) -> Obstacle:
    """Obstacle centred at centerline(s) + lateral * left-normal; boxes align with the path."""
    (x, y, _), heading = route.point_at(s)
    nx, ny = -math.sin(heading), math.cos(heading)
    cx, cy = x + lateral * nx, y + lateral * ny
    height = kw.pop("height", 0.5)
    name = kw.pop("name", f"{kind}@s={s:.1f},lat={lateral:+.2f}")
    if kind == "box":
        return Box(cx, cy, height, name=name, yaw=kw.pop("yaw", heading), **kw)
    if kind == "cylinder":
        return Cylinder(cx, cy, height, name=name, **kw)
    raise ValueError(f"unknown obstacle kind {kind}")


def blocked_corridor(route, s: float, extra: float = 0.6, depth: float = 0.3,
                     height: float = 0.8) -> Box:
    """A wall across the whole allowed corridor at s (no legal detour exists)."""
    seg = route.segment_at(s)
    width = 2 * seg["corridor_half_width"] + 2 * extra
    (x, y, _), heading = route.point_at(s)
    return Box(x, y, height, name=f"blocked_corridor@s={s:.1f}", size_x=depth, size_y=width,
               yaw=heading)


def from_dict(d: dict) -> Obstacle:
    d = dict(d)
    kind = d.pop("type")
    return {"box": Box, "cylinder": Cylinder}[kind](**d)
