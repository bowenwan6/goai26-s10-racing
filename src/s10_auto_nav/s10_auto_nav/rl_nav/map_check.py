"""Is what the LiDAR sees ahead the terrain the map already knows, or something new?

The follower's local grid and the 72-bin conservative scan both read rising terrain -- a slope, a
terrace edge, the rocks of a creek bed -- as blocked, and there is no telling that apart from a box
or a person from the robot's own sensors alone. The map can: a return lying on the mapped surface is
terrain; a return standing clearly above it is not in the map. This is map-based change detection,
point by point:

    obstacle point = inside the route's corridor ahead,
                     in the body-height band (-0.25 .. 0.75 m from the base, the scan's band),
                     on mapped ground,
                     more than ``rise`` above the mapped surface (ground + mapped obstacles).

``MapSurface`` is the robot's map raster (the same one route preparation used): ground heights,
known mask and static obstacle heights on a regular grid. On the robot it is loaded from an ``.npz``
saved next to the prepared route (``MapSurface.save``); in simulation it is built from the course
terrain before any test obstacle is added.
"""

from __future__ import annotations

import math

import numpy as np


class MapSurface:
    def __init__(self, ground, known, obstacle_height, origin, res):
        self.ground = np.asarray(ground, float)
        self.known = np.asarray(known, bool)
        obst = np.nan_to_num(np.asarray(obstacle_height, float))
        self.surface = np.where(obst > 0, self.ground + obst, self.ground)
        self.origin = (float(origin[0]), float(origin[1]))
        self.res = float(res)

    @classmethod
    def from_terrain(cls, terrain) -> MapSurface:
        return cls(
            terrain.ground, terrain.known, terrain.obstacle_height, terrain.origin, terrain.res
        )

    @classmethod
    def load(cls, path) -> MapSurface:
        z = np.load(path)
        return cls(
            z["ground"], z["known"], z["obstacle_height"], tuple(z["origin"]), float(z["res"])
        )

    def save(self, path) -> None:
        np.savez_compressed(
            path,
            ground=self.ground,
            known=self.known,
            obstacle_height=self.surface - self.ground,
            origin=np.array(self.origin),
            res=self.res,
        )

    def at(self, x, y):
        """(surface z, known) at map points; outside the raster: unknown."""
        ix = np.floor((np.asarray(x, float) - self.origin[0]) / self.res).astype(int)
        iy = np.floor((np.asarray(y, float) - self.origin[1]) / self.res).astype(int)
        ny, nx = self.ground.shape
        inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        ixc, iyc = np.clip(ix, 0, nx - 1), np.clip(iy, 0, ny - 1)
        return np.where(inside, self.surface[iyc, ixc], np.nan), inside & self.known[iyc, ixc]


def unexpected_ahead(
    points_yaw,
    pose,
    path,
    s_from,
    surface: MapSurface,
    reach=1.6,
    half_width=0.35,
    band=(-0.25, 0.75),
    rise=0.2,
    min_points=5,
):
    """Distance (m) to the nearest return in the route's corridor ahead that the map does not
    explain, or None. ``points_yaw``: N x 3 in the robot's yaw frame relative to the base (x
    forward, y left, z up); ``pose``: (x, y, z, yaw) of the base in the map frame."""
    if points_yaw is None or len(points_yaw) == 0:
        return None
    p = np.asarray(points_yaw, float)
    x, y, z, yaw = pose
    keep = (p[:, 2] >= band[0]) & (p[:, 2] <= band[1]) & (np.hypot(p[:, 0], p[:, 1]) <= reach + 0.5)
    p = p[keep]
    if len(p) == 0:
        return None
    c, s = math.cos(yaw), math.sin(yaw)
    wx, wy, wz = x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1], z + p[:, 2]
    lo, hi = s_from + 0.2, min(path.length, s_from + reach)
    if hi <= lo:
        return None
    s_pt, dist = path.project_many(np.column_stack([wx, wy]), lo, hi)
    corridor = (dist <= half_width) & (s_pt > lo + 1e-3) & (s_pt < hi - 1e-3)
    surf, known = surface.at(wx, wy)
    new = corridor & known & (wz - surf > rise)
    if int(new.sum()) < min_points:
        return None
    return float(np.hypot(p[new, 0], p[new, 1]).min())
