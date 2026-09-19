"""Is what the LiDAR sees ahead the terrain the map already knows, or something new?

The follower's local grid and the 72-bin conservative scan both read rising terrain -- a slope, a
terrace edge, the rocks of a creek bed -- as blocked, and there is no telling that apart from a box
or a person from the robot's own sensors alone. The map can: a return lying on the mapped surface is
terrain; a return standing clearly above it is not in the map. This is map-based change detection,
point by point:

    obstacle point = inside the route's corridor ahead,
                     in the body-height band (-0.25 .. 0.75 m from the base, the scan's band),
                     on mapped ground,
                     more than ``rise`` above the highest mapped surface (ground + mapped
                     obstacles) within ``slack`` of it.

The ``slack`` (0.3 m) absorbs localisation error: with the pose 0.2 m off, a rock's side seen by
the LiDAR lands beside the rock on the map -- above the ground there -- and must not read as new.
A box on flat ground stands clear of it all the same.

``MapSurface`` is the robot's map raster (the same one route preparation used): ground heights,
known mask and static obstacle heights on a regular grid. On the robot it is loaded from an ``.npz``
saved next to the prepared route (``MapSurface.save``); in simulation it is built from the course
terrain before any test obstacle is added.
"""

from __future__ import annotations

import math

import numpy as np
from scipy import ndimage


class MapSurface:
    def __init__(self, ground, known, obstacle_height, origin, res, slack=0.3):
        self.ground = np.asarray(ground, float)
        self.known = np.asarray(known, bool)
        obst = np.nan_to_num(np.asarray(obstacle_height, float))
        self.surface = np.where(obst > 0, self.ground + obst, self.ground)
        self.origin = (float(origin[0]), float(origin[1]))
        self.res = float(res)
        # Highest known surface within ``slack`` of every cell (unknown cells do not raise it).
        r = max(1, round(slack / self.res))
        yy, xx = np.mgrid[-r : r + 1, -r : r + 1]
        disk = (xx * xx + yy * yy) <= r * r
        lowest = float(np.nanmin(self.surface)) - 1.0
        self.surface_max = ndimage.grey_dilation(
            np.where(self.known, self.surface, lowest), footprint=disk
        )

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

    def at(self, x, y, highest_nearby=False):
        """(surface z, known) at map points; outside the raster: unknown. ``highest_nearby``: the
        highest known surface within the slack instead of the cell's own."""
        ix = np.floor((np.asarray(x, float) - self.origin[0]) / self.res).astype(int)
        iy = np.floor((np.asarray(y, float) - self.origin[1]) / self.res).astype(int)
        ny, nx = self.ground.shape
        inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        ixc, iyc = np.clip(ix, 0, nx - 1), np.clip(iy, 0, ny - 1)
        grid = self.surface_max if highest_nearby else self.surface
        return np.where(inside, grid[iyc, ixc], np.nan), inside & self.known[iyc, ixc]


def unexplained(points_yaw, pose, surface: MapSurface, reach=2.5, band=(-0.25, 0.75), rise=0.2):
    """Map-frame xy of the returns within ``reach`` that the map does not explain: in the
    body-height band, on mapped ground, more than ``rise`` above the highest mapped surface within
    the slack.
    ``points_yaw``: N x 3 in the robot's yaw frame relative to the base (x forward, y left, z up);
    ``pose``: (x, y, z, yaw) of the base in the map frame."""
    if points_yaw is None or len(points_yaw) == 0:
        return np.zeros((0, 2))
    p = np.asarray(points_yaw, float)
    x, y, z, yaw = pose
    keep = (p[:, 2] >= band[0]) & (p[:, 2] <= band[1]) & (np.hypot(p[:, 0], p[:, 1]) <= reach)
    p = p[keep]
    if len(p) == 0:
        return np.zeros((0, 2))
    c, s = math.cos(yaw), math.sin(yaw)
    wx, wy, wz = x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1], z + p[:, 2]
    surf, known = surface.at(wx, wy, highest_nearby=True)
    new = known & (wz - surf > rise)
    return np.column_stack([wx[new], wy[new]])


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
    new = unexplained(points_yaw, pose, surface, reach + 0.5, band, rise)
    if len(new) == 0:
        return None
    lo, hi = s_from + 0.2, min(path.length, s_from + reach)
    if hi <= lo:
        return None
    s_pt, dist = path.project_many(new, lo, hi)
    corridor = (dist <= half_width) & (s_pt > lo + 1e-3) & (s_pt < hi - 1e-3)
    if int(corridor.sum()) < min_points:
        return None
    return float(np.hypot(new[corridor, 0] - pose[0], new[corridor, 1] - pose[1]).min())
