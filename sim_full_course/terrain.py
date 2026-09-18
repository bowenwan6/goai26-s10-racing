"""2.5D terrain: ground heightfield + static obstacle layer + unknown mask + injected shapes.

Grid layout: array[row=iy, col=ix]; cell (iy, ix) centre = origin + (i + 0.5) * res, map frame.
Ground is sampled bilinearly between cell centres (from `ground_filled`, which only exists so
every point has a value); `known_at` reports whether the cell was observed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from sim_full_course.obstacles import Obstacle


@dataclass
class Terrain:
    ground: np.ndarray  # filled ground heights (no NaN)
    known: np.ndarray  # bool
    obstacle_height: np.ndarray  # height above ground of static obstacle layer, 0 = free
    origin: tuple[float, float]
    res: float
    multi_level: np.ndarray | None = None
    step: np.ndarray | None = None
    frame: str = "map"
    injected: list[Obstacle] = field(default_factory=list)

    def __post_init__(self):
        self.ground = np.asarray(self.ground, np.float64)
        self.known = np.asarray(self.known, bool)
        self.obstacle_height = np.nan_to_num(np.asarray(self.obstacle_height, np.float64))
        self.origin = (float(self.origin[0]), float(self.origin[1]))
        self.res = float(self.res)
        if self.ground.shape != self.known.shape or self.ground.shape != self.obstacle_height.shape:
            raise ValueError("layer shapes differ")
        if not np.isfinite(self.ground).all():
            raise ValueError("ground must be filled (no NaN); keep unknown in `known`")
        if self.step is None:
            self.step = _step_map(self.ground)
        self.step = np.where(self.known, np.nan_to_num(self.step), 0.0)
        self._obstacle_mask = self.obstacle_height > 0
        self._obst_top = self.ground + self.obstacle_height

    # ---------- construction ----------
    @classmethod
    def load(cls, npz: Path | str) -> Terrain:
        t = np.load(npz)
        return cls(
            ground=t["ground_filled"],
            known=t["known"],
            obstacle_height=np.where(t["obstacle"], t["obstacle_height"], 0.0),
            origin=tuple(t["origin"]),
            res=float(t["resolution"]),
            multi_level=t["multi_level"],
            step=t["step"],
            frame=str(t["frame"]),
        )

    @classmethod
    def flat(cls, size_x=20.0, size_y=10.0, res=0.1, z=0.0, origin=(-2.0, -5.0)) -> Terrain:
        ny, nx = round(size_y / res), round(size_x / res)
        return cls(np.full((ny, nx), z), np.ones((ny, nx), bool), np.zeros((ny, nx)), origin, res)

    @property
    def shape(self):
        return self.ground.shape

    # ---------- indexing ----------
    def cell(self, x, y):
        ny, nx = self.shape
        ix = np.floor((np.asarray(x) - self.origin[0]) / self.res).astype(int)
        iy = np.floor((np.asarray(y) - self.origin[1]) / self.res).astype(int)
        inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        return np.clip(iy, 0, ny - 1), np.clip(ix, 0, nx - 1), inside

    def ground_at(self, x, y):
        """Bilinear ground (outside the grid: clamped to the border)."""
        ny, nx = self.shape
        fx = np.clip((np.asarray(x, float) - self.origin[0]) / self.res - 0.5, 0, nx - 1)
        fy = np.clip((np.asarray(y, float) - self.origin[1]) / self.res - 0.5, 0, ny - 1)
        x0 = np.minimum(np.floor(fx).astype(int), nx - 2) if nx > 1 else np.zeros_like(fx, int)
        y0 = np.minimum(np.floor(fy).astype(int), ny - 2) if ny > 1 else np.zeros_like(fy, int)
        ax, ay = fx - x0, fy - y0
        g = self.ground
        x1 = np.minimum(x0 + 1, nx - 1)
        y1 = np.minimum(y0 + 1, ny - 1)
        return (
            g[y0, x0] * (1 - ax) * (1 - ay)
            + g[y0, x1] * ax * (1 - ay)
            + g[y1, x0] * (1 - ax) * ay
            + g[y1, x1] * ax * ay
        )

    def known_at(self, x, y):
        iy, ix, inside = self.cell(x, y)
        return self.known[iy, ix] & inside

    def static_obstacle_at(self, x, y):
        iy, ix, inside = self.cell(x, y)
        return self._obstacle_mask[iy, ix] & inside

    def step_at(self, x, y):
        iy, ix, inside = self.cell(x, y)
        return np.where(inside, self.step[iy, ix], 0.0)

    # ---------- obstacles ----------
    def inject(self, *obstacles: Obstacle) -> None:
        for o in obstacles:
            if o.base_z is None:
                o.base_z = float(self.ground_at(o.x, o.y))
            self.injected.append(o)

    def clear_injected(self) -> None:
        self.injected.clear()

    def injected_top(self, x, y):
        """Max top z of injected obstacles covering (x, y); -inf where none."""
        x = np.asarray(x, float)
        y = np.asarray(y, float)
        top = np.full(np.broadcast(x, y).shape, -np.inf)
        for o in self.injected:
            top = np.where(o.contains(x, y), np.maximum(top, o.base_z + o.height), top)
        return top

    def surface_at(self, x, y):
        """Highest solid surface (ground, static obstacle top, injected top)."""
        x = np.asarray(x, float)
        y = np.asarray(y, float)
        s = self.ground_at(x, y)
        iy, ix, inside = self.cell(x, y)
        ob = self._obstacle_mask[iy, ix] & inside
        s = np.where(ob, np.maximum(s, self._obst_top[iy, ix]), s)
        if self.injected:
            s = np.maximum(s, self.injected_top(x, y))
        return s

    def obstacle_at(self, x, y):
        """Occupied by the static layer or an injected shape (footprint collision test)."""
        occ = self.static_obstacle_at(x, y)
        for o in self.injected:
            occ = occ | o.contains(x, y)
        return occ

    def ground_plane(self, x, y, yaw, length=0.9, width=0.5):
        """Least-squares plane over the footprint -> (z_centre, pitch, roll).

        pitch > 0 = nose down, roll > 0 = left side up (ROS REP-103 rotations)."""
        u = np.linspace(-length / 2, length / 2, 5)
        v = np.linspace(-width / 2, width / 2, 3)
        uu, vv = np.meshgrid(u, v)
        uu, vv = uu.ravel(), vv.ravel()
        c, s = np.cos(yaw), np.sin(yaw)
        z = self.ground_at(x + c * uu - s * vv, y + s * uu + c * vv)
        a = np.column_stack([uu, vv, np.ones_like(uu)])
        (gu, gv, z0), *_ = np.linalg.lstsq(a, z, rcond=None)
        return float(z0), float(-np.arctan(gu)), float(np.arctan(gv))

    def obstacle_distance_field(self):
        """Euclidean distance (m) from each cell centre to the nearest static obstacle cell."""
        from scipy import ndimage

        if not self._obstacle_mask.any():
            return np.full(self.shape, np.inf)
        return ndimage.distance_transform_edt(~self._obstacle_mask) * self.res

    def crop(self, xmin, xmax, ymin, ymax) -> Terrain:
        iy0, ix0, _ = self.cell(xmin, ymin)
        iy1, ix1, _ = self.cell(xmax, ymax)
        sl = (slice(int(iy0), int(iy1) + 1), slice(int(ix0), int(ix1) + 1))
        return Terrain(
            self.ground[sl].copy(), self.known[sl].copy(), self.obstacle_height[sl].copy(),
            (self.origin[0] + int(ix0) * self.res, self.origin[1] + int(iy0) * self.res),
            self.res,
            None if self.multi_level is None else self.multi_level[sl].copy(),
            self.step[sl].copy(), self.frame,
        )


def _step_map(g):
    step = np.zeros_like(g)
    for dy, dx in ((0, 1), (1, 0)):
        d = np.abs(np.diff(g, axis=1 if dx else 0))
        if dx:
            step[:, 1:] = np.maximum(step[:, 1:], d)
            step[:, :-1] = np.maximum(step[:, :-1], d)
        else:
            step[1:, :] = np.maximum(step[1:, :], d)
            step[:-1, :] = np.maximum(step[:-1, :], d)
    return step
