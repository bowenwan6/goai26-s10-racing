"""Body-frame height map sampling.

A height map is the standard exteroceptive observation for perceptive legged locomotion:
a grid of terrain heights around the robot, expressed relative to the base. It is cheap
to compute (one downward ray per cell) and, unlike a raw depth image, it is invariant to
the robot's pitch and roll once the base height is subtracted.

On hardware the same grid is produced by accumulating lidar returns into a local
elevation map, so a policy trained on this observation transfers without a change of
input layout.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass
class HeightmapConfig:
    """Grid geometry, expressed in the robot's yaw-aligned frame."""

    #: Longitudinal extent, metres, relative to the base (forward positive).
    x_min: float = -0.6
    x_max: float = 1.2
    #: Lateral extent, metres.
    y_min: float = -0.6
    y_max: float = 0.6
    n_x: int = 13
    n_y: int = 9
    #: Rays start this far above the base to clear the chassis.
    ray_start_height: float = 1.5
    #: Maximum probe depth before a cell is treated as a void.
    max_depth: float = 4.0
    #: Heights are clipped to this band around the base to bound the observation.
    clip_below: float = -1.0
    clip_above: float = 1.0
    #: See ``LidarConfig.geom_group``: group 0 is the terrain, 1 and 2 are the robot and
    #: the visual track markers.
    geom_group: int | None = 0

    @property
    def n_cells(self) -> int:
        return self.n_x * self.n_y


class HeightmapSampler:
    """Samples terrain height on a yaw-aligned grid attached to the robot base.

    The grid follows the robot's heading but ignores pitch and roll, which keeps the
    observation stable while the base oscillates during a gait cycle.
    """

    def __init__(self, model, body_id: int, config: HeightmapConfig | None = None) -> None:
        self.model = model
        self.body_id = int(body_id)
        self.cfg = config or HeightmapConfig()

        xs = np.linspace(self.cfg.x_min, self.cfg.x_max, self.cfg.n_x)
        ys = np.linspace(self.cfg.y_min, self.cfg.y_max, self.cfg.n_y)
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        self._grid_local = np.stack([gx.ravel(), gy.ravel()], axis=-1)

        n = self.cfg.n_cells
        self._vec = np.tile(np.array([0.0, 0.0, -1.0]), n).astype(np.float64)
        self._dist = np.zeros(n, dtype=np.float64)
        self._geomid = np.full(n, -1, dtype=np.int32)

        if self.cfg.geom_group is None:
            self._geomgroup = None
        else:
            self._geomgroup = np.zeros(6, dtype=np.uint8)
            self._geomgroup[self.cfg.geom_group] = 1

    @staticmethod
    def _yaw_of(data, body_id: int) -> float:
        rot = data.xmat[body_id].reshape(3, 3)
        return float(np.arctan2(rot[1, 0], rot[0, 0]))

    def sample(self, data) -> np.ndarray:
        """Return terrain height relative to the base, shaped ``(n_x, n_y)``.

        Negative values mean the ground is below the base, which is the normal case.
        Cells where no geometry was found within ``max_depth`` are reported at
        ``clip_below`` so a void reads as a hole rather than as flat ground.
        """
        base_pos = data.xpos[self.body_id]
        yaw = self._yaw_of(data, self.body_id)
        c, s = np.cos(yaw), np.sin(yaw)
        rot_yaw = np.array([[c, -s], [s, c]])

        world_xy = self._grid_local @ rot_yaw.T + base_pos[:2]
        start_z = base_pos[2] + self.cfg.ray_start_height

        heights = np.empty(self.cfg.n_cells, dtype=np.float64)
        geomid = np.zeros(1, dtype=np.int32)
        for i, (x, y) in enumerate(world_xy):
            pnt = np.array([x, y, start_z], dtype=np.float64)
            dist = mujoco.mj_ray(
                m=self.model,
                d=data,
                pnt=pnt,
                vec=np.array([0.0, 0.0, -1.0]),
                geomgroup=self._geomgroup,
                flg_static=1,
                bodyexclude=self.body_id,
                geomid=geomid,
            )
            if dist < 0.0 or dist > self.cfg.max_depth:
                heights[i] = self.cfg.clip_below
            else:
                heights[i] = (start_z - dist) - base_pos[2]

        np.clip(heights, self.cfg.clip_below, self.cfg.clip_above, out=heights)
        return heights.reshape(self.cfg.n_x, self.cfg.n_y).astype(np.float32)
