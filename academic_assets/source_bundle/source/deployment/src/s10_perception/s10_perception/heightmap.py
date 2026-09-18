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

#: Overhead surfaces a single column may be stepped through before it is called a void.
#: The canopy over the y=5.5 straight is five geoms deep, so this is generous, and it is
#: bounded at all only because an unterminated loop here would stall the physics thread.
MAX_OVERHEAD_LAYERS = 12

#: How far beneath a passed-through surface the ray resumes. Small on purpose: the origin
#: then sits inside the geom just cleared, which MuJoCo will not intersect, so one step
#: clears a solid block as surely as a sheet.
OVERHEAD_SKIP = 1e-3


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


def grid_points(cfg: HeightmapConfig) -> np.ndarray:
    """Cell centres in the yaw-aligned base frame, shaped ``(n_cells, 2)``.

    Ordered x-major, matching the ``(n_x, n_y)`` layout the sampler returns.

    Module level rather than a method because the visualiser needs the same coordinates to
    place the published heights, and a second copy of this arithmetic would be free to
    drift away from the grid it claims to be drawing.
    """
    xs = np.linspace(cfg.x_min, cfg.x_max, cfg.n_x)
    ys = np.linspace(cfg.y_min, cfg.y_max, cfg.n_y)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    return np.stack([gx.ravel(), gy.ravel()], axis=-1)


class HeightmapSampler:
    """Samples terrain height on a yaw-aligned grid attached to the robot base.

    The grid follows the robot's heading but ignores pitch and roll, which keeps the
    observation stable while the base oscillates during a gait cycle.
    """

    def __init__(self, model, body_id: int, config: HeightmapConfig | None = None) -> None:
        self.model = model
        self.body_id = int(body_id)
        self.cfg = config or HeightmapConfig()

        self._grid_local = grid_points(self.cfg)

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

        Surfaces more than ``clip_above`` over the base are passed through rather than
        reported: see ``_ground_height``.
        """
        base_pos = data.xpos[self.body_id]
        yaw = self._yaw_of(data, self.body_id)
        c, s = np.cos(yaw), np.sin(yaw)
        rot_yaw = np.array([[c, -s], [s, c]])

        world_xy = self._grid_local @ rot_yaw.T + base_pos[:2]
        start_z = base_pos[2] + self.cfg.ray_start_height
        # Above this, a surface is something the robot drives under, not over.
        ceiling_z = base_pos[2] + self.cfg.clip_above

        heights = np.empty(self.cfg.n_cells, dtype=np.float64)
        geomid = np.zeros(1, dtype=np.int32)
        for i, (x, y) in enumerate(world_xy):
            hit_z = self._ground_height(data, x, y, start_z, ceiling_z, geomid)
            if hit_z is None:
                heights[i] = self.cfg.clip_below
            else:
                heights[i] = hit_z - base_pos[2]

        np.clip(heights, self.cfg.clip_below, self.cfg.clip_above, out=heights)
        return heights.reshape(self.cfg.n_x, self.cfg.n_y).astype(np.float32)

    def _ground_height(self, data, x, y, start_z, ceiling_z, geomid) -> float | None:
        """World height of the first surface in this column that is terrain, or None.

        A single downward ray cannot tell ground from a roof over it, and the course has
        roofs: over the straight at y=5.5 the column holds five overlapping geoms between
        2.08 and 2.37 m, with the real ground 1.6 m beneath at 0.479. Reporting the highest
        hit put the whole 13x9 grid at the clip ceiling, relief 1.42 on ground the base
        itself called flat, and cost sixteen seconds of crawling at the terrain scaler's
        0.25 floor -- under a canopy the robot passes below without touching.

        So hits above ``ceiling_z`` are stepped through rather than returned. Resuming just
        beneath a surface puts the ray origin inside that geom, which MuJoCo declines to
        intersect, so a solid obstacle is skipped as readily as a thin canopy. That is the
        intended division of labour and not an oversight: this grid exists to describe the
        terrain underfoot, and things standing at body height are the lidar's business --
        which is why the lidar reported a clear 1.00 through the whole crawl.
        """
        z = start_z
        # Bounded because a ray that never terminates would stall the physics loop. Twelve
        # is generous against the five-deep canopy that motivated this.
        for _ in range(MAX_OVERHEAD_LAYERS):
            dist = mujoco.mj_ray(
                m=self.model,
                d=data,
                pnt=np.array([x, y, z], dtype=np.float64),
                vec=np.array([0.0, 0.0, -1.0]),
                geomgroup=self._geomgroup,
                flg_static=1,
                bodyexclude=self.body_id,
                geomid=geomid,
            )
            if dist < 0.0:
                return None
            hit_z = z - dist
            # Depth is measured from the original start so overhead layers cannot extend
            # the reach of the sampler down a genuinely deep hole.
            if start_z - hit_z > self.cfg.max_depth:
                return None
            if hit_z <= ceiling_z:
                return hit_z
            z = hit_z - OVERHEAD_SKIP
        return None
