"""Ray-cast lidar for MuJoCo.

The contest MJCF ships with no exteroceptive sensor, so we synthesise one by casting
rays directly against the collision geometry with ``mj_multiRay``. This is roughly two
orders of magnitude cheaper than rendering a depth camera and maps cleanly onto the
physical Airy lidar carried by the S10.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np


@dataclass
class LidarConfig:
    """Geometry of the synthetic lidar.

    Defaults approximate the Airy unit mounted on the S10: a wide horizontal sweep with
    a modest vertical fan, sampled at policy rate rather than the sensor's native rate.
    """

    n_azimuth: int = 64
    n_elevation: int = 8
    fov_azimuth: float = 2 * np.pi
    elevation_min: float = np.deg2rad(-30.0)
    elevation_max: float = np.deg2rad(15.0)
    range_max: float = 12.0
    range_min: float = 0.05
    #: Sensor origin expressed in the base body frame (metres).
    mount_offset: np.ndarray = field(default_factory=lambda: np.array([0.20, 0.0, 0.12]))
    #: Only cast against this MuJoCo geom group; ``None`` casts against all groups.
    #:
    #: Group 0 is the world: the floor plus the 2185 environment meshes. Group 1 is the
    #: robot's own collision geoms and group 2 holds both its visual meshes and the 65
    #: non-colliding track markers -- casting against either would return the robot's own
    #: body, or phantom hits on the waypoint spheres we are supposed to be racing towards.
    geom_group: int | None = 0
    #: Exclude the robot's own body so the chassis does not occlude every ray.
    exclude_self: bool = True

    @property
    def n_rays(self) -> int:
        return self.n_azimuth * self.n_elevation


class RayCastLidar:
    """Casts a fixed ray pattern from a body-mounted origin every time it is sampled.

    The ray pattern is precomputed once in the sensor frame and rotated into the world
    frame on each call, so the per-step cost is one matrix multiply plus the MuJoCo ray
    query itself.
    """

    def __init__(self, model, body_id: int, config: LidarConfig | None = None) -> None:
        self.model = model
        self.body_id = int(body_id)
        self.cfg = config or LidarConfig()

        self._directions_local = self._build_ray_pattern(self.cfg)
        n = self.cfg.n_rays

        # Preallocated buffers reused across calls; mj_multiRay writes into them in place.
        self._vec_world = np.zeros(3 * n, dtype=np.float64)
        self._dist = np.zeros(n, dtype=np.float64)
        self._geomid = np.full(n, -1, dtype=np.int32)

        if self.cfg.geom_group is None:
            self._geomgroup = None
        else:
            self._geomgroup = np.zeros(6, dtype=np.uint8)
            self._geomgroup[self.cfg.geom_group] = 1

        self._bodyexclude = self.body_id if self.cfg.exclude_self else -1

    @staticmethod
    def _build_ray_pattern(cfg: LidarConfig) -> np.ndarray:
        """Return unit ray directions in the sensor frame, shaped ``(n_rays, 3)``.

        Rays are ordered elevation-major so that reshaping to ``(n_elevation, n_azimuth)``
        yields a range image with rows of constant pitch.
        """
        azimuth = np.linspace(
            -cfg.fov_azimuth / 2.0,
            cfg.fov_azimuth / 2.0,
            cfg.n_azimuth,
            endpoint=not np.isclose(cfg.fov_azimuth, 2 * np.pi),
        )
        elevation = np.linspace(cfg.elevation_min, cfg.elevation_max, cfg.n_elevation)

        el, az = np.meshgrid(elevation, azimuth, indexing="ij")
        cos_el = np.cos(el)
        directions = np.stack(
            [cos_el * np.cos(az), cos_el * np.sin(az), np.sin(el)],
            axis=-1,
        ).reshape(-1, 3)
        return np.ascontiguousarray(directions, dtype=np.float64)

    def origin(self, data) -> np.ndarray:
        """World-frame position of the sensor."""
        rot = data.xmat[self.body_id].reshape(3, 3)
        return data.xpos[self.body_id] + rot @ self.cfg.mount_offset

    def scan(self, data) -> np.ndarray:
        """Sample the lidar.

        Returns ranges in metres, shaped ``(n_elevation, n_azimuth)``. Rays that hit
        nothing are clamped to ``range_max`` rather than left as MuJoCo's ``-1`` sentinel,
        so the result is directly usable as a policy observation.
        """
        rot = data.xmat[self.body_id].reshape(3, 3)
        pnt = np.ascontiguousarray(self.origin(data), dtype=np.float64)

        # Sensor frame -> world frame. Directions stay unit length under rotation.
        world = self._directions_local @ rot.T
        self._vec_world[:] = world.reshape(-1)

        # ``normal`` is required by the binding but unused here; passing None skips the
        # extra per-ray normal computation.
        mujoco.mj_multiRay(
            m=self.model,
            d=data,
            pnt=pnt,
            vec=self._vec_world,
            geomgroup=self._geomgroup,
            flg_static=True,
            bodyexclude=self._bodyexclude,
            geomid=self._geomid,
            dist=self._dist,
            normal=None,
            nray=self.cfg.n_rays,
            cutoff=self.cfg.range_max,
        )

        ranges = np.where(self._dist < 0.0, self.cfg.range_max, self._dist)
        np.clip(ranges, self.cfg.range_min, self.cfg.range_max, out=ranges)
        return ranges.reshape(self.cfg.n_elevation, self.cfg.n_azimuth).astype(np.float32)

    def horizontal_ring(self, ranges: np.ndarray) -> np.ndarray:
        """Extract the elevation row closest to horizontal, for 2D visualisation."""
        elevations = np.linspace(
            self.cfg.elevation_min, self.cfg.elevation_max, self.cfg.n_elevation
        )
        return ranges[int(np.argmin(np.abs(elevations)))]

    @property
    def azimuth_angles(self) -> np.ndarray:
        return np.linspace(
            -self.cfg.fov_azimuth / 2.0,
            self.cfg.fov_azimuth / 2.0,
            self.cfg.n_azimuth,
            endpoint=not np.isclose(self.cfg.fov_azimuth, 2 * np.pi),
        )
