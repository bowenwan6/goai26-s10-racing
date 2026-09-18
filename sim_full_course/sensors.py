"""Synthetic LiDAR cloud from the terrain -> the REAL perception contracts.

Pipeline per tick (mirrors native_transfer/runtime.py::on_cloud):
  1. ray-cast a sensor-frame cloud from the TRUE pose against terrain surface (ground,
     static obstacle layer, injected shapes); hits on unknown cells return nothing;
     random dropout + range noise;
  2. points_in_yaw_frame(cloud, base_from_cloud, map_from_base_ESTIMATE)  (real code)
  3. height_grid(points) -> 13x9 values/valid/counts                      (real code)
  4. conservative_scan(points) -> 72 bins                                   (real code)
If the real modules cannot be imported, byte-for-byte copies below are used and
`REAL_CONTRACTS` is False (tests assert equality whenever the real ones import).

Ray sets: a near-field set aimed at uniformly sampled ground targets inside the height-grid
box (proxy for the sensor's near coverage; density configurable), and a ring set (azimuth x
elevation) out to 10 m for the scan. Occlusion comes from marching each ray through the
2.5D surface. Self-occlusion by the body is optional (off by default: the real mount's
near-body coverage is unverified).
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field

import numpy as np

from sim_full_course.io_utils import REPO
from sim_full_course.terrain import Terrain

for _p in (str(REPO), str(REPO / "src" / "s10_auto_nav")):
    if _p not in sys.path:
        sys.path.append(_p)

try:  # real contracts
    from real_transfer.geometry import height_grid, points_in_yaw_frame  # noqa: F401

    try:
        from native_transfer.contracts import conservative_scan
    except Exception:  # pragma: no cover - s10_auto_nav unavailable
        from sim_full_course._contract_copies import conservative_scan
    REAL_CONTRACTS = True
except Exception:  # pragma: no cover
    from sim_full_course._contract_copies import (  # noqa: F401
        conservative_scan,
        height_grid,
        points_in_yaw_frame,
    )

    REAL_CONTRACTS = False

GRID_BOX_X = (-0.675, 1.275)
GRID_BOX_Y = (-0.675, 0.675)


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def pose_to_matrix(x, y, z, roll, pitch, yaw):
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(roll, pitch, yaw)
    t[:3, 3] = [x, y, z]
    return t


def default_extrinsic():
    """base_T_cloud: sensor 0.20 m ahead, 0.12 m above the base origin, level (assumed)."""
    t = np.eye(4)
    t[:3, 3] = [0.20, 0.0, 0.12]
    return t


@dataclass
class SensorConfig:
    near_density: float = 450.0  # target returns / m^2 in the height-grid box
    ring_az_step_deg: float = 1.0
    ring_elev_deg: tuple = (-30.0, -20.0, -14.0, -9.0, -5.0, -2.0, 0.0, 2.0, 5.0, 10.0)
    max_range: float = 10.0
    ds: float = 0.04  # ray-march step (m)
    dropout: float = 0.10
    range_noise: float = 0.01
    self_occlusion: bool = False
    base_from_cloud: np.ndarray = field(default_factory=default_extrinsic)
    pose_noise_xy: float = 0.0
    pose_noise_yaw: float = 0.0
    pose_noise_z: float = 0.0


def _schedule(max_t: float, ds: float, grow: float = 0.02) -> np.ndarray:
    """Sample distances: ds up to ds/grow, then geometric growth (step = grow * t)."""
    ts, t = [], 0.0
    while t < max_t:
        t += max(ds, grow * t)
        ts.append(min(t, max_t))
    return np.array(ts)


def raycast(terrain: Terrain, origin: np.ndarray, dirs: np.ndarray, max_t, ds: float,
            z_ceiling: float | None = None):
    """March rays (origin (3,), dirs (N,3) unit) through the 2.5D surface.

    Step ds near the sensor, growing to 2 % of range further out (>= 0.2 m at 10 m).
    Rays climbing above `z_ceiling` (max surface in range) stop early. Returns t_hit (N,),
    NaN where nothing is hit within max_t; bisection-refined (3 halvings)."""
    n = len(dirs)
    max_t = np.broadcast_to(np.asarray(max_t, float), (n,))
    t_hit = np.full(n, np.nan)
    active = np.ones(n, bool)
    t_prev = np.zeros(n)
    for tk in _schedule(float(max_t.max()), ds):
        idx = np.flatnonzero(active)
        if not len(idx):
            break
        t = np.minimum(tk, max_t[idx])
        p = origin + dirs[idx] * t[:, None]
        below = p[:, 2] <= terrain.surface_at(p[:, 0], p[:, 1])
        hit = idx[below]
        if len(hit):
            lo, hi = t_prev[hit], t[below]
            for _ in range(3):
                mid = 0.5 * (lo + hi)
                q = origin + dirs[hit] * mid[:, None]
                b = q[:, 2] <= terrain.surface_at(q[:, 0], q[:, 1])
                hi = np.where(b, mid, hi)
                lo = np.where(b, lo, mid)
            t_hit[hit] = hi
            active[hit] = False
        gone = (~below) & (tk >= max_t[idx])
        if z_ceiling is not None:
            gone |= (~below) & (dirs[idx, 2] >= 0) & (p[:, 2] > z_ceiling)
        active[idx[gone]] = False
        t_prev[idx] = t
    return t_hit


class SensorModel:
    def __init__(self, terrain: Terrain, config: SensorConfig | None = None, seed: int = 0):
        self.terrain = terrain
        self.cfg = config or SensorConfig()
        self.rng = np.random.default_rng(seed)
        az = np.deg2rad(np.arange(-180, 180, self.cfg.ring_az_step_deg))
        el = np.deg2rad(np.asarray(self.cfg.ring_elev_deg))
        aa, ee = np.meshgrid(az, el)
        self._ring = np.column_stack([
            np.cos(ee.ravel()) * np.cos(aa.ravel()),
            np.cos(ee.ravel()) * np.sin(aa.ravel()),
            np.sin(ee.ravel()),
        ])

    # ---- cloud generation (sensor frame) ----
    def cloud(self, true_pose6) -> np.ndarray:
        """true_pose6 = (x, y, z, roll, pitch, yaw) of the base; returns sensor-frame Nx3."""
        cfg = self.cfg
        mb = pose_to_matrix(*true_pose6)
        ms = mb @ cfg.base_from_cloud
        o = ms[:3, 3]
        x, y, _, _, _, yaw = true_pose6
        c, s = math.cos(yaw), math.sin(yaw)
        # Near-field targets on the surface inside the height-grid box (yaw frame -> map).
        area = (GRID_BOX_X[1] - GRID_BOX_X[0]) * (GRID_BOX_Y[1] - GRID_BOX_Y[0])
        n = self.rng.poisson(cfg.near_density * area)
        u = self.rng.uniform(*GRID_BOX_X, n)
        v = self.rng.uniform(*GRID_BOX_Y, n)
        tx, ty = x + c * u - s * v, y + s * u + c * v
        tz = self.terrain.surface_at(tx, ty)
        d_near = np.column_stack([tx, ty, tz]) - o
        rng_near = np.linalg.norm(d_near, axis=1)
        d_near /= np.maximum(rng_near, 1e-9)[:, None]
        d_ring = self._ring @ ms[:3, :3].T
        dirs = np.vstack([d_near, d_ring])
        max_t = np.r_[rng_near + 0.3, np.full(len(d_ring), cfg.max_range + 0.3)]
        r = cfg.max_range + 0.5
        iy0, ix0, _ = self.terrain.cell(x - r, y - r)
        iy1, ix1, _ = self.terrain.cell(x + r, y + r)
        ceiling = float(np.max(self.terrain._obst_top[iy0:iy1 + 1, ix0:ix1 + 1]))
        if self.terrain.injected:
            ceiling = max(ceiling, max(ob.base_z + ob.height for ob in self.terrain.injected))
        t_hit = raycast(self.terrain, o, dirs, max_t, cfg.ds, z_ceiling=ceiling + 0.05)
        ok = np.isfinite(t_hit)
        pts = o + dirs[ok] * t_hit[ok, None]
        # Unknown terrain never returns (unless it is an injected obstacle, which is real).
        inj = self.terrain.injected_top(pts[:, 0], pts[:, 1]) >= pts[:, 2] - 1e-6 if \
            self.terrain.injected else np.zeros(len(pts), bool)
        keep = self.terrain.known_at(pts[:, 0], pts[:, 1]) | inj
        keep &= self.rng.random(len(pts)) >= cfg.dropout
        pts = pts[keep]
        if cfg.self_occlusion:
            local = (pts - mb[:3, 3]) @ mb[:3, :3]
            ol = (o - mb[:3, 3]) @ mb[:3, :3]
            pts = pts[~_segment_hits_box(ol, local, (-0.45, 0.45), (-0.25, 0.25), (-0.12, 0.08))]
        # Range noise along the ray, then express in the sensor frame.
        d = pts - o
        r = np.linalg.norm(d, axis=1, keepdims=True)
        pts = o + d * (1 + self.rng.normal(0, cfg.range_noise, (len(d), 1)) / np.maximum(r, 1e-6))
        return (pts - ms[:3, 3]) @ ms[:3, :3]

    def estimated_pose(self, true_pose6):
        cfg = self.cfg
        x, y, z, r, p, yaw = true_pose6
        return (
            x + self.rng.normal(0, cfg.pose_noise_xy) if cfg.pose_noise_xy else x,
            y + self.rng.normal(0, cfg.pose_noise_xy) if cfg.pose_noise_xy else y,
            z + self.rng.normal(0, cfg.pose_noise_z) if cfg.pose_noise_z else z,
            r,
            p,
            yaw + self.rng.normal(0, cfg.pose_noise_yaw) if cfg.pose_noise_yaw else yaw,
        )

    def observe(self, true_pose6, est_pose6=None):
        """-> dict(grid, valid, counts, scan, points_yaw, n_points)."""
        est = est_pose6 if est_pose6 is not None else true_pose6
        cloud = self.cloud(true_pose6)
        pts = points_in_yaw_frame(cloud, self.cfg.base_from_cloud, pose_to_matrix(*est))
        grid, valid, counts = height_grid(pts)
        scan = conservative_scan(pts)
        return {"grid": grid, "valid": valid, "counts": counts, "scan": scan,
                "points_yaw": pts, "n_points": len(pts)}


def _segment_hits_box(o, p, bx, by, bz):
    """Slab test: does segment o->p (local frame) cross the axis-aligned box?"""
    d = p - o
    tmin = np.zeros(len(p))
    tmax = np.ones(len(p)) * 0.999
    for k, (lo, hi) in enumerate((bx, by, bz)):
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (lo - o[k]) / d[:, k]
            t2 = (hi - o[k]) / d[:, k]
        par = np.abs(d[:, k]) < 1e-12
        inside = (o[k] >= lo) & (o[k] <= hi)
        t1 = np.where(par, np.where(inside, -np.inf, np.inf), t1)
        t2 = np.where(par, np.where(inside, np.inf, -np.inf), t2)
        tmin = np.maximum(tmin, np.minimum(t1, t2))
        tmax = np.minimum(tmax, np.maximum(t1, t2))
    return tmin <= tmax
