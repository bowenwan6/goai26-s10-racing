"""Explicit frame conversion and conservative, single-scan elevation projection.

No inferred extrinsics, hole filling, motion compensation or terrain certification.
All distances are metres; quaternions use ROS xyzw. Unknown cells stay unknown.
"""

from __future__ import annotations

import math

import numpy as np

GRID_SHAPE = (13, 9)
GRID_X = np.linspace(-0.6, 1.2, 13)
GRID_Y = np.linspace(-0.6, 0.6, 9)


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"expected finite vector of size {size}")
    return result


def quaternion_matrix(xyzw):
    q = vector(xyzw, 4)
    norm = float(np.linalg.norm(q))
    if abs(norm - 1.0) > 0.01:
        raise ValueError("quaternion is not unit length")
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rigid_transform(value):
    t = np.asarray(value, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError("explicit finite 4x4 transform required")
    r = t[:3, :3]
    if (
        not np.allclose(t[3], [0, 0, 0, 1], atol=1e-7)
        or not np.allclose(r.T @ r, np.eye(3), atol=1e-6)
        or not math.isclose(float(np.linalg.det(r)), 1.0, abs_tol=1e-6)
    ):
        raise ValueError("transform must be rigid and right handed")
    return t


def pose_matrix(position, orientation):
    t = np.eye(4)
    t[:3, :3] = quaternion_matrix(orientation)
    t[:3, 3] = vector(position, 3)
    return t


def base_pose(raw_pose, odom_child_from_base):
    """map_T_base = map_T_odom_child @ odom_child_T_base."""
    t = pose_matrix(raw_pose["position"], raw_pose["orientation"])
    return t @ rigid_transform(odom_child_from_base)


def yaw_of(transform):
    return math.atan2(transform[1, 0], transform[0, 0])


def points_in_yaw_frame(points, base_from_cloud, map_from_base):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("cloud must be Nx3")
    if len(points) > 1_000_000:
        raise ValueError("cloud exceeds diagnostic size limit")
    t = rigid_transform(base_from_cloud)
    mb = rigid_transform(map_from_base)
    points = points[np.isfinite(points).all(axis=1)]
    # First sensor -> full body; then full body -> gravity-aligned world axes;
    # finally remove heading only. Do not use just yaw for a pitched sensor/body.
    body = points @ t[:3, :3].T + t[:3, 3]
    world_delta = body @ mb[:3, :3].T
    yaw = yaw_of(mb)
    c, s = math.cos(yaw), math.sin(yaw)
    world_from_yaw = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return world_delta @ world_from_yaw


def height_grid(points, min_points=3, max_spread=0.08):
    """Return values/mask/counts; reject mixed levels and clipped deep cells.

    Topmost return is conservative against a small raised obstacle. A vertical face
    or two storeys in a cell is ambiguous, never averaged into drivable ground.
    This is NOT a complete elevation mapper: self-filtering/deskew/accumulation
    must be independently validated before hardware use.
    """
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3:
        raise ValueError("cloud must be Nx3")
    if not isinstance(min_points, int) or min_points < 1 or not 0 < max_spread < 1:
        raise ValueError("invalid grid quality limits")
    p = p[np.isfinite(p).all(axis=1)]
    # Outside the sampled local box, not evidence about any of its cells.
    p = p[(p[:, 0] >= -0.675) & (p[:, 0] < 1.275) & (p[:, 1] >= -0.675) & (p[:, 1] < 0.675)]
    ix = np.clip(np.floor((p[:, 0] + 0.675) / 0.15 + 1e-10).astype(int), 0, 12)
    iy = np.clip(np.floor((p[:, 1] + 0.675) / 0.15 + 1e-10).astype(int), 0, 8)
    ids = ix * 9 + iy
    values = np.full(117, -1.0)
    valid = np.zeros(117, dtype=bool)
    counts = np.bincount(ids, minlength=117)[:117]
    low = np.full(117, np.inf)
    high = np.full(117, -np.inf)
    np.minimum.at(low, ids, p[:, 2])
    np.maximum.at(high, ids, p[:, 2])
    observed = counts >= min_points
    valid[observed] = (
        (high[observed] - low[observed] <= max_spread)
        & (low[observed] > -1.0)
        & (high[observed] < 1.0)
    )
    values[valid] = high[valid]
    return values.reshape(GRID_SHAPE), valid.reshape(GRID_SHAPE), counts.reshape(GRID_SHAPE)


def decode_pointcloud2(msg):
    """Duck-typed ROS PointCloud2 decoding, including row padding and endianness."""
    w, h, step, row = int(msg.width), int(msg.height), int(msg.point_step), int(msg.row_step)
    if min(w, h, step) <= 0 or w * h > 1_000_000 or row < w * step:
        raise ValueError("invalid cloud dimensions")
    if len(msg.data) < h * row:
        raise ValueError("truncated cloud buffer")
    fields = {f.name: f for f in msg.fields}
    output = []
    for name in ("x", "y", "z"):
        f = fields.get(name)
        if f is None or f.datatype not in (7, 8) or f.count != 1:
            raise ValueError("cloud requires scalar float x/y/z")
        dtype = np.dtype((">" if msg.is_bigendian else "<") + ("f4" if f.datatype == 7 else "f8"))
        if f.offset < 0 or f.offset + dtype.itemsize > step:
            raise ValueError("invalid field offset")
        data = np.ndarray(
            (h, w), dtype=dtype, buffer=memoryview(msg.data), offset=f.offset, strides=(row, step)
        )
        output.append(data.reshape(-1))
    return np.column_stack(output).astype(float)
