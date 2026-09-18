"""Read-only loaders for the v3 map inputs (PCD, keyframe poses) and shared paths."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

MAP_ID = "0914_fr_v3-20260914-142008"
GOAI = Path(os.environ.get("GOAI_ROOT", "/Users/xxxwbwxxx/Documents/ChatGPT/GOAI"))
RAW = GOAI / "map-reviews" / MAP_ID / "raw" / MAP_ID
PCD_PATH = RAW / "full_cloud.pcd"
POSES_PATH = RAW / ".sessions" / "session_0" / "poses.txt"
ARTIFACTS = Path(os.environ.get("NAV_SIM_ARTIFACTS", str(GOAI / "wt-nav-sim-artifacts")))
REPO = Path(__file__).resolve().parents[1]
PKG = Path(__file__).resolve().parent

# Photo-matched route committed in this repo; the harness falls back to the placeholder.
REAL_ROUTE = Path(os.environ.get(
    "NAV_SIM_ROUTE", str(REPO / "tools" / "wp_match" / "out" / "route_v2.json")))

# Official course (reverse of photo/mapping time order), see route_v2 contract.
COURSE_EPOCH = (1789368238.0, 1789368895.0)


def load_pcd_xyz(path: Path | str = PCD_PATH) -> np.ndarray:
    """Binary PCD v0.7 with float32 fields; returns Nx3 float64 (map frame, m)."""
    raw = Path(path).read_bytes()
    marker = b"DATA binary\n"
    start = raw.index(marker) + len(marker)
    header = raw[:start].decode("ascii", "replace").splitlines()
    fields = next(line.split()[1:] for line in header if line.startswith("FIELDS"))
    sizes = [int(v) for v in next(line.split()[1:] for line in header if line.startswith("SIZE"))]
    types = next(line.split()[1:] for line in header if line.startswith("TYPE"))
    if set(sizes) != {4} or set(types) != {"F"}:
        raise ValueError("only float32 binary PCD supported")
    n = int(next(line.split()[1] for line in header if line.startswith("POINTS")))
    data = np.frombuffer(raw, dtype="<f4", count=n * len(fields), offset=start)
    data = data.reshape(n, len(fields))
    idx = [fields.index(k) for k in ("x", "y", "z")]
    xyz = data[:, idx].astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def load_poses(path: Path | str = POSES_PATH) -> np.ndarray:
    """Columns: t x y z qx qy qz qw (base reference, map frame)."""
    return np.loadtxt(path)


def quat_yaw(qx, qy, qz, qw):
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
