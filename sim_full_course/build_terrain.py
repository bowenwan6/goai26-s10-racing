"""Extract a 2.5D ground heightfield + obstacle layer along the mapping trajectory.

Input: v3 full_cloud.pcd + optimized keyframe poses (read-only). Output (not committed,
> 5 MB): ARTIFACTS/terrain/course_terrain.npz and PNG previews.

Per 0.10 m cell (row = y, col = x; origin = map XY of the lower-left corner of cell [0,0]):
  ground      top dense return of the lowest dense vertical cluster, capped at 0.30 m above
              its lowest dense return (support = >= 5 returns of the 3x3 neighbourhood within
              +-0.05 m); mean of the cell's returns within +-0.05 m of it. The v3 cloud has a
              layered multi-pass smear 0.05-0.4 m below the driven surface.
  known       ground was observed with enough density and passed the consistency checks.
  obstacle    >= 2 returns between (max ground of 3x3 neighbours + 0.10) and ground + 1.20.
              Cleared inside the driven footprint (0.9 x 0.5 m body at every keyframe pose):
              those returns are the operator / people / self, the robot drove there.
  multi_level a vertical gap >= 0.25 m followed by another dense layer 0.3..2.0 m above the
              ground (bridge / table / branch / overhang); or the lowest layer disagrees with
              the robot's own driven height (then the layer the robot drove on is used).
Points > 3.5 m above the nearest keyframe base (canopy, roofs) are discarded first.
Unknown cells are never free space: `ground_filled` exists only so a heightfield has a value.
"""

from __future__ import annotations

import argparse
import json
import time

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

from sim_full_course.io_utils import (
    ARTIFACTS,
    COURSE_EPOCH,
    MAP_ID,
    load_pcd_xyz,
    load_poses,
)

RES = 0.10
HALF_WIDTH = 5.0  # corridor half-width around the driven trajectory (10 m corridor)
MIN_PTS = 4  # raw returns for 'dense_unknown'
MIN_SUPPORT = 5
SUPPORT_DZ = 0.05
CLUSTER_GAP = 0.15
DESPIKE = 0.10
STACK = 0.30
LAYER = 0.10
BODY_Z = 0.43  # keyframe base reference above ground on flat terrain
CANOPY_CUT = 3.5
OBST_LO, OBST_HI = 0.10, 1.20
GAP = 0.25
ML_LO, ML_HI = 0.30, 2.0


def densify(xyz: np.ndarray, step: float = 0.2) -> np.ndarray:
    out = [xyz[:1]]
    for a, b in zip(xyz[:-1], xyz[1:], strict=True):
        n = max(1, int(np.ceil(np.linalg.norm(b[:2] - a[:2]) / step)))
        f = (np.arange(1, n + 1) / n)[:, None]
        out.append(a + f * (b - a))
    return np.vstack(out)


def lowest_dense_layer(cell, z, n_cells, nx):
    """cell/z sorted by (cell, z). A return is 'dense' when >= MIN_SUPPORT returns of the
    3x3 cell neighbourhood lie within +-SUPPORT_DZ of it (the v3 cloud is voxel-thinned to
    ~0.07 m, so one 0.10 m cell alone holds only ~3 ground returns). Ground = mean of the
    cell's own returns in [z0, z0 + LAYER] above the lowest dense return z0."""
    zmin = z.min()
    iy, ix = np.divmod(cell, nx)
    ny = n_cells // nx
    nb_cell, nb_z = [], []
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            jy, jx = iy + dy, ix + dx
            ok = (jy >= 0) & (jy < ny) & (jx >= 0) & (jx < nx)
            nb_cell.append((jy * nx + jx)[ok])
            nb_z.append(z[ok])
    nb_cell = np.concatenate(nb_cell)
    nb_z = np.concatenate(nb_z)
    o = np.lexsort((nb_z, nb_cell))
    key_all = nb_cell[o].astype(np.float64) * 64.0 + (nb_z[o] - zmin)
    key = cell.astype(np.float64) * 64.0 + (z - zmin)
    support = np.searchsorted(key_all, key + SUPPORT_DZ, "right") - np.searchsorted(
        key_all, key - SUPPORT_DZ, "left"
    )
    dense = support >= MIN_SUPPORT
    n = len(z)
    idx = np.arange(n)
    # Lowest dense return z0 per cell.
    di = np.flatnonzero(dense)
    cells, pos = np.unique(cell[di], return_index=True)
    z0 = np.full(n_cells, np.nan)
    z0[cells] = z[di[pos]]
    # Vertical clusters (gap >= CLUSTER_GAP starts a new one). The v3 cloud has a sparse
    # sub-surface / multi-pass smear 0.1-0.4 m below the driven surface, so the surface is the
    # TOP dense return of the lowest dense cluster (within z0 + STACK), not z0 itself: the
    # smear consists of 0.05-0.1 m spaced pass layers under the surface the robot drove on.
    # Clusters are cut on the 3x3 neighbourhood multiset (own cells hold only ~3 returns).
    nb_sorted_cell = nb_cell[o]
    brk = np.r_[True, (nb_sorted_cell[1:] != nb_sorted_cell[:-1]) | (np.diff(key_all) >= CLUSTER_GAP)]
    clu_all = np.cumsum(brk)
    clu = clu_all[np.searchsorted(key_all, key, "left")]
    clu0 = np.full(n_cells, -1)
    clu0[cells] = clu[di[pos]]
    cand = dense & (clu == clu0[cell]) & (z <= z0[cell] + STACK)
    ci = np.flatnonzero(cand)
    o = np.lexsort((z[ci], cell[ci]))
    ci = ci[o]
    last = np.r_[cell[ci][1:] != cell[ci][:-1], True]
    best = ci[last]
    lo = np.searchsorted(key, key[best] - SUPPORT_DZ, "left")
    hi = np.searchsorted(key, key[best] + SUPPORT_DZ, "right")
    csum = np.concatenate([[0.0], np.cumsum(z)])
    ground = np.full(n_cells, np.nan)
    gcount = np.zeros(n_cells, dtype=np.int32)
    ground[cell[best]] = (csum[hi] - csum[lo]) / (hi - lo)
    gcount[cell[best]] = hi - lo
    hi2 = np.searchsorted(key, key + SUPPORT_DZ, "right")
    lo2 = np.searchsorted(key, key - SUPPORT_DZ, "left")
    mean_win = (csum[hi2] - csum[lo2]) / (hi2 - lo2)
    del idx
    return ground, gcount, dense, mean_win


def _nanmedian5(G, known):
    ny, nx = G.shape
    pad = np.pad(np.where(known, G, np.nan), 2, constant_values=np.nan)
    stack = np.stack([pad[dy : dy + ny, dx : dx + nx] for dy in range(5) for dx in range(5)])
    with np.errstate(all="ignore"):
        import warnings

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            return np.nanmedian(stack, 0)


def fill_small_holes(G, known, max_range=0.06, min_known=6):
    """Unknown cell with >= min_known of 8 known neighbours spanning <= max_range m."""
    ny, nx = G.shape
    pad = np.pad(np.where(known, G, np.nan), 1, constant_values=np.nan)
    stack = np.stack([pad[1 + dy : 1 + dy + ny, 1 + dx : 1 + dx + nx]
                      for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx])
    n = np.isfinite(stack).sum(0)
    import warnings

    with np.errstate(all="ignore"), warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        rng = np.nanmax(stack, 0) - np.nanmin(stack, 0)
        med = np.nanmedian(stack, 0)
    fill = (~known) & (n >= min_known) & (rng <= max_range)
    return fill, med


def build(half_width=HALF_WIDTH, res=RES, out_dir=None, verbose=True):
    t0 = time.time()
    out_dir = out_dir or (ARTIFACTS / "terrain")
    out_dir.mkdir(parents=True, exist_ok=True)
    pts = load_pcd_xyz()
    poses = load_poses()
    traj = densify(poses[:, 1:4])
    tree = cKDTree(traj[:, :2])
    d, nn = tree.query(pts[:, :2], k=1, distance_upper_bound=half_width)
    keep = np.isfinite(d)
    pts, nn = pts[keep], nn[keep]
    below_canopy = pts[:, 2] <= traj[nn, 2] + CANOPY_CUT
    n_canopy = int((~below_canopy).sum())
    pts = pts[below_canopy]
    if verbose:
        print(f"corridor points {len(pts):,} (canopy/roof removed {n_canopy:,})")

    x0 = np.floor((pts[:, 0].min()) / res) * res
    y0 = np.floor((pts[:, 1].min()) / res) * res
    nx = int(np.ceil((pts[:, 0].max() - x0) / res)) + 1
    ny = int(np.ceil((pts[:, 1].max() - y0) / res)) + 1
    ix = np.floor((pts[:, 0] - x0) / res).astype(np.int64)
    iy = np.floor((pts[:, 1] - y0) / res).astype(np.int64)
    cell = iy * nx + ix
    order = np.lexsort((pts[:, 2], cell))
    cell, z = cell[order], pts[order, 2]
    n_cells = nx * ny
    ground, gcount, dense, mean_win = lowest_dense_layer(cell, z, n_cells, nx)
    raw_count = np.bincount(cell, minlength=n_cells)

    # Multi-level: dense layer starting after a vertical gap, 0.3..2.0 m above ground.
    prev_same = np.r_[False, cell[1:] == cell[:-1]]
    gap_before = np.r_[False, np.diff(z) >= GAP] & prev_same
    rel = z - ground[cell]
    ml_pt = dense & gap_before & (rel >= ML_LO) & (rel <= ML_HI)
    multi = np.zeros(n_cells, bool)
    multi[np.unique(cell[ml_pt])] = True

    # Driven-surface consistency: the robot physically stood on ground = base - BODY_Z.
    cx = x0 + (np.arange(nx) + 0.5) * res
    cy = y0 + (np.arange(ny) + 0.5) * res
    gx, gy = np.meshgrid(cx, cy)
    dist_traj, nn_c = tree.query(np.column_stack([gx.ravel(), gy.ravel()]), k=1)
    expected = traj[nn_c, 2] - BODY_Z
    near = dist_traj <= 0.30
    mismatch = near & np.isfinite(ground) & (np.abs(ground - expected) > 0.30)
    # If a dense layer exists near the driven height, use it (the robot was on it).
    driven_ok = dense & (np.abs(mean_win - expected[cell]) <= 0.15) & near[cell]
    fixed_cells = np.zeros(n_cells, bool)
    if driven_ok.any():
        fc = cell[driven_ok & mismatch[cell]]
        fz = mean_win[driven_ok & mismatch[cell]]
        if len(fc):
            u, first = np.unique(fc, return_index=True)
            ground[u] = fz[first]
            fixed_cells[u] = True
    multi |= mismatch

    G = ground.reshape(ny, nx)
    corridor = (dist_traj <= half_width).reshape(ny, nx)
    # Implausibly high lowest layer vs. local lower envelope: canopy/roof without ground.
    env = ndimage.minimum_filter(np.where(np.isfinite(G), G, np.inf), size=11)
    high = np.isfinite(G) & np.isfinite(env) & (G > env + 1.5)
    G[high] = np.nan
    known = np.isfinite(G) & corridor
    G[~known] = np.nan
    # Isolated low spikes from the sub-surface smear: > DESPIKE below the 5x5 median of known
    # cells. Straight stair edges keep their majority side, so risers survive.
    med5 = _nanmedian5(G, known)
    despiked = known & np.isfinite(med5) & (G < med5 - DESPIKE)
    G[despiked] = med5[despiked]
    interpolated, med = fill_small_holes(G, known)
    interpolated &= corridor
    G[interpolated] = med[interpolated]
    known |= interpolated

    # Obstacle layer relative to the max ground in the 3x3 neighbourhood.
    gmax = ndimage.maximum_filter(np.where(known, G, -np.inf), size=3).ravel()
    gflat = G.ravel()
    lo = gmax[cell] + OBST_LO
    hi = gflat[cell] + OBST_HI
    obst_pt = np.isfinite(lo) & np.isfinite(hi) & (z > lo) & (z <= hi)
    obst_count = np.bincount(cell[obst_pt], minlength=n_cells)
    obstacle = (obst_count >= 2).reshape(ny, nx) & known
    obst_top = np.full(n_cells, np.nan)
    if obst_pt.any():
        tmp = np.full(n_cells, -np.inf)
        np.maximum.at(tmp, cell[obst_pt], z[obst_pt])
        obst_top = np.where(np.isfinite(tmp), tmp - gflat, np.nan)
    # The robot physically drove through its own swept footprint, so returns there in the
    # obstacle band are dynamic (operator walking alongside, people) or self-returns.
    swept = swept_footprint(poses, x0, y0, res, nx, ny)
    cleared = obstacle & swept
    obstacle &= ~swept
    obst_h = np.where(obstacle, obst_top.reshape(ny, nx), 0.0)
    # Cells with no ground but many returns (wall faces, dense bushes) are unknown, and
    # also marked obstacle_unknown_dense so planners don't mistake them for sparse holes.
    dense_unknown = (~known) & corridor & (raw_count.reshape(ny, nx) >= MIN_PTS)

    # Fill for heightfield use only.
    idx = ndimage.distance_transform_edt(~known, return_distances=False, return_indices=True)
    filled = G[idx[0], idx[1]] if known.any() else np.zeros_like(G)

    step = np.zeros_like(filled)
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        sh = np.roll(np.roll(filled, dy, 0), dx, 1)
        step = np.maximum(step, np.abs(filled - sh))
    step[~known] = np.nan

    course = course_trajectory(poses)
    band = _band_mask(course[:, 1:3], x0, y0, res, nx, ny, 0.5)
    stats = {
        "map_id": MAP_ID,
        "frame": "map",
        "resolution_m": res,
        "shape_rows_cols": [ny, nx],
        "origin_xy": [float(x0), float(y0)],
        "extent_m": [float(nx * res), float(ny * res)],
        "corridor_half_width_m": half_width,
        "corridor_cells": int(corridor.sum()),
        "corridor_unknown_pct": float(100 * (1 - known[corridor].mean())),
        "course_band_0p5m_unknown_pct": float(100 * (1 - known[band].mean())),
        "full_traj_band_0p5m_unknown_pct": float(
            100 * (1 - known[_band_mask(traj[:, :2], x0, y0, res, nx, ny, 0.5)].mean())
        ),
        "despiked_low_cells": int(despiked.sum()),
        "interpolated_small_hole_cells": int(interpolated.sum()),
        "multi_level_cells": int((multi.reshape(ny, nx) & corridor).sum()),
        "multi_level_cells_on_course_band": int((multi.reshape(ny, nx) & band).sum()),
        "driven_height_mismatch_cells": int(mismatch.sum()),
        "driven_height_fixed_cells": int(fixed_cells.sum()),
        "canopy_or_roof_rejected_cells": int(high.sum()),
        "obstacle_cells": int(obstacle.sum()),
        "obstacle_cells_cleared_in_driven_footprint": int(cleared.sum()),
        "obstacle_cells_on_course_band": int((obstacle & band).sum()),
        "dense_unknown_cells": int(dense_unknown.sum()),
        "ground_z_range": [float(np.nanmin(G)), float(np.nanmax(G))],
        "points_in_corridor": len(pts),
        "canopy_points_removed": n_canopy,
        "build_seconds": round(time.time() - t0, 1),
    }
    np.savez_compressed(
        out_dir / "course_terrain.npz",
        ground=G.astype(np.float32),
        ground_filled=filled.astype(np.float32),
        known=known,
        obstacle=obstacle,
        obstacle_height=obst_h.astype(np.float32),
        multi_level=multi.reshape(ny, nx),
        dense_unknown=dense_unknown,
        interpolated=interpolated,
        swept=swept,
        cleared_obstacle=cleared,
        corridor=corridor,
        step=step.astype(np.float32),
        count=raw_count.reshape(ny, nx).astype(np.int32),
        origin=np.array([x0, y0]),
        resolution=np.array(res),
        frame=np.array("map"),
        map_id=np.array(MAP_ID),
        layout=np.array("array[row=iy, col=ix]; cell centre = origin + (i+0.5)*res"),
        trajectory=poses[:, :4],
        course_trajectory=course,
    )
    (out_dir / "terrain_stats.json").write_text(json.dumps(stats, indent=2))
    if verbose:
        print(json.dumps(stats, indent=2))
    return out_dir / "course_terrain.npz", stats


def swept_footprint(poses, x0, y0, res, nx, ny, length=0.9, width=0.5, step=0.1):
    """Cells covered by the 0.9 x 0.5 m body at every keyframe (interpolated every <= 0.1 m,
    yaw from the keyframe quaternions)."""
    from sim_full_course.io_utils import quat_yaw

    yaw = np.unwrap(quat_yaw(poses[:, 4], poses[:, 5], poses[:, 6], poses[:, 7]))
    dense = densify(np.column_stack([poses[:, 1:3], yaw]), step=step)
    u = np.arange(-length / 2, length / 2 + 1e-9, res / 2)
    v = np.arange(-width / 2, width / 2 + 1e-9, res / 2)
    uu, vv = (a.ravel() for a in np.meshgrid(u, v))
    mask = np.zeros(ny * nx, bool)
    for chunk in np.array_split(dense, max(1, len(dense) // 2000)):
        c, sn = np.cos(chunk[:, 2:3]), np.sin(chunk[:, 2:3])
        px = chunk[:, 0:1] + c * uu - sn * vv
        py = chunk[:, 1:2] + sn * uu + c * vv
        ix = np.floor((px - x0) / res).astype(int).ravel()
        iy = np.floor((py - y0) / res).astype(int).ravel()
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        mask[iy[ok] * nx + ix[ok]] = True
    return mask.reshape(ny, nx)


def course_trajectory(poses: np.ndarray) -> np.ndarray:
    """Keyframes in the official course order (reverse time) as [t, x, y, z_base]."""
    m = (poses[:, 0] >= COURSE_EPOCH[0]) & (poses[:, 0] <= COURSE_EPOCH[1])
    return poses[m][::-1, :4].copy()


def _band_mask(xy, x0, y0, res, nx, ny, radius):
    dense = densify(np.column_stack([xy, np.zeros(len(xy))]), step=res)[:, :2]
    tree = cKDTree(dense)
    cx = x0 + (np.arange(nx) + 0.5) * res
    cy = y0 + (np.arange(ny) + 0.5) * res
    gx, gy = np.meshgrid(cx, cy)
    d, _ = tree.query(np.column_stack([gx.ravel(), gy.ravel()]), k=1, distance_upper_bound=radius)
    return np.isfinite(d).reshape(ny, nx)


def render(npz_path, out_dir=None):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.load(npz_path)
    out_dir = out_dir or npz_path.parent
    x0, y0 = t["origin"]
    res = float(t["resolution"])
    ny, nx = t["ground"].shape
    ext = [x0, x0 + nx * res, y0, y0 + ny * res]
    traj, course = t["trajectory"], t["course_trajectory"]

    fig, ax = plt.subplots(figsize=(22, 14), dpi=110)
    g = np.ma.masked_invalid(t["ground"])
    ax.imshow(np.where(t["corridor"], 0.85, 1.0), origin="lower", extent=ext, cmap="gray",
              vmin=0, vmax=1)
    im = ax.imshow(g, origin="lower", extent=ext, cmap="terrain", interpolation="nearest")
    ob = np.ma.masked_where(~t["obstacle"], np.ones_like(g))
    ax.imshow(ob, origin="lower", extent=ext, cmap="autumn", alpha=0.9, interpolation="nearest")
    ml = np.ma.masked_where(~(t["multi_level"] & t["corridor"]), np.ones_like(g))
    ax.imshow(ml, origin="lower", extent=ext, cmap="cool", alpha=0.9, interpolation="nearest")
    ax.plot(traj[:, 1], traj[:, 2], "-", color="k", lw=0.5, alpha=0.5, label="full mapping traj")
    ax.plot(course[:, 1], course[:, 2], "-", color="m", lw=1.5, label="course (reverse time)")
    ax.plot(course[0, 1], course[0, 2], "go", ms=10, label="course start (door)")
    ax.plot(course[-1, 1], course[-1, 2], "rs", ms=10, label="course end")
    fig.colorbar(im, ax=ax, label="ground z (m, map)", shrink=0.6)
    ax.set_title("v3 course ground heightfield (grey=corridor unknown, orange=obstacle 0.1-1.2 m, "
                 "cyan=multi-level flag)")
    ax.set_aspect("equal")
    ax.legend(loc="upper right")
    fig.savefig(out_dir / "terrain_topview.png", bbox_inches="tight")
    ax.set_xlim(-3, 32)
    ax.set_ylim(-5, 26)
    ax.set_title("zoom: Start (door) + B stairs")
    fig.savefig(out_dir / "terrain_topview_start_B.png", bbox_inches="tight")
    plt.close(fig)

    # Elevation along the course.
    xy = course[:, 1:3]
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    ix = np.clip(((xy[:, 0] - x0) / res).astype(int), 0, nx - 1)
    iy = np.clip(((xy[:, 1] - y0) / res).astype(int), 0, ny - 1)
    fig, ax = plt.subplots(figsize=(16, 5), dpi=110)
    ax.plot(s, course[:, 3] - BODY_Z, "k.-", ms=3, label="keyframe base - 0.43")
    ax.plot(s, t["ground"][iy, ix], "g-", label="heightfield ground")
    ax.set_xlabel("course arc length s (m)")
    ax.set_ylabel("z (m, map)")
    ax.legend()
    ax.grid(alpha=0.3)
    ax.set_title("elevation along course (placeholder order: door -> B -> far end)")
    fig.savefig(out_dir / "terrain_elevation_profile.png", bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--half-width", type=float, default=HALF_WIDTH)
    ap.add_argument("--res", type=float, default=RES)
    a = ap.parse_args()
    path, _ = build(a.half_width, a.res)
    render(path)


if __name__ == "__main__":
    main()
