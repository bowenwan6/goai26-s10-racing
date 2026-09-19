"""Offline: keep the route on the structure it crosses -- a smooth lateral offset of the taught
line, found by dynamic programming over a clearance field of the map, and a re-planned line where
the taught one cannot be offset into the clear.

The taught line is where the mapping robot drove, but on the reconstruction it can hug a stair's
edge, clip a void, or run along a flight that the map draws narrower than it is. Instead of
hand-placed centrelines this picks, for every centreline sample, a lateral offset ``d`` from a
lattice (Viterbi over offsets, the 1-D version of a Frenet lattice planner) minimising

    w_off * |d|                                  stay near the taught line
  + w_clear / max(clearance, eps)                keep away from voids and drops
  + w_centre * max(0, min(cap, c_best) - clear)  on steps: the middle of the flight (c_best is the
  best
                                                 clearance any offset reaches at that sample)
  + w_smooth * (d_i - d_{i-1})^2                 no kinks (and |d_i - d_{i-1}| <= max_slope * ds)

subject to clearance(d) >= half_width + margin (+ the stairs actor's larger lateral error on steps),
where clearance is the distance from the offset point to the nearest hazard cell of the map: unknown
ground, or either side of a vertical jump taller than ``cliff`` (0.3 m: a stair riser is not a
hazard, the side of a raised flight is).

Where no offset of the taught line is clear -- it crosses a gap the map shows, as the taught line
between WP06 and WP07 of the course does -- the segment is re-planned: Dijkstra over the map cells
within ``corridor`` of the taught line, the clear ones only, each step paying extra for a clearance
deficit (so the path runs down the middle of a narrow flight), smoothed, and then centred as above.
Every change is reported.

Waypoints keep their position unless it is not clear; then the nearest clear point is used and the
move is reported, since a photo-matched waypoint beside a drop in the map needs confirming on site.

The margins come from the policy profile (``capability.PolicyProfile``): the body's half width, the
walking actor's lateral error off the steps and the stairs actor's on them.

The same function runs on the robot's own v3 point-cloud terrain
(``sim_full_course.terrain.Terrain``) and on the MuJoCo-derived one: it needs ``ground``/``known``
rasters with ``origin``/``res`` and ``ground_at``/``known_at``.
"""

from __future__ import annotations

import copy
import math

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra


def clearance_field(terrain, cliff=0.3, obstacle=0.05):
    """Distance (m) from every map cell to the nearest hazard cell: unknown, either side of a jump
    taller than ``cliff``, or under a static obstacle taller than ``obstacle``."""
    g, known, res = terrain.ground, terrain.known, terrain.res
    haz = ~known
    obst = getattr(terrain, "obstacle_height", None)
    if obst is not None:
        haz = haz | (np.asarray(obst) > obstacle)
    for ax in (0, 1):
        dz = np.abs(np.diff(g, axis=ax))
        jump = dz > cliff
        if ax == 0:
            haz[:-1, :] |= jump
            haz[1:, :] |= jump
        else:
            haz[:, :-1] |= jump
            haz[:, 1:] |= jump
    return ndimage.distance_transform_edt(~haz) * res


def riser_normal_field(terrain, sigma=0.3, cliff=0.3, min_rise=0.05):
    """Uphill direction of the steps around every cell: the ground gradient over the step edges only
    (a jump between ``min_rise`` and ``cliff`` across two cells, known ground), smoothed over
    ``sigma`` m by normalised convolution so a tread between two risers gets their direction. ->
    (ux, uy, coherence): coherence is the length of the mean unit gradient nearby (1 on a flight, ~0
    on rubble, 0 away from any edge), so only real flights constrain the route's heading."""
    g, known, res = terrain.ground, terrain.known, terrain.res
    gx = np.zeros(g.shape)
    gy = np.zeros(g.shape)
    gx[:, 1:-1] = g[:, 2:] - g[:, :-2]
    gy[1:-1, :] = g[2:, :] - g[:-2, :]
    ok = known.copy()
    ok[:, 1:-1] &= known[:, 2:] & known[:, :-2]
    ok[1:-1, :] &= known[2:, :] & known[:-2, :]
    mag = np.hypot(gx, gy)
    edge = ok & (mag > min_rise) & (mag < cliff)
    w = edge.astype(float)
    k = sigma / res
    sx = ndimage.gaussian_filter(np.where(edge, gx / np.maximum(mag, 1e-9), 0.0), k)
    sy = ndimage.gaussian_filter(np.where(edge, gy / np.maximum(mag, 1e-9), 0.0), k)
    sw = ndimage.gaussian_filter(w, k)
    norm = np.maximum(np.hypot(sx, sy), 1e-9)
    coherence = np.where(sw > 0.02, norm / np.maximum(sw, 1e-9), 0.0)
    return sx / norm, sy / norm, coherence


def _sample(field, terrain, x, y):
    ix = np.floor((np.asarray(x) - terrain.origin[0]) / terrain.res).astype(int)
    iy = np.floor((np.asarray(y) - terrain.origin[1]) / terrain.res).astype(int)
    ny, nx = field.shape
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    out = np.zeros(np.shape(x))
    out[ok] = field[iy[ok], ix[ok]]
    return out


def _resample(xy, ds):
    L = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    u = np.linspace(0.0, L[-1], max(2, math.ceil(L[-1] / ds) + 1))
    return np.column_stack([np.interp(u, L, xy[:, 0]), np.interp(u, L, xy[:, 1])])


def _on_steps(terrain, xy, rise=0.035, reach=0.6):
    """Per point: is there a step (a rise over ``rise`` between neighbouring samples) within
    ``reach`` along the line?"""
    z = terrain.ground_at(xy[:, 0], xy[:, 1])
    L = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    ds = max(L[-1] / max(len(L) - 1, 1), 1e-6)
    w = max(1, round(float(reach / ds)))
    jumps = np.abs(np.diff(z, prepend=z[0])) > rise
    return np.convolve(jumps, np.ones(2 * w + 1), mode="same") > 0


def centre_route(
    route_dict,
    terrain,
    profile=None,
    half_width=0.3,
    margin=0.1,
    stairs_margin=0.2,
    search=1.6,
    step=0.05,
    ds=0.2,
    w_off=1.0,
    w_clear=0.3,
    w_centre=3.0,
    centre_cap=0.7,
    w_smooth=20.0,
    max_slope=0.8,
    cliff=0.3,
    relax=((1.0, 0.0), (1.0, 0.05), (1.25, 0.1)),
    corridor=2.5,
    band=True,
    w_bend_steps=5.0,
    w_bend_flat=0.5,
    w_keep=0.5,
    band_target_extra=0.1,
    skew_limit_deg=25.0,
    w_skew=60.0,
    detour_ratio=1.3,
    frozen=(),
):
    """Returns (route, report). Centrelines are resampled at ``ds`` and re-grounded.

    A segment with no feasible profile is retried with a steeper allowed offset change and a smaller
    margin (``relax``: (slope factor, margin reduction) pairs, in order); if none works it is
    re-planned (Dijkstra in a corridor of the taught line) and the new line centred; what it took is
    reported.

    Segments named in ``frozen`` (hand-validated lines, already in ``route_dict``) are kept as they
    are: only their ends are snapped to the waypoints, and the band holds them fixed.

    A taught line more than ``detour_ratio`` times longer than the shortest clear line between its
    waypoints (in the same corridor) is replaced by that line: the taught line is a hint of where
    the ground is good, not an order to repeat the mapping robot's overshoots.

    On steps (edges within 0.6 m along the line) the margin is ``stairs_margin`` and the line is
    pulled to the middle of the flight (up to ``centre_cap`` m from either side).

    Finally (``band``) the whole route is relaxed as an elastic band with the waypoints pinned, so
    it bends smoothly through them: bending costs ``w_bend_steps`` on steps (the stairs actor barely
    turns) and ``w_bend_flat`` elsewhere, leaving the centred line costs ``w_keep``, clearance under
    the margin plus ``band_target_extra`` costs a lot, and so does running on steps more than
    ``skew_limit_deg`` off the risers (the stairs actor squares itself to them and hardly steers
    against that). A segment the band would bring under its hard margin keeps the centred line.
    Where a waypoint cannot be reached within the skew limit the report says how far off square the
    route has to run, and where."""
    if profile is not None:
        half_width, margin, stairs_margin = (
            profile.body_half_width,
            profile.walk.lateral_error,
            profile.climb.lateral_error,
        )
        skew_limit_deg = profile.climb.max_entry_skew_deg
    field = clearance_field(terrain, cliff)
    offs = np.arange(-search, search + 1e-9, step)
    r = copy.deepcopy(route_dict)
    report = {
        "segments": [],
        "waypoints": [],
        "params": {
            "half_width": half_width,
            "margin": margin,
            "stairs_margin": stairs_margin,
            "centre_cap": centre_cap,
            "corridor": corridor,
        },
    }

    # Waypoints first: unchanged if clear, else the nearest clear point (reported).
    for wp in r["waypoints"]:
        p = np.asarray(wp["position"][:2], float)
        near = terrain.ground_at(
            p[0] + np.array([-0.45, 0, 0.45, -0.45, 0, 0.45, -0.45, 0, 0.45]),
            p[1] + np.array([-0.45, -0.45, -0.45, 0, 0, 0, 0.45, 0.45, 0.45]),
        )
        wp_need = half_width + (stairs_margin if float(np.ptp(near)) > 0.1 else margin)
        if _sample(field, terrain, p[0], p[1]) >= wp_need:
            continue
        best, best_d = None, math.inf
        for dx in np.arange(-search, search + 1e-9, step):
            for dy in np.arange(-search, search + 1e-9, step):
                q = p + np.array([dx, dy])
                dd = math.hypot(dx, dy)
                if dd < best_d and _sample(field, terrain, q[0], q[1]) >= wp_need:
                    best, best_d = q, dd
        if best is None:
            report["waypoints"].append(
                {"id": wp["id"], "shift_m": None, "note": "no clear point nearby"}
            )
            continue
        report["waypoints"].append(
            {
                "id": wp["id"],
                "shift_m": round(best_d, 3),
                "from": p.round(3).tolist(),
                "to": best.round(3).tolist(),
                "clearance_before_m": round(float(_sample(field, terrain, *p)), 2),
            }
        )
        wp["position"] = [float(best[0]), float(best[1]), float(terrain.ground_at(*best))]

    kw = dict(
        offs=offs,
        ds=ds,
        w_off=w_off,
        w_clear=w_clear,
        w_centre=w_centre,
        centre_cap=centre_cap,
        w_smooth=w_smooth,
        stairs_extra=stairs_margin - margin,
    )
    for k, seg in enumerate(r["segments"]):
        a = np.asarray(r["waypoints"][k]["position"][:2], float)
        b = np.asarray(r["waypoints"][k + 1]["position"][:2], float)
        c = np.asarray(seg["centerline"], float)[:, :2].copy()
        c[0], c[-1] = a, b
        if seg["id"] in frozen:
            # Its waypoints may have moved (clearance): keep the shape, snap the ends, keep the
            # spacing.
            c = _resample(c, ds)
            seg["centerline"] = np.column_stack([c, terrain.ground_at(c[:, 0], c[:, 1])]).tolist()
            clear = _sample(field, terrain, c[:, 0], c[:, 1])
            report["segments"].append(
                {
                    "id": seg["id"],
                    "note": "frozen (hand-validated line kept)",
                    "min_clearance_m": round(float(clear.min()), 2),
                }
            )
            continue
        detour = None
        length = float(np.linalg.norm(np.diff(c, axis=0), axis=1).sum())
        if length > detour_ratio * float(np.linalg.norm(b - a)) + 0.5:
            # The taught line wanders (the mapping robot overshot a waypoint and came back, or went
            # round something): if the shortest clear line in its corridor is much shorter, that is
            # the route.
            alt = _replan(c, terrain, field, half_width + margin, corridor, centre_cap, ds)
            if alt is not None:
                alt_len = float(np.linalg.norm(np.diff(alt, axis=0), axis=1).sum())
                if length > detour_ratio * alt_len + 0.3:
                    detour = (
                        f"taught line is a detour ({length:.1f} m against {alt_len:.1f} m): "
                        "re-planned"
                    )
                    c = alt
        row = _centre_line(c, terrain, field, half_width + margin, max_slope, relax, margin, **kw)
        if row is not None and detour:
            row["note"] = detour + (f"; {row['note']}" if row.get("note") else "")
        if row is None:
            path = _replan(
                c,
                terrain,
                field,
                half_width + margin - max(m for _, m in relax),
                corridor,
                centre_cap,
                ds,
            )
            if path is None:
                seg["centerline"] = np.column_stack(
                    [c, terrain.ground_at(c[:, 0], c[:, 1])]
                ).tolist()
                report["segments"].append(
                    {"id": seg["id"], "note": "no clear line in the corridor; taught line kept"}
                )
                continue
            row = _centre_line(
                path, terrain, field, half_width + margin, max_slope, relax, margin, **kw
            )
            if row is None:
                xy = path
                row = {"note": "re-planned (not centred)"}
            else:
                row["note"] = "re-planned: the taught line crosses a hazard" + (
                    f"; {row['note']}" if row.get("note") else ""
                )
                xy = row.pop("xy")
        else:
            xy = row.pop("xy")
        seg["centerline"] = np.column_stack([xy, terrain.ground_at(xy[:, 0], xy[:, 1])]).tolist()
        clear = _sample(field, terrain, xy[:, 0], xy[:, 1])
        if row.get("moved") or row.get("note"):
            row.pop("moved", None)
            report["segments"].append(
                {"id": seg["id"], **row, "min_clearance_m": round(float(clear.min()), 2)}
            )
    if band:
        _band_route(
            r,
            terrain,
            field,
            half_width + margin,
            stairs_margin - margin,
            band_target_extra,
            w_bend_steps,
            w_bend_flat,
            w_keep,
            report,
            skew_limit=math.radians(skew_limit_deg),
            w_skew=w_skew,
            frozen=set(frozen),
        )
    return r, report


def _field_sampler(field, terrain):
    """Bilinear clearance and its gradient at arbitrary map points."""
    gy, gx = np.gradient(field, terrain.res)
    ny, nx = field.shape

    def sample(P):
        fx = (P[:, 0] - terrain.origin[0]) / terrain.res - 0.5
        fy = (P[:, 1] - terrain.origin[1]) / terrain.res - 0.5
        i = np.clip(np.floor(fx).astype(int), 0, nx - 2)
        j = np.clip(np.floor(fy).astype(int), 0, ny - 2)
        tx, ty = np.clip(fx - i, 0, 1), np.clip(fy - j, 0, 1)

        def bil(a):
            return (
                (1 - tx) * (1 - ty) * a[j, i]
                + tx * (1 - ty) * a[j, i + 1]
                + (1 - tx) * ty * a[j + 1, i]
                + tx * ty * a[j + 1, i + 1]
            )

        return bil(field), bil(gx), bil(gy)

    return sample


def _band_route(
    r,
    terrain,
    field,
    need_flat,
    stairs_extra,
    target_extra,
    w_bend_steps,
    w_bend_flat,
    w_keep,
    report,
    w_clear=200.0,
    iters=2000,
    skew_limit=math.radians(25.0),
    w_skew=20.0,
    frozen=frozenset(),
):
    """Elastic band over the whole route (Quinlan & Khatib): minimise

        sum w_bend |p_{i+1} - 2 p_i + p_{i-1}|^2 / ds^3     bending (~ integral of curvature^2)
      + w_keep  * ds * |p_i - q_i|^2                          stay near the centred line q
      + w_clear * ds * max(0, target_i - clearance(p_i))^2    room from hazards
      + w_skew  * ds * max(0, skew_i - skew_limit)^2          on steps: within skew_limit of the
      risers

    with every waypoint pinned, so the line bends through a waypoint instead of kinking at it.
    skew_i is the angle between the route and the local riser normal (either way: up or down the
    flight)."""
    from scipy.optimize import minimize

    segs = [np.asarray(sg["centerline"], float)[:, :2] for sg in r["segments"]]
    if not segs:
        return
    pts, pinned, spans = [segs[0][0]], [True], []
    for c, sg in zip(segs, r["segments"], strict=True):
        spans.append((len(pts) - 1, len(pts) - 1 + len(c) - 1))  # first and last index, inclusive
        pts.extend(c[1:])
        # A frozen (hand-validated) segment is held fixed as a whole.
        pinned.extend([sg["id"] in frozen] * (len(c) - 2) + [True])
    q = np.asarray(pts, float)
    pinned = np.asarray(pinned)
    n = len(q)
    if n < 3:
        return
    ds = float(np.linalg.norm(np.diff(q, axis=0), axis=1).mean())
    steps = _on_steps(terrain, q)
    need = need_flat + np.where(steps, stairs_extra, 0.0)
    target = need + target_extra
    w_bend = np.where(steps, w_bend_steps, w_bend_flat)
    sample = _field_sampler(field, terrain)
    ux, uy, uw = riser_normal_field(terrain)

    def normal_at(P):
        ix = np.clip(
            np.floor((P[:, 0] - terrain.origin[0]) / terrain.res).astype(int), 0, ux.shape[1] - 1
        )
        iy = np.clip(
            np.floor((P[:, 1] - terrain.origin[1]) / terrain.res).astype(int), 0, ux.shape[0] - 1
        )
        return np.arctan2(uy[iy, ix], ux[iy, ix]), uw[iy, ix]

    # The riser direction is taken where the centred line runs (it moves little), and only where
    # there are risers nearby.
    n_yaw, n_w = normal_at(q)
    skewed = steps & (n_w > 0.8)
    free = ~pinned

    def skew_terms(P):
        # Heading over +-2 samples (0.8 m): the robot's heading, not the direction of a single
        # sample step, so the band cannot satisfy the limit by tacking.
        t = np.zeros_like(P)
        t[2:-2] = P[4:] - P[:-4]
        t[:2], t[-2:] = P[2:4] - P[0:2], P[-2:] - P[-4:-2]
        th = np.arctan2(t[:, 1], t[:, 0])
        d = (th - n_yaw + np.pi) % (2 * np.pi) - np.pi
        # Up or down the flight: fold to [-pi/2, pi/2].
        d = np.where(d > np.pi / 2, d - np.pi, np.where(d < -np.pi / 2, d + np.pi, d))
        over = np.where(skewed, np.maximum(0.0, np.abs(d) - skew_limit), 0.0)
        return t, d, over

    def energy(v):
        P = q.copy()
        P[free] = v.reshape(-1, 2)
        grad = np.zeros_like(P)
        D2 = P[2:] - 2 * P[1:-1] + P[:-2]
        wb = w_bend[1:-1, None] / ds**3
        e = float((wb * D2**2).sum())
        g2 = 2 * wb * D2
        grad[2:] += g2
        grad[1:-1] -= 2 * g2
        grad[:-2] += g2
        dq = P - q
        e += w_keep * ds * float((dq**2).sum())
        grad += 2 * w_keep * ds * dq
        c, cgx, cgy = sample(P)
        deficit = np.maximum(0.0, target - c)
        e += w_clear * ds * float((deficit**2).sum())
        grad[:, 0] -= 2 * w_clear * ds * deficit * cgx
        grad[:, 1] -= 2 * w_clear * ds * deficit * cgy
        t, d, over = skew_terms(P)
        e += w_skew * ds * float((over**2).sum())
        # d(theta)/dt = (-ty, tx) / |t|^2; t_i = P_{i+1} - P_{i-1}.
        coef = 2 * w_skew * ds * over * np.sign(d) / np.maximum((t**2).sum(axis=1), 1e-9)
        gt = coef[:, None] * np.column_stack([-t[:, 1], t[:, 0]])
        grad[4:] += gt[2:-2]
        grad[:-4] -= gt[2:-2]
        grad[2:4] += gt[:2]
        grad[0:2] -= gt[:2]
        grad[-2:] += gt[-2:]
        grad[-4:-2] -= gt[-2:]
        return e, grad[free].ravel()

    res = minimize(energy, q[free].ravel(), jac=True, method="L-BFGS-B", options={"maxiter": iters})
    P = q.copy()
    P[free] = res.x.reshape(-1, 2)
    clear_new = _sample(field, terrain, P[:, 0], P[:, 1])
    clear_old = _sample(field, terrain, q[:, 0], q[:, 1])
    rows = []
    for (i0, i1), sg in zip(spans, r["segments"], strict=True):
        idx = np.arange(i0, i1 + 1)
        # Keep the band only if it stays clear of the hard margin (or no worse than the centred
        # line).
        ok = (clear_new[idx] >= np.minimum(need[idx], clear_old[idx]) - 0.02).all()
        if not ok:
            rows.append({"id": sg["id"], "band": "rejected: would cut the margin"})
            continue
        if sg["id"] not in frozen:
            xy = _resample(P[idx], ds)
            sg["centerline"] = np.column_stack([xy, terrain.ground_at(xy[:, 0], xy[:, 1])]).tolist()
        moved = float(np.linalg.norm(P[idx] - q[idx], axis=1).max())
        _, d_new, over_new = skew_terms(P)
        worst = float(np.max(np.where(skewed[idx], np.abs(d_new[idx]), 0.0)))
        row = {"id": sg["id"]}
        if moved > 0.05:
            row.update(
                {
                    "band_max_shift_m": round(moved, 2),
                    "min_clearance_m": round(float(clear_new[idx].min()), 2),
                }
            )
        if skewed[idx].any():
            row["max_skew_on_steps_deg"] = round(math.degrees(worst), 1)
            if (over_new[idx] > math.radians(3)).any():
                bad = idx[over_new[idx] > math.radians(3)]
                row["skew_warning"] = (
                    f"runs up to {math.degrees(worst):.0f} deg off the risers over "
                    f"{len(bad) * ds:.1f} m near ({P[bad[0], 0]:.1f}, {P[bad[0], 1]:.1f}): a "
                    "waypoint the stairs actor cannot reach square-on"
                )
        if len(row) > 1:
            rows.append(row)
    report["band"] = {
        "iterations": int(res.nit),
        "converged": bool(res.success),
        "skew_limit_deg": round(math.degrees(skew_limit), 1),
        "segments": rows,
    }


def _centre_line(c, terrain, field, need, max_slope, relax, margin, **kw):
    for slope_f, margin_cut in relax:
        out = _centre_segment(c, terrain, field, need - margin_cut, max_slope * slope_f, **kw)
        if out is not None:
            xy, prof = out
            row = {
                "xy": xy,
                "max_offset_m": round(float(np.abs(prof).max()), 2),
                "samples_moved": int((np.abs(prof) > 1e-9).sum()),
                "samples": len(prof),
                "moved": bool((np.abs(prof) > 1e-9).any()),
            }
            if (slope_f, margin_cut) != relax[0]:
                row["note"] = (
                    f"needed max slope {max_slope * slope_f:.2f} and margin "
                    f"{margin - margin_cut:.2f} m"
                )
            return row
    return None


def _centre_segment(
    c,
    terrain,
    field,
    need,
    max_slope,
    offs,
    ds,
    w_off,
    w_clear,
    w_centre,
    centre_cap,
    w_smooth,
    stairs_extra,
):
    """Viterbi over lateral offsets of polyline ``c`` (ends pinned). -> (xy resampled at ds,
    offsets) or None."""
    base = _resample(np.asarray(c, float)[:, :2], ds)
    n = len(base)
    du = float(np.linalg.norm(np.diff(base, axis=0), axis=1).mean()) if n > 1 else ds
    tang = np.gradient(base, axis=0)
    tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-9)
    nrm = np.column_stack([-tang[:, 1], tang[:, 0]])
    cand = base[:, None, :] + offs[None, :, None] * nrm[:, None, :]
    clear = _sample(field, terrain, cand[..., 0], cand[..., 1])
    steps = _on_steps(terrain, base)
    feasible = clear >= (need + np.where(steps, stairs_extra, 0.0))[:, None]
    cost = w_off * np.abs(offs)[None, :] + w_clear / np.maximum(clear, 0.05)
    # On steps: pull towards the middle of the flight, up to centre_cap from either side.
    want = np.minimum(centre_cap, np.where(feasible, clear, 0.0).max(axis=1))
    cost = cost + np.where(steps, 1.0, 0.0)[:, None] * w_centre * np.maximum(
        0.0, want[:, None] - clear
    )
    cost = np.where(feasible, cost, np.inf)
    # Ends pinned to the waypoints (offset 0; the waypoint itself was already moved if it had to
    # be).
    mid = int(np.argmin(np.abs(offs)))
    pin = np.full(len(offs), np.inf)
    pin[mid] = 0.0
    cost[0], cost[-1] = pin, pin
    if n > 2 and not np.isfinite(cost[1:-1]).any(axis=1).all():
        return None
    dmax = max_slope * du + 1e-9
    jumps = np.abs(offs[:, None] - offs[None, :])
    trans = np.where(jumps <= dmax, w_smooth * jumps**2, np.inf)
    acc = cost[0].copy()
    back = np.zeros((n, len(offs)), int)
    for i in range(1, n):
        tot = acc[:, None] + trans  # from j (rows) to l (cols)
        back[i] = np.argmin(tot, axis=0)
        acc = tot[back[i], np.arange(len(offs))] + cost[i]
    if not np.isfinite(acc.min()):
        return None
    j = int(np.argmin(acc))
    prof = np.zeros(n)
    for i in range(n - 1, -1, -1):
        prof[i] = offs[j]
        j = back[i, j]
    xy = base + prof[:, None] * nrm
    # Offsets that change quickly stretch the polyline; resample at ds (route_v2 allows 0.3 m).
    return _resample(xy, ds), prof


def _replan(c, terrain, field, need, corridor, centre_cap, ds, w_deficit=4.0, sigma=0.3):
    """Dijkstra over the clear map cells within ``corridor`` of polyline ``c``, from its first point
    to its last; each step costs its length times (1 + w_deficit * clearance deficit / centre_cap).
    Smoothed (Gaussian, ends pinned, backed off while it stays clear). -> (N, 2) or None."""
    res, (ox, oy) = terrain.res, terrain.origin
    lo = np.floor((c.min(axis=0) - corridor - [ox, oy]) / res).astype(int)
    hi = np.ceil((c.max(axis=0) + corridor - [ox, oy]) / res).astype(int)
    ny, nx = field.shape
    i0, j0 = max(lo[0], 0), max(lo[1], 0)
    i1, j1 = min(hi[0], nx), min(hi[1], ny)
    sub = field[j0:j1, i0:i1]
    h, w = sub.shape
    # Corridor: cells within `corridor` of the taught line.
    line = _resample(c, res / 2)
    mask = np.zeros((h, w), bool)
    li = np.floor((line[:, 0] - ox) / res).astype(int) - i0
    lj = np.floor((line[:, 1] - oy) / res).astype(int) - j0
    ok = (li >= 0) & (li < w) & (lj >= 0) & (lj < h)
    mask[lj[ok], li[ok]] = True
    in_corr = ndimage.distance_transform_edt(~mask) * res <= corridor
    free = in_corr & (sub >= need)

    def cell(p):
        return (int(np.floor((p[1] - oy) / res)) - j0, int(np.floor((p[0] - ox) / res)) - i0)

    start, goal = cell(c[0]), cell(c[-1])
    for q in (start, goal):
        if not (0 <= q[0] < h and 0 <= q[1] < w):
            return None
        # The waypoints themselves are clear (moved if they were not); open a small disc around them
        # in case a margin reduction made them marginal.
        yy, xx = np.ogrid[:h, :w]
        free |= ((yy - q[0]) ** 2 + (xx - q[1]) ** 2 <= 9) & in_corr
    unit = 1.0 + w_deficit * np.maximum(0.0, centre_cap - sub) / centre_cap
    idx = np.arange(h * w).reshape(h, w)
    rows, cols, wts = [], [], []
    for dj, di in ((0, 1), (1, 0), (1, 1), (1, -1)):
        # Cell (j, i) and its neighbour (j + dj, i + di), as two aligned slices.
        ja, jb = (0, h - dj) if dj >= 0 else (-dj, h), (dj, h) if dj >= 0 else (0, h + dj)
        ia, ib = (0, w - di) if di >= 0 else (-di, w), (di, w) if di >= 0 else (0, w + di)
        a = free[ja[0] : ja[1], ia[0] : ia[1]]
        b = free[jb[0] : jb[1], ib[0] : ib[1]]
        a_i = idx[ja[0] : ja[1], ia[0] : ia[1]]
        b_i = idx[jb[0] : jb[1], ib[0] : ib[1]]
        ua = unit[ja[0] : ja[1], ia[0] : ia[1]]
        ub = unit[jb[0] : jb[1], ib[0] : ib[1]]
        both = a & b
        L = res * math.hypot(dj, di)
        rows.append(a_i[both])
        cols.append(b_i[both])
        wts.append(L * 0.5 * (ua[both] + ub[both]))
    rows, cols, wts = np.concatenate(rows), np.concatenate(cols), np.concatenate(wts)
    graph = coo_matrix(
        (np.r_[wts, wts], (np.r_[rows, cols], np.r_[cols, rows])), shape=(h * w, h * w)
    ).tocsr()
    s_idx, g_idx = idx[start], idx[goal]
    dist, pred = dijkstra(graph, indices=s_idx, return_predecessors=True)
    if not np.isfinite(dist[g_idx]):
        return None
    chain = [g_idx]
    while chain[-1] != s_idx:
        chain.append(pred[chain[-1]])
    chain = np.array(chain[::-1])
    pj, pi = np.divmod(chain, w)
    xy = np.column_stack([ox + (pi + i0 + 0.5) * res, oy + (pj + j0 + 0.5) * res])
    xy[0], xy[-1] = c[0], c[-1]
    xy = _resample(xy, ds / 2)
    # Smooth: Gaussian along the line, ends pinned; back off sigma until it stays clear.
    for sg in (sigma, sigma / 2, sigma / 4, 0.0):
        if sg <= 0:
            out = xy
            break
        k = sg / (ds / 2)
        sm = np.column_stack(
            [
                ndimage.gaussian_filter1d(xy[:, 0], k, mode="nearest"),
                ndimage.gaussian_filter1d(xy[:, 1], k, mode="nearest"),
            ]
        )
        ramp = np.clip(
            np.minimum(np.arange(len(xy)), np.arange(len(xy))[::-1]) / max(3 * k, 1), 0, 1
        )[:, None]
        sm = ramp * sm + (1 - ramp) * xy
        if (_sample(field, terrain, sm[:, 0], sm[:, 1]) >= need - 0.02).all():
            out = sm
            break
    return _resample(out, ds)


# ---------------------------------------------------------------------------------------------
# The first version's route preparation (s10-rl-sprint route_terrain.sanitize_route, ported
# unchanged): the route the full-course simulation ran end to end. The default for this course;
# centre_route above is the alternative for a map without hand-validated segments.


def _hazard(terrain, xy, z_ref, drop=0.45):
    """Void (not in the map) or a level change of more than ``drop`` from the reference height."""
    z = terrain.ground_at(xy[..., 0], xy[..., 1])
    return (~terrain.known_at(xy[..., 0], xy[..., 1])) | (np.abs(z - z_ref) > drop)


def _nearest_clear(terrain, p, radius=0.4, search=1.2, step=0.05):
    """Nearest point to ``p`` whose disc of ``radius`` holds no hazard (None within ``search``)."""
    ang = np.linspace(0, 2 * np.pi, 16, endpoint=False)
    disc = np.vstack(
        [[0.0, 0.0]]
        + [np.column_stack([rr * np.cos(ang), rr * np.sin(ang)]) for rr in (radius / 2, radius)]
    )
    best, best_d = None, np.inf
    for dx in np.arange(-search, search + 1e-9, step):
        for dy in np.arange(-search, search + 1e-9, step):
            dd = math.hypot(dx, dy)
            if dd > search or dd >= best_d:
                continue
            q = p[:2] + np.array([dx, dy])
            if not terrain.known_at(q[0], q[1]):
                continue
            if not _hazard(terrain, q + disc, float(terrain.ground_at(q[0], q[1]))).any():
                best, best_d = q, dd
    return best


def sanitize_route(
    route_dict, terrain, half_width=0.4, narrow=0.0, search=1.5, smooth=1.2, skip=()
):
    """Keep the robot's whole width on the structure it is driving on.

    1. Waypoints closer than ``half_width`` to a void or a drop of more than 0.45 m move to the
       nearest point with that clearance (reported: they are photo-matched drafts, and the map's
       stair edges are not exact either).
    2. At every centreline point, the lateral offsets whose +-``half_width`` cross-section has no
       hazard form intervals. On a narrow structure (clear interval under ``narrow`` m) the path
       goes down its middle; elsewhere the taught offset 0 stays unless it is not clear, in which
       case the nearest clear offset is taken.
    3. The offset profile is smoothed over ``smooth`` m, pinned to the waypoints, and a smoothed
       point that is no longer clear falls back to its own target. Centrelines are then resampled
       at 0.2 m. Segments in ``skip`` (hand-validated) are left as they are.
    """
    r = copy.deepcopy(route_dict)
    report = {"segments": [], "waypoints": []}
    for wp in r["waypoints"]:
        p = np.asarray(wp["position"][:2], float)
        z = float(terrain.ground_at(*p))
        ang = np.linspace(0, 2 * np.pi, 16, endpoint=False)
        disc = p + np.vstack(
            [[0.0, 0.0]]
            + [
                np.column_stack([rr * np.cos(ang), rr * np.sin(ang)])
                for rr in (half_width / 2, half_width)
            ]
        )
        if terrain.known_at(*p) and not _hazard(terrain, disc, z).any():
            continue
        q = _nearest_clear(terrain, p, half_width)
        if q is None:
            report["waypoints"].append(
                {"id": wp["id"], "shift_m": None, "note": "no clear point within 1.2 m"}
            )
            continue
        report["waypoints"].append(
            {
                "id": wp["id"],
                "shift_m": round(float(np.linalg.norm(q - p)), 3),
                "from": [round(v, 3) for v in p],
                "to": [round(v, 3) for v in q],
            }
        )
        wp["position"] = [float(q[0]), float(q[1]), float(terrain.ground_at(*q))]

    offs = np.arange(-search, search + 1e-9, 0.05)
    lat = np.linspace(-half_width, half_width, 9)
    for k, seg in enumerate(r["segments"]):
        if seg["id"] in skip:
            continue
        c = np.asarray(seg["centerline"], float)
        c[0, :2] = r["waypoints"][k]["position"][:2]
        c[-1, :2] = r["waypoints"][k + 1]["position"][:2]
        tang = np.gradient(c[:, :2], axis=0)
        tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-9)
        normal = np.column_stack([-tang[:, 1], tang[:, 0]])
        target = np.zeros(len(c))
        clear_at = []
        changed = np.zeros(len(c), bool)
        unfixable = 0
        for i, (p, n) in enumerate(zip(c[:, :2], normal, strict=True)):
            q = p + offs[:, None] * n
            zc = terrain.ground_at(q[:, 0], q[:, 1])
            xs = q[:, None, :] + lat[None, :, None] * n
            clear = ~_hazard(terrain, xs, zc[:, None]).any(axis=1) & terrain.known_at(
                q[:, 0], q[:, 1]
            )
            clear_at.append(clear)
            if i in (0, len(c) - 1):
                continue
            if not clear.any():
                unfixable += 1
                continue
            idx = np.flatnonzero(clear)
            j0 = idx[np.argmin(np.abs(offs[idx]))]
            lo = hi = j0
            while lo - 1 >= 0 and clear[lo - 1]:
                lo -= 1
            while hi + 1 < len(offs) and clear[hi + 1]:
                hi += 1
            width = offs[hi] - offs[lo]
            if narrow > 0 and width < narrow:
                target[i] = 0.5 * (offs[lo] + offs[hi])
            elif not clear[len(offs) // 2]:
                target[i] = offs[j0] + (0.1 if offs[j0] > 0 else -0.1)
            changed[i] = abs(target[i]) > 1e-9
        if (
            not changed.any()
            and unfixable == 0
            and np.allclose(c[0, :2], seg["centerline"][0][:2])
            and np.allclose(c[-1, :2], seg["centerline"][-1][:2])
        ):
            continue
        s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(c[:, :2], axis=0), axis=1))]
        prof = target.copy()
        if changed.any():
            w = np.exp(-0.5 * ((s[:, None] - s[None, :]) / (smooth / 2)) ** 2)
            prof = (w @ target) / w.sum(axis=1)
            prof[0] = prof[-1] = 0.0
            ramp = np.clip(np.minimum(s, s[-1] - s) / 0.8, 0.0, 1.0)
            prof *= ramp
            for i in range(1, len(c) - 1):
                j = int(np.argmin(np.abs(offs - prof[i])))
                if not clear_at[i][j]:
                    prof[i] = target[i]
        c[:, :2] += prof[:, None] * normal
        length = float(np.sum(np.linalg.norm(np.diff(c[:, :2], axis=0), axis=1)))
        n_pts = max(2, math.ceil(length / 0.2) + 1)
        seg_len = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(c[:, :2], axis=0), axis=1))]
        u = np.linspace(0.0, length, n_pts)
        c = np.column_stack([np.interp(u, seg_len, c[:, kk]) for kk in range(3)])
        c[:, 2] = terrain.ground_at(c[:, 0], c[:, 1])
        seg["centerline"] = c.tolist()
        report["segments"].append(
            {
                "id": seg["id"],
                "points_moved": int(changed.sum()),
                "max_shift_m": round(float(np.abs(prof).max()), 2),
                "unfixable_points": unfixable,
            }
        )
    return r, report
