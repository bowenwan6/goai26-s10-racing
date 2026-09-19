"""Offline: where along a route_v2 the stairs actor is needed, with the map's guess at each edge.

Input is a map terrain -- anything with ``ground_at(x, y)`` and ``known_at(x, y)`` over map
coordinates: the nav simulation's ``Terrain`` built from the v3 point cloud on the robot side, the
MuJoCo-derived one in simulation -- and the route's centreline (``RoutePath.points``, so arc lengths
match the follower's).

A manoeuvre is a stretch the walking actor cannot take:

    edge_up     a rise of more than 3.5 cm within 5 cm of route (a step edge)
    edge_down   a drop of more than 12 cm within 15 cm (a ledge; crossed at an angle it spreads)
    slope_up    a climb steeper than 11 deg over 0.4 m
    cross       a side slope over 5 deg held for 1 m (the walking actor slides downhill on it)

Edges closer than ``merge_gap`` are one manoeuvre, padded so the stairs actor holds the robot from
before the front wheels reach the first edge until the rear wheels clear the last. A hairpin inside
one (the route turning more than 45 deg within a metre) splits it, so the turn happens on the
walking actor.

Each manoeuvre carries **priors** for the runtime, never commands: the first edge as a line (fitted
in 2-D to the step-edge cells around the route, so a flight crossed at 60 deg still gets one), its
height, and the deck height after the last edge. The runtime accepts a measured edge only near that
line, and falls back on it only to approach.

The thresholds are the walking actor's limits from the policy profile
(``capability.PolicyProfile``), so a retrained policy that takes more changes where the stairs actor
is needed without touching this code.

Drop-only manoeuvres (a ledge to step off, nothing to climb) are marked for the walking actor: it
steps down 0.25 m square-on at 0.5 m/s, where the stairs actor, slow, jams a wheel on the lip.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field

import numpy as np


@dataclass
class Maneuver:
    id: str
    s0: float  # the stairs actor is wanted from here (first edge - pre)
    s1: float  # ... until here (last edge + post)
    s_first: float
    s_last: float
    policy: str  # "stairs" | "walk_descend"
    kinds: list = field(default_factory=list)
    max_edge_up: float = 0.0
    max_edge_down: float = 0.0
    max_slope_deg: float = 0.0
    max_cross_deg: float = 0.0
    turn_deg: float = 0.0
    rise_m: float = 0.0
    prior_point: list | None = None  # map xy on the first edge
    prior_normal_yaw: float | None = None
    prior_height: float = 0.0
    prior_inliers: int = 0
    deck_z: float | None = None
    first_skew_deg: float | None = None
    #: Arc length of the edge the prior describes (None without one). When it lies well after
    #: s_first (the manoeuvre starts on a slope or a side slope), the runtime hands over at s_first
    #: on the route and does not wait for the edge.
    s_edge: float | None = None
    skew_limit_deg: float = 25.0
    warnings: list = field(default_factory=list)

    @property
    def has_edge(self) -> bool:
        return self.prior_point is not None


def resample(poly, step):
    poly = np.asarray(poly, float)
    seg = np.linalg.norm(np.diff(poly[:, :2], axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    t = np.arange(0.0, s[-1], step)
    return t, np.column_stack([np.interp(t, s, poly[:, k]) for k in range(poly.shape[1])])


def _edge_prior(
    terrain,
    p,
    yaw,
    want_up,
    radius=0.9,
    res=0.05,
    min_rise=0.03,
    cliff=0.3,
    tol=0.05,
    near=0.2,
    min_inliers=8,
    min_span=0.4,
):
    """The edge the route meets at ``p`` (heading ``yaw``) as a 2-D line, in any orientation:
    step-edge cells (a jump over ``min_rise`` across two cells, known ground on both sides, rising
    -- or dropping -- along the heading) around ``p``; every cell proposes the line through it
    square to its own gradient, the proposal passing within ``near`` of ``p`` with the most agreeing
    cells wins (a staircase has several parallel risers in view; this keeps the one the route
    meets), refined by a total-least-squares fit. Returns (point, normal_yaw, height, inliers) in
    map frame, the normal pointing the way the route crosses, or None."""
    ax = np.arange(-radius, radius + 1e-9, res)
    X, Y = np.meshgrid(p[0] + ax, p[1] + ax)
    Z = terrain.ground_at(X, Y)
    K = np.asarray(terrain.known_at(X, Y), bool)
    gx = np.zeros(Z.shape)
    gy = np.zeros(Z.shape)
    gx[:, 1:-1] = Z[:, 2:] - Z[:, :-2]
    gy[1:-1, :] = Z[2:, :] - Z[:-2, :]
    ok = K.copy()
    ok[:, 1:-1] &= K[:, 2:] & K[:, :-2]
    ok[1:-1, :] &= K[2:, :] & K[:-2, :]
    ok[[0, -1], :] = False
    ok[:, [0, -1]] = False
    mag = np.hypot(gx, gy)
    sign = 1.0 if want_up else -1.0
    along = sign * (gx * math.cos(yaw) + gy * math.sin(yaw))
    edge = ok & (mag > min_rise) & (mag < cliff) & (along > 0.2 * mag)
    if int(edge.sum()) < min_inliers:
        return None
    P = np.column_stack([X[edge], Y[edge]])
    N = sign * np.column_stack([gx[edge], gy[edge]]) / mag[edge][:, None]  # crossing direction
    # Every cell proposes a line; score by agreement (distance and direction).
    off = np.einsum("ijk,ik->ij", P[None, :, :] - P[:, None, :], N)  # [proposal, cell]
    agree = (np.abs(off) < tol) & (math.cos(math.radians(20)) < (N @ N.T))
    d_p = np.abs(np.einsum("ik,ik->i", p[None, :2] - P, N))
    score = np.where(d_p <= near, agree.sum(axis=1), -1)
    best = int(np.argmax(score))
    if score[best] < min_inliers:
        return None
    inl = agree[best]
    Q = P[inl]
    c = Q.mean(axis=0)
    _u, _sv, vt = np.linalg.svd(Q - c)
    direction = vt[0]
    span = float(np.ptp((Q - c) @ direction))
    if span < min_span:
        return None
    normal = np.array([-direction[1], direction[0]])
    if normal @ N[inl].mean(axis=0) < 0:
        normal = -normal
    point = c + ((p[:2] - c) @ direction) * direction  # foot of p on the line
    normal_yaw = math.atan2(normal[1], normal[0])
    z_lo = float(terrain.ground_at(*(point - 0.12 * normal)))
    z_hi = float(terrain.ground_at(*(point + 0.12 * normal)))
    return point, normal_yaw, z_hi - z_lo, int(inl.sum())


def annotate(
    terrain,
    line,
    profile=None,
    edge_up=0.035,
    edge_down=0.12,
    slope_deg=11.0,
    slope_run=0.4,
    cross_deg=5.0,
    cross_run=1.0,
    merge_gap=0.8,
    pre=0.8,
    post=0.7,
    step=0.025,
    cliff=0.3,
    split_hairpins=False,
) -> list[Maneuver]:
    """``profile`` (capability.PolicyProfile) sets what the walking actor takes: edge_up, slope_deg
    and cross_deg are its limits; the warnings use the stairs actor's. ``edge_down`` is not a limit
    (the walking actor steps off 0.25 m) but the height from which a ledge is taken square-on.

    With the defaults this is the first version's climb-zone annotation (route_terrain.climb_zones)
    zone for zone. ``split_hairpins`` also splits a manoeuvre at a sharp turn on a landing, for the
    walking actor to turn there -- the first version kept the stairs actor through them, and handing
    over mid-way costs a SafeHold on the real SDK."""
    if profile is not None:
        edge_up, slope_deg, cross_deg = (
            profile.walk.max_step_up,
            profile.walk.max_slope_deg,
            profile.walk.max_cross_deg,
        )
    s, pts = resample(line, step)
    z = terrain.ground_at(pts[:, 0], pts[:, 1])
    known = np.asarray(terrain.known_at(pts[:, 0], pts[:, 1]), bool)
    k = max(1, round(0.05 / step))
    rise = z[k:] - z[:-k]
    both = known[k:] & known[:-k]
    marks = [(s[i], "edge_up", float(rise[i])) for i in np.flatnonzero((rise > edge_up) & both)]
    k3 = max(1, round(0.15 / step))
    fall = z[:-k3] - z[k3:]
    drops = np.flatnonzero((fall > edge_down) & known[:-k3] & known[k3:])
    marks += [(s[i], "edge_down", float(fall[i])) for i in drops]
    n = max(1, round(slope_run / step))
    grade = np.degrees(np.arctan((z[n:] - z[:-n]) / (s[n:] - s[:-n])))
    marks += [
        (s[i] + slope_run / 2, "slope_up", float(grade[i]))
        for i in np.flatnonzero((grade > slope_deg) & known[n:] & known[:-n])
    ]
    tang = np.gradient(pts[:, :2], axis=0)
    tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-9)
    nrm = np.column_stack([-tang[:, 1], tang[:, 0]])
    zl = terrain.ground_at(pts[:, 0] + 0.4 * nrm[:, 0], pts[:, 1] + 0.4 * nrm[:, 1])
    zr = terrain.ground_at(pts[:, 0] - 0.4 * nrm[:, 0], pts[:, 1] - 0.4 * nrm[:, 1])
    kl = np.asarray(
        terrain.known_at(pts[:, 0] + 0.4 * nrm[:, 0], pts[:, 1] + 0.4 * nrm[:, 1]), bool
    )
    kr = np.asarray(
        terrain.known_at(pts[:, 0] - 0.4 * nrm[:, 0], pts[:, 1] - 0.4 * nrm[:, 1]), bool
    )
    cross = np.where(kl & kr, np.degrees(np.arctan(np.abs(zl - zr) / 0.8)), 0.0)
    run = max(1, round(cross_run / step))
    steady = np.convolve(cross > cross_deg, np.ones(run), mode="same") >= run * 0.9
    marks += [(s[i], "cross", float(cross[i])) for i in np.flatnonzero(steady)]
    # The route itself must not cross unknown ground or a jump taller than a riser: that is a route
    # preparation failure, reported on the manoeuvre it falls in (or on its own).
    hazards = [(s[i], "unknown ground") for i in np.flatnonzero(~known)]
    hazards += [
        (s[i], f"{abs(rise[i]):.2f} m jump") for i in np.flatnonzero((np.abs(rise) > cliff) & both)
    ]
    marks.sort()

    groups = []
    for sm, kind, val in marks:
        if groups and sm - groups[-1]["last"] <= merge_gap:
            groups[-1]["last"] = sm
            groups[-1]["marks"].append((sm, kind, val))
        else:
            groups.append({"first": sm, "last": sm, "marks": [(sm, kind, val)]})
    total = float(s[-1])
    for g in groups:
        g["s0"], g["s1"] = max(0.0, g["first"] - pre), min(total, g["last"] + post)
    merged = []
    for g in groups:
        if merged and g["s0"] <= merged[-1]["s1"] + 0.3:
            merged[-1]["last"], merged[-1]["s1"] = g["last"], g["s1"]
            merged[-1]["marks"] += g["marks"]
        else:
            merged.append(g)

    heading = np.unwrap(np.arctan2(tang[:, 1], tang[:, 0]))
    w = max(1, round(0.5 / step))
    out = []
    for g in merged:
        pieces = _split_hairpins(g, s, heading, w) if split_hairpins else [(g["s0"], g["s1"])]
        for n_piece, (p0, p1) in enumerate(pieces):
            # The first piece starts ``pre`` before its first mark; a later one starts past the
            # turn.
            lead = pre if n_piece == 0 else 0.0
            mk = [m for m in g["marks"] if p0 + (pre - lead) - pre - 1e-6 <= m[0] <= p1 + 1e-6]
            if not mk or p1 - p0 < 0.3:
                continue
            m = _make(len(out), p0, p1, mk, s, pts, z, heading, terrain, pre, post, profile)
            out.append(m)
    for sh, what in hazards:
        host = next((m for m in out if m.s0 - 1e-6 <= sh <= m.s1 + 1e-6), None)
        note = f"route crosses {what} at s={sh:.2f}: fix the route before running it"
        if host is not None and not any(w.startswith("route crosses ") for w in host.warnings):
            host.warnings.append(note)
    return out


def route_hazards(terrain, line, step=0.025, cliff=0.3):
    """Arc lengths where the route crosses unknown ground or a jump taller than ``cliff`` (should
    be none)."""
    s, pts = resample(line, step)
    z = terrain.ground_at(pts[:, 0], pts[:, 1])
    known = np.asarray(terrain.known_at(pts[:, 0], pts[:, 1]), bool)
    k = max(1, round(0.05 / step))
    jump = np.abs(z[k:] - z[:-k]) > cliff
    return sorted(
        {round(float(v), 2) for v in s[np.flatnonzero(~known)]}
        | {round(float(v), 2) for v in s[np.flatnonzero(jump & known[k:] & known[:-k])]}
    )


def _split_hairpins(g, s, heading, w, landing=1.2, clear=0.5):
    """Split a manoeuvre at a sharp turn (over 45 deg within a metre) that lies on a landing: an
    edge-free stretch of at least ``landing`` m with the turn at least ``clear`` m from both of its
    edges, where the walking actor has room to turn. A turn between steps stays with the stairs
    actor."""
    a, b = np.searchsorted(s, [g["s0"], g["s1"]])
    edges = np.array([m[0] for m in g["marks"] if m[1] in ("edge_up", "edge_down")])
    pieces, start, i = [], g["s0"], a + w
    while i < b - w:
        if abs(heading[i + w] - heading[i - w]) > math.radians(45):
            hi = min(b - w, i + 2 * w)
            j = i + int(np.argmax(np.abs(np.diff(heading[i:hi])))) if hi - i > 1 else i
            before = edges[(edges <= s[j]) & (edges >= g["s0"])]
            after = edges[(edges > s[j]) & (edges <= g["s1"])]
            e0 = before.max() if before.size else -np.inf
            e1 = after.min() if after.size else np.inf
            if e1 - e0 < landing or s[j] - e0 < clear or e1 - s[j] < clear:
                i += 1
                continue
            end, resume = max(s[j] - 0.2, e0 + 0.6), s[j] + 0.2
            if end >= resume:
                i += 1
                continue
            pieces.append((start, end))
            start, i = resume, j + 2 * w
            continue
        i += 1
    pieces.append((start, g["s1"]))
    return pieces


def _make(k, p0, p1, marks, s, pts, z, heading, terrain, pre, post, profile=None):
    kinds = sorted({m[1] for m in marks})
    up = [m[2] for m in marks if m[1] == "edge_up"]
    down = [m[2] for m in marks if m[1] == "edge_down"]
    slope = [m[2] for m in marks if m[1] == "slope_up"]
    cross = [m[2] for m in marks if m[1] == "cross"]
    s_first = max(p0, min(m[0] for m in marks))
    s_last = min(p1, max(m[0] for m in marks))
    a, b = np.searchsorted(s, [p0, p1])
    b = max(b, a + 1)
    m = Maneuver(
        id=f"M{k:02d}",
        s0=round(p0, 3),
        s1=round(p1, 3),
        s_first=round(s_first, 3),
        s_last=round(s_last, 3),
        policy="walk_descend" if (down and not up and not slope and not cross) else "stairs",
        kinds=kinds,
        max_edge_up=round(max(up, default=0.0), 3),
        max_edge_down=round(max(down, default=0.0), 3),
        max_slope_deg=round(max(slope, default=0.0), 1),
        max_cross_deg=round(max(cross, default=0.0), 1),
        turn_deg=round(float(np.degrees(heading[a:b].max() - heading[a:b].min())), 1),
        rise_m=round(float(z[min(b, len(z) - 1)] - z[a]), 3),
    )
    # The prior is the edge the runtime will gate on: the first rise for the stairs actor, the first
    # drop for a ledge the walking actor steps off.
    want_kind = "edge_down" if m.policy == "walk_descend" else "edge_up"
    edge_marks = sorted(mm for mm in marks if mm[1] == want_kind)
    if edge_marks:
        sf, first_kind, _ = edge_marks[0]
        m.s_edge = round(float(sf), 3)
        i = int(np.clip(np.searchsorted(s, sf), 0, len(s) - 1))
        prior = _edge_prior(terrain, pts[i], float(heading[i]), want_up=first_kind == "edge_up")
        if prior is not None:
            point, normal, height, inl = prior
            m.prior_point = [round(float(point[0]), 3), round(float(point[1]), 3)]
            m.prior_normal_yaw = round(float(normal), 4)
            m.prior_height = round(float(height), 3)
            m.prior_inliers = inl
            skew = math.degrees(abs((normal - heading[i] + math.pi) % (2 * math.pi) - math.pi))
            m.first_skew_deg = round(skew, 1)
            if skew > 10:
                m.warnings.append(
                    f"route crosses the first edge {skew:.0f} deg off square; the "
                    "runtime aligns to the measured edge, check the approach has room"
                )
        else:
            m.warnings.append(
                "no straight edge found around the route's first edge: the runtime gates the "
                "entry on an edge it measures near the route, with a wide gate"
            )
    j = int(np.clip(np.searchsorted(s, p1), 0, len(s) - 1))
    m.deck_z = round(float(z[j]), 3)
    climb_step = profile.climb.max_step_up if profile is not None else 0.18
    climb_turns = profile.climb.turns_on_stairs if profile is not None else False
    skew_ok = profile.climb.max_entry_skew_deg if profile is not None else 25.0
    if m.max_edge_up > climb_step:
        m.warnings.append(
            f"edge {m.max_edge_up:.2f} m is above the stairs actor's tested {climb_step:.2f} m"
        )
    if m.turn_deg > 25 and m.policy == "stairs" and not climb_turns:
        m.warnings.append(
            f"route turns {m.turn_deg:.0f} deg inside the climb; the stairs actor barely turns"
        )
    m.skew_limit_deg = skew_ok
    return m


def save(path, maneuvers, route_id="", source=""):
    doc = {
        "schema": "s10_rl_maneuvers_v1",
        "route": route_id,
        "source": source,
        "maneuvers": [asdict(m) for m in maneuvers],
    }
    with open(path, "w") as f:
        json.dump(doc, f, indent=1)


def load(path) -> list[Maneuver]:
    with open(path) as f:
        doc = json.load(f)
    if doc.get("schema") != "s10_rl_maneuvers_v1":
        raise ValueError(f"{path}: not an s10_rl_maneuvers_v1 file")
    return [Maneuver(**m) for m in doc["maneuvers"]]
