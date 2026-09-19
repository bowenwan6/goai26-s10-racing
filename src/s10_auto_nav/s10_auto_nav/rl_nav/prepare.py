"""Offline preparation for the robot: the robot's map raster and route_v2 in, everything ``rl_nav``
loads out.

    ros2 run s10_auto_nav rl_nav_prepare --route route_v2.json --terrain course_terrain.npz \\
        [--overrides overrides.json] [--profile policy_profile.json] [--method first|centre] \\
        --out prepared/

writes

    route_rl.json      the route handed to the follower and the runner. --method first (default):
                       the first version's preparation (route_prep.sanitize_route: waypoints off
                       hazards, the line shifted into the clear; hand-validated centrelines from
                       --overrides kept as they are) -- the route the full course ran on. centre:
                       route_prep.centre_route (centred on steps, re-planned, banded).
    maneuvers.json     where the stairs actor is needed (maneuvers.annotate)
    map_surface.npz    the map's surface, for telling terrain the map knows from something new
    prepare_report.json  every change route preparation made, every manoeuvre warning, and any
                       place the route still crosses unknown ground or a drop (there should be none)

The terrain is the nav simulation's ``course_terrain.npz`` (``sim_full_course.build_terrain``:
ground, known mask and obstacle layer from the v3 point cloud) or a ``MapSurface`` file. Nothing
here reads a simulator.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path

import numpy as np

from s10_auto_nav.rl_nav import maneuvers as maneuver_io
from s10_auto_nav.rl_nav.capability import PolicyProfile
from s10_auto_nav.rl_nav.map_check import MapSurface
from s10_auto_nav.rl_nav.route_prep import centre_route, sanitize_route
from s10_auto_nav.route_v2 import RoutePath, RouteV2


class MapTerrain:
    """The robot's map raster with the lookups route preparation needs (bilinear ground, mask)."""

    def __init__(self, ground, known, obstacle_height, origin, res):
        self.ground = np.asarray(ground, float)
        self.known = np.asarray(known, bool)
        self.obstacle_height = np.nan_to_num(np.asarray(obstacle_height, float))
        self.origin = (float(origin[0]), float(origin[1]))
        self.res = float(res)

    @classmethod
    def load(cls, path) -> MapTerrain:
        z = np.load(path)
        if "ground_filled" in z:  # sim_full_course.build_terrain
            obst = np.where(z["obstacle"], z["obstacle_height"], 0.0) if "obstacle" in z else 0.0
            return cls(
                z["ground_filled"],
                z["known"],
                obst + np.zeros(z["known"].shape),
                tuple(z["origin"]),
                float(z["resolution"]),
            )
        return cls(
            z["ground"], z["known"], z["obstacle_height"], tuple(z["origin"]), float(z["res"])
        )

    def ground_at(self, x, y):
        ny, nx = self.ground.shape
        fx = np.clip((np.asarray(x, float) - self.origin[0]) / self.res - 0.5, 0, nx - 1)
        fy = np.clip((np.asarray(y, float) - self.origin[1]) / self.res - 0.5, 0, ny - 1)
        x0 = np.minimum(np.floor(fx).astype(int), nx - 2)
        y0 = np.minimum(np.floor(fy).astype(int), ny - 2)
        ax, ay = fx - x0, fy - y0
        g = self.ground
        return (
            (1 - ax) * (1 - ay) * g[y0, x0]
            + ax * (1 - ay) * g[y0, x0 + 1]
            + (1 - ax) * ay * g[y0 + 1, x0]
            + ax * ay * g[y0 + 1, x0 + 1]
        )

    def known_at(self, x, y):
        ny, nx = self.known.shape
        ix = np.floor((np.asarray(x, float) - self.origin[0]) / self.res).astype(int)
        iy = np.floor((np.asarray(y, float) - self.origin[1]) / self.res).astype(int)
        inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        return self.known[np.clip(iy, 0, ny - 1), np.clip(ix, 0, nx - 1)] & inside


def apply_overrides(route, overrides, terrain, ds=0.2):
    """Hand-validated edits: {"waypoints": {id: [x, y]}, "centerlines": {id: [[x, y], ...]},
    "centerline_tails": {id: {"from": [x, y], "points": [[x, y], ...]}}} (a tail keeps the taught
    line up to the point nearest ``from``, then the given points). A centreline is densified at
    ``ds``, re-grounded and snapped to its (possibly moved) waypoints. -> (route, ids of the
    segments given whole centrelines, edits)."""
    r = copy.deepcopy(route)
    ids = [w["id"] for w in r["waypoints"]]
    edits = []
    for wid, xy in overrides.get("waypoints", {}).items():
        if wid in ids:
            wp = r["waypoints"][ids.index(wid)]
            old = wp["position"][:2]
            wp["position"] = [float(xy[0]), float(xy[1]), float(terrain.ground_at(xy[0], xy[1]))]
            edits.append(
                {"waypoint": wid, "shift_m": round(math.hypot(xy[0] - old[0], xy[1] - old[1]), 3)}
            )
    frozen = set()
    for k, seg in enumerate(r["segments"]):
        ctrl = overrides.get("centerlines", {}).get(seg["id"])
        tail = overrides.get("centerline_tails", {}).get(seg["id"])
        whole = ctrl is not None
        if ctrl is None and tail is not None:
            orig = np.asarray(seg["centerline"], float)[:, :2]
            cut = int(np.argmin(np.linalg.norm(orig - np.asarray(tail["from"], float), axis=1)))
            ctrl = np.vstack([orig[: cut + 1], np.asarray(tail["points"], float)]).tolist()
        if ctrl is None:
            continue
        ctrl = np.asarray(ctrl, float)
        ctrl[0] = r["waypoints"][k]["position"][:2]
        ctrl[-1] = r["waypoints"][k + 1]["position"][:2]
        d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(ctrl, axis=0), axis=1))]
        u = np.linspace(0.0, d[-1], max(2, math.ceil(d[-1] / ds) + 1))
        c = np.column_stack([np.interp(u, d, ctrl[:, 0]), np.interp(u, d, ctrl[:, 1])])
        seg["centerline"] = np.column_stack([c, terrain.ground_at(c[:, 0], c[:, 1])]).tolist()
        if whole:
            frozen.add(seg["id"])
        edits.append({"segment": seg["id"], "points": len(ctrl)})
    return r, frozen, edits


def prepare(route, terrain, profile, overrides=None, method="first"):
    frozen, edits = set(), []
    if overrides:
        route, frozen, edits = apply_overrides(route, overrides, terrain)
    # Segment flags (allow_detour, corridor) stay as taught, as the first version ran them: the
    # follower's own grid planner does not detour on the steps; the runner plans detours on the map.
    if method == "centre":
        prepared, report = centre_route(route, terrain, profile, frozen=frozen)
    else:
        prepared, report = sanitize_route(route, terrain, skip=frozen)
    path = RoutePath(RouteV2.from_dict(prepared))
    mans = maneuver_io.annotate(terrain, path.points, profile)
    hazards = maneuver_io.route_hazards(terrain, path.points)
    report = {
        "overrides": edits,
        "route_prep": report,
        "hazards_s": hazards,
        "maneuvers": [
            {
                "id": m.id,
                "s0": m.s0,
                "s1": m.s1,
                "policy": m.policy,
                "kinds": m.kinds,
                "warnings": m.warnings,
            }
            for m in mans
        ],
    }
    return prepared, mans, report


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--route", required=True)
    ap.add_argument("--terrain", required=True)
    ap.add_argument("--overrides", default="")
    ap.add_argument("--profile", default="")
    ap.add_argument("--method", choices=("first", "centre"), default="first")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    terrain = MapTerrain.load(args.terrain)
    profile = PolicyProfile.load(args.profile) if args.profile else PolicyProfile.default()
    with open(args.route) as f:
        route = json.load(f)
    overrides = None
    if args.overrides:
        with open(args.overrides) as f:
            overrides = json.load(f)
    prepared, mans, report = prepare(route, terrain, profile, overrides, args.method)
    (out / "route_rl.json").write_text(json.dumps(prepared, indent=1))
    maneuver_io.save(out / "maneuvers.json", mans, prepared.get("map_id", ""), str(args.terrain))
    MapSurface(
        terrain.ground, terrain.known, terrain.obstacle_height, terrain.origin, terrain.res
    ).save(out / "map_surface.npz")
    (out / "prepare_report.json").write_text(json.dumps(report, indent=1, ensure_ascii=False))
    warn = sum(len(m["warnings"]) for m in report["maneuvers"])
    print(
        f"{len(prepared['waypoints'])} waypoints, {len(mans)} manoeuvres ({warn} warnings), "
        f"{len(report['hazards_s'])} route samples on unknown ground or a drop -> {out}"
    )


if __name__ == "__main__":
    main()
