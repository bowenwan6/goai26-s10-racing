#!/usr/bin/env python3
"""Turn a 采集助手 (/teach) session into the files the navigation node loads.

  python3 tools/teach_to_route.py <session_dir> --map-id <x_nav map name> --out <route_dir>
        [--path-recording <name>] [--body-z-offset 0.41] [--flat-speed 1.0] [--stairs-speed 0.30]
  python3 tools/teach_to_route.py --straight X Y Z YAW LENGTH --map-id <map> --out <route_dir>

Session input (written by tools/s10_mapping_web/teach_worker.py on the AGX, ~/teach/sessions/<id>/):
  marks.jsonl        WP01..WP30 marks (3 s still samples, pass/fail), SWIN/SWOUT pairs, VOID rows
  <path_*>.trail.csv the taught path (wall_time,x,y,z,yaw), one row every 5 cm

Output (route_dir):
  route_v2.json      s10_route_v2: waypoints = last passing mark of each WP (z = ground:
                     pose z - body_z_offset), centreline = the taught trail between them,
                     gait stairs where a SWIN..SWOUT zone overlaps the segment
  maneuvers.json     s10_rl_maneuvers_v1: one stairs zone per SWIN/SWOUT pair (arc length on
                     the route; no edge prior -> the runner hands over at the zone start)
  report.json        what was used, what was skipped, re-test differences, warnings

--straight makes a two-waypoint route LENGTH m straight ahead of the given pose (the first
motion test); it needs no session.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "nav"))
from s10_auto_nav.rl_nav.maneuvers import Maneuver, save as save_maneuvers  # noqa: E402
from s10_auto_nav.route_v2 import RoutePath, RouteV2  # noqa: E402

WP_IDS = ["WP%02d" % i for i in range(1, 31)]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def read_marks(session):
    rows = []
    with open(session / "marks.jsonl") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    voided = {r["target"] for r in rows if r.get("kind") == "VOID"}
    live = [r for r in rows if r.get("kind") != "VOID" and r["seq"] not in voided and not r.get("void")]
    return rows, live


def read_trail(path):
    pts = []
    with open(path) as f:
        for r in csv.DictReader(f):
            pts.append([float(r["wall_time"]), float(r["x"]), float(r["y"]), float(r["z"]), float(r["yaw"])])
    a = np.asarray(pts, float)
    if len(a) < 2:
        raise SystemExit(f"{path}: fewer than 2 trail points")
    return a


def resample_xy(line, step=0.20):
    """Resample a 3-D polyline at `step` horizontal spacing, keeping both ends."""
    line = np.asarray(line, float)
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(line[:, :2], axis=0), axis=1))]
    if d[-1] < 1e-6:
        return line[[0, -1]]
    s = np.arange(0.0, d[-1], step)
    if d[-1] - s[-1] < step * 0.25:
        s = s[:-1]
    s = np.r_[s, d[-1]]
    return np.column_stack([np.interp(s, d, line[:, k]) for k in range(3)])


def nearest_index(trail_xy, p, start):
    d = np.linalg.norm(trail_xy[start:, :2] - np.asarray(p[:2]), axis=1)
    return start + int(np.argmin(d)), float(d.min())


def build_from_session(args):
    session = Path(args.session).expanduser()
    rows, live = read_marks(session)
    report = dict(session=str(session), marks_total=len(rows), marks_live=len(live), skipped=[], retests={}, warnings=[])
    # --- waypoints: last passing mark per WP, earlier passing marks recorded as re-tests
    wps = {}
    for r in live:
        if r.get("kind") != "WP":
            continue
        res = r.get("result") or {}
        if not res.get("passed"):
            report["skipped"].append(dict(seq=r["seq"], wp=r["wp_id"], reasons=res.get("reasons")))
            continue
        if r["wp_id"] in wps:
            prev = wps[r["wp_id"]]
            dxy = math.hypot(r["pose"][0] - prev["pose"][0], r["pose"][1] - prev["pose"][1])
            report["retests"].setdefault(r["wp_id"], []).append(dict(dxy_m=round(dxy, 3), dz_m=round(r["pose"][2] - prev["pose"][2], 3),
                                                                     dyaw_deg=round(math.degrees(wrap(r["pose"][3] - prev["pose"][3])), 2)))
        wps[r["wp_id"]] = r
    ids = [w for w in WP_IDS if w in wps]
    if args.last_wp:
        ids = [w for w in ids if w <= args.last_wp]   # shorter test route; zones beyond it are dropped below
    if len(ids) < 2:
        raise SystemExit("fewer than 2 passing WP marks")
    missing = [w for w in WP_IDS if w not in wps]
    if missing:
        report["warnings"].append(f"WPs without a passing mark (skipped in the chain): {missing}")
    # --- the taught path
    trails = sorted(session.glob("path_*.trail.csv"))
    if args.path_recording:
        trails = [session / (args.path_recording + ".trail.csv")]
    if not trails:
        raise SystemExit("no path_*.trail.csv in the session (run ③ 示教 first)")
    trail = read_trail(trails[-1])
    report["path_recording"] = trails[-1].name
    txy = trail[:, 1:4]
    # --- segments: trail between consecutive WPs, monotone along the trail
    zoff = float(args.body_z_offset)
    waypoints, segments, cursor = [], [], 0
    idx = []
    for w in ids:
        i, dist = nearest_index(txy, wps[w]["pose"], cursor)
        if dist > 1.0:
            report["warnings"].append(f"{w}: taught path passes {dist:.2f} m from the mark")
        idx.append(i)
        cursor = i
    for k, w in enumerate(ids):
        p = wps[w]["pose"]
        waypoints.append(dict(id=w, position=[round(p[0], 3), round(p[1], 3), round(p[2] - zoff, 3)], yaw=round(p[3], 4),
                              radius_xy=float(wps[w].get("radius_xy") or args.radius), tol_z=float(args.tol_z), terrain="", confidence="high",
                              mark_seq=wps[w]["seq"], mark_std_xy=wps[w]["result"].get("std_xy")))
    # --- SWIN/SWOUT zones on the trail
    # Paired in the order they lie ALONG THE TAUGHT PATH, not the order they were pressed: a
    # switch point marked on the way back (SWOUT first, then SWIN) is still one zone.
    zones_xy, open_ = [], None
    sw = [r for r in live if r.get("kind") in ("SWIN", "SWOUT") and (r.get("result") or {}).get("passed")]
    along = sorted(sw, key=lambda r: nearest_index(txy, r["pose"], 0)[0])
    if [r["seq"] for r in along] != [r["seq"] for r in sw]:
        report["warnings"].append("switch marks re-ordered by position on the taught path: seq " +
                                  ", ".join(f"{r['seq']}:{r['kind']}" for r in along))
    for r in along:
        if r["kind"] == "SWIN":
            if open_ is not None:
                report["warnings"].append(f"SWIN seq {open_['seq']} without SWOUT: ignored")
            open_ = r
        elif open_ is not None:
            zones_xy.append((open_, r))
            open_ = None
        else:
            report["warnings"].append(f"SWOUT seq {r['seq']} without SWIN: ignored")
    if open_ is not None:
        report["warnings"].append(f"SWIN seq {open_['seq']} without SWOUT: ignored")
    zone_idx = []
    for a, b in zones_xy:
        ia, da = nearest_index(txy, a["pose"], 0)
        ib, db = nearest_index(txy, b["pose"], ia)
        if args.last_wp and ib > idx[-1]:
            report["warnings"].append(f"switch pair seq {a['seq']}/{b['seq']} lies beyond {ids[-1]}: dropped (--last-wp)")
            continue
        zone_idx.append((ia, ib))
        if max(da, db) > 1.0:
            report["warnings"].append(f"switch pair seq {a['seq']}/{b['seq']} is {max(da, db):.2f} m off the taught path")
    for k in range(len(ids) - 1):
        i0, i1 = idx[k], idx[k + 1]
        if i1 <= i0:
            raise SystemExit(f"taught path does not go {ids[k]} -> {ids[k + 1]} in order (trail indices {i0} >= {i1})")
        line = txy[i0:i1 + 1].copy()
        line[:, 2] -= zoff
        line[0, :2] = waypoints[k]["position"][:2]
        line[-1, :2] = waypoints[k + 1]["position"][:2]
        line = resample_xy(line, float(args.step))
        # stairs segment = a switch zone covers at least half of it. A zone that only clips the
        # end of a long flat segment leaves it flat (full flat speed, detours allowed); the runner
        # still hands over to the stairs gait at the zone start, by arc length.
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(np.asarray(txy)[:, :2], axis=0), axis=1))]
        cover = sum(max(0.0, arc[min(ib, i1)] - arc[max(ia, i0)]) for ia, ib in zone_idx if min(ib, i1) > max(ia, i0))
        stairs = cover >= 0.5 * (arc[i1] - arc[i0])
        length = float(np.sum(np.linalg.norm(np.diff(line[:, :2], axis=0), axis=1)))
        segments.append(dict(id=f"{ids[k]}-{ids[k + 1]}", **{"from": ids[k], "to": ids[k + 1]},
                             gait="stairs" if stairs else "flat",
                             speed_limit=float(args.stairs_speed if stairs else args.flat_speed),
                             allow_detour=not stairs, corridor_half_width=0.5 if stairs else 0.8,
                             length_m=round(length, 2), centerline=[[round(float(v), 3) for v in p] for p in line]))
    doc = dict(schema="s10_route_v2", map_id=args.map_id, frame="map", z_reference="ground",
               status=f"FIELD: WPs and taught path from teach session {session.name}; body_z_offset {zoff}",
               order="official WP01 = door Start, WP30 = far end", source=dict(session=str(session), path_recording=trails[-1].name,
                                                                                body_z_offset=zoff),
               waypoints=waypoints, segments=segments)
    return doc, zone_idx, txy, report


def build_straight(args):
    x, y, z, yaw, length = (float(v) for v in args.straight)
    z -= float(args.body_z_offset)
    p1 = [x + length * math.cos(yaw), y + length * math.sin(yaw), z]
    line = resample_xy(np.array([[x, y, z], p1]), 0.20)
    doc = dict(schema="s10_route_v2", map_id=args.map_id, frame="map", z_reference="ground",
               status=f"STRAIGHT {length} m test route from ({x:.2f},{y:.2f}) yaw {yaw:.2f}",
               waypoints=[dict(id="WP01", position=[round(x, 3), round(y, 3), round(z, 3)], yaw=round(yaw, 4), radius_xy=float(args.radius), tol_z=float(args.tol_z), confidence="high"),
                          dict(id="WP02", position=[round(v, 3) for v in p1], yaw=round(yaw, 4), radius_xy=float(args.radius), tol_z=float(args.tol_z), confidence="high")],
               segments=[dict(id="WP01-WP02", **{"from": "WP01", "to": "WP02"}, gait="flat", speed_limit=float(args.flat_speed),
                              allow_detour=False, corridor_half_width=0.5, centerline=[[round(float(v), 3) for v in p] for p in line])])
    return doc, [], None, dict(straight=args.straight, warnings=[])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", nargs="?")
    ap.add_argument("--straight", nargs=5, metavar=("X", "Y", "Z", "YAW", "LENGTH"))
    ap.add_argument("--map-id", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--path-recording", default="")
    ap.add_argument("--step", type=float, default=0.20, help="centreline spacing, m (teach_line routes: 0.10, so the route file is the verified line)")
    ap.add_argument("--last-wp", default="", help="stop the route at this WP (e.g. WP02 for a flat first test)")
    ap.add_argument("--body-z-offset", type=float, default=0.41)
    ap.add_argument("--flat-speed", type=float, default=1.0, help="per-segment cap; the run speed is set at arm time")
    ap.add_argument("--stairs-speed", type=float, default=0.30)
    ap.add_argument("--radius", type=float, default=0.20)
    ap.add_argument("--tol-z", type=float, default=0.30)
    ap.add_argument("--pre", type=float, default=0.3, help="stairs zone starts this far before SWIN (m)")
    ap.add_argument("--post", type=float, default=0.3, help="... and ends this far after SWOUT (m)")
    args = ap.parse_args()
    if not args.session and not args.straight:
        ap.error("a session dir or --straight is required")
    doc, zone_idx, txy, report = build_straight(args) if args.straight else build_from_session(args)
    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    route_path = out / "route_v2.json"
    with open(route_path, "w") as f:
        json.dump(doc, f, indent=1)
    route = RouteV2.load(route_path)  # validates
    path = RoutePath(route)
    zones = []
    for n, (ia, ib) in enumerate(zone_idx):
        s0 = path.project(txy[ia][:2]) if hasattr(path, "project") else None
        # arc length of the SWIN/SWOUT points along the built route
        sa = float(_s_on_path(path, txy[ia][:2]))
        sb = float(_s_on_path(path, txy[ib][:2]))
        zones.append(Maneuver(id=f"SW{n:02d}", s0=max(0.0, sa - args.pre), s1=min(path.length, sb + args.post), s_first=sa, s_last=sb,
                              policy="stairs", kinds=["teach:SWIN/SWOUT"], warnings=["no edge prior (from teach marks): handover at s_first"]))
        _ = s0
    save_maneuvers(out / "maneuvers.json", zones, route_id=args.map_id, source="teach_to_route")
    tj = Path(args.session).expanduser() / "terrain.json" if args.session else None
    if tj is not None and tj.exists():                     # teach_line: steep stretches (line arc length -> route arc length), contact waypoints
        td = json.loads(tj.read_text())
        trail_s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(np.asarray(txy)[:, :2], axis=0), axis=1))]
        def to_route(sv):
            k = int(np.argmin(np.abs(trail_s - sv))); near = np.abs(np.asarray(path.s, float) - sv) < 8.0   # same stretch of the course, not the way back
            pts = np.asarray(path.points if hasattr(path, "points") else path.xyz, float)[:, :2]
            dd = np.where(near, np.linalg.norm(pts - np.asarray(txy[k][:2], float), axis=1), np.inf)
            return round(float(np.asarray(path.s, float)[int(np.argmin(dd))]), 2)
        L_ = float(path.length)                              # a shortened route (--last-wp): what lies beyond its end is dropped
        td["steep"] = [[to_route(a_), to_route(min(b_, L_))] for a_, b_ in td.get("steep", []) if a_ < L_ - 0.5]
        td["gaits"] = [[to_route(a_), to_route(min(b_, L_)), n_] for a_, b_, n_ in td.get("gaits", []) if a_ < L_ - 0.5]
        keep_ids = {w.id for w in route.waypoints}
        td["contact"] = [q for q in td.get("contact", []) if q["id"] in keep_ids]
        (out / "terrain.json").write_text(json.dumps(td, indent=1))
    report.update(waypoints=len(route.waypoints), segments=len(route.segments), length_m=round(float(path.length), 1),
                  stairs_segments=[s.id for s in route.segments if s.gait == "stairs"],
                  zones=[dict(id=z.id, s0=round(z.s0, 2), s1=round(z.s1, 2)) for z in zones])
    with open(out / "report.json", "w") as f:
        json.dump(report, f, indent=1, ensure_ascii=False)
    print(json.dumps(report, indent=1, ensure_ascii=False))
    print(f"wrote {route_path}, {out / 'maneuvers.json'}, {out / 'report.json'}")


def _s_on_path(path, xy):
    pts = np.asarray(path.points if hasattr(path, "points") else path.xyz, float)
    d = np.linalg.norm(pts[:, :2] - np.asarray(xy, float), axis=1)
    i = int(np.argmin(d))
    s = np.asarray(path.s, float)
    return s[i]


if __name__ == "__main__":
    main()
