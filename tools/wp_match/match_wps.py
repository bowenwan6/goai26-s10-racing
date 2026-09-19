"""Bind the 30 waypoint photos to v3 map coordinates and build a route_v2 draft.

The photos were taken during the v3 mapping run (2026-09-14 14:43:58-14:54:55 CST) with the robot
in frame, so each photo time indexes the saved keyframe trajectory. The red markers themselves are
not visible to the LiDAR; the point cloud is only used to put Z on the ground and to measure the
terrain along each segment.

Official order runs the photos backwards: official WP01 is the last photo (door, low Start) and
official WP30 the first photo (far, high end). See ROUTE_V2_CONTRACT.md.

Run: python3 tools/wp_match/match_wps.py --out tools/wp_match/out
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import median_filter
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

HERE = Path(__file__).resolve().parent
MAP_ID = "0914_fr_v3-20260914-142008"
DEFAULT_RAW = Path(
    "<home>/Documents/ChatGPT/GOAI/map-reviews/0914_fr_v3-20260914-142008/raw/"
    "0914_fr_v3-20260914-142008"
)

# Visual review of every photo (2026-09-19). `robot` = robot visible in frame; `gap_m` = rough
# robot-to-marker distance read from the image; `hard` = terrain that must be driven in stairs gait
# (True: segments on both sides; "out": only the segment leaving this WP in official order).
PHOTO_NOTES = {
    "IMG_6988.HEIC": dict(robot=False, gap_m=None, terrain="pavement_fork", hard=False,
                          marker="red dot on granular pavement before a path fork"),
    "IMG_6989.HEIC": dict(robot=True, gap_m=1.0, terrain="dry_grass", hard=False,
                          marker="red flag on dry grass, robot about 1 m in front"),
    "IMG_6990.HEIC": dict(robot=False, gap_m=None, terrain="gabion_top", hard=True,
                          marker="red dot on gabion top; a second flag on the grass behind it"),
    "IMG_6991.HEIC": dict(robot=False, gap_m=None, terrain="gabion_top", hard=True,
                          marker="red dot on the gabion top, close-up of the ledge edge"),
    "IMG_6992.HEIC": dict(robot=True, gap_m=1.0, terrain="dry_grass", hard=False,
                          marker="red flag on dry grass next to the robot's leg"),
    # Hard: the route climbs from this pavement edge onto the grass mound (~0.24 m kerb in the
    # course heightfield; the 1.8 m median profile alone smooths it to a 0.12 grade).
    "IMG_6993.HEIC": dict(robot=False, gap_m=None, terrain="pavement_edge_mound", hard="out",
                          marker="red dot on the granular pavement edge below a grass mound"),
    "IMG_6994.HEIC": dict(robot=False, gap_m=None, terrain="asphalt_curve", hard=False,
                          marker="red dot on a wide asphalt curve"),
    "IMG_6995.HEIC": dict(robot=True, gap_m=1.5, terrain="asphalt", hard=False,
                          marker="red dot on asphalt beside the robot"),
    "IMG_6996.HEIC": dict(robot=True, gap_m=1.5, terrain="asphalt_branch", hard=False,
                          marker="red dot on asphalt near a side-path branch"),
    "IMG_6997.HEIC": dict(robot=True, gap_m=0.5, terrain="asphalt_curve", hard=False,
                          marker="red dot at the robot's front leg"),
    "IMG_6998.HEIC": dict(robot=True, gap_m=2.0, terrain="low_platform_bumps", hard=True,
                          marker="red dot on a stone platform edge between yellow bumps"),
    "IMG_6999.HEIC": dict(robot=True, gap_m=1.0, terrain="stepped_platform", hard=True,
                          marker="red dot on a platform step, yellow bumps on the next tier"),
    "IMG_7001.HEIC": dict(robot=True, gap_m=0.5, terrain="plaza", hard=False,
                          marker="red dot on plaza tiles right by the robot"),
    "IMG_7002.HEIC": dict(robot=True, gap_m=1.5, terrain="plaza", hard=False,
                          marker="red dot on plaza tiles near the pavilion deck"),
    "IMG_7003.HEIC": dict(robot=False, gap_m=None, terrain="rock_bed_entry", hard=True,
                          marker="red flags in the creek bed beside the wooden deck (two visible)"),
    "IMG_7004.HEIC": dict(robot=True, gap_m=1.0, terrain="rock_bed", hard=True,
                          marker="red dot on a flat boulder just ahead of the robot"),
    "IMG_7005.HEIC": dict(robot=True, gap_m=1.0, terrain="flat_boulder_top", hard=True,
                          marker="red dot on top of the flat boulder, robot behind it"),
    "IMG_7006.HEIC": dict(robot=True, gap_m=0.7, terrain="loose_cobbles", hard=True,
                          marker="red flag in cobbles beside the robot"),
    "IMG_7007.HEIC": dict(robot=True, gap_m=2.0, terrain="rock_grass", hard=True,
                          marker="red marker in tall grass in front of the robot"),
    "IMG_7008.HEIC": dict(robot=True, gap_m=1.5, terrain="grass_to_pavement", hard=True,
                          marker="red dot on pavement edge, robot still in the grass"),
    "IMG_7009.HEIC": dict(robot=True, gap_m=1.0, terrain="low_steps", hard=True,
                          marker="red dot on the upper step next to the robot"),
    "IMG_7010.HEIC": dict(robot=True, gap_m=1.2, terrain="low_steps", hard=True,
                          marker="red dot on the upper step ahead of the robot"),
    "IMG_7011.HEIC": dict(robot=True, gap_m=2.0, terrain="step_edge", hard=True,
                          marker="red dot on a step edge, robot about 2 m beyond it"),
    "IMG_7012.HEIC": dict(robot=True, gap_m=0.4, terrain="stairs_railing", hard=True,
                          marker="red dot on a stair tread at the robot's front wheel"),
    "IMG_7013.HEIC": dict(robot=True, gap_m=0.5, terrain="stairs", hard=True,
                          marker="red dot at the step edge the robot straddles"),
    "IMG_7014.HEIC": dict(robot=True, gap_m=0.4, terrain="stair_landing", hard=True,
                          marker="red dot on the landing beside the robot"),
    "IMG_7015.HEIC": dict(robot=True, gap_m=0.4, terrain="curved_stairs", hard=True,
                          marker="red dot on a curved stair tread beside the robot"),
    "IMG_7016.HEIC": dict(robot=True, gap_m=0.4, terrain="stairs_foot", hard=False,
                          marker="red dot on flat ground beside the robot, stairs to the right"),
    "IMG_7017 2.HEIC": dict(robot=True, gap_m=0.4, terrain="door_plaza", hard=False,
                            marker="red dot on flat ground beside the robot"),
    "IMG_7018 2.HEIC": dict(robot=True, gap_m=0.5, terrain="door_start", hard=False,
                            marker="red dot outside the glass doors beside the robot"),
}

# Robot-to-marker distance plus clock uncertainty (the robot's clock sync to the phone is
# unverified; ~3 s at ~0.5 m/s adds ~1.5 m along track).
CLOCK_SLACK_M = 1.0


def load_photos(path: Path) -> list[dict]:
    rows = []
    for line in path.read_text().splitlines():
        name, created, lat, lon, _gps, heading = line.split("|")
        t = dt.datetime.strptime(created, "%Y-%m-%d %H:%M:%S +0000").replace(tzinfo=dt.UTC)
        rows.append(dict(photo=name, epoch=t.timestamp(), lat=float(lat), lon=float(lon),
                         camera_heading_deg=float(heading)))
    rows.sort(key=lambda r: r["epoch"])
    return rows


def load_pcd_xyz(path: Path) -> np.ndarray:
    header = {}
    with path.open("rb") as f:
        while True:
            words = f.readline().decode("ascii").split()
            if words and not words[0].startswith("#"):
                header[words[0]] = words[1:]
                if words[0] == "DATA":
                    break
        offset = f.tell()
    assert header["DATA"] == ["binary"] and header["FIELDS"][:3] == ["x", "y", "z"]
    n = int(header["POINTS"][0])
    width = len(header["FIELDS"])
    data = np.memmap(path, mode="r", dtype="<f4", offset=offset, shape=(n, width))
    return np.asarray(data[:, :3], dtype=np.float64)


class Trajectory:
    def __init__(self, poses: np.ndarray):
        self.t = poses[:, 0]
        self.xyz = poses[:, 1:4]
        self.slerp = Slerp(self.t, Rotation.from_quat(poses[:, 4:8]))

    def at(self, t: float) -> tuple[np.ndarray, float, float]:
        xyz = np.array([np.interp(t, self.t, self.xyz[:, k]) for k in range(3)])
        yaw, pitch, _roll = self.slerp([t]).as_euler("ZYX")[0]
        return xyz, float(yaw), float(pitch)

    def nearest_gap(self, t: float) -> float:
        i = np.searchsorted(self.t, t)
        return float(min(abs(self.t[max(i - 1, 0)] - t), abs(self.t[min(i, len(self.t) - 1)] - t)))

    def between(self, t0: float, t1: float) -> np.ndarray:
        """Positions from t0 to t1 (either order), including interpolated end points."""
        lo, hi = min(t0, t1), max(t0, t1)
        inner = self.xyz[(self.t > lo) & (self.t < hi)]
        path = np.vstack([self.at(lo)[0], inner, self.at(hi)[0]])
        return path if t0 <= t1 else path[::-1]


class Ground:
    """Ground height from the map cloud, restricted to a corridor around the driven path."""

    def __init__(self, cloud: np.ndarray, path_xyz: np.ndarray, corridor: float = 4.0):
        tree = cKDTree(path_xyz[:, :2])
        d, _ = tree.query(cloud[:, :2], distance_upper_bound=corridor)
        self.pts = cloud[np.isfinite(d)]
        self.tree = cKDTree(self.pts[:, :2])

    def at(self, xy, ref_z: float, radius: float = 0.3) -> tuple[float, int]:
        """Robust ground below a robot reference height `ref_z` (base is ~0.43 m above ground)."""
        idx = self.tree.query_ball_point(xy, radius)
        if not idx:
            return math.nan, 0
        z = self.pts[idx, 2]
        z = z[(z > ref_z - 1.2) & (z < ref_z - 0.15)]
        if len(z) < 5:
            return math.nan, len(z)
        # The v3 surface is ~0.3 m thick (noise below the true ground), so neither the minimum
        # nor a low percentile is the ground; take the densest 4 cm layer instead.
        bins = np.arange(z.min(), z.max() + 0.08, 0.04)
        hist, edges = np.histogram(z, bins=bins)
        k = int(np.argmax(np.convolve(hist, [1, 1, 1], mode="same")))
        layer = z[(z >= edges[max(k - 1, 0)]) & (z <= edges[min(k + 2, len(edges) - 1)])]
        return float(np.median(layer)), len(z)


def densify(path: np.ndarray, step: float = 0.2) -> np.ndarray:
    seg = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
    keep = np.r_[True, seg > 1e-3]
    path = path[keep]
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
    if s[-1] < step:
        return path[[0, -1]]
    q = np.linspace(0.0, s[-1], int(math.ceil(s[-1] / step)) + 1)
    return np.column_stack([np.interp(q, s, path[:, k]) for k in range(3)])


def wp_confidence(note: dict, kf_gap: float) -> tuple[str, float]:
    if not note["robot"]:
        return "low", 3.0
    radius = note["gap_m"] + CLOCK_SLACK_M + (1.0 if kf_gap > 4.0 else 0.0)
    return ("high" if radius <= 1.6 else "medium"), round(radius, 1)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", type=Path, default=DEFAULT_RAW)
    ap.add_argument("--exif", type=Path, default=HERE / "photo_exif.txt")
    ap.add_argument("--out", type=Path, default=HERE / "out")
    ap.add_argument("--step-threshold", type=float, default=0.12,
                    help="smoothed ground step (m) within 0.4 m that forces stairs gait")
    ap.add_argument("--grade-threshold", type=float, default=0.15,
                    help="ground grade over 2 m that forces stairs gait")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    photos = load_photos(args.exif)
    assert len(photos) == 30, len(photos)
    traj = Trajectory(np.loadtxt(args.raw / ".sessions/session_0/poses.txt"))
    t_first, t_last = photos[0]["epoch"], photos[-1]["epoch"]
    course_path = traj.between(t_first - 20, t_last + 5)
    ground = Ground(load_pcd_xyz(args.raw / "full_cloud.pcd"), course_path)

    # Official order = reverse photo order.
    official = list(reversed(photos))
    wps = []
    for k, ph in enumerate(official, start=1):
        note = PHOTO_NOTES[ph["photo"]]
        xyz, heading, pitch = traj.at(ph["epoch"])
        gz, n = ground.at(xyz[:2], xyz[2])
        gap = traj.nearest_gap(ph["epoch"])
        conf, radius = wp_confidence(note, gap)
        wps.append(dict(
            id=f"WP{k:02d}", photo=ph["photo"], photo_order=photos.index(ph) + 1,
            photo_time_cst=dt.datetime.fromtimestamp(ph["epoch"], dt.timezone(dt.timedelta(hours=8))).isoformat(),
            robot_pose_xyz=[round(float(v), 3) for v in xyz],
            robot_heading_in_mapping_run=round(heading, 3), robot_pitch=round(pitch, 3),
            position=[round(float(xyz[0]), 3), round(float(xyz[1]), 3),
                      round(gz if math.isfinite(gz) else float(xyz[2]) - 0.43, 3)],
            ground_points=n, nearest_keyframe_gap_s=round(gap, 1),
            terrain=note["terrain"], hard_terrain=note["hard"], marker=note["marker"],
            robot_in_photo=note["robot"], confidence=conf, uncertainty_radius_m=radius,
            gps_latlon=[ph["lat"], ph["lon"]],
        ))

    segments = []
    for a, b in zip(wps[:-1], wps[1:], strict=True):
        ta = next(p["epoch"] for p in photos if p["photo"] == a["photo"])
        tb = next(p["epoch"] for p in photos if p["photo"] == b["photo"])
        raw = traj.between(ta, tb)  # ta > tb: reversed drive direction
        line = densify(raw, 0.2)
        gz = np.array([ground.at(p[:2], p[2])[0] for p in line])
        fill = np.isnan(gz)
        gz[fill] = line[fill, 2] - 0.43
        # Terrain relief along the centreline. The v3 ground layer flickers by ~0.1 m on flat
        # asphalt, so measure on a 1.8 m median-filtered profile: largest change within 0.4 m
        # (steps) and over 2 m (grade).
        zf = median_filter(gz, size=9, mode="nearest")
        max_step = float(np.abs(zf[2:] - zf[:-2]).max()) if len(zf) > 2 else 0.0
        max_grade = float(np.abs(zf[10:] - zf[:-10]).max() / 2.0) if len(zf) > 10 else 0.0
        climb = float(gz[-1] - gz[0])
        length = float(np.sum(np.linalg.norm(np.diff(line[:, :2], axis=0), axis=1)))
        hard_ends = [w for w, side in ((a, "out"), (b, "in"))
                     if w["hard_terrain"] is True or w["hard_terrain"] == side]
        reasons = []
        if hard_ends:
            reasons.append("photo:" + ",".join(w["terrain"] for w in hard_ends))
        if max_step > args.step_threshold:
            reasons.append(f"step {max_step:.2f}>{args.step_threshold}")
        if max_grade > args.grade_threshold:
            reasons.append(f"grade {max_grade:.2f}>{args.grade_threshold}")
        hard = bool(reasons)
        gait = "stairs" if hard else "flat"
        segments.append(dict(
            id=f"{a['id']}-{b['id']}", **{"from": a["id"]}, to=b["id"], gait=gait,
            speed_limit=0.15 if hard else 0.20, allow_detour=not hard,
            corridor_half_width=0.25 if hard else 0.8,
            length_m=round(length, 2), climb_m=round(climb, 2), max_step_m=round(max_step, 3),
            max_grade=round(max_grade, 3), ground_fill_fraction=round(float(fill.mean()), 3),
            gait_reason="; ".join(reasons),
            centerline=[[round(float(x), 3), round(float(y), 3), round(float(z), 3)]
                        for (x, y, _), z in zip(line, gz, strict=True)],
        ))

    # Heading leaving each WP = direction of the first 1 m of its outgoing segment.
    for w, seg in zip(wps, [*segments, None], strict=True):
        line = np.array((seg or segments[-1])["centerline"])
        if seg is None:
            p0, p1 = line[-min(6, len(line))], line[-1]
        else:
            p0, p1 = line[0], line[min(5, len(line) - 1)]
        w["yaw"] = round(math.atan2(p1[1] - p0[1], p1[0] - p0[0]), 3)

    route = dict(
        schema="s10_route_v2", map_id=MAP_ID, frame="map", z_reference="ground",
        status="DRAFT: positions from photo time on the mapping trajectory; field confirmation required",
        order="official WP01 = door Start (last photo); photos are in reverse order",
        waypoints=[dict(id=w["id"], position=w["position"], yaw=w["yaw"], radius_xy=0.20, tol_z=0.20,
                        terrain=w["terrain"], confidence=w["confidence"],
                        uncertainty_radius_m=w["uncertainty_radius_m"], photo=w["photo"]) for w in wps],
        segments=segments,
    )
    (args.out / "route_v2.json").write_text(json.dumps(route, indent=1, ensure_ascii=False))
    (args.out / "wp30_v3_matched.json").write_text(json.dumps(wps, indent=1, ensure_ascii=False))
    with (args.out / "wp30_v3_matched.csv").open("w", newline="") as f:
        cols = ["id", "photo", "photo_order", "photo_time_cst", "position", "yaw", "terrain",
                "hard_terrain", "confidence", "uncertainty_radius_m", "robot_in_photo",
                "nearest_keyframe_gap_s", "marker"]
        wr = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        wr.writeheader()
        for w in wps:
            wr.writerow(w)
    with (args.out / "segments.csv").open("w", newline="") as f:
        cols = ["id", "gait", "speed_limit", "allow_detour", "corridor_half_width", "length_m",
                "climb_m", "max_step_m", "max_grade", "ground_fill_fraction", "gait_reason"]
        wr = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        wr.writeheader()
        for s in segments:
            wr.writerow(s)
    print(f"wrote {args.out}")
    for w in wps:
        print(w["id"], w["photo"][:13].ljust(13), w["position"], w["confidence"], w["terrain"])
    for s in segments:
        print(s["id"], s["gait"], s["length_m"], "climb", s["climb_m"], "step", s["max_step_m"],
              "grade", s["max_grade"], s["gait_reason"])


if __name__ == "__main__":
    main()
