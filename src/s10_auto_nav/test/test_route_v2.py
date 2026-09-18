"""route_v2 loading, projection, conversions and terrain cross-check (no ROS)."""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np
import pytest
from route_v2_helpers import FIXTURE, fixture_dict, line, make_route, polyline

from s10_auto_nav.route_v2 import (
    GAIT_CODES,
    RouteTracker,
    RouteV2,
    RouteValidationError,
    TerrainCrossCheck,
)
from s10_auto_nav.waypoints import Course

REAL_ROUTE = Path(
    os.environ.get(
        "S10_ROUTE_V2",
        str(
            Path(__file__).resolve().parents[4]
            / "wt-wp-match/tools/wp_match/out/route_v2.json"
        ),
    )
)


# ------------------------------------------------------------------ loading


def test_fixture_loads_and_ignores_unknown_fields():
    route = RouteV2.load(FIXTURE)
    assert [w.id for w in route.waypoints] == ["WP01", "WP02", "WP03", "WP04"]
    assert [s.gait for s in route.segments] == ["flat", "stairs", "flat"]
    assert route.segments[1].allow_detour is False
    assert route.segments[1].corridor_half_width == 0.25
    assert route.waypoints[3].yaw is None


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda d: d.update(schema="s10_route_v1"), "schema"),
        (lambda d: d.update(frame="odom"), "frame"),
        (lambda d: d.update(z_reference="base"), "z_reference"),
        (lambda d: d["segments"][0].update(gait="run"), "gait"),
        (lambda d: d["segments"][0].update(allow_detour="yes"), "allow_detour"),
        (lambda d: d["segments"][0].update(corridor_half_width=0), "corridor"),
        (lambda d: d["segments"][1].update({"from": "WP01"}), "ordered chain"),
        (lambda d: d["waypoints"][1].update(radius_xy=-1), "radius_xy"),
        (lambda d: d["waypoints"][1].update(id="WP01"), "unique"),
        (lambda d: d["waypoints"][0].update(confidence="certain"), "confidence"),
        (lambda d: d["segments"][0]["centerline"].__delitem__(slice(1, -1)), "spacing"),
        (
            lambda d: [p.__setitem__(1, p[1] + 0.6) for p in d["segments"][0]["centerline"]],
            "does not end near",
        ),
        (lambda d: d["segments"].pop(), "exactly one segment"),
        (lambda d: d["waypoints"][0].update(position=[0, 0, float("nan")]), "finite"),
    ],
)
def test_contract_violations_are_rejected(mutate, message):
    data = fixture_dict()
    mutate(data)
    with pytest.raises(RouteValidationError, match=message):
        RouteV2.from_dict(data)


def test_expected_map_id_is_enforced():
    with pytest.raises(RouteValidationError, match="map_id"):
        RouteV2.from_dict(fixture_dict(), expected_map_id="0914_fr_v3-20260914-142008")


def test_stairs_centreline_spacing_is_checked_horizontally():
    # 0.2 m horizontal + 0.2 m rise per point = 0.28 m in 3-D: still dense enough.
    data = fixture_dict()
    data["segments"][1]["centerline"] = [[6.0, 0.2 * i, 0.2 * i / 3 * 1.0] for i in range(16)]
    data["segments"][1]["centerline"][-1] = [6.0, 3.0, 1.0]
    RouteV2.from_dict(data)


# --------------------------------------------------------------- conversions


def test_gait_kinds_use_incoming_segment_semantics():
    route = RouteV2.load(FIXTURE)
    # kind = gait for the segment INTO the target; the start uses the first segment.
    assert route.gait_kinds() == ["flat", "flat", "stairs", "flat"]
    points = route.native_points()
    assert [p["index"] for p in points] == [0, 1, 2, 3]
    assert points[2]["kind"] == "stairs" and points[2]["corridor_half_width"] == 0.25
    assert points[2]["speed_limit"] == 0.15


def test_course_conversion_keeps_ordered_xyz_gating(tmp_path):
    import yaml

    route = RouteV2.load(FIXTURE)
    course = route.to_course(body_z_offset=0.42)
    assert isinstance(course, Course)
    # Standing on WP03 (upper level) does not score WP01..WP02.
    assert not course.update(np.array([6.0, 3.0, 1.42]))
    assert course.update(np.array([0.1, 0.0, 0.42]))
    # Wrong storey: right XY, z off by more than tol_z.
    assert not course.update(np.array([6.0, 0.0, 1.42]))
    assert course.update(np.array([6.0, 0.15, 0.42]))
    # And the YAML form round-trips through the legacy loader with per-WP radius.
    path = tmp_path / "course.yaml"
    path.write_text(yaml.safe_dump(route.course_yaml_dict(0.42)))
    legacy = Course.from_yaml(path, score_radius=0.5)
    assert legacy.waypoints[0].radius == 0.2 and legacy.waypoints[0].height_tolerance == 0.2
    assert not legacy.update(np.array([0.3, 0.0, 0.42]))  # 0.3 > per-WP 0.2 despite 0.5
    assert legacy.update(np.array([0.1, 0.0, 0.42]))


def test_legacy_course_without_per_waypoint_fields_is_unchanged(tmp_path):
    import yaml

    path = tmp_path / "c.yaml"
    path.write_text(yaml.safe_dump({"waypoints": [{"position": [1, 0, 0]}]}))
    course = Course.from_yaml(path, score_radius=0.5)
    assert course.waypoints[0].radius is None
    assert course.update(np.array([0.6, 0.0]))


# ---------------------------------------------------------------- projection


def test_straight_projection_signs_and_arclength():
    path = make_route([[0, 0, 0], [5, 0, 0]]).path()
    p = path.project([2.3, 0.4])
    assert p.s == pytest.approx(2.3) and p.d == pytest.approx(0.4)
    assert path.project([2.3, -0.4]).d == pytest.approx(-0.4)
    assert p.tangent_yaw == pytest.approx(0.0)
    assert path.point_at(2.3)[:2] == pytest.approx([2.3, 0.0])


def test_corner_projection_is_continuous_and_signed():
    # L-shaped route: east 4 m then north 4 m.
    route = make_route(
        [[0, 0, 0], [4, 4, 0]], centerlines=[polyline([[0, 0, 0], [4, 0, 0], [4, 4, 0]])]
    )
    path = route.path()
    tracker = RouteTracker(path)
    prev = -1.0
    # Drive around the inside of the corner at d = +0.3 (left).
    for s in np.linspace(0.0, 8.0, 81):
        xy = path.offset_points(np.array([s]), np.array([0.3]))[0]
        p = tracker.update(xy, 0.0)
        # Inside a corner an offset point is genuinely nearer the other leg, so s may step
        # back by up to ~2d there -- but never further, and never relocks elsewhere.
        assert p.s >= prev - 2 * 0.3 - 0.05
        assert abs(p.s - s) <= 2 * 0.3 + 0.05
        assert 0.0 <= p.d <= 0.3 + 1e-9
        prev = max(prev, p.s)
    assert tracker.s == pytest.approx(8.0, abs=0.05)
    # Outside of the corner: the vertex is the foot, sign stays negative (right).
    assert path.project([4.4, -0.4], s_hint=4.0, window=(1, 1)).d < 0


def test_switchback_tracker_does_not_snap_to_the_parallel_leg():
    # Hairpin: east 6 m along y=0, turn, back west along y=1.2.
    pts = [[0, 0, 0], [6, 0, 0], [6, 1.2, 0], [0, 1.2, 0]]
    route = make_route([[0, 0, 0], [0, 1.2, 0]], centerlines=[polyline(pts)])
    path = route.path()
    # Global projection of a point between the legs is ambiguous...
    mid = [3.0, 0.65]
    assert path.project(mid).s > 7.0  # (closer to the return leg)
    # ...but a tracker that came along the first leg stays on it.
    tracker = RouteTracker(path)
    for x in np.linspace(0.0, 3.0, 31):
        tracker.update([x, 0.1], 0.0)
    p = tracker.update(mid, 0.0)
    assert p.s == pytest.approx(3.0, abs=0.05)
    assert p.d == pytest.approx(0.65, abs=1e-6)


def test_two_floors_are_separated_by_z():
    # Ground floor east, stairs up, upper floor straight back OVER the ground floor.
    ground = line([0, 0, 0], [6, 0, 0])
    stairs = line([6, 0, 0], [6, 3, 3.0])
    upper = line([6, 3, 3.0], [6, 0.3, 3.0])[1:] + line([6, 0.3, 3.0], [0, 0.3, 3.0])[1:]
    route = make_route(
        [[0, 0, 0], [6, 0, 0], [6, 3, 3.0], [0, 0.3, 3.0]],
        centerlines=[ground, stairs, [[6, 3, 3.0], *upper]],
        gaits=["flat", "stairs", "flat"],
    )
    path = route.path()
    ground_hit = path.project([3.0, 0.1], z=0.0)
    upper_hit = path.project([3.0, 0.1], z=3.0)
    assert ground_hit.s == pytest.approx(3.0, abs=0.01) and ground_hit.segment_index == 0
    assert upper_hit.segment_index == 2 and not upper_hit.z_mismatch
    # A tracker on the ground floor never jumps upstairs even though XY is closer.
    tracker = RouteTracker(path)
    tracker.update([0.0, 0.0], 0.0)
    for x in np.linspace(0.0, 5.0, 26):
        p = tracker.update([x, 0.25], 0.0)
        assert p.segment_index == 0
    # No candidate on this z at all -> flagged.
    assert path.project([3.0, 0.1], z=6.0).z_mismatch


def test_segment_lookup_by_s():
    path = RouteV2.load(FIXTURE).path()
    assert path.waypoint_s == pytest.approx([0.0, 6.0, 9.0, 13.0])
    assert path.gait_at(3.0) == "flat"
    assert path.gait_at(7.0) == "stairs"
    assert path.allow_detour_at(7.0) is False
    assert path.corridor_at(10.0) == 0.8
    assert path.speed_limit_at(7.5) == 0.15
    assert path.segment_at(6.5).gait_code == GAIT_CODES["stairs"]


# ------------------------------------------------------------ cross-check


def test_cross_check_holds_on_flat_segment_pitch_but_never_switches():
    check = TerrainCrossCheck()
    res = None
    for _ in range(10):
        res = check.update("flat", 0.1, pitch=math.radians(15))
    assert res.level == "hold" and "stairs evidence" in res.reason
    # Same evidence on a stairs segment is expected -> ok.
    check.reset()
    for _ in range(10):
        res = check.update("stairs", 0.1, pitch=math.radians(15))
    assert res.level == "ok"
    # A single spike (under the dwell) does not hold.
    check.reset()
    res = check.update("flat", 0.1, pitch=math.radians(15))
    assert res.level == "ok"


def test_cross_check_warns_only_for_heightmap_or_flat_stairs():
    check = TerrainCrossCheck()
    grid = np.full((13, 9), -0.42)
    grid[10:] += 0.3  # a riser across the whole corridor
    for _ in range(10):
        res = check.update("flat", 0.1, pitch=0.0, height=grid, mask=np.ones_like(grid, bool))
    assert res.level == "warn"
    check.reset()
    flat = np.full((13, 9), -0.42)
    for _ in range(10):
        res = check.update("stairs", 0.1, pitch=0.0, height=flat, mask=np.ones_like(flat, bool))
    assert res.level == "warn" and "flat terrain" in res.reason
    # Mostly invalid height -> no evidence, no warning.
    check.reset()
    res = check.update("flat", 0.1, pitch=0.0, height=grid, mask=np.zeros_like(grid, bool))
    assert res.level == "ok"


# ----------------------------------------------------------- the real route


@pytest.mark.skipif(not REAL_ROUTE.exists(), reason=f"real route not present at {REAL_ROUTE}")
def test_real_route_v2_loads_and_projects_continuously():
    route = RouteV2.load(REAL_ROUTE, expected_map_id="0914_fr_v3-20260914-142008")
    assert len(route.waypoints) == 30 and len(route.segments) == 29
    assert route.waypoints[0].id == "WP01" and route.waypoints[-1].id == "WP30"
    path = route.path()
    assert np.all(np.diff(path.waypoint_s) > 0)
    z = path.points[:, 2]
    assert z.min() < 0.0 and z.max() > 5.0  # B stairs climb ~5.5 m
    for seg in route.segments:
        assert (seg.gait == "stairs") == (not seg.allow_detour)
    # 1) Every centreline vertex projects back onto itself, with and without a hint.
    for i, p in enumerate(path.points):
        assert path.project(p[:2], p[2]).s == pytest.approx(path.s[i], abs=0.3)
    # 2) A tracker following the route with 5 cm noise never jumps back or relocks.
    rng = np.random.default_rng(0)
    tracker = RouteTracker(path)
    prev = None
    for i, p in enumerate(path.points):
        proj = tracker.update(p[:2] + rng.normal(0, 0.05, 2), p[2])
        assert abs(proj.s - path.s[i]) < 0.3
        assert not proj.z_mismatch
        if prev is not None:
            assert proj.s >= prev - 0.3
        prev = proj.s
    # 3) Same with a lateral offset sweeping the full per-segment corridor.
    tracker = RouteTracker(path)
    for s in np.arange(0.0, path.length, 0.05):
        w = path.corridor_at(s)
        d = w * math.sin(s / 1.7)
        xy = path.offset_points(np.array([s]), np.array([d]))[0]
        proj = tracker.update(xy, path.point_at(s)[2])
        assert abs(proj.s - s) < w + 0.1
        assert not proj.z_mismatch
    # 4) Native / legacy conversions stay consistent.
    kinds = route.gait_kinds()
    assert kinds[0] == route.segments[0].gait
    assert kinds[1:] == [s.gait for s in route.segments]
    assert len(route.to_course(0.42)) == 30
