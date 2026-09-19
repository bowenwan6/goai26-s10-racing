"""rl_nav_prepare: the robot's map raster and route_v2 in, what the runner loads out -- heights
from the map, one speed limit, the first version's route preparation, the manoeuvres."""

import json

import numpy as np
from route_v2_helpers import line, make_route

from s10_auto_nav.rl_nav.capability import PolicyProfile
from s10_auto_nav.rl_nav.map_check import MapSurface
from s10_auto_nav.rl_nav.prepare import MapTerrain, main, prepare


def step_map(res=0.05):
    """Flat ground at 0 with a 0.15 m step up at x = 5 (everything known)."""
    n = int(30.0 / res)
    origin = (-10.0, -15.0)
    xs = origin[0] + (np.arange(n) + 0.5) * res
    gx, _ = np.meshgrid(xs, xs)
    ground = np.where(gx >= 5.0, 0.15, 0.0)
    return MapSurface(ground, np.ones((n, n), bool), np.zeros((n, n)), origin, res)


def route_dict():
    """route_v2 as the robot gets it: photo-matched heights 0.2 m off the map, and the follower's
    slow default speed limits."""
    pts = [[0, 0, -0.2], [4, 0, -0.2], [9, 0, 0.35]]
    make_route(pts)  # the same route validates
    wps = [
        {
            "id": f"WP{i + 1:02d}",
            "position": p,
            "yaw": None,
            "radius_xy": 0.2,
            "tol_z": 0.2,
            "terrain": "t",
            "confidence": "high",
        }
        for i, p in enumerate(pts)
    ]
    segs = [
        {
            "id": f"WP{i + 1:02d}-WP{i + 2:02d}",
            "from": f"WP{i + 1:02d}",
            "to": f"WP{i + 2:02d}",
            "gait": "flat",
            "speed_limit": 0.2,
            "allow_detour": True,
            "corridor_half_width": 0.8,
            "centerline": line(pts[i], pts[i + 1]),
        }
        for i in range(len(pts) - 1)
    ]
    return {
        "schema": "s10_route_v2",
        "map_id": "synthetic-test-only",
        "frame": "map",
        "z_reference": "ground",
        "status": "DRAFT",
        "waypoints": wps,
        "segments": segs,
    }


def test_prepare_grounds_the_route_on_the_map_and_finds_the_step(tmp_path):
    surface = step_map()
    surface.save(tmp_path / "map.npz")
    terrain = MapTerrain.load(tmp_path / "map.npz")
    prepared, mans, report = prepare(route_dict(), terrain, PolicyProfile.default())
    assert {s["speed_limit"] for s in prepared["segments"]} == {0.6}
    for wp in prepared["waypoints"]:
        x, y, z = wp["position"]
        assert abs(z - float(terrain.ground_at(x, y))) < 1e-6
    assert len(report["regrounded_waypoints"]) == 3
    assert len(mans) == 1 and "edge_up" in mans[0].kinds
    assert mans[0].s0 < 5.0 < mans[0].s1
    assert report["hazards_s"] == []


def test_prepare_command_line_writes_what_the_node_loads(tmp_path):
    step_map().save(tmp_path / "map.npz")
    (tmp_path / "route.json").write_text(json.dumps(route_dict()))
    main(
        [
            "--route",
            str(tmp_path / "route.json"),
            "--terrain",
            str(tmp_path / "map.npz"),
            "--out",
            str(tmp_path / "out"),
        ]
    )
    for name in ("route_rl.json", "maneuvers.json", "map_surface.npz", "prepare_report.json"):
        assert (tmp_path / "out" / name).exists()
    MapSurface.load(tmp_path / "out" / "map_surface.npz")
