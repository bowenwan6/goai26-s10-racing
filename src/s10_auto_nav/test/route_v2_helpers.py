"""Shared builders for the route_v2 tests (pure Python, no ROS)."""

from __future__ import annotations

import copy
import json
from pathlib import Path

import numpy as np

from s10_auto_nav.route_v2 import RouteV2

FIXTURE = Path(__file__).with_name("fixtures") / "route_v2_synthetic.json"


def fixture_dict() -> dict:
    return json.loads(FIXTURE.read_text())


def line(a, b, step=0.2):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(np.ceil(np.linalg.norm(b[:2] - a[:2]) / step)))
    return [list(a + (b - a) * i / n) for i in range(n + 1)]


def polyline(points, step=0.2):
    out = []
    for a, b in zip(points, points[1:], strict=False):
        seg = line(a, b, step)
        out.extend(seg if not out else seg[1:])
    return out


def make_route(
    waypoints,
    *,
    gait="flat",
    allow_detour=True,
    corridor=0.8,
    speed=0.2,
    centerlines=None,
    gaits=None,
) -> RouteV2:
    """Route through ``waypoints`` ([x, y, z] ground) with straight or given centrelines."""
    n = len(waypoints)
    data = {
        "schema": "s10_route_v2",
        "map_id": "synthetic-test-only",
        "frame": "map",
        "z_reference": "ground",
        "status": "DRAFT",
        "waypoints": [
            {
                "id": f"WP{i + 1:02d}",
                "position": list(map(float, p)),
                "yaw": None,
                "radius_xy": 0.2,
                "tol_z": 0.2,
                "terrain": "t",
                "confidence": "high",
            }
            for i, p in enumerate(waypoints)
        ],
        "segments": [],
    }
    for i in range(n - 1):
        g = gaits[i] if gaits else gait
        data["segments"].append(
            {
                "id": f"WP{i + 1:02d}-WP{i + 2:02d}",
                "from": f"WP{i + 1:02d}",
                "to": f"WP{i + 2:02d}",
                "gait": g,
                "speed_limit": speed if g == "flat" else 0.15,
                "allow_detour": allow_detour if g == "flat" else False,
                "corridor_half_width": corridor if g == "flat" else 0.25,
                "centerline": (
                    centerlines[i] if centerlines else line(waypoints[i], waypoints[i + 1])
                ),
            }
        )
    return RouteV2.from_dict(copy.deepcopy(data))
