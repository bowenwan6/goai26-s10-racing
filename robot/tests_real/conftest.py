from __future__ import annotations

import math

import numpy as np
import pytest

from real_transfer.geometry import GRID_X, GRID_Y
from real_transfer.shadow import VERIFICATIONS, ShadowSession


def flat_points(z=-0.42):
    return [[x + d, y, z] for x in GRID_X for y in GRID_Y for d in (-0.01, 0, 0.01)]


@pytest.fixture
def config():
    return {
        "mode": "shadow_only",
        "map_id": "synthetic-test-only",
        "verified": dict.fromkeys(VERIFICATIONS, True),
        "frames": {
            "map": "map",
            "odom_child": "base_link",
            "base": "base_link",
            "cloud": "sensor",
            "scan": "base_link",
        },
        "odom_child_from_base": np.eye(4).tolist(),
        "base_from_cloud": np.eye(4).tolist(),
        "route": [
            {"position": [2, 0, 0.42], "kind": "flat", "radius_xy": 0.15, "tolerance_z": 0.2}
        ],
    }


def snapshot(t=0.0):
    meta = {"stamp": 1000 + t, "received": 10 + t}
    return {
        "wall_time": 1000 + t,
        "monotonic_time": 10 + t,
        "inputs": {
            "pose": dict(
                meta,
                frame="map",
                child_frame="base_link",
                position=[0, 0, 0.42],
                orientation=[0, 0, 0, 1],
            ),
            "cloud": dict(meta, frame="sensor", points=flat_points()),
            "scan": dict(
                meta,
                frame="base_link",
                ranges=[math.inf] * 72,
                angles=np.linspace(-math.pi, math.pi, 72, endpoint=False).tolist(),
                range_min=0.05,
                range_max=10.0,
            ),
            "localization": dict(
                meta,
                code=0,
                **{"global": True},
                map_id="synthetic-test-only",
                session_id="test-session",
            ),
        },
    }


def admitted_session(config):
    session = ShadowSession(config)
    for i in range(61):
        result = session.step(snapshot(i * 0.02))
    assert result["candidate_computed"], result["reasons"]
    return session
