import math

import pytest

from s10_auto_nav.pitfail_metrics import (
    EntryGate,
    ObstacleFrame,
    interpolate_row,
)


def frame():
    return ObstacleFrame.from_config(
        {
            "id": "gate16",
            "edge_center": [12.0, 3.0, 0.5],
            "normal": [1.0, 0.0],
            "tangent": [0.0, 1.0],
        }
    )


def good_row(**overrides):
    row = {
        "obstacle_distance": 0.6,
        "lateral_error": 0.0,
        "heading_error": 0.0,
        "vx": 0.0,
        "vy": 0.0,
        "yaw_rate": 0.0,
        "tilt_deg": 0.0,
    }
    row.update(overrides)
    return row


def test_obstacle_frame_uses_wall_normal_not_route_heading():
    measured = frame().measure([11.4, 3.1], math.radians(5), [0.2, -0.1])
    assert measured["obstacle_distance"] == pytest.approx(0.6)
    assert measured["lateral_error"] == pytest.approx(0.1)
    assert measured["heading_error"] == pytest.approx(math.radians(5))
    assert measured["forward_speed"] == pytest.approx(0.2)
    assert measured["lateral_speed"] == pytest.approx(-0.1)


def test_gate_requires_continuous_dwell_and_resets():
    gate = EntryGate()
    assert not gate.update(0.0, good_row())[0]
    assert not gate.update(0.39, good_row())[0]
    assert gate.update(0.40, good_row())[0]
    assert not gate.update(0.41, good_row(yaw_rate=0.2))[0]
    assert not gate.update(0.80, good_row())[0]
    assert gate.update(1.20, good_row())[0]


@pytest.mark.parametrize(
    "override",
    [
        {"obstacle_distance": 0.71},
        {"lateral_error": 0.081},
        {"heading_error": math.radians(6.1)},
        {"vx": 0.051},
        {"yaw_rate": 0.101},
        {"tilt_deg": 12.1},
    ],
)
def test_gate_rejects_each_boundary_violation(override):
    allowed, checks = EntryGate().update(1.0, good_row(**override))
    assert not allowed
    assert not all(checks.values())


def test_interpolation_stays_inside_bracket_and_keeps_nearest_label():
    before = {"t": 1.0, "obstacle_distance": 0.61, "vx": 0.2, "mode": "APPROACH"}
    after = {"t": 1.1, "obstacle_distance": 0.59, "vx": 0.4, "mode": "ALIGN"}
    row = interpolate_row(before, after, 0.6)
    assert row["t"] == pytest.approx(1.05)
    assert row["obstacle_distance"] == 0.6
    assert row["vx"] == pytest.approx(0.3)
    assert row["mode"] == "ALIGN"
