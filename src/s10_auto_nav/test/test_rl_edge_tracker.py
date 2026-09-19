"""Step-edge detection from the 13x9 height grid, and its map-frame tracking against a prior."""

import math

import numpy as np
import pytest

from s10_auto_nav.rl_nav.edge_tracker import (
    NX,
    NY,
    XS,
    YS,
    EdgeTracker,
    detect_edge,
    side_margins,
)


def synthetic_grid(surface, pose=(0.0, 0.0, 0.0), base_z=0.42, spread=0.08):
    """Emulate the height_grid contract: per cell the top of 5x5 samples, invalid when they span
    more than ``spread`` (a riser inside the cell) -- the case a crisp stair produces."""
    x0, y0, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    vals = np.full((NX, NY), -1.0)
    valid = np.zeros((NX, NY), bool)
    off = np.linspace(-0.07, 0.07, 5)
    for i, x in enumerate(XS):
        for j, y in enumerate(YS):
            u, v = np.meshgrid(x + off, y + off)
            wx, wy = x0 + c * u - s * v, y0 + s * u + c * v
            z = surface(wx, wy)
            if np.all(np.isfinite(z)) and z.max() - z.min() <= spread:
                vals[i, j] = z.max() - base_z
                valid[i, j] = True
    return vals, valid


def step(edge_x=0.6, height=0.15):
    return lambda x, y: np.where(x >= edge_x, height, 0.0)


@pytest.mark.parametrize("yaw_deg", [0.0, 10.0, -15.0])
def test_detects_a_square_step_and_its_angle(yaw_deg):
    yaw = math.radians(yaw_deg)
    vals, valid = synthetic_grid(step(0.7), pose=(0.0, 0.0, yaw))
    fix = detect_edge(vals, valid)
    assert fix is not None
    assert fix.distance == pytest.approx(0.7, abs=0.08)
    assert math.degrees(fix.angle) == pytest.approx(-yaw_deg, abs=4.0)
    assert fix.height == pytest.approx(0.15, abs=0.02)
    assert fix.inliers >= 6


def test_drop_is_negative_and_filtered_by_direction():
    vals, valid = synthetic_grid(step(0.6, -0.2))
    fix = detect_edge(vals, valid)
    assert fix is not None and fix.height < -0.15
    assert detect_edge(vals, valid, want="up") is None


def test_flat_ground_and_single_bump_give_nothing():
    vals, valid = synthetic_grid(lambda x, y: np.zeros_like(x))
    assert detect_edge(vals, valid) is None

    def bump(x, y):
        return np.where((np.abs(x - 0.8) < 0.1) & (np.abs(y) < 0.1), 0.2, 0.0)

    vals, valid = synthetic_grid(bump)
    assert detect_edge(vals, valid) is None  # two columns cannot make a four-point line


def test_side_margins_see_a_narrow_flight():
    # A 1.0 m wide structure centred 0.2 m to the left of the robot: drops beyond y=+0.7 and y=-0.3.
    surf = lambda x, y: np.where((y < 0.7) & (y > -0.3), 0.0, -2.0)  # noqa: E731
    vals, valid = synthetic_grid(surf)
    left, right = side_margins(vals, valid)
    assert left is None or left > 0.2  # the left drop is at the grid's edge or beyond
    assert right is not None and right < 0.2


def test_tracker_gates_against_the_prior_and_smooths():
    prior_point, prior_yaw = np.array([0.7, 0.0]), 0.0
    tr = EdgeTracker(prior_point, prior_yaw, 0.15)
    est = tr.update(0.0, (0.0, 0.0), 0.0, *synthetic_grid(step(0.75)))
    assert est.measured and est.point[0] == pytest.approx(0.75, abs=0.08)
    # A fix 1.15 m from the prior line (another step, further back) is rejected; the estimate stays.
    est2 = tr.update(0.2, (-1.2, 0.0), 0.0, *synthetic_grid(step(-0.45), pose=(-1.2, 0.0, 0.0)))
    assert tr.rejected == 1
    assert est2.point[0] == pytest.approx(0.75, abs=0.1)
    d, _lat, head = EdgeTracker.coordinates(est2, (0.2, 0.1), math.radians(5))
    assert d == pytest.approx(0.55, abs=0.1)
    assert math.degrees(head) == pytest.approx(5.0, abs=3.0)


def test_tracker_follows_a_misplaced_prior_within_the_gate():
    # The map put the edge 0.3 m early and 10 deg off; the measurement wins.
    tr = EdgeTracker(np.array([0.4, 0.0]), math.radians(10.0), 0.15)
    for k in range(5):
        est = tr.update(0.1 * k, (0.0, 0.0), 0.0, *synthetic_grid(step(0.7)))
    assert est.measured
    assert math.degrees(est.normal_yaw) == pytest.approx(0.0, abs=4.0)
    d, _, _ = EdgeTracker.coordinates(est, (0.0, 0.0), 0.0)
    assert d == pytest.approx(0.7, abs=0.08)
