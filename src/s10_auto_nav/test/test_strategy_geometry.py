import math

import pytest

from s10_auto_nav.strategy_router_node import _obstacle_coordinates


def test_obstacle_coordinates_use_the_measured_wall_frame():
    distance, lateral, heading = _obstacle_coordinates(
        [12.04593, 32.71090],
        math.radians(-17.7),
        [12.64593, 32.49969],
        [1.0, 0.0],
        [0.0, 1.0],
    )
    assert distance == pytest.approx(0.6)
    assert lateral == pytest.approx(0.21121)
    assert math.degrees(heading) == pytest.approx(-17.7)


def test_obstacle_coordinates_wrap_heading_error():
    _, _, heading = _obstacle_coordinates(
        [0.0, 0.0], math.radians(359), [1.0, 0.0], [1.0, 0.0], [0.0, 1.0]
    )
    assert math.degrees(heading) == pytest.approx(-1.0)
