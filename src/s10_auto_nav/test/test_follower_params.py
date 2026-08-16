"""What the follower node does with the parameters it is given.

The controller and the classifier are tested on their own elsewhere. What is not covered
there is the wiring between them and the YAML: a knob can be declared, documented, shipped
in ``nav.yaml`` and still never reach the object that uses it, and nothing in a pure unit
test would notice. Every test here therefore constructs the real node and reads the value
back off the real controller.

The node needs ROS, so these skip without it. The YAML checks do not, and are kept separate
for that reason -- they are the ones that catch a parameter renamed in one file only.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from s10_auto_nav.terrain import TerrainKind

REPO = Path(__file__).resolve().parents[3]
NAV_YAML = REPO / "src" / "s10_bringup" / "config" / "nav.yaml"

#: Three waypoints in a straight line is enough course for a node to construct.
COURSE = {"waypoints": [{"index": i, "position": [float(i), 0.0, 0.0]} for i in range(3)]}


# --------------------------------------------------------------- the shipped config


def shipped() -> dict:
    return yaml.safe_load(NAV_YAML.read_text())["waypoint_follower"]["ros__parameters"]


def test_the_brake_distances_ship_in_nav_yaml():
    params = shipped()
    assert params["brake_distance"] == 0.0, "the flat default must still inherit the lookahead"
    assert params["stair_brake_distance"] == 0.4


def test_no_parameter_in_nav_yaml_is_unknown_to_the_node():
    """A typo in the YAML is silently ignored by ROS unless the node declares the name."""
    source = (REPO / "src" / "s10_auto_nav" / "s10_auto_nav" / "follower_node.py").read_text()
    for name in shipped():
        assert f'declare_parameter("{name}"' in source, f"nav.yaml sets unknown parameter {name}"


# --------------------------------------------------------------- the node

try:
    import rclpy
except ImportError:  # pragma: no cover - depends on the environment, not the code
    rclpy = None

needs_ros = pytest.mark.skipif(rclpy is None, reason="ROS 2 is not installed here")


@pytest.fixture
def node(tmp_path, request):
    """The real follower node, built with the parameter overrides marked on the test.

    The overrides go in as ``--ros-args -p``, which is the same path a parameter file takes:
    the node's own constructor runs unmodified and reads them back through
    ``get_parameter``. Reaching in and assigning the attributes afterwards would pass
    whether or not the parameter is wired to anything, which is the failure this file exists
    to catch.
    """
    from s10_auto_nav.follower_node import WaypointFollowerNode

    course = tmp_path / "course.yaml"
    course.write_text(yaml.safe_dump(COURSE))
    overrides = {"course_file": str(course)}
    for mark in request.node.iter_markers("params"):
        overrides.update(mark.kwargs)

    args = ["--ros-args"]
    for name, value in overrides.items():
        args += ["-p", f"{name}:={value}"]

    rclpy.init(args=args)
    try:
        follower = WaypointFollowerNode()
        yield follower
        follower.destroy_node()
    finally:
        rclpy.shutdown()


@needs_ros
def test_the_default_brake_distance_is_still_the_lookahead(node):
    """The shipped default must not change what the raced configuration did."""
    assert node.controller.gains.brake_distance is None
    assert node.flat_brake_distance is None


@needs_ros
@pytest.mark.params(brake_distance=0.4)
def test_the_ros_parameter_reaches_the_controller(node):
    """The check the mandate asks for: set it in ROS, read it off the controller."""
    assert node.controller.gains.brake_distance == pytest.approx(0.4)
    assert node.flat_brake_distance == pytest.approx(0.4)


@needs_ros
@pytest.mark.params(stair_brake_distance=0.4)
def test_terrain_not_the_operator_selects_the_stair_brake(node):
    """One configuration for the whole course; the height map picks between the two."""
    assert node._brake_distance_for(TerrainKind.FLAT) is None
    assert node._brake_distance_for(TerrainKind.STAIRS) == pytest.approx(0.4)
    assert node._brake_distance_for(TerrainKind.RAMP) == pytest.approx(0.4)
    assert node._brake_distance_for(TerrainKind.HIGH_BARRIER) == pytest.approx(0.4)


@needs_ros
@pytest.mark.params(brake_distance=0.6, stair_brake_distance=0.0)
def test_a_disabled_stair_brake_falls_back_to_the_flat_one(node):
    """Zero is the sentinel for "unset" on both knobs, and must not brake at zero metres."""
    assert node._brake_distance_for(TerrainKind.STAIRS) == pytest.approx(0.6)
