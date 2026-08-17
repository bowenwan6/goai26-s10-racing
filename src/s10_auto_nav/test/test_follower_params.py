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

import numpy as np
import pytest
import yaml

from s10_auto_nav.terrain import TerrainKind, TerrainVerdict

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


# ------------------------------------------------- the height map read against the verdict


def _verdict(kind: TerrainKind) -> TerrainVerdict:
    return TerrainVerdict(kind=kind, confidence=1.0, reason="for the test", candidate=kind)


def _step_of(rise_m: float, node) -> np.ndarray:
    """A height map with a hard lip of ``rise_m`` at the far edge of the patch.

    Heights are relative to the base, so flat ground under a standing S10 reads about
    -0.42 m; the shape is what matters here and the offset only has to be plausible.

    The lip goes in the last two rows rather than across the far half on purpose.
    ``ground_clearance`` measures relief as the residual from a plane fitted to the patch,
    so a step spread over half the patch is largely absorbed by the tilt of that plane: the
    same 0.26 m over rows 7 upward leaves a residual of 0.11 m and scores 0.91, which is not
    a braked robot and would make the assertions below vacuous. Two rows of 0.5 m leave a
    residual of 0.27 m, which is the 0.26 m the waypoint-18 log implies.
    """
    grid = np.full((13, 9), -0.42)
    grid[11:, :] += rise_m
    node._heightmap = grid
    node._heightmap_age = 0.0
    return grid


@needs_ros
def test_a_rise_the_robot_should_drive_at_does_not_also_slow_it_down(node):
    """The defect that pinned a run: one measurement read as both route and hazard.

    Asserted as an inequality against the same map classified the other way, so it fails if
    the suppression is removed *or* if ``ground_clearance`` stops seeing the step at all --
    the second would make the first vacuous.
    """
    _step_of(0.5, node)
    braked = node._terrain_scale_for(_verdict(TerrainKind.BLOCKED))
    assert braked < 0.6, "the height map is not seeing the step; the rest of this proves nothing"
    for kind in (TerrainKind.STAIRS, TerrainKind.RAMP, TerrainKind.HIGH_BARRIER):
        assert node._terrain_scale_for(_verdict(kind)) == pytest.approx(1.0)


@needs_ros
def test_ground_that_is_in_the_way_is_still_slowed_for(node):
    """The suppression is keyed on the verdict, not on the relief, and must stay that way."""
    _step_of(0.5, node)
    assert node._terrain_scale_for(_verdict(TerrainKind.BLOCKED)) < 1.0
    assert node._terrain_scale_for(_verdict(TerrainKind.DROP)) < 1.0


@needs_ros
def test_not_knowing_what_is_ahead_slows_down_without_stopping(node):
    """Zero would be self-sealing: the height map only changes when the robot moves."""
    _step_of(0.0, node)
    scale = node._terrain_scale_for(_verdict(TerrainKind.UNKNOWN))
    assert 0.0 < scale <= 0.5


# ------------------------------------------------------------------- the stall watchdog


def _creep(
    node,
    *,
    from_m: float,
    speed: float,
    seconds: float,
    closes: float | None = None,
    dt: float = 0.05,
) -> bool:
    """Run the watchdog while the robot crawls toward waypoint 0 from ``from_m`` metres out.

    ``speed`` is the odometry speed and ``closes`` is the rate the gap actually shrinks; they
    default to being the same, which is the honest pairing for a robot pointed at its gate.
    Passing them separately is how the swinging case is written -- wheels turning, ground
    covered, no ground covered *toward the gate*. ``speed=0`` holds it in place. Returns
    whether the watchdog ever ordered a reversal.

    The commanded forward speed is written straight onto the controller because the property
    that exposes it is read-only, and the watchdog reads the *commanded* value on purpose --
    see its docstring. 0.1 m/s is what the approach taper asks for at 0.2 m from a gate under
    the shipped 1.4 m lookahead.
    """
    from s10_auto_nav.pure_pursuit import Command

    gap = from_m
    rate = speed if closes is None else closes
    tripped = False
    for _ in range(int(seconds / dt)):
        node._pose_xy = np.array([gap, 0.0])
        node._speed = speed
        node.controller._last = Command(forward=0.101)
        node._update_stall_watchdog(dt)
        tripped = tripped or node._recovering_for > 0.0
        gap = max(0.0, gap - rate * dt)
    return tripped


def _swing(node, *, from_m: float, seconds: float, closes: float = 0.0, dt: float = 0.05) -> bool:
    """The other shape of stuck: wheels turning, gate no nearer.

    Speed alternates 0.04 / 0.11 / 0.06, which is read off the log at (18.9, 29.6). Two of
    the three are under ``stall_speed`` and the third is over it, so the fast clock is reset
    before it can ever run out -- which is why that run reported no stalls while sitting in
    one place for the whole of its time limit.
    """
    from s10_auto_nav.pure_pursuit import Command

    speeds = [0.04, 0.11, 0.06]
    gap = from_m
    tripped = False
    for tick in range(int(seconds / dt)):
        node._pose_xy = np.array([gap, 0.0])
        node._speed = speeds[tick % len(speeds)]
        node.controller._last = Command(forward=0.101)
        node._update_stall_watchdog(dt)
        tripped = tripped or node._recovering_for > 0.0
        gap = max(0.0, gap - closes * dt)
    return tripped


@needs_ros
def test_creeping_into_a_gate_is_not_a_stall(node):
    """The defect that zeroed a scorecard: four gates reached, then reversed back out of.

    0.03 m/s is the speed the policy actually achieved inside the taper at waypoints 17, 19,
    21 and 22, and it is below ``stall_speed``. Six seconds is more than twice the timeout,
    so a watchdog that only looks at speed fails this on the first run through.
    """
    assert not _creep(node, from_m=0.35, speed=0.03, seconds=6.0)


@needs_ros
def test_the_same_speed_with_no_ground_covered_still_trips(node):
    """The veto is progress, not slowness -- otherwise the watchdog stops working at all."""
    assert _creep(node, from_m=0.35, speed=0.0, seconds=6.0)


@needs_ros
def test_a_robot_inching_below_the_ratchet_is_not_saved_by_it(node):
    """Closing is measured against the closest approach so far, not against last tick.

    A robot sliding down a ledge a fraction of a millimetre at a time gets nearer on every
    single tick and would clear a tick-to-tick comparison indefinitely.
    """
    assert _creep(node, from_m=0.35, speed=0.002, seconds=6.0)


@needs_ros
def test_swinging_between_two_avoidance_choices_is_caught(node):
    """The run that sat at (18.9, 29.6) for the whole time limit reporting zero stalls.

    Every speed here is one the robot actually reported there, and the fast clock cannot fire
    on any of them, so this fails outright on a watchdog that only measures speed.
    """
    assert not _swing(node, from_m=6.9, seconds=6.0), "12 s has not elapsed yet"
    assert _swing(node, from_m=6.9, seconds=20.0)


@needs_ros
def test_swinging_while_actually_getting_there_is_left_alone(node):
    """The same speeds, closing on the gate: an obstacle skirted, not an obstacle fought.

    Without this the slow clock would punish every legitimate detour, and the course has
    several. 0.1 m/s closes the 5 cm ratchet five times inside the 12 s window.
    """
    assert not _swing(node, from_m=6.9, seconds=20.0, closes=0.1)
