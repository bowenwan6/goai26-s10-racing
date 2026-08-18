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

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from s10_auto_nav.terrain import TerrainKind, TerrainVerdict

REPO = Path(__file__).resolve().parents[3]
NAV_YAML = REPO / "src" / "s10_bringup" / "config" / "nav.yaml"
STRATEGY_YAMLS = [
    REPO / "src" / "s10_bringup" / "config" / "strategy.yaml",
    REPO / "src" / "s10_bringup" / "config" / "strategy_gate16.yaml",
]

#: Three waypoints in a straight line is enough course for a node to construct.
COURSE = {"waypoints": [{"index": i, "position": [float(i), 0.0, 0.0]} for i in range(3)]}


# --------------------------------------------------------------- the shipped config


def shipped() -> dict:
    return yaml.safe_load(NAV_YAML.read_text())["waypoint_follower"]["ros__parameters"]


def test_the_brake_distances_ship_in_nav_yaml():
    params = shipped()
    assert params["advance_radius"] == params["score_radius"] == 0.18
    assert params["pivot_threshold_deg"] == 30.0
    assert params["corner_retreat_waypoints"] == [26, 27]
    assert params["corner_retreat_distance"] == 0.7
    assert params["corner_retreat_speed"] == 0.3
    assert params["corner_align_tolerance_deg"] == 10.0
    assert params["committed_terrain_waypoints"] == [28, 30]
    assert params["committed_runup_waypoints"] == [28]
    assert params["committed_runup_trigger"] == 0.55
    assert params["committed_runup_distance"] == 1.5
    assert params["committed_runup_timeout"] == 20.0
    assert params["route_hint_waypoints"] == [31, 32]
    assert params["route_hint_points"] == [29.35, 17.8, 30.55, 18.5]
    assert params["route_hint_radius"] == 0.25
    assert params["route_hint_speed"] == 0.5
    assert params["route_hint_max_tilt_deg"] == 12.0
    assert params["route_hint_stable_hold"] == 0.5
    assert params["brake_distance"] == 0.0, "the flat default must still inherit the lookahead"
    assert params["stair_brake_distance"] == 0.4
    assert params["barrier_escape_angle_deg"] == 60.0
    assert params["barrier_escape_distance"] == 1.2
    assert params["barrier_bypass_forward"] == 3.0
    assert params["barrier_bypass_gate_standoff"] == 0.6
    assert params["barrier_bypass_lateral"] == 1.2
    assert params["barrier_clear_dwell"] == 2.0
    assert params["target_clearance_margin"] == 0.25


def test_no_parameter_in_nav_yaml_is_unknown_to_the_node():
    """A typo in the YAML is silently ignored by ROS unless the node declares the name."""
    source = (REPO / "src" / "s10_auto_nav" / "s10_auto_nav" / "follower_node.py").read_text()
    for name in shipped():
        assert f'declare_parameter("{name}"' in source, f"nav.yaml sets unknown parameter {name}"


def test_the_node_default_cannot_abandon_an_unscored_gate():
    source = (REPO / "src" / "s10_auto_nav" / "s10_auto_nav" / "follower_node.py").read_text()
    assert 'declare_parameter("advance_radius", 0.18)' in source
    assert 'declare_parameter("score_radius", 0.18)' in source


def test_router_and_follower_ship_with_the_same_strict_radius():
    follower = shipped()
    for path in STRATEGY_YAMLS:
        router = yaml.safe_load(path.read_text())["strategy_router"]["ros__parameters"]
        assert router["advance_radius"] == follower["advance_radius"] == 0.18
        assert router["score_radius"] == follower["score_radius"] == 0.18


def test_a_scored_gate_resets_command_slew_before_the_next_leg():
    source = (REPO / "src" / "s10_auto_nav" / "s10_auto_nav" / "follower_node.py").read_text()
    gate_change = source[source.index("if self.course.update") : source.index("if self.course.finished")]
    assert "self.controller.reset()" in gate_change


def test_a_clear_final_bypass_can_use_step_commit_again():
    source = (REPO / "src" / "s10_auto_nav" / "s10_auto_nav" / "follower_node.py").read_text()
    assert "suppressing_step_commit = bypassing_barrier and not self._barrier_final_phase" in source


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
def test_an_edge_corner_retreats_over_the_incoming_path_before_turning(node):
    from s10_auto_nav.pure_pursuit import wrap_angle

    node.corner_retreat_waypoints = {1}
    node.course.waypoints[2].position[:] = [1.0, 1.0, 0.0]
    node._pose_xy = np.array([0.9, 0.0])
    node._yaw = 0.0
    node._begin_corner_retreat(1)

    assert node._corner_retreat_target == pytest.approx([0.3, 0.0])
    retreat = node._corner_transition_command(0.02)
    assert retreat.forward == pytest.approx(-node.corner_retreat_speed)
    assert retreat.yaw_rate == pytest.approx(0.0)

    # The reverse gait need only enter the body-clear staging area; it does not have to
    # intersect an exact point. This 14 cm cross-track miss reproduced sequence6's runaway
    # reverse before the capture radius was corrected.
    node._pose_xy = node._corner_retreat_target + np.array([0.0, 0.14])
    align = node._corner_transition_command(0.02)
    assert align.forward == pytest.approx(0.0)
    assert align.lateral == pytest.approx(0.0)
    assert align.yaw_rate > 0.0
    assert wrap_angle(node._corner_outgoing_yaw - node._yaw) > node.corner_align_tolerance


@needs_ros
def test_committed_terrain_uses_a_distance_defined_runup(node):
    node.committed_terrain_waypoints = {0}
    node.committed_runup_waypoints = {0}
    node._pose_xy = np.array([0.5, 0.0])
    node._yaw = math.pi

    backing = node._committed_runup_command(0.02)
    assert node._committed_runup_phase == "back"
    assert backing.forward == pytest.approx(-node.step_commit.config.speed)

    node._pose_xy = np.array([node.committed_runup_distance, 0.0])
    charging = node._committed_runup_command(0.02)
    assert node._committed_runup_phase == "push"
    assert charging.forward == pytest.approx(node.step_commit.config.speed)


@needs_ros
def test_committed_terrain_never_turns_a_failed_charge_into_an_infinite_push(node):
    node.committed_terrain_waypoints = {0}
    node.committed_runup_waypoints = {0}
    node._pose_xy = np.array([0.4, 0.0])
    node._yaw = math.pi
    node._committed_runup_phase = "push"
    node._committed_runup_attempts = node.step_commit.config.attempts
    node._committed_runup_elapsed = node.step_commit.config.timeout

    command = node._committed_runup_command(0.02)

    assert node._committed_runup_phase == "back"
    assert node._committed_runup_attempts == node.step_commit.config.attempts + 1
    assert command.forward == pytest.approx(-node.step_commit.config.speed)


@needs_ros
def test_route_hint_is_body_clear_staging_not_gate_acceptance(node):
    node.route_hints = {0: np.array([0.5, 0.0])}
    node._route_hints_completed.clear()
    node._pose_xy = np.array([0.0, 0.0])
    node._yaw = 0.0

    command = None
    for _ in range(25):
        command = node._route_hint_command(0.02)
    assert 0.0 < command.forward <= node.route_hint_speed
    assert node.course.cursor == 0


@needs_ros
def test_route_hint_defers_to_terrain_recovery_until_chassis_is_stable(node):
    node.route_hints = {0: np.array([0.5, 0.0])}
    node._pose_xy = np.array([0.0, 0.0])
    node._tilt = math.radians(16.0)
    for _ in range(100):
        assert node._route_hint_command(0.02) is None
    node._tilt = 0.0
    for _ in range(24):
        assert node._route_hint_command(0.02) is None
    assert node._route_hint_command(0.02) is not None

    node._pose_xy = np.array([0.5, 0.0])
    assert node._route_hint_command(0.02) is None
    assert 0 in node._route_hints_completed
    assert node.course.cursor == 0


@needs_ros
def test_committed_terrain_runup_has_bounded_cross_track_correction(node):
    node.committed_terrain_waypoints = {0}
    node.committed_runup_waypoints = {0}
    node._pose_xy = np.array([0.4, 0.3])
    node._yaw = math.pi

    command = node._committed_runup_command(0.02)
    assert 0.0 < command.lateral <= 0.1


@needs_ros
def test_the_default_brake_distance_is_still_the_lookahead(node):
    """The shipped default must not change what the raced configuration did."""
    assert node.controller.gains.brake_distance is None
    assert node.flat_brake_distance is None
    assert node.committed_terrain_waypoints == {28, 30}
    assert node.committed_runup_waypoints == {28}


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
    for kind in (TerrainKind.STAIRS, TerrainKind.RAMP):
        assert node._terrain_scale_for(_verdict(kind)) == pytest.approx(1.0)


@needs_ros
def test_a_rise_past_what_the_wheels_can_step_is_slowed_for(node):
    """The converse, and the half of the pair that the speed scale sees.

    HIGH_BARRIER was in ``DRIVE_AT_IT`` and so took this suppression too, which meant the
    one verdict that says "the wheels cannot take this" was also the one that guaranteed
    full speed into it. On the leg to waypoint 24 the scale had fallen to 0.25 and the
    robot arrived at 0.78 m/s anyway.
    """
    _step_of(0.5, node)
    assert node._terrain_scale_for(_verdict(TerrainKind.HIGH_BARRIER)) < 1.0


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


# ------------------------------------------- a barrier the planner cannot find a way round


def _somewhere(node):
    """Give the node a pose, which the back-off path always has and the log line reads."""
    node._pose_xy = np.array([28.8, 16.3])
    return node


@needs_ros
def test_a_barrier_sends_the_planner_looking_before_anything_else(node):
    """HIGH_BARRIER is a reason to replan, not a verdict that the rise is impassable.

    ``barrier_rise`` is a threshold on a plane-fit residual and sits deliberately below
    anything the robot has been measured crossing, so treating it as a traversability limit
    would abandon rises the machine could have taken. The first response must be to look for
    a way round.
    """
    assert not node._climbing_a_barrier(_verdict(TerrainKind.HIGH_BARRIER))


@needs_ros
def test_a_new_legs_old_heightmap_cannot_latch_a_detour_before_alignment(node):
    """A body-frame map is evidence about the bearing it faces, not the next gate yet."""
    node._pose_xy = np.array([30.935, 14.715])
    node._yaw = math.radians(-167.0)  # WP28 -> WP29 arrival heading in final4.
    gate = np.array([29.535, 16.3275])
    node._heightmap = np.full((13, 9), -0.42)
    node._heightmap[8:, 5:] = -0.10

    barrier = _verdict(TerrainKind.HIGH_BARRIER)
    assert node._barrier_escape_side_for(barrier, 0.02, gate) == 0
    assert node._barrier_bypass_target is None

    node._yaw = math.atan2(*(gate - node._pose_xy)[::-1])
    assert node._barrier_escape_side_for(barrier, 0.02, gate) != 0
    assert node._barrier_bypass_target is not None


@needs_ros
@pytest.mark.params(barrier_clear_dwell=2.0)
def test_brief_barrier_label_jitter_keeps_the_committed_side(node):
    _somewhere(node)
    node._heightmap = np.full((13, 9), -0.42)
    node._heightmap[8:, 5:] = -0.10
    side = node._barrier_escape_side_for(
        _verdict(TerrainKind.HIGH_BARRIER), 0.02, np.array([31.635, 15.465])
    )
    assert side != 0

    for _ in range(90):
        assert node._barrier_escape_side_for(
            _verdict(TerrainKind.BLOCKED), 0.02, np.array([31.635, 15.465])
        ) == side


@needs_ros
@pytest.mark.params(barrier_clear_dwell=2.0)
def test_sustained_clearance_releases_escape_once_per_gate(node):
    _somewhere(node)
    node._heightmap = np.full((13, 9), -0.42)
    node._heightmap[8:, 5:] = -0.10
    barrier = _verdict(TerrainKind.HIGH_BARRIER)
    gate = np.array([31.635, 15.465])
    side = node._barrier_escape_side_for(barrier, 0.02, gate)
    assert side != 0
    assert node._barrier_bypass_target is not None
    assert node._barrier_corner_target is not None

    for _ in range(150):
        assert node._barrier_escape_side_for(
            _verdict(TerrainKind.BLOCKED), 0.02, gate
        ) == side

    node._pose_xy += np.array([1.21, 0.0])
    for _ in range(101):
        released = node._barrier_escape_side_for(
            _verdict(TerrainKind.BLOCKED), 0.02, gate
        )
    assert released == 0
    assert node._barrier_escape_done
    assert node._barrier_escape_side_for(barrier, 0.02, gate) == 0

    node._reset_barrier_escape()
    assert node._barrier_escape_side_for(barrier, 0.02, gate) == side


@needs_ros
def test_a_short_barrier_leg_scales_the_escape_to_the_room_available(node):
    node._pose_xy = np.array([31.635, 15.465])
    node._heightmap = np.full((13, 9), -0.42)
    node._heightmap[8:, 5:] = -0.10
    gate = np.array([33.165, 15.180])

    side = node._barrier_escape_side_for(
        _verdict(TerrainKind.HIGH_BARRIER), 0.02, gate
    )

    assert side != 0
    assert node._barrier_escape_required == pytest.approx(0.4 * np.linalg.norm(gate - node._pose_xy))
    assert node._barrier_escape_required < node.barrier_escape_distance


@needs_ros
@pytest.mark.params(barrier_detour_attempts=3)
def test_a_barrier_with_no_way_round_is_climbed_after_the_detours_fail(node):
    """The fallback. Without it a full-width wall with no detour backs off forever.

    A back-off is what the follower does whenever it tried something and got nowhere, so
    that event is what is counted; three of them and it drives at the thing instead. What
    bounds the climb from there is ``StepCommit``, which concedes after its own three
    attempts.
    """
    barrier = _verdict(TerrainKind.HIGH_BARRIER)
    _somewhere(node)
    for _ in range(2):
        node._note_barrier_detour_failed(barrier)
        assert not node._climbing_a_barrier(barrier), "still worth another look for a detour"
    node._note_barrier_detour_failed(barrier)
    assert node._climbing_a_barrier(barrier)


@needs_ros
@pytest.mark.params(barrier_detour_attempts=1)
def test_the_climb_decision_latches_rather_than_flapping(node):
    """Driving at the barrier zeroes the blocked clock, so an unlatched test would flap.

    The follower would alternate between steering and charging at control rate, which is the
    failure the terrain classifier was written to remove in the first place.
    """
    barrier = _verdict(TerrainKind.HIGH_BARRIER)
    _somewhere(node)
    node._note_barrier_detour_failed(barrier)
    assert all(node._climbing_a_barrier(barrier) for _ in range(50))


@needs_ros
@pytest.mark.params(barrier_detour_attempts=1)
def test_getting_past_the_barrier_releases_the_climb_and_the_tally(node):
    """Two barriers in a row are two problems, not one with six attempts."""
    barrier = _verdict(TerrainKind.HIGH_BARRIER)
    _somewhere(node)
    node._note_barrier_detour_failed(barrier)
    assert node._climbing_a_barrier(barrier)

    assert not node._climbing_a_barrier(_verdict(TerrainKind.FLAT))
    assert not node._climbing_a_barrier(barrier), "the next barrier starts fresh"


@needs_ros
def test_only_a_barrier_counts_towards_climbing_one(node):
    """A wall with room beside it is BLOCKED, and steering around it is the right answer."""
    _somewhere(node)
    for kind in (TerrainKind.BLOCKED, TerrainKind.DROP, TerrainKind.UNKNOWN):
        for _ in range(10):
            node._note_barrier_detour_failed(_verdict(kind))
        assert not node._climbing_a_barrier(_verdict(kind))


@needs_ros
@pytest.mark.params(barrier_detour_attempts=1)
def test_a_climb_that_gets_nowhere_hands_back_to_the_planner(node):
    """Neither answer is known to be right, so neither gets the run to itself.

    The climb is bounded by the same evidence that started it: another back-off, with the
    decision already made, says the charge is not working either. The planner gets it back --
    from wherever the attempt left the robot, which is not where it gave up before.
    """
    barrier = _verdict(TerrainKind.HIGH_BARRIER)
    _somewhere(node)
    node._note_barrier_detour_failed(barrier)
    assert node._climbing_a_barrier(barrier)

    node._note_barrier_detour_failed(barrier)
    assert not node._climbing_a_barrier(barrier), "back to looking for a way round"

    node._note_barrier_detour_failed(barrier)
    assert node._climbing_a_barrier(barrier), "and round again, rather than settling"


@needs_ros
def test_deciding_to_climb_the_barrier_also_lifts_the_throttle(node):
    """The decision is worth nothing if the speed it needs is still being scaled away.

    A 0.26 m rise scales forward to 0.24 m/s, against 0.60 for the slowest crossing ever
    measured. Made and not carried through, the decision to climb reads in a log exactly like
    the 1100 s this run spent held at that command in a 0.25 m box.
    """
    _step_of(0.5, node)
    barrier = _verdict(TerrainKind.HIGH_BARRIER)

    assert node._terrain_scale_for(barrier) < 1.0, "still a wall while a detour is in hand"
    assert node._terrain_scale_for(barrier, charging=True) == pytest.approx(1.0)


@needs_ros
def test_backing_off_for_no_progress_counts_towards_climbing(node):
    """The clock that actually fires in front of a full-width rise.

    Counting only the no-drivable-heading back-off left this unreachable where it was needed:
    the planner always had a heading to offer, so that clock never accumulated, and the
    no-progress clock did all 1100 s of the work uncounted.
    """
    _somewhere(node)
    assert not node._backed_off
    node._trip_recovery("No progress for 12.0s while driving", gate=7.1)
    assert node._backed_off, "the tick that classifies the terrain reads this and counts it"
