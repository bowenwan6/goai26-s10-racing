"""Waypoint follower node.

Consumes ``/ground_truth/odom``, ``/scan`` and ``/perception/heightmap``, and emits
``/cmd_vel``. The locomotion policy receives that command through the ROS command interface
patched into the contest SDK, so this node is the entire autonomy layer: swapping it out
changes racing behaviour without retraining.

Steering is pure pursuit toward the next gate, but the bearing it asks for is vetted
against the lidar first. That is not a refinement -- sweeping the shipped track shows 12 of
32 legs cross terrain more than 0.35 m above the gate plane, so a follower that trusts the
straight line cannot finish. See ``local_planner``.

A stall watchdog is included because the dominant failure mode on the elevated sections is
not falling over but wedging against a ledge while the policy happily keeps walking in
place. Detecting that and backing off recovers a run that would otherwise never finish.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from s10_auto_nav.local_planner import (
    AvoidanceConfig,
    LocalPlanner,
    Relief,
    ground_clearance,
    terrain_relief,
)
from s10_auto_nav.pure_pursuit import (
    Command,
    PurePursuitController,
    PursuitGains,
    wrap_angle,
)
from s10_auto_nav.step_commit import StepCommit, StepCommitConfig
from s10_auto_nav.terrain import (
    TerrainClassifier,
    TerrainKind,
    TerrainReading,
    TerrainVerdict,
)
from s10_auto_nav.waypoints import Course

CONTROL_RATE_HZ = 50.0

#: Perception older than this is ignored and the follower reverts to plain pure pursuit.
#: The sensors publish at ~44 Hz, so this is generous; it exists so that a dead perception
#: node degrades the run rather than freezing it.
SENSOR_TIMEOUT_S = 0.5

#: Speed may be cut immediately but is restored gradually, in units of full scale per
#: second. Braking for an obstacle should be prompt; accelerating out of one should not
#: undo the slew limiting the controller just applied.
SCALE_RECOVERY_RATE = 1.5

#: Speed ceiling while the terrain classifier cannot say what is ahead. Not zero: the
#: height map only changes when the robot moves, so stopping on UNKNOWN keeps it unknown.
UNKNOWN_TERRAIN_SCALE = 0.5

#: How much closer to its gate the robot must get before the stall watchdog accepts that it
#: is making progress, metres.
#:
#: Speed alone cannot tell a wedged robot from one creeping deliberately. The approach taper
#: commands ``max_forward * distance / brake_distance``, which at 0.2 m from the gate with the
#: default 1.4 m taper is 0.10 m/s -- above the watchdog's 0.1 m/s "is it even trying" test,
#: while the achieved speed of 0.03-0.04 m/s sits below ``stall_speed`` of 0.08. So the last
#: 20 cm of every approach looked exactly like a stall, and after ``stall_timeout`` the
#: watchdog reversed at 0.4 m/s. Measured: gates 17, 19, 21 and 22 of the continuous waypoint
#: 16 to 32 run were each pushed back out at closest approaches of 0.201, 0.200, 0.203 and
#: 0.200 m, all four inside the scoring radius but not for the tick it needed.
#:
#: 5 cm because at the 0.03 m/s worst case observed the robot still covers 7.5 cm inside the
#: 2.5 s timeout, so a real approach always resets the ratchet with room to spare, while a
#: robot held against a wall closes nothing and still trips on schedule.
STALL_PROGRESS_M = 0.05


def _optional_positive(value) -> float | None:
    """Read a ROS float parameter where zero means "unset".

    ROS parameters cannot be None, and the brake distance genuinely has a third state:
    inherit the lookahead, which is what it did before the knob existed. Zero is not a
    meaningful braking distance, so it is the sentinel.
    """
    number = float(value)
    return number if number > 0.0 else None


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def tilt_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Angle between the base's own up axis and world up, radians.

    One number rather than roll and pitch separately: a fall is a fall whichever way the
    robot went over, and the combined angle does not wrap.
    """
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, up_z)))


def pitch_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Elevation of the base's forward axis above horizontal, radians; nose-up positive.

    Tilt alone cannot tell a climb from a descent, and the two want opposite responses.
    """
    nose_z = 2.0 * (x * z - w * y)
    return math.asin(max(-1.0, min(1.0, nose_z)))


class WaypointFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_follower")

        self.declare_parameter("course_file", "")
        self.declare_parameter("control_rate", CONTROL_RATE_HZ)
        self.declare_parameter("advance_radius", 0.18)
        # The contest's radius, from course.yaml's own metadata. A knob rather than a
        # constant only so a stricter scorer can be raced against; it is not a tuning
        # parameter and raising it above 0.2 makes the follower claim gates it did not take.
        self.declare_parameter("score_radius", 0.18)
        self.declare_parameter("max_forward", PursuitGains.max_forward)
        self.declare_parameter("terrain_max_forward", PursuitGains.max_forward)
        # A populated default makes rclpy declare INTEGER_ARRAY. An empty Python list is
        # inferred as BYTE_ARRAY and rejects the integer YAML override before startup.
        self.declare_parameter("fast_flat_waypoints", [2, 10, 14, 22])
        self.declare_parameter("fast_flat_min_gate_distance", 3.0)
        self.declare_parameter("fast_flat_max_heading_deg", 8.0)
        self.declare_parameter("fast_flat_max_cross_track", 0.15)
        self.declare_parameter("fast_flat_max_tilt_deg", 6.0)
        self.declare_parameter("fast_flat_max_pitch_deg", 5.0)
        self.declare_parameter("max_lateral", PursuitGains.max_lateral)
        self.declare_parameter("max_yaw_rate", PursuitGains.max_yaw_rate)
        self.declare_parameter("lookahead", PursuitGains.lookahead)
        self.declare_parameter("yaw_gain", PursuitGains.yaw_gain)
        self.declare_parameter("pivot_threshold_deg", math.degrees(PursuitGains.pivot_threshold))
        self.declare_parameter("corner_retreat_waypoints", [26, 27])
        self.declare_parameter("corner_retreat_distance", 0.7)
        self.declare_parameter("corner_retreat_speed", 0.3)
        self.declare_parameter("corner_align_tolerance_deg", 10.0)
        self.declare_parameter("committed_terrain_waypoints", [28, 30])
        self.declare_parameter("committed_runup_waypoints", [28])
        self.declare_parameter("committed_runup_trigger", 0.55)
        self.declare_parameter("committed_runup_distance", 1.5)
        self.declare_parameter("committed_runup_timeout", 20.0)
        # Body-clear staging points for legs whose straight chord starts beside a drop or
        # pillar. Entries are paired by index with XY coordinates in route_hint_points.
        self.declare_parameter("route_hint_waypoints", [31, 32])
        self.declare_parameter("route_hint_points", [29.35, 17.8, 30.55, 18.5])
        self.declare_parameter("route_hint_radius", 0.25)
        self.declare_parameter("route_hint_speed", 0.5)
        self.declare_parameter("route_hint_max_tilt_deg", 12.0)
        self.declare_parameter("route_hint_stable_hold", 0.5)
        # Approach braking. Zero means "inherit the lookahead", which is what the brake did
        # before the knob existed, so the shipped default changes nothing. The stair value
        # is selected automatically by the terrain classifier, not by the operator: see
        # _brake_distance_for.
        self.declare_parameter("brake_distance", 0.0)
        self.declare_parameter("stair_brake_distance", 0.4)
        self.declare_parameter("stall_speed", 0.08)
        self.declare_parameter("stall_timeout", 2.5)
        self.declare_parameter("progress_timeout", 12.0)
        self.declare_parameter("recovery_duration", 1.0)
        self.declare_parameter("climb_pitch_deg", 8.0)
        self.declare_parameter("climb_speed", 0.5)
        self.declare_parameter("climb_progress_distance", 0.25)
        self.declare_parameter("climb_progress_window", 1.5)
        self.declare_parameter("climb_level_dwell", 1.5)
        self.declare_parameter("climb_timeout", 40.0)
        self.declare_parameter("climb_yaw_rate", 0.15)
        self.declare_parameter("climb_backup", 3.0)
        self.declare_parameter("climb_attempts", 3)

        self.declare_parameter("avoidance_enabled", True)
        self.declare_parameter("max_deviation_deg", math.degrees(AvoidanceConfig.max_deviation))
        self.declare_parameter("corridor_half_width", AvoidanceConfig.corridor_half_width)
        self.declare_parameter("probe_distance", AvoidanceConfig.probe_distance)
        self.declare_parameter("blocked_distance", AvoidanceConfig.blocked_distance)
        self.declare_parameter("clearance_weight", AvoidanceConfig.clearance_weight)
        self.declare_parameter("deviation_weight", AvoidanceConfig.deviation_weight)
        self.declare_parameter("barrier_escape_angle_deg", 60.0)
        self.declare_parameter("barrier_escape_distance", 1.2)
        self.declare_parameter("barrier_bypass_forward", 3.0)
        self.declare_parameter("barrier_bypass_gate_standoff", 0.6)
        self.declare_parameter("barrier_bypass_lateral", 1.2)
        self.declare_parameter("barrier_clear_dwell", 2.0)
        self.declare_parameter("target_clearance_margin", AvoidanceConfig.target_clearance_margin)
        self.declare_parameter("blocked_timeout", 0.4)
        self.declare_parameter("barrier_detour_attempts", 3)
        self.declare_parameter("max_step", 0.35)
        self.declare_parameter("max_drop", 0.5)

        # Where the follower's command goes. The default keeps the standalone race
        # behaviour byte for byte: this node publishes /cmd_vel and is the only thing that
        # does. Under the strategy router it is pointed at /strategy/nav_cmd_vel instead, so
        # the router -- not the follower -- decides what reaches /cmd_vel, and there is never
        # a moment when both are writing to it.
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        course_file = self.get_parameter("course_file").value
        if not course_file:
            raise RuntimeError("Parameter 'course_file' is required")

        self.control_rate = float(self.get_parameter("control_rate").value)
        self.course = Course.from_yaml(
            course_file,
            advance_radius=float(self.get_parameter("advance_radius").value),
            score_radius=float(self.get_parameter("score_radius").value),
        )
        self.controller = PurePursuitController(
            PursuitGains(
                max_forward=float(self.get_parameter("max_forward").value),
                max_lateral=float(self.get_parameter("max_lateral").value),
                max_yaw_rate=float(self.get_parameter("max_yaw_rate").value),
                lookahead=float(self.get_parameter("lookahead").value),
                yaw_gain=float(self.get_parameter("yaw_gain").value),
                pivot_threshold=math.radians(
                    float(self.get_parameter("pivot_threshold_deg").value)
                ),
                brake_distance=_optional_positive(self.get_parameter("brake_distance").value),
            )
        )
        self.flat_brake_distance = self.controller.gains.brake_distance
        self.fast_flat_forward = self.controller.gains.max_forward
        self.terrain_max_forward = float(
            self.get_parameter("terrain_max_forward").value
        )
        self.fast_flat_waypoints = {
            int(value) for value in self.get_parameter("fast_flat_waypoints").value
        }
        self.fast_flat_min_gate_distance = float(
            self.get_parameter("fast_flat_min_gate_distance").value
        )
        self.fast_flat_max_heading = math.radians(
            float(self.get_parameter("fast_flat_max_heading_deg").value)
        )
        self.fast_flat_max_cross_track = float(
            self.get_parameter("fast_flat_max_cross_track").value
        )
        self.fast_flat_max_tilt = math.radians(
            float(self.get_parameter("fast_flat_max_tilt_deg").value)
        )
        self.fast_flat_max_pitch = math.radians(
            float(self.get_parameter("fast_flat_max_pitch_deg").value)
        )
        self.stair_brake_distance = _optional_positive(
            self.get_parameter("stair_brake_distance").value
        )
        self.corner_retreat_waypoints = {
            int(value) for value in self.get_parameter("corner_retreat_waypoints").value
        }
        self.corner_retreat_distance = float(
            self.get_parameter("corner_retreat_distance").value
        )
        self.corner_retreat_speed = float(self.get_parameter("corner_retreat_speed").value)
        self.corner_align_tolerance = math.radians(
            float(self.get_parameter("corner_align_tolerance_deg").value)
        )
        self.committed_terrain_waypoints = {
            int(value) for value in self.get_parameter("committed_terrain_waypoints").value
        }
        self.committed_runup_waypoints = {
            int(value) for value in self.get_parameter("committed_runup_waypoints").value
        }
        self.committed_runup_trigger = float(
            self.get_parameter("committed_runup_trigger").value
        )
        self.committed_runup_distance = float(
            self.get_parameter("committed_runup_distance").value
        )
        self.committed_runup_timeout = float(
            self.get_parameter("committed_runup_timeout").value
        )
        hint_waypoints = [
            int(value) for value in self.get_parameter("route_hint_waypoints").value
        ]
        hint_values = [float(value) for value in self.get_parameter("route_hint_points").value]
        if len(hint_values) != 2 * len(hint_waypoints):
            raise ValueError("route_hint_points must contain one XY pair per route waypoint")
        self.route_hints = {
            waypoint: np.asarray(hint_values[2 * i : 2 * i + 2], dtype=float)
            for i, waypoint in enumerate(hint_waypoints)
        }
        self.route_hint_radius = float(self.get_parameter("route_hint_radius").value)
        self.route_hint_speed = float(self.get_parameter("route_hint_speed").value)
        self.route_hint_max_tilt = math.radians(
            float(self.get_parameter("route_hint_max_tilt_deg").value)
        )
        self.route_hint_stable_hold = float(
            self.get_parameter("route_hint_stable_hold").value
        )

        self.stall_speed = float(self.get_parameter("stall_speed").value)
        self.stall_timeout = float(self.get_parameter("stall_timeout").value)
        self.progress_timeout = float(self.get_parameter("progress_timeout").value)
        self.recovery_duration = float(self.get_parameter("recovery_duration").value)
        self.step_commit = StepCommit(
            StepCommitConfig(
                pitch_threshold=math.radians(
                    float(self.get_parameter("climb_pitch_deg").value)
                ),
                speed=float(self.get_parameter("climb_speed").value),
                progress_distance=float(
                    self.get_parameter("climb_progress_distance").value
                ),
                progress_window=float(
                    self.get_parameter("climb_progress_window").value
                ),
                level_dwell=float(self.get_parameter("climb_level_dwell").value),
                yaw_rate=float(self.get_parameter("climb_yaw_rate").value),
                timeout=float(self.get_parameter("climb_timeout").value),
                backup=float(self.get_parameter("climb_backup").value),
                attempts=int(self.get_parameter("climb_attempts").value),
            )
        )

        self.avoidance_enabled = bool(self.get_parameter("avoidance_enabled").value)
        self.planner = LocalPlanner(
            AvoidanceConfig(
                max_deviation=math.radians(
                    float(self.get_parameter("max_deviation_deg").value)
                ),
                corridor_half_width=float(self.get_parameter("corridor_half_width").value),
                probe_distance=float(self.get_parameter("probe_distance").value),
                blocked_distance=float(self.get_parameter("blocked_distance").value),
                clearance_weight=float(self.get_parameter("clearance_weight").value),
                deviation_weight=float(self.get_parameter("deviation_weight").value),
                barrier_escape_angle=math.radians(
                    float(self.get_parameter("barrier_escape_angle_deg").value)
                ),
                target_clearance_margin=float(
                    self.get_parameter("target_clearance_margin").value
                ),
            )
        )
        self.blocked_timeout = float(self.get_parameter("blocked_timeout").value)
        self.barrier_escape_distance = float(
            self.get_parameter("barrier_escape_distance").value
        )
        self.barrier_bypass_forward = float(
            self.get_parameter("barrier_bypass_forward").value
        )
        self.barrier_bypass_gate_standoff = float(
            self.get_parameter("barrier_bypass_gate_standoff").value
        )
        self.barrier_bypass_lateral = float(
            self.get_parameter("barrier_bypass_lateral").value
        )
        self.barrier_clear_dwell = float(
            self.get_parameter("barrier_clear_dwell").value
        )
        self.barrier_detour_attempts = int(
            self.get_parameter("barrier_detour_attempts").value
        )
        self.max_step = float(self.get_parameter("max_step").value)
        self.max_drop = float(self.get_parameter("max_drop").value)

        self._pose_xy: np.ndarray | None = None
        self._yaw = 0.0
        self._tilt = 0.0
        self._pitch = 0.0
        self._speed = 0.0
        self._stalled_for = 0.0
        #: Closest the robot has been to its current gate since the watchdog last cleared,
        #: metres. None until the first tick with a pose and a target.
        self._stall_reference: float | None = None
        self._no_progress_for = 0.0
        self._recovering_for = 0.0
        self._committing = False

        self._ranges: np.ndarray | None = None
        self._beam_angles: np.ndarray | None = None
        self._scan_age = math.inf
        self._heightmap: np.ndarray | None = None
        self._heightmap_age = math.inf
        self._blocked_for = 0.0
        #: Back-offs the planner has made at the current HIGH_BARRIER, and whether that has
        #: gone on long enough to give up on a detour and climb the thing instead.
        self._barrier_detours = 0
        self._climbing_barrier = False
        #: Side selected from the height map for the current high barrier. It is latched so
        #: gait noise cannot send successive ticks around opposite ends of the same wall.
        self._barrier_side = 0
        #: A wall edge has to stay out of HIGH_BARRIER for this clock to expire before the
        #: committed side is released. Once released, it stays released for this gate: the
        #: same wall remains visible beside the robot after it has cleared the corner and
        #: must not start a second escape away from the target.
        self._barrier_clear_for = 0.0
        self._barrier_escape_done = False
        self._barrier_escape_origin: np.ndarray | None = None
        self._barrier_escape_required = self.barrier_escape_distance
        self._barrier_bypass_target: np.ndarray | None = None
        self._barrier_corner_target: np.ndarray | None = None
        self._barrier_corner_phase = False
        self._barrier_final_phase = False
        self._corner_retreat_target: np.ndarray | None = None
        self._corner_incoming_yaw: float | None = None
        self._corner_outgoing_yaw: float | None = None
        self._committed_runup_phase: str | None = None
        self._committed_runup_elapsed = 0.0
        self._committed_runup_attempts = 0
        self._route_hints_completed: set[int] = set()
        self._route_hint_stable_for = 0.0
        #: Set by every path that backs the robot off, cleared by the tick that reads it.
        #: The reader needs a terrain verdict that the setters run too early to have.
        self._backed_off = False
        self._speed_scale = 1.0
        self._log_countdown = 0.0
        self.terrain_classifier = TerrainClassifier()
        self._last_forward = 0.0
        self._strategy_mode = ""

        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        cmd_topic = str(self.get_parameter("cmd_vel_topic").value) or "/cmd_vel"
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.progress_pub = self.create_publisher(Float32, "/nav/progress", latched)
        self.terrain_pub = self.create_publisher(String, "/nav/terrain", 10)
        self.finished_pub = self.create_publisher(Bool, "/nav/finished", latched)
        self.create_subscription(Odometry, "/ground_truth/odom", self._odom_callback, 50)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)
        self.create_subscription(
            Float32MultiArray, "/perception/heightmap", self._heightmap_callback, 10
        )
        self.create_subscription(String, "/strategy/mode", self._strategy_mode_callback, 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self._control_step)
        self.get_logger().info(
            f"Following {len(self.course)} waypoints from {course_file} "
            f"at {self.control_rate:.0f} Hz"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        self._pose_xy = np.array([p.x, p.y])
        self._yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._tilt = tilt_from_quaternion(q.x, q.y, q.z, q.w)
        self._pitch = pitch_from_quaternion(q.x, q.y, q.z, q.w)
        self._speed = float(math.hypot(v.x, v.y))

    def _scan_callback(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, float)
        # A beam that hit nothing comes back at range_max; treating it as an obstacle
        # there would put a phantom wall around the robot at all times.
        ranges[ranges >= msg.range_max - 1e-3] = math.inf
        self._ranges = ranges
        self._beam_angles = msg.angle_min + msg.angle_increment * np.arange(len(ranges))
        self._scan_age = 0.0

    def _heightmap_callback(self, msg: Float32MultiArray) -> None:
        shape = tuple(int(dim.size) for dim in msg.layout.dim)
        data = np.asarray(msg.data, float)
        self._heightmap = data.reshape(shape) if np.prod(shape) == data.size else data
        self._heightmap_age = 0.0

    def _strategy_mode_callback(self, msg: String) -> None:
        previous = self._strategy_mode
        self._strategy_mode = msg.data
        if msg.data != "navigate" or previous not in {
            "climb",
            "verify_clear",
            "handoff",
        }:
            return
        # The follower keeps computing while the router owns /cmd_vel so its sensor and
        # course state stay current. Its transient manoeuvre state must not survive that
        # preemption, though: otherwise a barrier detour selected while Gate16 was moving
        # the robot is executed only after the handoff, against a different pose.
        self.controller.reset()
        self.step_commit.reset()
        self.terrain_classifier.reset()
        self._reset_barrier_escape()
        self._reset_committed_runup()
        self._barrier_detours = 0
        self._climbing_barrier = False
        self._blocked_for = 0.0
        self._stalled_for = 0.0
        self._no_progress_for = 0.0
        self._stall_reference = None
        self._recovering_for = 0.0
        self._committing = False
        self._backed_off = False
        self._corner_retreat_target = None
        self._corner_incoming_yaw = None
        self._corner_outgoing_yaw = None
        self.get_logger().info("Strategy handoff reset transient follower state")

    def _control_step(self) -> None:
        if self._pose_xy is None:
            return  # No pose yet; stay silent rather than command blind.

        dt = 1.0 / self.control_rate
        self._scan_age += dt
        self._heightmap_age += dt

        if self.course.update(self._pose_xy):
            # The stall ratchet measures the gap to *a* gate, so it means nothing once the
            # gate changes: the new one is metres further off than the old one was, and a
            # reference carried over from the last approach is one the robot cannot beat for
            # as long as it takes to drive the leg.
            self._stall_reference = None
            self._stalled_for = 0.0
            self._no_progress_for = 0.0
            # The next leg may turn sharply at the gate. Carrying the previous leg's slew
            # memory means compute() takes several ticks to remove forward/lateral motion;
            # after WP26 that translated the body 0.6 m north while it tried to rotate east
            # and sent it off the narrow deck. A scored gate is a genuine discontinuity in
            # the target, so start its command ramp from rest.
            self.controller.reset()
            self._reset_barrier_escape()
            self._reset_committed_runup()
            reached = self.course.cursor - 1
            self._begin_corner_retreat(reached)
            self.get_logger().info(
                f"Waypoint {reached} reached ({self.course.cursor}/{len(self.course)}), "
                f"{self.course.remaining_distance(self._pose_xy):.1f} m remaining"
            )
            self._publish_progress()

        if self.course.finished:
            self._publish_stop()
            return

        corner_command = self._corner_transition_command(dt)
        if corner_command is not None:
            # The retreat follows the last metres of the path that were just proved safe,
            # then aligns away from the edge. Obstacle and stall reactions would only
            # interrupt that short, bounded manoeuvre and put the robot back at the gate.
            self._stalled_for = 0.0
            self._no_progress_for = 0.0
            self._stall_reference = None
            self._last_forward = corner_command.forward
            self._publish(corner_command)
            self._log_state(corner_command, 1.0, 1.0, False, dt)
            return

        runup_command = self._committed_runup_command(dt)
        if runup_command is not None:
            self._stalled_for = 0.0
            self._no_progress_for = 0.0
            self._stall_reference = None
            self._last_forward = runup_command.forward
            self._publish(runup_command)
            self._log_state(runup_command, 1.0, 1.0, False, dt)
            return

        route_hint_command = self._route_hint_command(dt)
        if route_hint_command is not None:
            # This is a short, measured body-clear connector, analogous to the corner
            # retreat above. Generic scan recovery at WP31 repeatedly backed into the same
            # pose because the target chord begins beside the upper-deck drop.
            self._stalled_for = 0.0
            self._no_progress_for = 0.0
            self._stall_reference = None
            self._last_forward = route_hint_command.forward
            self._publish(route_hint_command)
            self._log_state(route_hint_command, 1.0, 1.0, False, dt)
            return

        if self._recovering_for > 0.0:
            self._recovering_for -= dt
            self._publish(self._recovery_command())
            return

        was_committing = self._committing
        bypassing_barrier = (
            self._barrier_side != 0 or self._barrier_bypass_target is not None
        )
        committed_terrain = self.course.target.index in self.committed_terrain_waypoints
        suppressing_step_commit = bypassing_barrier and not self._barrier_final_phase
        if suppressing_step_commit or committed_terrain:
            # A selected route around a height-map-confirmed wall is stronger evidence than
            # pitch alone. Contact with its corner can pitch the body 50+ degrees; treating
            # that as a stair replaced the bypass with a 0.7 m/s wall charge and caused the
            # measured escape10 fall at 75 degrees. Suppression ends on the clear final leg:
            # final2 reached WP29's last ledge, pitched 23 degrees 0.8 m from the gate, but
            # could not use the run-up because the stale bypass flag still owned the route.
            # A committed terrain route has the
            # complementary evidence: WP27->28 crossed 3/3 under continuous pure pursuit,
            # while StepCommit's bounded back-offs repeatedly pulled it down from 0.3 m
            # short of WP28. In both cases the selected route owns the manoeuvre.
            self.step_commit.reset()
            climbing = False
        else:
            climbing = self.step_commit.update(self._pitch, self._pose_xy, dt)
        self._committing = climbing
        if was_committing and self.step_commit.conceded:
            # Logged once on the edge, because this is the moment the follower stops calling
            # the thing in front of it terrain and starts calling it an obstruction, and a
            # run that ends badly afterwards is unreadable without knowing when that happened.
            self.get_logger().warn(
                f"Gave up climbing after {self.step_commit.failures} attempts at "
                f"{self._pose_xy[0]:.1f},{self._pose_xy[1]:.1f}; steering around it instead"
            )

        carrot = self.course.lookahead_point(self._pose_xy, self.controller.lookahead_distance())

        if climbing:
            # Both the avoidance and the stall watchdog are wrong while pitched, for the
            # same reason: neither can tell terrain from an obstruction. The /scan ring
            # lives in the body frame, so nose-down 17 degrees aims its nominal +2 degrees
            # about 15 degrees below horizontal, into the floor a couple of metres ahead --
            # the planner then steers around ground returns. And the watchdog's job, when
            # the wheels are astride an edge, is to interrupt the one thing that gets them
            # off it. So while pitched the follower drives at the raw carrot and commits.
            self._blocked_for = 0.0
            self._stalled_for = 0.0
            self._no_progress_for = 0.0
            self._stall_reference = None
            command = self.controller.compute(self._pose_xy, self._yaw, carrot, dt)
            published = self.step_commit.command(command)
            # The scale memory has to learn what was actually sent, or the controller
            # spends the step after the ledge ramping down from a speed it never commanded.
            self._speed_scale = 1.0
            self._publish(published)
            self._log_state(published, 1.0, 1.0, climbing, dt)
            return

        gate_distance = float(np.linalg.norm(self.course.target.xy - self._pose_xy))
        near_scoring_gate = gate_distance <= 0.5
        if self._barrier_final_phase or near_scoring_gate:
            # Inside the final body-clear corridor, reversing is strictly harmful. At
            # escape21 the robot reached 0.213 m, then the low-speed watchdog repeatedly
            # pushed it back out because the ordinary flat-ground taper was below the gait's
            # useful envelope. Keep closing instead; the route state is reset on the score.
            self._stalled_for = 0.0
            self._no_progress_for = 0.0
            self._stall_reference = None
        else:
            self._update_stall_watchdog(dt)
        verdict = self._classify_terrain(dt)
        self.terrain_pub.publish(String(data=f"{verdict.kind.value}:{verdict.confidence:.2f}"))
        self.controller.gains.brake_distance = self._brake_distance_for(verdict.kind)

        # The watchdog runs before the classifier, so the tick that trips it cannot say what
        # the robot was backing away from. The answer arrives here, one classification later
        # and still on the same tick, because the recovery itself does not start until the
        # next one.
        counted = self._backed_off
        if counted:
            self._backed_off = False
            self._note_barrier_detour_failed(verdict)

        climbing_barrier = (
            False if bypassing_barrier else self._climbing_a_barrier(verdict)
        )
        charging = verdict.drive_at_it or climbing_barrier or committed_terrain
        if charging:
            # The lidar return and the rise underneath it are the same object, so steering
            # around it only finds another part of it. This is the waypoint 17 case: the
            # avoidance re-chose a side every tick for a hundred seconds while the height
            # map reported 0.31 m of stairs dead ahead the whole time. WP27->28 is the other
            # measured case: the track's stacked/spiral geometry alternates DROP and
            # HIGH_BARRIER even though three controlled trials climbed the route smoothly.
            # Its waypoint-index commitment is explicit in nav.yaml rather than inferred
            # from that ambiguous local projection.
            target, scale = carrot, 1.0
            self._blocked_for = 0.0
        else:
            barrier_side = self._barrier_escape_side_for(
                verdict, dt, self.course.target.xy
            )
            avoidance_carrot = carrot
            if barrier_side == 0 and self._barrier_bypass_target is not None:
                if np.linalg.norm(self._pose_xy - self._barrier_bypass_target) <= 0.35:
                    if self._barrier_corner_target is not None:
                        self.get_logger().info(
                            f"Barrier bypass reached at {self._pose_xy[0]:.1f},"
                            f"{self._pose_xy[1]:.1f}; entering waypoint {self.course.cursor}"
                        )
                        # Do not stop beside the far corner and pivot ninety degrees. In
                        # escape17 the body made contact there, yaw stayed at 90 degrees
                        # despite a sustained -0.7 command, and WP24 remained 0.61 m away.
                        # From the measured-safe low lane the diagonal to the gate is clear,
                        # so carry one continuous arc through the scoring point.
                        self._barrier_bypass_target = self.course.target.xy.copy()
                        self._barrier_corner_target = None
                        self._barrier_corner_phase = False
                        self._barrier_final_phase = True
                        self._stalled_for = 0.0
                        self._no_progress_for = 0.0
                        self._stall_reference = None
                        avoidance_carrot = self._barrier_bypass_target
                    elif not self._barrier_final_phase:
                        self.get_logger().info(
                            f"Barrier corner reached at {self._pose_xy[0]:.1f},"
                            f"{self._pose_xy[1]:.1f}; entering waypoint {self.course.cursor}"
                        )
                        # Preserve the planned corridor through the scoring point. Handing
                        # this last half metre back to the generic scan planner made it turn
                        # broadside to the pillar beyond WP24 and recede at 0.348 m. The
                        # body-clear route and the target horizon have already established
                        # that the gate itself is reachable; drive the short final segment
                        # directly and let the course reset this state only after scoring.
                        self._barrier_bypass_target = self.course.target.xy.copy()
                        self._barrier_corner_phase = False
                        self._barrier_final_phase = True
                        avoidance_carrot = self._barrier_bypass_target
                else:
                    avoidance_carrot = self._barrier_bypass_target
            if self._barrier_final_phase:
                self._blocked_for = 0.0
                # The final few decimetres need the same short taper as a ledge approach.
                # The flat 1.4 m taper commanded only 0.11 m/s at escape21's 0.213 m near
                # miss; the locomotion policy oscillated in place instead of scoring.
                self.controller.gains.brake_distance = (
                    self.stair_brake_distance or self.flat_brake_distance
                )
                target, scale = avoidance_carrot, 1.0
            else:
                target, scale = self._avoidance_target(avoidance_carrot, dt, barrier_side)

        if near_scoring_gate:
            # Below the ordinary 1.4 m taper the policy can be commanded at 0.10-0.15 m/s,
            # where it oscillates rather than translating. A 0.4 m taper still brakes, but
            # carried WP23->24 through a 13 mm miss and gives every strict gate the same
            # treatment instead of a waypoint-specific exception.
            self.controller.gains.brake_distance = (
                self.stair_brake_distance or self.flat_brake_distance
            )

        if self._blocked_for >= self.blocked_timeout:
            self.get_logger().warn(
                f"No drivable heading for {self._blocked_for:.1f}s at waypoint "
                f"{self.course.cursor}; backing off"
            )
            # Directly rather than through the flag, because a back-off taken for want of a
            # heading is evidence about the barrier that is in front of the robot *now*; read
            # two ticks later, after a metre of reversing, it may not be in front of it any
            # more. ``counted`` keeps a tick that trips both clocks from counting twice.
            if not counted:
                self._note_barrier_detour_failed(verdict)
            self._recovering_for = self.recovery_duration
            self._blocked_for = 0.0
            self.controller.reset()
            self._publish(self._recovery_command())
            return

        # The temporary route comes from a body-clear plan and only admits relief the wheels
        # can traverse (up to max_step). Re-applying the generic relief throttle after the
        # wall edge is clear held escape15 at 0.21-0.29 m/s against a harmless 0.26 m map
        # return for most of 180 seconds; measured crossings need at least 0.6 m/s. An older
        # unthrottled route clipped the wall because it had only 0.8 m lateral clearance;
        # the measured-safe route now uses 1.2 m. Keep the conservative throttle during the
        # initial side escape, then give both planned route legs the locomotion authority
        # they need. Lidar still vets every heading.
        terrain = (
            1.0
            if self._barrier_side == 0 and self._barrier_bypass_target is not None
            else self._terrain_scale_for(verdict, charging)
        )
        self.controller.gains.max_forward = self._forward_limit_for(
            target=target,
            carrot=carrot,
            gate_distance=gate_distance,
            verdict=verdict,
            lidar_scale=scale,
            terrain_scale=terrain,
        )
        command = self.controller.compute(self._pose_xy, self._yaw, target, dt)
        published = self._apply_speed_scale(command, min(scale, terrain), dt)
        self._last_forward = published.forward
        self._publish(published)
        self._log_state(published, scale, terrain, climbing, dt, verdict)

    def _forward_limit_for(
        self,
        *,
        target: np.ndarray,
        carrot: np.ndarray,
        gate_distance: float,
        verdict: TerrainVerdict,
        lidar_scale: float,
        terrain_scale: float,
    ) -> float:
        """Admit the measured high-speed gait only on straight, known-clear flat legs.

        The fixed course has stacked decks that make a truly flat lane alternate between
        FLAT and BLOCKED in the local height projection. For that reason the route allowlist
        is primary, while the live checks prove that no avoidance target, terrain throttle,
        large steering correction or unstable attitude is active. Losing any one condition
        returns the accepted 0.7 m/s terrain ceiling before the next command is computed.
        """
        course_target = self.course.target
        if course_target is None or course_target.index not in self.fast_flat_waypoints:
            return self.terrain_max_forward
        if gate_distance < self.fast_flat_min_gate_distance:
            return self.terrain_max_forward
        if self._strategy_mode not in {"", "navigate"}:
            return self.terrain_max_forward
        if verdict.kind not in {TerrainKind.FLAT, TerrainKind.BLOCKED}:
            return self.terrain_max_forward
        if lidar_scale < 0.999 or terrain_scale < 0.999:
            return self.terrain_max_forward
        if np.linalg.norm(np.asarray(target) - np.asarray(carrot)) > 0.05:
            return self.terrain_max_forward
        if abs(self._tilt) > self.fast_flat_max_tilt:
            return self.terrain_max_forward
        if abs(self._pitch) > self.fast_flat_max_pitch:
            return self.terrain_max_forward

        delta = np.asarray(target, float) - self._pose_xy
        heading_error = wrap_angle(math.atan2(delta[1], delta[0]) - self._yaw)
        cross_track = -math.sin(self._yaw) * delta[0] + math.cos(self._yaw) * delta[1]
        if abs(heading_error) > self.fast_flat_max_heading:
            return self.terrain_max_forward
        if abs(cross_track) > self.fast_flat_max_cross_track:
            return self.terrain_max_forward
        return self.fast_flat_forward

    def _log_state(
        self,
        command: Command,
        scale: float,
        terrain: float,
        climbing: bool,
        dt: float,
        verdict: TerrainVerdict | None = None,
    ) -> None:
        """One line a second on what the follower is doing and why.

        Without this a stalled run is silent: the robot simply stops making progress and
        there is nothing to distinguish a wall in front of it from a controller that has
        braked to zero on its own.
        """
        self._log_countdown -= dt
        if self._log_countdown > 0.0:
            return
        self._log_countdown = 1.0

        target = self.course.target
        gate = "done" if target is None else f"{np.linalg.norm(target.xy - self._pose_xy):.1f}m"
        relief = 0.0 if self._heightmap is None else float(np.ptp(self._heightmap))
        mode = ""
        if climbing:
            mode = " RUNUP" if self.step_commit.backing else " CLIMB"
        elif self._barrier_side:
            mode = " DETOUR_L" if self._barrier_side > 0 else " DETOUR_R"
        elif self._corner_retreat_target is not None:
            mode = " CORNER_RETREAT"
        elif self._corner_outgoing_yaw is not None:
            mode = " CORNER_ALIGN"
        elif self._committed_runup_phase == "back":
            mode = " TERRAIN_RUNUP"
        elif self._committed_runup_phase == "push":
            mode = " TERRAIN_PUSH"
        self.get_logger().info(
            f"wp {self.course.cursor}/{len(self.course)} {gate} "
            f"at ({self._pose_xy[0]:.1f},{self._pose_xy[1]:.1f}) "
            f"cmd fwd={command.forward:+.2f} lat={command.lateral:+.2f} "
            f"yaw={command.yaw_rate:+.2f} "
            f"speed={self._speed:.2f} lidar={scale:.2f} "
            f"terrain={terrain:.2f} relief={relief:.2f} "
            f"pitch={math.degrees(self._pitch):+.0f}deg "
            f"tilt={math.degrees(self._tilt):.0f}deg{mode}"
            + (f" [{verdict}]" if verdict is not None else "")
        )

    def _begin_corner_retreat(self, reached: int) -> None:
        """Arm a safe run-up for a sharp turn at an edge-adjacent scored gate.

        WP26 and WP27 sit on the narrow transfer deck. A nominally in-place yaw command
        still walks the learned locomotion policy roughly 0.6 m in its old travel direction,
        enough to fall off the deck. Once the gate has scored, retreat along the segment we
        have just traversed, align there, and approach the next gate with the new heading.

        The waypoint indices are explicit configuration because this is a property of the
        fixed race scene, not something a planar lidar can infer reliably beneath the upper
        storey. Keeping the list in nav.yaml also prevents this conservative manoeuvre being
        applied to every ordinary corner on the course.
        """
        if reached <= 0 or reached >= len(self.course) - 1:
            return
        gate = self.course.waypoints[reached]
        if gate.index not in self.corner_retreat_waypoints:
            return

        previous = self.course.waypoints[reached - 1].xy
        following = self.course.waypoints[reached + 1].xy
        incoming = gate.xy - previous
        outgoing = following - gate.xy
        incoming_length = float(np.linalg.norm(incoming))
        outgoing_length = float(np.linalg.norm(outgoing))
        if incoming_length < 1e-6 or outgoing_length < 1e-6:
            return

        incoming /= incoming_length
        outgoing /= outgoing_length
        self._corner_retreat_target = gate.xy - self.corner_retreat_distance * incoming
        self._corner_incoming_yaw = math.atan2(incoming[1], incoming[0])
        self._corner_outgoing_yaw = math.atan2(outgoing[1], outgoing[0])
        self.get_logger().info(
            f"Waypoint {gate.index} needs an edge-safe turn; retreating "
            f"{self.corner_retreat_distance:.1f} m before aligning to the next leg"
        )

    def _corner_transition_command(self, dt: float) -> Command | None:
        """Return the bounded retreat/alignment command, or None outside the manoeuvre."""
        if self._corner_retreat_target is not None:
            delta = self._corner_retreat_target - self._pose_xy
            distance = float(np.linalg.norm(delta))
            # This is a staging area, not a scoring point. The reverse gait carries a
            # centimetre-scale cross-track offset; demanding a 12 cm Euclidean hit let the
            # WP27 trial pass 14 cm beside the target and continue backing off the opposite
            # edge. A 20 cm capture still leaves at least the planned half-metre of extra
            # turn margin and terminates monotonically in the body-clear area.
            if distance > 0.2:
                incoming_yaw = self._corner_incoming_yaw or 0.0
                yaw_error = wrap_angle(incoming_yaw - self._yaw)
                cross_track = -math.sin(self._yaw) * delta[0] + math.cos(self._yaw) * delta[1]
                return Command(
                    forward=-self.corner_retreat_speed,
                    lateral=float(
                        np.clip(
                            self.controller.gains.lateral_gain * cross_track,
                            -0.2,
                            0.2,
                        )
                    ),
                    yaw_rate=float(
                        np.clip(
                            self.controller.gains.yaw_gain * yaw_error,
                            -self.controller.gains.max_yaw_rate,
                            self.controller.gains.max_yaw_rate,
                        )
                    ),
                )
            self._corner_retreat_target = None
            self.controller.reset()
            self.get_logger().info("Corner retreat complete; aligning in the safe run-up")

        if self._corner_outgoing_yaw is None:
            return None
        yaw_error = wrap_angle(self._corner_outgoing_yaw - self._yaw)
        if abs(yaw_error) <= self.corner_align_tolerance:
            self._corner_incoming_yaw = None
            self._corner_outgoing_yaw = None
            self.controller.reset()
            self.get_logger().info("Corner alignment complete; resuming waypoint pursuit")
            return None
        return Command(
            yaw_rate=float(
                np.clip(
                    self.controller.gains.yaw_gain * yaw_error,
                    -self.controller.gains.max_yaw_rate,
                    self.controller.gains.max_yaw_rate,
                )
            )
        )

    def _reset_committed_runup(self) -> None:
        self._committed_runup_phase = None
        self._committed_runup_elapsed = 0.0
        self._committed_runup_attempts = 0

    def _route_hint_command(self, dt: float) -> Command | None:
        """Follow one measured body-clear staging point before the ordered gate.

        The hint never advances the course and therefore cannot claim a gate. It only
        shapes the start of a leg; normal perception and pursuit resume after entering the
        staging radius, while the strict 0.2 m cursor remains the sole acceptance rule.
        """
        target = self.course.target
        if target is None or target.index in self._route_hints_completed:
            return None
        hint = self.route_hints.get(target.index)
        if hint is None:
            self._route_hint_stable_for = 0.0
            return None
        # A gate can score while the rear wheels are still descending its obstacle. Taking
        # over immediately then suppresses StepCommit's climb/run-up recovery and can spin
        # indefinitely on the lip. Require a settled chassis before this connector owns the
        # command; until then the normal terrain controller below remains authoritative.
        if self._tilt > self.route_hint_max_tilt:
            self._route_hint_stable_for = 0.0
            return None
        self._route_hint_stable_for += dt
        if self._route_hint_stable_for < self.route_hint_stable_hold:
            return None
        distance = float(np.linalg.norm(hint - self._pose_xy))
        if distance <= self.route_hint_radius:
            self._route_hints_completed.add(target.index)
            self._route_hint_stable_for = 0.0
            self.controller.reset()
            self._reset_barrier_escape()
            self.get_logger().info(
                f"Route staging point reached for waypoint {target.index}; resuming perception"
            )
            return None
        command = self.controller.compute(self._pose_xy, self._yaw, hint, dt)
        return Command(
            forward=float(np.clip(command.forward, -self.route_hint_speed, self.route_hint_speed)),
            lateral=float(np.clip(command.lateral, -self.route_hint_speed, self.route_hint_speed)),
            yaw_rate=command.yaw_rate,
        )

    def _committed_runup_command(self, dt: float) -> Command | None:
        """Build momentum for a known route whose final riser defeats a standing push.

        The WP27->28 ramp reaches 0.31-0.32 m from the strict gate and then high-centres on
        its last 0.231 m riser. StepCommit's three-second reverse moved only about 0.2 m on
        the incline, so each retry began from essentially the same stuck state. Distance,
        not time, defines a real run-up here; the time limit merely bounds a failed reverse.
        """
        target = self.course.target
        if target is None or target.index not in self.committed_runup_waypoints:
            self._reset_committed_runup()
            return None

        delta = target.xy - self._pose_xy
        distance = float(np.linalg.norm(delta))
        if self._committed_runup_phase is None:
            if distance > self.committed_runup_trigger:
                return None
            self._committed_runup_phase = "back"
            self._committed_runup_elapsed = 0.0
            self.controller.reset()
            self.get_logger().info(
                f"Committed terrain stopped {distance:.2f} m from waypoint {target.index}; "
                f"backing to a {self.committed_runup_distance:.1f} m run-up"
            )

        desired_yaw = math.atan2(delta[1], delta[0])
        yaw_error = wrap_angle(desired_yaw - self._yaw)
        yaw_rate = float(
            np.clip(
                self.controller.gains.yaw_gain * yaw_error,
                -self.step_commit.config.yaw_rate,
                self.step_commit.config.yaw_rate,
            )
        )
        cross_track = -math.sin(self._yaw) * delta[0] + math.cos(self._yaw) * delta[1]
        # Straight-only sequence final3 drifted 0.8 m across the ramp during its charge and
        # fell from the east edge. Keep the correction deliberately small: enough to hold
        # the centreline over several seconds, not enough to scrub a wheel sideways on the
        # final lip (the failure StepCommit's zero-lateral rule protects against).
        lateral = float(
            np.clip(self.controller.gains.lateral_gain * cross_track, -0.1, 0.1)
        )
        self._committed_runup_elapsed += dt

        if self._committed_runup_phase == "back":
            backed_up = distance >= self.committed_runup_distance
            timed_out = self._committed_runup_elapsed >= self.committed_runup_timeout
            if not backed_up and not timed_out:
                return Command(
                    forward=-self.step_commit.config.speed,
                    lateral=lateral,
                    yaw_rate=yaw_rate,
                )
            self._committed_runup_phase = "push"
            self._committed_runup_elapsed = 0.0
            self.controller.reset()
            self.get_logger().info(
                f"Terrain run-up ready {distance:.2f} m from waypoint {target.index}; charging"
            )

        # A failed charge gets another distance-defined run-up.  The old final-attempt
        # behaviour held ``forward=+speed`` forever after the retry counter was exhausted.
        # On WP28 that can high-centre the chassis 0.35--0.45 m short of the strict gate:
        # every wheel command remains non-zero, so none of the ordinary stall recovery can
        # take ownership and the run never makes another meaningful attempt.  Keep each
        # charge bounded and rebuild momentum every time.  ``_committed_runup_attempts`` is
        # retained as telemetry; the segment recorder's run deadline remains the outer
        # bound on repeated physical attempts.
        if self._committed_runup_elapsed >= self.step_commit.config.timeout:
            self._committed_runup_attempts += 1
            self._committed_runup_elapsed = 0.0
            self._committed_runup_phase = "back"
            self.get_logger().warn(
                f"Terrain charge {self._committed_runup_attempts} stopped {distance:.2f} m "
                f"from waypoint {target.index}; building another bounded run-up"
            )
            return Command(
                forward=-self.step_commit.config.speed,
                lateral=lateral,
                yaw_rate=yaw_rate,
            )
        return Command(
            forward=self.step_commit.config.speed,
            lateral=lateral,
            yaw_rate=yaw_rate,
        )

    def _climbing_a_barrier(self, verdict: TerrainVerdict) -> bool:
        """Whether to drive at a HIGH_BARRIER because there is demonstrably no way round it.

        ``barrier_rise`` is a threshold on a plane-fit residual, not a measurement of what
        the machine can climb, and it is set where it is to catch a wall early enough to
        steer -- deliberately below anything the robot has been seen to cross. So a
        HIGH_BARRIER is a *reason to look for a detour*, not a verdict that the obstacle is
        impassable, and treating it as the latter would give up on rises the robot could
        have taken.

        Hence the order: steering gets first refusal, and only after
        ``barrier_detour_attempts`` back-offs have failed to get the robot anywhere does the
        follower drive at the thing instead. The climb that follows is not open-ended either
        -- ``StepCommit`` owns it from the moment the body pitches, and concedes after its
        own three bounded attempts.

        "Steering" is doing less work in that sentence than it looks. ``LocalPlanner.plan``
        takes ranges and bearings and nothing else, so a barrier the height map can see and
        the lidar cannot is one the planner has no way to steer around: on the leg to
        waypoint 19 it reported a clear corridor on every one of the 892 ticks the classifier
        spent calling the rise a wall. What the count really measures is attempts that got
        nowhere, whatever the planner thought it was doing. That is the honest reading, and
        it is still the right thing to count -- but it means the first phase is a genuine
        detour only where the obstacle shows up in the scan.

        Counted in back-offs rather than in seconds, because seconds cannot be counted from
        here. Every back-off resets the clock it would have been kept on -- ``_blocked_for``
        every 0.4 s, ``_no_progress_for`` every 12 -- so a timer started here would never
        mature. What survives a reset is the event, so the events are what get counted.

        The latch matters too. Driving at the barrier immediately clears ``_blocked_for``,
        so without one the condition would go false on the very next tick and the follower
        would alternate between steering and charging at control rate, which is the
        flip-flop this whole module exists to remove.
        """
        if verdict.kind is not TerrainKind.HIGH_BARRIER:
            self._barrier_detours = 0
            self._climbing_barrier = False
            return False
        # Released by leaving HIGH_BARRIER above -- either over it, or turned aside far
        # enough that the map no longer reads a wall across the corridor.
        return self._climbing_barrier

    def _reset_barrier_escape(self) -> None:
        """Forget the detour only when the ordered gate changes.

        A cleared wall remains visible over the robot's shoulder, so resetting on terrain
        alone can immediately start a second detour away from the gate. The course cursor is
        the unambiguous boundary between independent obstacles.
        """
        self._barrier_side = 0
        self._barrier_clear_for = 0.0
        self._barrier_escape_done = False
        self._barrier_escape_origin = None
        self._barrier_escape_required = self.barrier_escape_distance
        self._barrier_bypass_target = None
        self._barrier_corner_target = None
        self._barrier_corner_phase = False
        self._barrier_final_phase = False

    def _barrier_escape_side_for(
        self, verdict: TerrainVerdict, dt: float, gate: np.ndarray
    ) -> int:
        """Return a stable escape side through brief terrain-classifier label changes.

        The measured waypoint-24 approach alternates HIGH_BARRIER and BLOCKED about once a
        second as gait motion changes how much of the wall fills the height map. Releasing
        the side on each BLOCKED sample alternated +60 and -40 degree commands and parked
        the robot at the wall's corner. Require continuous contrary evidence before leaving
        the detour, then suppress re-entry until the gate changes so the wall beside/behind
        the robot cannot pull it away from the target again.
        """
        if self._barrier_escape_done:
            return 0

        if verdict.kind is TerrainKind.HIGH_BARRIER:
            self._barrier_clear_for = 0.0
            if self._barrier_side == 0:
                # The height map is fixed in the body frame. Immediately after a scored
                # gate it can therefore still be looking straight back down the previous
                # leg while pure pursuit pivots toward the new one.  In final4, WP29 was
                # taken facing west, WP30 lay 62 degrees to the right, and the map of the
                # old approach was mistaken for a wall on the new route.  That latched a
                # right-hand detour and walked the robot off an otherwise clear diagonal.
                # Wait until the sensor actually faces the candidate leg before using it
                # to choose a side.  Once selected, the existing latch remains authoritative
                # through later yaw changes and classifier jitter.
                delta = gate - self._pose_xy
                gate_bearing = math.atan2(delta[1], delta[0])
                if abs(wrap_angle(gate_bearing - self._yaw)) > self.controller.gains.pivot_threshold:
                    return 0
                self._barrier_side = self.planner.barrier_escape_side(self._heightmap)
                if self._barrier_side:
                    self._barrier_escape_origin = self._pose_xy.copy()
                    gate_distance = float(np.linalg.norm(delta))
                    if gate_distance > 1e-6:
                        lateral = min(
                            self.barrier_bypass_lateral,
                            max(0.5, 0.4 * gate_distance),
                        )
                        self._barrier_escape_required = min(
                            self.barrier_escape_distance, lateral
                        )
                        forward = min(
                            self.barrier_bypass_forward,
                            max(0.0, gate_distance - self.barrier_bypass_gate_standoff),
                        )
                        along = delta / gate_distance
                        left = np.array([-along[1], along[0]])
                        self._barrier_bypass_target = (
                            self._pose_xy
                            + forward * along
                            + self._barrier_side * lateral * left
                        )
                        self._barrier_corner_target = (
                            gate - self.barrier_bypass_gate_standoff * along
                        )
                    side = "left" if self._barrier_side > 0 else "right"
                    self.get_logger().warn(
                        f"High barrier at {self._pose_xy[0]:.1f},{self._pose_xy[1]:.1f}; "
                        f"height map opens to the {side}, committing to that escape"
                    )
            return self._barrier_side

        if self._barrier_side == 0:
            return 0

        distance = (
            0.0
            if self._barrier_escape_origin is None
            else float(np.linalg.norm(self._pose_xy - self._barrier_escape_origin))
        )
        if distance < self._barrier_escape_required:
            self._barrier_clear_for = 0.0
            return self._barrier_side

        self._barrier_clear_for += dt
        if self._barrier_clear_for < self.barrier_clear_dwell:
            return self._barrier_side

        # The stall watchdog normally measures progress toward the ordered gate. During
        # the escape it must measure the temporary route instead: moving sideways around a
        # wall can leave gate distance unchanged for several seconds and escape14 proved
        # that reversing at precisely that phase boundary puts the robot back into the
        # obstacle. Start the new phase with a fresh ratchet.
        self._stalled_for = 0.0
        self._no_progress_for = 0.0
        self._stall_reference = None
        self.get_logger().info(
            f"Barrier edge clear for {self._barrier_clear_for:.1f}s at "
            f"{self._pose_xy[0]:.1f},{self._pose_xy[1]:.1f}; following the bypass route"
        )
        self._barrier_side = 0
        self._barrier_clear_for = 0.0
        self._barrier_escape_done = True
        return 0

    def _note_barrier_detour_failed(self, verdict: TerrainVerdict) -> None:
        """Record a back-off, and switch between steering round the barrier and climbing it.

        Called from every path that backs the robot off, because each of them is a report
        that the follower tried something and got nowhere. Hanging this on the blocked
        timeout alone -- which is where it started -- made it unreachable in the case it was
        written for: in front of the rise to waypoint 19 the planner always had *a* heading,
        so ``_blocked_for`` never accumulated, and what actually fired for 1100 s was the
        no-progress clock. The robot backed off eighty-odd times, never counted one of them,
        and finished the run inside a 0.25 m box having never once tried to climb.

        Both directions, because neither answer is known to be right. Detours run out first
        and the climb takes over; a climb that gets nowhere hands back, and the planner --
        by then looking from wherever the failed attempts left the robot -- gets another go.
        Alternating costs a few seconds per switch and cannot wedge; committing to either
        one for good is exactly how both of the runs before this were lost.
        """
        if verdict.kind is not TerrainKind.HIGH_BARRIER:
            return
        if self._climbing_barrier:
            self._climbing_barrier = False
            self._barrier_detours = 0
            self.get_logger().warn(
                f"Climbing the barrier at {self._pose_xy[0]:.1f},{self._pose_xy[1]:.1f} "
                f"got nowhere either; looking for a way round again"
            )
            return
        self._barrier_detours += 1
        if self._barrier_detours < self.barrier_detour_attempts:
            return
        self._climbing_barrier = True
        self.get_logger().warn(
            f"No way round the barrier at {self._pose_xy[0]:.1f},{self._pose_xy[1]:.1f} "
            f"after {self._barrier_detours} attempts; climbing it instead"
        )

    def _avoidance_target(
        self, carrot: np.ndarray, dt: float, barrier_side: int = 0
    ) -> tuple[np.ndarray, float]:
        """Redirect the carrot onto a drivable bearing, and report the speed it allows.

        Only the bearing is overridden; the distance to the carrot is preserved, so the
        controller still brakes into a gate it is approaching rather than charging a point
        held permanently a full lookahead away.
        """
        if (
            not self.avoidance_enabled
            or self._ranges is None
            or self._scan_age > SENSOR_TIMEOUT_S
        ):
            self._blocked_for = 0.0
            return carrot, 1.0

        delta = carrot - self._pose_xy
        distance = float(np.linalg.norm(delta))
        if distance < 1e-6:
            return carrot, 1.0

        goal_bearing = wrap_angle(math.atan2(delta[1], delta[0]) - self._yaw)
        if barrier_side:
            steering = self.planner.plan_barrier_escape(
                self._ranges, self._beam_angles, goal_bearing, barrier_side
            )
        else:
            steering = self.planner.plan(
                self._ranges, self._beam_angles, goal_bearing, travel_distance=distance
            )

        self._blocked_for = self._blocked_for + dt if steering.blocked else 0.0

        heading = self._yaw + steering.heading
        redirected = self._pose_xy + distance * np.array(
            [math.cos(heading), math.sin(heading)]
        )
        return redirected, steering.speed_scale

    def _classify_terrain(self, dt: float) -> TerrainVerdict:
        """Assemble one reading from what the follower already has, and ask what is ahead.

        Everything here is a measurement the follower was making anyway; the classifier's
        contribution is arbitrating between them rather than letting whichever sensor
        happened to be consulted last decide.
        """
        relief = (
            terrain_relief(self._heightmap)
            if self._heightmap is not None and self._heightmap_age <= SENSOR_TIMEOUT_S
            else Relief()
        )
        obstacle = math.inf
        if self._ranges is not None and self._scan_age <= SENSOR_TIMEOUT_S:
            ahead = np.asarray(self._ranges, float)
            finite = ahead[np.isfinite(ahead) & (ahead > 0.0)]
            if finite.size:
                obstacle = float(finite.min())

        reading = TerrainReading(
            lidar_clearance=obstacle,
            obstacle_distance=obstacle,
            relief_rise=relief.rise,
            relief_drop=relief.drop,
            rise_fraction=relief.rise_fraction,
            slope_deg=relief.slope_deg,
            pitch_deg=math.degrees(self._pitch),
            roll_deg=math.degrees(self._roll_from_tilt()),
            speed=self._speed,
            commanded_forward=self._last_forward,
            sensor_age=max(self._scan_age, self._heightmap_age),
        )
        return self.terrain_classifier.update(reading, dt)

    def _roll_from_tilt(self) -> float:
        """Roll implied by the measured tilt and pitch, radians, unsigned.

        The follower tracks total tilt rather than roll, because a fall is a fall whichever
        way the robot went over. The classifier wants the pair, and recovers the same total
        from them, so this inverts ``tilt = acos(cos(pitch) cos(roll))`` rather than reading
        the quaternion a second time and risking the two disagreeing.
        """
        cos_pitch = math.cos(self._pitch)
        if cos_pitch <= 1e-6:
            return 0.0
        return math.acos(max(-1.0, min(1.0, math.cos(self._tilt) / cos_pitch)))

    def _brake_distance_for(self, kind: TerrainKind) -> float | None:
        """Pick the approach braking profile from the terrain, not from the operator.

        A waypoint sitting on or just past an incline is otherwise approached at a speed
        chosen for flat ground, and the robot runs out of push with its rear axle still on
        a riser. The short profile is applied where that happens and nowhere else, so no
        segment needs its own configuration.
        """
        if kind in (TerrainKind.STAIRS, TerrainKind.RAMP, TerrainKind.HIGH_BARRIER):
            return self.stair_brake_distance or self.flat_brake_distance
        return self.flat_brake_distance

    def _terrain_scale(self) -> float:
        """Speed the terrain underfoot allows; the lidar ring is blind to steps and drops."""
        if self._heightmap is None or self._heightmap_age > SENSOR_TIMEOUT_S:
            return 1.0
        return ground_clearance(self._heightmap, self.max_step, self.max_drop)

    def _terrain_scale_for(self, verdict: TerrainVerdict, charging: bool = False) -> float:
        """The same scale, read in the light of what the classifier decided the ground is.

        The scale and the verdict come from one height map, and without this they were read
        with opposite intent: the classifier said "this rise is the route, drive at it" and
        the scale took the same rise as a reason to drive at a third of the speed. Braking
        for the route is how a run dies on the first edge.

        The numbers are already in nav.yaml, from the y=20.5 drop: 0.50 m/s wedged for a full
        20 s trial, 0.60 crossed in 7.1 s, 0.70 in 4.3 s. A 0.26 m rise scales to 0.35, which
        against ``max_forward`` is 0.24 m/s -- less than half of anything ever measured to
        work. On the leg from waypoint 18 to 19 the robot held exactly that command, at
        exactly one position, for the whole 900 s run budget.

        This is the decision the pitched-climb branch in ``_tick`` already makes; the
        difference is that this one fires before the nose is up, which on a lip taken square
        is the only moment it can still be taken.

        ``charging`` carries the same decision for a barrier the follower has given up
        steering round. It is separate from ``drive_at_it`` because it is not a claim about
        what the ground is -- the classifier still says wall, and still means it. It is a
        claim that this is the last thing left to try, and trying it at a third speed is not
        trying it: the 900 s parked at 0.24 m/s above is exactly what a run does when the
        decision to climb is made and the throttle is not told.
        """
        if charging or verdict.drive_at_it:
            return 1.0
        if verdict.kind is TerrainKind.UNKNOWN:
            # Not knowing is a reason to go slowly, not a reason to stop: the height map only
            # changes when the robot moves, so freezing on UNKNOWN keeps it unknown.
            return min(self._terrain_scale(), UNKNOWN_TERRAIN_SCALE)
        return self._terrain_scale()

    def _apply_speed_scale(self, command: Command, scale: float, dt: float) -> Command:
        """Scale translation only, braking at once but accelerating back gradually."""
        if scale < self._speed_scale:
            self._speed_scale = scale
        else:
            self._speed_scale = min(scale, self._speed_scale + SCALE_RECOVERY_RATE * dt)

        # Yaw is deliberately left unscaled: slowing for an obstacle must not also stop the
        # turn that clears it, or the robot merely creeps into the obstacle more slowly.
        return Command(
            forward=command.forward * self._speed_scale,
            lateral=command.lateral * self._speed_scale,
            yaw_rate=command.yaw_rate,
        )

    def _update_stall_watchdog(self, dt: float) -> None:
        """Trip into recovery when the follower wants progress but is not getting it.

        This tests what the controller asked for, not what was published after scaling.
        Testing the published command looks more truthful and is a trap: the obstacle
        scaling can hold forward speed at zero, which then reads as "not commanded to
        move", and the watchdog sleeps through the one failure it exists to catch. A run
        was lost this way, parked against an arch for five minutes in silence.

        Speed is necessary evidence of a stall but not sufficient, so ground covered toward
        the gate has a veto: a robot getting closer is not stuck no matter how slowly it is
        doing it. Without that veto the watchdog fires hardest exactly where the taper is
        slowest, which is the last few centimetres of an approach -- see ``STALL_PROGRESS_M``
        for the four gates it cost.

        Speed is not necessary either, which is the second clock. Ground covered toward the
        gate is the only thing that means anything on its own, so a robot that is being told
        to drive and is not getting nearer is in trouble whatever the wheels are doing.
        """
        gate = None
        target_xy = self._barrier_bypass_target
        if target_xy is None:
            target = self.course.target
            target_xy = None if target is None else target.xy
        if target_xy is not None and self._pose_xy is not None:
            gate = float(np.linalg.norm(target_xy - self._pose_xy))

        # Seeded here rather than only on the clearing branch below. Left unseeded it stays
        # None through the whole of a slow approach -- the branch that would set it is the
        # one the slow approach never takes -- so ``closing`` reads False forever and the
        # ratchet silently does nothing. Caught by the creeping-approach test.
        if self._stall_reference is None:
            self._stall_reference = gate

        trying = abs(self.controller.last_command.forward) > 0.1
        slow = trying and self._speed < self.stall_speed
        # Against the closest approach so far, not against last tick: a robot inching in at
        # under a millimetre a tick is closing every tick and would clear a tick-to-tick test
        # forever while going nowhere.
        closing = (
            gate is not None
            and self._stall_reference is not None
            and gate <= self._stall_reference - STALL_PROGRESS_M
        )

        if closing:
            self._stalled_for = 0.0
            self._no_progress_for = 0.0
            self._stall_reference = gate
            return

        # Two clocks against the same reference, because "stuck" has two shapes and the fast
        # one only sees the first. Wedged against a wall the wheels stop, and that is caught
        # in stall_timeout. Caught between two avoidance choices the wheels do not stop -- the
        # robot swings left, swings right, and covers ground the whole time. Measured at
        # (18.9, 29.6) on the leg to waypoint 18: speed alternating 0.04, 0.11, 0.07, 0.06,
        # so every tick above stall_speed reset the fast clock before it could ever reach 2.5
        # s, and the run sat there until the time limit with the watchdog reporting no stalls
        # at all. Distance to the gate did not change by 5 cm in any of it.
        if slow:
            self._stalled_for += dt
        else:
            self._stalled_for = 0.0
        if trying:
            self._no_progress_for += dt
        else:
            self._no_progress_for = 0.0

        if self._stalled_for >= self.stall_timeout:
            self._trip_recovery(f"Stalled for {self._stalled_for:.1f}s", gate)
        elif self._no_progress_for >= self.progress_timeout:
            self._trip_recovery(
                f"No progress for {self._no_progress_for:.1f}s while driving", gate
            )

    def _trip_recovery(self, why: str, gate: float | None) -> None:
        """Hand the follower to the recovery command and restart both watchdog clocks."""
        self.get_logger().warn(f"{why} at waypoint {self.course.cursor}; backing off")
        self._backed_off = True
        self._recovering_for = self.recovery_duration
        self._stalled_for = 0.0
        self._no_progress_for = 0.0
        self._stall_reference = gate
        self.controller.reset()

    def _recovery_command(self) -> Command:
        """Reverse while yawing toward the target to unwedge from a ledge or wall."""
        target = self.course.target
        yaw_rate = 0.0
        target_xy = self._barrier_bypass_target
        if target_xy is None and target is not None:
            target_xy = target.xy
        if target_xy is not None and self._pose_xy is not None:
            delta = target_xy - self._pose_xy
            yaw_rate = float(
                np.clip(wrap_angle(math.atan2(delta[1], delta[0]) - self._yaw), -0.8, 0.8)
            )
        return Command(forward=-0.4, lateral=0.0, yaw_rate=yaw_rate)

    def _publish(self, command: Command) -> None:
        msg = Twist()
        msg.linear.x = float(command.forward)
        msg.linear.y = float(command.lateral)
        msg.angular.z = float(command.yaw_rate)
        self.cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())
        if not getattr(self, "_announced_finish", False):
            self._announced_finish = True
            self.get_logger().info("Course complete; holding position")
            self.finished_pub.publish(Bool(data=True))
            self._publish_progress()

    def _publish_progress(self) -> None:
        self.progress_pub.publish(Float32(data=float(self.course.cursor) / len(self.course)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # See viz_node.main: SIGTERM has already closed the context by this point.
        pass
    finally:
        # Only worth attempting while the context is alive. It matters when it is: without
        # this the robot keeps the last command it was given after the follower exits, and
        # a stopped follower leaves a driving robot.
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
