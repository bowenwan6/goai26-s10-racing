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
from std_msgs.msg import Bool, Float32, Float32MultiArray

from s10_auto_nav.local_planner import AvoidanceConfig, LocalPlanner, ground_clearance
from s10_auto_nav.pure_pursuit import (
    Command,
    PurePursuitController,
    PursuitGains,
    wrap_angle,
)
from s10_auto_nav.step_commit import StepCommit, StepCommitConfig
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
        self.declare_parameter("advance_radius", 0.35)
        self.declare_parameter("max_forward", PursuitGains.max_forward)
        self.declare_parameter("max_lateral", PursuitGains.max_lateral)
        self.declare_parameter("max_yaw_rate", PursuitGains.max_yaw_rate)
        self.declare_parameter("lookahead", PursuitGains.lookahead)
        self.declare_parameter("yaw_gain", PursuitGains.yaw_gain)
        self.declare_parameter("stall_speed", 0.08)
        self.declare_parameter("stall_timeout", 2.5)
        self.declare_parameter("recovery_duration", 1.0)
        self.declare_parameter("climb_pitch_deg", 8.0)
        self.declare_parameter("climb_speed", 0.5)
        self.declare_parameter("climb_progress_speed", 0.35)
        self.declare_parameter("climb_timeout", 40.0)
        self.declare_parameter("climb_yaw_rate", 0.15)
        self.declare_parameter("climb_backup", 3.0)

        self.declare_parameter("avoidance_enabled", True)
        self.declare_parameter("max_deviation_deg", math.degrees(AvoidanceConfig.max_deviation))
        self.declare_parameter("corridor_half_width", AvoidanceConfig.corridor_half_width)
        self.declare_parameter("probe_distance", AvoidanceConfig.probe_distance)
        self.declare_parameter("blocked_distance", AvoidanceConfig.blocked_distance)
        self.declare_parameter("clearance_weight", AvoidanceConfig.clearance_weight)
        self.declare_parameter("deviation_weight", AvoidanceConfig.deviation_weight)
        self.declare_parameter("blocked_timeout", 0.4)
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
            course_file, advance_radius=float(self.get_parameter("advance_radius").value)
        )
        self.controller = PurePursuitController(
            PursuitGains(
                max_forward=float(self.get_parameter("max_forward").value),
                max_lateral=float(self.get_parameter("max_lateral").value),
                max_yaw_rate=float(self.get_parameter("max_yaw_rate").value),
                lookahead=float(self.get_parameter("lookahead").value),
                yaw_gain=float(self.get_parameter("yaw_gain").value),
            )
        )

        self.stall_speed = float(self.get_parameter("stall_speed").value)
        self.stall_timeout = float(self.get_parameter("stall_timeout").value)
        self.recovery_duration = float(self.get_parameter("recovery_duration").value)
        self.step_commit = StepCommit(
            StepCommitConfig(
                pitch_threshold=math.radians(
                    float(self.get_parameter("climb_pitch_deg").value)
                ),
                speed=float(self.get_parameter("climb_speed").value),
                progress_speed=float(self.get_parameter("climb_progress_speed").value),
                yaw_rate=float(self.get_parameter("climb_yaw_rate").value),
                timeout=float(self.get_parameter("climb_timeout").value),
                backup=float(self.get_parameter("climb_backup").value),
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
            )
        )
        self.blocked_timeout = float(self.get_parameter("blocked_timeout").value)
        self.max_step = float(self.get_parameter("max_step").value)
        self.max_drop = float(self.get_parameter("max_drop").value)

        self._pose_xy: np.ndarray | None = None
        self._yaw = 0.0
        self._tilt = 0.0
        self._pitch = 0.0
        self._speed = 0.0
        self._stalled_for = 0.0
        self._recovering_for = 0.0

        self._ranges: np.ndarray | None = None
        self._beam_angles: np.ndarray | None = None
        self._scan_age = math.inf
        self._heightmap: np.ndarray | None = None
        self._heightmap_age = math.inf
        self._blocked_for = 0.0
        self._speed_scale = 1.0
        self._log_countdown = 0.0

        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        cmd_topic = str(self.get_parameter("cmd_vel_topic").value) or "/cmd_vel"
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.progress_pub = self.create_publisher(Float32, "/nav/progress", latched)
        self.finished_pub = self.create_publisher(Bool, "/nav/finished", latched)
        self.create_subscription(Odometry, "/ground_truth/odom", self._odom_callback, 50)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)
        self.create_subscription(
            Float32MultiArray, "/perception/heightmap", self._heightmap_callback, 10
        )

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

    def _control_step(self) -> None:
        if self._pose_xy is None:
            return  # No pose yet; stay silent rather than command blind.

        dt = 1.0 / self.control_rate
        self._scan_age += dt
        self._heightmap_age += dt

        if self.course.update(self._pose_xy):
            reached = self.course.cursor - 1
            self.get_logger().info(
                f"Waypoint {reached} reached ({self.course.cursor}/{len(self.course)}), "
                f"{self.course.remaining_distance(self._pose_xy):.1f} m remaining"
            )
            self._publish_progress()

        if self.course.finished:
            self._publish_stop()
            return

        if self._recovering_for > 0.0:
            self._recovering_for -= dt
            self._publish(self._recovery_command())
            return

        climbing = self.step_commit.update(self._pitch, self._speed, dt)
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
            command = self.controller.compute(self._pose_xy, self._yaw, carrot, dt)
            published = self.step_commit.command(command)
            # The scale memory has to learn what was actually sent, or the controller
            # spends the step after the ledge ramping down from a speed it never commanded.
            self._speed_scale = 1.0
            self._publish(published)
            self._log_state(published, 1.0, 1.0, climbing, dt)
            return

        self._update_stall_watchdog(dt)
        target, scale = self._avoidance_target(carrot, dt)

        if self._blocked_for >= self.blocked_timeout:
            self.get_logger().warn(
                f"No drivable heading for {self._blocked_for:.1f}s at waypoint "
                f"{self.course.cursor}; backing off"
            )
            self._recovering_for = self.recovery_duration
            self._blocked_for = 0.0
            self.controller.reset()
            self._publish(self._recovery_command())
            return

        terrain = self._terrain_scale()
        command = self.controller.compute(self._pose_xy, self._yaw, target, dt)
        published = self._apply_speed_scale(command, min(scale, terrain), dt)
        self._publish(published)
        self._log_state(published, scale, terrain, climbing, dt)

    def _log_state(
        self, command: Command, scale: float, terrain: float, climbing: bool, dt: float
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
        self.get_logger().info(
            f"wp {self.course.cursor}/{len(self.course)} {gate} "
            f"at ({self._pose_xy[0]:.1f},{self._pose_xy[1]:.1f}) "
            f"cmd fwd={command.forward:+.2f} lat={command.lateral:+.2f} "
            f"yaw={command.yaw_rate:+.2f} "
            f"speed={self._speed:.2f} lidar={scale:.2f} "
            f"terrain={terrain:.2f} relief={relief:.2f} "
            f"pitch={math.degrees(self._pitch):+.0f}deg "
            f"tilt={math.degrees(self._tilt):.0f}deg{mode}"
        )

    def _avoidance_target(self, carrot: np.ndarray, dt: float) -> tuple[np.ndarray, float]:
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
        steering = self.planner.plan(self._ranges, self._beam_angles, goal_bearing)

        self._blocked_for = self._blocked_for + dt if steering.blocked else 0.0

        heading = self._yaw + steering.heading
        redirected = self._pose_xy + distance * np.array(
            [math.cos(heading), math.sin(heading)]
        )
        return redirected, steering.speed_scale

    def _terrain_scale(self) -> float:
        """Speed the terrain underfoot allows; the lidar ring is blind to steps and drops."""
        if self._heightmap is None or self._heightmap_age > SENSOR_TIMEOUT_S:
            return 1.0
        return ground_clearance(self._heightmap, self.max_step, self.max_drop)

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
        """
        commanded = abs(self.controller.last_command.forward) > 0.1
        if commanded and self._speed < self.stall_speed:
            self._stalled_for += dt
            if self._stalled_for >= self.stall_timeout:
                self.get_logger().warn(
                    f"Stalled for {self._stalled_for:.1f}s at waypoint {self.course.cursor}; "
                    "backing off"
                )
                self._recovering_for = self.recovery_duration
                self._stalled_for = 0.0
                self.controller.reset()
        else:
            self._stalled_for = 0.0

    def _recovery_command(self) -> Command:
        """Reverse while yawing toward the target to unwedge from a ledge or wall."""
        target = self.course.target
        yaw_rate = 0.0
        if target is not None and self._pose_xy is not None:
            delta = target.xy - self._pose_xy
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
