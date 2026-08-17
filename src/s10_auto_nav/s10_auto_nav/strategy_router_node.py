"""ROS adapter for the strategy router. All the ROS lives here; none of it lives in the router.

The split is deliberate. :mod:`s10_auto_nav.strategy.router` is a pure function from state to
decision, which is why it can be driven through a fall, a stale lidar and a lying policy in
microseconds of test time. This file does the three things that cannot be tested that way:
subscribes, publishes, and -- the part that matters -- owns the execution boundary.

**The boundary rule, concretely.** With this node running, the follower does not publish
``/cmd_vel``; it is remapped to ``/strategy/nav_cmd_vel`` and this node is the sole publisher
of ``/cmd_vel``. That is not a stylistic preference. Two nodes publishing to one topic is not
an error in ROS, it is a merge, and the result is a robot driven by whichever message arrived
last at 50 Hz. This project has already lost an afternoon to exactly that -- an orphaned
container's follower publishing into a later run -- and the symptom was misread as terrain.

For a policy that emits 16 joint targets the same rule applies one level down, and there the
enforcement is not here. ``/JOINTS_CMD`` is written by the contest SDK from inside the
``rl_deploy`` process, as a ``drdds::msg::JointsDataCmd``. This node cannot take that topic
away from it: a second publisher from out here would be a different message type on the same
name, and stopping the official policy writing is not something an external node can do at
all.

So the gate lives where the writing happens -- ``integration/joint_command_owner.hpp``,
installed into the SDK by ``scripts/patch_upstream.py``, through which every joint command
``RLControlState`` produces now passes. This node's part is to *ask*: it publishes joint
targets on ``/strategy/climb_joints`` and the owner it wants on ``/strategy/joint_owner``,
and reads back on ``/joints/owner`` who is actually driving. :class:`JointArbiter` still
gates what leaves this node, which is a different and weaker guarantee, and is kept because
a request the gate refuses should never have been sent in the first place.

Requested and actual ownership are recorded separately in ``/strategy/status`` on purpose. If
they ever disagree for longer than the gate's handover window, that is the fact worth having
in the log, and collapsing them into one field would hide it.

There is no attempt anywhere in this file to convert a 16-dimensional joint action into a
Twist. They are not the same kind of thing, the manoeuvre is not expressible as a body
velocity, and a conversion would silently produce a plausible command that does something
else.

Topics published:

``/cmd_vel``            the one command, whoever it came from
``/strategy/mode``      current state machine mode
``/strategy/source``    who owns the boundary this tick
``/strategy/status``    JSON: mode, source, reason, policy status, retries, active policy
``/strategy/transition`` a line per transition, for the log
``/strategy/climb_joints`` 16 joint targets, when a joint-mode policy is driving
``/strategy/joint_owner`` which controller this node is asking the SDK gate to run

Topics subscribed for ownership:

``/joints/owner``       who the SDK gate says is actually driving the actuators
"""

from __future__ import annotations

import contextlib
import json
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

from s10_auto_nav.strategy.arbiter import JointArbiter
from s10_auto_nav.strategy.mock_policy import MockClimbPolicy, MockScenario
from s10_auto_nav.strategy.policy import PolicyObservation
from s10_auto_nav.strategy.router import RobotState, Router, RouterConfig, Source
from s10_auto_nav.strategy.scripted_policy import ScriptedClimbPolicy
from s10_auto_nav.waypoints import Course


def _yaw_pitch_roll(q) -> tuple[float, float, float]:
    x, y, z, w = q.x, q.y, q.z, q.w
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    return yaw, pitch, roll


def _obstacle_coordinates(position, yaw, edge, normal, tangent) -> tuple[float, float, float]:
    """Pose in a measured obstacle frame: distance, lateral error, heading error."""
    position = np.asarray(position, dtype=float)
    edge = np.asarray(edge, dtype=float)
    normal = np.asarray(normal, dtype=float)
    tangent = np.asarray(tangent, dtype=float)
    distance = float(np.dot(edge - position, normal))
    lateral = float(np.dot(position - edge, tangent))
    normal_yaw = math.atan2(float(normal[1]), float(normal[0]))
    heading = (float(yaw) - normal_yaw + math.pi) % (2.0 * math.pi) - math.pi
    return distance, lateral, heading


class StrategyRouterNode(Node):
    def __init__(self) -> None:
        super().__init__("strategy_router")

        self.declare_parameter("course_file", "")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("nav_cmd_topic", "/strategy/nav_cmd_vel")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        # Off by default. A router with no policy configured is a pass-through, which is what
        # a race run should get until a climb policy has earned its place.
        self.declare_parameter("climb_enabled", False)
        #: ``mock`` or ``scripted``. There is deliberately no ``dagger`` value yet; when a
        #: real checkpoint exists it is added here and nothing else in this file changes.
        self.declare_parameter("climb_policy", "mock")
        self.declare_parameter("climb_segment", [15, 16])
        self.declare_parameter("scripted_trajectory", "")
        self.declare_parameter("mock_scenario", "succeed")
        self.declare_parameter("mock_duration", 2.0)
        # World x of the obstacle's leading edge, if it is known more precisely than "at the
        # waypoint". For Gate 16 it is: the lip was measured at x = 12.6459, and the waypoint
        # is not on it. NaN means fall back to the target waypoint, which is the honest
        # default for a segment nobody has measured.
        self.declare_parameter("climb_obstacle_x", float("nan"))
        self.declare_parameter("climb_edge_center", [float("nan"), float("nan")])
        self.declare_parameter("climb_normal", [float("nan"), float("nan")])
        self.declare_parameter("climb_tangent", [float("nan"), float("nan")])
        self.declare_parameter("advance_radius", 0.35)

        self.declare_parameter("approach_enter", RouterConfig.approach_enter)
        self.declare_parameter("approach_speed_scale", RouterConfig.approach_speed_scale)
        self.declare_parameter("align_enter", RouterConfig.align_enter)
        self.declare_parameter("climb_timeout", RouterConfig.climb_timeout)
        self.declare_parameter("max_retries", RouterConfig.max_retries)
        self.declare_parameter("sensor_timeout", RouterConfig.sensor_timeout)
        # Run the state machine but leave the follower's command alone; see RouterConfig.
        self.declare_parameter("router_shadow", RouterConfig.shadow)
        self.declare_parameter("measurement_only", RouterConfig.measurement_only)
        # The entry gate's distance band and yaw-rate limit, exposed so the pit-failure
        # experiment can sweep them without a code change. Provisional values.
        self.declare_parameter("ready_distance_min", RouterConfig.ready_distance_min)
        self.declare_parameter("ready_distance_max", RouterConfig.ready_distance_max)
        self.declare_parameter("max_entry_yaw_rate", RouterConfig.max_entry_yaw_rate)

        rate = float(self.get_parameter("control_rate").value)
        config = RouterConfig(
            control_rate=rate,
            shadow=bool(self.get_parameter("router_shadow").value),
            measurement_only=bool(self.get_parameter("measurement_only").value),
            approach_enter=float(self.get_parameter("approach_enter").value),
            approach_exit=float(self.get_parameter("approach_enter").value) + 0.6,
            approach_speed_scale=float(self.get_parameter("approach_speed_scale").value),
            align_enter=float(self.get_parameter("align_enter").value),
            align_exit=float(self.get_parameter("align_enter").value) + 0.3,
            ready_distance_min=float(self.get_parameter("ready_distance_min").value),
            ready_distance_max=float(self.get_parameter("ready_distance_max").value),
            max_entry_yaw_rate=float(self.get_parameter("max_entry_yaw_rate").value),
            climb_timeout=float(self.get_parameter("climb_timeout").value),
            max_retries=int(self.get_parameter("max_retries").value),
            sensor_timeout=float(self.get_parameter("sensor_timeout").value),
        )

        policies, segment_policies = self._build_policies()
        self.router = Router(config, policies=policies, segment_policies=segment_policies)
        # Deliberately not /JOINTS_CMD -- see the module docstring. That topic belongs to the
        # SDK; what goes out here is a request and a set of targets, and the gate inside
        # rl_deploy decides whether either is used.
        self._joint_pub = self.create_publisher(Float32MultiArray, "/strategy/climb_joints", 10)
        self._owner_pub = self.create_publisher(String, "/strategy/joint_owner", 10)
        self.arbiter = JointArbiter(
            lambda values: self._joint_pub.publish(Float32MultiArray(data=values.tolist()))
        )
        # What the gate says, as opposed to what was asked for. Starts as unknown rather than
        # as "official": before the first message there is no evidence either way, and
        # assuming the safe answer is how an unrun gate passes for a working one.
        self._actual_owner = "unknown"
        self.create_subscription(String, "/joints/owner", self._on_joint_owner, 10)

        course_file = self.get_parameter("course_file").value
        # The router keeps its own cursor rather than subscribing to the follower's, so that
        # a follower that has stalled or been pre-empted cannot freeze the router's idea of
        # which segment it is on. Same course file, same advance radius, read only.
        self.course = (
            Course.from_yaml(
                course_file, advance_radius=float(self.get_parameter("advance_radius").value)
            )
            if course_file
            else None
        )
        self._obstacle_x = float(self.get_parameter("climb_obstacle_x").value)
        edge = np.asarray(self.get_parameter("climb_edge_center").value, dtype=float)
        normal = np.asarray(self.get_parameter("climb_normal").value, dtype=float)
        tangent = np.asarray(self.get_parameter("climb_tangent").value, dtype=float)
        self._obstacle_frame = None
        if (
            edge.shape == normal.shape == tangent.shape == (2,)
            and np.all(np.isfinite(np.concatenate([edge, normal, tangent])))
            and abs(float(np.linalg.norm(normal)) - 1.0) <= 1e-6
            and abs(float(np.dot(normal, tangent))) <= 1e-6
        ):
            self._obstacle_frame = (edge, normal, tangent)
        self._climb_segment = tuple(int(v) for v in self.get_parameter("climb_segment").value)

        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.mode_pub = self.create_publisher(String, "/strategy/mode", latched)
        self.source_pub = self.create_publisher(String, "/strategy/source", latched)
        self.status_pub = self.create_publisher(String, "/strategy/status", 10)
        self.transition_pub = self.create_publisher(String, "/strategy/transition", 10)

        self.create_subscription(
            Twist, str(self.get_parameter("nav_cmd_topic").value), self._nav_callback, 10
        )
        self.create_subscription(Odometry, "/ground_truth/odom", self._odom_callback, 50)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)
        self.create_subscription(
            Float32MultiArray, "/perception/heightmap", self._heightmap_callback, 10
        )
        self.create_subscription(Float32, "/nav/progress", self._progress_callback, 10)
        self.create_subscription(Bool, "/nav/finished", self._finished_callback, 10)

        self._nav_command = (0.0, 0.0, 0.0)
        self._position = np.zeros(3)
        self._ypr = (0.0, 0.0, 0.0)
        self._speed = 0.0
        self._yaw_rate = 0.0
        self._odom_time = 0.0
        self._lidar_time = 0.0
        self._heightmap_time = 0.0
        self._travelled = 0.0
        self._finished = False
        self._segment = (0, 1)
        self._obstacle_distance = math.inf
        self._lateral_error = 0.0
        self._heading_error = 0.0
        self._history_seen = 0

        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Strategy router at {rate:.0f} Hz, publishing {cmd_topic}; "
            f"climb {'enabled' if policies else 'disabled (pass-through)'}"
        )

    # -- construction ------------------------------------------------------

    def _build_policies(self):
        if not bool(self.get_parameter("climb_enabled").value):
            return {}, {}
        segment = tuple(int(v) for v in self.get_parameter("climb_segment").value)
        kind = str(self.get_parameter("climb_policy").value).lower()
        if kind == "scripted":
            path = str(self.get_parameter("scripted_trajectory").value)
            if not path:
                raise RuntimeError("climb_policy 'scripted' requires scripted_trajectory")
            policy = ScriptedClimbPolicy.from_csv(path)
            self.get_logger().warning(
                "climb policy is a SCRIPTED BASELINE replaying "
                f"{len(policy.trajectory)} recorded frames from {path}. It performs no "
                "inference and must not be reported as a learned result."
            )
        elif kind == "mock":
            policy = MockClimbPolicy(
                MockScenario(str(self.get_parameter("mock_scenario").value)),
                duration=float(self.get_parameter("mock_duration").value),
            )
            self.get_logger().warning(
                "climb policy is a MOCK. It does not climb anything; it exercises the "
                "router's handling of a policy that succeeds, fails, hangs or lies."
            )
        else:
            raise RuntimeError(f"unknown climb_policy '{kind}' (expected mock or scripted)")
        return {"climb_policy": policy}, {segment: "climb_policy"}

    # -- subscriptions -----------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _nav_callback(self, msg: Twist) -> None:
        self._nav_command = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self._position = np.array([p.x, p.y, p.z])
        self._ypr = _yaw_pitch_roll(msg.pose.pose.orientation)
        self._speed = float(math.hypot(v.x, v.y))
        self._yaw_rate = float(msg.twist.twist.angular.z)
        self._odom_time = self._now()

    def _scan_callback(self, msg: LaserScan) -> None:
        self._lidar_time = self._now()

    def _heightmap_callback(self, msg: Float32MultiArray) -> None:
        self._heightmap_time = self._now()

    def _progress_callback(self, msg: Float32) -> None:
        self._travelled = float(msg.data)

    def _finished_callback(self, msg: Bool) -> None:
        self._finished = bool(msg.data)

    # -- the tick ----------------------------------------------------------

    def _update_segment(self) -> None:
        """Track which leg of the course the robot is on, and how far the obstacle is.

        Distance is signed and measured *along the segment*, not as a radius, because the
        router's whole approach/align sequence depends on knowing that the edge has gone
        behind. A radius cannot express that: it is positive on both sides.
        """
        if self.course is None:
            return
        self.course.update(self._position[:2])
        cursor = self.course.cursor
        if self.course.finished or cursor == 0:
            self._segment = (max(cursor - 1, 0), cursor)
            self._obstacle_distance = math.inf
            self._lateral_error = 0.0
            self._heading_error = 0.0
            return
        previous = self.course.waypoints[cursor - 1]
        target = self.course.waypoints[cursor]
        self._segment = (previous.index, target.index)

        if self._segment != self._climb_segment:
            # Only the configured climb segment has an obstacle line. Everything else is
            # ordinary driving and must read as "nothing ahead", or the router would arm on
            # every waypoint in the course.
            self._obstacle_distance = math.inf
            self._lateral_error = 0.0
            self._heading_error = 0.0
            return

        heading = np.asarray(target.position[:2], float) - np.asarray(previous.position[:2], float)
        norm = float(np.linalg.norm(heading))
        if norm < 1e-6:
            self._obstacle_distance = math.inf
            return
        heading = heading / norm
        if self._obstacle_frame is not None:
            edge, normal, tangent = self._obstacle_frame
            (
                self._obstacle_distance,
                self._lateral_error,
                self._heading_error,
            ) = _obstacle_coordinates(self._position[:2], self._ypr[0], edge, normal, tangent)
            return
        if math.isnan(self._obstacle_x):
            edge = np.asarray(target.position[:2], float)
        else:
            # A measured edge is given as a world x plane; project it onto the segment line.
            along = (self._obstacle_x - float(previous.position[0])) / (heading[0] or 1e-6)
            edge = np.asarray(previous.position[:2], float) + heading * along
        self._obstacle_distance = float(np.dot(edge - self._position[:2], heading))
        tangent = np.array([-heading[1], heading[0]])
        self._lateral_error = float(np.dot(self._position[:2] - edge, tangent))
        target_yaw = math.atan2(float(heading[1]), float(heading[0]))
        self._heading_error = (self._ypr[0] - target_yaw + math.pi) % (2.0 * math.pi) - math.pi

    def _state(self) -> RobotState:
        yaw, pitch, roll = self._ypr
        return RobotState(
            t=self._now(),
            segment=self._segment,
            position=self._position,
            yaw=yaw,
            pitch=pitch,
            roll=roll,
            speed=self._speed,
            yaw_rate=self._yaw_rate,
            odom_time=self._odom_time,
            lidar_time=self._lidar_time,
            heightmap_time=self._heightmap_time,
            obstacle_distance=self._obstacle_distance,
            lateral_error=self._lateral_error,
            heading_error=self._heading_error,
            travelled=self._travelled,
            course_finished=self._finished,
        )

    def _tick(self) -> None:
        self._update_segment()
        state = self._state()
        observation = PolicyObservation(
            t=state.t,
            segment=state.segment,
            position=state.position,
            yaw=state.yaw,
            pitch=state.pitch,
            roll=state.roll,
            linear_velocity=np.array(
                [state.speed * math.cos(state.yaw), state.speed * math.sin(state.yaw), 0.0]
            ),
            yaw_rate=state.yaw_rate,
            joint_positions=np.zeros(16),
            joint_velocities=np.zeros(16),
            obstacle_delta=np.array([state.obstacle_distance, state.lateral_error, 0.0]),
            odom_time=state.odom_time,
            lidar_time=state.lidar_time,
            heightmap_time=state.heightmap_time,
        )
        out = self.router.tick(state, self._nav_command, observation)

        # The climb policy holds /JOINTS_CMD only while it is actually driving, and hands it
        # straight back. Granting on entry to CLIMB and revoking on every other mode is more
        # robust than pairing grant/release across transitions, which is the version that
        # leaks the grant when a transition is missed.
        if out.source is Source.POLICY and out.joints is not None:
            self.arbiter.grant(JointArbiter.CLIMB)
            self.arbiter.forward(JointArbiter.CLIMB, out.joints)
        else:
            self.arbiter.grant(JointArbiter.OFFICIAL)
        # Asked every tick rather than once per transition. The gate ignores a request for
        # the owner it already has, so repeating costs nothing, and a single request lost on
        # the way to another process would otherwise leave the two permanently disagreeing.
        self._owner_pub.publish(String(data=self.arbiter.owner))

        self._publish_command(out)
        self._publish_status(out)

    def _on_joint_owner(self, msg: String) -> None:
        if msg.data != self._actual_owner:
            self.get_logger().info(f"joint command owner is now {msg.data}")
        self._actual_owner = msg.data

    def _publish_command(self, out) -> None:
        """One publish, or a deliberate silence. Never two."""
        if out.source is Source.NONE:
            return
        command = out.command
        if command is None:
            # A JOINT-mode policy tick produces no Twist. Holding the body still is the
            # correct thing to send: the joints are being commanded elsewhere and a stale
            # Twist would fight them.
            command = (0.0, 0.0, 0.0)
        scale = out.speed_scale
        msg = Twist()
        msg.linear.x = float(command[0]) * scale
        msg.linear.y = float(command[1]) * scale
        msg.angular.z = float(command[2]) * scale
        self.cmd_pub.publish(msg)

    def _publish_status(self, out) -> None:
        self.mode_pub.publish(String(data=out.mode.value))
        self.source_pub.publish(String(data=out.source.value))
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "mode": out.mode.value,
                        "source": out.source.value,
                        "reason": out.reason,
                        "policy_status": (out.policy_status.value if out.policy_status else None),
                        "active_policy": self.router.active_policy_name,
                        "retries": self.router.retries,
                        "joint_owner": self.arbiter.owner,
                        "joint_owner_actual": self._actual_owner,
                    }
                )
            )
        )
        for t, was, now, why in self.router.history[self._history_seen :]:
            self.transition_pub.publish(String(data=f"{t:.3f} {was.value} -> {now.value}: {why}"))
            self.get_logger().info(f"{was.value} -> {now.value}: {why}")
        self._history_seen = len(self.router.history)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StrategyRouterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A router that stops without zeroing leaves the last command in force, which for a
        # node whose whole purpose is command ownership would be a poor way to go out.
        with contextlib.suppress(Exception):
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
