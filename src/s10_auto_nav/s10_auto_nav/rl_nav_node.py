"""ROS 2 node for RL route navigation: the route follower core and the manoeuvre router, on the
robot. It runs the same code as the MuJoCo harness (s10-rl-sprint
``scripts/tools/route_rl_real_stack.py``), fed from topics instead of the simulator.

Subscribed:

``odom`` (nav_msgs/Odometry)
    Map-frame localisation pose; remap to the localiser's topic (``/ground_truth/odom`` in the
    contest simulator). Orientation gives yaw, pitch and roll; the body-frame twist gives forward
    speed and yaw rate.
``/perception/heightmap`` (std_msgs/Float32MultiArray)
    The 13x9 ``real_transfer.geometry.height_grid``: robot yaw frame, unknown cells <= -1.
``/scan`` (sensor_msgs/LaserScan)
    For the follower's local grid, and binned to 72 for the router's corridor check.
``points_topic`` (std_msgs/Float32MultiArray, N x 3, optional)
    LiDAR points in the robot's yaw frame relative to the base -- the cloud the height grid is built
    from. With ``map_surface_path`` it lets the router tell terrain the map knows from something new
    (``rl_nav.map_check``); without it the router falls back on the conservative scan.
``/joints/owner`` (std_msgs/String)
    Who the SDK joint gate says is driving.

Published:

``/cmd_vel`` (geometry_msgs/Twist)
    The one body-velocity command. Nothing else may publish it while this node runs: do not start
    the follower node, or remap it away.
``/strategy/joint_owner`` (std_msgs/String)
    ``official`` (walking actor), ``stairs_stable`` (stairs actor) or ``stop``. The SDK gate
    (``integration/joint_command_owner.hpp``) takes official -> stairs_stable as a moving handover
    and anything else through SafeHold. The strategy router node must not run alongside.
``/rl_nav/status`` (std_msgs/String)
    JSON: mode, manoeuvre, reason, target, s, owner requested and reported, the ALIGN checks.
``/nav/progress`` (std_msgs/Float32), ``/nav/finished`` (std_msgs/Bool)
    Waypoints reached / total, and done (both latched).

It will not act without a pose younger than ``odom_timeout`` (zero command, owner kept) or a height
grid younger than the router's ``grid_stale`` (the router stops by itself).

Parameters: ``route_path`` (prepared route_v2 JSON), ``maneuvers_path`` (its
``s10_rl_maneuvers_v1`` JSON), ``profile_path`` (policy capability profile; default the packaged
one), ``map_surface_path`` (``MapSurface`` .npz of the same map; optional), ``points_topic``,
``control_rate`` (Hz), ``body_z_offset``, ``lookahead``, ``fuse_frames``, ``max_step_flat``
(follower core, harness values), ``odom_timeout`` (s).
"""

from __future__ import annotations

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

from s10_auto_nav.pure_pursuit import PurePursuitController, PursuitGains
from s10_auto_nav.rl_nav import maneuvers as maneuver_io
from s10_auto_nav.rl_nav.capability import PolicyProfile
from s10_auto_nav.rl_nav.maneuver_router import ManeuverRouter, Mode, NavInput, RouterParams
from s10_auto_nav.rl_nav.map_check import MapSurface
from s10_auto_nav.route_follower import RouteFollowerConfig, RouteFollowerCore
from s10_auto_nav.route_planner import LocalGridConfig
from s10_auto_nav.route_v2 import CrossCheckConfig, RouteV2


def _rpy(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class RlNavNode(Node):
    def __init__(self) -> None:
        super().__init__("rl_nav")
        self.declare_parameter("route_path", "")
        self.declare_parameter("maneuvers_path", "")
        self.declare_parameter("profile_path", "")
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("body_z_offset", 0.42)
        self.declare_parameter("lookahead", 1.0)
        self.declare_parameter("fuse_frames", 3)
        self.declare_parameter("max_step_flat", 0.07)
        self.declare_parameter("odom_timeout", 0.3)
        self.declare_parameter("map_surface_path", "")
        self.declare_parameter("points_topic", "")

        route_path = str(self.get_parameter("route_path").value)
        man_path = str(self.get_parameter("maneuvers_path").value)
        if not route_path or not man_path:
            raise RuntimeError(
                "rl_nav needs route_path and maneuvers_path (route_prep + maneuvers output)"
            )
        prof_path = str(self.get_parameter("profile_path").value)
        self.profile = PolicyProfile.load(prof_path) if prof_path else PolicyProfile.default()
        self.rate = float(self.get_parameter("control_rate").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)

        route = RouteV2.load(route_path)
        gains = PursuitGains(
            max_forward=0.6,
            max_lateral=0.2,
            max_yaw_rate=0.6,
            lookahead=1.0,
            lookahead_speed_gain=0.0,
            pivot_threshold=math.radians(35.0),
            align_falloff=math.radians(60.0),
            brake_distance=0.05,
        )
        self.follower = RouteFollowerCore(
            route,
            RouteFollowerConfig(
                body_z_offset=float(self.get_parameter("body_z_offset").value),
                control_rate=self.rate,
                lookahead=float(self.get_parameter("lookahead").value),
                cross_check=CrossCheckConfig(stairs_pitch_deg=25.0),
                grid=LocalGridConfig(
                    max_step_flat=float(self.get_parameter("max_step_flat").value),
                    fuse_frames=int(self.get_parameter("fuse_frames").value),
                ),
            ),
            controller=PurePursuitController(gains),
        )
        mans = maneuver_io.load(man_path)
        surface_path = str(self.get_parameter("map_surface_path").value)
        surface = MapSurface.load(surface_path) if surface_path else None
        self.router = ManeuverRouter(
            self.follower.path, mans, RouterParams.from_profile(self.profile), map_surface=surface
        )
        self.n_wp = len(route.waypoints)
        self.reached = 0
        for m in mans:
            for w in m.warnings:
                self.get_logger().warn(f"{m.id} (s {m.s0:.1f}-{m.s1:.1f}): {w}")

        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.owner_pub = self.create_publisher(String, "/strategy/joint_owner", 10)
        self.status_pub = self.create_publisher(String, "/rl_nav/status", 10)
        self.progress_pub = self.create_publisher(Float32, "/nav/progress", latched)
        self.finished_pub = self.create_publisher(Bool, "/nav/finished", latched)
        self.create_subscription(Odometry, "odom", self._on_odom, 50)
        self.create_subscription(Float32MultiArray, "/perception/heightmap", self._on_grid, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(String, "/joints/owner", self._on_owner, 10)
        points_topic = str(self.get_parameter("points_topic").value)
        if points_topic:
            self.create_subscription(Float32MultiArray, points_topic, self._on_points, 5)

        self.pose = None  # (x, y, z, roll, pitch, yaw, v_forward, yaw_rate)
        self.pose_t = -math.inf
        self.grid = self.valid = None
        self.grid_t = -math.inf
        self.grid_fresh = False
        self.ranges = self.angles = None
        self.scan_fresh = False
        self.owner_reported = None
        self.points = None
        self.points_t = -math.inf
        self.owner_requested = None
        self.last_mode = None
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate, self._tick)
        self.progress_pub.publish(Float32(data=0.0))
        self.get_logger().info(
            f"rl_nav: {self.n_wp} waypoints, {len(mans)} manoeuvres, "
            f"{self.follower.path.length:.1f} m; profile: {self.profile.source}"
        )

    # ------------------------------------------------------------------ inputs
    def _now(self) -> float:
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        roll, pitch, yaw = _rpy(q.x, q.y, q.z, q.w)
        tw = msg.twist.twist
        self.pose = (p.x, p.y, p.z, roll, pitch, yaw, tw.linear.x, tw.angular.z)
        self.pose_t = self._now()

    def _on_grid(self, msg: Float32MultiArray) -> None:
        data = np.asarray(msg.data, float)
        if data.size != 13 * 9:
            self.get_logger().warn(
                f"height grid of {data.size} cells ignored (13x9 expected)",
                throttle_duration_sec=5.0,
            )
            return
        grid = data.reshape(13, 9)
        self.grid, self.valid = grid, np.isfinite(grid) & (grid > -1.0)
        self.grid_t = self._now()
        self.grid_fresh = True

    def _on_scan(self, msg: LaserScan) -> None:
        r = np.asarray(msg.ranges, float)
        r[r >= msg.range_max - 1e-3] = math.inf
        self.ranges, self.angles = r, msg.angle_min + msg.angle_increment * np.arange(len(r))
        self.scan_fresh = True

    def _scan72(self):
        """The router's corridor check wants the 72-bin conservative scan (5 deg bins from -pi,
        robot yaw frame, NaN where nothing returned): the nearest return per bin."""
        if self.ranges is None:
            return None
        out = np.full(72, np.nan)
        ok = np.isfinite(self.ranges) & (self.ranges > 0.25)
        if ok.any():
            ids = np.clip(
                np.floor(
                    (np.arctan2(np.sin(self.angles[ok]), np.cos(self.angles[ok])) + math.pi)
                    / (2 * math.pi)
                    * 72
                ).astype(int),
                0,
                71,
            )
            best = np.full(72, np.inf)
            np.minimum.at(best, ids, self.ranges[ok])
            out[np.isfinite(best)] = best[np.isfinite(best)]
        return out

    def _on_points(self, msg: Float32MultiArray) -> None:
        data = np.asarray(msg.data, float)
        if data.size % 3 == 0:
            self.points = data.reshape(-1, 3)
            self.points_t = self._now()

    def _on_owner(self, msg: String) -> None:
        self.owner_reported = msg.data

    # ------------------------------------------------------------------ tick
    def _publish(self, cmd, owner) -> None:
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = (float(v) for v in cmd)
        self.cmd_pub.publish(tw)
        # Re-sent every tick, not only on change: the gate keys on the latest request, and one
        # dropped message must not leave the robot on the wrong actor.
        self.owner_pub.publish(String(data=owner))
        self.owner_requested = owner

    def _tick(self) -> None:
        t = self._now()
        if self.pose is None or t - self.pose_t > self.odom_timeout:
            self._publish((0.0, 0.0, 0.0), self.owner_requested or "official")
            self._status(t, "no pose", None)
            return
        x, y, z, roll, pitch, yaw, v_fwd, yaw_rate = self.pose
        grid = self.grid if self.grid_fresh else None
        valid = self.valid if self.grid_fresh else None
        scan = (self.ranges, self.angles) if self.scan_fresh else (None, None)
        self.grid_fresh = self.scan_fresh = False
        f = self.follower.step(
            t, (x, y, z, yaw), grid, valid, scan[0], scan[1], pitch=pitch, roll=roll
        )
        if f.reached:
            self.reached += len(f.reached)
            self.progress_pub.publish(Float32(data=self.reached / self.n_wp))
            self.get_logger().info(f"{', '.join(f.reached)} reached ({self.reached}/{self.n_wp})")
        out = self.router.step(
            NavInput(
                t,
                x,
                y,
                z,
                yaw,
                pitch,
                roll,
                yaw_rate,
                v_fwd,
                self.grid,
                self.valid,
                self.grid_t,
                f,
                self.follower.last_grid,
                self.owner_reported,
                self._scan72(),
                self.points if self._now() - self.points_t < 0.5 else None,
            )
        )
        self._publish(out.command, out.owner)
        if out.mode != self.last_mode:
            log = self.router.log[-1] if self.router.log else {}
            self.get_logger().info(f"{self.last_mode} -> {out.mode.value}: {log.get('why', '')}")
            self.last_mode = out.mode
        if out.mode == Mode.DONE:
            self.finished_pub.publish(Bool(data=True))
        self._status(t, out.reason, out, f)

    def _status(self, t, reason, out, f=None) -> None:
        doc = {
            "t": round(t, 2),
            "reason": reason,
            "owner_requested": self.owner_requested,
            "owner_reported": self.owner_reported,
        }
        if out is not None:
            m = self.router._man()
            doc.update(
                {
                    "mode": out.mode.value,
                    "maneuver": m.id if m else None,
                    "cmd": [round(v, 3) for v in out.command],
                    "info": out.info,
                }
            )
        if f is not None:
            doc.update({"target": f.target_id, "s": round(float(f.s), 2), "follower": f.status})
        self.status_pub.publish(String(data=json.dumps(doc)))

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RlNavNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
