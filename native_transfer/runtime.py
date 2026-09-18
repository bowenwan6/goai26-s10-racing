"""Native Start/B test harness. Observer by default; no native command publishers.

The existing WaypointFollowerNode runs with captured outputs inside this process.
NativeGaitRouter wraps the existing Router and is the only native output boundary.
--enable-motion only makes the explicit arm service available; it does not arm.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
import rclpy
from drdds.msg import Gait, LocationStatus, MotionInfo, MotionStatus, NavCmd, StdMsgInt32, Steer
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from std_srvs.srv import Trigger

from native_transfer.contracts import (
    SourceClock,
    admission_reasons,
    conservative_scan,
    validate_route,
)
from native_transfer.router import Feedback, Limits, NativeGaitRouter
from real_transfer.geometry import (
    base_pose,
    decode_pointcloud2,
    height_grid,
    points_in_yaw_frame,
    yaw_of,
)
from s10_auto_nav.follower_node import WaypointFollowerNode
from s10_auto_nav.strategy.router import RobotState


class Capture:
    def __init__(self):
        self.last = None

    def publish(self, message):
        self.last = message


class NativeTest(Node):
    def __init__(self, args, log):
        super().__init__("native_start_b", enable_rosout=False)
        self.args = args
        self.config = json.loads(Path(args.config).read_text())
        self.points = validate_route(json.loads(Path(args.route).read_text()))
        if args.velocity_probe:
            distance = np.linalg.norm(
                np.subtract(self.points[-1]["position"], self.points[0]["position"])
            )
            if (
                len(self.points) != 2
                or not 0.20 <= distance <= 0.30
                or abs(self.points[1]["position"][2] - self.points[0]["position"][2]) > 0.03
                or self.points[0]["kind"] != self.points[1]["kind"]
            ):
                raise ValueError(
                    "velocity probe needs two same-gait points 0.20-0.30m apart on flat ground"
                )
        self.gate = NativeGaitRouter(
            [args.probe_gait] if args.probe_gait else [p["kind"] for p in self.points],
            limits=Limits(max_duration=8, max_flat=0.10, max_stairs=0.10)
            if args.velocity_probe
            else None,
            probe_only=bool(args.probe_gait),
        )
        self.source_clock = SourceClock()
        self.motion = self.location = self.hes = None
        self.pose = None
        self.times = {}
        self.input_faults = {}
        self.sensor_info = {}
        self.map_context = None
        self.map_context_session = None
        self.map_from_base = None
        self.pose_xyz = None
        self.previous_pose = None
        self.perception_valid = False
        self.command = (0.0, 0.0, 0.0)
        self.progress = 0.0
        self.nav_pub = self.gait_pub = None
        self.latest_feedback = None
        self.last_decision = None
        self.last_report = 0.0
        self.start_time = time.monotonic()
        self.log = log
        from native_transfer.app_control import AppControl
        self.app_control = AppControl(args.app_control_file) if getattr(args, "app_control_file", None) else None
        self.app_event = None
        self.follower = None
        if not args.probe_gait:
            # Keep the production follower logic; only capture its command transport.
            # All remaining ROS outputs are remapped under the test namespace.
            import yaml

            body_points = copy.deepcopy(self.points)
            offset = self.config.get("body_z_offset")
            for p in body_points:
                p["position"][2] += float(offset or 0)
            course = Path(args.output + ".course.yaml")
            with course.open("x") as handle:
                yaml.safe_dump({"waypoints": body_points}, handle)
            overrides = {
                "course_file": str(course.resolve()),
                "control_rate": 10.0,
                "height_tolerance": 0.20,
                "score_radius": 0.03 if args.velocity_probe else 0.20,
                "cmd_vel_topic": "/native_start_b/follower_candidate",
                "max_forward": 0.20,
                "terrain_max_forward": 0.20,
                "max_lateral": 0.05,
                "max_yaw_rate": 0.20,
                "lookahead": 0.5,
                "lookahead_speed_gain": 0.0,
                "pivot_threshold_deg": 10.0,
                "align_falloff_deg": 30.0,
                "forward_slew": 0.2,
                "lateral_slew": 0.1,
                "yaw_slew": 0.3,
                "stall_speed": 0.02,
                "stall_timeout": 4.0,
                "progress_timeout": 15.0,
                "climb_speed": 0.15,
                "climb_timeout": 180.0,
                "fast_flat_waypoints": [-1],
                "corner_retreat_waypoints": [-1],
                "committed_terrain_waypoints": [-1],
                "committed_runup_waypoints": [-1],
                "same_level_corridor_waypoints": [-1],
                "route_hint_waypoints": [-1],
                "route_hint_points": [0.0, 0.0],
                "corner_preview_waypoints": [-1],
                "waypoint_speed_limit_indices": [-1],
                "waypoint_speed_limit_values": [0.2],
            }
            # Disable automatic recovery commands at the native boundary (router latches
            # reverse attempts). Race-specific waypoint exceptions above do not apply.
            self.follower = WaypointFollowerNode(
                use_global_arguments=False,
                parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()],
                cli_args=[
                    "--ros-args",
                    "-r",
                    "/nav/progress:=/native_start_b/progress",
                    "-r",
                    "/nav/finished:=/native_start_b/finished",
                    "-r",
                    "/nav/terrain:=/native_start_b/terrain",
                ],
            )
            self.follower.timer.cancel()
            for name in ("cmd_pub", "progress_pub", "terrain_pub", "finished_pub"):
                self.follower.destroy_publisher(getattr(self.follower, name))
                setattr(self.follower, name, Capture())
        self.create_subscription(Odometry, "/ODOM", self.on_pose, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.config["cloud_topic"], self.on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(
            MotionInfo, "/MOTION_INFO", self.on_motion, qos_profile_sensor_data
        )
        self.create_subscription(
            LocationStatus, "/LOCATION_STATUS", self.on_location, qos_profile_sensor_data
        )
        self.create_subscription(StdMsgInt32, "/HES_STATUS", self.on_hes, qos_profile_sensor_data)
        self.create_subscription(String, "/native_start_b/map_context", self.on_context, 10)
        self.create_subscription(Steer, "/HANDLE_STEER", self.on_handle, qos_profile_sensor_data)
        self.create_subscription(
            MotionStatus, "/MOTION_STATUS", self.on_motion_status, qos_profile_sensor_data
        )
        # Stop requests are always available, including in observer mode.
        self.create_service(Trigger, "/native_start_b/cancel", self.cancel)
        if args.enable_motion:
            self.create_service(Trigger, "/native_start_b/arm", self.arm)
        self.create_timer(0.1, self.tick)

    def stamp(self, msg):
        stamp = msg.header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def fault(self, key, reason):
        self.input_faults[key] = reason
        if self.gate.ever_armed:
            self.gate.fail(time.monotonic(), reason)

    def valid(self, key):
        if not self.gate.ever_armed:
            self.input_faults.pop(key, None)

    def on_motion(self, msg):
        try:
            self.source_clock.check("motion", self.stamp(msg), time.time())
            self.motion = msg.data
            self.times["motion"] = time.monotonic()
            self.valid("motion")
        except ValueError as exc:
            self.fault("motion", str(exc))

    def on_location(self, msg):
        # Firmware's LocationStatus header is zero. Preserve that fact; its arrival
        # age plus the separately observed global-mode/session heartbeat are required.
        self.location = msg.data
        self.times["location"] = time.monotonic()

    def on_hes(self, msg):
        self.hes = msg.value
        self.times["hes"] = time.monotonic()

    def on_handle(self, msg):
        values = [getattr(msg.data, k) for k in ("x", "y", "z", "roll", "pitch", "yaw")]
        if any(not math.isfinite(v) or abs(v) > 0.05 for v in values):
            self.fault("operator", "operator_stick_active")
        else:
            self.valid("operator")

    def on_motion_status(self, msg):
        errors = [k for k in msg.data.get_fields_and_field_types() if getattr(msg.data, k) != 0]
        if errors:
            self.fault("motion_status", "motion_fault:" + ",".join(errors))
        else:
            self.valid("motion_status")

    def on_context(self, msg):
        try:
            value = json.loads(msg.data)
            if not 0 <= time.time() - value["wall_time"] <= 2.0:
                raise ValueError("map_context_stale")
            session = (value["map_id"], value["session_id"])
            if self.map_context_session is not None and session != self.map_context_session:
                raise ValueError("map_or_localization_session_changed")
            self.map_context_session = session
            self.map_context = value
            self.times["context"] = time.monotonic()
            self.valid("context")
        except (ValueError, KeyError, TypeError) as exc:
            self.fault("context", str(exc))

    def on_pose(self, msg):
        now = time.monotonic()
        try:
            self.source_clock.check("pose", self.stamp(msg), time.time())
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            raw = {"position": [p.x, p.y, p.z], "orientation": [q.x, q.y, q.z, q.w]}
            if self.args.probe_gait:
                transform = np.eye(4)
            else:
                frames = self.config["frames"]
                if (msg.header.frame_id, msg.child_frame_id) != (
                    frames["map"],
                    frames["odom_child"],
                ):
                    raise ValueError("odom_frame_contract_mismatch")
                transform = self.config["odom_child_from_base"]
            mb = base_pose(raw, transform)
            xyz = mb[:3, 3]
            if self.previous_pose is not None:
                old_time, old_xyz = self.previous_pose
                if np.linalg.norm(xyz - old_xyz) > 0.10 + (now - old_time):
                    raise ValueError("localization_pose_jump")
            self.previous_pose = (now, xyz.copy())
            self.pose_xyz, self.map_from_base = xyz, mb
            self.pose = copy.deepcopy(msg)
            # Give the original follower the calibrated full base pose.
            from scipy.spatial.transform import Rotation

            rot = Rotation.from_matrix(mb[:3, :3]).as_quat()
            (
                self.pose.pose.pose.position.x,
                self.pose.pose.pose.position.y,
                self.pose.pose.pose.position.z,
            ) = map(float, xyz)
            qout = self.pose.pose.pose.orientation
            qout.x, qout.y, qout.z, qout.w = map(float, rot)
            self.times["pose"] = now
            self.valid("pose")
            self.sensor_info["odom"] = {
                "frame": msg.header.frame_id,
                "child": msg.child_frame_id,
                "raw_xyz": raw["position"],
            }
        except (ValueError, KeyError, TypeError) as exc:
            self.fault("pose", str(exc))

    def on_cloud(self, msg):
        if self.args.probe_gait:
            return
        self.perception_valid = False
        try:
            self.source_clock.check("cloud", self.stamp(msg), time.time())
            self.sensor_info["cloud"] = {
                "frame": msg.header.frame_id,
                "points": msg.width * msg.height,
            }
            if msg.header.frame_id != self.config["frames"]["cloud"]:
                raise ValueError("cloud_frame_contract_mismatch")
            if self.pose is None or abs(self.stamp(msg) - self.stamp(self.pose)) > 0.10:
                raise ValueError("pose_cloud_time_skew")
            points = points_in_yaw_frame(
                decode_pointcloud2(msg), self.config["base_from_cloud"], self.map_from_base
            )
            grid, valid, _ = height_grid(points)
            ranges = conservative_scan(points)
            self.sensor_info["height_valid_fraction"] = float(valid.mean())
            self.sensor_info["scan_known_fraction"] = float(np.isfinite(ranges).mean())
            # Unknown terrain is never converted into a clear scan or a flat cell.
            self.perception_valid = bool(valid.all() and np.isfinite(ranges).all())
            self.times["cloud"] = time.monotonic()
            self.valid("cloud")
            if not self.perception_valid:
                return
            scan = LaserScan()
            scan.header = msg.header
            scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / len(ranges)
            scan.range_min, scan.range_max = 0.05, 11.0
            scan.ranges = ranges.astype(float).tolist()
            height = Float32MultiArray(data=grid.ravel().tolist())
            height.layout.dim = [MultiArrayDimension(size=13), MultiArrayDimension(size=9)]
            self.follower._scan_callback(scan)
            self.follower._heightmap_callback(height)
        except (ValueError, KeyError, TypeError) as exc:
            self.fault("cloud", str(exc))

    def feedback(self):
        now = time.monotonic()
        if self.pose is None or self.motion is None:
            return None
        if self.follower is not None:
            self.follower._odom_callback(self.pose)
            # Reuse the follower's existing router handoff reset. Waiting for native
            # gait feedback must not accumulate a stall/back-off to execute on release.
            self.follower._strategy_mode_callback(
                String(data="navigate" if self.gate.state == "active" else "climb")
            )
            if self.perception_valid:
                self.follower._control_step()
                msg = self.follower.cmd_pub.last
                if msg is not None:
                    self.command = (msg.linear.x, msg.linear.y, msg.angular.z)
                    self.times["command"] = now
            cursor = self.follower.course.cursor
            complete = self.follower.course.finished
        else:
            cursor, complete = 0, False
        xyz = self.pose_xyz
        yaw = yaw_of(self.map_from_base)
        pitch = math.asin(float(np.clip(-self.map_from_base[2, 0], -1, 1)))
        roll = math.atan2(self.map_from_base[2, 1], self.map_from_base[2, 2])
        if not self.args.probe_gait and cursor < len(self.points):
            target = np.array(self.points[cursor]["position"][:2])
            start = np.array(self.points[max(0, cursor - 1)]["position"][:2])
            line = target - start
            length = np.linalg.norm(line)
            along = 0.0 if length < 1e-6 else float(np.dot(xyz[:2] - start, line / length))
            closest = start if length < 1e-6 else start + np.clip(along, 0, length) * line / length
            if np.linalg.norm(xyz[:2] - closest) > 0.35:
                self.fault("route", "route_corridor_exceeded")
            else:
                self.valid("route")
            self.progress = max(self.progress, cursor * 100.0 + max(0.0, along))
        robot = RobotState(
            t=now,
            segment=(max(0, cursor - 1), cursor),
            position=xyz,
            yaw=yaw,
            pitch=pitch,
            roll=roll,
            speed=math.hypot(self.motion.vel_x, self.motion.vel_y),
            yaw_rate=self.motion.vel_yaw,
            odom_time=self.times.get("pose", 0),
            lidar_time=self.times.get("cloud", 0),
            heightmap_time=self.times.get("cloud", 0),
            travelled=self.progress,
            course_finished=complete,
        )
        expected_count = 1 if self.nav_pub else 0
        exclusive = (
            self.count_publishers("/NAV_CMD") == expected_count
            and self.count_subscribers("/NAV_CMD") >= 1
            and self.count_subscribers("/GAIT") >= 1
        )
        context_ok = (
            self.map_context is not None
            and now - self.times.get("context", 0) < 2.0
            and self.map_context.get("map_id") == self.config["map_id"]
            and self.map_context.get("global_mode") is True
            and bool(self.map_context.get("session_id"))
        )
        localized = (
            self.location is not None
            and context_ok
            and self.location.total_status == self.config.get("healthy_location_code")
        )
        return Feedback(
            robot=robot,
            gait=self.motion.gait_state.gait,
            motion_state=self.motion.motion_state.state,
            hes=self.hes if self.hes is not None else -1,
            motion_received=self.times.get("motion", 0),
            status_received=self.times.get("location", 0),
            hes_received=self.times.get("hes", 0),
            command_received=self.times.get("command", 0),
            localized=localized,
            perception_valid=self.perception_valid,
            exclusive_control=exclusive,
            command=self.command,
            external_fault=";".join(self.input_faults.values()),
        ), cursor

    def arm(self, request, response):
        if self.args.probe_gait:
            reasons = (
                []
                if self.config.get("verified", {}).get("operator_on_site") is True
                else ["operator_on_site_unconfirmed"]
            )
        else:
            reasons = admission_reasons(self.config, policy_probe=self.args.velocity_probe)
        result = self.feedback()
        try:
            if result is None:
                raise ValueError("missing robot feedback")
            self.gate.arm(result[0], admission=not reasons)
            self.nav_pub = self.create_publisher(NavCmd, "/NAV_CMD", 1)
            self.gait_pub = self.create_publisher(Gait, "/GAIT", 1)
            response.success, response.message = True, "armed; zero/acknowledgement gate first"
        except ValueError as exc:
            response.success, response.message = False, ";".join([*reasons, str(exc)])
        return response

    def cancel(self, request, response):
        self.gate.cancel(time.monotonic())
        self.publish_zero()
        response.success, response.message = True, "cancel latched; restart required"
        return response

    def publish_zero(self):
        if self.nav_pub is not None:
            msg = NavCmd()
            msg.header.stamp = self.get_clock().now().to_msg()
            self.nav_pub.publish(msg)

    def tick(self):
        if self.app_control is not None:
            action = self.app_control.poll(time.monotonic())
            if action == "cancel":
                self.cancel(None, Trigger.Response())
                self.app_event = {"action": "cancel", "success": True, "message": "app_cancel_or_lease_expired"}
            elif action == "arm":
                response = self.arm(None, Trigger.Response())
                self.app_event = {"action": "arm", "success": response.success, "message": response.message}
        result = self.feedback()
        if result is not None:
            feedback, cursor = result
            self.latest_feedback = feedback
            self.last_decision = decision = self.gate.tick(feedback, cursor)
            if decision.publish and self.nav_pub is not None:
                msg = NavCmd()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.data.x_vel, msg.data.y_vel, msg.data.yaw_vel = map(float, decision.command)
                self.nav_pub.publish(msg)
                if decision.request_gait is not None:
                    gait = Gait()
                    gait.header.stamp = self.get_clock().now().to_msg()
                    gait.data.gait = decision.request_gait
                    self.gait_pub.publish(gait)
        elif self.gate.ever_armed:
            self.gate.cancel(time.monotonic(), "feedback_missing")
            self.publish_zero()
        now = time.monotonic()
        record = {
            "wall_time": time.time(),
            "elapsed": now - self.start_time,
            "observer": not self.args.enable_motion,
            "probe_gait": self.args.probe_gait,
            "velocity_probe": self.args.velocity_probe,
            "native_publishers_created": self.nav_pub is not None,
            "state": self.gate.state,
            "reason": self.gate.reason,
            "admission_missing": admission_reasons(self.config),
            "input_faults": self.input_faults,
            "sensors": self.sensor_info,
            "location_code": None if self.location is None else self.location.total_status,
            "gait": None if self.motion is None else self.motion.gait_state.gait,
            "motion_state": None if self.motion is None else self.motion.motion_state.state,
            "measured_velocity": None
            if self.motion is None
            else [self.motion.vel_x, self.motion.vel_y, self.motion.vel_yaw],
            "nav_cmd_publishers": self.count_publishers("/NAV_CMD"),
            "nav_cmd_subscribers": self.count_subscribers("/NAV_CMD"),
            "actual_output": None if self.last_decision is None else asdict(self.last_decision),
            "target": None if result is None else result[1],
            "position": None if self.pose_xyz is None else self.pose_xyz.tolist(),
            "accepted_gaits": sorted(self.gate.accepted_gaits),
            "velocity_response_verified": False,
            "app_event": self.app_event,
            "hes": self.hes,
            "map_context": self.map_context,
            "stream_ages": {k: now - v for k, v in self.times.items()},
            "gait_subscribers": self.count_subscribers("/GAIT"),
        }
        self.log.write(json.dumps(record, allow_nan=False) + "\n")
        if now - self.last_report >= 2:
            print(json.dumps(record, allow_nan=False), flush=True)
            self.last_report = now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--route", required=True)
    parser.add_argument("--output", required=True, help="new JSONL path; existing files refused")
    parser.add_argument("--duration", type=float, default=15)
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--probe-gait", choices=["flat", "stairs"])
    parser.add_argument("--app-control-file", help="private local app lease; expired lease latches cancel")
    parser.add_argument(
        "--velocity-probe",
        action="store_true",
        help="bounded 0.20-0.30m policy response test; still requires explicit arm",
    )
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 900:
        parser.error("duration must be in (0,900] seconds")
    if args.probe_gait and args.velocity_probe:
        parser.error("stationary acknowledgement and velocity tests are separate sessions")
    if args.app_control_file and not args.enable_motion:
        parser.error("app control requires --enable-motion")
    rclpy.init(args=[])
    with Path(args.output).open("x", buffering=1) as log:
        node = NativeTest(args, log)
        try:
            end = time.monotonic() + args.duration
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.gate.cancel(time.monotonic(), "process_shutdown")
            for _ in range(3):
                node.publish_zero()
                rclpy.spin_once(node, timeout_sec=0.02)
            if node.follower is not None:
                node.follower.destroy_node()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
