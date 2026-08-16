"""Watch a segment run, record it, and decide when it is over.

This is the only node the full-stack segment harness adds to the race launch. Everything
that drives the robot -- simulator, perception, follower with its local planner, step commit
and stall recovery, the strategy router, and the SDK's ``rl_deploy`` -- is the production
stack, launched from the production launch file with the production parameters. The one
thing that differs is where the robot starts, and that is done by the simulator reading
``S10_SPAWN_XY``; see :mod:`s10_perception.segment_spawn`.

The distinction matters because the earlier, cheaper harness
(``training/s10_climb/segment_runner.py``) composed only the pure-pursuit controller and the
course. It reported four "the robot stalls on the approach" failures that the production
follower's :class:`~s10_auto_nav.step_commit.StepCommit` exists specifically to prevent, and
three "the robot drives into a wall" failures on a stack that had no lidar and no local
planner. Those results describe pure pursuit on its own; they do not describe the race. This
node exists so that a claim about the race is made against the thing that runs in the race.

It records rather than judges wherever it can. The outcome is decided from ground truth --
did the base reach the end waypoint, did it fall, did it run out of time -- and the router's
own account of itself is written down beside it, not trusted in place of it.

Subscribed: ``/ground_truth/odom``, ``/cmd_vel``, ``/scan``, ``/nav/finished``,
``/strategy/mode``, ``/strategy/source``, ``/strategy/status``, ``/strategy/transition``.
Written: ``<out>/<name>.csv`` per control tick, ``<out>/<name>.json`` for the summary.
"""

from __future__ import annotations

import contextlib
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from s10_auto_nav.waypoints import Course

#: Past this the robot is on its way over and will not recover; ending the run here saves
#: the seconds it would otherwise spend sliding, and matches the segment runner's threshold
#: so the two harnesses' numbers can be compared.
FALL_TILT_DEG = 70.0
#: How far below its own starting height the base may sink before the run is called a fall.
#: Generous, because the robot legitimately drops off ledges on this course.
FALL_BELOW_M = 1.5
#: Ground speed under which the robot counts as stalled, and for how long before it is
#: reported. Reported only -- a stall is not a termination, because recovering from one is
#: exactly what the production stack is supposed to do and cutting the run short would hide
#: whether it did.
STALL_SPEED = 0.05
STALL_SECONDS = 5.0

#: How long after the grace period the robot has to have reached its spawn height before
#: the start pose is called illegal. The SDK's stand-up takes about two seconds.
STAND_SECONDS = 4.0


def _yaw_pitch_roll(q) -> tuple[float, float, float]:
    x, y, z, w = q.x, q.y, q.z, q.w
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    return yaw, pitch, roll


def _tilt_deg(q) -> float:
    """Angle between the body's up axis and world up, degrees."""
    up_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


class SegmentRecorder(Node):
    def __init__(self) -> None:
        super().__init__("segment_recorder")

        self.declare_parameter("course_file", "")
        self.declare_parameter("out_dir", "/tmp/s10_segment")
        self.declare_parameter("run_name", "segment")
        self.declare_parameter("start_waypoint", 0)
        self.declare_parameter("end_waypoint", 1)
        self.declare_parameter("seed", 0)
        self.declare_parameter("max_time", 120.0)
        # The SDK stands the robot up from its crouched spawn before the policy takes over.
        # Nothing before that says anything about the segment, so the clock and the fall
        # detector both wait it out.
        self.declare_parameter("grace", 3.0)
        self.declare_parameter("record_rate", 20.0)
        self.declare_parameter("reach_radius", 0.35)

        course_file = str(self.get_parameter("course_file").value)
        if not course_file:
            raise RuntimeError("Parameter 'course_file' is required")
        self.course = Course.from_yaml(course_file)
        self.start = int(self.get_parameter("start_waypoint").value)
        self.end = int(self.get_parameter("end_waypoint").value)
        self.seed = int(self.get_parameter("seed").value)
        self.max_time = float(self.get_parameter("max_time").value)
        self.grace = float(self.get_parameter("grace").value)
        self.reach_radius = float(self.get_parameter("reach_radius").value)
        self.out_dir = Path(str(self.get_parameter("out_dir").value))
        self.run_name = str(self.get_parameter("run_name").value)

        # The follower is given a sub-course, so its own waypoint numbering starts at zero.
        # Ours is the real course's, because that is what every other artefact is indexed by.
        self.targets = [w.position for w in self.course.waypoints]
        self.goal = np.asarray(self.targets[-1], dtype=float)

        self._odom = None
        self._cmd = Twist()
        self._scan_ahead = float("inf")
        self._mode = "-"
        self._source = "-"
        self._status = ""
        self._transitions: list[str] = []
        self._nav_finished = False
        self._terrain = "-"

        self._rows: list[dict] = []
        self._t0 = None
        self._start_z = None
        self._travelled = 0.0
        self._last_xy = None
        self._max_tilt = 0.0
        self._stalled_for = 0.0
        self._stalls = 0
        self._outcome = "did not start"
        self._invalid = False
        self._stood_up = False
        self._finished = False

        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_odom, 50)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Bool, "/nav/finished", self._on_finished, latched)
        self.create_subscription(String, "/strategy/mode", self._on_mode, 10)
        self.create_subscription(String, "/strategy/source", self._on_source, 10)
        self.create_subscription(String, "/strategy/status", self._on_status, 10)
        self.create_subscription(String, "/strategy/transition", self._on_transition, 20)
        self.create_subscription(String, "/nav/terrain", self._on_terrain, 10)

        self._dt = 1.0 / float(self.get_parameter("record_rate").value)
        self.timer = self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f"[segment] recording {self.start}->{self.end} seed {self.seed} "
            f"to {self.out_dir / self.run_name}.csv, limit {self.max_time:.0f} s"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd = msg

    def _on_terrain(self, msg: String) -> None:
        self._terrain = msg.data

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, float)
        angles = msg.angle_min + msg.angle_increment * np.arange(len(ranges))
        # What is in front, taken as the beams within 20 degrees of straight ahead. A beam
        # that hit nothing comes back at range_max and would otherwise read as a wall there.
        ahead = np.abs(angles) < math.radians(20.0)
        forward = ranges[ahead]
        forward = forward[forward < msg.range_max - 1e-3]
        self._scan_ahead = float(forward.min()) if forward.size else float("inf")

    def _on_finished(self, msg: Bool) -> None:
        self._nav_finished = bool(msg.data)

    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data

    def _on_source(self, msg: String) -> None:
        self._source = msg.data

    def _on_status(self, msg: String) -> None:
        self._status = msg.data

    def _on_transition(self, msg: String) -> None:
        self._transitions.append(msg.data)

    def _tick(self) -> None:
        if self._finished or self._odom is None:
            return

        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0

        pose = self._odom.pose.pose
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        yaw, pitch, roll = _yaw_pitch_roll(pose.orientation)
        tilt = _tilt_deg(pose.orientation)
        v = self._odom.twist.twist.linear
        speed = float(math.hypot(v.x, v.y))

        if self._last_xy is not None:
            self._travelled += float(np.linalg.norm(position[:2] - self._last_xy))
        self._last_xy = position[:2].copy()
        if self._start_z is None:
            self._start_z = float(position[2])

        settled = t >= self.grace
        if settled:
            self._max_tilt = max(self._max_tilt, tilt)
            if speed < STALL_SPEED:
                self._stalled_for += self._dt
                if self._stalled_for >= STALL_SECONDS:
                    self._stalls += 1
                    self._stalled_for = 0.0
            else:
                self._stalled_for = 0.0

        distance_to_goal = float(np.linalg.norm(position[:2] - self.goal[:2]))
        self._rows.append(
            {
                "t": round(t, 3),
                "x": round(float(position[0]), 4),
                "y": round(float(position[1]), 4),
                "z": round(float(position[2]), 4),
                "yaw_deg": round(math.degrees(yaw), 2),
                "pitch_deg": round(math.degrees(pitch), 2),
                "roll_deg": round(math.degrees(roll), 2),
                "tilt_deg": round(tilt, 2),
                "speed": round(speed, 3),
                "cmd_forward": round(self._cmd.linear.x, 3),
                "cmd_lateral": round(self._cmd.linear.y, 3),
                "cmd_yaw": round(self._cmd.angular.z, 3),
                "scan_ahead": round(min(self._scan_ahead, 99.0), 3),
                "goal_distance": round(distance_to_goal, 3),
                "mode": self._mode,
                "source": self._source,
                "terrain": self._terrain,
            }
        )

        if settled and not self._stood_up and position[2] >= self._start_z:
            self._stood_up = True

        if self._nav_finished or distance_to_goal <= self.reach_radius:
            self._stop("reached the end waypoint", reached=True)
        elif settled and not self._stood_up and t >= self.grace + STAND_SECONDS:
            # The robot is placed crouched and the SDK stands it up, so the base can only
            # go up from where it was put. A base that has settled *below* its own spawn
            # height left the surface it was placed on -- the waypoint was a point on the
            # route rather than a place to stand. Waypoint 23 does this: spawned at 0.679
            # it settles at 0.471, then spends the run wedged 0.37 m from a wall face
            # reversing at -0.4 m/s without moving. Counting that as the navigation stack
            # failing the segment would be measuring the harness, not the robot.
            self._stop("invalid spawn: never stood up at the start pose", invalid=True)
        elif settled and tilt > FALL_TILT_DEG:
            self._stop(f"fell over (tilt {tilt:.0f} deg)")
        elif settled and position[2] < self._start_z - FALL_BELOW_M:
            self._stop("fell off the course")
        elif t >= self.max_time:
            self._stop(f"ran out of time after {self.max_time:.0f} s")

    def _stop(self, outcome: str, *, reached: bool = False, invalid: bool = False) -> None:
        self._finished = True
        self._outcome = outcome
        self._invalid = invalid
        self.write()
        self.get_logger().info(f"[segment] {self.run_name}: {outcome}")
        # Exiting is the signal: the launch file turns this process's exit into a shutdown
        # of the whole stack, so the run ends without anything having to be killed by hand.
        raise SystemExit(0 if reached else 1)

    def summary(self) -> dict:
        modes: dict[str, float] = {}
        for row in self._rows:
            modes[row["mode"]] = modes.get(row["mode"], 0.0) + self._dt
        last = self._rows[-1] if self._rows else {}
        return {
            "start": self.start,
            "end": self.end,
            "seed": self.seed,
            "reached": self._outcome == "reached the end waypoint",
            # A run that never got a legal start pose is not evidence either way, and is
            # kept separate from a pass and from a failure rather than folded into either.
            "valid": not self._invalid,
            "outcome": self._outcome,
            "elapsed_s": round(last.get("t", 0.0), 2),
            "distance_m": round(self._travelled, 2),
            "goal_distance_m": round(last.get("goal_distance", float("nan")), 3),
            "max_tilt_deg": round(self._max_tilt, 1),
            "stalls": self._stalls,
            "final_xyz": [last.get("x", 0.0), last.get("y", 0.0), last.get("z", 0.0)],
            "mode_seconds": {k: round(v, 2) for k, v in sorted(modes.items())},
            "sources": sorted({row["source"] for row in self._rows}),
            "transitions": self._transitions,
            "router_status": self._status,
            "csv": str(self.out_dir / f"{self.run_name}.csv"),
        }

    def write(self) -> None:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        if self._rows:
            path = self.out_dir / f"{self.run_name}.csv"
            with path.open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(self._rows[0]))
                writer.writeheader()
                writer.writerows(self._rows)
        (self.out_dir / f"{self.run_name}.json").write_text(json.dumps(self.summary(), indent=2))


def main() -> None:
    rclpy.init()
    node = SegmentRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # A run cut short from outside still has data worth keeping, and a partial CSV with
        # an honest "interrupted" outcome beats no file at all.
        node.write()
    except SystemExit:
        raise
    finally:
        node.destroy_node()
        # Shutting down twice raises; the launch may already have done it for us.
        with contextlib.suppress(RuntimeError):
            rclpy.shutdown()


if __name__ == "__main__":
    main()
