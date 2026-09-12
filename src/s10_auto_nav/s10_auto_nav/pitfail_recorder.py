"""Record the production navigation stack's arrival state at a measured obstacle."""

from __future__ import annotations

import contextlib
import csv
import hashlib
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from drdds.msg import JointsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String

from s10_auto_nav.pitfail_metrics import (
    EntryGate,
    EntryGateConfig,
    ObstacleFrame,
    interpolate_row,
)

EVENT_LEVELS = (1.2, 0.8, 0.6, 0.4)
FALL_TILT_DEG = 70.0
FRONT_AXLE_M = 0.228

STATE_FIELDS = [
    "t",
    "x",
    "y",
    "z",
    "qw",
    "qx",
    "qy",
    "qz",
    "yaw_deg",
    "pitch_deg",
    "roll_deg",
    "tilt_deg",
    "cmd_forward",
    "cmd_lateral",
    "cmd_yaw",
    "vx",
    "vy",
    "vz",
    "yaw_rate",
    "obstacle_distance",
    "lateral_error",
    "heading_error",
    "forward_speed",
    "lateral_speed",
    "wheel_distance_min",
    "front_axle_distance",
    "scan_ahead",
    "mode",
    "source",
    "joint_owner",
    "joint_owner_actual",
    "terrain",
    "waypoint_id",
    "target_normal_yaw",
    "climb_allowed",
]
JOINT_FIELDS = [f"q.{i}" for i in range(16)] + [f"dq.{i}" for i in range(16)]
TRACE_FIELDS = STATE_FIELDS + JOINT_FIELDS
EVENT_FIELDS = ["run_id", "obstacle_id", "experiment", "seed", "event", *TRACE_FIELDS]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _angles(q) -> tuple[float, float, float, float]:
    x, y, z, w = q.x, q.y, q.z, q.w
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    up_z = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    return yaw, pitch, roll, tilt


def _load_obstacle(path: Path, obstacle_id: str) -> tuple[dict, ObstacleFrame]:
    document = yaml.safe_load(path.read_text())
    matches = [o for o in document.get("obstacles", []) if str(o.get("id")) == obstacle_id]
    if len(matches) != 1:
        raise RuntimeError(f"expected one obstacle {obstacle_id!r} in {path}, found {len(matches)}")
    return matches[0], ObstacleFrame.from_config(matches[0])


class PitfailRecorder(Node):
    def __init__(self) -> None:
        super().__init__("pitfail_recorder")
        for name, default in (
            ("course_file", ""),
            ("out_root", "/res/pitfail"),
            ("run_id", "pitfail"),
            ("obstacle_id", "gate16"),
            ("obstacles_file", "/res/pitfail/config/obstacles.yaml"),
            ("experiment", "A"),
            ("label", ""),
            ("seed", 0),
            ("max_time", 180.0),
            ("grace", 3.0),
            ("record_rate", 50.0),
            ("reach_radius", 0.18),
            ("stop_before_collision_distance", 0.15),
            ("ready_distance_min", 0.45),
            ("ready_distance_max", 0.70),
            ("max_entry_yaw_rate", 0.10),
        ):
            self.declare_parameter(name, default)

        self.run_id = str(self.get_parameter("run_id").value)
        self.obstacle_id = str(self.get_parameter("obstacle_id").value)
        self.experiment = str(self.get_parameter("experiment").value).upper()
        self.label = str(self.get_parameter("label").value)
        self.seed = int(self.get_parameter("seed").value)
        self.max_time = float(self.get_parameter("max_time").value)
        self.grace = float(self.get_parameter("grace").value)
        self.stop_distance = float(self.get_parameter("stop_before_collision_distance").value)
        self.advance_radius = float(self.get_parameter("reach_radius").value)
        self.obstacles_path = Path(str(self.get_parameter("obstacles_file").value))
        self.obstacle, self.frame = _load_obstacle(self.obstacles_path, self.obstacle_id)
        self.target_waypoint = int(self.obstacle["segment"][1])
        root = Path(str(self.get_parameter("out_root").value))
        self.run_dir = root / "raw" / self.obstacle_id / self.run_id
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.obstacles_sha = _sha256(self.obstacles_path)

        gate_config = EntryGateConfig(
            distance_min=float(self.get_parameter("ready_distance_min").value),
            distance_max=float(self.get_parameter("ready_distance_max").value),
            max_yaw_rate=float(self.get_parameter("max_entry_yaw_rate").value),
        )
        self.gate = EntryGate(gate_config)
        self._gate_config = gate_config

        self._odom = None
        self._cmd = Twist()
        self._scan_ahead = math.nan
        self._heightmap = np.full((13, 9), math.nan, dtype=np.float32)
        self._q = np.full(16, math.nan)
        self._dq = np.full(16, math.nan)
        self._mode = "-"
        self._source = "-"
        self._terrain = "-"
        self._joint_owner = "unknown"
        self._joint_owner_actual = "unknown"
        self._nav_finished = False
        self._transitions: list[str] = []

        self._t0: float | None = None
        self._start_z: float | None = None
        self._finished = False
        self._outcome = "interrupted"
        self._failure_class = ""
        self._valid = True
        self._previous: dict | None = None
        self._seen_events: set[str] = set()
        self._rows: list[dict] = []
        self._heightmaps: list[np.ndarray] = []
        self._entry: dict | None = None

        self._trace_handle = (self.run_dir / "trace.csv").open("w", newline="")
        self._trace_writer = csv.DictWriter(self._trace_handle, fieldnames=TRACE_FIELDS)
        self._trace_writer.writeheader()
        self._events_handle = (self.run_dir / "events.csv").open("w", newline="")
        self._events_writer = csv.DictWriter(self._events_handle, fieldnames=EVENT_FIELDS)
        self._events_writer.writeheader()

        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_odom, 50)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Float32MultiArray, "/perception/heightmap", self._on_heightmap, 10)
        self.create_subscription(JointsData, "/JOINTS_DATA", self._on_joints, 50)
        self.create_subscription(Bool, "/nav/finished", self._on_finished, latched)
        self.create_subscription(String, "/strategy/mode", self._on_mode, 10)
        self.create_subscription(String, "/strategy/source", self._on_source, 10)
        self.create_subscription(String, "/strategy/status", self._on_status, 10)
        self.create_subscription(String, "/strategy/transition", self._on_transition, 20)
        self.create_subscription(String, "/nav/terrain", self._on_terrain, 10)

        rate = float(self.get_parameter("record_rate").value)
        self._dt = 1.0 / rate
        self.timer = self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f"[pitfail] {self.experiment}/{self.run_id}: measuring {self.obstacle_id} "
            f"at {rate:.0f} Hz into {self.run_dir}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd = msg

    def _on_finished(self, msg: Bool) -> None:
        self._nav_finished = bool(msg.data)

    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data

    def _on_source(self, msg: String) -> None:
        self._source = msg.data

    def _on_terrain(self, msg: String) -> None:
        self._terrain = msg.data

    def _on_transition(self, msg: String) -> None:
        self._transitions.append(msg.data)

    def _on_status(self, msg: String) -> None:
        with contextlib.suppress(json.JSONDecodeError):
            status = json.loads(msg.data)
            self._joint_owner = str(status.get("joint_owner", self._joint_owner))
            self._joint_owner_actual = str(
                status.get("joint_owner_actual", self._joint_owner_actual)
            )

    def _on_heightmap(self, msg: Float32MultiArray) -> None:
        data = np.asarray(msg.data, dtype=np.float32)
        shape = tuple(int(dim.size) for dim in msg.layout.dim)
        if shape and int(np.prod(shape)) == data.size:
            self._heightmap = data.reshape(shape).copy()
        elif data.size == 117:
            self._heightmap = data.reshape(13, 9).copy()

    def _on_joints(self, msg: JointsData) -> None:
        joints = msg.data.joints_data
        if len(joints) == 16:
            self._q = np.asarray([joint.position for joint in joints], dtype=float)
            self._dq = np.asarray([joint.velocity for joint in joints], dtype=float)

    def _on_scan(self, msg: LaserScan) -> None:
        values = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + msg.angle_increment * np.arange(values.size)
        mask = (np.abs(angles) < math.radians(20.0)) & np.isfinite(values)
        mask &= values < msg.range_max - 1e-3
        self._scan_ahead = float(np.min(values[mask])) if np.any(mask) else math.nan

    def _row(self, t: float) -> dict:
        pose = self._odom.pose.pose
        twist = self._odom.twist.twist
        yaw, pitch, roll, tilt = _angles(pose.orientation)
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        velocity = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        measured = self.frame.measure(position[:2], yaw, velocity[:2])
        heading = np.array([math.cos(yaw), math.sin(yaw)])
        front_xy = position[:2] + FRONT_AXLE_M * heading
        front_distance = float(np.dot(self.frame.edge_center[:2] - front_xy, self.frame.normal))
        row = {
            "t": t,
            "x": float(position[0]),
            "y": float(position[1]),
            "z": float(position[2]),
            "qw": float(pose.orientation.w),
            "qx": float(pose.orientation.x),
            "qy": float(pose.orientation.y),
            "qz": float(pose.orientation.z),
            "yaw_deg": math.degrees(yaw),
            "pitch_deg": math.degrees(pitch),
            "roll_deg": math.degrees(roll),
            "tilt_deg": tilt,
            "cmd_forward": float(self._cmd.linear.x),
            "cmd_lateral": float(self._cmd.linear.y),
            "cmd_yaw": float(self._cmd.angular.z),
            "vx": float(velocity[0]),
            "vy": float(velocity[1]),
            "vz": float(velocity[2]),
            "yaw_rate": float(twist.angular.z),
            **measured,
            "wheel_distance_min": math.nan,
            "front_axle_distance": front_distance,
            "scan_ahead": self._scan_ahead,
            "mode": self._mode,
            "source": self._source,
            "joint_owner": self._joint_owner,
            "joint_owner_actual": self._joint_owner_actual,
            "terrain": self._terrain,
            "waypoint_id": self.target_waypoint,
            "target_normal_yaw": self.frame.normal_yaw,
            "climb_allowed": 0,
        }
        row.update({f"q.{i}": float(v) for i, v in enumerate(self._q)})
        row.update({f"dq.{i}": float(v) for i, v in enumerate(self._dq)})
        allowed, _ = self.gate.update(t, row)
        row["climb_allowed"] = int(allowed)
        return row

    def _tick(self) -> None:
        if self._finished or self._odom is None:
            return
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0
        row = self._row(t)
        self._trace_writer.writerow(row)
        self._rows.append(dict(row))
        self._heightmaps.append(self._heightmap.copy())
        if len(self._rows) % 50 == 0:
            self._trace_handle.flush()

        if self._previous is not None:
            for level in EVENT_LEVELS:
                event = f"d{level:g}"
                if event in self._seen_events:
                    continue
                if (
                    float(self._previous["obstacle_distance"])
                    >= level
                    > float(row["obstacle_distance"])
                ):
                    self._write_event(event, interpolate_row(self._previous, row, level))
        if row["climb_allowed"] and "climb_ready" not in self._seen_events:
            self._write_event("climb_ready", row)
            self._entry = self._entry_payload(row)
            (self.run_dir / "entry.json").write_text(
                json.dumps(self._entry, indent=2, allow_nan=True) + "\n"
            )
            if self.experiment == "B":
                self._stop("entry gate reached; no climb attempted")
                return
        self._previous = dict(row)

        if self._start_z is None:
            self._start_z = float(row["z"])
        settled = t >= self.grace
        if settled and float(row["tilt_deg"]) > FALL_TILT_DEG:
            self._stop("fell before arrival", failure_class="fall")
        elif settled and float(row["front_axle_distance"]) <= self.stop_distance:
            self._stop("arrival measured at front-axle stop line")
        elif self._nav_finished:
            self._stop("navigation finished before obstacle stop line")
        elif t >= self.max_time:
            self._stop("timed out before arrival", failure_class="timeout")

    def _write_event(self, event: str, row: dict) -> None:
        record = {
            "run_id": self.run_id,
            "obstacle_id": self.obstacle_id,
            "experiment": self.experiment,
            "seed": self.seed,
            "event": event,
            **row,
        }
        self._events_writer.writerow(record)
        self._events_handle.flush()
        self._seen_events.add(event)

    def _entry_payload(self, row: dict) -> dict:
        return {
            "run_id": self.run_id,
            "obstacle_id": self.obstacle_id,
            "experiment": self.experiment,
            "seed": self.seed,
            "obstacles_sha256": self.obstacles_sha,
            "state": row,
            "joint_position": self._q.tolist(),
            "joint_velocity": self._dq.tolist(),
            "heightmap_13x9": self._heightmap.tolist(),
            "gate": self._gate_config.__dict__,
        }

    def _stop(
        self,
        outcome: str,
        *,
        valid: bool = True,
        failure_class: str = "",
    ) -> None:
        if self._finished:
            return
        self._finished = True
        self._outcome = outcome
        self._valid = valid
        self._failure_class = failure_class
        self.write()
        self.get_logger().info(f"[pitfail] {self.run_id}: {outcome}")
        raise SystemExit(0 if valid else 1)

    def write(self) -> None:
        if self._trace_handle and not self._trace_handle.closed:
            self._trace_handle.flush()
            self._trace_handle.close()
        if self._events_handle and not self._events_handle.closed:
            self._events_handle.flush()
            self._events_handle.close()
        if self._rows:
            arrays = {
                field.replace(".", "_"): np.asarray(
                    [row.get(field, math.nan) for row in self._rows]
                )
                for field in TRACE_FIELDS
                if field not in {"mode", "source", "joint_owner", "joint_owner_actual", "terrain"}
            }
            arrays["heightmap"] = np.asarray(self._heightmaps, dtype=np.float32)
            arrays["joint_position"] = np.asarray(
                [[row[f"q.{i}"] for i in range(16)] for row in self._rows]
            )
            arrays["joint_velocity"] = np.asarray(
                [[row[f"dq.{i}"] for i in range(16)] for row in self._rows]
            )
            np.savez_compressed(self.run_dir / "state.npz", **arrays)
        last = self._rows[-1] if self._rows else {}
        result = {
            "run_id": self.run_id,
            "obstacle_id": self.obstacle_id,
            "experiment": self.experiment,
            "label": self.label,
            "seed": self.seed,
            "advance_radius_m": self.advance_radius,
            "valid": self._valid,
            "outcome": self._outcome,
            "failure_class": self._failure_class,
            "elapsed_s": float(last.get("t", 0.0)),
            "events": sorted(self._seen_events),
            "climb_ready": "climb_ready" in self._seen_events,
            "final_obstacle_distance_m": last.get("obstacle_distance"),
            "final_front_axle_distance_m": last.get("front_axle_distance"),
            "stop_reference": "front_axle_distance",
            "obstacles_sha256": self.obstacles_sha,
            "nan_fields": ["wheel_distance_min"],
            "heightmap_shape": [13, 9],
            "transitions": self._transitions,
        }
        (self.run_dir / "result.json").write_text(
            json.dumps(result, indent=2, allow_nan=True) + "\n"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PitfailRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.write()
    except SystemExit:
        raise
    finally:
        node.destroy_node()
        with contextlib.suppress(RuntimeError):
            rclpy.shutdown()


if __name__ == "__main__":
    main()
