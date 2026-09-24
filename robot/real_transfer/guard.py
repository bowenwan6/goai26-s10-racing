"""Fail-closed admission for SHADOW calculations, not a certified safety controller.

Both acquisition (wall/ROS) and local arrival (monotonic) clocks are checked.
A repeated old source stamp cannot be made fresh by a recent DDS callback.
After a fault, construct a new session explicitly; there is no automatic re-arm.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GuardLimits:
    sensor_age_s: float = 0.25
    status_age_s: float = 2.0
    future_s: float = 0.02
    pose_cloud_skew_s: float = 0.05
    clock_jump_s: float = 0.1
    warmup_s: float = 1.0
    pose_jump_margin_m: float = 0.10
    max_pose_speed_mps: float = 1.0
    max_pose_yaw_rate: float = 1.0

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in self.__dict__.values()):
            raise ValueError("guard limits must be finite and positive")


class ShadowGuard:
    def __init__(self, map_id, limits=None):
        if not isinstance(map_id, str) or not map_id:
            raise ValueError("map identity required")
        self.map_id = map_id
        self.limits = limits or GuardLimits()
        self.previous = {}
        self.clock = None
        self.session = None
        self.good_since = None
        self.faults = set()

    def evaluate(self, wall, mono, inputs, position=None, yaw=None):
        reasons = []
        lim = self.limits
        if not math.isfinite(wall) or not math.isfinite(mono):
            self.faults.add("invalid_clock")
            return sorted(self.faults)
        if self.clock is not None:
            dw, dm = wall - self.clock[0], mono - self.clock[1]
            if dm < 0 or abs(dw - dm) > lim.clock_jump_s:
                self.faults.add("clock_jump")
        self.clock = (wall, mono)
        for name in ("pose", "cloud", "scan", "localization"):
            data = inputs.get(name)
            if data is None:
                reasons.append(f"missing_{name}")
                continue
            stamp, received = data.get("stamp"), data.get("received")
            if (
                not isinstance(stamp, (int, float))
                or not math.isfinite(stamp)
                or stamp <= 0
                or not isinstance(received, (int, float))
                or not math.isfinite(received)
            ):
                self.faults.add(f"invalid_time_{name}")
                continue
            age_limit = lim.status_age_s if name == "localization" else lim.sensor_age_s
            if not -lim.future_s <= wall - stamp <= age_limit:
                reasons.append(f"source_age_{name}")
            if not 0 <= mono - received <= age_limit:
                reasons.append(f"arrival_age_{name}")
            old = self.previous.get(name)
            if old and (
                stamp < old[0] or received < old[1] or (received > old[1] and stamp <= old[0])
            ):
                self.faults.add(f"replayed_or_reordered_{name}")
            if (
                name == "pose"
                and old
                and stamp > old[0]
                and position is not None
                and old[2] is not None
            ):
                dt = stamp - old[0]
                if (
                    np.linalg.norm(np.asarray(position) - old[2])
                    > lim.pose_jump_margin_m + lim.max_pose_speed_mps * dt
                ):
                    self.faults.add("pose_jump")
                if yaw is not None and old[3] is not None:
                    delta = math.atan2(math.sin(yaw - old[3]), math.cos(yaw - old[3]))
                    if abs(delta) > 0.1 + lim.max_pose_yaw_rate * dt:
                        self.faults.add("yaw_jump")
            self.previous[name] = (stamp, received, position, yaw)
        loc = inputs.get("localization") or {}
        if loc.get("map_id") != self.map_id:
            reasons.append("map_mismatch")
        if (
            type(loc.get("code")) is not int
            or loc.get("code") != 0
            or loc.get("global") is not True
        ):
            reasons.append("global_localization_not_valid")
        session = loc.get("session_id")
        if not session:
            reasons.append("missing_localization_session")
        elif self.session is None:
            self.session = (loc.get("map_id"), session)
        elif self.session != (loc.get("map_id"), session):
            self.faults.add("localization_session_changed")
        pose, cloud = inputs.get("pose"), inputs.get("cloud")
        if (
            pose
            and cloud
            and all(isinstance(x.get("stamp"), (int, float)) for x in (pose, cloud))
            and abs(pose["stamp"] - cloud["stamp"]) > lim.pose_cloud_skew_s
        ):
            reasons.append("pose_cloud_not_synchronized")
        reasons += sorted(self.faults)
        # Once admitted, any fault requires an explicit session restart.
        if reasons:
            if self.good_since is not None and mono - self.good_since >= lim.warmup_s:
                self.faults.add("session_requires_restart")
            self.good_since = None
        elif self.good_since is None:
            self.good_since = mono
        if not reasons and mono - self.good_since < lim.warmup_s:
            reasons.append("warming_up")
        return sorted(set(reasons))
