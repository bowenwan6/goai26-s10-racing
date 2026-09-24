"""Reuse the existing pursuit/avoidance cores through a read-only hardware boundary.

This is a conservative FLAT-ground shadow profile, NOT the complete racing follower
or a stair actor. It has no ROS import, publisher, socket or enable-motion switch.
"""

from __future__ import annotations

import math

import numpy as np

from real_transfer.geometry import (
    base_pose,
    height_grid,
    points_in_yaw_frame,
    rigid_transform,
    vector,
    yaw_of,
)
from real_transfer.guard import ShadowGuard
from s10_auto_nav.local_planner import AvoidanceConfig, LocalPlanner
from s10_auto_nav.pure_pursuit import PurePursuitController, PursuitGains

VERIFICATIONS = (
    "odom_frame_and_origin",
    "cloud_extrinsics",
    "cloud_deskew_and_self_filter",
    "scan_frame_and_no_return",
    "time_sync",
    "real_route",
)


class ShadowSession:
    def __init__(self, config, limits=None):
        if config.get("mode") != "shadow_only":
            raise ValueError("only shadow_only mode exists")
        self.config = config
        self.guard = ShadowGuard(config["map_id"], limits)
        self.controller = PurePursuitController(
            PursuitGains(
                max_forward=0.2,
                max_lateral=0.1,
                max_yaw_rate=0.3,
                lookahead=0.5,
                forward_slew=0.3,
                lateral_slew=0.2,
                yaw_slew=0.5,
            )
        )
        self.planner = LocalPlanner(AvoidanceConfig())
        self.route = config.get("route", [])
        self.cursor = 0
        self.last_tick = None
        for wp in self.route:
            vector(wp["position"], 3)
            if wp.get("kind") not in {"flat", "ramp", "stairs", "unknown"}:
                raise ValueError("explicit route kind required")
            for key in ("radius_xy", "tolerance_z"):
                if not math.isfinite(wp[key]) or not 0 < wp[key] <= 0.5:
                    raise ValueError("explicit bounded waypoint tolerances required")

    def configuration_reasons(self):
        reasons = []
        for key in VERIFICATIONS:
            if self.config.get("verified", {}).get(key) is not True:
                reasons.append(f"unverified_{key}")
        if not self.route:
            reasons.append("no_real_route")
        return reasons

    def step(self, snapshot):
        """Return a JSON-compatible report. `transport_command` is ALWAYS zero."""
        if not isinstance(snapshot, dict) or not isinstance(snapshot.get("inputs"), dict):
            raise ValueError("snapshot and inputs must be objects")
        if any(not isinstance(value, dict) for value in snapshot["inputs"].values()):
            raise ValueError("input records must be objects")
        output = {
            "mode": "shadow_only",
            "motion_enabled": False,
            "transport_command": [0.0, 0.0, 0.0],
            "candidate": [0.0, 0.0, 0.0],
            "candidate_computed": False,
            "cursor": self.cursor,
            "reasons": [],
        }
        reasons = self.configuration_reasons()
        wall, mono = snapshot["wall_time"], snapshot["monotonic_time"]
        inputs = snapshot.get("inputs", {})
        pose, cloud, scan = (inputs.get(k) for k in ("pose", "cloud", "scan"))
        mb, grid, mask, ranges, angles = None, None, None, None, None
        try:
            frames = self.config["frames"]
            if pose:
                if pose["frame"] != frames["map"] or pose["child_frame"] != frames["odom_child"]:
                    raise ValueError("pose_frame_mismatch")
                mb = base_pose(pose, self.config["odom_child_from_base"])
            if cloud and mb is not None:
                if cloud["frame"] != frames["cloud"]:
                    raise ValueError("cloud_frame_mismatch")
                local = points_in_yaw_frame(cloud["points"], self.config["base_from_cloud"], mb)
                grid, mask, counts = height_grid(local)
                output["heightmap"] = {
                    "shape": [13, 9],
                    "values": grid.ravel().tolist(),
                    "valid": mask.ravel().tolist(),
                    "counts": counts.ravel().tolist(),
                    "valid_fraction": float(mask.mean()),
                    "convention": "clip(terrain_z-base_z,-1,1),x-major,yaw-aligned",
                }
                if not mask.all():
                    reasons.append("heightmap_unknown_or_ambiguous")
                elif float(np.ptp(grid)) > 0.08:
                    reasons.append("nonflat_terrain_not_admitted")
            if scan:
                if scan["frame"] != frames["scan"]:
                    raise ValueError("scan_frame_mismatch")
                # The observer requires a scan already expressed at the base origin
                # with x forward / y left. A sensor-centred scan cannot just be relabelled.
                if frames["scan"] != frames["base"]:
                    raise ValueError("scan_not_base_aligned")
                ranges = np.asarray(scan["ranges"], float)
                angles = np.asarray(scan["angles"], float)
                if "no_return" in scan:
                    no_return = np.asarray(scan["no_return"])
                    if no_return.dtype != bool or no_return.shape != ranges.shape:
                        raise ValueError("invalid_no_return_mask")
                    if np.any(no_return & ~np.isnan(ranges)):
                        raise ValueError("inconsistent_no_return_mask")
                    ranges = np.where(no_return, np.inf, ranges)
                if (
                    ranges.ndim != 1
                    or ranges.shape != angles.shape
                    or len(ranges) < 63
                    or len(ranges) > 4096
                    or not np.isfinite(angles).all()
                    or not np.allclose(np.diff(angles), angles[1] - angles[0], atol=1e-6)
                    or not 0 < angles[1] - angles[0] <= 0.1
                    or not math.isclose(
                        angles[-1] - angles[0] + angles[1] - angles[0], 2 * math.pi, abs_tol=0.02
                    )
                ):
                    raise ValueError("scan_requires_regular_full_circle")
                range_min, range_max = scan["range_min"], scan["range_max"]
                if (
                    not math.isfinite(range_min)
                    or not math.isfinite(range_max)
                    or not 0 < range_min < range_max
                    or range_max < 4
                ):
                    raise ValueError("invalid_scan_limits")
                if (
                    np.isnan(ranges).any()
                    or np.isneginf(ranges).any()
                    or np.any(ranges < range_min)
                    or np.any(np.isfinite(ranges) & (ranges > range_max))
                ):
                    raise ValueError("invalid_or_unknown_scan")
        except (ValueError, TypeError, KeyError, IndexError) as exc:
            reasons.append(f"invalid_input:{exc}")
        position = None if mb is None else mb[:3, 3]
        yaw = None if mb is None else yaw_of(mb)
        reasons += self.guard.evaluate(wall, mono, inputs, position, yaw)
        if mb is not None:
            tilt = math.acos(float(np.clip(mb[2, 2], -1, 1)))
            if tilt > math.radians(10):
                reasons.append("tilt_exceeds_flat_shadow_profile")
        dt = 0.02 if self.last_tick is None else mono - self.last_tick
        self.last_tick = mono
        if not math.isfinite(dt) or not 0 < dt <= 0.1:
            reasons.append("control_tick_gap")
        if not reasons and self.cursor < len(self.route):
            wp = self.route[self.cursor]
            target = vector(wp["position"], 3)
            if wp["kind"] != "flat":
                reasons.append("segment_requires_separate_validation")
            elif abs(target[2] - position[2]) > wp["tolerance_z"]:
                reasons.append("target_level_mismatch")
            elif np.linalg.norm(target[:2] - position[:2]) <= wp["radius_xy"]:
                self.cursor += 1
                reasons.append("waypoint_reached_hold_one_tick")
            else:
                delta = target[:2] - position[:2]
                distance = float(np.linalg.norm(delta))
                bearing = math.atan2(delta[1], delta[0]) - yaw
                # Do not ignore obstacles just beyond a waypoint in this profile.
                steer = self.planner.plan(ranges, angles, bearing)
                if steer.blocked:
                    reasons.append("obstacle_blocks_path")
                else:
                    heading = yaw + steer.heading
                    carrot = position[:2] + min(0.5, distance) * np.array(
                        [
                            math.cos(heading),
                            math.sin(heading),
                        ]
                    )
                    command = self.controller.compute(position[:2], yaw, carrot, dt)
                    candidate = np.array(command.as_tuple(), dtype=float)
                    candidate[:2] *= steer.speed_scale
                    # Bound planar norm as well as axes. These are offline test
                    # settings, not calibrated limits for the physical robot.
                    speed = np.linalg.norm(candidate[:2])
                    if speed > 0.2:
                        candidate[:2] *= 0.2 / speed
                    if not np.isfinite(candidate).all():
                        reasons.append("nonfinite_command")
                    else:
                        output["candidate"] = candidate.tolist()
                        output["candidate_computed"] = True
        elif not reasons:
            reasons.append("route_complete")
        if reasons:
            self.controller.reset()
        # Malformed geometry/config/route/tilt failures also latch once a session
        # has admitted a calculation. Merely restoring packets never resumes it.
        normal_holds = {"waypoint_reached_hold_one_tick", "route_complete"}
        if reasons and output.get("candidate_computed"):
            raise AssertionError("blocked calculation produced candidate")
        if reasons and getattr(self, "admitted", False) and not set(reasons) <= normal_holds:
            self.guard.faults.add("session_requires_restart")
        self.admitted = getattr(self, "admitted", False) or output["candidate_computed"]
        output["cursor"] = self.cursor
        output["reasons"] = sorted(set(reasons))
        return output


def validate_transforms(config):
    """Optional preflight: refuse placeholders before subscribing to sensors."""
    for key in ("odom_child_from_base", "base_from_cloud"):
        rigid_transform(config[key])
