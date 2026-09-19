"""Native gait acknowledgement and command ownership around the existing Router.

No ROS, model loading or actuator access. A decision alone never enables motion.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from s10_auto_nav.strategy.router import Mode, RobotState, Router, RouterConfig

GAITS = {"flat": 0x3002, "stairs": 0x3003}
ZERO = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Feedback:
    robot: RobotState
    gait: int
    motion_state: int
    hes: int
    motion_received: float
    status_received: float
    hes_received: float
    command_received: float
    localized: bool
    perception_valid: bool
    exclusive_control: bool
    command: tuple[float, float, float] = ZERO
    external_fault: str = ""


@dataclass(frozen=True)
class Decision:
    state: str
    reason: str
    command: tuple[float, float, float] = ZERO
    request_gait: int | None = None
    expected_gait: int | None = None
    publish: bool = False


@dataclass(frozen=True)
class Limits:
    freshness: float = 0.35
    ack_dwell: float = 0.4
    switch_timeout: float = 4.0
    settle_dwell: float = 0.4
    stationary_speed: float = 0.04
    stationary_yaw_rate: float = 0.08
    max_duration: float = 600.0
    max_no_progress: float = 15.0
    max_flat: float = 0.20
    max_stairs: float = 0.15
    max_lateral: float = 0.05
    max_yaw: float = 0.20
    max_tilt: float = math.radians(35)

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError("native limits must be positive and finite")


class NativeGaitRouter:
    """Explicit arm, stop-before-switch, fresh acknowledgement, latched failure.

    Targets select the gait for their INCOMING segment. Segments must include safe
    staging/exit points; a gait switch is not inferred from instantaneous body tilt.
    The original Router retains sensor/tilt/course completion checks. Native policies
    stay in the robot; the external SDK joint-policy branch is never activated.
    """

    def __init__(
        self,
        kinds: list[str],
        limits: Limits | None = None,
        *,
        probe_only=False,
        speed_caps: list[float] | None = None,
    ):
        if not kinds or any(k not in GAITS for k in kinds):
            raise ValueError("every target must explicitly select flat or stairs")
        # Optional per-target forward cap (route_v2 segment speed_limit). It can only
        # lower the gait cap below, never raise it.
        if speed_caps is not None and (
            len(speed_caps) != len(kinds)
            or any(not math.isfinite(v) or v <= 0 for v in speed_caps)
        ):
            raise ValueError("speed_caps must give one positive finite cap per target")
        self.speed_caps = None if speed_caps is None else tuple(float(v) for v in speed_caps)
        self.kinds = tuple(kinds)
        self.limits = limits or Limits()
        self.probe_only = probe_only
        self.router = Router(RouterConfig(control_rate=10, fall_tilt=self.limits.max_tilt))
        self.state = "disarmed"
        self.reason = "explicit arm required"
        self.expected = None
        self.started = None
        self.previous_time = None
        self.switch_started = None
        self.requested_at = None
        self.ack_since = None
        self.settle_since = None
        self.last_feedback = None
        self.progress_at = None
        self.progress = 0.0
        self.ever_armed = False
        self.accepted_gaits: set[int] = set()
        self.events: list[dict] = []

    def _set(self, state, reason, now):
        if (state, reason) != (self.state, self.reason):
            self.events.append({"time": now, "state": state, "reason": reason})
        self.state, self.reason = state, reason

    def _health(self, f: Feedback):
        now = f.robot.t
        times = [f.motion_received, f.hes_received, f.robot.odom_time]
        if not self.probe_only:
            times += [
                f.status_received,
                f.command_received,
                f.robot.lidar_time,
                f.robot.heightmap_time,
            ]
        if not math.isfinite(now) or any(
            not math.isfinite(t) or t <= 0 or not 0 <= now - t <= self.limits.freshness
            for t in times
        ):
            return "missing_stale_or_future_input"
        values = [
            *f.robot.position,
            f.robot.yaw,
            f.robot.pitch,
            f.robot.roll,
            f.robot.speed,
            f.robot.yaw_rate,
            f.robot.travelled,
            *f.command,
        ]
        if len(f.command) != 3 or not all(math.isfinite(v) for v in values):
            return "nonfinite_input"
        if f.external_fault:
            return f.external_fault
        if not f.exclusive_control:
            return "control_source_conflict"
        if f.hes != 0:
            return "emergency_stop"
        if f.motion_state != 17:
            return "native_rl_not_active_or_operator_takeover"
        if not self.probe_only and not f.localized:
            return "global_localization_unavailable"
        if not self.probe_only and not f.perception_valid:
            return "perception_unverified_or_unknown"
        if f.robot.tilt >= self.limits.max_tilt:
            return "tilt_limit"
        return ""

    def arm(self, feedback: Feedback, *, admission: bool):
        # Failures require a new session, never an automatic re-arm when packets return.
        if self.state != "disarmed":
            raise ValueError("restart session after stop/fault/completion")
        reason = self._health(feedback)
        if not admission or reason:
            raise ValueError(reason or "field_admission_incomplete")
        self.ever_armed = True
        self.started = self.progress_at = feedback.robot.t
        self.progress = feedback.robot.travelled
        self._set("settling", "waiting for stationary gait handoff", feedback.robot.t)

    def cancel(self, now, reason="operator_cancel"):
        self._set("stopped", reason, now)

    def fail(self, now, reason):
        self._set("fault", reason, now)

    def tick(self, f: Feedback, target: int) -> Decision:
        now, c = f.robot.t, self.limits
        if self.state in {"disarmed", "fault", "stopped", "done"}:
            return Decision(
                self.state, self.reason, expected_gait=self.expected, publish=self.ever_armed
            )
        reason = self._health(f)
        if self.previous_time is not None and not 0 < now - self.previous_time <= c.freshness:
            reason = "control_loop_time_gap"
        self.previous_time = now
        if now - self.started > c.max_duration:
            reason = "test_duration_limit"
        if reason:
            self._set("fault", reason, now)
            return Decision(self.state, reason, publish=True)
        if f.robot.course_finished:
            self._set("done", "course complete; zero hold", now)
            return Decision(self.state, self.reason, publish=True)
        if not isinstance(target, int) or not 0 <= target < len(self.kinds):
            self._set("fault", "invalid_route_target", now)
            return Decision(self.state, self.reason, publish=True)
        required = GAITS[self.kinds[target]]
        if required != self.expected:
            self.expected = required
            self.switch_started = now
            self.requested_at = self.ack_since = self.settle_since = None
            self._set("settling", "stop before gait switch", now)
        if self.state in {"settling", "switching"}:
            if now - self.switch_started > c.switch_timeout:
                self._set("fault", "gait_handoff_timeout", now)
            elif self.state == "settling":
                stationary = (
                    abs(f.robot.speed) <= c.stationary_speed
                    and abs(f.robot.yaw_rate) <= c.stationary_yaw_rate
                )
                self.settle_since = (
                    (now if self.settle_since is None else self.settle_since)
                    if stationary
                    else None
                )
                if self.settle_since is not None and now - self.settle_since >= c.settle_dwell:
                    self.requested_at = now
                    self._set("switching", "waiting for fresh gait acknowledgement", now)
                    return Decision(
                        self.state,
                        self.reason,
                        request_gait=required,
                        expected_gait=required,
                        publish=True,
                    )
            else:
                fresh = f.motion_received > self.requested_at
                matching = f.gait == required and fresh
                if matching and f.motion_received != self.last_feedback:
                    self.ack_since = now if self.ack_since is None else self.ack_since
                    if now - self.ack_since >= c.ack_dwell:
                        self.accepted_gaits.add(required)
                        self._set("active", "native gait acknowledged", now)
                        self.progress_at = now
                elif not matching:
                    self.ack_since = None
                self.last_feedback = f.motion_received
            # The acknowledgement tick also holds zero; velocity begins next tick.
            return Decision(self.state, self.reason, expected_gait=required, publish=True)
        if f.gait != required:
            self._set("fault", "unexpected_gait_change_or_operator_takeover", now)
            return Decision(self.state, self.reason, publish=True)
        if self.probe_only:
            return Decision(
                self.state,
                "stationary acknowledgement probe; velocity disabled",
                expected_gait=required,
                publish=True,
            )
        if f.robot.travelled >= self.progress + 0.05:
            self.progress, self.progress_at = f.robot.travelled, now
        if now - self.progress_at > c.max_no_progress:
            self._set("fault", "no_route_progress", now)
            return Decision(self.state, self.reason, publish=True)
        out = self.router.tick(f.robot, f.command)
        if out.mode in {Mode.ABORT, Mode.DONE}:
            self._set("done" if out.mode is Mode.DONE else "fault", out.reason, now)
            return Decision(self.state, self.reason, publish=True)
        vx, vy, yaw = out.command
        # No autonomous backing away on these stairs; require operator review.
        if vx < -0.01:
            self._set("fault", "follower_requested_reverse", now)
            return Decision(self.state, self.reason, publish=True)
        cap = c.max_stairs if required == GAITS["stairs"] else c.max_flat
        if self.speed_caps is not None:
            cap = min(cap, self.speed_caps[target])
        command = (
            min(cap, max(0.0, vx)),
            max(-c.max_lateral, min(c.max_lateral, vy)),
            max(-c.max_yaw, min(c.max_yaw, yaw)),
        )
        return Decision(self.state, self.reason, command, expected_gait=required, publish=True)
