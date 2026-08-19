"""Lifecycle adapter for the v1.5 confidence-fallback SDK-local Gate 16 policy.

The ONNX graphs and actuator decoding live in ``rl_deploy`` so they consume the calibrated
``RobotBasicState`` and produce wheel velocity targets without a ROS actuator round trip.
This adapter deliberately contains no inference. It gives the router the ordinary policy
lifecycle while carrying the command that the low-level actor must observe.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from s10_auto_nav.strategy.policy import (
    ActionKind,
    PolicyAction,
    PolicyObservation,
    PolicyResult,
    PolicyStatus,
)

GATE16_ARMED_MODES = frozenset({"climb_ready", "climb", "verify_clear"})


def gate16_should_own(policy, mode: str, *, prewarm_ready: bool = False) -> bool:
    """Own only after the router proves the complete moving-entry envelope."""
    return bool(getattr(policy, "is_gate16_policy", False) and str(mode) in GATE16_ARMED_MODES)


def gate16_owner_request(
    policy,
    mode: str,
    *,
    prewarm_ready: bool = False,
    entry_mode: str | None = None,
) -> str:
    """Return the two-phase SDK request for this router mode.

    Shadow inference keeps the ONNX sessions warm, but its actions are never treated as
    physical history. ``gate16_climb`` takes ownership and arms residual together only after
    the router proves the moving-entry contract.
    """
    if (
        getattr(policy, "is_gate16_policy", False)
        and str(mode)
        in {
            "approach",
            "align",
        }
        and not prewarm_ready
    ):
        # Build a warm candidate without taking actuator ownership before staging.
        return "gate16_shadow"
    if not gate16_should_own(policy, mode, prewarm_ready=prewarm_ready):
        return "official"
    if entry_mode == "stable_fallback":
        return "gate16_climb_fallback"
    return "gate16_climb"


@dataclass
class Gate16Config:
    command_forward: float = 0.25
    fallback_command_forward: float | None = None
    command_lateral: float = 0.0
    command_yaw_rate: float = 0.0
    profile_file: str = ""
    obstacle_edge: tuple[float, float] = (12.64593, 32.49969)
    obstacle_normal: tuple[float, float] = (1.0, 0.0)
    deck_z: float = 0.4787248
    wheel_radius: float = 0.081
    front_clearance: float = 0.02
    front_height_fraction: float = 0.70
    settle_policy_steps: int = 30


@dataclass(frozen=True)
class _CommandProfile:
    name: str
    entry_speed_mps: float
    entry_yaw_deg: float
    speed_tolerance_mps: float
    yaw_tolerance_deg: float
    settle_forward_mps: float
    push_forward_mps: float


class StableGate16Policy:
    """Remote handle for the frozen-checkpoint Gate16 v1.5 actor."""

    action_kind = ActionKind.DELEGATED
    owner_name = "gate16"
    is_gate16_policy = True
    requires_moving_entry = True
    requires_physical_clear = True

    def __init__(self, config: Gate16Config | None = None):
        self.config = config or Gate16Config()
        self._profiles = self._load_profiles(self.config.profile_file)
        self.reset_count = 0
        self.reset()

    @staticmethod
    def _load_profiles(path: str) -> tuple[_CommandProfile, ...]:
        if not path:
            return ()
        root = json.loads(Path(path).read_text())
        speed_tolerance = float(root["speed_tolerance_mps"])
        yaw_tolerance = float(root["yaw_tolerance_deg"])
        if speed_tolerance <= 0.0 or yaw_tolerance <= 0.0:
            raise ValueError("Gate16 command-profile tolerances must be positive")
        profiles = []
        for item in root["profiles"]:
            profile = _CommandProfile(
                name=str(item["name"]),
                entry_speed_mps=float(item["entry_speed_mps"]),
                entry_yaw_deg=float(item["entry_yaw_deg"]),
                speed_tolerance_mps=float(item.get("speed_tolerance_mps", speed_tolerance)),
                yaw_tolerance_deg=float(item.get("yaw_tolerance_deg", yaw_tolerance)),
                settle_forward_mps=float(item["settle_command_forward_mps"]),
                push_forward_mps=float(item["push_command_forward_mps"]),
            )
            if profile.speed_tolerance_mps <= 0.0 or profile.yaw_tolerance_deg <= 0.0:
                raise ValueError("Gate16 command-profile tolerances must be positive")
            profiles.append(profile)
        return tuple(profiles)

    def _entry_yaw_deg(self, observation: PolicyObservation) -> float:
        normal = np.asarray(self.config.obstacle_normal, dtype=float)
        normal_yaw = math.atan2(float(normal[1]), float(normal[0]))
        return math.degrees(
            (float(observation.yaw) - normal_yaw + math.pi) % (2.0 * math.pi) - math.pi
        )

    def _entry_speed_mps(self, observation: PolicyObservation) -> float:
        normal = np.asarray(self.config.obstacle_normal, dtype=float)
        return float(np.dot(np.asarray(observation.linear_velocity[:2], dtype=float), normal))

    def _select_profile(self, speed_mps: float, yaw_deg: float) -> _CommandProfile | None:
        candidates: list[tuple[float, _CommandProfile]] = []
        for profile in self._profiles:
            speed_error = abs(speed_mps - profile.entry_speed_mps)
            yaw_error = abs(yaw_deg - profile.entry_yaw_deg)
            if (
                speed_error <= profile.speed_tolerance_mps
                and yaw_error <= profile.yaw_tolerance_deg
            ):
                score = (
                    speed_error / profile.speed_tolerance_mps
                    + yaw_error / profile.yaw_tolerance_deg
                )
                candidates.append((score, profile))
        return min(candidates, key=lambda item: item[0])[1] if candidates else None

    def _both_front_supported(self, observation: PolicyObservation) -> bool:
        if observation.wheel_positions is None:
            return False
        wheels = np.asarray(observation.wheel_positions, dtype=float)
        if wheels.shape != (4, 3) or not np.all(np.isfinite(wheels)):
            return False
        edge = np.asarray(self.config.obstacle_edge, dtype=float)
        normal = np.asarray(self.config.obstacle_normal, dtype=float)
        forward_margin = (wheels[:2, :2] - edge) @ normal
        height_threshold = (
            self.config.deck_z + self.config.front_height_fraction * self.config.wheel_radius
        )
        return bool(
            np.all(forward_margin >= self.config.front_clearance)
            and np.all(wheels[:2, 2] >= height_threshold)
        )

    def reset(self) -> None:
        self.reset_count += 1
        self._status = PolicyStatus.IDLE
        self._started_at: float | None = None
        self._last_t = 0.0
        self._reason = ""
        self._profile: _CommandProfile | None = None
        self._phase = "entry"
        self._settle_steps = 0
        self._entry_speed = 0.0
        self._entry_yaw_deg_value = 0.0
        self._entry_mode = "unmatched"

    def start(self, observation: PolicyObservation) -> None:
        self._started_at = float(observation.t)
        self._last_t = self._started_at
        self._status = PolicyStatus.RUNNING
        self._reason = "Gate16 v1.5 actor requested (source 216b77a)"
        self._entry_speed = self._entry_speed_mps(observation)
        self._entry_yaw_deg_value = self._entry_yaw_deg(observation)
        self._profile = self._select_profile(self._entry_speed, self._entry_yaw_deg_value)
        self._entry_mode = (
            "fast_profile"
            if self._profile is not None
            else (
                "stable_fallback"
                if self.config.fallback_command_forward is not None
                else "unmatched"
            )
        )
        print(
            "Gate16 command profile selected: "
            f"{self._profile.name if self._profile is not None else 'unmatched'} "
            f"mode={self._entry_mode} speed={self._entry_speed:.3f} "
            f"yaw={self._entry_yaw_deg_value:.3f}",
            flush=True,
        )

    def step(self, observation: PolicyObservation) -> PolicyAction:
        self._last_t = float(observation.t)
        if (
            self._profile is not None
            and self._phase == "entry"
            and self._both_front_supported(observation)
        ):
            self._phase = "settle"
            self._settle_steps = 0
            print("Gate16 command profile phase: settle", flush=True)
        reported_phase = self._phase
        if self._phase == "settle":
            command_forward = self._profile.settle_forward_mps
            self._settle_steps += 1
            if self._settle_steps >= self.config.settle_policy_steps:
                self._phase = "push"
                print("Gate16 command profile phase: push", flush=True)
        elif self._phase == "push":
            command_forward = self._profile.push_forward_mps
        elif self.config.fallback_command_forward is not None and self._profile is None:
            command_forward = self.config.fallback_command_forward
        else:
            command_forward = self.config.command_forward
        profile_name = self._profile.name if self._profile is not None else "unmatched"
        runtime = (
            "confidence_fallback_v1_5"
            if self.config.fallback_command_forward is not None
            else "adaptive_v3_b824f7f"
        )
        return PolicyAction(
            ActionKind.DELEGATED,
            self._status,
            twist=(
                float(command_forward),
                float(self.config.command_lateral),
                float(self.config.command_yaw_rate),
            ),
            info={
                "owner": self.owner_name,
                "runtime": runtime,
                "entry_mode": self._entry_mode,
                "profile": profile_name,
                "phase": reported_phase,
                "entry_speed_mps": self._entry_speed,
                "entry_yaw_deg": self._entry_yaw_deg_value,
            },
        )

    def succeed(self, reason: str = "four wheels verified on upper platform") -> None:
        self._status = PolicyStatus.SUCCEEDED
        self._reason = reason

    def fail(self, reason: str) -> None:
        self._status = PolicyStatus.FAILED
        self._reason = reason

    def cancel(self) -> None:
        if self._status is PolicyStatus.RUNNING:
            self._status = PolicyStatus.CANCELLED
            self._reason = "cancelled by router"

    def is_finished(self) -> bool:
        return self._status.terminal

    def result(self) -> PolicyResult:
        elapsed = 0.0 if self._started_at is None else self._last_t - self._started_at
        return PolicyResult(
            self._status,
            self._reason,
            elapsed,
            {
                "stable_gate16_checkpoint": True,
                "profile": (
                    "confidence_fallback_v1_5"
                    if self.config.fallback_command_forward is not None
                    else "adaptive_v3_b824f7f"
                ),
                "entry_mode": self._entry_mode,
                "command_profile": (
                    self._profile.name if self._profile is not None else "unmatched"
                ),
                "phase": self._phase,
            },
        )
