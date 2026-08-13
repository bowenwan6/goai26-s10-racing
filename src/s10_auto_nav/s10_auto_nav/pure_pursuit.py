"""Pure-pursuit steering with heading-aware speed scheduling.

The locomotion policy consumes a velocity command, so navigation reduces to producing
``(forward, lateral, yaw_rate)`` from the robot pose and the course. Two behaviours matter
for lap time:

* **Speed is scheduled on heading error.** Commanding full forward speed while badly
  misaligned wastes distance and risks tipping on the elevated sections, so forward speed
  is scaled down as alignment degrades and the robot turns in place past a threshold.
* **Lateral velocity is used, not ignored.** The S10 is a wheel-legged holonomic-ish
  platform; small sideways corrections hold the racing line without scrubbing yaw.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class PursuitGains:
    """Tuning surface for the follower.

    The defaults match the shipped policy's trained envelope, which is the ceiling the
    upstream keyboard interface enforces. Exceeding it does not trade safety for lap
    time, it simply stops working: the policy ignores commands it never saw in training.
    See ``s10_bringup/config/nav.yaml``.
    """

    max_forward: float = 0.7
    max_lateral: float = 0.4
    max_yaw_rate: float = 0.7

    #: Carrot distance along the course. Larger is smoother and faster but cuts corners
    #: harder, which can miss a waypoint's 0.2 m scoring radius.
    lookahead: float = 1.4
    #: Lookahead grows with speed so fast sections steer gently.
    lookahead_speed_gain: float = 0.5

    yaw_gain: float = 1.8
    lateral_gain: float = 0.9

    #: Beyond this heading error the robot turns in place instead of driving forward.
    pivot_threshold: float = math.radians(75.0)
    #: Heading error at which forward speed has decayed to its floor.
    align_falloff: float = math.radians(60.0)
    #: Fraction of max_forward retained when badly misaligned.
    min_speed_fraction: float = 0.15

    #: Slew limits, applied per control step, to keep commands smooth for the policy.
    forward_slew: float = 3.0
    yaw_slew: float = 6.0


@dataclass
class Command:
    forward: float = 0.0
    lateral: float = 0.0
    yaw_rate: float = 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        return self.forward, self.lateral, self.yaw_rate


def wrap_angle(angle: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class PurePursuitController:
    """Stateless-in-principle controller; holds only the slew-rate memory."""

    def __init__(self, gains: PursuitGains | None = None) -> None:
        self.gains = gains or PursuitGains()
        self._last = Command()

    def reset(self) -> None:
        self._last = Command()

    @property
    def last_command(self) -> Command:
        """Most recently emitted command, for watchdogs and logging."""
        return self._last

    def lookahead_distance(self) -> float:
        g = self.gains
        return g.lookahead + g.lookahead_speed_gain * abs(self._last.forward)

    def compute(
        self,
        position_xy: np.ndarray,
        yaw: float,
        target_xy: np.ndarray,
        dt: float,
    ) -> Command:
        g = self.gains

        delta = np.asarray(target_xy, float) - np.asarray(position_xy, float)
        distance = float(np.linalg.norm(delta))
        if distance < 1e-6:
            return self._slew(Command(), dt)

        heading_error = wrap_angle(math.atan2(delta[1], delta[0]) - yaw)
        abs_error = abs(heading_error)

        yaw_rate = float(np.clip(g.yaw_gain * heading_error, -g.max_yaw_rate, g.max_yaw_rate))

        if abs_error > g.pivot_threshold:
            # Too far off course to make progress; rotate on the spot.
            return self._slew(Command(forward=0.0, lateral=0.0, yaw_rate=yaw_rate), dt)

        # Cosine-shaped falloff: full speed when aligned, floored when near the pivot limit.
        alignment = max(0.0, 1.0 - abs_error / g.align_falloff)
        speed_scale = g.min_speed_fraction + (1.0 - g.min_speed_fraction) * alignment
        forward = g.max_forward * speed_scale

        # Cross-track offset in the body frame, corrected laterally rather than by yawing.
        cross_track = -math.sin(yaw) * delta[0] + math.cos(yaw) * delta[1]
        lateral = float(np.clip(g.lateral_gain * cross_track, -g.max_lateral, g.max_lateral))

        # Brake into the final approach so the last waypoint is not overshot.
        forward = min(forward, g.max_forward * min(1.0, distance / max(g.lookahead, 1e-6)))

        return self._slew(Command(forward=forward, lateral=lateral, yaw_rate=yaw_rate), dt)

    def _slew(self, target: Command, dt: float) -> Command:
        g = self.gains
        forward = _rate_limit(self._last.forward, target.forward, g.forward_slew * dt)
        yaw_rate = _rate_limit(self._last.yaw_rate, target.yaw_rate, g.yaw_slew * dt)
        self._last = Command(forward=forward, lateral=target.lateral, yaw_rate=yaw_rate)
        return self._last


def _rate_limit(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target
