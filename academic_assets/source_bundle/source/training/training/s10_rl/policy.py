"""S10 low-level policy contract.

The network owns all sixteen actuators on every control step.  The first twelve
outputs are leg joint *position* residuals and the last four are wheel *velocity*
commands.  Keeping that distinction explicit prevents a common but fatal mistake:
treating the S10 as either a quadruped that happens to have wheels, or a four-wheel
base whose legs are held fixed.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

from s10_rl.observation import DEFAULT_JOINT_POS, POLICY_ORDER, ROBOT_ORDER

LEG_JOINTS = tuple(POLICY_ORDER[:12])
WHEEL_JOINTS = tuple(POLICY_ORDER[12:])

# Policy order: four three-DoF legs followed by four wheels.
POLICY_ACTION_SCALE = (
    0.125,
    0.25,
    0.25,
    0.125,
    0.25,
    0.25,
    0.125,
    0.25,
    0.25,
    0.125,
    0.25,
    0.25,
    5.0,
    5.0,
    5.0,
    5.0,
)

# For each robot-order joint, index of the corresponding policy output.
POLICY_TO_ROBOT_INDEX = tuple(POLICY_ORDER.index(name) for name in ROBOT_ORDER)
ROBOT_DEFAULT_JOINT_POS = tuple(
    DEFAULT_JOINT_POS[POLICY_ORDER.index(name)] for name in ROBOT_ORDER
)


@dataclass(frozen=True)
class ActuatorLimits:
    """Limits from the official S10 URDF, not the higher M20 limits."""

    leg_effort: float = 50.0
    leg_velocity: float = 25.76
    wheel_effort: float = 14.0
    wheel_velocity: float = 65.50


@dataclass(frozen=True)
class JointTargets:
    """Decoded targets in the interleaved order expected by the robot bridge."""

    leg_positions: dict[str, float]
    wheel_velocities: dict[str, float]


def decode_policy_action(action: Sequence[float]) -> JointTargets:
    """Convert one 16-D policy action into simultaneous leg and wheel targets.

    Leg outputs are residuals around the default stance.  Wheel outputs have no
    position offset and are interpreted as velocities.  The result is keyed by joint
    name so a caller cannot accidentally send policy order to the robot-order bridge.
    """

    if len(action) != len(POLICY_ORDER):
        raise ValueError(f"expected 16 policy actions, got {len(action)}")

    scaled_policy = [
        float(action[index]) * POLICY_ACTION_SCALE[index] + DEFAULT_JOINT_POS[index]
        for index in range(len(POLICY_ORDER))
    ]
    robot_targets = {
        joint: scaled_policy[POLICY_TO_ROBOT_INDEX[index]]
        for index, joint in enumerate(ROBOT_ORDER)
    }
    return JointTargets(
        leg_positions={name: robot_targets[name] for name in ROBOT_ORDER if "wheel" not in name},
        wheel_velocities={name: robot_targets[name] for name in ROBOT_ORDER if "wheel" in name},
    )
