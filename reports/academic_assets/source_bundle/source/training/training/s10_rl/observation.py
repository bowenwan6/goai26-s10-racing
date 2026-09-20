"""The observation layout, defined once.

Three places must agree on the exact ordering and scaling of the policy input: the
training environment, the ONNX export, and the C++ runner that feeds the deployed model.
A silent disagreement between them does not raise an error -- it produces a policy that
walks slightly wrong, which is expensive to diagnose. So the layout is defined here and
everything else derives from it.

The baseline layout matches the policy shipped with the contest SDK and is purely
proprioceptive: it has no terrain input at all, which is why the stock policy cannot
handle the elevated sections of the course. ``PERCEPTIVE`` extends it with a height map.
"""

from __future__ import annotations

from dataclasses import dataclass, field

MOTOR_NUM = 16
ACTION_DIM = 16

#: Applied to base angular velocity before it enters the observation.
OMEGA_SCALE = 0.25
#: Applied to joint velocities before they enter the observation.
DOF_VEL_SCALE = 0.05

# The literal groupings below mirror the robot's physical layout; keep them readable.
# fmt: off

#: Joint ordering used by the robot's actuators, four legs of four joints.
ROBOT_ORDER = [
    "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
    "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
    "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
    "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint",
]

#: Joint ordering used by the policy: all leg joints first, then all wheels.
POLICY_ORDER = [
    "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint",
    "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint",
    "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint",
    "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint",
    "fl_wheel_joint", "fr_wheel_joint", "hl_wheel_joint", "hr_wheel_joint",
]

#: Default joint positions in policy order. Wheels are continuous, so their default is 0.
DEFAULT_JOINT_POS = [
    0.0, -0.3, 0.6,
    0.0, -0.3, 0.6,
    0.0, 0.3, -0.6,
    0.0, 0.3, -0.6,
    0.0, 0.0, 0.0, 0.0,
]

#: Per-joint scaling from network output to joint target, in robot order.
ACTION_SCALE = [0.125, 0.25, 0.25, 5.0] * 4

# fmt: on


@dataclass(frozen=True)
class ObservationTerm:
    name: str
    size: int
    description: str


@dataclass(frozen=True)
class ObservationSpec:
    """An ordered list of observation terms and the offsets they occupy."""

    name: str
    terms: list[ObservationTerm] = field(default_factory=list)

    @property
    def dim(self) -> int:
        return sum(term.size for term in self.terms)

    def offsets(self) -> dict[str, tuple[int, int]]:
        """Map term name to its ``(start, end)`` slice in the flat observation vector."""
        result: dict[str, tuple[int, int]] = {}
        cursor = 0
        for term in self.terms:
            result[term.name] = (cursor, cursor + term.size)
            cursor += term.size
        return result

    def extended_with(self, name: str, *terms: ObservationTerm) -> ObservationSpec:
        """Return a new spec with terms appended.

        Perception is appended rather than inserted so that the proprioceptive prefix keeps
        the same offsets as the baseline. That lets a perceptive policy be initialised from
        baseline weights instead of trained from scratch.
        """
        return ObservationSpec(name=name, terms=[*self.terms, *terms])

    def describe(self) -> str:
        lines = [f"{self.name}: {self.dim} dimensions", ""]
        spans = list(self.offsets().values())
        for index, term in enumerate(self.terms):
            start, end = spans[index]
            lines.append(f"  [{start:3d}:{end:3d}]  {term.name:<20} {term.description}")
        return "\n".join(lines)


#: The layout consumed by the policy shipped with the contest SDK.
BASELINE = ObservationSpec(
    name="baseline",
    terms=[
        ObservationTerm("base_angular_velocity", 3, f"body frame, scaled by {OMEGA_SCALE}"),
        ObservationTerm("projected_gravity", 3, "gravity direction in the body frame"),
        ObservationTerm("velocity_command", 3, "forward, lateral, yaw rate"),
        ObservationTerm("joint_position", 16, "policy order, minus default, wheels zeroed"),
        ObservationTerm("joint_velocity", 16, f"policy order, scaled by {DOF_VEL_SCALE}"),
        ObservationTerm("last_action", 16, "previous network output"),
    ],
)

HEIGHTMAP_ROWS = 13
HEIGHTMAP_COLS = 9
#: Isaac Lab's ``mdp.height_scan`` subtracts this from base_z - terrain_z.
HEIGHT_SCAN_OFFSET = 0.5
#: MuJoCo uses the lower clip bound as a no-hit sentinel.
HEIGHTMAP_VOID_SENTINEL = -1.0

#: Baseline plus a body-frame terrain height map. This is what we deploy.
PERCEPTIVE = BASELINE.extended_with(
    "perceptive",
    ObservationTerm(
        "heightmap",
        HEIGHTMAP_ROWS * HEIGHTMAP_COLS,
        f"{HEIGHTMAP_ROWS}x{HEIGHTMAP_COLS} grid, terrain height relative to the base",
    ),
)


def raw_heightmap_to_policy(values: list[float]) -> list[float]:
    """Convert MuJoCo ``terrain_z - base_z`` values to Isaac Lab height-scan values.

    Isaac Lab trains on ``base_z - terrain_z - 0.5``. A MuJoCo no-hit cell is kept at
    ``-1`` to match Isaac Lab's clipped negative infinity instead of being mistaken for
    terrain 1 m below the base.
    """
    expected = HEIGHTMAP_ROWS * HEIGHTMAP_COLS
    if len(values) != expected:
        raise ValueError(f"expected {expected} height cells, got {len(values)}")

    converted = []
    for value in values:
        value = float(value)
        if value <= HEIGHTMAP_VOID_SENTINEL:
            converted.append(HEIGHTMAP_VOID_SENTINEL)
        else:
            converted.append(max(-1.0, min(1.0, -value - HEIGHT_SCAN_OFFSET)))
    return converted


def cpp_constant(spec: ObservationSpec) -> str:
    """Render the C++ line that must be kept in sync in the SDK's policy runner.

    ``s10_policy_runner.hpp`` hardcodes ``observation_dim``. Changing the layout without
    changing that constant loads the model successfully and then feeds it misaligned data.
    """
    return (
        f"const int observation_dim = {spec.dim};"
        f"  // {spec.name}: generated by training/s10_rl/observation.py"
    )


if __name__ == "__main__":
    for spec in (BASELINE, PERCEPTIVE):
        print(spec.describe())
        print(f"\n  {cpp_constant(spec)}\n")
