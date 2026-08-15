"""A headless copy of the race dynamics, for designing manoeuvres the policy cannot do.

The course has a 0.377 m vertical step at waypoint 15 that the shipped locomotion policy
cannot climb, and the basin containing that waypoint has no other exit. Getting out needs a
purpose-built whole-body motion, and designing one by launching the full stack for every
attempt is far too slow to iterate on.

This module removes everything except the physics. The simulator computes

    input_tq = kp * (pos_cmd - q) + kd * (vel_cmd - dq) + tau_ff
    data.ctrl[:] = input_tq

and writes it straight to the actuators, which are pure torque sources: ``gaintype`` 0,
``biastype`` 0, ``gear`` 1.0, so there is no hidden internal controller. Reproducing that one
line against the same scene reproduces the race exactly. An episode then costs about a second
instead of a container launch, which is what makes searching over manoeuvres affordable.

Two details of the interface matter more than they look:

``kp`` and ``kd`` arrive per joint on every JointDataCmd message. The 80/2 the policy runner
uses is its own choice, not a property of the robot, and a manoeuvre is free to ask for
something stiffer. It has to: at 80 the robot sags roughly 10 cm under its own weight in a
tall stance, which is a third of the step height.

Wheel joints are commanded with ``kp`` forced to zero, so they take velocity, not position.
A trajectory that tries to hold a wheel at an angle will silently do nothing at all.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

#: The scene the race runs on, as installed by scripts/setup_upstream.sh.
TRACK_XML = Path(
    "upstream/goai_embodied_future_material/src/S10_sdk_deploy/"
    "S10_description/s10_mjcf/mjcf/S10_track.xml"
)

#: The race overrides the scene's own timestep. `mujoco_simulation_ros2.py` sets
#: ``model.opt.timestep = DT`` with ``DT = 0.001``, half of what S10_track.xml declares, and
#: the difference is not cosmetic: at 0.002 the contact solver cannot resolve a wheel arriving
#: at the vertical face of the Gate 16 wall, and the robot is thrown metres into the air.
SIM_TIMESTEP = 0.001

#: Upstream's standing pose, JOINT_INIT["S10"] from mujoco_simulation_ros2.py.
JOINT_INIT = np.array([-0.438, -1.16, 2.76, 0.0,
                       0.438, -1.16, 2.76, 0.0,
                       -0.438, 1.16, -2.76, 0.0,
                       0.438, 1.16, -2.76, 0.0], dtype=np.float64)

#: Gains the policy runner deploys, per leg: hipx, hipy, knee, wheel.
POLICY_KP = np.tile([80.0, 80.0, 80.0, 0.0], 4)
POLICY_KD = np.tile([2.0, 2.0, 2.0, 0.6], 4)

LEGS = ("fl", "fr", "hl", "hr")
JOINT_NAMES = [f"{leg}_{part}_joint"
               for leg in LEGS for part in ("hipx", "hipy", "knee", "wheel")]


def gains(kp: float, kd: float | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Uniform leg gains, with wheels left in velocity mode where they belong.

    Damping defaults to scaling as the square root of stiffness, which keeps the joint near
    the same damping ratio; holding kd fixed while raising kp makes the leg ring instead of
    settling, and the ringing shows up as the robot bouncing off the lip.
    """
    kd = 2.0 * np.sqrt(kp / 80.0) if kd is None else kd
    return np.tile([kp, kp, kp, 0.0], 4), np.tile([kd, kd, kd, 0.6], 4)


def find_track_xml(start: Path | None = None) -> Path:
    """Locate the scene by walking up from here, so callers need no working directory."""
    here = Path(__file__).resolve() if start is None else Path(start).resolve()
    for parent in [here, *here.parents]:
        candidate = parent / TRACK_XML
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        f"{TRACK_XML} not found above {here}; run scripts/setup_upstream.sh first"
    )


class Sandbox:
    """The race's dynamics and control law, with no ROS attached."""

    def __init__(self, xml: Path | str | None = None):
        self.model = mujoco.MjModel.from_xml_path(str(xml or find_track_xml()))
        self.model.opt.timestep = SIM_TIMESTEP
        self.data = mujoco.MjData(self.model)

        self.qadr, self.vadr = [], []
        for name in JOINT_NAMES:
            j = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if j < 0:
                raise ValueError(f"joint {name!r} is not in the scene")
            self.qadr.append(self.model.jnt_qposadr[j])
            self.vadr.append(self.model.jnt_dofadr[j])
        self.qadr = np.array(self.qadr)
        self.vadr = np.array(self.vadr)

        self.torque_lo = self.model.actuator_ctrlrange[:, 0].copy()
        self.torque_hi = self.model.actuator_ctrlrange[:, 1].copy()
        self.base = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        #: Set per joint whenever a commanded torque had to be clipped. A manoeuvre that
        #: saturates is one whose margins are imaginary, so this is worth reporting.
        self.saturated = np.zeros(16, dtype=bool)

    # -- state ------------------------------------------------------------------

    def reset(self, base_xyz, pitch_deg: float = 0.0, joints=None) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = base_xyz
        half = np.radians(pitch_deg) / 2.0
        self.data.qpos[3:7] = (np.cos(half), 0.0, np.sin(half), 0.0)
        self.data.qpos[self.qadr] = JOINT_INIT if joints is None else joints
        self.data.qvel[:] = 0.0
        self.saturated[:] = False
        mujoco.mj_forward(self.model, self.data)

    @property
    def q(self) -> np.ndarray:
        return self.data.qpos[self.qadr]

    @property
    def dq(self) -> np.ndarray:
        return self.data.qvel[self.vadr]

    @property
    def base_pos(self) -> np.ndarray:
        return self.data.qpos[:3].copy()

    def tilt_deg(self) -> float:
        """Angle between the base's own up axis and world up, in degrees."""
        up = self.data.xmat[self.base].reshape(3, 3)[:, 2]
        return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))

    # -- control ----------------------------------------------------------------

    def step(self, pos_cmd, vel_cmd=None, tau_ff=None, kp=None, kd=None) -> None:
        """Advance one timestep under the simulator's own control law."""
        vel_cmd = np.zeros(16) if vel_cmd is None else vel_cmd
        tau_ff = np.zeros(16) if tau_ff is None else tau_ff
        kp = POLICY_KP if kp is None else kp
        kd = POLICY_KD if kd is None else kd

        tau = kp * (pos_cmd - self.q) + kd * (vel_cmd - self.dq) + tau_ff
        self.saturated |= (tau < self.torque_lo) | (tau > self.torque_hi)
        self.data.ctrl[:] = np.clip(tau, self.torque_lo, self.torque_hi)
        mujoco.mj_step(self.model, self.data)

    def hold(self, seconds: float, pos_cmd, **kwargs) -> None:
        """Track one fixed target for a while."""
        for _ in range(int(round(seconds / self.model.opt.timestep))):
            self.step(pos_cmd, **kwargs)
