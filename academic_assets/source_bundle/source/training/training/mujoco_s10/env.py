"""A small, dependency-light S10 MuJoCo environment.

The control path matches the complete official simulator -> DDS -> policy chain:

* the network emits twelve leg position residuals plus four wheel velocities;
* the simulator and DDS calibration transforms cancel before the policy, so the
  network reads and writes MuJoCo joint coordinates directly;
* the simulator receives PD torques at 1 kHz while the policy runs at 50 Hz.

This is intentionally not a generic Gym wrapper.  The trainer needs only ``reset`` and
``step``, which keeps the hot loop free of ROS and optional framework dependencies.
"""

from __future__ import annotations

import math
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np

from s10_rl.observation import (
    DEFAULT_JOINT_POS,
    DOF_VEL_SCALE,
    OMEGA_SCALE,
    POLICY_ORDER,
    ROBOT_ORDER,
    raw_heightmap_to_policy,
)
from s10_rl.policy import POLICY_ACTION_SCALE

from .config import EnvConfig


KP = np.tile(np.asarray([80.0, 80.0, 80.0, 0.0]), 4)
KD = np.tile(np.asarray([2.0, 2.0, 2.0, 0.6]), 4)
EFFORT_LIMIT = np.tile(np.asarray([50.0, 50.0, 50.0, 14.0]), 4)
POLICY_SCALE = np.asarray(POLICY_ACTION_SCALE, dtype=np.float64)
DEFAULT_POLICY = np.asarray(DEFAULT_JOINT_POS, dtype=np.float64)
POLICY_TO_ROBOT = np.asarray([POLICY_ORDER.index(name) for name in ROBOT_ORDER])
ROBOT_TO_POLICY = np.asarray([ROBOT_ORDER.index(name) for name in POLICY_ORDER])
ROBOT_STAND_Q = DEFAULT_POLICY[POLICY_TO_ROBOT]
GRAVITY_WORLD = np.asarray([0.0, 0.0, -1.0])
WHEEL_RADIUS = 0.081
# This is not the erroneous [-1, 1] policy clamp removed in 010.  It is a
# catastrophic-state guard above every useful deployment command: leg outputs of
# +/-8 still cover the full joint range after scaling, while wheel outputs of
# +/-25 correspond to about 10.1 m/s at the tyre surface (beyond the published
# 8 m/s capability).  Reaching it means the recurrent last-action feedback has
# already diverged and the episode should be rejected, not integrated further.
ACTION_ABS_GUARD = np.asarray([8.0] * 12 + [25.0] * 4, dtype=np.float64)

# Official-policy Gate-16 trace, t=35.064 s (CSV file row 2232).  Coordinates are
# transplanted relative to the measured lip/floor; wheel angles are wrapped on reset
# because the collision geometry is rotationally symmetric.  This is an initial-state
# seed, not an action demonstration: the logged action is used only as the policy's
# recurrent last-action observation.
TRACE_FLOOR_Z = 0.102
TRACE_DECK_DEPTH = 0.377
TRACE_LIP_X = 12.6459
TRACE_QPOS = np.asarray(
    [
        12.68617430, 33.36546658, 0.67974150,
        0.94150073, 0.01551215, -0.33550301, 0.02781137,
        -0.07099035, -0.43085271, 0.64426676, -545.85596843,
        -0.05500289, -0.56179426, 0.89544809, -593.86335390,
        0.15089254, -0.41464376, -0.23685920, -499.18905094,
        0.03025968, -0.17891638, -0.80384152, -207.53619080,
    ],
    dtype=np.float64,
)
TRACE_JOINT_QVEL = np.asarray(
    [
        0.23021005, 1.26111696, -1.65760938, -28.57025423,
        -0.35488386, 0.21591702, 3.15056595, -11.83094001,
        0.42112560, -1.45784829, 1.65550967, -0.03808652,
        0.51438951, -0.59018655, -0.38904952, -15.71914460,
    ],
    dtype=np.float64,
)
TRACE_LAST_ACTION = np.asarray(
    [
        -0.82203293, -0.14728158, 0.22838817,
        -0.98641014, -1.61542492, 0.98627648,
        1.31921673, -6.44335194, 0.34827008,
        0.42936614, -0.87123687, -0.94150863,
        -6.18069878, -3.59509315, -4.55580750, -2.34647408,
    ],
    dtype=np.float32,
)


def _quat_from_yaw(yaw: float) -> np.ndarray:
    return np.asarray([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


def _quat_from_pitch(pitch: float) -> np.ndarray:
    return np.asarray([math.cos(pitch / 2.0), 0.0, math.sin(pitch / 2.0), 0.0])


def _pit_model(xml_path: Path, depth: float, length: float, cfg: EnvConfig):
    """Compile the official S10 XML with two raised slabs around a depression.

    The original XML is never modified.  A short-lived sibling XML preserves the
    official relative mesh directory during compilation and is removed immediately.
    """

    root = ET.parse(xml_path).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError(f"{xml_path} has no worldbody")

    half_gap = length / 2.0
    half_length = cfg.platform_half_length
    z_half = max(depth / 2.0, 0.00025)
    common = {
        "type": "box",
        "size": f"{half_length:.9g} {cfg.platform_half_width:.9g} {z_half:.9g}",
        "contype": "1",
        "conaffinity": "1",
        "condim": "3",
        # Group 1 is already used by S10 wheel/leg geoms.  A dedicated group 5
        # keeps height-map rays terrain-only, matching the organizer runtime's
        # use of group 0 for course meshes.
        "group": "5",
        "friction": "1 0.01 0.001",
        "rgba": "0.24 0.52 0.24 1",
    }
    # The S10 XML already provides the physical floor in MuJoCo's default geom
    # group 0, while the variable-height platforms live in dedicated group 5.
    # Height-map rays deliberately select only group 5 so they cannot hit the
    # robot's own group-1 collision geoms.  Without a group-5 pit floor, 95 of
    # the 117 samples are reported as misses, which changes the base actor's
    # action even when the synthetic and organizer ledges have identical shape.
    # This invisible, non-colliding plane restores the organizer observation:
    # rays see either the pit floor or the raised platform, while physical
    # contacts continue to use the untouched official floor.
    ET.SubElement(
        worldbody,
        "geom",
        name="training_heightmap_floor",
        type="plane",
        size="0 0 0.05",
        pos="0 0 0",
        contype="0",
        conaffinity="0",
        group="5",
        # MuJoCo ray casting skips fully transparent geoms.  A near-zero alpha
        # keeps the helper visually imperceptible while leaving it ray-visible.
        rgba="0 0 0 0.001",
    )
    left_center = cfg.pit_center_x - half_gap - half_length
    right_center = cfg.pit_center_x + half_gap + half_length
    ET.SubElement(
        worldbody,
        "geom",
        name="training_left_platform",
        pos=f"{left_center:.9g} 0 {z_half:.9g}",
        **common,
    )
    ET.SubElement(
        worldbody,
        "geom",
        name="training_right_platform",
        pos=f"{right_center:.9g} 0 {z_half:.9g}",
        **common,
    )

    temp_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", suffix=".xml", prefix="s10_training_", dir=xml_path.parent, delete=False
        ) as stream:
            temp_name = stream.name
            ET.ElementTree(root).write(stream, encoding="utf-8", xml_declaration=True)
        model = mujoco.MjModel.from_xml_path(temp_name)
    finally:
        if temp_name is not None:
            Path(temp_name).unlink(missing_ok=True)
    model.opt.timestep = cfg.physics_dt
    return model


class S10PitEnv:
    def __init__(self, config: EnvConfig) -> None:
        config.validate()
        self.cfg = config
        self.rng = np.random.default_rng(config.seed)
        self.last_action = np.zeros(16, dtype=np.float32)
        self.step_count = 0
        self.episode_return = 0.0
        self.best_right_wheels = 0
        self.best_front_lift = 0.0
        self.best_rear_lift = 0.0
        self.best_com_progress = 0.0
        self.pit_depth = float(config.pit_depth)
        self.pit_length = float(config.pit_length_min)
        self.model = None
        self.data = None
        self.base_id = -1
        self.left_platform_id = -1
        self.right_platform_id = -1
        self.wheel_body_ids = np.empty(0, dtype=np.int32)
        self._ray_geom_group = np.zeros(6, dtype=np.uint8)
        # Dedicated terrain-ray group; S10 itself uses groups 0 and 1.
        self._ray_geom_group[5] = 1
        xs = np.linspace(-0.60, 1.20, 13)
        ys = np.linspace(-0.60, 0.60, 9)
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        self._height_grid = np.stack([gx.ravel(), gy.ravel()], axis=-1)
        self._build_model()

    def _build_model(self) -> None:
        self.pit_length = 0.5 * (self.cfg.pit_length_min + self.cfg.pit_length_max)
        self.model = _pit_model(Path(self.cfg.xml_path), self.pit_depth, self.pit_length, self.cfg)
        self.data = mujoco.MjData(self.model)
        self.base_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link"
        )
        if self.base_id < 0 or self.model.nu != 16:
            raise ValueError("official S10 model must contain base_link and 16 actuators")
        self.left_platform_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "training_left_platform"
        )
        self.right_platform_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "training_right_platform"
        )
        if min(self.left_platform_id, self.right_platform_id) < 0:
            raise ValueError("training platform geoms are missing")
        self.wheel_body_ids = np.asarray(
            [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                for name in ("fl_wheel", "fr_wheel", "hl_wheel", "hr_wheel")
            ],
            dtype=np.int32,
        )
        if np.any(self.wheel_body_ids < 0):
            raise ValueError("official S10 wheel bodies are missing")

    def _set_pit_geometry(self) -> None:
        half_gap = self.pit_length / 2.0
        half_length = self.cfg.platform_half_length
        # MuJoCo box sizes must stay strictly positive. A zero-depth rehearsal is
        # represented by a sub-millimetre slab whose top is effectively the floor.
        z_half = max(self.pit_depth / 2.0, 0.00025)
        for geom_id in (self.left_platform_id, self.right_platform_id):
            self.model.geom_size[geom_id] = np.asarray(
                [half_length, self.cfg.platform_half_width, z_half]
            )
            self.model.geom_pos[geom_id, 2] = z_half
        self.model.geom_pos[self.left_platform_id, 0] = (
            self.cfg.pit_center_x - half_gap - half_length
        )
        self.model.geom_pos[self.right_platform_id, 0] = (
            self.cfg.pit_center_x + half_gap + half_length
        )

    def set_pit_depth(self, depth: float) -> None:
        depth = float(depth)
        if not 0.0 <= depth <= 0.50:
            raise ValueError("pit depth must be in [0, 0.50]")
        if abs(depth - self.pit_depth) > 1.0e-9:
            self.pit_depth = depth
            self._set_pit_geometry()

    def _set_front_up_pose(self) -> None:
        """Kinematically place front wheels on top and rear wheels on the pit floor.

        This recreates the team's observed stuck state without copying a controller.
        The pitch is solved from the official S10 forward kinematics for the current
        ledge height; no approximate wheelbase constant is baked into the reset.
        """

        right_lip_x = self.cfg.pit_center_x + self.pit_length / 2.0
        target_front_x = right_lip_x + self.rng.uniform(0.04, 0.08)
        target_rear_bottom = 0.0025

        def wheel_height_delta(pitch_magnitude: float) -> float:
            self.data.qpos[:3] = 0.0
            self.data.qpos[3:7] = _quat_from_pitch(-pitch_magnitude)
            mujoco.mj_forward(self.model, self.data)
            wheel_z = self.data.xpos[self.wheel_body_ids, 2]
            return float(np.mean(wheel_z[:2]) - np.mean(wheel_z[2:]))

        low, high = 0.0, 1.35
        if wheel_height_delta(high) < self.pit_depth:
            raise ValueError(
                f"front-up stand geometry cannot span {self.pit_depth:.3f} m"
            )
        for _ in range(32):
            middle = 0.5 * (low + high)
            if wheel_height_delta(middle) < self.pit_depth:
                low = middle
            else:
                high = middle
        pitch = -0.5 * (low + high)
        self.data.qpos[:3] = 0.0
        self.data.qpos[3:7] = _quat_from_pitch(pitch)
        mujoco.mj_forward(self.model, self.data)
        wheel_xyz = self.data.xpos[self.wheel_body_ids]
        self.data.qpos[0] += target_front_x - float(np.mean(wheel_xyz[:2, 0]))
        self.data.qpos[1] = self.rng.uniform(-0.03, 0.03)
        self.data.qpos[2] += (
            target_rear_bottom + WHEEL_RADIUS - float(np.mean(wheel_xyz[2:, 2]))
        )
        mujoco.mj_forward(self.model, self.data)

    def _set_trace_rear_pose(self) -> None:
        """Transplant the measured official-policy rear-wheel bottleneck state."""

        right_lip_x = self.cfg.pit_center_x + self.pit_length / 2.0
        qpos = TRACE_QPOS.copy()
        qpos[0] = right_lip_x + (TRACE_QPOS[0] - TRACE_LIP_X)
        qpos[1] = self.rng.uniform(-0.015, 0.015)
        # Preserve the measured front-wheel/deck relationship when the robustness
        # stage moves a few centimetres around the actual 0.377 m wall.
        qpos[2] = (
            TRACE_QPOS[2]
            - TRACE_FLOOR_Z
            + (self.pit_depth - TRACE_DECK_DEPTH)
            + self.rng.uniform(-0.003, 0.003)
        )
        qpos[7:23] += self.rng.normal(0.0, 0.005, size=16)
        for wheel_qpos in (10, 14, 18, 22):
            qpos[wheel_qpos] = (qpos[wheel_qpos] + math.pi) % (2.0 * math.pi) - math.pi
        self.data.qpos[:] = qpos
        self.data.qvel[:] = 0.0
        # Retain part of the real motion without making every reset inherit the same
        # high-energy oscillation that caused the original policy to wedge.
        self.data.qvel[6:22] = TRACE_JOINT_QVEL * self.rng.uniform(0.0, 0.50)
        self.last_action[:] = TRACE_LAST_ACTION
        mujoco.mj_forward(self.model, self.data)

    def reset(self, *, pit_depth: float | None = None) -> np.ndarray:
        if pit_depth is not None:
            self.set_pit_depth(pit_depth)
        self.pit_length = float(
            self.rng.uniform(self.cfg.pit_length_min, self.cfg.pit_length_max)
        )
        self._set_pit_geometry()

        assert self.data is not None
        mujoco.mj_resetData(self.model, self.data)
        if self.cfg.reset_mode == "front_up":
            start_x = 0.0
            surface_z = 0.0005
        elif self.cfg.reset_mode == "exit_only":
            right_lip_x = self.cfg.pit_center_x + self.pit_length / 2.0
            start_x = right_lip_x - self.rng.uniform(
                self.cfg.exit_approach_min, self.cfg.exit_approach_max
            )
            surface_z = 0.0005
        else:
            start_x = self.cfg.start_x + self.rng.uniform(-0.08, 0.08)
            surface_z = max(self.pit_depth, 0.0005)
        # RLControl starts only after the official four-second stand-up state.
        self.data.qpos[7:23] = ROBOT_STAND_Q
        self.data.qvel[:] = 0.0
        self.last_action.fill(0.0)
        self.step_count = 0
        self.episode_return = 0.0
        self.best_right_wheels = 0
        self.best_front_lift = 0.0
        self.best_rear_lift = 0.0
        self.best_com_progress = 0.0
        if self.cfg.reset_mode == "front_up":
            use_trace_seed = (
                self.pit_depth >= 0.35
                and self.rng.random() < self.cfg.trace_seed_probability
            )
            if use_trace_seed:
                self._set_trace_rear_pose()
            else:
                self._set_front_up_pose()
        else:
            self.data.qpos[:3] = np.asarray(
                [
                    start_x,
                    self.rng.uniform(
                        -self.cfg.reset_lateral_range,
                        self.cfg.reset_lateral_range,
                    ),
                    0.50 + surface_z + self.rng.uniform(-0.005, 0.005),
                ]
            )
            self.data.qpos[3:7] = _quat_from_yaw(
                self.rng.uniform(-self.cfg.reset_yaw_range, self.cfg.reset_yaw_range)
            )
        mujoco.mj_forward(self.model, self.data)
        # Reproduce the settled end of StandUpState without a multi-second reset.
        if self.cfg.reset_mode != "front_up":
            wheel_bottom = float(
                np.min(self.data.xpos[self.wheel_body_ids, 2] - WHEEL_RADIUS)
            )
            self.data.qpos[2] += surface_z - wheel_bottom + 0.002
            mujoco.mj_forward(self.model, self.data)
        if self.cfg.reset_mode == "exit_only" and self.data.ncon:
            min_contact_distance = min(
                float(self.data.contact[index].dist) for index in range(self.data.ncon)
            )
            if min_contact_distance < -0.01:
                raise RuntimeError(
                    "exit_only reset intersects terrain by "
                    f"{-min_contact_distance:.3f} m; increase pit length or move the start"
                )
        right_lip_x = self.cfg.pit_center_x + self.pit_length / 2.0
        wheel_xyz = self.data.xpos[self.wheel_body_ids]
        self.best_right_wheels = int(
            np.count_nonzero(
                (wheel_xyz[:, 0] >= right_lip_x + 0.02)
                & (wheel_xyz[:, 2] >= self.pit_depth + 0.70 * WHEEL_RADIUS)
            )
        )
        front_approach = np.clip(
            (wheel_xyz[:2, 0] - (right_lip_x - 0.45)) / 0.35, 0.0, 1.0
        )
        front_lift = np.clip(
            (wheel_xyz[:2, 2] - WHEEL_RADIUS) / max(self.pit_depth, 0.05),
            0.0,
            1.0,
        )
        self.best_front_lift = float(np.mean(front_approach * front_lift))
        rear_approach = np.clip(
            (wheel_xyz[2:, 0] - (right_lip_x - 0.35)) / 0.35, 0.0, 1.0
        )
        rear_lift = np.clip(
            (wheel_xyz[2:, 2] - WHEEL_RADIUS) / max(self.pit_depth, 0.05),
            0.0,
            1.0,
        )
        self.best_rear_lift = float(np.mean(rear_approach * rear_lift))
        self.best_com_progress = float(
            np.clip((self.data.qpos[0] - (right_lip_x - 0.35)) / 0.70, 0.0, 1.0)
        )
        return self.observation()

    def _projected_gravity(self) -> np.ndarray:
        rot = self.data.xmat[self.base_id].reshape(3, 3)
        return rot.T @ GRAVITY_WORLD

    def _heightmap(self) -> np.ndarray:
        base = self.data.xpos[self.base_id]
        rot = self.data.xmat[self.base_id].reshape(3, 3)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        rot2 = np.asarray([[c, -s], [s, c]])
        world_xy = self._height_grid @ rot2.T + base[:2]
        start_z = float(base[2] + 1.5)
        values = np.empty(117, dtype=np.float64)
        geomid = np.zeros(1, dtype=np.int32)
        ray = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)
        for index, (x, y) in enumerate(world_xy):
            dist = mujoco.mj_ray(
                self.model,
                self.data,
                np.asarray([x, y, start_z], dtype=np.float64),
                ray,
                self._ray_geom_group,
                1,
                self.base_id,
                geomid,
            )
            values[index] = -1.0 if dist < 0.0 or dist > 4.0 else (start_z - dist) - base[2]
        np.clip(values, -1.0, 1.0, out=values)
        return np.asarray(raw_heightmap_to_policy(values.tolist()), dtype=np.float32)

    def right_wheels_on_exit(self) -> int:
        """Return how many wheel centers have geometrically cleared the exit lip."""
        right_lip_x = self.cfg.pit_center_x + self.pit_length / 2.0
        wheel_xyz = self.data.xpos[self.wheel_body_ids]
        return int(
            np.count_nonzero(
                (wheel_xyz[:, 0] >= right_lip_x + 0.02)
                & (wheel_xyz[:, 2] >= self.pit_depth + 0.70 * WHEEL_RADIUS)
            )
        )

    def observation(self) -> np.ndarray:
        raw_q = self.data.qpos[7:23]
        raw_dq = self.data.qvel[6:22]
        q_policy = raw_q[ROBOT_TO_POLICY].copy()
        q_policy[12:16] = 0.0
        q_policy -= DEFAULT_POLICY
        dq_policy = raw_dq[ROBOT_TO_POLICY] * DOF_VEL_SCALE

        # framequat(4), accelerometer(3), gyro(3) in the official S10 sensor block.
        omega = np.asarray(self.data.sensordata[7:10]) * OMEGA_SCALE
        command = np.asarray([self.cfg.command_forward, 0.0, 0.0])
        obs = np.concatenate(
            [omega, self._projected_gravity(), command, q_policy, dq_policy, self.last_action]
        ).astype(np.float32)
        if self.cfg.observation_dim == 174:
            obs = np.concatenate([obs, self._heightmap()]).astype(np.float32)
        if obs.shape != (self.cfg.observation_dim,):
            raise RuntimeError(f"observation shape {obs.shape} does not match {self.cfg.observation_dim}")
        return obs

    def _apply_action(self, action: np.ndarray) -> np.ndarray:
        action = np.asarray(action, dtype=np.float64).reshape(16)
        if not np.all(np.isfinite(action)):
            raise FloatingPointError("policy action contains NaN or infinity")
        # Match the official S10 runner exactly: its ONNX output is multiplied by
        # action_scale_robot without a tanh or [-1, 1] clamp.  Clipping here silently
        # limited wheels to 5 rad/s and also prevented the large leg residuals needed
        # for extreme step-up motions.  Physical safety remains enforced by the same
        # actuator torque limits used below.
        scaled_policy = action * POLICY_SCALE + DEFAULT_POLICY
        robot_targets = scaled_policy[POLICY_TO_ROBOT]

        raw_pos_target = np.zeros(16, dtype=np.float64)
        raw_vel_target = np.zeros(16, dtype=np.float64)
        for leg in range(4):
            start = leg * 4
            raw_pos_target[start : start + 3] = robot_targets[start : start + 3]
            raw_vel_target[start + 3] = robot_targets[start + 3]

        torque = np.zeros(16, dtype=np.float64)
        for _ in range(self.cfg.frame_skip):
            raw_q = self.data.qpos[7:23]
            raw_dq = self.data.qvel[6:22]
            torque = KP * (raw_pos_target - raw_q) + KD * (raw_vel_target - raw_dq)
            np.clip(torque, -EFFORT_LIMIT, EFFORT_LIMIT, out=torque)
            self.data.ctrl[:] = torque
            mujoco.mj_step(self.model, self.data)
        return torque

    def step(self, action: np.ndarray):
        x_before = float(self.data.qpos[0])
        previous = self.last_action.copy()
        requested_action = np.asarray(action, dtype=np.float64).reshape(16)
        if not np.all(np.isfinite(requested_action)):
            raise FloatingPointError("policy action contains NaN or infinity")
        diverged = bool(np.any(np.abs(requested_action) > ACTION_ABS_GUARD))
        guarded_action = np.clip(requested_action, -ACTION_ABS_GUARD, ACTION_ABS_GUARD)
        torque = self._apply_action(guarded_action)
        # Normal actions remain exactly identical to the official runner.  Only a
        # policy state already outside physical usefulness is contained so it cannot
        # recursively amplify through the last-action observation.
        self.last_action[:] = guarded_action.astype(np.float32)
        self.step_count += 1

        x_after = float(self.data.qpos[0])
        y_after = float(self.data.qpos[1])
        vx = (x_after - x_before) / self.cfg.control_dt
        gravity = self._projected_gravity()
        velocity_tracking = math.exp(-((vx - self.cfg.command_forward) ** 2) / 0.25)
        # At reduced obstacle-approach speeds the Gaussian alone rewards standing
        # still.  Gate it by actual forward motion so a pause may be useful briefly,
        # but cannot become the best long-horizon policy.
        moving_gate = float(
            np.clip(vx / max(abs(self.cfg.command_forward), 0.10), 0.0, 1.0)
        )
        progress = float(np.clip(vx, -1.0, 2.0))
        tilt = 1.0 + float(gravity[2])
        action_rate = float(np.mean(np.square(self.last_action - previous)))
        effort = float(np.mean(np.square(torque / EFFORT_LIMIT)))

        # Sparse success alone is too late to teach the mechanically necessary
        # front-wheels-first climb.  Reward each *new* wheel that genuinely reaches
        # the exit platform (both beyond the lip and above its top).  Because the
        # bonus is one-shot and tied to the exit geometry, it cannot be collected by
        # jumping in place or repeatedly breaking contact.
        right_lip_x = self.cfg.pit_center_x + self.pit_length / 2.0
        wheel_xyz = self.data.xpos[self.wheel_body_ids]
        right_wheels = self.right_wheels_on_exit()
        new_right_wheels = max(0, right_wheels - self.best_right_wheels)
        self.best_right_wheels = max(self.best_right_wheels, right_wheels)

        # Potential-based front-wheel lift shaping.  The old sparse reward appeared
        # only after a wheel was already on top, leaving no gradient at 15 cm.  This
        # one-shot score grows only when both front wheels approach the exit and lift
        # their bottoms toward the platform top; lifting in place far from the lip
        # earns nothing and repeating the same motion cannot farm reward.
        front_xyz = wheel_xyz[:2]
        approach = np.clip(
            (front_xyz[:, 0] - (right_lip_x - 0.45)) / 0.35,
            0.0,
            1.0,
        )
        lift_fraction = np.clip(
            (front_xyz[:, 2] - WHEEL_RADIUS) / max(self.pit_depth, 0.05),
            0.0,
            1.0,
        )
        front_lift_score = float(np.mean(approach * lift_fraction))
        new_front_lift = max(0.0, front_lift_score - self.best_front_lift)
        self.best_front_lift = max(self.best_front_lift, front_lift_score)

        rear_xyz = wheel_xyz[2:]
        rear_approach = np.clip(
            (rear_xyz[:, 0] - (right_lip_x - 0.35)) / 0.35,
            0.0,
            1.0,
        )
        rear_lift_fraction = np.clip(
            (rear_xyz[:, 2] - WHEEL_RADIUS) / max(self.pit_depth, 0.05),
            0.0,
            1.0,
        )
        rear_lift_score = float(np.mean(rear_approach * rear_lift_fraction))
        new_rear_lift = max(0.0, rear_lift_score - self.best_rear_lift)
        self.best_rear_lift = max(self.best_rear_lift, rear_lift_score)

        com_progress = float(
            np.clip((x_after - (right_lip_x - 0.35)) / 0.70, 0.0, 1.0)
        )
        new_com_progress = max(0.0, com_progress - self.best_com_progress)
        self.best_com_progress = max(self.best_com_progress, com_progress)

        # A 0.377 m exit lip requires deliberate pitch.  Keep the ordinary-locomotion
        # posture cost away from the obstacle, but soften it while the base is within
        # one body length of either lip so that stopping upright at the wall is not the
        # cheapest behaviour.
        obstacle_near = abs(x_after - self.cfg.pit_center_x) <= self.pit_length / 2.0 + 0.65
        tilt_weight = 1.5 if obstacle_near else 6.0

        reward = (
            2.0 * progress
            + 1.5 * velocity_tracking * moving_gate
            - tilt_weight * tilt * tilt
            - 2.0 * y_after * y_after
            - 0.02 * action_rate
            - 0.01 * effort
        ) * self.cfg.control_dt
        reward += 1.5 * new_right_wheels
        reward += 4.0 * new_front_lift
        reward += 8.0 * new_rear_lift
        reward += 3.0 * new_com_progress
        if self.cfg.reset_mode == "front_up":
            reward -= 0.02 * max(0, 2 - right_wheels)
        # Remove the positive local optimum at vx=0.  A stationary robot previously
        # earned velocity-kernel reward for the entire timeout horizon.
        reward -= 0.01

        # Success means the complete robot has cleared the exit and continued by
        # approximately one chassis length; merely raising the base does not count.
        finish_x = self.cfg.pit_center_x + self.pit_length / 2.0 + 0.65
        success = x_after >= finish_x and right_wheels == 4
        fallen = bool(self.data.qpos[2] < 0.24 or gravity[2] > -0.25)
        # A policy that crosses the finish line on the final control step succeeded;
        # do not report the same episode as both success and timeout.
        timeout = self.step_count >= self.cfg.max_steps and not (success or diverged)
        done = success or fallen or timeout or diverged
        if success:
            reward += 20.0
        if diverged:
            reward -= 10.0
        elif fallen:
            reward -= 5.0
        elif timeout:
            reward -= 5.0

        self.episode_return += reward
        info = {
            "success": success,
            "fallen": fallen,
            "timeout": timeout,
            "diverged": diverged,
            "max_abs_action": float(np.max(np.abs(requested_action))),
            "pit_depth": self.pit_depth,
            "pit_length": self.pit_length,
            "reset_mode": self.cfg.reset_mode,
            "finish_x": finish_x,
            "x": x_after,
            "right_wheels": right_wheels,
            "front_lift_score": front_lift_score,
            "rear_lift_score": rear_lift_score,
            "com_progress": com_progress,
            "episode_return": self.episode_return,
        }
        return self.observation(), float(reward), done, info


def clone_config(config: EnvConfig, *, seed: int, pit_depth: float | None = None) -> EnvConfig:
    return replace(config, seed=seed, pit_depth=config.pit_depth if pit_depth is None else pit_depth)
