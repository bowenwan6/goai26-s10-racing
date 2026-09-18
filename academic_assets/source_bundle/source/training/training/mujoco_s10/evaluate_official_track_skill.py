#!/usr/bin/env python3
"""Evaluate a climb skill directly in the organizer's S10_track.xml scene."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from s10_rl.observation import raw_heightmap_to_policy

from .cem_climb import capture_snapshot
from .cem_full_climb import full_rollout
from .distill_cem import _load_inference_actor
from .front_retention import FrontRetentionSample
from .env import (
    ACTION_ABS_GUARD,
    DEFAULT_POLICY,
    DOF_VEL_SCALE,
    EFFORT_LIMIT,
    GRAVITY_WORLD,
    KD,
    KP,
    OMEGA_SCALE,
    POLICY_SCALE,
    POLICY_TO_ROBOT,
    ROBOT_STAND_Q,
    ROBOT_TO_POLICY,
    WHEEL_RADIUS,
    _quat_from_yaw,
)


class OfficialTrackRuntime:
    """Minimal 50 Hz policy runtime over the untouched official track model."""

    def __init__(self, xml_path: Path, *, max_steps: int = 850) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.model.opt.timestep = 0.001
        self.data = mujoco.MjData(self.model)
        self.base_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link"
        )
        self.wheel_body_ids = np.asarray(
            [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                for name in ("fl_wheel", "fr_wheel", "hl_wheel", "hr_wheel")
            ],
            dtype=np.int32,
        )
        if self.base_id < 0 or np.any(self.wheel_body_ids < 0) or self.model.nu != 16:
            raise ValueError("official track must contain the 16-actuator S10")

        # Organizer-track ray casts at y=33.365: floor 0.101814 m, deck
        # 0.478725 m, exit lip x approximately 12.646 m.
        self.floor_z = 0.10181425
        self.deck_z = 0.47872480
        self.pit_depth = self.deck_z - self.floor_z
        self.pit_length = 2.25
        self.right_lip_x = 12.646
        self.cfg = SimpleNamespace(
            pit_center_x=self.right_lip_x - self.pit_length / 2.0,
            command_forward=0.30,
        )
        self.max_steps = int(max_steps)
        self.last_action = np.zeros(16, dtype=np.float32)
        self.step_count = 0
        self.episode_return = 0.0
        self.best_right_wheels = 0
        self.best_front_lift = 0.0
        self.best_rear_lift = 0.0
        self.best_com_progress = 0.0
        self._geomgroup = np.zeros(6, dtype=np.uint8)
        # Organizer terrain meshes use MuJoCo's default group 0.  Group 1 is
        # used by the simplified training fixture, so carrying that mask into
        # S10_track.xml makes every height-map ray miss the course.
        self._geomgroup[0] = 1
        xs = np.linspace(-0.60, 1.20, 13)
        ys = np.linspace(-0.60, 0.60, 9)
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        self._height_grid = np.stack([gx.ravel(), gy.ravel()], axis=-1)

    def set_pit_depth(self, depth: float) -> None:
        if abs(float(depth) - self.pit_depth) > 0.002:
            raise ValueError("official track height is fixed")

    def _set_pit_geometry(self) -> None:
        """Compatibility hook: the organizer scene geometry stays untouched."""
        return None

    def reset_floor(
        self,
        *,
        distance: float,
        lateral: float,
        yaw: float,
        base_height_noise: float = 0.0,
        entry_center_y: float = 33.365,
        forward_speed: float = 0.0,
        lateral_speed: float = 0.0,
        yaw_rate: float = 0.0,
    ) -> np.ndarray:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[7:23] = ROBOT_STAND_Q
        self.data.qvel[:] = 0.0
        self.data.qpos[:3] = np.asarray(
            [
                self.right_lip_x - distance,
                entry_center_y + lateral,
                self.floor_z + 0.50 + base_height_noise,
            ]
        )
        self.data.qpos[3:7] = _quat_from_yaw(yaw)
        # MuJoCo free-joint linear velocity is world-frame.  Navigation logs expose
        # forward/lateral velocity in the body frame, so rotate it by the measured
        # heading before starting the deterministic handoff episode.
        cosine, sine = math.cos(yaw), math.sin(yaw)
        self.data.qvel[0] = cosine * forward_speed - sine * lateral_speed
        self.data.qvel[1] = sine * forward_speed + cosine * lateral_speed
        self.data.qvel[5] = yaw_rate
        self.data.ctrl[:] = 0.0
        self.last_action.fill(0.0)
        self.step_count = 0
        self.episode_return = 0.0
        self.best_right_wheels = 0
        self.best_front_lift = 0.0
        self.best_rear_lift = 0.0
        self.best_com_progress = 0.0
        mujoco.mj_forward(self.model, self.data)
        wheel_bottom = float(
            np.min(self.data.xpos[self.wheel_body_ids, 2] - WHEEL_RADIUS)
        )
        self.data.qpos[2] += self.floor_z + 0.002 - wheel_bottom
        mujoco.mj_forward(self.model, self.data)
        if self.data.ncon:
            penetration = min(
                float(self.data.contact[index].dist) for index in range(self.data.ncon)
            )
            if penetration < -0.01:
                raise RuntimeError(
                    f"official-track reset penetrates geometry by {-penetration:.3f} m"
                )
        self._refresh_metrics()
        return self.observation()

    def _projected_gravity(self) -> np.ndarray:
        rotation = self.data.xmat[self.base_id].reshape(3, 3)
        return rotation.T @ GRAVITY_WORLD

    def _heightmap(self) -> np.ndarray:
        base = self.data.xpos[self.base_id]
        rotation = self.data.xmat[self.base_id].reshape(3, 3)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        world_xy = self._height_grid @ np.asarray([[c, -s], [s, c]]).T + base[:2]
        start_z = float(base[2] + 1.5)
        values = np.empty(117, dtype=np.float64)
        ray = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)
        geomid = np.zeros(1, dtype=np.int32)
        for index, (x, y) in enumerate(world_xy):
            distance = mujoco.mj_ray(
                self.model,
                self.data,
                np.asarray([x, y, start_z], dtype=np.float64),
                ray,
                self._geomgroup,
                1,
                self.base_id,
                geomid,
            )
            values[index] = (
                -1.0
                if distance < 0.0 or distance > 4.0
                else (start_z - distance) - base[2]
            )
        np.clip(values, -1.0, 1.0, out=values)
        return np.asarray(raw_heightmap_to_policy(values.tolist()), dtype=np.float32)

    def observation(self) -> np.ndarray:
        raw_q = self.data.qpos[7:23]
        raw_dq = self.data.qvel[6:22]
        q_policy = raw_q[ROBOT_TO_POLICY].copy()
        q_policy[12:16] = 0.0
        q_policy -= DEFAULT_POLICY
        dq_policy = raw_dq[ROBOT_TO_POLICY] * DOF_VEL_SCALE
        omega = np.asarray(self.data.sensordata[7:10]) * OMEGA_SCALE
        command = np.asarray([self.cfg.command_forward, 0.0, 0.0])
        return np.concatenate(
            [
                omega,
                self._projected_gravity(),
                command,
                q_policy,
                dq_policy,
                self.last_action,
                self._heightmap(),
            ]
        ).astype(np.float32)

    def _apply_action(self, action: np.ndarray) -> None:
        scaled_policy = np.asarray(action, dtype=np.float64) * POLICY_SCALE + DEFAULT_POLICY
        robot_targets = scaled_policy[POLICY_TO_ROBOT]
        position_target = np.zeros(16, dtype=np.float64)
        velocity_target = np.zeros(16, dtype=np.float64)
        for leg in range(4):
            start = 4 * leg
            position_target[start : start + 3] = robot_targets[start : start + 3]
            velocity_target[start + 3] = robot_targets[start + 3]
        for _ in range(20):
            q = self.data.qpos[7:23]
            dq = self.data.qvel[6:22]
            torque = KP * (position_target - q) + KD * (velocity_target - dq)
            np.clip(torque, -EFFORT_LIMIT, EFFORT_LIMIT, out=torque)
            self.data.ctrl[:] = torque
            mujoco.mj_step(self.model, self.data)

    def right_wheels_on_exit(self) -> int:
        wheels = self.data.xpos[self.wheel_body_ids]
        return int(
            np.count_nonzero(
                (wheels[:, 0] >= self.right_lip_x + 0.02)
                & (wheels[:, 2] >= self.deck_z + 0.70 * WHEEL_RADIUS)
            )
        )

    def front_retention_sample(self) -> FrontRetentionSample:
        """Return instantaneous support and balance state for reward/evaluation.

        A wheel is supported only after its centre is beyond the exit lip and high
        enough to be carried by the upper deck.  The minimum front clearance is the
        weaker of forward and vertical margins, so either a slide-back or a vertical
        drop is visible to the tracker.
        """
        wheels = self.data.xpos[self.wheel_body_ids]
        x_margin = wheels[:, 0] - (self.right_lip_x + 0.02)
        z_margin = wheels[:, 2] - (self.deck_z + 0.70 * WHEEL_RADIUS)
        supported = (x_margin >= 0.0) & (z_margin >= 0.0)
        front_margin = np.minimum(x_margin[:2], z_margin[:2])
        gravity = self._projected_gravity()
        return FrontRetentionSample(
            step=int(self.step_count),
            front_supported=int(np.count_nonzero(supported[:2])),
            rear_supported=int(np.count_nonzero(supported[2:])),
            front_min_clearance_m=float(np.min(front_margin)),
            balance_error=float(np.linalg.norm(gravity[:2])),
        )

    def _refresh_metrics(self) -> None:
        wheels = self.data.xpos[self.wheel_body_ids]
        self.best_right_wheels = max(self.best_right_wheels, self.right_wheels_on_exit())
        front_x = np.clip(
            (wheels[:2, 0] - (self.right_lip_x - 0.45)) / 0.35, 0.0, 1.0
        )
        rear_x = np.clip(
            (wheels[2:, 0] - (self.right_lip_x - 0.35)) / 0.35, 0.0, 1.0
        )
        front_z = np.clip(
            (wheels[:2, 2] - self.floor_z - WHEEL_RADIUS) / self.pit_depth,
            0.0,
            1.0,
        )
        rear_z = np.clip(
            (wheels[2:, 2] - self.floor_z - WHEEL_RADIUS) / self.pit_depth,
            0.0,
            1.0,
        )
        self.best_front_lift = max(
            self.best_front_lift, float(np.mean(front_x * front_z))
        )
        self.best_rear_lift = max(
            self.best_rear_lift, float(np.mean(rear_x * rear_z))
        )
        self.best_com_progress = max(
            self.best_com_progress,
            float(
                np.clip(
                    (self.data.qpos[0] - (self.right_lip_x - 0.35)) / 0.70,
                    0.0,
                    1.0,
                )
            ),
        )

    def step(self, action: np.ndarray):
        requested = np.asarray(action, dtype=np.float64).reshape(16)
        diverged = bool(
            not np.all(np.isfinite(requested))
            or np.any(np.abs(requested) > ACTION_ABS_GUARD)
        )
        guarded = np.clip(requested, -ACTION_ABS_GUARD, ACTION_ABS_GUARD)
        self._apply_action(guarded)
        self.last_action[:] = guarded.astype(np.float32)
        self.step_count += 1
        self._refresh_metrics()
        gravity = self._projected_gravity()
        fallen = bool(
            gravity[2] > -0.20 or self.data.qpos[2] < self.floor_z + 0.12
        )
        success = self.right_wheels_on_exit() == 4
        done = bool(success or fallen or diverged or self.step_count >= self.max_steps)
        info = {
            "success": success,
            "fallen": fallen,
            "diverged": diverged,
            "right_wheels": self.right_wheels_on_exit(),
            "x": float(self.data.qpos[0]),
            "episode_return": 0.0,
            "pit_depth": self.pit_depth,
        }
        return self.observation(), 0.0, done, info


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--xml", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trials", type=int, default=50)
    parser.add_argument("--seed", type=int, default=120_377)
    parser.add_argument("--distance-min", type=float, default=0.80)
    parser.add_argument("--distance-max", type=float, default=1.20)
    parser.add_argument("--lateral-range", type=float, default=0.08)
    parser.add_argument("--yaw-range", type=float, default=0.05)
    parser.add_argument("--height-noise", type=float, default=0.005)
    parser.add_argument("--heading-gain", type=float)
    parser.add_argument("--prealign-tolerance", type=float)
    parser.add_argument("--prealign-steps", type=int)
    parser.add_argument("--trigger-settle-steps", type=int)
    parser.add_argument("--phase-sync-start", type=float)
    parser.add_argument("--phase-sync-threshold", type=float, default=0.15)
    parser.add_argument("--phase-sync-max-steps", type=int, default=50)
    parser.add_argument("--pretrigger-forward", type=float)
    args = parser.parse_args()

    archive = np.load(args.trajectory)
    parameters = np.asarray(archive["approach_knots"], dtype=np.float64)
    rear_residual = np.asarray(archive["rear_residual"], dtype=np.float32)
    approach_horizon = int(np.asarray(archive["approach_residual"]).shape[0])
    trigger_distance = float(archive["trigger_distance"])
    heading_gain = (
        float(archive["heading_align_gain"])
        if "heading_align_gain" in archive.files
        else 0.0
    )
    tolerance = (
        float(archive["prealign_tolerance"])
        if "prealign_tolerance" in archive.files
        else 0.0
    )
    prealign_steps = (
        int(archive["prealign_max_steps"])
        if "prealign_max_steps" in archive.files
        else 0
    )
    settle_steps = (
        int(archive["trigger_settle_steps"])
        if "trigger_settle_steps" in archive.files
        else 0
    )
    if args.heading_gain is not None:
        heading_gain = args.heading_gain
    if args.prealign_tolerance is not None:
        tolerance = args.prealign_tolerance
    if args.prealign_steps is not None:
        prealign_steps = args.prealign_steps
    if args.trigger_settle_steps is not None:
        settle_steps = args.trigger_settle_steps
    pretrigger_forward = args.pretrigger_forward
    if pretrigger_forward is None and "pretrigger_forward" in archive.files:
        stored_forward = float(archive["pretrigger_forward"])
        if np.isfinite(stored_forward):
            pretrigger_forward = stored_forward
    actor = _load_inference_actor(args.checkpoint, 174)
    env = OfficialTrackRuntime(args.xml)
    phase_reference_qpos = None
    phase_reference_qvel = None
    if args.phase_sync_start is not None:
        if "phase_reference_qpos" in archive.files:
            phase_reference_qpos = np.asarray(
                archive["phase_reference_qpos"], dtype=np.float64
            )
            phase_reference_qvel = np.asarray(
                archive["phase_reference_qvel"], dtype=np.float64
            )
        else:
            recorded_qpos = np.asarray(archive["qpos"], dtype=np.float64)
            recorded_qvel = np.asarray(archive["qvel"], dtype=np.float64)
            candidates = np.flatnonzero(
                recorded_qpos[:, 0] >= env.right_lip_x - trigger_distance
            )
            reference_index = int(candidates[0]) if candidates.size else 0
            phase_reference_qpos = recorded_qpos[reference_index, 7:23]
            phase_reference_qvel = recorded_qvel[reference_index, 6:22]
    rng = np.random.default_rng(args.seed)
    rows = []
    recorded = None
    best_record = None
    best_key = (-1, -float("inf"))
    for trial in range(args.trials):
        distance = float(rng.uniform(args.distance_min, args.distance_max))
        lateral = float(rng.uniform(-args.lateral_range, args.lateral_range))
        yaw = float(rng.uniform(-args.yaw_range, args.yaw_range))
        base_height_noise = float(rng.uniform(-args.height_noise, args.height_noise))
        env.reset_floor(
            distance=distance,
            lateral=lateral,
            yaw=yaw,
            base_height_noise=base_height_noise,
        )
        result = full_rollout(
            env,
            capture_snapshot(env),
            actor,
            actor,
            parameters,
            rear_residual,
            approach_horizon,
            trigger_distance,
            heading_gain,
            tolerance,
            prealign_steps,
            settle_steps,
            phase_reference_qpos,
            phase_reference_qvel,
            args.phase_sync_start,
            args.phase_sync_threshold,
            args.phase_sync_max_steps,
            pretrigger_forward,
            record=True,
        )
        if recorded is None and result.success:
            recorded = result
        candidate_key = (
            int(result.final_info.get("right_wheels", 0)),
            float(result.final_info.get("x", -float("inf"))),
        )
        if candidate_key > best_key:
            best_key = candidate_key
            best_record = result
        rows.append(
            {
                "trial": trial,
                "initial_distance": distance,
                "initial_lateral": lateral,
                "initial_yaw": yaw,
                "prealign_steps": int(result.final_info.get("prealign_steps", 0)),
                "prealign_final_yaw": float(
                    result.final_info.get("prealign_final_yaw", yaw)
                ),
                "phase_sync_steps": int(
                    result.final_info.get("phase_sync_steps", 0)
                ),
                "phase_sync_min_error": float(
                    result.final_info.get("phase_sync_min_error", float("inf"))
                ),
                "success": int(result.success),
                "fallen": int(result.fallen),
                "rear_triggered": int(result.final_info.get("rear_triggered", False)),
                "right_wheels": int(result.final_info.get("right_wheels", 0)),
                "steps": result.steps,
                "final_x": float(result.final_info["x"]),
            }
        )
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "evaluation.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    replay = recorded if recorded is not None else best_record
    if replay is not None:
        np.savez_compressed(
            args.output / "diagnostic_replay.npz",
            qpos=replay.qpos,
            qvel=replay.qvel,
            actions=replay.actions,
            observations=replay.observations,
            rewards=replay.rewards,
            pit_depth=np.asarray(env.pit_depth),
            pit_length=np.asarray(env.pit_length),
        )
    success_rate = float(np.mean([row["success"] for row in rows]))
    fall_rate = float(np.mean([row["fallen"] for row in rows]))
    summary = {
        "scene": str(args.xml.resolve()),
        "trials": len(rows),
        "success_rate": success_rate,
        "fall_rate": fall_rate,
        "rear_trigger_rate": float(
            np.mean([row["rear_triggered"] for row in rows])
        ),
        "depth_m": env.pit_depth,
        "pit_width_m": env.pit_length,
        "pass_rate": 0.95,
        "pass": bool(success_rate >= 0.95),
        "successful_replay": str((args.output / "diagnostic_replay.npz").resolve())
        if recorded is not None
        else None,
        "diagnostic_replay": str((args.output / "diagnostic_replay.npz").resolve())
        if replay is not None
        else None,
    }
    (args.output / "evaluation.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
