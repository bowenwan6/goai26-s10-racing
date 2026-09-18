"""Kinematic S10 on the heightfield: (x, y, yaw) integrated from native-limited commands.

z = ground-plane height under the body + body_z_offset; pitch/roll from a least-squares
plane over the 0.9 x 0.5 m footprint. No dynamics, no leg/wheel contact: a planning-level
proxy. Limits from docs/S10_NAVIGATION_MAPPING_SENSOR_DESIGN_REFERENCE_ZH.md sec. 8.5 and
the route_v2 contract (stairs gait 0.15 m/s).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from sim_full_course.terrain import Terrain

GAIT_VX_MAX = {"flat": 0.20, "stairs": 0.15}


@dataclass
class RobotLimits:
    vy_max: float = 0.05
    wz_max: float = 0.20
    vx_min: float = 0.0  # native boundary rejects obvious reversing
    slew_vx: float = 0.2  # m/s^2
    slew_vy: float = 0.1
    slew_wz: float = 0.3  # rad/s^2
    gait_switch_time: float = 3.0  # s, stationary
    stationary_speed: float = 0.01
    step_limit: dict = field(default_factory=lambda: {"flat": 0.10, "stairs": 0.22})
    footprint_length: float = 0.9
    footprint_width: float = 0.5


@dataclass
class RobotState:
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    gait: str = "flat"
    z: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0


def _slew(cur, target, rate, dt):
    return cur + float(np.clip(target - cur, -rate * dt, rate * dt))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class KinematicRobot:
    def __init__(self, terrain: Terrain, x, y, yaw, gait="flat", body_z_offset=0.43,
                 limits: RobotLimits | None = None):
        self.terrain = terrain
        self.lim = limits or RobotLimits()
        self.body_z_offset = body_z_offset
        self.state = RobotState(x, y, _wrap(yaw), gait=gait)
        self.switch_remaining = 0.0
        self.pending_gait: str | None = None
        self._update_attitude()
        L, W = self.lim.footprint_length, self.lim.footprint_width
        u = np.arange(-L / 2, L / 2 + 1e-9, 0.05)
        v = np.arange(-W / 2, W / 2 + 1e-9, 0.05)
        uu, vv = np.meshgrid(u, v)
        self._fp = np.column_stack([uu.ravel(), vv.ravel()])

    # ---- geometry ----
    def footprint_points(self, x=None, y=None, yaw=None):
        s = self.state
        x = s.x if x is None else x
        y = s.y if y is None else y
        yaw = s.yaw if yaw is None else yaw
        c, sn = math.cos(yaw), math.sin(yaw)
        u, v = self._fp[:, 0], self._fp[:, 1]
        return x + c * u - sn * v, y + sn * u + c * v

    def pose6(self):
        s = self.state
        return (s.x, s.y, s.z, s.roll, s.pitch, s.yaw)

    def _update_attitude(self):
        s = self.state
        z0, s.pitch, s.roll = self.terrain.ground_plane(s.x, s.y, s.yaw, self.lim.footprint_length,
                                                        self.lim.footprint_width)
        s.z = z0 + self.body_z_offset

    def check(self, x, y, yaw):
        """Footprint check at a candidate pose -> dict(collision, step, unknown_frac)."""
        px, py = self.footprint_points(x, y, yaw)
        n_coll = int(self.terrain.obstacle_at(px, py).sum())
        step = float(np.max(self.terrain.step_at(px, py)))
        unknown = float(1 - self.terrain.known_at(px, py).mean())
        return {"collision": n_coll > 0, "n_coll": n_coll, "step": step, "unknown_frac": unknown,
                "step_violation": step > self.lim.step_limit[self.state.gait]}

    # ---- stepping ----
    def request_gait(self, gait: str | None):
        if gait is None or gait == self.state.gait or gait == self.pending_gait:
            return
        if gait not in GAIT_VX_MAX:
            raise ValueError(f"unknown gait {gait}")
        self.pending_gait = gait

    @property
    def switching(self) -> bool:
        return self.pending_gait is not None

    def step(self, cmd, dt: float, speed_cap: float | None = None, block_on_collision=True):
        """cmd = (vx, vy, wz) requested. Returns dict of events for this tick."""
        s, lim = self.state, self.lim
        ev = {"gait_switched": None, "collision": False, "blocked": False}
        vx_c, vy_c, wz_c = (float(v) for v in cmd)
        if self.pending_gait is not None:
            vx_c = vy_c = wz_c = 0.0  # native: stop, switch, then continue
            moving = max(abs(s.vx), abs(s.vy), abs(s.wz)) > lim.stationary_speed
            if not moving:
                if self.switch_remaining <= 0:
                    self.switch_remaining = lim.gait_switch_time
                self.switch_remaining -= dt
                if self.switch_remaining <= 1e-9:
                    s.gait = self.pending_gait
                    ev["gait_switched"] = s.gait
                    self.pending_gait = None
                    self.switch_remaining = 0.0
        vmax = GAIT_VX_MAX[s.gait] if speed_cap is None else min(GAIT_VX_MAX[s.gait], speed_cap)
        vx_c = float(np.clip(vx_c, lim.vx_min, vmax))
        vy_c = float(np.clip(vy_c, -lim.vy_max, lim.vy_max))
        wz_c = float(np.clip(wz_c, -lim.wz_max, lim.wz_max))
        vx = _slew(s.vx, vx_c, lim.slew_vx, dt)
        vy = _slew(s.vy, vy_c, lim.slew_vy, dt)
        wz = _slew(s.wz, wz_c, lim.slew_wz, dt)
        yaw_mid = s.yaw + 0.5 * wz * dt
        c, sn = math.cos(yaw_mid), math.sin(yaw_mid)
        nx = s.x + (c * vx - sn * vy) * dt
        ny = s.y + (sn * vx + c * vy) * dt
        nyaw = _wrap(s.yaw + wz * dt)
        chk = self.check(nx, ny, nyaw)
        ev.update(chk)
        if block_on_collision and chk["n_coll"] > self.check(s.x, s.y, s.yaw)["n_coll"]:
            # Kinematic proxy for contact: a move that pushes the footprint further into an
            # obstacle is refused and the body stops (moving out of overlap is allowed).
            ev["blocked"] = True
            s.vx = s.vy = s.wz = 0.0
        else:
            s.x, s.y, s.yaw = nx, ny, nyaw
            s.vx, s.vy, s.wz = vx, vy, wz
        self._update_attitude()
        return ev
