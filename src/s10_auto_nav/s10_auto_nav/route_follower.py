"""ROS-free route_v2 follower core: pose + height grid + scan + time -> command.

This is the entry point a simulator or the ROS node calls once per control tick::

    core = RouteFollowerCore.from_file("route_v2.json", body_z_offset=0.42, native=True)
    out = core.step(t, (x, y, z, yaw), height=grid13x9, mask=valid13x9,
                    scan_ranges=ranges72, pitch=pitch, roll=roll, current_gait="flat")
    vx, vy, wz = out.command          # body frame, m/s and rad/s
    out.gait_request                  # "flat" | "stairs" for the segment INTO the target
    out.status, out.reason            # RUNNING/DETOUR/ASTAR/GATE/BLOCKED/... see below

Ordered gating is unchanged from the legacy follower: the current waypoint must be reached
within its ``radius_xy`` and ``tol_z`` before the next one becomes the target, one per
tick. Heights in ``height`` are relative to the base (robot yaw frame, 13 x 9 X-major, as
``real_transfer.geometry.height_grid``); ``z`` is the base height; route z is ground and
``body_z_offset`` converts between them.

Status values (command is zero for every status except RUNNING/DETOUR/ASTAR/GATE; ALIGN
only turns in place):

* ``RUNNING``   tracking the centreline (``plan.status == TRACK``)
* ``DETOUR``    lateral-offset candidate chosen
* ``ASTAR``     fallback grid path
* ``ALIGN``     turning in place (zero forward) so the next frame sees down a sharp bend
* ``GATE``      final straight approach to the current waypoint
* ``BLOCKED``   no admissible local path (reason says why) -- stop, never reverse
* ``OFF_CORRIDOR`` |d| beyond corridor_half_width + margin -- stop
* ``HOLD_TERRAIN`` terrain cross-check hold (stairs evidence on a flat-gait segment)
* ``WAIT_GAIT`` ``current_gait`` given and different from the required gait
* ``STALE_INPUT`` no fresh observation within ``stale_after``
* ``DONE``      all waypoints reached
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from s10_auto_nav.pure_pursuit import Command, PurePursuitController, PursuitGains
from s10_auto_nav.route_planner import (
    LocalGrid,
    LocalGridBuilder,
    LocalGridConfig,
    Observation,
    PlanResult,
    RoutePlanner,
    RoutePlannerConfig,
)
from s10_auto_nav.route_v2 import (
    GAIT_CODES,
    CrossCheckConfig,
    CrossCheckResult,
    Projection,
    RoutePath,
    RouteTracker,
    RouteV2,
    TerrainCrossCheck,
)

MOVING = {"RUNNING", "DETOUR", "ASTAR", "GATE", "ALIGN"}


def native_gains() -> PursuitGains:
    """The native runtime's follower overrides (docs section 8.5)."""
    return PursuitGains(
        max_forward=0.20,
        max_lateral=0.05,
        max_yaw_rate=0.20,
        lookahead=0.5,
        lookahead_speed_gain=0.0,
        pivot_threshold=math.radians(10.0),
        align_falloff=math.radians(30.0),
        forward_slew=0.2,
        lateral_slew=0.1,
        yaw_slew=0.3,
    )


@dataclass
class RouteFollowerConfig:
    body_z_offset: float = 0.0
    control_rate: float = 10.0
    #: Carrot distance along the planned path; None = the controller's own
    #: ``lookahead_distance()``. 1.0 m keeps rejoin bearings small enough for the native
    #: 10 degree pivot threshold (the controller still brakes over its own 0.5 m).
    lookahead: float | None = 1.0
    #: Stop (BLOCKED, reason no_route_progress) if s has not advanced by
    #: ``progress_distance`` for this long while nominally moving. Seconds.
    no_progress_timeout: float = 20.0
    progress_distance: float = 0.10
    stale_after: float = 0.5
    #: Stop when |d| exceeds corridor_half_width + this.
    corridor_margin: float = 0.10
    tracker_back: float = 1.0
    tracker_forward: float = 2.5
    #: Projection may not run further than this past the current unscored gate.
    gate_s_slack: float = 1.0
    planner: RoutePlannerConfig = field(default_factory=RoutePlannerConfig)
    grid: LocalGridConfig = field(default_factory=LocalGridConfig)
    cross_check: CrossCheckConfig = field(default_factory=CrossCheckConfig)


@dataclass(frozen=True)
class FollowerOutput:
    command: tuple[float, float, float]
    gait_request: str | None
    gait_code: int | None
    status: str
    reason: str
    target_index: int
    target_id: str | None
    segment_id: str | None
    reached: tuple[str, ...] = ()
    s: float = float("nan")
    d: float = float("nan")
    corridor_half_width: float = float("nan")
    speed_limit: float = float("nan")
    plan: PlanResult | None = None
    terrain: CrossCheckResult | None = None
    finished: bool = False

    @property
    def moving(self) -> bool:
        return self.status in MOVING


class RouteFollowerCore:
    def __init__(
        self,
        route: RouteV2,
        config: RouteFollowerConfig | None = None,
        *,
        controller: PurePursuitController | None = None,
        course=None,
    ) -> None:
        self.route = route
        self.config = config or RouteFollowerConfig()
        self.path = RoutePath(route)
        self.tracker = RouteTracker(
            self.path, back=self.config.tracker_back, forward=self.config.tracker_forward
        )
        self.planner = RoutePlanner(self.config.planner)
        self.grid_builder = LocalGridBuilder(self.config.grid, self.config.planner.footprint)
        self.cross_check = TerrainCrossCheck(self.config.cross_check)
        self.controller = controller or PurePursuitController(native_gains())
        self._base_max_forward = self.controller.gains.max_forward
        if course is None:
            course = route.to_course(self.config.body_z_offset)
        elif len(course) != len(route.waypoints):
            raise ValueError("course and route_v2 disagree on the number of waypoints")
        self.course = course
        self._prev_choice = 0.0
        self._last_t: float | None = None
        self._last_obs_t: float | None = None
        self._progress_s: float | None = None
        self._progress_t: float | None = None
        self.last_output: FollowerOutput | None = None
        self.last_grid: LocalGrid | None = None

    @classmethod
    def from_file(
        cls, path, *, body_z_offset: float = 0.0, native: bool = True, **config_kwargs
    ) -> RouteFollowerCore:
        config = RouteFollowerConfig(body_z_offset=body_z_offset, **config_kwargs)
        controller = PurePursuitController(native_gains() if native else PursuitGains())
        return cls(RouteV2.load(path), config, controller=controller)

    # ---------------------------------------------------------------- helpers

    @property
    def cursor(self) -> int:
        return self.course.cursor

    @property
    def finished(self) -> bool:
        return self.course.finished

    def reset(self) -> None:
        self.course.reset()
        self.tracker.reset()
        self.grid_builder.reset()
        self.cross_check.reset()
        self.controller.reset()
        self.planner.reset()
        self._prev_choice = 0.0
        self._last_t = self._last_obs_t = None
        self._progress_s = self._progress_t = None

    def reset_transient(self) -> None:
        """Forget slew/choice/fusion state but keep the course cursor (router handoff)."""
        self.controller.reset()
        self.planner.reset()
        self.cross_check.reset()
        self._prev_choice = 0.0
        self._progress_s = self._progress_t = None

    def required_gait(self) -> str | None:
        if self.finished:
            return None
        return self.route.incoming_segment(self.cursor).gait

    def _stop(self, status, reason, **kw) -> FollowerOutput:
        self.controller.reset()
        self.planner.reset()
        gait = self.required_gait()
        target = None if self.finished else self.route.waypoints[self.cursor].id
        seg = None if self.finished else self.route.incoming_segment(self.cursor)
        out = FollowerOutput(
            command=(0.0, 0.0, 0.0),
            gait_request=gait,
            gait_code=None if gait is None else GAIT_CODES[gait],
            status=status,
            reason=reason,
            target_index=self.cursor,
            target_id=target,
            segment_id=None if seg is None else seg.id,
            finished=self.finished,
            **kw,
        )
        self.last_output = out
        return out

    # ------------------------------------------------------------------- step

    def observe(self, t, pose, height=None, mask=None, scan_ranges=None, scan_angles=None):
        if height is None and scan_ranges is None:
            return
        self.grid_builder.add(
            Observation(
                t=float(t),
                pose=tuple(float(v) for v in pose),
                height=None if height is None else np.asarray(height, float),
                mask=None if mask is None else np.asarray(mask, bool),
                scan_ranges=None if scan_ranges is None else np.asarray(scan_ranges, float),
                scan_angles=None if scan_angles is None else np.asarray(scan_angles, float),
            )
        )
        self._last_obs_t = float(t)

    def step(
        self,
        t: float,
        pose,
        height=None,
        mask=None,
        scan_ranges=None,
        scan_angles=None,
        *,
        pitch: float = 0.0,
        roll: float = 0.0,
        current_gait=None,
    ) -> FollowerOutput:
        cfg = self.config
        x, y, z, yaw = (float(v) for v in pose)
        if not all(math.isfinite(v) for v in (x, y, z, yaw)):
            return self._stop("STALE_INPUT", "nonfinite pose")
        dt = 1.0 / cfg.control_rate if self._last_t is None else float(t) - self._last_t
        if dt <= 0:
            dt = 1.0 / cfg.control_rate
        self._last_t = float(t)
        self.observe(t, (x, y, z, yaw), height, mask, scan_ranges, scan_angles)

        reached = []
        if self.course.update(np.array([x, y, z])):
            reached.append(self.route.waypoints[self.cursor - 1].id)
            self.controller.reset()
            self.planner.reset()
            self._prev_choice = 0.0
        if self.finished:
            return self._stop("DONE", "all waypoints reached", reached=tuple(reached))

        idx = self.cursor
        segment = self.route.incoming_segment(idx)
        gait = segment.gait
        s_gate = float(self.path.waypoint_s[idx])
        gate_xy = self.route.waypoints[idx].xy
        z_ground = z - cfg.body_z_offset
        proj: Projection = self.tracker.update(
            (x, y), z_ground, s_max=s_gate + cfg.gate_s_slack
        )
        # Corridor of the segment the robot is actually on (projection), not only the target's.
        seg_here = self.route.segments[proj.segment_index]
        width = max(seg_here.corridor_half_width, segment.corridor_half_width) if idx > 0 else segment.corridor_half_width
        common = {
            "reached": tuple(reached),
            "s": proj.s,
            "d": proj.d,
            "corridor_half_width": segment.corridor_half_width,
            "speed_limit": segment.speed_limit,
        }

        if current_gait is not None:
            code = current_gait if isinstance(current_gait, int) else GAIT_CODES.get(current_gait)
            if code != GAIT_CODES[gait]:
                return self._stop("WAIT_GAIT", f"required {gait}", **common)
        if self._last_obs_t is None or float(t) - self._last_obs_t > cfg.stale_after:
            return self._stop("STALE_INPUT", "no fresh height/scan observation", **common)
        terrain = self.cross_check.update(
            gait, dt, pitch=pitch, height=None if height is None else np.asarray(height, float),
            mask=mask,
        )
        common["terrain"] = terrain
        if proj.z_mismatch:
            return self._stop("OFF_CORRIDOR", f"z {z_ground:.2f} does not match route near s={proj.s:.1f}", **common)
        if abs(proj.d) > width + cfg.corridor_margin and idx > 0:
            return self._stop(
                "OFF_CORRIDOR", f"|d|={abs(proj.d):.2f}m > corridor {width:.2f}+{cfg.corridor_margin:.2f}", **common
            )
        if terrain.hold:
            return self._stop("HOLD_TERRAIN", terrain.reason, **common)

        grid_mode = gait
        if (gait == "stairs" and not segment.allow_detour
                and self.planner.config.stairs_trust_taught_path):
            grid_mode = "stairs_taught"
        grid = self.grid_builder.build((x, y), gait=grid_mode, now=float(t), yaw=yaw)
        self.last_grid = grid
        lookahead = cfg.lookahead if cfg.lookahead is not None else self.controller.lookahead_distance()
        plan = self.planner.plan(
            self.path, proj, (x, y, yaw), grid, segment=segment, s_gate=s_gate,
            gate_xy=gate_xy, prev_choice=self._prev_choice, lookahead=lookahead,
        )
        common["plan"] = plan
        if plan.blocked:
            self._progress_s = self._progress_t = None
            return self._stop("BLOCKED", plan.reason, **common)
        # Livelock guard: moving statuses must translate into route progress.
        if self._progress_s is None or proj.s >= self._progress_s + cfg.progress_distance or reached:
            self._progress_s, self._progress_t = proj.s, float(t)
        elif float(t) - self._progress_t > cfg.no_progress_timeout:
            return self._stop(
                "BLOCKED", f"no_route_progress for {cfg.no_progress_timeout:.0f}s", **common
            )
        if plan.status in ("TRACK", "DETOUR"):
            self._prev_choice = plan.d_target
        gains = self.controller.gains
        gains.max_forward = min(self._base_max_forward, segment.speed_limit)
        command = self.controller.compute(np.array([x, y]), yaw, plan.carrot, dt)
        scale = float(np.clip(plan.speed_scale, 0.0, 1.0))
        published = Command(command.forward * scale, command.lateral * scale, command.yaw_rate)
        # Never reverse on our own.
        forward = max(0.0, published.forward)
        status = "RUNNING" if plan.status == "TRACK" else plan.status
        out = FollowerOutput(
            command=(forward, published.lateral, published.yaw_rate),
            gait_request=gait,
            gait_code=GAIT_CODES[gait],
            status=status,
            reason=plan.reason,
            target_index=idx,
            target_id=self.route.waypoints[idx].id,
            segment_id=segment.id,
            finished=False,
            **common,
        )
        self.last_output = out
        return out
