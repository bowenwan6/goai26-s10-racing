"""The strategy router: which controller owns the robot, and when it is allowed to.

There is one hard invariant and everything else serves it: **at any instant exactly one
source may write to the execution boundary.** Not "usually one"; one. The failure this
prevents is already in this project's history -- an abandoned container's follower went on
publishing ``/cmd_vel`` into the next run's simulator, and the afternoon that cost was spent
debugging terrain that was never the problem. A router that hands out authority is the same
hazard with a shorter wire, so authority is represented explicitly (:class:`Source`), is
returned with every single tick (:class:`RouterOutput`), and is asserted in the tests.

The state machine is small. What is not small is the set of ways it must refuse to proceed,
and that asymmetry is the design:

* ``NAVIGATE`` is the default and the destination. Every other state is a detour that must
  end back here or in ``ABORT``.
* ``APPROACH`` and ``ALIGN`` exist because the climb has an entry specification. Phase 2's
  measurements are blunt about this: run the same manoeuvre from a settled straddle and it
  costs 19 N.m of a 50 N.m ceiling, run it from the wedge the shipped policy leaves behind
  and the *entry transient alone* spends 85-148 N.m before the manoeuvre starts. Three of
  five wedges could not be settled into a straddle at all. ``ALIGN``'s job is therefore to
  **produce** an entry state, not to bless whatever the approach happened to leave.
* ``CLIMB_READY`` is a distinct state rather than a branch inside ``ALIGN`` because starting
  a policy is a one-shot side effect -- ``reset`` then ``start`` -- and one-shot side effects
  inside a state that is re-entered at 50 Hz get executed 50 times a second.
* ``VERIFY_CLEAR`` exists because a policy's ``SUCCEEDED`` is a claim about the policy, not a
  measurement of the robot. It is checked against geometry and contact, and it loses.
* ``RECOVER`` is bounded. An unbounded retry is how a run gets parked against an arch for
  five minutes in silence.
* ``ABORT`` is terminal and stops the robot. Reaching it is a bad outcome; reaching it
  quietly, while still commanding motion, is a worse one.

The router is deliberately free of ROS. It is a function from :class:`RobotState` to
:class:`RouterOutput` plus internal timers, which is what makes the acceptance tests able to
drive it through a fall, a stale lidar and a lying policy in a few microseconds each.
"""

from __future__ import annotations

import contextlib
import math
from dataclasses import dataclass, field, replace
from enum import Enum

import numpy as np

from s10_auto_nav.strategy.gates import Debounced, Hysteresis
from s10_auto_nav.strategy.policy import (
    ActionKind,
    ClimbPolicyAdapter,
    PolicyObservation,
    PolicyStatus,
)


class Mode(Enum):
    NAVIGATE = "navigate"
    APPROACH = "approach"
    ALIGN = "align"
    CLIMB_READY = "climb_ready"
    CLIMB = "climb"
    VERIFY_CLEAR = "verify_clear"
    HANDOFF = "handoff"
    RECOVER = "recover"
    ABORT = "abort"
    DONE = "done"


class Source(Enum):
    """Who owns the execution boundary this tick.

    ``NONE`` is not "no opinion", it is an active instruction to publish nothing, and it is
    distinct from ``ROUTER`` commanding zero. Publishing a zero twist asserts control and
    holds the robot still; publishing nothing leaves whatever was last sent in force. The
    router uses ``ROUTER`` with a zero command for every safety stop precisely because
    ``NONE`` would let a stale command keep driving.
    """

    NAV = "nav"
    POLICY = "policy"
    ROUTER = "router"
    NONE = "none"


@dataclass(frozen=True)
class RobotState:
    """Everything the router is allowed to look at.

    Optional fields are optional because the ROS side cannot always supply them -- wheel
    positions and contacts come from the simulator, not from ``/ground_truth/odom`` -- and
    the router degrades to weaker verification rather than refusing to run. Where a check
    depends on a field that is ``None``, the check *fails closed*: an unverifiable climb is
    not a verified one. See :meth:`Router._physically_clear`.
    """

    t: float
    segment: tuple[int, int]
    position: np.ndarray
    yaw: float
    pitch: float
    roll: float
    speed: float
    yaw_rate: float
    odom_time: float
    lidar_time: float
    heightmap_time: float
    forward_speed: float | None = None
    #: Distance to the entry line of the obstacle for this segment, metres. Negative once
    #: the robot is past it.
    obstacle_distance: float = math.inf
    #: Signed offset from the segment's centreline, metres, and heading error, radians.
    lateral_error: float = 0.0
    heading_error: float = 0.0
    #: Course progress, metres, monotone. Used for "is anything happening at all".
    travelled: float = 0.0
    course_finished: bool = False
    joint_positions: np.ndarray | None = None
    joint_velocities: np.ndarray | None = None
    joint_torques: np.ndarray | None = None
    wheel_positions: np.ndarray | None = None
    wheel_contacts: np.ndarray | None = None
    obstacle_edge: np.ndarray | None = None
    obstacle_normal: np.ndarray | None = None
    #: Ground/deck height at the segment's target waypoint. Segment stair policies use this
    #: only to prove that all wheels reached the upper platform; it is not an entry trigger.
    segment_target_z: float | None = None
    #: Current XY distance to the segment target. Used only for bounded summit slowdown;
    #: strict waypoint acceptance remains owned by the course/evaluator.
    segment_target_distance: float = math.inf
    #: Bearing error to the current waypoint itself, deliberately excluding any path
    #: lookahead into the following segment.
    segment_target_heading_error: float = 0.0
    actual_joint_owner: str = "unknown"

    @property
    def tilt(self) -> float:
        """Combined lean, radians. A fall is a fall whichever way the robot went over."""
        return math.acos(max(-1.0, min(1.0, math.cos(self.pitch) * math.cos(self.roll))))


def required_sensors_started(odom_time: float, lidar_time: float, heightmap_time: float) -> bool:
    """Whether every input required by normal navigation has produced at least one sample.

    A zero timestamp means "never received", not "three seconds stale".  Keeping this
    distinction outside the age watchdog lets a slow first MuJoCo model load hold a safe zero
    command without weakening the strict runtime stale-sensor timeout after startup.
    """
    return all(
        math.isfinite(timestamp) and timestamp > 0.0
        for timestamp in (odom_time, lidar_time, heightmap_time)
    )


def obstacle_coordinates(position, yaw, edge, normal, tangent) -> tuple[float, float, float]:
    """Pose in a measured obstacle frame: distance, lateral error and heading error."""
    position = np.asarray(position, dtype=float)
    edge = np.asarray(edge, dtype=float)
    normal = np.asarray(normal, dtype=float)
    tangent = np.asarray(tangent, dtype=float)
    distance = float(np.dot(edge - position, normal))
    lateral = float(np.dot(position - edge, tangent))
    normal_yaw = math.atan2(float(normal[1]), float(normal[0]))
    heading = (float(yaw) - normal_yaw + math.pi) % (2.0 * math.pi) - math.pi
    return distance, lateral, heading


@dataclass
class RouterConfig:
    """Thresholds, dwell times and limits.

    Every dwell here is a claim that a condition has to persist, and the values are chosen so
    the slowest of them is still short against the manoeuvre it gates. They are parameters
    rather than constants because the acceptance tests need to drive them to their edges.
    """

    control_rate: float = 50.0

    #: Observe without touching the robot. The state machine runs in full -- it arms, it
    #: aligns, it evaluates the entry gate and it records every transition -- but the command
    #: that leaves :meth:`Router.tick` is the follower's, unscaled, whatever mode says. This
    #: is how the pit-failure experiment measures what the router *would* have done to an
    #: unmodified production run; switching the router off instead would measure nothing.
    #: Off by default, so a scored run cannot end up in it by accident.
    shadow: bool = False
    #: Stop at CLIMB_READY without starting a policy. Test harnesses use this to measure a
    #: handoff state while guaranteeing that /JOINTS_CMD remains under official ownership.
    measurement_only: bool = False

    #: Distance at which a segment with a configured policy stops being ordinary driving.
    approach_enter: float = 3.0
    approach_exit: float = 3.6
    #: How far the edge may be *behind* the base and still count as approachable. Non-zero so
    #: that a robot part-way onto the step, where the measured distance goes slightly
    #: negative, is not thrown back to NAVIGATE mid-manoeuvre.
    approach_behind: float = 0.6
    #: ...and at which the router takes the wheel to build the entry state.
    align_enter: float = 1.2
    align_exit: float = 1.5

    #: The capture envelope. A climb may not start outside all of these, simultaneously,
    #: held for ``ready_dwell``.
    max_lateral_error: float = 0.08
    max_heading_error: float = math.radians(6.0)
    max_entry_speed: float = 0.05
    min_entry_speed: float = 0.10
    target_entry_speed: float = 0.25
    #: Calibrated official-follower command for the final moving entry. This may differ from
    #: ``target_entry_speed`` because the latter is measured obstacle-normal velocity, not
    #: the actor's command input.
    gate16_prewarm_forward: float = 0.25
    gate16_staging_lead: float = 0.25
    gate16_staging_tolerance: float = 0.10
    #: v1.5 keeps the adaptive-v3 contract as the preferred path, but may select the
    #: stable-v1 moving-entry contract at the far staging point when pose confidence is
    #: outside the fast envelope. The selected contract is fixed for the attempt.
    gate16_fallback_enabled: bool = False
    #: Keep the probability-sensitive adaptive path behind an explicit experiment switch.
    #: Competition configuration uses stable fallback for every Gate16 attempt.
    gate16_fast_adapter_enabled: bool = False
    gate16_fallback_ready_distance_min: float = 0.62
    gate16_fallback_ready_distance_max: float = 0.70
    gate16_fallback_ready_dwell: float = 0.10
    gate16_fallback_min_entry_speed: float = 0.08
    gate16_fallback_max_entry_speed: float = 0.20
    gate16_fallback_target_entry_speed: float = 0.15
    gate16_fallback_max_lateral_error: float = 0.25
    gate16_fallback_max_heading_error: float = math.radians(6.0)
    gate16_fallback_max_yaw_rate: float = 0.10
    max_entry_tilt: float = math.radians(12.0)
    #: How close the edge must be, and no closer. Without a distance band the readiness
    #: check passed anywhere inside the align radius, including 1.2 m short of the edge and
    #: including part-way onto the step, so "aligned" said nothing about where the robot was.
    #: **These two numbers are provisional experiment thresholds, not proven policy limits.**
    #: Nothing has yet measured the entry distance a climb needs; the pit-failure experiment
    #: varies them, and what it finds replaces them.
    ready_distance_min: float = 0.45
    ready_distance_max: float = 0.70
    #: A robot still turning is not settled even when its heading error happens to read zero
    #: as it swings through. ``max_entry_speed`` does not catch this: a pivot in place has no
    #: ground speed at all.
    max_entry_yaw_rate: float = 0.10
    ready_dwell: float = 0.40
    #: Losing alignment is acted on faster than gaining it; see ``Debounced``.
    unready_dwell: float = 0.15

    #: What ``ALIGN`` is allowed to command while it builds that state.
    align_speed: float = 0.25
    align_yaw_rate: float = 0.35
    align_lateral: float = 0.20
    #: Giving up on ever aligning, and dropping into RECOVER.
    align_timeout: float = 12.0

    #: Approach is ordinary navigation with a ceiling, not a router command.
    approach_speed_scale: float = 0.5

    #: A climb that has not finished by now is not going to.
    climb_timeout: float = 20.0
    #: ...nor is one that has not moved the robot at all.
    climb_progress_epsilon: float = 0.05
    climb_progress_window: float = 5.0

    #: Verification. All of these must hold together, for ``verify_hold`` seconds.
    verify_hold: float = 2.0
    verify_timeout: float = 0.0
    verify_unready_dwell: float = 0.0
    verify_min_contacts: int = 3
    #: Metres a wheel centre must be past the obstacle edge to count as over it.
    verify_clearance: float = 0.0
    verify_deck_z: float = 0.47872480
    verify_wheel_radius: float = 0.081
    verify_height_fraction: float = 0.70
    #: Metres of forward progress required after handing back, and the time allowed for it.
    resume_distance: float = 0.5
    resume_timeout: float = 15.0
    #: Canonical b824 post-climb cruise while the official follower settles back in.
    climb_exit_forward: float = 0.5
    climb_exit_duration: float = 0.8
    near_target_finish_forward: float = 0.5
    near_target_finish_duration: float = 10.0
    near_target_finish_forward_gain: float = 1.0
    near_target_finish_reverse_limit: float = 0.3
    near_target_finish_max_speed: float = 0.7
    near_target_finish_lateral_gain: float = 1.0
    near_target_finish_lateral_limit: float = 0.25
    near_target_finish_yaw_gain: float = 2.5
    near_target_finish_yaw_limit: float = 0.7

    #: Recovery.
    recover_speed: float = 0.35
    recover_duration: float = 2.0
    max_retries: int = 3

    #: Safety. Sensors older than this stop the robot; older than the abort multiple of it,
    #: and the run is over.
    sensor_timeout: float = 0.5
    sensor_abort_timeout: float = 3.0
    #: Past this the robot is falling and no controller is going to save it.
    fall_tilt: float = math.radians(60.0)
    #: Sustained torque over the ceiling. One tick is contact noise; a second of it is not.
    torque_ceiling: float = 50.0
    torque_grace: float = 1.0


@dataclass
class RouterOutput:
    """One tick of decision. Always complete, never partial."""

    mode: Mode
    source: Source
    #: Only meaningful when ``source`` is ``ROUTER``; ``NAV`` forwards the follower's command
    #: unchanged apart from ``speed_scale``, and ``POLICY`` carries the policy's action.
    #: ``None`` means this tick carries no body-velocity opinion at all, which is what a
    #: 16-joint policy produces. It is distinct from a zero twist, which is an assertion
    #: that the body should hold still. Deciding what to actually publish for ``None`` is
    #: the ROS layer's job, not this one's.
    command: tuple[float, float, float] | None = (0.0, 0.0, 0.0)
    joints: np.ndarray | None = None
    speed_scale: float = 1.0
    reason: str = ""
    transitioned: bool = False
    policy_status: PolicyStatus = PolicyStatus.IDLE


@dataclass
class _Attempt:
    """Bookkeeping for one climb attempt, reset on every retry."""

    started_at: float = 0.0
    start_travelled: float = 0.0
    best_travelled: float = 0.0
    best_physical_progress: float = 0.0
    last_progress_at: float = 0.0
    entry_position: np.ndarray | None = None
    entry_obstacle_distance: float = math.inf
    segment: tuple[int, int] = (-1, -1)
    gate16: bool = False
    gate16_entry_mode: str = ""
    delegated: bool = False
    requires_physical_clear: bool = False
    near_target_settling: bool = False


class Router:
    """The state machine. One instance per run; ``tick`` at the control rate."""

    def __init__(
        self,
        config: RouterConfig | None = None,
        policies: dict[str, ClimbPolicyAdapter] | None = None,
        segment_policies: dict[tuple[int, int], str] | None = None,
    ):
        self.config = config or RouterConfig()
        #: Named policies, and the segments that use them. The mapping lives here rather than
        #: as an ``if segment == (15, 16)`` somewhere in the follower because the moment that
        #: test appears in two files they disagree, and one of them is the one that runs.
        self.policies = dict(policies or {})
        self.segment_policies = dict(segment_policies or {})

        c = self.config
        self.mode = Mode.NAVIGATE
        self._near_approach = Hysteresis(c.approach_enter, c.approach_exit, rising=False)
        self._near_align = Hysteresis(c.align_enter, c.align_exit, rising=False)
        self._in_envelope = Debounced(c.ready_dwell, c.unready_dwell)
        self._gate16_fallback_envelope = Debounced(c.gate16_fallback_ready_dwell, c.unready_dwell)
        self._verified = Debounced(c.verify_hold, c.verify_unready_dwell)
        self._gate16_staging_complete = False
        self._gate16_prewarm_ready = False
        self._gate16_entry_mode: str | None = None

        self._entered_at = 0.0
        self._t = 0.0
        self._stale_since: float | None = None
        self._torque_over_since: float | None = None
        self._retries = 0
        self._attempt = _Attempt()
        self._active: ClimbPolicyAdapter | None = None
        self._active_name = ""
        self._policy_status = PolicyStatus.IDLE
        self._resume_deadline = math.inf
        self._resume_from = 0.0
        self._resume_position: np.ndarray | None = None
        self._climb_exit_deadline = -math.inf
        #: Segments whose obstacle has been physically verified as crossed. Without this the
        #: router re-arms on the segment it just cleared -- the edge is still *within* the
        #: approach radius after the climb, only behind instead of ahead -- and climbs the
        #: same step forever. Discarded again if the crossing is later retracted.
        self._cleared: set[tuple[int, int]] = set()
        self._last_reason = "start"
        #: Purely for the log and the tests: every transition, in order.
        self.history: list[tuple[float, Mode, Mode, str]] = []

    # ------------------------------------------------------------------ helpers

    @property
    def active_policy_name(self) -> str:
        return self._active_name

    def _near_target_finish_command(self, state: RobotState) -> tuple[float, float, float]:
        """Close a strict gate after the official actor acknowledges platform handoff."""
        heading = state.segment_target_heading_error
        distance = state.segment_target_distance
        if state.speed > self.config.near_target_finish_max_speed:
            return (0.0, 0.0, 0.0)
        forward = self.config.near_target_finish_forward_gain * distance * math.cos(heading)
        lateral = self.config.near_target_finish_lateral_gain * distance * math.sin(heading)
        return (
            float(
                np.clip(
                    forward,
                    -self.config.near_target_finish_reverse_limit,
                    self.config.near_target_finish_forward,
                )
            ),
            float(
                np.clip(
                    lateral,
                    -self.config.near_target_finish_lateral_limit,
                    self.config.near_target_finish_lateral_limit,
                )
            ),
            float(
                np.clip(
                    self.config.near_target_finish_yaw_gain * heading,
                    -self.config.near_target_finish_yaw_limit,
                    self.config.near_target_finish_yaw_limit,
                )
            ),
        )

    @property
    def active_action_kind(self) -> ActionKind | None:
        return None if self._active is None else self._active.action_kind

    @property
    def gate16_prewarm_ready(self) -> bool:
        """The official actor has reached the far staging point and aligned to the lip."""
        return self._gate16_prewarm_ready

    @property
    def gate16_entry_mode(self) -> str | None:
        """The confidence-selected Gate16 contract for this attempt."""
        return self._gate16_entry_mode

    @property
    def retries(self) -> int:
        return self._retries

    def policy_for(self, segment: tuple[int, int]) -> str:
        return self.segment_policies.get(tuple(segment), "")

    def _policy_entry(self, policy) -> tuple[float, float, float, float, float]:
        """Return command, speed bounds and distance bounds for one policy.

        Gate16 keeps the router defaults.  A compatible delegated actor may publish explicit
        bounds, but cannot widen the router's heading, lateral, attitude or freshness gates.
        """
        c = self.config
        command = float(getattr(policy, "command_forward", c.target_entry_speed))
        speed_min = float(getattr(policy, "entry_speed_min", c.min_entry_speed))
        speed_max = float(getattr(policy, "entry_speed_max", c.max_entry_speed))
        distance_min = float(getattr(policy, "ready_distance_min", c.ready_distance_min))
        distance_max = float(getattr(policy, "ready_distance_max", c.ready_distance_max))
        return command, speed_min, speed_max, distance_min, distance_max

    def _gate16_entry_command(self) -> float:
        if self._gate16_entry_mode == "stable_fallback":
            return self.config.gate16_fallback_target_entry_speed
        return self.config.target_entry_speed

    def _go(self, mode: Mode, reason: str) -> None:
        if mode is self.mode:
            return
        self.history.append((self._t, self.mode, mode, reason))
        self.mode = mode
        self._entered_at = self._t
        self._last_reason = reason

    @property
    def _elapsed(self) -> float:
        return self._t - self._entered_at

    def _cancel_active(self, reason: str) -> None:
        """Stop whatever is running. Safe to call when nothing is."""
        if self._active is not None:
            # A broken policy must not break the router on the way out.
            with contextlib.suppress(Exception):
                self._active.cancel()
        self._active = None
        self._active_name = ""
        self._policy_status = PolicyStatus.IDLE
        self._last_reason = reason

    # ------------------------------------------------------------------ safety

    def _sensor_age(self, state: RobotState) -> float:
        if self._attempt.gate16 and self.mode in (Mode.CLIMB, Mode.VERIFY_CLEAR):
            # Gate16 consumes the calibrated SDK state and the height map. Lidar remains a
            # navigation input, but it is intentionally preempted during this low-level
            # manoeuvre and must not cancel a valid climb because one scan was delayed.
            return max(state.t - state.odom_time, state.t - state.heightmap_time)
        if self._attempt.delegated and self.mode is Mode.CLIMB:
            # The in-process 57D stair actor consumes the current SDK robot state and no
            # height map.  Lidar and height-map delays must not revoke its ownership, while
            # odometry still gives the router an independent stale-state stop condition.
            return state.t - state.odom_time
        return max(
            state.t - state.odom_time, state.t - state.lidar_time, state.t - state.heightmap_time
        )

    def _torque_violation(self, state: RobotState, dt: float) -> bool:
        """Sustained torque over the ceiling, not a single spike.

        Contacts in this simulator are 5 ms springs, so a robot standing perfectly still
        still chatters and a one-tick test would fire constantly. The grace period is what
        makes this a measurement of the controller rather than of the contact model.
        """
        if state.joint_torques is None:
            self._torque_over_since = None
            return False
        over = float(np.abs(np.asarray(state.joint_torques)).max()) > self.config.torque_ceiling
        if not over:
            self._torque_over_since = None
            return False
        if self._torque_over_since is None:
            self._torque_over_since = state.t
        return state.t - self._torque_over_since >= self.config.torque_grace

    # ------------------------------------------------------------------ the tick

    def tick(
        self,
        state: RobotState,
        nav_command: tuple[float, float, float],
        observation: PolicyObservation | None = None,
    ) -> RouterOutput:
        """Advance one control step and say who drives.

        ``nav_command`` is what the follower would like to publish. The router either
        forwards it, scales it, replaces it, or ignores it -- but it is always passed in, so
        there is never a moment where the follower is publishing on its own authority.
        """
        dt = 1.0 / self.config.control_rate
        self._t = state.t
        before = self.mode

        out = self._tick_inner(state, nav_command, observation, dt)
        out.transitioned = self.mode is not before
        out.mode = self.mode
        out.policy_status = self._policy_status
        if self.config.shadow:
            # Applied here, at the one place every decision leaves, rather than inside the
            # states: a shadow check per state is a shadow check that one day gets forgotten
            # in a new one. ``speed_scale`` is reset as well -- APPROACH's 0.5 cap is a
            # change to the robot's behaviour, and an observer that halves the approach
            # speed is not observing the run it claims to be. The mode, the transition and
            # the counterfactual source survive in the output, which is the whole point.
            out = replace(
                out,
                source=Source.NAV,
                command=nav_command,
                joints=None,
                speed_scale=1.0,
                reason=f"shadow ({out.source.value}): {out.reason}",
            )
        return out

    def _tick_inner(
        self,
        state: RobotState,
        nav_command: tuple[float, float, float],
        observation: PolicyObservation | None,
        dt: float,
    ) -> RouterOutput:
        c = self.config

        if self.mode in (Mode.ABORT, Mode.DONE):
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        # --- unconditional safety, before any state logic ----------------------
        #
        # These run first and in this order because each one invalidates the reasoning of
        # everything below it. A router that decides it is aligned using a two-second-old
        # pose has not decided anything.
        age = self._sensor_age(state)
        if age > c.sensor_timeout:
            if self._stale_since is None:
                self._stale_since = state.t
            stale_for = state.t - self._stale_since
            if stale_for >= c.sensor_abort_timeout:
                self._cancel_active("sensors_lost")
                self._go(Mode.ABORT, f"sensors stale {stale_for:.1f}s")
                return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
            # A stop, not a hand-back: ``Source.NONE`` would leave the last command running.
            if self.mode is Mode.CLIMB:
                self._cancel_active("sensors_stale")
                self._go(Mode.RECOVER, f"sensors stale {age:.2f}s during climb")
            return RouterOutput(self.mode, Source.ROUTER, reason=f"sensors stale {age:.2f}s")
        self._stale_since = None

        if state.tilt >= c.fall_tilt:
            self._cancel_active("fallen")
            self._go(Mode.ABORT, f"tilt {math.degrees(state.tilt):.0f}deg")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        # This watchdog belongs to the low-level Gate 16 controller.  Applying its
        # deliberately conservative ceiling while the existing follower owns the robot
        # makes normal terrain contacts abort an otherwise healthy waypoint run.
        delegated_owns_control = (
            self._attempt.gate16 or self._attempt.delegated
        ) and self.mode in (Mode.CLIMB, Mode.VERIFY_CLEAR)
        if not delegated_owns_control:
            self._torque_over_since = None
        elif self._torque_violation(state, dt):
            self._cancel_active("torque")
            self._go(Mode.ABORT, "sustained torque over ceiling")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        if state.course_finished:
            self._cancel_active("finished")
            self._go(Mode.DONE, "course complete")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        handler = {
            Mode.NAVIGATE: self._navigate,
            Mode.APPROACH: self._approach,
            Mode.ALIGN: self._align,
            Mode.CLIMB_READY: self._climb_ready,
            Mode.CLIMB: self._climb,
            Mode.VERIFY_CLEAR: self._verify_clear,
            Mode.HANDOFF: self._handoff,
            Mode.RECOVER: self._recover,
        }[self.mode]
        return handler(state, nav_command, observation, dt)

    # ------------------------------------------------------------------ states

    def _navigate(self, state, nav_command, observation, dt) -> RouterOutput:
        # The post-climb progress requirement is enforced here rather than inside
        # VERIFY_CLEAR because making the progress requires driving, and driving is
        # NAVIGATE's job. Verification that ends the moment the robot is static would pass a
        # robot balanced on the lip with nowhere to go.
        if self._resume_deadline < math.inf:
            physical_resume = (
                0.0
                if self._resume_position is None
                else float(
                    np.linalg.norm(
                        np.asarray(state.position[:2], float) - self._resume_position[:2]
                    )
                )
            )
            if (
                state.travelled - self._resume_from >= self.config.resume_distance
                or physical_resume >= self.config.resume_distance
            ):
                self._resume_deadline = math.inf
                self._resume_position = None
                self._last_reason = "resume confirmed"
            elif state.t >= self._resume_deadline:
                self._resume_deadline = math.inf
                self._resume_position = None
                # Retracted means retracted: the segment goes back to being uncleared, or the
                # retry that follows would be refused by the latch it just set.
                self._cleared.discard(tuple(self._attempt.segment))
                self._go(Mode.RECOVER, "no progress after climb; verification retracted")
                return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        if state.t < self._climb_exit_deadline:
            if self._attempt.near_target_settling and tuple(state.segment) == tuple(
                self._attempt.segment
            ):
                return RouterOutput(
                    self.mode,
                    Source.ROUTER,
                    command=self._near_target_finish_command(state),
                    reason="closing strict waypoint after platform handoff",
                )
            if self._attempt.near_target_settling:
                self._climb_exit_deadline = -math.inf
                return RouterOutput(
                    self.mode,
                    Source.NAV,
                    command=nav_command,
                    reason="near-target gate completed; follower resumed",
                )
            return RouterOutput(
                self.mode,
                Source.NAV,
                command=(self.config.climb_exit_forward, 0.0, 0.0),
                reason="canonical 0.5 m/s climb exit",
            )

        name = self.policy_for(state.segment)
        policy = self.policies.get(name)
        delegated = bool(
            policy is not None and getattr(policy, "action_kind", None) is ActionKind.DELEGATED
        )
        if delegated and state.actual_joint_owner != "official":
            return RouterOutput(
                self.mode,
                Source.NAV,
                command=nav_command,
                reason=f"waiting for official owner ({state.actual_joint_owner})",
            )
        segment_owned = bool(getattr(policy, "owns_entire_segment", False))
        done = tuple(state.segment) in self._cleared
        ready_to_start = bool(
            policy is not None
            and (not hasattr(policy, "ready_to_start") or policy.ready_to_start(state))
        )
        if segment_owned and ready_to_start and not done:
            self._retries = 0
            self._go(Mode.ALIGN, f"segment {state.segment} uses {name} from its start")
            return RouterOutput(
                self.mode,
                Source.NAV,
                command=nav_command,
                speed_scale=self.config.approach_speed_scale,
                reason=self._last_reason,
            )
        near = self._near_approach.update(state.obstacle_distance)
        # The edge has to be *ahead*. `near` is a distance band and is equally satisfied by an
        # obstacle 1 m behind, which is exactly the state the robot is in immediately after a
        # successful climb. Both this and the cleared latch below are needed: the latch alone
        # would still let the router arm on an edge it had reversed past during RECOVER.
        ahead = state.obstacle_distance > -self.config.approach_behind
        if name and name in self.policies and near and ahead and not done:
            self._retries = 0
            self._go(Mode.APPROACH, f"segment {state.segment} uses {name}")
            return RouterOutput(
                self.mode,
                Source.NAV,
                command=nav_command,
                speed_scale=self.config.approach_speed_scale,
                reason=self._last_reason,
            )
        return RouterOutput(self.mode, Source.NAV, command=nav_command, reason="navigating")

    def _approach(self, state, nav_command, observation, dt) -> RouterOutput:
        if not self._near_approach.update(state.obstacle_distance):
            self._go(Mode.NAVIGATE, "obstacle no longer ahead")
            return RouterOutput(
                self.mode, Source.NAV, command=nav_command, reason=self._last_reason
            )
        if self._near_align.update(state.obstacle_distance):
            self._gate16_staging_complete = False
            self._gate16_prewarm_ready = False
            self._gate16_entry_mode = None
            self._in_envelope.reset(False)
            self._gate16_fallback_envelope.reset(False)
            self._go(Mode.ALIGN, "within align radius")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
        return RouterOutput(
            self.mode,
            Source.NAV,
            command=nav_command,
            speed_scale=self.config.approach_speed_scale,
            reason="approaching under speed cap",
        )

    def _align(self, state, nav_command, observation, dt) -> RouterOutput:
        """Build the entry state, and refuse to hand over until it exists.

        This is the state phase 2 says the whole climb depends on, so it does not merely
        wait for good numbers -- it drives toward them, and it gives up rather than waiting
        forever. Three of the five wedges the shipped policy produces could not be settled
        into a straddle at all, which is exactly the case ``align_timeout`` exists for.
        """
        c = self.config
        name = self.policy_for(state.segment)
        policy = self.policies.get(name)
        gate16 = bool(getattr(policy, "is_gate16_policy", False))
        segment_owned = bool(getattr(policy, "owns_entire_segment", False))
        requires_moving_entry = bool(getattr(policy, "requires_moving_entry", gate16))
        command_forward, entry_speed_min, entry_speed_max, ready_min, ready_max = (
            self._policy_entry(policy)
        )
        if segment_owned:
            start_from_rest = bool(getattr(policy, "start_from_rest", False))
            measured_forward = state.speed if state.forward_speed is None else state.forward_speed
            heading_aligned = abs(state.heading_error) <= c.max_heading_error
            lateral_aligned = abs(state.lateral_error) <= c.max_lateral_error * 2.0
            in_envelope = (
                lateral_aligned
                and heading_aligned
                and (start_from_rest or entry_speed_min <= measured_forward <= entry_speed_max)
                and abs(state.yaw_rate) <= c.max_entry_yaw_rate
                and state.tilt <= c.max_entry_tilt
            )
            if self._in_envelope.update(in_envelope, dt):
                self._go(Mode.CLIMB_READY, "segment-policy entry envelope held")
                return RouterOutput(
                    self.mode,
                    Source.ROUTER,
                    command=(command_forward, 0.0, 0.0),
                    reason=self._last_reason,
                )
            if self._elapsed >= c.align_timeout:
                self._go(Mode.RECOVER, f"could not align in {c.align_timeout:.0f}s")
                return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
            yaw = float(np.clip(-state.heading_error * 2.0, -c.align_yaw_rate, c.align_yaw_rate))
            if start_from_rest:
                # A segment test can spawn with 10--20 degrees of yaw error while the robot
                # stands up. Correct that well before the first riser without consuming the
                # stair runway. Once inside the heading gate, stop steering and let yaw rate
                # settle before handing over the actor.
                forward = 0.0 if not heading_aligned else command_forward
                yaw = yaw if not heading_aligned else 0.0
            else:
                forward = command_forward
            return RouterOutput(
                self.mode,
                Source.ROUTER,
                command=(forward, 0.0, yaw),
                reason="aligning for stairs_stable segment ownership",
            )
        entry_speed = state.speed if state.forward_speed is None else state.forward_speed
        fast_in_envelope = (
            (not gate16 or not c.gate16_fallback_enabled or c.gate16_fast_adapter_enabled)
            and ready_min <= state.obstacle_distance <= ready_max
            and abs(state.lateral_error) <= c.max_lateral_error
            and abs(state.heading_error) <= c.max_heading_error
            and (
                entry_speed_min <= entry_speed <= entry_speed_max
                if requires_moving_entry
                else state.speed <= entry_speed_max
            )
            and abs(state.yaw_rate) <= c.max_entry_yaw_rate
            and state.tilt <= c.max_entry_tilt
        )
        fast_ready = self._in_envelope.update(fast_in_envelope, dt)
        fallback_in_envelope = bool(
            requires_moving_entry
            and c.gate16_fallback_enabled
            and c.gate16_fallback_ready_distance_min
            <= state.obstacle_distance
            <= c.gate16_fallback_ready_distance_max
            and abs(state.lateral_error) <= c.gate16_fallback_max_lateral_error
            and abs(state.heading_error) <= c.gate16_fallback_max_heading_error
            and c.gate16_fallback_min_entry_speed
            <= entry_speed
            <= c.gate16_fallback_max_entry_speed
            and abs(state.yaw_rate) <= c.gate16_fallback_max_yaw_rate
            and state.tilt <= c.max_entry_tilt
        )
        fallback_ready = self._gate16_fallback_envelope.update(fallback_in_envelope, dt)

        # Prefer the fast contract whenever both are possible. Once one is selected, do not
        # oscillate between speed targets while crossing the final 30 cm of runway.
        if requires_moving_entry and self._gate16_entry_mode is None:
            if fast_ready:
                self._gate16_entry_mode = "fast_profile"
            elif fallback_ready:
                self._gate16_entry_mode = "stable_fallback"
        entry_ready = fast_ready if self._gate16_entry_mode != "stable_fallback" else fallback_ready
        if entry_ready:
            entry_mode = self._gate16_entry_mode or "default"
            self._go(Mode.CLIMB_READY, f"{entry_mode} entry envelope held")
            # Preserve the frozen moving handoff on the transition tick. A command-less
            # RouterOutput becomes a zero Twist in the ROS adapter, which previously made
            # the runner select its unvalidated frozen-default profile at exactly the lip.
            return RouterOutput(
                self.mode,
                Source.ROUTER,
                command=(
                    self._gate16_entry_command() if gate16 else command_forward,
                    0.0,
                    0.0,
                ),
                reason=self._last_reason,
            )

        if self._elapsed >= c.align_timeout:
            self._go(Mode.RECOVER, f"could not align in {c.align_timeout:.0f}s")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        # WP15 approaches Gate 16 with about half a metre of lateral error. Correct x/y/yaw
        # together while still far from the lip, using forward curvature rather than the
        # ineffective lateral channel. An in-place yaw at the staging point made the wheeled
        # base orbit by more than a metre in the failed full-stack trial. The
        # Gate16 actor is requested only inside the configured staging band, after yaw and
        # lateral error are already small. The adaptive-v3 deployment uses about 1.50 m so
        # its recurrent-by-history 174D input settles before residual arm.
        if gate16 and not self._gate16_prewarm_ready:
            if state.obstacle_edge is None or state.obstacle_normal is None:
                self._go(Mode.RECOVER, "Gate16 staging has no obstacle frame")
                return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
            staging_distance = c.ready_distance_max + c.gate16_staging_lead
            distance_error = state.obstacle_distance - staging_distance
            staged = abs(distance_error) <= c.gate16_staging_tolerance
            fast_aligned = (
                (not c.gate16_fallback_enabled or c.gate16_fast_adapter_enabled)
                and abs(state.heading_error) <= c.max_heading_error
                and abs(state.lateral_error) <= c.max_lateral_error
            )
            fallback_aligned = bool(
                c.gate16_fallback_enabled
                and abs(state.heading_error) <= c.gate16_fallback_max_heading_error
                and abs(state.lateral_error) <= c.gate16_fallback_max_lateral_error
            )
            if staged and (fast_aligned or fallback_aligned):
                self._gate16_staging_complete = True
                self._gate16_prewarm_ready = True
                self._gate16_entry_mode = "fast_profile" if fast_aligned else "stable_fallback"
            else:
                # The official wheeled actor does not translate sideways reliably. Capture
                # the centreline as a forward arc instead: steer into the cross-track error
                # while far away. Do not taper the curve merely because x is close: the
                # first trial did that with 0.24 m of cross-track error left and then had no
                # kinematic way to remove it. Capture lateral position first; straighten
                # only after it is inside the staging tolerance.
                # WP15 releases the Gate16 segment only 1.67 m from the lip with about
                # 0.55 m cross-track error. Capture that large error decisively, then use a
                # one-metre lookahead below 0.15 m so yaw and lateral error converge
                # together instead of holding a 30--45 deg arc until the last 60 cm.
                if abs(state.lateral_error) > 0.15:
                    lookahead = max(0.30, 0.40 * max(0.0, distance_error))
                else:
                    lookahead = 1.0
                raw_capture_heading = float(
                    np.clip(
                        -math.atan2(state.lateral_error, lookahead),
                        -math.radians(45.0),
                        math.radians(45.0),
                    )
                )
                # The Gate16 actor's lateral command did not measurably remove a 9 cm
                # cross-track error in the focused full-stack run.  Capture the strict
                # 8 cm entry bound while the official actor still owns the wheels; the
                # policy warm-up runway is for speed/yaw settling, not lane acquisition.
                lateral_aligned = abs(state.lateral_error) <= c.max_lateral_error
                desired_heading = 0.0 if lateral_aligned else raw_capture_heading
                heading_delta = (desired_heading - state.heading_error + math.pi) % (
                    2.0 * math.pi
                ) - math.pi
                heading = float(np.clip(heading_delta * 3.0, -c.align_yaw_rate, c.align_yaw_rate))
                heading_aligned = abs(state.heading_error) <= c.max_heading_error
                if not lateral_aligned:
                    # 0.12 m/s was below the official actor's effective locomotion range in
                    # the measured trial (actual speed collapsed to ~0.04 m/s). 0.25 m/s is
                    # both the tested policy command and enough to keep the wheels rolling.
                    forward = (
                        c.align_speed
                        if distance_error >= -c.gate16_staging_tolerance
                        else -min(c.align_speed, 0.15)
                    )
                elif not heading_aligned:
                    # Straighten while rolling.  The competition handoff explicitly
                    # forbids stopping at the lip for an in-place turn, and the measured
                    # official actor also orbited laterally when commanded to do so.
                    forward = c.align_speed
                elif distance_error > c.gate16_staging_tolerance:
                    forward = c.align_speed
                elif distance_error < -c.gate16_staging_tolerance:
                    forward = -min(c.align_speed, 0.15)
                else:
                    # Position is in-band but yaw/lateral is still settling. This is well
                    # before the lip and prevents alignment from consuming the runway
                    # reserved for Gate16 base warmup.
                    forward = 0.0
                return RouterOutput(
                    self.mode,
                    Source.ROUTER,
                    command=(forward, 0.0, heading),
                    reason=(
                        "far-field Gate16 alignment "
                        f"(d={state.obstacle_distance:.2f}m, "
                        f"lat={state.lateral_error:+.2f}m, "
                        f"yaw={math.degrees(state.heading_error):+.1f}deg)"
                    ),
                )

        # Yaw first, then lateral, then establish the moving Gate16 entry. Correcting all
        # three at once on a lip walks the contact points along the edge, which is the failure
        # `StepCommit`
        # was written to avoid and it applies just as well here.
        fallback_selected = self._gate16_entry_mode == "stable_fallback"
        heading_tolerance = (
            c.gate16_fallback_max_heading_error if fallback_selected else c.max_heading_error
        )
        lateral_tolerance = (
            c.gate16_fallback_max_lateral_error if fallback_selected else c.max_lateral_error
        )
        ready_distance_min = (
            c.gate16_fallback_ready_distance_min if fallback_selected else c.ready_distance_min
        )
        heading = float(np.clip(-state.heading_error * 1.5, -c.align_yaw_rate, c.align_yaw_rate))
        lateral = 0.0
        forward = 0.0
        if abs(state.heading_error) <= heading_tolerance * 2.0:
            lateral = float(np.clip(-state.lateral_error * 1.2, -c.align_lateral, c.align_lateral))
            if abs(state.lateral_error) <= lateral_tolerance * 2.0:
                if state.obstacle_distance < ready_distance_min:
                    entry_command = self._gate16_entry_command() if gate16 else command_forward
                    forward = -min(c.align_speed, entry_command)
                elif requires_moving_entry:
                    forward = self._gate16_entry_command() if gate16 else command_forward
                else:
                    gap = state.obstacle_distance - c.align_enter * 0.5
                    forward = float(np.clip(gap * 0.8, -c.align_speed, c.align_speed))
        active_envelope = self._gate16_fallback_envelope if fallback_selected else self._in_envelope
        settling = " (settling)" if active_envelope.settling else ""
        return RouterOutput(
            self.mode,
            Source.ROUTER,
            command=(forward, lateral, heading),
            reason="aligning" + settling,
        )

    def _climb_ready(self, state, nav_command, observation, dt) -> RouterOutput:
        """Reset and start the policy. Exactly once, which is why this is its own state."""
        if self.config.measurement_only:
            return RouterOutput(
                self.mode,
                Source.ROUTER,
                command=(0.0, 0.0, 0.0),
                reason="measurement-only handoff reached",
            )
        name = self.policy_for(state.segment)
        policy = self.policies.get(name)
        if policy is None:
            self._go(Mode.NAVIGATE, f"no policy for segment {state.segment}")
            return RouterOutput(
                self.mode, Source.NAV, command=nav_command, reason=self._last_reason
            )
        if observation is None:
            self._go(Mode.RECOVER, "no observation to start the policy from")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        try:
            policy.reset()
            policy.start(observation)
        except Exception as exc:  # a policy that throws is a failed attempt
            self._cancel_active("start raised")
            self._go(Mode.RECOVER, f"policy {name} raised on start: {type(exc).__name__}")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        self._active = policy
        self._active_name = name
        self._policy_status = PolicyStatus.RUNNING
        self._attempt = _Attempt(
            started_at=state.t,
            start_travelled=state.travelled,
            best_travelled=state.travelled,
            last_progress_at=state.t,
            entry_position=np.asarray(state.position, float).copy(),
            entry_obstacle_distance=state.obstacle_distance,
            segment=tuple(state.segment),
            gate16=bool(getattr(policy, "is_gate16_policy", False)),
            gate16_entry_mode=self._gate16_entry_mode or "default",
            delegated=getattr(policy, "action_kind", None) is ActionKind.DELEGATED,
            requires_physical_clear=bool(getattr(policy, "requires_physical_clear", False)),
        )
        mode_suffix = f" ({self._gate16_entry_mode})" if self._gate16_entry_mode else ""
        self._go(Mode.CLIMB, f"started {name}{mode_suffix}")
        # The delegated SDK actor starts on this same tick. Keep its command/profile input
        # at the selected moving-entry speed instead of injecting a one-frame stop.
        command_forward, _, _, _, _ = self._policy_entry(policy)
        return RouterOutput(
            self.mode,
            Source.ROUTER,
            command=(
                self._gate16_entry_command() if self._attempt.gate16 else command_forward,
                0.0,
                0.0,
            ),
            reason=self._last_reason,
        )

    def _climb(self, state, nav_command, observation, dt) -> RouterOutput:
        """The policy drives. The router only watches for the ways it can go wrong.

        Navigation is not merely ignored here, it is denied the boundary: ``Source.POLICY``
        is returned on every tick including the ones where the policy produces nothing, so
        there is no window in which the follower's command could reach the robot.
        """
        c = self.config
        policy = self._active
        if policy is None or observation is None:
            self._go(Mode.RECOVER, "climb without an active policy")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        if tuple(state.segment) != tuple(self._attempt.segment):
            completed = tuple(self._attempt.segment)
            self._cleared.add(completed)
            next_name = self.policy_for(state.segment)
            next_ready = bool(
                next_name == self._active_name
                and (not hasattr(policy, "ready_to_start") or policy.ready_to_start(state))
            )
            if bool(getattr(policy, "owns_entire_segment", False)) and next_ready:
                self._attempt.started_at = state.t
                self._attempt.start_travelled = state.travelled
                self._attempt.best_travelled = state.travelled
                self._attempt.last_progress_at = state.t
                self._attempt.segment = tuple(state.segment)
                if hasattr(policy, "continue_segment"):
                    policy.continue_segment()
            else:
                if hasattr(policy, "succeed"):
                    policy.succeed(f"strict waypoint completed segment {completed}")
                delegated = self._attempt.delegated
                self._cancel_active("strict target waypoint reached")
                if delegated:
                    self._go(Mode.HANDOFF, "stairs segment complete; returning official owner")
                    return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
                self._go(Mode.NAVIGATE, "segment policy complete")
                return RouterOutput(
                    self.mode, Source.NAV, command=nav_command, reason=self._last_reason
                )

        if hasattr(policy, "completion_candidate") and not bool(
            getattr(policy, "handoff_requires_strict_target", False)
        ):
            complete, why = policy.completion_candidate(
                state,
                dt,
                self._attempt.entry_position,
            )
            if complete:
                completed = tuple(self._attempt.segment)
                self._cleared.add(completed)
                self._resume_from = state.travelled
                self._resume_position = np.asarray(state.position, float).copy()
                self._resume_deadline = state.t + self.config.resume_timeout
                if hasattr(policy, "succeed"):
                    policy.succeed(why)
                self._cancel_active("upper platform verified")
                self._go(
                    Mode.HANDOFF,
                    f"stairs segment {completed} clear: {why}; returning official owner",
                )
                return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        elapsed = state.t - self._attempt.started_at
        climb_timeout = float(getattr(policy, "climb_timeout", c.climb_timeout))
        if elapsed >= climb_timeout:
            self._cancel_active("timeout")
            self._policy_status = PolicyStatus.TIMED_OUT
            self._go(Mode.RECOVER, f"climb timed out after {elapsed:.1f}s")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        physical_progress = 0.0
        if (
            self._attempt.gate16
            and self._attempt.entry_position is not None
            and state.obstacle_normal is not None
        ):
            physical_progress = float(
                np.dot(
                    np.asarray(state.position[:2], float) - self._attempt.entry_position[:2],
                    np.asarray(state.obstacle_normal, float),
                )
            )
        elif (
            bool(getattr(policy, "is_stairs_stable_policy", False))
            and self._attempt.entry_position is not None
        ):
            # /nav/progress is a waypoint-completion fraction, not metres travelled. It
            # remains constant throughout a stair segment and used to cancel a robot that
            # was visibly climbing after exactly one progress window. For stairs, measure
            # physical displacement from the ownership point; the monotone best below
            # rejects oscillation while allowing continuous rolling over many treads.
            physical_progress = float(
                np.linalg.norm(
                    np.asarray(state.position[:2], float) - self._attempt.entry_position[:2]
                )
            )
        if (
            state.travelled > self._attempt.best_travelled + c.climb_progress_epsilon
            or physical_progress > self._attempt.best_physical_progress + c.climb_progress_epsilon
        ):
            self._attempt.best_travelled = max(self._attempt.best_travelled, state.travelled)
            self._attempt.best_physical_progress = max(
                self._attempt.best_physical_progress, physical_progress
            )
            self._attempt.last_progress_at = state.t
        progress_window = float(getattr(policy, "climb_progress_window", c.climb_progress_window))
        if state.t - self._attempt.last_progress_at >= progress_window:
            self._cancel_active("no progress")
            self._go(Mode.RECOVER, f"no progress for {progress_window:.0f}s during climb")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        try:
            action = policy.step(observation)
        except Exception as exc:  # see start; the run continues without it
            self._cancel_active("step raised")
            self._go(Mode.RECOVER, f"policy raised: {type(exc).__name__}")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        self._policy_status = action.status
        if bool(getattr(policy, "is_stairs_stable_policy", False)):
            if (
                not self._attempt.near_target_settling
                and hasattr(policy, "ready_to_settle")
                and policy.ready_to_settle(state, dt)
            ):
                self._attempt.near_target_settling = True
            if self._attempt.near_target_settling:
                completed = tuple(self._attempt.segment)
                self._cleared.add(completed)
                if hasattr(policy, "succeed"):
                    policy.succeed("near-target upper platform verified")
                self._cancel_active("near-target upper platform verified")
                self._go(
                    Mode.HANDOFF,
                    f"stairs segment {completed} reached upper platform; returning official owner",
                )
                return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
        if self._attempt.requires_physical_clear:
            clear, why = self._physically_clear(state)
            if clear:
                self._go(Mode.VERIFY_CLEAR, f"four-wheel candidate: {why}")
                self._verified.reset(False)
                return RouterOutput(
                    self.mode,
                    Source.POLICY,
                    command=action.twist or (0.0, 0.0, 0.0),
                    reason=self._last_reason,
                )
        if action.status is PolicyStatus.SUCCEEDED:
            # Note what is *not* happening here: a hand back to NAVIGATE.
            self._go(Mode.VERIFY_CLEAR, "policy reports success; verifying")
            self._verified.reset(False)
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
        if action.status in (PolicyStatus.FAILED, PolicyStatus.CANCELLED, PolicyStatus.TIMED_OUT):
            self._cancel_active(action.status.value)
            self._go(Mode.RECOVER, f"policy reported {action.status.value}")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        if action.kind in (ActionKind.TWIST, ActionKind.DELEGATED):
            command = action.twist or (0.0, 0.0, 0.0)
            if (
                action.kind is ActionKind.DELEGATED
                and bool(getattr(policy, "is_stairs_stable_policy", False))
                and policy.config.navigation_yaw_rate_limit > 0.0
            ):
                # Correction-capable checkpoints can follow a nearby centreline point, reuse
                # the official follower's live steering, or aim at the current strict target.
                # Learned joints remain the sole actuator owner while the router feeds them
                # bounded Q/E-like body commands.
                yaw_limit = policy.config.navigation_yaw_rate_limit
                lateral_limit = policy.config.navigation_lateral_limit
                if policy.config.navigation_steering_source == "follower":
                    lateral_command = nav_command[1]
                    yaw_command = nav_command[2]
                elif policy.config.navigation_steering_source == "target":
                    lateral_command = 0.0
                    if policy.config.navigation_target_lateral_gain > 0.0 and math.isfinite(
                        state.segment_target_distance
                    ):
                        lateral_command = (
                            policy.config.navigation_target_lateral_gain
                            * state.segment_target_distance
                            * math.sin(state.segment_target_heading_error)
                        )
                    yaw_command = (
                        policy.config.navigation_target_yaw_gain
                        * state.segment_target_heading_error
                    )
                else:
                    lateral_command = -state.lateral_error
                    desired_heading_error = math.atan2(
                        -state.lateral_error,
                        policy.config.navigation_lookahead,
                    )
                    yaw_command = desired_heading_error - state.heading_error
                command = (
                    command[0],
                    float(
                        np.clip(
                            lateral_command,
                            -lateral_limit,
                            lateral_limit,
                        )
                    ),
                    float(np.clip(yaw_command, -yaw_limit, yaw_limit)),
                )
            if (
                action.kind is ActionKind.DELEGATED
                and bool(getattr(policy, "is_stairs_stable_policy", False))
                and policy.config.summit_slowdown_distance > 0.0
                and math.isfinite(state.segment_target_distance)
            ):
                fraction = float(
                    np.clip(
                        state.segment_target_distance / policy.config.summit_slowdown_distance,
                        0.0,
                        1.0,
                    )
                )
                slowed_forward = policy.config.summit_command_forward + fraction * (
                    command[0] - policy.config.summit_command_forward
                )
                command = (slowed_forward, command[1], command[2])
            policy_detail = ""
            if action.kind is ActionKind.DELEGATED and action.info:
                policy_detail = (
                    f"; entry_mode={action.info.get('entry_mode', 'unknown')}"
                    f" profile={action.info.get('profile', 'unknown')}"
                    f" phase={action.info.get('phase', 'unknown')}"
                )
            return RouterOutput(
                self.mode,
                Source.POLICY,
                command=command,
                reason=(
                    "climbing"
                    if action.kind is ActionKind.TWIST
                    else (f"climbing (actual owner {state.actual_joint_owner}{policy_detail})")
                ),
            )
        # No twist: a joint action is not a body velocity and must not be turned into one.
        return RouterOutput(
            self.mode, Source.POLICY, command=None, joints=action.joints, reason="climbing"
        )

    def _physically_clear(self, state: RobotState) -> tuple[bool, str]:
        """Did the robot actually get over the thing? Measured, not asked.

        Fails closed on missing data. If the wheel positions are not available there is no
        way to know that four wheel centres cleared the edge, and "we could not check" must
        not read the same as "it is fine" -- that is precisely the substitution this whole
        state exists to prevent.
        """
        c = self.config
        if state.wheel_positions is None or state.wheel_contacts is None:
            return False, "no wheel state; cannot verify"
        wheels = np.asarray(state.wheel_positions, float)
        contacts = np.asarray(state.wheel_contacts, bool)
        if wheels.shape != (4, 3) or contacts.shape != (4,):
            return False, "malformed wheel state"
        if state.obstacle_edge is None or state.obstacle_normal is None:
            if self._attempt.gate16:
                return False, "no frozen obstacle frame"
            past = int(
                np.sum(
                    state.obstacle_distance + (wheels[:, 0] - state.position[0])
                    <= -c.verify_clearance
                )
            )
        else:
            edge = np.asarray(state.obstacle_edge, float)
            normal = np.asarray(state.obstacle_normal, float)
            if edge.shape != (2,) or normal.shape != (2,) or not np.all(np.isfinite(edge)):
                return False, "malformed obstacle frame"
            forward_margin = (wheels[:, :2] - edge) @ normal - c.verify_clearance
            past = int(np.sum(forward_margin >= 0.0))
        if past < 4:
            return False, f"only {past}/4 wheel centres past the edge"
        if self._attempt.requires_physical_clear:
            deck_z = float(getattr(self._active, "verification_deck_z", c.verify_deck_z))
            deck_threshold = deck_z + c.verify_height_fraction * c.verify_wheel_radius
            high = int(np.sum(wheels[:, 2] >= deck_threshold))
            if high < 4:
                return False, f"only {high}/4 wheel centres above the deck threshold"
        touching = int(np.sum(contacts))
        if touching < c.verify_min_contacts:
            return False, f"only {touching} wheels in contact"
        if state.joint_positions is not None:
            q = np.asarray(state.joint_positions, float)
            if not np.all(np.isfinite(q)):
                return False, "non-finite joint positions"
        if state.tilt > c.max_entry_tilt * 2.0:
            return False, f"tilted {math.degrees(state.tilt):.0f}deg"
        return True, "four wheels over, contacts and attitude sane"

    def _verify_clear(self, state, nav_command, observation, dt) -> RouterOutput:
        """Keep Gate16 in control, verify geometry, then start an acknowledged handoff."""
        ok, why = self._physically_clear(state)
        if self._verified.update(ok and state.speed <= self.config.max_entry_speed * 4.0, dt):
            self._resume_from = state.travelled
            self._resume_position = np.asarray(state.position, float).copy()
            self._resume_deadline = state.t + self.config.resume_timeout
            self._cleared.add(tuple(self._attempt.segment))
            gate16 = self._attempt.gate16
            if self._active is not None and hasattr(self._active, "succeed"):
                self._active.succeed()
            self._cancel_active("verified")
            if not gate16:
                self._go(Mode.NAVIGATE, f"verified: {why}")
                return RouterOutput(
                    self.mode, Source.NAV, command=nav_command, reason=self._last_reason
                )
            self._go(Mode.HANDOFF, f"verified: {why}; returning official owner")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
        verify_timeout = max(self.config.verify_hold * 3.0, self.config.verify_timeout)
        if self._elapsed >= verify_timeout:
            self._cancel_active("verification failed")
            self._go(Mode.RECOVER, f"verification failed: {why}")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
        if self._active is None or observation is None:
            self._go(Mode.RECOVER, "verification lost active Gate16 policy")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
        try:
            action = self._active.step(observation)
        except Exception as exc:
            self._cancel_active("verification step raised")
            self._go(Mode.RECOVER, f"policy raised while verifying: {type(exc).__name__}")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
        return RouterOutput(
            self.mode,
            Source.POLICY,
            # Rear-wheel completion ends the push command. Gate16 still owns the actuator
            # matrix through this dwell, but its command observation is zero so verification
            # does not keep accelerating a robot already on the upper platform.
            command=(0.0, 0.0, 0.0) if self._attempt.gate16 else action.twist,
            reason=f"verifying under Gate16 ownership: {why}",
        )

    def _handoff(self, state, nav_command, observation, dt) -> RouterOutput:
        """Hold navigation until the SDK confirms the reset official actor is in control."""
        if state.actual_joint_owner == "official":
            if self._attempt.near_target_settling:
                self._climb_exit_deadline = state.t + self.config.near_target_finish_duration
            else:
                self._climb_exit_deadline = state.t + self.config.climb_exit_duration
            self._go(Mode.NAVIGATE, "official owner acknowledged; follower resumed")
            return RouterOutput(
                self.mode,
                (Source.ROUTER if self._attempt.near_target_settling else Source.NAV),
                command=(
                    self._near_target_finish_command(state)
                    if self._attempt.near_target_settling
                    else (self.config.climb_exit_forward, 0.0, 0.0)
                ),
                reason=self._last_reason,
            )
        if self._elapsed >= 2.0:
            self._go(Mode.ABORT, f"official handoff not acknowledged ({state.actual_joint_owner})")
        return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

    def _recover(self, state, nav_command, observation, dt) -> RouterOutput:
        """Back straight off, then try again -- a bounded number of times."""
        c = self.config
        if self._attempt.gate16:
            wheels = (
                None if state.wheel_positions is None else np.asarray(state.wheel_positions, float)
            )
            safe_lower = bool(
                wheels is not None
                and wheels.shape == (4, 3)
                and np.all(
                    wheels[:, 2]
                    < c.verify_deck_z + c.verify_height_fraction * c.verify_wheel_radius
                )
            )
            if not safe_lower:
                self._go(Mode.ABORT, "Gate16 recovery refused: robot may be straddling the lip")
                return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
        if self._elapsed < c.recover_duration:
            return RouterOutput(
                self.mode, Source.ROUTER, command=(-c.recover_speed, 0.0, 0.0), reason="backing off"
            )
        self._retries += 1
        if self._retries > c.max_retries:
            self._go(Mode.ABORT, f"{self._retries - 1} attempts exhausted")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
        self._in_envelope.reset(False)
        self._gate16_fallback_envelope.reset(False)
        self._near_align.reset(False)
        self._gate16_staging_complete = False
        self._gate16_prewarm_ready = False
        self._gate16_entry_mode = None
        self._go(Mode.ALIGN, f"retry {self._retries}/{c.max_retries}")
        return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)


def observation_from_state(state: RobotState, *, joints: int = 16) -> PolicyObservation:
    """Build a policy observation from a router state, filling what is not measured.

    A convenience for callers that have a :class:`RobotState` and nothing richer. It zero
    fills rather than inventing plausible values, because a policy fed a plausible-looking
    fabricated joint vector fails in a way that looks like a policy bug.
    """
    zeros = np.zeros(joints)
    return PolicyObservation(
        t=state.t,
        segment=tuple(state.segment),
        position=np.asarray(state.position, float),
        yaw=state.yaw,
        pitch=state.pitch,
        roll=state.roll,
        linear_velocity=np.array(
            [state.speed * math.cos(state.yaw), state.speed * math.sin(state.yaw), 0.0]
        ),
        yaw_rate=state.yaw_rate,
        joint_positions=zeros
        if state.joint_positions is None
        else np.asarray(state.joint_positions, float),
        joint_velocities=zeros
        if state.joint_velocities is None
        else np.asarray(state.joint_velocities, float),
        obstacle_delta=np.array([state.obstacle_distance, state.lateral_error, 0.0]),
        odom_time=state.odom_time,
        lidar_time=state.lidar_time,
        heightmap_time=state.heightmap_time,
        wheel_positions=state.wheel_positions,
        wheel_contacts=state.wheel_contacts,
        joint_torques=state.joint_torques,
    )


__all__ = [
    "Mode",
    "RobotState",
    "Router",
    "RouterConfig",
    "RouterOutput",
    "Source",
    "field",
    "observation_from_state",
    "obstacle_coordinates",
    "replace",
    "required_sensors_started",
]
