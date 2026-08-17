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

    @property
    def tilt(self) -> float:
        """Combined lean, radians. A fall is a fall whichever way the robot went over."""
        return math.acos(max(-1.0, min(1.0, math.cos(self.pitch) * math.cos(self.roll))))


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
    verify_min_contacts: int = 3
    #: Metres a wheel centre must be past the obstacle edge to count as over it.
    verify_clearance: float = 0.0
    #: Metres of forward progress required after handing back, and the time allowed for it.
    resume_distance: float = 0.5
    resume_timeout: float = 15.0

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
    last_progress_at: float = 0.0
    entry_position: np.ndarray | None = None
    entry_obstacle_distance: float = math.inf


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
        self._verified = Debounced(c.verify_hold, 0.0)

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

    @property
    def retries(self) -> int:
        return self._retries

    def policy_for(self, segment: tuple[int, int]) -> str:
        return self.segment_policies.get(tuple(segment), "")

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

        if self._torque_violation(state, dt):
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
            if state.travelled - self._resume_from >= self.config.resume_distance:
                self._resume_deadline = math.inf
                self._last_reason = "resume confirmed"
            elif state.t >= self._resume_deadline:
                self._resume_deadline = math.inf
                # Retracted means retracted: the segment goes back to being uncleared, or the
                # retry that follows would be refused by the latch it just set.
                self._cleared.discard(tuple(state.segment))
                self._go(Mode.RECOVER, "no progress after climb; verification retracted")
                return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        name = self.policy_for(state.segment)
        near = self._near_approach.update(state.obstacle_distance)
        # The edge has to be *ahead*. `near` is a distance band and is equally satisfied by an
        # obstacle 1 m behind, which is exactly the state the robot is in immediately after a
        # successful climb. Both this and the cleared latch below are needed: the latch alone
        # would still let the router arm on an edge it had reversed past during RECOVER.
        ahead = state.obstacle_distance > -self.config.approach_behind
        done = tuple(state.segment) in self._cleared
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
        in_envelope = (
            c.ready_distance_min <= state.obstacle_distance <= c.ready_distance_max
            and abs(state.lateral_error) <= c.max_lateral_error
            and abs(state.heading_error) <= c.max_heading_error
            and state.speed <= c.max_entry_speed
            and abs(state.yaw_rate) <= c.max_entry_yaw_rate
            and state.tilt <= c.max_entry_tilt
        )
        if self._in_envelope.update(in_envelope, dt):
            self._go(Mode.CLIMB_READY, "entry envelope held")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        if self._elapsed >= c.align_timeout:
            self._go(Mode.RECOVER, f"could not align in {c.align_timeout:.0f}s")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        # Yaw first, then lateral, then close the remaining gap: correcting all three at once
        # on a lip walks the contact points along the edge, which is the failure `StepCommit`
        # was written to avoid and it applies just as well here.
        heading = float(np.clip(-state.heading_error * 1.5, -c.align_yaw_rate, c.align_yaw_rate))
        lateral = 0.0
        forward = 0.0
        if abs(state.heading_error) <= c.max_heading_error * 2.0:
            lateral = float(np.clip(-state.lateral_error * 1.2, -c.align_lateral, c.align_lateral))
            if abs(state.lateral_error) <= c.max_lateral_error * 2.0:
                gap = state.obstacle_distance - c.align_enter * 0.5
                forward = float(np.clip(gap * 0.8, -c.align_speed, c.align_speed))
        settling = " (settling)" if self._in_envelope.settling else ""
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
        )
        self._go(Mode.CLIMB, f"started {name}")
        return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

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

        elapsed = state.t - self._attempt.started_at
        if elapsed >= c.climb_timeout:
            self._cancel_active("timeout")
            self._policy_status = PolicyStatus.TIMED_OUT
            self._go(Mode.RECOVER, f"climb timed out after {elapsed:.1f}s")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        if state.travelled > self._attempt.best_travelled + c.climb_progress_epsilon:
            self._attempt.best_travelled = state.travelled
            self._attempt.last_progress_at = state.t
        elif state.t - self._attempt.last_progress_at >= c.climb_progress_window:
            self._cancel_active("no progress")
            self._go(Mode.RECOVER, f"no progress for {c.climb_progress_window:.0f}s during climb")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        try:
            action = policy.step(observation)
        except Exception as exc:  # see start; the run continues without it
            self._cancel_active("step raised")
            self._go(Mode.RECOVER, f"policy raised: {type(exc).__name__}")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        self._policy_status = action.status
        if action.status is PolicyStatus.SUCCEEDED:
            # Note what is *not* happening here: a hand back to NAVIGATE.
            self._go(Mode.VERIFY_CLEAR, "policy reports success; verifying")
            self._verified.reset(False)
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
        if action.status in (PolicyStatus.FAILED, PolicyStatus.CANCELLED, PolicyStatus.TIMED_OUT):
            self._cancel_active(action.status.value)
            self._go(Mode.RECOVER, f"policy reported {action.status.value}")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)

        if action.kind is ActionKind.TWIST:
            return RouterOutput(
                self.mode, Source.POLICY, command=action.twist or (0.0, 0.0, 0.0), reason="climbing"
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
        # obstacle_distance is measured to the base; a negative value means the base is past
        # the edge, and the wheels are checked individually against the same edge.
        past = int(
            np.sum(
                state.obstacle_distance + (wheels[:, 0] - state.position[0]) <= -c.verify_clearance
            )
        )
        if past < 4:
            return False, f"only {past}/4 wheel centres past the edge"
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
        """Hold still, check the geometry, and only then hand back."""
        ok, why = self._physically_clear(state)
        if self._verified.update(ok and state.speed <= self.config.max_entry_speed * 4.0, dt):
            self._resume_from = state.travelled
            self._resume_deadline = state.t + self.config.resume_timeout
            self._cleared.add(tuple(state.segment))
            self._cancel_active("verified")
            self._go(Mode.NAVIGATE, f"verified: {why}")
            return RouterOutput(
                self.mode, Source.NAV, command=nav_command, reason=self._last_reason
            )
        if self._elapsed >= self.config.verify_hold * 3.0:
            self._cancel_active("verification failed")
            self._go(Mode.RECOVER, f"verification failed: {why}")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
        return RouterOutput(self.mode, Source.ROUTER, reason=f"verifying: {why}")

    def _recover(self, state, nav_command, observation, dt) -> RouterOutput:
        """Back straight off, then try again -- a bounded number of times."""
        c = self.config
        if self._elapsed < c.recover_duration:
            return RouterOutput(
                self.mode, Source.ROUTER, command=(-c.recover_speed, 0.0, 0.0), reason="backing off"
            )
        self._retries += 1
        if self._retries > c.max_retries:
            self._go(Mode.ABORT, f"{self._retries - 1} attempts exhausted")
            return RouterOutput(self.mode, Source.ROUTER, reason=self._last_reason)
        self._in_envelope.reset(False)
        self._near_align.reset(False)
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
    "Source",
    "RobotState",
    "RouterConfig",
    "RouterOutput",
    "Router",
    "observation_from_state",
    "replace",
    "field",
]
