"""The interface a climb policy has to satisfy, and nothing about how one is implemented.

The point of this module is that the router never learns which kind of policy it is driving.
A teammate's DAgger checkpoint, a hand-written trajectory and a test double all arrive here
as the same six calls, so swapping one for another is a configuration change rather than an
edit to the state machine. That constraint is worth stating because the obvious shortcut --
letting the router ask "am I running the scripted one?" -- would make every later swap a
code change, and the checkpoint does not exist yet to be swapped in.

Two facts about the interface earn their awkwardness:

``step`` returns a status alongside the action, rather than the router inferring completion
from the action going quiet. A policy that stops producing motion has not necessarily
finished; it may have jammed, and those want opposite responses.

``result`` is separate from ``is_finished`` and is allowed to say ``SUCCEEDED`` when the
robot is still in the pit. **A policy's own verdict is a claim, not a measurement.** The
router treats it as a request to check, never as an answer -- see ``VERIFY_CLEAR`` in
``router.py``, which can and does overrule it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Protocol, runtime_checkable

import numpy as np


class PolicyStatus(Enum):
    """Where a policy is in its lifecycle.

    ``TIMED_OUT`` is distinct from ``FAILED`` on purpose: a policy that ran out of time may
    have been making progress and is worth retrying from a better entry state, whereas one
    that reported failure has told us it knows it cannot. The router routes them to the same
    place today and the distinction is recorded in the transition reason, so the logs can
    tell the two apart even while the response is shared.
    """

    IDLE = "idle"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"
    TIMED_OUT = "timed_out"

    @property
    def terminal(self) -> bool:
        return self in (
            PolicyStatus.SUCCEEDED,
            PolicyStatus.FAILED,
            PolicyStatus.CANCELLED,
            PolicyStatus.TIMED_OUT,
        )


class ActionKind(Enum):
    """Which control boundary a policy writes to.

    These are not interchangeable and must not be converted into one another. A ``TWIST`` is
    a velocity request that the shipped locomotion policy turns into joint targets; a
    ``JOINT`` action *is* the joint target, 16 values, bypassing that policy entirely. There
    is no meaningful projection from 16 joint angles onto three velocities -- the whole
    reason a climb needs joint control is that the manoeuvre is not expressible as a body
    velocity -- so a helper that "converts" them would be inventing data.

    The router uses this to decide which arbiter owns the boundary, and refuses to run a
    ``JOINT`` policy alongside the shipped one.
    """

    TWIST = "twist"
    JOINT = "joint"
    #: A low-level actor running inside the SDK process. ``twist`` is the command embedded
    #: in that actor's observation, not a substitute for its actuator output.
    DELEGATED = "delegated"


@dataclass(frozen=True)
class PolicyObservation:
    """What a climb policy is given each tick.

    Deliberately a superset of what any current policy uses. The fields cost nothing to
    carry and the alternative -- growing the struct once a teammate's model turns out to
    need contact flags -- means touching the router, the node, the mocks and every test.

    Timestamps are carried rather than ages so the consumer decides what "stale" means; the
    router's threshold is not necessarily a policy's.
    """

    t: float
    segment: tuple[int, int]
    position: np.ndarray  # (3,) world
    yaw: float
    pitch: float
    roll: float
    linear_velocity: np.ndarray  # (3,) world
    yaw_rate: float
    joint_positions: np.ndarray  # (16,)
    joint_velocities: np.ndarray  # (16,)
    obstacle_delta: np.ndarray  # (3,) obstacle origin relative to the base, body frame
    odom_time: float
    lidar_time: float
    heightmap_time: float
    wheel_positions: np.ndarray | None = None  # (4, 3) world, when the sim exposes it
    wheel_contacts: np.ndarray | None = None  # (4,) bool
    joint_torques: np.ndarray | None = None  # (16,)


@dataclass
class PolicyAction:
    """One tick of output, tagged with which boundary it belongs to.

    ``twist`` and ``joints`` are mutually exclusive and the constructor enforces it rather
    than trusting callers, because the failure it prevents is two publishers fighting over
    the robot -- which costs an afternoon of debugging terrain that was never the problem.
    """

    kind: ActionKind
    status: PolicyStatus
    twist: tuple[float, float, float] | None = None  # forward, lateral, yaw_rate
    joints: np.ndarray | None = None  # (16,)
    info: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.kind is ActionKind.TWIST:
            if self.joints is not None:
                raise ValueError("a TWIST action must not carry joint targets")
            if self.twist is None and self.status is PolicyStatus.RUNNING:
                raise ValueError("a running TWIST action must carry a twist")
        elif self.kind is ActionKind.JOINT:
            if self.twist is not None:
                raise ValueError("a JOINT action must not carry a twist")
            if self.status is PolicyStatus.RUNNING and (
                self.joints is None or np.shape(self.joints) != (16,)
            ):
                raise ValueError("a running JOINT action must carry 16 joint targets")
        else:
            if self.joints is not None:
                raise ValueError("a DELEGATED action must not carry ROS joint targets")
            if self.status is PolicyStatus.RUNNING and self.twist is None:
                raise ValueError("a running DELEGATED action must carry its observation command")


@dataclass(frozen=True)
class PolicyResult:
    """What the policy claims happened. Checked, never believed."""

    status: PolicyStatus
    reason: str = ""
    elapsed: float = 0.0
    info: dict = field(default_factory=dict)


@runtime_checkable
class ClimbPolicyAdapter(Protocol):
    """The whole contract. Six calls, no inheritance required.

    A ``Protocol`` rather than a base class so a teammate can hand over an object built any
    way they like -- including one wrapping an ONNX session they would rather not restructure
    -- and it satisfies this by having the methods. ``runtime_checkable`` makes the node able
    to reject a misconfigured plugin at startup with a useful message instead of at the first
    control tick with an ``AttributeError``.
    """

    #: Which boundary this policy writes to. Read once, at load, to pick the arbiter.
    action_kind: ActionKind

    def reset(self) -> None:
        """Drop all history: action buffers, recurrent state, elapsed time, everything.

        Called before every ``start``, including retries. A policy carrying observation
        history from a previous failed attempt is being fed a discontinuity it never saw in
        training -- the robot teleports back to the approach -- and the router has no way to
        tell that apart from the policy simply behaving badly.
        """

    def start(self, observation: PolicyObservation) -> None:
        """Begin a run from this entry state."""

    def step(self, observation: PolicyObservation) -> PolicyAction:
        """One control tick."""

    def cancel(self) -> None:
        """Stop now. Must be safe to call from any state, including before ``start``."""

    def is_finished(self) -> bool:
        """True once the policy will produce no further useful actions."""

    def result(self) -> PolicyResult:
        """The policy's own account of the run. Advisory."""
