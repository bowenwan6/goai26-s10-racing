"""A climb policy that does nothing useful, on purpose, in precisely specified ways.

The teammate's DAgger work has not produced a checkpoint, an export or an interface note, and
waiting for one would leave the router untested against the cases that actually matter. Those
cases are not "does the policy climb well" -- that is the policy's problem -- but "what does
the router do when the policy lies, hangs, or explodes", which is the router's problem and is
testable today.

So this is not a stand-in for a climb. It is a set of failure modes with a policy-shaped
interface, and every one of them is drawn from something that has actually gone wrong here or
is an obvious way for an inference path to go wrong:

``SUCCEED``          the happy path, and the one that must still be verified before it counts.
``FAIL``             an honest failure. Cheap; the router should retry.
``NEVER_FINISH``     runs forever reporting RUNNING. This is what a policy stuck in a local
                     minimum looks like from outside, and only the router's timeout ends it.
``GO_SILENT``        stops updating status *and* stops producing motion, without finishing.
                     Distinct from NEVER_FINISH because a router that watches only status
                     never notices, and a router that watches only motion mistakes a slow
                     manoeuvre for this.
``RAISE``            throws from ``step``. An ONNX session whose input shape drifted does
                     exactly this, at the first tick, in production.
``LIE``              reports SUCCEEDED without the robot having moved. The single most
                     important case in the file: it is the one that decides whether
                     ``VERIFY_CLEAR`` is load bearing or decorative.

Determinism is a requirement, not a nicety -- these back assertions about exact tick counts,
so nothing here samples a random number or reads a clock. Elapsed time comes from the
observation, which the test controls.
"""

from __future__ import annotations

from enum import Enum

import numpy as np

from s10_auto_nav.strategy.policy import (
    ActionKind,
    PolicyAction,
    PolicyObservation,
    PolicyResult,
    PolicyStatus,
)


class MockScenario(Enum):
    SUCCEED = "succeed"
    FAIL = "fail"
    NEVER_FINISH = "never_finish"
    GO_SILENT = "go_silent"
    RAISE = "raise"
    LIE = "lie"


class MockClimbPolicy:
    """Deterministic test double for :class:`ClimbPolicyAdapter`.

    ``duration`` is in seconds of observation time, not ticks, so a test that changes the
    control rate does not silently change what is being tested.
    """

    def __init__(
        self,
        scenario: MockScenario = MockScenario.SUCCEED,
        *,
        action_kind: ActionKind = ActionKind.TWIST,
        duration: float = 2.0,
        forward: float = 0.4,
        joint_amplitude: float = 0.1,
    ):
        self.scenario = scenario
        self.action_kind = action_kind
        self.duration = float(duration)
        self.forward = float(forward)
        self.joint_amplitude = float(joint_amplitude)

        self.reset_count = 0
        self.start_count = 0
        self.cancel_count = 0
        self.step_count = 0
        self._status = PolicyStatus.IDLE
        self._t0: float | None = None
        self._last_t = 0.0
        self._reason = ""

    # -- lifecycle ---------------------------------------------------------

    def reset(self) -> None:
        # Everything except the call counters, which are what the tests assert on. A reset
        # that also cleared them would make "was reset called before start" unanswerable.
        self.reset_count += 1
        self._status = PolicyStatus.IDLE
        self._t0 = None
        self._last_t = 0.0
        self._reason = ""

    def start(self, observation: PolicyObservation) -> None:
        self.start_count += 1
        self._t0 = float(observation.t)
        self._last_t = float(observation.t)
        self._status = PolicyStatus.RUNNING

    def cancel(self) -> None:
        self.cancel_count += 1
        # Only a running policy can be cancelled; cancelling a finished one must not
        # rewrite its verdict, or a router that cancels defensively after success would
        # erase the result it was about to verify.
        if self._status is PolicyStatus.RUNNING:
            self._status = PolicyStatus.CANCELLED
            self._reason = "cancelled by router"

    def is_finished(self) -> bool:
        return self._status.terminal

    def result(self) -> PolicyResult:
        return PolicyResult(
            status=self._status,
            reason=self._reason,
            elapsed=0.0 if self._t0 is None else self._last_t - self._t0,
            info={"scenario": self.scenario.value, "steps": self.step_count},
        )

    # -- the tick ----------------------------------------------------------

    def step(self, observation: PolicyObservation) -> PolicyAction:
        self.step_count += 1
        self._last_t = float(observation.t)
        if self._t0 is None:
            raise RuntimeError("step before start")
        elapsed = self._last_t - self._t0

        if self.scenario is MockScenario.RAISE:
            raise RuntimeError("simulated inference failure")

        if self._status.terminal:
            return self._action(PolicyStatus.CANCELLED, moving=False)

        done = elapsed >= self.duration
        if self.scenario is MockScenario.NEVER_FINISH:
            return self._action(PolicyStatus.RUNNING, moving=True)
        if self.scenario is MockScenario.GO_SILENT:
            # Still RUNNING, but no motion at all after `duration`. The router has to catch
            # this through progress, not through status.
            return self._action(PolicyStatus.RUNNING, moving=not done)
        if not done:
            return self._action(PolicyStatus.RUNNING, moving=True)

        if self.scenario in (MockScenario.SUCCEED, MockScenario.LIE):
            self._status = PolicyStatus.SUCCEEDED
            self._reason = (
                "finished"
                if self.scenario is MockScenario.SUCCEED
                else "claims success without moving"
            )
        else:
            self._status = PolicyStatus.FAILED
            self._reason = "simulated failure"
        return self._action(self._status, moving=False)

    def _action(self, status: PolicyStatus, *, moving: bool) -> PolicyAction:
        if self.action_kind is ActionKind.TWIST:
            twist = (self.forward if moving else 0.0, 0.0, 0.0)
            # A terminal action carries no command: the router must not be able to keep
            # driving on the strength of the last tick of a finished policy.
            return PolicyAction(ActionKind.TWIST, status, twist=None if status.terminal else twist)
        joints = np.full(16, self.joint_amplitude) if moving else np.zeros(16)
        return PolicyAction(ActionKind.JOINT, status, joints=None if status.terminal else joints)
