"""A recorded trajectory played back through the policy interface. Not a learned policy.

**This is a scripted baseline and must never be reported as a DAgger result.** It contains no
model and does no inference; it replays joint targets that came from the Gate 16 manoeuvre
search in ``resources/``, and its success rate is a statement about that search plus this
playback, nothing more. The distinction matters because the two will be compared, and a
scripted open-loop replay flatters itself on the seeds it was tuned on -- the phase 2 log is
explicit that the tripod's headline "1 in 5" was really "1 in 1 near-static entry state and 4
falls", which is exactly the error a baseline invites.

What it *is* good for is proving the plumbing end to end: a 16-dimensional joint action really
does reach ``/JOINTS_CMD`` through the arbiter, the router really does hold navigation off for
the duration, and ``VERIFY_CLEAR`` really does see the wheels afterwards. None of that can be
tested with a Twist mock, and all of it is where integration bugs live.

Two properties are enforced rather than assumed, because an open-loop replay has no feedback
to catch its own mistakes:

* every commanded joint target is clipped into the joint limits, and
* the replay refuses to start if the robot's current joint positions are further than
  ``max_entry_gap`` from the trajectory's first frame.

The second is the one that matters. Replaying a trajectory recorded from a settled straddle
onto a robot in some other pose is not "a bad attempt", it is a step discontinuity into a
position controller with kp 80 -- it commands whatever torque the gap demands, and the
recorded 19 N.m manoeuvre becomes a 148 N.m one. Refusing is the only honest option.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from s10_auto_nav.strategy.policy import (
    ActionKind,
    PolicyAction,
    PolicyObservation,
    PolicyResult,
    PolicyStatus,
)

#: Per-joint limits of the S10 leg chain, repeated for the four legs. Wheels are velocity
#: controlled and unbounded in position, so their slots are left wide and the clip is a no-op
#: there rather than a silent zeroing.
JOINT_LO = np.tile(np.array([-0.60, -2.50, -2.70, -np.inf]), 4)
JOINT_HI = np.tile(np.array([0.60, 2.50, 2.70, np.inf]), 4)


@dataclass
class ScriptedConfig:
    #: Seconds between trajectory frames. The recorded manoeuvres are sampled at the agent
    #: rate, not the physics rate.
    frame_dt: float = 0.02
    #: How far the robot's joints may be from frame 0 before the replay refuses to start.
    max_entry_gap: float = 0.35
    #: Hold the last frame this long before declaring success, so the body settles onto its
    #: wheels rather than being judged mid-motion.
    settle: float = 0.5
    #: Recording of what was commanded and observed, for post-hoc analysis.
    record: bool = True


class ScriptedClimbPolicy:
    """Replays a joint trajectory, clipped to the limits, with an entry-gap check.

    ``action_kind`` is always ``JOINT``: the manoeuvre is not expressible as a body velocity,
    which is the entire reason it needed a trajectory in the first place.
    """

    action_kind = ActionKind.JOINT
    #: Read by the node and the report writer so a scripted run cannot be mislabelled
    #: downstream by accident.
    is_scripted_baseline = True

    def __init__(
        self,
        trajectory: np.ndarray,
        config: ScriptedConfig | None = None,
        *,
        name: str = "scripted",
    ):
        traj = np.asarray(trajectory, float)
        if traj.ndim != 2 or traj.shape[1] != 16:
            raise ValueError(f"trajectory must be (frames, 16), got {traj.shape}")
        if len(traj) == 0:
            raise ValueError("trajectory is empty")
        self.trajectory = np.clip(traj, JOINT_LO, JOINT_HI)
        self.clipped_frames = int(np.sum(np.any(traj != self.trajectory, axis=1)))
        self.config = config or ScriptedConfig()
        self.name = name

        self.reset_count = 0
        self._status = PolicyStatus.IDLE
        self._t0: float | None = None
        self._last_t = 0.0
        self._reason = ""
        self.log: list[dict] = []

    @classmethod
    def from_csv(cls, path: str | Path, config: ScriptedConfig | None = None):
        """Load frames of 16 joint targets from a CSV, one frame per row.

        A header row is optional and detected rather than configured, because the two
        producers of these files -- the manoeuvre search and a hand edit -- disagree about
        whether to write one, and a flag would just move the mistake.
        """
        rows: list[list[float]] = []
        with Path(path).open() as fh:
            for row in csv.reader(fh):
                if not row:
                    continue
                try:
                    values = [float(v) for v in row]
                except ValueError:
                    continue  # a header, or a comment
                rows.append(values)
        if not rows:
            raise ValueError(f"no numeric rows in {path}")
        return cls(np.asarray(rows, float), config, name=Path(path).stem)

    @property
    def duration(self) -> float:
        return len(self.trajectory) * self.config.frame_dt

    def reset(self) -> None:
        self.reset_count += 1
        self._status = PolicyStatus.IDLE
        self._t0 = None
        self._reason = ""
        self.log = []

    def start(self, observation: PolicyObservation) -> None:
        q = np.asarray(observation.joint_positions, float)
        if q.shape == (16,):
            # Legs only: a wheel's position is a free coordinate that has spun to wherever it
            # spun to, and comparing it against a recorded value would reject every entry.
            legs = np.delete(np.arange(16), np.s_[3::4])
            gap = float(np.abs(q[legs] - self.trajectory[0][legs]).max())
            if gap > self.config.max_entry_gap:
                self._status = PolicyStatus.FAILED
                self._reason = (
                    f"entry gap {gap:.3f} rad exceeds {self.config.max_entry_gap:.2f}; "
                    "replaying from here would command a torque step"
                )
                return
        self._t0 = float(observation.t)
        self._last_t = self._t0
        self._status = PolicyStatus.RUNNING
        self._reason = ""

    def cancel(self) -> None:
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
            info={
                "scripted_baseline": True,
                "name": self.name,
                "frames": len(self.trajectory),
                "clipped_frames": self.clipped_frames,
            },
        )

    def step(self, observation: PolicyObservation) -> PolicyAction:
        if self._status is not PolicyStatus.RUNNING:
            return PolicyAction(ActionKind.JOINT, self._status)
        self._last_t = float(observation.t)
        elapsed = self._last_t - (self._t0 or 0.0)

        if elapsed >= self.duration + self.config.settle:
            self._status = PolicyStatus.SUCCEEDED
            # Wording chosen so a log reader cannot mistake this for a verdict about the
            # robot. The router will check; this only says the tape ran out.
            self._reason = "trajectory replayed to the end"
            return PolicyAction(ActionKind.JOINT, self._status)

        index = min(int(elapsed / self.config.frame_dt), len(self.trajectory) - 1)
        target = self.trajectory[index]
        if self.config.record:
            self.log.append(
                {
                    "t": self._last_t,
                    "elapsed": elapsed,
                    "frame": index,
                    "position": np.asarray(observation.position, float).tolist(),
                    "pitch": observation.pitch,
                    "roll": observation.roll,
                    "q": np.asarray(observation.joint_positions, float).tolist(),
                    "target": target.tolist(),
                }
            )
        return PolicyAction(
            ActionKind.JOINT,
            PolicyStatus.RUNNING,
            joints=target.copy(),
            info={"frame": index, "scripted_baseline": True},
        )

    def write_log(self, path: str | Path) -> Path:
        """Dump the observation/action record as CSV for replay and analysis."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(
                ["t", "elapsed", "frame", "x", "y", "z", "pitch", "roll"]
                + [f"q{i}" for i in range(16)]
                + [f"u{i}" for i in range(16)]
            )
            for row in self.log:
                writer.writerow(
                    [
                        row["t"],
                        row["elapsed"],
                        row["frame"],
                        *row["position"],
                        row["pitch"],
                        row["roll"],
                        *row["q"],
                        *row["target"],
                    ]
                )
        return path


def hold_trajectory(q0: np.ndarray, seconds: float, frame_dt: float = 0.02) -> np.ndarray:
    """The simplest possible trajectory: stand still. Useful as a plumbing test.

    It exercises the whole 16-dimensional path -- adapter, arbiter, ``/JOINTS_CMD`` -- while
    asking the robot to do nothing, so a failure is unambiguously the plumbing.
    """
    frames = max(1, int(round(seconds / frame_dt)))
    return np.tile(np.asarray(q0, float).reshape(1, 16), (frames, 1))
