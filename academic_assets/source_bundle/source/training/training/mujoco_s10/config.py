"""Configuration for the contest-native MuJoCo S10 training task."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


# Team measurement: the exit lip is 0.377 m and the current controller reaches
# roughly 0.350 m.  Wheel-sized obstacles teach the approach first; the dense
# terminal stages then isolate the missing 0.027 m.  The 0.40 m stage is the
# competition robustness margin; 0.45 and 0.50 m are separate product-capability
# stretch stages matching DEEP Robotics' published S10 obstacle-height claim.
PIT_CURRICULUM = (
    tuple(index / 100.0 for index in range(5, 38))
    + (0.377,)
    + tuple(index / 100.0 for index in range(38, 51))
)


@dataclass
class EnvConfig:
    xml_path: Path
    observation_dim: int = 57
    physics_dt: float = 0.001
    control_dt: float = 0.020
    # Curriculum first teaches a complete traversal.  The contest actor needs almost
    # the full six seconds even on the 5 cm rehearsal, which made geometry learning
    # indistinguishable from a speed trial.  Final evaluation can still request a
    # tighter horizon explicitly.
    episode_seconds: float = 10.0
    command_forward: float = 0.6
    pit_depth: float = PIT_CURRICULUM[0]
    # Official-track ray casts at y=33.365 locate the 0.377 m trench edges at
    # x=10.395-10.3975 and x=12.645-12.6475: approximately 2.25 m apart.
    # The earlier 0.65 m surrogate could spawn the rear half of S10 inside the left
    # platform and produced invalid full-climb trajectories.
    pit_length_min: float = 2.25
    pit_length_max: float = 2.25
    pit_center_x: float = 0.0
    # ``exit_only`` starts on the lower pit floor close to the exit wall.
    # ``front_up`` is the stronger reverse curriculum observed by the team: the
    # front wheels already rest on the platform while the rear wheels stay below.
    reset_mode: str = "full_pit"
    # At the hard stages, mix in the team's measured official-policy state where
    # both front wheels are already on the 0.377 m deck and the rear wheels are
    # wedged 7.6-9.2 cm behind the lip.  This trains the missing continuation rather
    # than asking PPO to rediscover the successful first half of the manoeuvre.
    trace_seed_probability: float = 0.0
    exit_approach_min: float = 0.80
    exit_approach_max: float = 1.20
    reset_lateral_range: float = 0.08
    reset_yaw_range: float = 0.05
    platform_half_length: float = 3.0
    platform_half_width: float = 2.0
    start_x: float = -1.10
    finish_x: float = 0.85
    seed: int = 0

    @property
    def frame_skip(self) -> int:
        ratio = self.control_dt / self.physics_dt
        rounded = int(round(ratio))
        if abs(ratio - rounded) > 1.0e-9:
            raise ValueError("control_dt must be an integer multiple of physics_dt")
        return rounded

    @property
    def max_steps(self) -> int:
        return int(round(self.episode_seconds / self.control_dt))

    def validate(self) -> None:
        if self.observation_dim not in (57, 174):
            raise ValueError("observation_dim must be 57 or 174")
        if not 0.0 <= self.pit_depth <= 0.50:
            raise ValueError("pit_depth must be in [0, 0.50] metres")
        if self.pit_length_min <= 0 or self.pit_length_max < self.pit_length_min:
            raise ValueError("invalid pit length range")
        if self.reset_mode not in ("full_pit", "exit_only", "front_up"):
            raise ValueError("reset_mode must be full_pit, exit_only or front_up")
        if not 0.0 <= self.trace_seed_probability <= 1.0:
            raise ValueError("trace_seed_probability must be in [0, 1]")
        if self.exit_approach_min <= 0 or self.exit_approach_max < self.exit_approach_min:
            raise ValueError("invalid exit approach range")
        if self.reset_lateral_range < 0.0 or self.reset_yaw_range < 0.0:
            raise ValueError("reset lateral and yaw ranges must be non-negative")
        if self.episode_seconds <= 0.0:
            raise ValueError("episode_seconds must be positive")
        if not Path(self.xml_path).is_file():
            raise FileNotFoundError(self.xml_path)
        _ = self.frame_skip
