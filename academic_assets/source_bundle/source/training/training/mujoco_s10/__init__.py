"""Native MuJoCo training backend for the GOAI S10.

The package deliberately keeps ROS out of the learning loop.  It uses the same MJCF,
calibration, observation order and 20 ms low-level command contract as the contest SDK.
"""

from .config import EnvConfig, PIT_CURRICULUM

__all__ = ["EnvConfig", "PIT_CURRICULUM"]
