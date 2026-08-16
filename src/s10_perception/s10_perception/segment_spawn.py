"""Start the simulator somewhere other than the start line, for testing only.

The race is one 33-waypoint run from a fixed pose, which is the wrong shape for the
question "does the stack get from waypoint 27 to waypoint 28". Answering that by running
the whole race costs minutes and only reports on the segment if everything before it went
right.

This moves the spawn. It is deliberately inert unless :envvar:`S10_SPAWN_XY` is set, so a
scored run is byte for byte what it was: :func:`from_env` returns ``None``, the simulator
never calls :meth:`SpawnOverride.apply`, and nothing else in this module runs. The switch
is an environment variable rather than a node parameter because the pose has to be in place
before the base class finishes constructing, which is before parameters are readable.

The pose it produces is the one the race produces, not a convenient one: crouched at
``JOINT_INIT``, :attr:`SpawnOverride.height` above the ground, upright. The SDK's own state
machine then stands the robot up exactly as it does at the start line. Spawning already
standing was tried first and was worse than wrong -- from some heights the robot bounced and
landed on its back, which reads as a segment the robot cannot do rather than as a spawn it
was never given.

Environment:

``S10_SPAWN_XY``      ``"x,y"`` in world metres. Absent disables everything here.
``S10_SPAWN_YAW``     heading in radians, normally towards the next waypoint.
``S10_SPAWN_HEIGHT``  metres above the sampled ground; defaults to the race's 0.2.
``S10_SPAWN_SEED``    jitters pose and joints; 0 means the exact nominal pose.
``S10_SPAWN_INDEX``   waypoint the spawn corresponds to, so the simulator's own progress
                      counter looks for the right one next instead of the start line's.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import mujoco
import numpy as np

#: Terrain only. Group 1 is the robot and group 2 the decorative route ribbon, which has
#: collision disabled and floats above the ground -- a downward ray hits it first and reports
#: it as the floor. Excluding the robot matters as well: with it included the height under a
#: pose already occupied would be the top of the robot's own base.
TERRAIN_ONLY = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)

#: Position jitter, metres, and heading jitter, degrees, one sigma. Small on purpose: this
#: represents arriving at the waypoint slightly differently, not being dropped elsewhere.
POSITION_SIGMA = 0.05
YAW_SIGMA_DEG = 4.0
JOINT_SIGMA = 0.01

#: Points sampled to decide whether a pose is standing on one surface, in the body frame,
#: metres. Roughly the wheel rectangle plus its centre. A single downward ray cannot tell
#: the middle of the floor from the top of a wall the robot is about to fall off.
FOOTPRINT = (
    (0.0, 0.0),
    (0.30, 0.22),
    (0.30, -0.22),
    (-0.30, 0.22),
    (-0.30, -0.22),
)

#: Ground under the footprint may vary by this much and still be one surface, metres.
#: Larger than the course's stair risers would allow a spawn straddling an edge.
FOOTPRINT_TOLERANCE = 0.12

#: How far back along the approach a legal pose is looked for, and in what increments.
SEARCH_LIMIT = 1.5
SEARCH_STEP = 0.10


def level_ground(model, data, x: float, y: float, yaw: float) -> float:
    """Ground height at ``(x, y)``, or NaN if the footprint there is not on one surface.

    Waypoint 23 is the case that motivates this. A single ray at the waypoint returns
    0.479 m, which is the top of a wall beside it; the robot spawned 0.2 m above that,
    dropped off it, and spent the run wedged at 0.37 m from a wall face reversing at
    -0.4 m/s without moving. That is not the navigation stack failing the segment, and
    counting it as such would have been the second wrong conclusion in this file's history.
    """
    heights = []
    for dx, dy in FOOTPRINT:
        px = x + dx * math.cos(yaw) - dy * math.sin(yaw)
        py = y + dx * math.sin(yaw) + dy * math.cos(yaw)
        height = ground_height(model, data, px, py, 0.0)
        if not math.isfinite(height):
            return float("nan")
        heights.append(height)
    if max(heights) - min(heights) > FOOTPRINT_TOLERANCE:
        return float("nan")
    return max(heights)


def ground_height(model, data, x: float, y: float, z_ref: float, *, ceiling: float = 14.0) -> float:
    """Height of the surface at ``(x, y)`` that a robot near ``z_ref`` would stand on.

    Casting down from high above finds the roof rather than the floor wherever the course
    runs over itself, and the surfaces cannot be told apart by the order the rays return
    them in -- the scene is meshes, and a ray crosses a non-convex mesh any number of times.
    So each candidate is tested: fire again from just above it and see whether the ray comes
    back to the same surface, which it only does if that space is empty.

    Returns NaN when there is no such surface.
    """
    z = ceiling
    down = np.array([0.0, 0.0, -1.0])
    gid = np.zeros(1, dtype=np.int32)
    best = float("nan")
    for _ in range(24):
        dist = mujoco.mj_ray(model, data, np.array([x, y, z]), down, TERRAIN_ONLY, 1, -1, gid)
        if dist < 0:
            break
        hit = z - dist
        back = mujoco.mj_ray(
            model, data, np.array([x, y, hit + 0.25]), down, TERRAIN_ONLY, 1, -1, gid
        )
        if back >= 0 and abs(back - 0.25) < 1e-6:
            best = hit
            if hit <= z_ref + 0.9:
                return hit
        nxt = hit - 0.005
        if nxt >= z - 1e-9:
            break
        z = nxt
    return best


@dataclass(frozen=True)
class SpawnOverride:
    """Where to put the robot instead of the start line."""

    x: float
    y: float
    yaw: float
    height: float = 0.2
    seed: int = 0
    waypoint_index: int | None = None

    @classmethod
    def from_env(cls, env: dict[str, str] | None = None) -> SpawnOverride | None:
        """Read the override, or ``None`` when the simulator should behave as it always has."""
        env = os.environ if env is None else env
        raw = env.get("S10_SPAWN_XY", "").strip()
        if not raw:
            return None
        try:
            x, y = (float(part) for part in raw.split(","))
        except ValueError as exc:  # A typo here would otherwise spawn at (0, 0) in silence.
            raise ValueError(f"S10_SPAWN_XY must be 'x,y', got {raw!r}") from exc
        index = env.get("S10_SPAWN_INDEX", "").strip()
        return cls(
            x=x,
            y=y,
            yaw=float(env.get("S10_SPAWN_YAW", "0.0")),
            height=float(env.get("S10_SPAWN_HEIGHT", "0.2")),
            seed=int(env.get("S10_SPAWN_SEED", "0")),
            waypoint_index=int(index) if index else None,
        )

    def pose(
        self, model, data, joint_init: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return ``(base_xyz, base_quat_wxyz, joint_positions)`` for this spawn.

        Separate from :meth:`apply` so the arithmetic can be tested without a simulator
        being driven, and so a caller that wants to reject a spawn can see it first.
        """
        rng = np.random.default_rng(self.seed)
        jitter = rng.normal(0.0, POSITION_SIGMA, 2) if self.seed else np.zeros(2)
        x = self.x + float(jitter[0])
        y = self.y + float(jitter[1])
        yaw = self.yaw + (math.radians(rng.normal(0.0, YAW_SIGMA_DEG)) if self.seed else 0.0)
        joints = np.asarray(joint_init, dtype=np.float64).copy()
        if self.seed:
            joints += rng.normal(0.0, JOINT_SIGMA, joints.size)

        # A waypoint is a point on the route, not necessarily a place to stand: several sit
        # hard against the wall the robot is meant to pass. The robot reaches them from
        # behind, so the pose it actually occupies on the way in is a little short of the
        # waypoint, and that is where a legal spawn is looked for. Backing off along -yaw
        # keeps the robot on the route rather than displacing it sideways off the line.
        back = 0.0
        while back <= SEARCH_LIMIT:
            px = x - back * math.cos(yaw)
            py = y - back * math.sin(yaw)
            floor = level_ground(model, data, px, py, yaw)
            if math.isfinite(floor):
                half = yaw / 2.0
                quat = np.array([math.cos(half), 0.0, 0.0, math.sin(half)])
                return np.array([px, py, floor + self.height]), quat, joints
            back += SEARCH_STEP

        raise ValueError(
            f"no ground level enough to stand on within {SEARCH_LIMIT:.1f}m behind "
            f"({x:.3f}, {y:.3f}); this spawn is not a segment the robot failed"
        )

    def apply(self, model, data, joint_init: np.ndarray) -> np.ndarray:
        """Write the spawn into ``data.qpos`` and return the base position used.

        The model must have been through one :func:`mujoco.mj_forward` already: static
        geometry is placed by the first forward pass, and rays cast before it report every
        terrain geom sitting at the origin, which looks exactly like "no ground anywhere".
        """
        mujoco.mj_forward(model, data)
        base, quat, joints = self.pose(model, data, joint_init)
        data.qpos[:3] = base
        data.qpos[3:7] = quat
        data.qpos[7 : 7 + joints.size] = joints
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        return base
