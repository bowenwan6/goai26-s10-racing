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
``S10_SPAWN_Z``       the waypoint's own z, naming the storey to stand on. See below.
``S10_SPAWN_HEIGHT``  metres above the sampled ground; defaults to the race's 0.2.
``S10_SPAWN_SEED``    selects deterministic jitter; 0 means the exact nominal pose.
``S10_SPAWN_POSITION_SIGMA`` position jitter sigma in metres; defaults to 0.05.
``S10_SPAWN_YAW_SIGMA_DEG`` heading jitter sigma in degrees; defaults to 4.0.
``S10_SPAWN_JOINT_SIGMA`` joint-position jitter sigma; defaults to 0.01.
``S10_SPAWN_INDEX``   waypoint the spawn corresponds to, so the simulator's own progress
                      counter looks for the right one next instead of the start line's.

**The storey reference is not optional.** This course runs over itself: waypoints 23 to 25
are on a deck at z = 1.670 and waypoints 28 to 32 on one at 3.750, with open floor at 0.479
underneath both. "The ground at (x, y)" therefore has several answers and only one of them
is the right one, so the caller has to say which storey it meant. It is the waypoint's own z
because that is the one number that is already known to be on the route.
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

#: How far a surface may sit below and above the storey reference and still be that storey.
#: The two are asymmetric on purpose. Below is tight: the reference is a waypoint, waypoints
#: are published on the surface they are traversed on, and the gap between the two is
#: centimetres -- while the gap to the storey underneath is metres. Above is looser because a
#: waypoint just short of a step is legitimately below the surface the footprint lands on.
STOREY_BELOW = 0.5
STOREY_ABOVE = 0.9


def level_ground(model, data, x: float, y: float, yaw: float, z_ref: float = 0.0) -> float:
    """Ground height at ``(x, y)`` on storey ``z_ref``, or NaN if it is not one surface.

    Waypoint 23 is the case that motivates this, twice over. A single ray at the waypoint
    returns 0.479 m, which is the top of a wall beside it; the robot spawned 0.2 m above that,
    dropped off it, and spent the run wedged at 0.37 m from a wall face reversing at
    -0.4 m/s without moving. That is not the navigation stack failing the segment, and
    counting it as such would have been the second wrong conclusion in this file's history.

    The footprint fixed the wall top. It did not fix the storey: waypoint 23 is at z = 1.670
    and 0.479 is the open floor a metre and a half *below* the deck it is on, which the
    footprint agrees is beautifully level because it is. Thirty-six runs were collected
    against that, all of them of a robot boxed in under the deck it was supposed to be
    driving on. Passing the waypoint's own z is what makes the answer the right storey, and
    the reason it is a required argument in practice rather than a nicety.
    """
    heights = []
    for dx, dy in FOOTPRINT:
        px = x + dx * math.cos(yaw) - dy * math.sin(yaw)
        py = y + dx * math.sin(yaw) + dy * math.cos(yaw)
        height = ground_height(model, data, px, py, z_ref)
        if not math.isfinite(height):
            return float("nan")
        heights.append(height)
    if max(heights) - min(heights) > FOOTPRINT_TOLERANCE:
        return float("nan")
    return max(heights)


def ground_height(model, data, x: float, y: float, z_ref: float, *, ceiling: float = 14.0) -> float:
    """Height of the surface at ``(x, y)`` on the storey ``z_ref`` names.

    Casting down from high above finds the roof rather than the floor wherever the course
    runs over itself, and the surfaces cannot be told apart by the order the rays return
    them in -- the scene is meshes, and a ray crosses a non-convex mesh any number of times.
    So each candidate is tested: fire again from just above it and see whether the ray comes
    back to the same surface, which it only does if that space is empty.

    Free space is necessary and not sufficient. The floor under a deck is free space too, and
    picking it because it was the first free surface found is how a spawn ends up a storey
    down. So a candidate must also be within :data:`STOREY_BELOW` / :data:`STOREY_ABOVE` of
    the reference, and a walk that finds no such candidate returns NaN rather than the
    nearest thing it did find. Returning something plausible from the wrong storey is worse
    than returning nothing: nothing is caught by the caller, and plausible is not.
    """
    z = ceiling
    down = np.array([0.0, 0.0, -1.0])
    gid = np.zeros(1, dtype=np.int32)
    for _ in range(24):
        dist = mujoco.mj_ray(model, data, np.array([x, y, z]), down, TERRAIN_ONLY, 1, -1, gid)
        if dist < 0:
            break
        hit = z - dist
        back = mujoco.mj_ray(
            model, data, np.array([x, y, hit + 0.25]), down, TERRAIN_ONLY, 1, -1, gid
        )
        if back >= 0 and abs(back - 0.25) < 1e-6 and hit <= z_ref + STOREY_ABOVE:
            return hit if hit >= z_ref - STOREY_BELOW else float("nan")
        nxt = hit - 0.005
        if nxt >= z - 1e-9:
            break
        z = nxt
    return float("nan")


@dataclass(frozen=True)
class SpawnOverride:
    """Where to put the robot instead of the start line."""

    x: float
    y: float
    yaw: float
    height: float = 0.2
    seed: int = 0
    waypoint_index: int | None = None
    #: The storey to stand on, as the waypoint's own z. Zero is the ground floor, which is
    #: what a single-storey scene and every test fixture in this repository has.
    z_ref: float = 0.0
    position_sigma: float = POSITION_SIGMA
    yaw_sigma_deg: float = YAW_SIGMA_DEG
    joint_sigma: float = JOINT_SIGMA

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
            z_ref=float(env.get("S10_SPAWN_Z", "0.0")),
            position_sigma=float(env.get("S10_SPAWN_POSITION_SIGMA", POSITION_SIGMA)),
            yaw_sigma_deg=float(env.get("S10_SPAWN_YAW_SIGMA_DEG", YAW_SIGMA_DEG)),
            joint_sigma=float(env.get("S10_SPAWN_JOINT_SIGMA", JOINT_SIGMA)),
        )

    def pose(
        self, model, data, joint_init: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return ``(base_xyz, base_quat_wxyz, joint_positions)`` for this spawn.

        Separate from :meth:`apply` so the arithmetic can be tested without a simulator
        being driven, and so a caller that wants to reject a spawn can see it first.
        """
        rng = np.random.default_rng(self.seed)
        jitter = rng.normal(0.0, self.position_sigma, 2) if self.seed else np.zeros(2)
        x = self.x + float(jitter[0])
        y = self.y + float(jitter[1])
        yaw = self.yaw + (math.radians(rng.normal(0.0, self.yaw_sigma_deg)) if self.seed else 0.0)
        joints = np.asarray(joint_init, dtype=np.float64).copy()
        if self.seed:
            joints += rng.normal(0.0, self.joint_sigma, joints.size)

        # A waypoint is a point on the route, not necessarily a place to stand: several sit
        # hard against the wall the robot is meant to pass. The robot reaches them from
        # behind, so the pose it actually occupies on the way in is a little short of the
        # waypoint, and that is where a legal spawn is looked for. Backing off along -yaw
        # keeps the robot on the route rather than displacing it sideways off the line.
        back = 0.0
        while back <= SEARCH_LIMIT:
            px = x - back * math.cos(yaw)
            py = y - back * math.sin(yaw)
            floor = level_ground(model, data, px, py, yaw, self.z_ref)
            if math.isfinite(floor):
                half = yaw / 2.0
                quat = np.array([math.cos(half), 0.0, 0.0, math.sin(half)])
                return np.array([px, py, floor + self.height]), quat, joints
            back += SEARCH_STEP

        raise ValueError(
            f"no ground level enough to stand on within {SEARCH_LIMIT:.1f}m behind "
            f"({x:.3f}, {y:.3f}) on the storey at z={self.z_ref:.3f}; this spawn is not a "
            "segment the robot failed"
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
