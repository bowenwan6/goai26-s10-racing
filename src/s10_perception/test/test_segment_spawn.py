"""The spawn override is a test-only switch, so what it must prove first is that it is off.

Everything else here is arithmetic. The ray casting is exercised against a scene built in
the test rather than against the track, so a failure points at this module and not at a
change in the course.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np
import pytest

from s10_perception.segment_spawn import SpawnOverride, ground_height

#: The scene below has two hinges where the robot has sixteen. The count is irrelevant to
#: everything this module does -- it copies whatever it is given into ``qpos[7:]`` -- and a
#: two-joint scene keeps the fixture readable.
JOINT_INIT = np.array([-1.16, 2.76])

#: A floor, and a block on it, so "the ground under (x, y)" has two different answers.
SCENE = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="20 20 0.1" pos="0 0 0"/>
    <geom name="block" type="box" size="1 1 0.25" pos="5 0 0.25"/>
    <body name="base" pos="0 0 1">
      <freejoint/>
      <geom name="body" type="box" size="0.2 0.2 0.1"/>
      <body name="limb" pos="0.2 0 0">
        <joint name="hip" type="hinge" axis="0 1 0"/>
        <geom name="limb_geom" type="capsule" size="0.03" fromto="0 0 0 0.2 0 0"/>
        <body name="shin" pos="0.2 0 0">
          <joint name="knee" type="hinge" axis="0 1 0"/>
          <geom name="shin_geom" type="capsule" size="0.03" fromto="0 0 0 0.2 0 0"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


@pytest.fixture
def scene():
    model = mujoco.MjModel.from_xml_string(SCENE)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_no_environment_means_no_override():
    """The scored run's guarantee. Every other test here is about a run that is not scored."""
    assert SpawnOverride.from_env({}) is None
    assert SpawnOverride.from_env({"S10_SPAWN_XY": "  "}) is None
    assert SpawnOverride.from_env({"S10_SPAWN_YAW": "1.0", "S10_SPAWN_SEED": "3"}) is None


def test_the_environment_is_read_in_full():
    spawn = SpawnOverride.from_env(
        {
            "S10_SPAWN_XY": "12.5,-3.25",
            "S10_SPAWN_YAW": "1.5",
            "S10_SPAWN_HEIGHT": "0.3",
            "S10_SPAWN_SEED": "4",
            "S10_SPAWN_INDEX": "16",
        }
    )
    assert spawn == SpawnOverride(x=12.5, y=-3.25, yaw=1.5, height=0.3, seed=4, waypoint_index=16)


def test_a_malformed_coordinate_is_refused_rather_than_rounded_to_the_origin():
    """Silently spawning at (0, 0) would look like a segment the robot cannot do."""
    with pytest.raises(ValueError, match="S10_SPAWN_XY"):
        SpawnOverride.from_env({"S10_SPAWN_XY": "12.5 -3.25"})


def test_ground_height_finds_the_floor_and_the_top_of_a_block(scene):
    model, data = scene
    assert ground_height(model, data, 0.0, 0.0, 0.0) == pytest.approx(0.0, abs=1e-6)
    assert ground_height(model, data, 5.0, 0.0, 0.0) == pytest.approx(0.5, abs=1e-6)


def test_seed_zero_is_the_nominal_pose(scene):
    model, data = scene
    spawn = SpawnOverride(x=5.0, y=0.0, yaw=0.0, height=0.2)
    base, quat, joints = spawn.pose(model, data, JOINT_INIT)
    assert base == pytest.approx([5.0, 0.0, 0.7])
    assert quat == pytest.approx([1.0, 0.0, 0.0, 0.0])
    assert joints == pytest.approx(JOINT_INIT)


def test_a_seed_jitters_the_pose_reproducibly(scene):
    model, data = scene
    once = SpawnOverride(x=0.0, y=0.0, yaw=0.0, seed=3).pose(model, data, JOINT_INIT)
    again = SpawnOverride(x=0.0, y=0.0, yaw=0.0, seed=3).pose(model, data, JOINT_INIT)
    other = SpawnOverride(x=0.0, y=0.0, yaw=0.0, seed=4).pose(model, data, JOINT_INIT)
    assert once[0] == pytest.approx(again[0])
    assert once[0][:2] != pytest.approx(other[0][:2])
    # Jitter represents arriving slightly differently, not being dropped somewhere else.
    assert np.linalg.norm(once[0][:2]) < 0.3


def test_yaw_becomes_a_rotation_about_z_only(scene):
    model, data = scene
    _, quat, _ = SpawnOverride(x=0.0, y=0.0, yaw=math.pi / 2).pose(model, data, JOINT_INIT)
    assert quat == pytest.approx([math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)])


def test_apply_writes_the_pose_and_clears_the_velocity(scene):
    model, data = scene
    data.qvel[:] = 1.0
    base = SpawnOverride(x=5.0, y=0.0, yaw=0.0, height=0.2).apply(model, data, JOINT_INIT)
    assert data.qpos[:3] == pytest.approx([5.0, 0.0, 0.7])
    assert base == pytest.approx([5.0, 0.0, 0.7])
    assert data.qvel == pytest.approx(np.zeros_like(data.qvel))


def test_a_spawn_over_nothing_is_refused(scene):
    model, data = scene
    with pytest.raises(ValueError, match="no ground"):
        SpawnOverride(x=500.0, y=500.0, yaw=0.0).pose(model, data, JOINT_INIT)
