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

from s10_perception.segment_spawn import SpawnOverride, ground_height, level_ground

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


#: A deck on stilts over open floor, which is the shape the course actually has at waypoints
#: 23 to 25 and 28 to 32 and the shape that made a whole sweep worthless. Both surfaces are
#: level, both are free space above, and a downward ray finds whichever the caller asks for.
#: The deck is 3 m up so that no plausible tolerance can confuse it with the floor.
TWO_STOREY = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="20 20 0.1" pos="0 0 0"/>
    <geom name="deck" type="box" size="4 4 0.1" pos="0 0 3.0"/>
    <geom name="pillar" type="box" size="0.2 0.2 1.45" pos="3.5 3.5 1.45"/>
  </worldbody>
</mujoco>
"""


@pytest.fixture
def scene():
    model = mujoco.MjModel.from_xml_string(SCENE)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


@pytest.fixture
def two_storey():
    model = mujoco.MjModel.from_xml_string(TWO_STOREY)
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


def test_spawn_jitter_components_can_be_controlled_independently():
    spawn = SpawnOverride.from_env(
        {
            "S10_SPAWN_XY": "1,2",
            "S10_SPAWN_SEED": "7",
            "S10_SPAWN_POSITION_SIGMA": "0.03",
            "S10_SPAWN_YAW_SIGMA_DEG": "2.5",
            "S10_SPAWN_JOINT_SIGMA": "0",
        }
    )
    assert spawn.position_sigma == 0.03
    assert spawn.yaw_sigma_deg == 2.5
    assert spawn.joint_sigma == 0.0


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
    with pytest.raises(ValueError, match="level enough"):
        SpawnOverride(x=500.0, y=500.0, yaw=0.0).pose(model, data, JOINT_INIT)


def test_level_ground_rejects_a_footprint_straddling_an_edge(scene):
    """The block's edge is at x=4. A pose centred there has half its wheels 0.5 m lower."""
    model, data = scene
    assert level_ground(model, data, 5.0, 0.0, 0.0) == pytest.approx(0.5, abs=1e-6)
    assert math.isnan(level_ground(model, data, 4.0, 0.0, 0.0))


def test_level_ground_follows_the_heading(scene):
    """The footprint is longer than it is wide, so which way it points changes the answer.

    The block's edge is at y=1. From y=0.75 the near 0.22 m half-width clears it and the
    near 0.30 m half-length does not, so the same point is legal facing along the block
    and not legal facing across it.
    """
    model, data = scene
    assert level_ground(model, data, 5.0, 0.75, 0.0) == pytest.approx(0.5, abs=1e-6)
    assert math.isnan(level_ground(model, data, 5.0, 0.75, math.pi / 2))


def test_a_spawn_against_an_edge_backs_off_along_the_approach(scene):
    """Waypoint 23's failure mode: the waypoint itself is not a place to stand."""
    model, data = scene
    # Facing +x at the block's leading edge. Backing off along -yaw finds the floor.
    base, _, _ = SpawnOverride(x=4.0, y=0.0, yaw=0.0, height=0.2).pose(model, data, JOINT_INIT)
    assert base[0] < 4.0, "should have moved back along the approach"
    assert base[2] == pytest.approx(0.2, abs=1e-6), "and be standing on the floor, not the block"


def test_backing_off_does_not_move_a_pose_that_is_already_legal(scene):
    """Otherwise every spawn drifts backwards and no segment starts where it says."""
    model, data = scene
    base, _, _ = SpawnOverride(x=0.0, y=0.0, yaw=0.0).pose(model, data, JOINT_INIT)
    assert base[:2] == pytest.approx([0.0, 0.0])


# --------------------------------------------------------------------- storeys
#
# The rest of this file is about one bug, because it cost more than any other here: a course
# that runs over itself has several correct answers to "the ground at (x, y)", the lookup
# returned the lowest, and thirty-six full-stack runs were collected of a robot boxed in
# under the deck it was supposed to be driving on. See resources/post_wp16/walls/VOID.md.


def test_the_storey_reference_is_read_from_the_environment():
    spawn = SpawnOverride.from_env({"S10_SPAWN_XY": "1,2", "S10_SPAWN_Z": "1.670"})
    assert spawn.z_ref == 1.670


def test_the_storey_reference_defaults_to_the_ground_floor():
    """Every fixture in this file and every single-storey scene relies on this."""
    assert SpawnOverride.from_env({"S10_SPAWN_XY": "1,2"}).z_ref == 0.0


def test_the_reference_chooses_between_two_surfaces_at_the_same_point(two_storey):
    """The whole bug in three lines: same (x, y), two floors, and the caller says which."""
    model, data = two_storey
    assert ground_height(model, data, 0.0, 0.0, 0.0) == pytest.approx(0.0, abs=1e-6)
    assert ground_height(model, data, 0.0, 0.0, 3.1) == pytest.approx(3.1, abs=1e-6)
    assert level_ground(model, data, 0.0, 0.0, 0.0, 0.0) == pytest.approx(0.0, abs=1e-6)
    assert level_ground(model, data, 0.0, 0.0, 0.0, 3.1) == pytest.approx(3.1, abs=1e-6)


def test_a_reference_with_no_surface_near_it_is_refused_rather_than_approximated(two_storey):
    """The floor is 1.5 m below and the deck 1.5 m above; neither is the storey asked for.

    Returning the nearer of the two would be the original bug with a smaller error, and it
    would be silent. NaN reaches the caller, which raises, which is a run that is thrown away
    rather than a run that is counted.
    """
    model, data = two_storey
    assert math.isnan(ground_height(model, data, 0.0, 0.0, 1.5))
    assert math.isnan(level_ground(model, data, 0.0, 0.0, 0.0, 1.5))


def test_a_waypoint_just_short_of_a_step_still_finds_the_step(two_storey):
    """Above the reference is the looser side on purpose, and this is why.

    A waypoint is published on the route, and the route crosses steps. Being a few
    centimetres below the surface the footprint lands on is normal and must not be refused.
    """
    model, data = two_storey
    assert ground_height(model, data, 0.0, 0.0, 2.9) == pytest.approx(3.1, abs=1e-6)


def test_the_pose_lands_on_the_storey_it_was_given(two_storey):
    model, data = two_storey
    on_deck = SpawnOverride(x=0.0, y=0.0, yaw=0.0, height=0.2, z_ref=3.1)
    on_floor = SpawnOverride(x=0.0, y=0.0, yaw=0.0, height=0.2, z_ref=0.0)
    assert on_deck.pose(model, data, JOINT_INIT)[0][2] == pytest.approx(3.3, abs=1e-6)
    assert on_floor.pose(model, data, JOINT_INIT)[0][2] == pytest.approx(0.2, abs=1e-6)


def test_the_refusal_names_the_storey_it_could_not_find(two_storey):
    """A run that dies has to say which of the several right answers it wanted."""
    model, data = two_storey
    with pytest.raises(ValueError, match=r"storey at z=1\.500"):
        SpawnOverride(x=0.0, y=0.0, yaw=0.0, z_ref=1.5).pose(model, data, JOINT_INIT)
