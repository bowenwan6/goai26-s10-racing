"""Height map sampling, on models small enough to reason about by hand.

The case that matters here is a roof. On the shipped track the straight at y=5.5 runs under
a canopy about 1.5 m above the base, and reporting it as terrain put every cell of the grid
at the clip ceiling: relief 1.42 on ground the robot was standing level on, the terrain
scaler pinned to its 0.25 floor, and sixteen seconds spent crawling under something the
robot never touched. A three-geom model reproduces that in milliseconds.
"""

import mujoco
import numpy as np
import pytest

from s10_perception.heightmap import HeightmapConfig, HeightmapSampler

#: Where the base sits in every model below, and so the reference all heights are relative
#: to. Matches the S10's standing height closely enough for the numbers to read naturally.
BASE_Z = 0.42


def model_with(extra_geoms: str):
    """A floor at z=0, a base body at BASE_Z, and whatever else the test needs.

    The base's own geom is group 1, as on the real robot, so it is excluded from sampling
    both by group and by `bodyexclude`.
    """
    xml = f"""
    <mujoco>
      <worldbody>
        <geom name="floor" type="plane" size="20 20 0.1" pos="0 0 0" group="0"/>
        {extra_geoms}
        <body name="base_link" pos="0 0 {BASE_Z}">
          <freejoint/>
          <geom name="chassis" type="box" size="0.2 0.2 0.1" group="1"/>
        </body>
      </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    return model, data, body_id


def sample(extra_geoms="", config=None):
    model, data, body_id = model_with(extra_geoms)
    sampler = HeightmapSampler(model, body_id, config)
    return sampler.sample(data)


def test_flat_ground_reads_as_the_base_height():
    heights = sample()
    assert np.allclose(heights, -BASE_Z, atol=1e-3)


#: A step whose edge falls inside the grid, so one sample sees both levels. The grid runs
#: x in [-0.6, 1.2]; this box spans x from +0.3 out, leaving the rear cells on the floor.
#: Top face at z=0.20, well under the clip ceiling, so it is terrain and must be seen.
STEP = '<geom name="step" type="box" size="2.7 5 0.10" pos="3.0 0 0.10" group="0"/>'


def test_a_step_within_reach_is_reported():
    heights = sample(STEP)
    assert heights.max() == pytest.approx(0.20 - BASE_Z, abs=1e-3)
    assert heights.min() == pytest.approx(-BASE_Z, abs=1e-3)


def test_a_canopy_overhead_is_not_terrain():
    """The y=5.5 failure: a roof above the clip ceiling must not read as ground."""
    # Underside 1.75 m up -- far above the 1.42 m ceiling, and squarely in the path of a
    # ray that starts 1.5 m over the base.
    heights = sample('<geom name="canopy" type="box" size="5 5 0.05" pos="0 0 1.80" group="0"/>')
    assert np.allclose(heights, -BASE_Z, atol=1e-3), "the floor beneath the roof, not the roof"


def test_a_layered_canopy_is_not_terrain_either():
    """The real one is five overlapping geoms deep, not a single sheet."""
    layers = "".join(
        f'<geom name="layer{i}" type="box" size="5 5 0.02" pos="0 0 {1.6 + 0.08 * i}" group="0"/>'
        for i in range(5)
    )
    assert np.allclose(sample(layers), -BASE_Z, atol=1e-3)


def test_a_canopy_does_not_hide_a_step_under_it():
    """Passing through the roof must still find whatever is genuinely underfoot."""
    canopy = '<geom name="canopy" type="box" size="5 5 0.05" pos="0 0 1.80" group="0"/>'
    heights = sample(canopy + STEP)
    assert heights.max() == pytest.approx(0.20 - BASE_Z, abs=1e-3)
    assert heights.min() == pytest.approx(-BASE_Z, abs=1e-3)


def test_an_empty_column_reads_as_a_hole():
    """No floor at all: a void must read as a hole, not as flat ground."""
    xml = f"""
    <mujoco>
      <worldbody>
        <body name="base_link" pos="0 0 {BASE_Z}">
          <freejoint/>
          <geom name="chassis" type="box" size="0.2 0.2 0.1" group="1"/>
        </body>
      </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    cfg = HeightmapConfig()
    heights = HeightmapSampler(model, body_id, cfg).sample(data)
    assert np.allclose(heights, cfg.clip_below)


def test_ground_beyond_max_depth_reads_as_a_hole():
    """A pit deeper than the sampler's reach is a hole, and stays one."""
    cfg = HeightmapConfig(max_depth=1.0)
    # Floor is BASE_Z + ray_start_height below the ray origin, which exceeds max_depth.
    heights = sample(config=cfg)
    assert np.allclose(heights, cfg.clip_below)


def test_the_grid_has_the_configured_shape():
    cfg = HeightmapConfig()
    assert sample(config=cfg).shape == (cfg.n_x, cfg.n_y)
