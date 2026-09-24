"""Actual MuJoCo geometry sampling, not robot dynamics or a full-course run."""

from __future__ import annotations

import math

import mujoco
import numpy as np
import pytest

from real_transfer.geometry import height_grid, points_in_yaw_frame, pose_matrix
from s10_perception.heightmap import HeightmapConfig, HeightmapSampler, grid_points


@pytest.mark.parametrize("step_height", [0, 0.1, 0.2])
@pytest.mark.parametrize("yaw", [0, math.pi / 2])
def test_point_projection_matches_mujoco_sampled_surface(step_height, yaw):
    step = (
        (
            f'<geom type="box" size="2 4 {step_height / 2}" '
            f'pos="2.375 0 {step_height / 2}" group="0"/>'
        )
        if step_height
        else ""
    )
    model = mujoco.MjModel.from_xml_string(f"""
        <mujoco><worldbody>
          <geom type="plane" size="10 10 .1" group="0"/>
          {step}
          <body name="base" pos="0 0 .42">
            <freejoint/><geom type="box" size=".2 .2 .1" group="1"/>
          </body>
        </worldbody></mujoco>
    """)
    data = mujoco.MjData(model)
    data.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    mujoco.mj_forward(model, data)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    expected = HeightmapSampler(model, body).sample(data)
    yaw_points = np.column_stack((grid_points(HeightmapConfig()), expected.ravel()))
    yaw_points = np.repeat(yaw_points, 3, axis=0)
    # Simulate a nontrivial sensor mounting, invert it to create sensor-frame data.
    t = np.eye(4)
    t[:3, 3] = [0.1, -0.05, 0.2]
    t[:3, :3] = pose_matrix([0, 0, 0], [0, math.sin(0.1), 0, math.cos(0.1)])[:3, :3]
    sensor_points = (yaw_points - t[:3, 3]) @ t[:3, :3]
    mb = pose_matrix([0, 0, 0.42], [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)])
    local = points_in_yaw_frame(sensor_points, t, mb)
    grid, valid, _ = height_grid(local)
    assert valid.all()
    np.testing.assert_allclose(grid, expected, atol=1e-6)
