"""Exercise the real MuJoCo multi-ray binding used by the simulator."""

import mujoco
import numpy as np

from s10_perception.lidar import LidarConfig, RayCastLidar


def test_scan_matches_mujoco_331_multiray_signature():
    model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <geom name="floor" type="plane" size="5 5 0.1" group="0"/>
            <body name="base_link" pos="0 0 0.5">
              <freejoint/>
              <geom name="base" type="box" size="0.1 0.1 0.1" group="1"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    config = LidarConfig(n_azimuth=8, n_elevation=2, range_max=5.0)

    ranges = RayCastLidar(model, body_id, config).scan(data)

    assert ranges.shape == (2, 8)
    assert ranges.dtype == np.float32
    assert np.all(np.isfinite(ranges))
    assert np.all((config.range_min <= ranges) & (ranges <= config.range_max))
