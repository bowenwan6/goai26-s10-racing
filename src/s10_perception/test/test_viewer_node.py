import numpy as np

from s10_perception.viewer_node import _QPOS_FRAME, _raw_joint_positions, _upstream


def test_published_joint_positions_are_converted_for_mujoco():
    np.testing.assert_allclose(_raw_joint_positions(np.zeros(16)), _upstream.POS_OFFSET_RAD)


def test_windows_viewer_frame_contains_base_and_all_joints():
    qpos = np.arange(23, dtype=float)
    assert _QPOS_FRAME.unpack(_QPOS_FRAME.pack(*qpos)) == tuple(qpos)
