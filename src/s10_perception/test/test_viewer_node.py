import numpy as np
import mujoco

from scripts.windows_viewer import _add_waypoint_labels, _control_key, _waypoint_labels
from s10_perception.upstream import default_track_xml
from s10_perception.viewer_node import _QPOS_FRAME, _raw_joint_positions, _upstream


def test_published_joint_positions_are_converted_for_mujoco():
    np.testing.assert_allclose(_raw_joint_positions(np.zeros(16)), _upstream.POS_OFFSET_RAD)


def test_windows_viewer_frame_contains_base_and_all_joints():
    qpos = np.arange(23, dtype=float)
    assert _QPOS_FRAME.unpack(_QPOS_FRAME.pack(*qpos)) == tuple(qpos)


def test_windows_viewer_captures_only_robot_keys():
    assert _control_key(ord("W")) == "w"
    assert _control_key(ord("P")) == "p"
    assert _control_key(ord("L")) == "l"
    assert _control_key(ord("K")) == "k"
    assert _control_key(ord("8")) == "8"
    assert _control_key(0xDB) == "["
    assert _control_key(ord("B")) is None
    assert _control_key(ord("Q"), modified=True) is None


def test_waypoint_labels_are_one_based_and_ordered():
    model = mujoco.MjModel.from_xml_path(str(default_track_xml()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    labels = _waypoint_labels(model, data)
    scene = mujoco.MjvScene(model, maxgeom=len(labels))
    _add_waypoint_labels(scene, labels)

    assert [label for label, _ in labels] == [f"WP{i}" for i in range(1, 34)]
    assert scene.ngeom == 33
    assert scene.geoms[0].label == "WP1"
