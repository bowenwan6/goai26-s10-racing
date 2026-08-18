from types import SimpleNamespace
from unittest.mock import Mock

import mujoco
import numpy as np

from s10_perception.sim_node import (
    _MANUAL_FRAME,
    _MANUAL_MAGIC,
    PerceptionSimulationNode,
    _decode_manual_control,
    _set_wheel_friction,
)
from s10_perception.upstream import default_track_xml


def test_disabled_perception_keeps_odometry_and_skips_raycasts():
    stamp = object()
    clock = Mock()
    clock.now.return_value.to_msg.return_value = stamp
    node = SimpleNamespace(
        get_clock=Mock(return_value=clock),
        _publish_odometry=Mock(),
        use_perception=False,
    )

    PerceptionSimulationNode._publish_perception(node)

    node._publish_odometry.assert_called_once_with(stamp)


def test_wheel_friction_only_changes_wheel_collisions():
    model = mujoco.MjModel.from_xml_path(str(default_track_xml()))
    original = model.geom_friction[:, 0].copy()

    _set_wheel_friction(model, 2.0)

    changed = model.geom_friction[:, 0] != original
    assert changed.sum() == 4
    np.testing.assert_allclose(model.geom_friction[changed, 0], 2.0)


def test_manual_control_packet_is_validated():
    targets = np.arange(16, dtype=np.float32)
    mode, decoded = _decode_manual_control(_MANUAL_FRAME.pack(_MANUAL_MAGIC, 1, *targets))

    assert mode == 1
    np.testing.assert_array_equal(decoded, targets)
    assert _decode_manual_control(b"bad") is None


def test_start_waypoint_uses_one_based_path_order(monkeypatch):
    positions = np.zeros((18, 3))
    positions[15] = [11.1225, 33.0075, 0.1]
    positions[16] = [16.275, 31.29, 0.475]
    node = SimpleNamespace(
        track_waypoint_positions=positions,
        data=SimpleNamespace(qpos=np.zeros(23), qvel=np.ones(22)),
        model=object(),
        get_logger=Mock(return_value=Mock()),
    )
    forward = Mock()
    monkeypatch.setenv("S10_START_WAYPOINT", "16")
    monkeypatch.setattr("s10_perception.sim_node.mujoco.mj_forward", forward)

    PerceptionSimulationNode._set_start_waypoint(node)

    np.testing.assert_allclose(node.data.qpos[:3], [11.1225, 33.0075, 0.3])
    assert np.count_nonzero(node.data.qvel) == 0
    forward.assert_called_once_with(node.model, node.data)


def test_start_yaw_can_face_an_obstacle_normal(monkeypatch):
    positions = np.zeros((18, 3))
    positions[15] = [11.1225, 33.0075, 0.1]
    positions[16] = [16.275, 31.29, 0.475]
    node = SimpleNamespace(
        track_waypoint_positions=positions,
        data=SimpleNamespace(qpos=np.zeros(23), qvel=np.ones(22)),
        model=object(),
        get_logger=Mock(return_value=Mock()),
    )
    monkeypatch.setenv("S10_START_WAYPOINT", "16")
    monkeypatch.setenv("S10_START_YAW_DEG", "0")
    monkeypatch.setattr("s10_perception.sim_node.mujoco.mj_forward", Mock())

    PerceptionSimulationNode._set_start_waypoint(node)

    np.testing.assert_allclose(node.data.qpos[3:7], [1.0, 0.0, 0.0, 0.0])
