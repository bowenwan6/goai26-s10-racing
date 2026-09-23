import json
from types import SimpleNamespace
from unittest.mock import Mock

import mujoco
import numpy as np

from s10_perception.sim_node import PerceptionSimulationNode, _check_him_actuators
from s10_perception.upstream import default_track_xml


def test_reset_publishes_fresh_feedback_then_policy_event(monkeypatch):
    order = []
    node = SimpleNamespace(
        model=object(), data=SimpleNamespace(qpos=np.zeros(23)), _reset_qpos=np.ones(23),
        last_base_linvel=np.ones(3), get_logger=Mock(return_value=Mock()),
        _publish_robot_state=lambda step: order.append("feedback"),
        reset_done_pub=SimpleNamespace(publish=lambda msg: order.append("reset_done")),
        **{name: np.ones(16) for name in ("kp_cmd", "kd_cmd", "pos_cmd", "vel_cmd", "tau_ff", "input_tq")},
    )
    monkeypatch.setattr("s10_perception.sim_node.mujoco.mj_resetData", lambda *args: None)
    monkeypatch.setattr("s10_perception.sim_node.mujoco.mj_forward", lambda *args: None)
    PerceptionSimulationNode._reset_sim(node, None)
    assert order == ["feedback", "reset_done"]
    assert not node.kp_cmd.any() and not node.tau_ff.any()
    np.testing.assert_array_equal(node.data.qpos, node._reset_qpos)


def test_him_torque_limits_checked_at_mujoco_actuators(tmp_path, monkeypatch):
    import pytest
    model = mujoco.MjModel.from_xml_path(str(default_track_xml()))
    cfg = {"policy_type": "s10_him", "torque_limits": [50, 50, 50, 14] * 4,
           "dof_names": [f"{leg}_{joint}_joint" for leg in ("fl", "fr", "hl", "hr")
                         for joint in ("hipx", "hipy", "knee", "wheel")]}
    sidecar = tmp_path / "policy.json"
    sidecar.write_text(json.dumps(cfg))
    monkeypatch.setenv("S10_POLICY_PATH", str(sidecar.with_suffix(".onnx")))
    for key in ("S10_SECOND_POLICY_PATH", "S10_DOWN_POLICY_PATH", "S10_SPEEDTURN_POLICY_PATH"):
        monkeypatch.delenv(key, raising=False)
    assert _check_him_actuators(model) is True
    model.actuator_ctrlrange[3, 1] = 13
    with pytest.raises(ValueError, match="torque limits/order"):
        _check_him_actuators(model)
    monkeypatch.delenv("S10_POLICY_PATH")
    assert _check_him_actuators(model) is False
