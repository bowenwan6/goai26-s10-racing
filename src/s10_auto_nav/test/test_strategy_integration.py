"""Integration checks for the router's wiring, launch configuration and joint arbiter.

These cover what the pure state machine tests cannot: that exactly one node publishes
``/cmd_vel`` in each configuration, that the launch default is genuinely the pre-router race,
that both action interfaces reach the boundary in their own shape, and that the official
policy and a climb policy can never both hold ``/JOINTS_CMD``.

Only the last two tests need a ROS installation, and they skip without one. Everything else --
the arbiter, the launch description, the config, both action interfaces -- is deliberately
reachable without a node, a simulator or a running daemon, because those are the checks worth
having on every run rather than only where ROS happens to be installed.
"""

from __future__ import annotations

import ast
from pathlib import Path

import numpy as np
import pytest
import yaml
from test_router import SEGMENT, state

from s10_auto_nav.strategy.arbiter import CLIMB, OFFICIAL, JointArbiter
from s10_auto_nav.strategy.mock_policy import MockClimbPolicy, MockScenario
from s10_auto_nav.strategy.policy import ActionKind
from s10_auto_nav.strategy.router import (
    Router,
    RouterConfig,
    Source,
    observation_from_state,
)
from s10_auto_nav.strategy.scripted_policy import ScriptedClimbPolicy, hold_trajectory

REPO = Path(__file__).resolve().parents[3]
LAUNCH_FILE = REPO / "src" / "s10_bringup" / "launch" / "race.launch.py"
STRATEGY_YAML = REPO / "src" / "s10_bringup" / "config" / "strategy.yaml"
FOLLOWER = REPO / "src" / "s10_auto_nav" / "s10_auto_nav" / "follower_node.py"
ROUTER_NODE = REPO / "src" / "s10_auto_nav" / "s10_auto_nav" / "strategy_router_node.py"


# --------------------------------------------------------------- the arbiter


def test_official_owns_the_joint_topic_by_default():
    """A climb policy has to be handed the actuators; it never starts holding them."""
    sent = []
    a = JointArbiter(sent.append)
    assert a.owner == OFFICIAL
    assert a.forward(CLIMB, np.zeros(16)) is False
    assert sent == []
    assert a.refused == 1


def test_only_one_owner_can_publish_joints():
    sent = []
    a = JointArbiter(sent.append)
    a.grant(CLIMB)
    assert a.forward(CLIMB, np.zeros(16)) is True
    assert a.forward(OFFICIAL, np.zeros(16)) is False
    a.grant(OFFICIAL)
    assert a.forward(CLIMB, np.zeros(16)) is False
    assert len(sent) == 1


def test_the_arbiter_rejects_malformed_joint_commands():
    """A wrong-length or non-finite action must not reach sixteen actuators."""
    sent = []
    a = JointArbiter(sent.append)
    a.grant(CLIMB)
    assert a.forward(CLIMB, np.zeros(12)) is False
    assert a.forward(CLIMB, np.full(16, np.nan)) is False
    assert a.forward(CLIMB, None) is False
    assert sent == []


def test_granting_an_unknown_owner_is_an_error():
    with pytest.raises(ValueError):
        JointArbiter().grant("dagger")


# --------------------------------------------------------------- launch wiring


def test_launch_file_parses():
    ast.parse(LAUNCH_FILE.read_text())


def test_the_router_is_off_by_default():
    """The scored run must be exactly what it was before the router existed."""
    source = LAUNCH_FILE.read_text()
    window = source[source.index('"strategy_router",') :][:400]
    assert 'default_value="false"' in window


def test_the_follower_moves_rather_than_duplicating_its_output():
    """With the router on, /cmd_vel must have one publisher, not two."""
    source = LAUNCH_FILE.read_text()
    assert "follower_cmd_topic = PythonExpression(" in source
    assert '"cmd_vel_topic": follower_cmd_topic' in source
    assert "/strategy/nav_cmd_vel" in source
    # The router node is conditional; the follower is not. If that were reversed, turning the
    # router off would leave nothing publishing /cmd_vel at all.
    router_block = source[source.index('executable="strategy_router"') :][:400]
    assert "condition=IfCondition(router)" in router_block


def test_the_follower_default_topic_is_unchanged():
    assert 'self.declare_parameter("cmd_vel_topic", "/cmd_vel")' in FOLLOWER.read_text()


def test_strategy_config_ships_and_keeps_climb_off():
    params = yaml.safe_load(STRATEGY_YAML.read_text())["strategy_router"]["ros__parameters"]
    assert params["climb_enabled"] is False
    assert tuple(params["climb_segment"]) == SEGMENT
    assert params["cmd_vel_topic"] == "/cmd_vel"
    assert params["nav_cmd_topic"] == "/strategy/nav_cmd_vel"


def test_the_segment_mapping_is_not_hard_coded_in_the_nodes():
    """WP16 must be a config value, not a literal scattered through the nodes.

    Not a subtle failure, but a slow one: the same waypoint test written into three files,
    two of which get updated and one of which is the one that runs.
    """
    for path in (FOLLOWER, ROUTER_NODE):
        source = path.read_text()
        for literal in ("== 16", "== (15, 16)", "waypoint == 15"):
            assert literal not in source, f"{path.name} hard codes {literal!r}"


# --------------------------------------------------------------- both interfaces


def _router_in_climb(kind: ActionKind):
    """A router sitting in CLIMB with a policy that will not finish, and the time there."""
    from test_router import run_to_climb

    policy = MockClimbPolicy(MockScenario.NEVER_FINISH, action_kind=kind)
    router = Router(
        RouterConfig(),
        policies={"climb_policy": policy},
        segment_policies={SEGMENT: "climb_policy"},
    )
    return router, run_to_climb(router)


def test_a_twist_policy_reaches_the_boundary_as_a_twist():
    router, t = _router_in_climb(ActionKind.TWIST)
    s = state(t, obstacle_distance=0.6)
    out = router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
    assert out.source is Source.POLICY
    assert out.command is not None
    assert out.joints is None


def test_a_joint_policy_reaches_the_boundary_as_sixteen_joints():
    router, t = _router_in_climb(ActionKind.JOINT)
    s = state(t, obstacle_distance=0.6)
    out = router.tick(s, (0.5, 0.0, 0.0), observation_from_state(s))
    assert out.source is Source.POLICY
    assert out.joints is not None
    assert np.shape(out.joints) == (16,)
    # Emphatically not converted into a body velocity. The manoeuvre is not expressible as
    # one, which is the whole reason it needed a trajectory, and a silent conversion would
    # produce a plausible command that does something else.
    assert out.command is None


def test_navigation_owns_nothing_while_a_joint_policy_is_driving():
    router, t = _router_in_climb(ActionKind.JOINT)
    for _ in range(50):
        s = state(t, obstacle_distance=0.6)
        out = router.tick(s, (9.9, 9.9, 9.9), observation_from_state(s))
        assert out.source is Source.POLICY
        assert out.command is None
        t += 0.02


# --------------------------------------------------------------- scripted baseline


def test_the_scripted_policy_is_labelled_a_baseline_where_it_is_read():
    policy = ScriptedClimbPolicy(hold_trajectory(np.zeros(16), 0.2))
    assert policy.is_scripted_baseline is True
    assert policy.result().info["scripted_baseline"] is True
    assert policy.action_kind is ActionKind.JOINT


def test_the_scripted_policy_refuses_a_pose_it_was_not_recorded_from():
    """Replaying from the wrong pose is a torque step, not merely a poor attempt."""
    policy = ScriptedClimbPolicy(hold_trajectory(np.zeros(16), 0.2))
    s = state(0.0, joint_positions=np.full(16, 1.5))
    policy.start(observation_from_state(s))
    assert policy.is_finished()
    assert "entry gap" in policy.result().reason


def test_the_scripted_policy_accepts_the_pose_it_was_recorded_from():
    policy = ScriptedClimbPolicy(hold_trajectory(np.zeros(16), 0.2))
    s = state(0.0, joint_positions=np.zeros(16))
    policy.start(observation_from_state(s))
    assert not policy.is_finished()


def test_the_scripted_policy_clips_to_the_joint_limits():
    policy = ScriptedClimbPolicy(np.full((5, 16), 9.0))
    assert policy.clipped_frames == 5
    assert float(policy.trajectory[:, 0].max()) <= 0.60
    assert float(policy.trajectory[:, 1].max()) <= 2.50


# --------------------------------------------------------------- the node itself

# Not `pytest.importorskip` at module scope: that would skip this whole file, including the
# arbiter and launch checks, on any machine without ROS -- which is exactly where a silent
# skip is most likely to be mistaken for a pass.
try:
    import rclpy
except ImportError:  # pragma: no cover - depends on the environment, not the code
    rclpy = None

needs_ros = pytest.mark.skipif(rclpy is None, reason="ROS 2 is not installed here")


@pytest.fixture
def ros():
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


@needs_ros
def test_the_router_node_is_the_only_publisher_of_cmd_vel(ros):
    from s10_auto_nav.strategy_router_node import StrategyRouterNode

    node = StrategyRouterNode()
    try:
        names = dict(node.get_topic_names_and_types())
        for topic in ("/cmd_vel", "/strategy/mode", "/strategy/status", "/strategy/source"):
            assert topic in names, f"{topic} not advertised"
        assert node.count_publishers("/cmd_vel") == 1
    finally:
        node.destroy_node()


@needs_ros
def test_the_router_node_shuts_down_cleanly(ros):
    from s10_auto_nav.strategy_router_node import StrategyRouterNode

    StrategyRouterNode().destroy_node()
