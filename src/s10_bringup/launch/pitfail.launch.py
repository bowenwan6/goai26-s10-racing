"""One approach to one obstacle, on the production stack, with the arrival state recorded.

This is ``race.launch.py`` plus a recorder, and it is the same shape as ``segment.launch.py``
for the same reason: the race launch is *included*, not restated, so the simulator, the
perception node, the follower -- with its local planner, step commit and stall recovery --
and the strategy router are the same actions with the same parameter files a scored run gets.
The question this harness exists to answer is "what state does the production stack arrive in",
and every difference between this launch and the scored one is a way of getting the wrong
answer to it.

What differs, and nothing else does:

* the course is a slice of the real one, written per run by ``scripts/run_pitfail.py``.
  ``course.yaml`` is generated and is never edited;
* the spawn is the obstacle's ``approach_from`` waypoint, via ``S10_SPAWN_XY`` and
  ``S10_SPAWN_Z`` in the environment. Both, always. See :mod:`s10_perception.segment_spawn`:
  the course runs over itself and a spawn given only (x, y) picks the storey underneath, which
  is how thirty-six runs were once collected of a robot boxed in below the deck it was meant
  to be driving on;
* ``pitfail_recorder`` replaces ``segment_recorder``. It measures against the obstacle frame
  rather than against the end waypoint, and it ends the run.

``rl_deploy`` is not launched here, for the reason ``race.launch.py`` does not launch it: it
has to run from the SDK overlay. ``scripts/run_pitfail.py`` starts it alongside, first.

**Shadow mode.** Experiment A needs the router to watch an otherwise untouched run: it arms,
it aligns, it evaluates the entry gate and it logs every transition, but the command that
reaches the robot stays the follower's. That is ``router_shadow``, and it is a property of the
router rather than of this file -- see ``RouterConfig.shadow``. Switching the router off
instead would have been easier and would have measured nothing.

**The gate parameters** are launch arguments so the experiment can sweep the entry band
without a code change. They reach the router through ``SetParameter`` rather than through a
parameter file because ``race.launch.py`` gives the router one file and that file is the
production one. Global parameters are applied before a node's own parameter list, so anything
named in ``router_params`` would win; none of the four names below appears in ``strategy.yaml``
or in any file this harness writes, and that is deliberate -- one value, one place.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _typed(name: str, kind: type) -> ParameterValue:
    """A launch argument with its type spelled out.

    Launch arguments arrive as strings and the router declares these as a bool and three
    floats. Left to guess, a parameter override of the wrong type is not a warning -- the
    node raises at ``declare_parameter`` and the run dies during bring-up.
    """
    return ParameterValue(LaunchConfiguration(name), value_type=kind)


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("s10_bringup")

    course_file = LaunchConfiguration("course_file")
    out_root = LaunchConfiguration("out_root")
    run_id = LaunchConfiguration("run_id")
    obstacle_id = LaunchConfiguration("obstacle_id")

    recorder = Node(
        package="s10_auto_nav",
        executable="pitfail_recorder",
        name="pitfail_recorder",
        output="screen",
        parameters=[
            {
                "course_file": course_file,
                "out_root": out_root,
                "run_id": run_id,
                "obstacle_id": obstacle_id,
                "obstacles_file": LaunchConfiguration("obstacles_file"),
                "max_time": _typed("max_time", float),
                "grace": _typed("grace", float),
                "reach_radius": _typed("reach_radius", float),
                "stop_before_collision_distance": _typed("stop_before_collision_distance", float),
                "ready_distance_min": _typed("ready_distance_min", float),
                "ready_distance_max": _typed("ready_distance_max", float),
                "max_entry_yaw_rate": _typed("max_entry_yaw_rate", float),
                # Not in the recorder's contract; passed so the label a run was collected
                # under travels with the run rather than being reconstructed from its id.
                # A recorder that has not declared them ignores them.
                "experiment": LaunchConfiguration("experiment"),
                "label": LaunchConfiguration("label"),
                "seed": _typed("seed", int),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("course_file", description="Sub-course to follow."),
            DeclareLaunchArgument("out_root", default_value="/res/pitfail"),
            DeclareLaunchArgument("run_id", default_value="pitfail"),
            DeclareLaunchArgument("obstacle_id", default_value=""),
            DeclareLaunchArgument(
                "obstacles_file", default_value="/res/pitfail/config/obstacles.yaml"
            ),
            DeclareLaunchArgument(
                "experiment",
                default_value="A",
                description="A natural arrival, B router handoff, C restored snapshot.",
            ),
            DeclareLaunchArgument(
                "label",
                default_value="",
                description=(
                    "Free text carried into the run's own artefacts. Experiment B sets it to "
                    "'gate measurement, no climb attempted': no climb policy exists, and a "
                    "run that reaches CLIMB_READY has measured a gate, not a climb."
                ),
            ),
            DeclareLaunchArgument("seed", default_value="0"),
            DeclareLaunchArgument("max_time", default_value="180.0"),
            DeclareLaunchArgument("grace", default_value="3.0"),
            DeclareLaunchArgument("reach_radius", default_value="0.18"),
            DeclareLaunchArgument(
                "stop_before_collision_distance",
                default_value="0.15",
                description=(
                    "Metres short of the edge at which the recorder ends the run. The arrival "
                    "state is the measurement; driving into the face afterwards adds nothing "
                    "and costs a wedged robot the next run has to be started around."
                ),
            ),
            DeclareLaunchArgument(
                "nav_params",
                default_value=PathJoinSubstitution([bringup_share, "config", "nav.yaml"]),
            ),
            DeclareLaunchArgument(
                "router_params",
                default_value=PathJoinSubstitution([bringup_share, "config", "strategy.yaml"]),
                description=(
                    "Router parameter file. run_pitfail.py generates one per obstacle so the "
                    "router arms on the segment under test; the default is the production "
                    "file, under which the router is a pass-through."
                ),
            ),
            DeclareLaunchArgument(
                "strategy_router",
                default_value="true",
                description="Both experiments need the router present, if only to watch.",
            ),
            DeclareLaunchArgument(
                "router_shadow",
                default_value="false",
                description=(
                    "Run the state machine but leave the follower's command alone. True for "
                    "experiment A, false for experiment B."
                ),
            ),
            DeclareLaunchArgument(
                "ready_distance_min",
                default_value="0.45",
                description="Entry gate distance band, metres. Provisional; being measured.",
            ),
            DeclareLaunchArgument("ready_distance_max", default_value="0.70"),
            DeclareLaunchArgument(
                "max_entry_yaw_rate",
                default_value="0.10",
                description="Entry gate yaw-rate limit, rad/s. Provisional; being measured.",
            ),
            GroupAction(
                [
                    SetParameter(name="router_shadow", value=_typed("router_shadow", bool)),
                    SetParameter(
                        name="ready_distance_min", value=_typed("ready_distance_min", float)
                    ),
                    SetParameter(
                        name="ready_distance_max", value=_typed("ready_distance_max", float)
                    ),
                    SetParameter(
                        name="max_entry_yaw_rate",
                        value=_typed("max_entry_yaw_rate", float),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([bringup_share, "launch", "race.launch.py"])
                        ),
                        launch_arguments={
                            "course_file": course_file,
                            "nav_params": LaunchConfiguration("nav_params"),
                            "router_params": LaunchConfiguration("router_params"),
                            "strategy_router": LaunchConfiguration("strategy_router"),
                        }.items(),
                    ),
                ],
                scoped=True,
            ),
            recorder,
            # The recorder's exit ends the run, and the run ending takes the simulator with
            # it. Without this an unattended sweep leaves one MuJoCo context per run alive,
            # and the second run discovers what that does to the first one's robot.
            RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=[Shutdown()])),
        ]
    )
