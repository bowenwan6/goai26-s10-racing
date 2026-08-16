"""One leg of the course, on the production stack.

This is ``race.launch.py`` plus a recorder, and deliberately nothing else. It includes the
race launch rather than restating it, so the simulator, the perception node, the waypoint
follower -- with its local planner, step commit and stall recovery -- and the strategy
router are the same actions with the same parameter files that a scored run gets. The two
differences are both outside those nodes:

* the course handed to the follower is a slice of the real one, written to its own file by
  ``scripts/run_segment.py``. ``course.yaml`` is generated and is never edited;
* the simulator spawns at the segment's first waypoint, via ``S10_SPAWN_XY`` in the
  environment. See :mod:`s10_perception.segment_spawn`.

``rl_deploy`` is not launched here for the same reason ``race.launch.py`` does not launch
it: it has to run from the SDK overlay. ``scripts/run_segment.py`` starts it alongside.

The recorder's exit ends the run. That is what makes an unattended sweep possible: the
recorder decides the segment is over -- reached, fallen, or out of time -- writes its CSV
and JSON, and exits, and the shutdown handler below takes the rest of the stack down with
it rather than leaving a simulator running into the next run.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("s10_bringup")

    course_file = LaunchConfiguration("course_file")
    out_dir = LaunchConfiguration("out_dir")
    run_name = LaunchConfiguration("run_name")

    recorder = Node(
        package="s10_auto_nav",
        executable="segment_recorder",
        name="segment_recorder",
        output="screen",
        parameters=[
            {
                "course_file": course_file,
                "out_dir": out_dir,
                "run_name": run_name,
                "start_waypoint": LaunchConfiguration("start_waypoint"),
                "end_waypoint": LaunchConfiguration("end_waypoint"),
                "seed": LaunchConfiguration("seed"),
                "max_time": LaunchConfiguration("max_time"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("course_file", description="Sub-course to follow."),
            DeclareLaunchArgument("out_dir", default_value="/tmp/s10_segment"),
            DeclareLaunchArgument("run_name", default_value="segment"),
            DeclareLaunchArgument("start_waypoint", default_value="0"),
            DeclareLaunchArgument("end_waypoint", default_value="1"),
            DeclareLaunchArgument("seed", default_value="0"),
            DeclareLaunchArgument("max_time", default_value="120.0"),
            DeclareLaunchArgument(
                "nav_params",
                default_value=PathJoinSubstitution([bringup_share, "config", "nav.yaml"]),
            ),
            DeclareLaunchArgument("strategy_router", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "race.launch.py"])
                ),
                launch_arguments={
                    "course_file": course_file,
                    "nav_params": LaunchConfiguration("nav_params"),
                    "strategy_router": LaunchConfiguration("strategy_router"),
                }.items(),
            ),
            recorder,
            RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=[Shutdown()])),
        ]
    )
