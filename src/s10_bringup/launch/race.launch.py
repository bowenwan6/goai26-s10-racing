"""Full autonomous racing run.

Brings up the perception simulator and the waypoint follower. The locomotion policy
(`rl_deploy`, from the contest SDK) is launched separately because it must run in the
upstream workspace overlay; see the README for the two-terminal procedure or use
`scripts/run_race.sh`, which sequences all three.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("s10_bringup")

    course_file = LaunchConfiguration("course_file")
    nav_params = LaunchConfiguration("nav_params")
    router_params = LaunchConfiguration("router_params")
    mjcf = LaunchConfiguration("mjcf")
    launch_sim = LaunchConfiguration("launch_sim")
    viz = LaunchConfiguration("viz")
    rviz = LaunchConfiguration("rviz")
    router = LaunchConfiguration("strategy_router")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # The follower's output moves rather than being duplicated. With the router on it feeds
    # /strategy/nav_cmd_vel and the router is the only publisher of /cmd_vel; with the router
    # off the follower publishes /cmd_vel exactly as before. Expressed as one substitution
    # instead of two conditional Node blocks so that "who publishes /cmd_vel" has a single
    # answer visible in one place.
    follower_cmd_topic = PythonExpression(
        ["'/strategy/nav_cmd_vel' if '", router, "' == 'true' else '/cmd_vel'"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "course_file",
                default_value=PathJoinSubstitution([bringup_share, "config", "course.yaml"]),
                description="Waypoint course to follow.",
            ),
            DeclareLaunchArgument(
                "nav_params",
                default_value=PathJoinSubstitution([bringup_share, "config", "nav.yaml"]),
                description="Waypoint follower parameter file.",
            ),
            DeclareLaunchArgument(
                "router_params",
                default_value=PathJoinSubstitution([bringup_share, "config", "strategy.yaml"]),
                description="Strategy router parameter file.",
            ),
            DeclareLaunchArgument(
                "mjcf",
                default_value="",
                description="Custom MJCF path. Empty uses the upstream track scene.",
            ),
            DeclareLaunchArgument(
                "launch_sim",
                default_value="true",
                description="Set false to attach to a simulator that is already running.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use MuJoCo /clock for navigation and strategy timing.",
            ),
            Node(
                package="s10_perception",
                executable="sim_node",
                name="mujoco_simulation",
                output="screen",
                arguments=["--xml-path", mjcf],
                condition=IfCondition(launch_sim),
            ),
            DeclareLaunchArgument(
                "viz",
                default_value="false",
                description=(
                    "Publish TF and PointCloud2 views of the perception topics. Off by "
                    "default so a scored run carries no display cost."
                ),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Also open RViz on the bundled config. Implies viz:=true.",
            ),
            DeclareLaunchArgument(
                "strategy_router",
                default_value="false",
                description=(
                    "Insert the strategy router between the follower and /cmd_vel. Off by "
                    "default: a scored run must behave exactly as it did before this "
                    "existed. On, the follower publishes /strategy/nav_cmd_vel and the "
                    "router owns /cmd_vel."
                ),
            ),
            Node(
                package="s10_auto_nav",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[
                    nav_params,
                    {
                        "course_file": course_file,
                        "cmd_vel_topic": follower_cmd_topic,
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
            ),
            Node(
                package="s10_auto_nav",
                executable="strategy_router",
                name="strategy_router",
                output="screen",
                parameters=[
                    router_params,
                    {
                        "course_file": course_file,
                        "nav_cmd_topic": "/strategy/nav_cmd_vel",
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
                condition=IfCondition(router),
            ),
            Node(
                package="s10_perception",
                executable="viz_node",
                name="perception_viz",
                output="screen",
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                # Launch arguments arrive as the strings "true"/"false", not as booleans,
                # so the comparison has to be spelled out rather than written as `viz or
                # rviz` -- which would be true for the string "false" as well.
                condition=IfCondition(
                    PythonExpression(["'", viz, "' == 'true' or '", rviz, "' == 'true'"])
                ),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution([bringup_share, "rviz", "perception.rviz"]),
                ],
                condition=IfCondition(rviz),
            ),
        ]
    )
