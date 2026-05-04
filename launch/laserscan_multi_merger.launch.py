import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = (
        FindPackageShare("ira_laser_tools") / "config" / "ira_laser_tools.yaml"
    )
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulated clock if true.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS logger level for the node.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace to launch the node into.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Optional YAML parameter file for laserscan_multi_merger.",
            ),
            Node(
                package="ira_laser_tools",
                executable="laserscan_multi_merger",
                name="laserscan_multi_merger",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                ros_arguments=["--log-level", LaunchConfiguration("log_level")],
            ),
        ]
    )
