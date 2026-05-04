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
                "params_file",
                default_value=params_file,
                description="Optional YAML parameter file for laserscan_multi_merger.",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="ira_static_broadcaster1",
                arguments="--yaw 0.3 --frame-id base_link --child-frame-id scan1".split(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="ira_static_broadcaster2",
                arguments="--frame-id base_link --child-frame-id scan2".split(),
            ),
            Node(
                package="ira_laser_tools",
                executable="laserscan_virtualizer",
                name="laserscan_virtualizer",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                ],
            ),
        ]
    )
