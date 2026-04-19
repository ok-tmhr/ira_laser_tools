import launch
from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    params_file = (
        get_package_share_path("ira_laser_tools") / "config" / "ira_laser_tools.yaml"
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
                description="ROS logger level for the container.",
            ),
            DeclareLaunchArgument(
                "ns",
                default_value="",
                description="Namespace to launch the component into.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file.as_posix(),
                description="Optional YAML parameter file for LaserscanMerger.",
            ),
            ComposableNodeContainer(
                name="laserscan_container",
                package="rclcpp_components",
                executable="component_container",
                namespace="",
                composable_node_descriptions=[
                    ComposableNode(
                        package="ira_laser_tools",
                        plugin="ira_laser_tools::LaserscanMerger",
                        name="laserscan_merger",
                        namespace=LaunchConfiguration("ns"),
                        parameters=[
                            LaunchConfiguration("params_file"),
                            {"use_sim_time": LaunchConfiguration("use_sim_time")},
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    )
                ],
                ros_arguments=["--log-level", LaunchConfiguration("log_level")],
            ),
        ]
    )
