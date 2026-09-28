import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share = get_package_share_directory("mono_camera_calibration")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(share, "config", "calibration.yaml")),
        DeclareLaunchArgument("gui", default_value="true"),
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container_mt",
            name="mono_calibration_container",
            namespace="/",
            output="screen",
            emulate_tty=True,
            composable_node_descriptions=[ComposableNode(
                package="mono_camera_calibration",
                plugin="mono_camera_calibration::MonoCalibrationNode",
                namespace="/mono_calibration",
                name="calibrator",
                parameters=[LaunchConfiguration("config")],
                extra_arguments=[{"use_intra_process_comms": True}],
            )],
        ),
        Node(
            package="mono_camera_calibration",
            executable="mono_calibration_gui",
            name="mono_calibration_gui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
    ])
