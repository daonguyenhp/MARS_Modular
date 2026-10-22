from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("mars_paper_rviz")
    return LaunchDescription([
        DeclareLaunchArgument("playback_hz", default_value="1.5"),
        DeclareLaunchArgument("run_rviz", default_value="true"),
        Node(
            package="mars_paper_rviz",
            executable="paper_rviz_node",
            name="paper_rviz",
            output="screen",
            parameters=[{"playback_hz": ParameterValue(
                LaunchConfiguration("playback_hz"), value_type=float)}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="paper_rviz_view",
            output="screen",
            arguments=["-d", os.path.join(share, "rviz", "paper_fig8_fig9.rviz")],
            condition=IfCondition(LaunchConfiguration("run_rviz")),
        ),
    ])
