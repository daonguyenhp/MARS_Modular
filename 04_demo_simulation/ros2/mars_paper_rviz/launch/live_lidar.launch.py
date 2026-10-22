from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    share = get_package_share_directory("mars_paper_rviz")
    return LaunchDescription([
        DeclareLaunchArgument("run_rviz", default_value="true"),
        DeclareLaunchArgument("speed", default_value="8.0"),
        Node(
            package="mars_paper_rviz",
            executable="live_lidar_node",
            name="live_lidar_robot",
            output="screen",
            parameters=[{"speed": ParameterValue(
                LaunchConfiguration("speed"), value_type=float)}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="live_lidar_view",
            output="screen",
            arguments=["-d", os.path.join(share, "rviz", "paper_fig8_fig9.rviz")],
            condition=IfCondition(LaunchConfiguration("run_rviz")),
        ),
    ])
