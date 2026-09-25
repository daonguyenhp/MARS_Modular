from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("mars_mode1_sim")
    rviz_config = os.path.join(share, "rviz", "mode1.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="hard_alley"),
        DeclareLaunchArgument("run_rviz", default_value="true"),
        Node(
            package="mars_mode1_sim",
            executable="mode1_rviz_node",
            name="mode1_rviz",
            output="screen",
            parameters=[{
                "map_id": LaunchConfiguration("map"),
                "playback_hz": 2.0,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("run_rviz")),
        ),
    ])
