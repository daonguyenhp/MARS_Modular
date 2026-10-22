"""Gazebo physics + bridged sensors + online paper algorithm + RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("mars_online_gazebo"))
    gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = share / "worlds" / "paper_alley.sdf"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {world}"}.items(),
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="paper_bridge",
        parameters=[{"config_file": str(share / "launch" / "bridge.yaml")}],
        output="screen",
    )
    navigation = Node(
        package="mars_online_gazebo", executable="online_navigation_node",
        parameters=[{"use_sim_time": True, "goal_x": 8.6, "goal_y": 6.8}],
        output="screen",
    )
    rviz = Node(
        package="rviz2", executable="rviz2", output="screen",
        arguments=["-d", str(share / "rviz" / "online_paper_alley.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        gazebo, bridge, navigation, rviz,
    ])
