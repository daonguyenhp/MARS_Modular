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
    tb3_share = Path(get_package_share_directory("turtlebot3_gazebo"))
    tb3_model = tb3_share / "models" / "turtlebot3_burger" / "model.sdf"
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {world}"}.items(),
    )
    robot = Node(
        package="ros_gz_sim", executable="create", name="turtlebot3_spawn",
        arguments=[
            "-name", "turtlebot3_burger", "-file", str(tb3_model),
            "-x", start_x, "-y", start_y, "-z", "0.01",
        ], output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="paper_bridge",
        parameters=[{"config_file": str(share / "launch" / "bridge.yaml")}],
        output="screen",
    )
    navigation = Node(
        package="mars_online_gazebo", executable="online_navigation_node",
        parameters=[{"use_sim_time": True, "goal_x": goal_x, "goal_y": goal_y}],
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
        DeclareLaunchArgument("start_x", default_value="3.4"),
        DeclareLaunchArgument("start_y", default_value="6.0"),
        DeclareLaunchArgument("goal_x", default_value="8.6"),
        DeclareLaunchArgument("goal_y", default_value="6.8"),
        gazebo, robot, bridge, navigation, rviz,
    ])
