"""Standalone visualization of the IRB 2600 + spray nozzle, no MoveIt.

Useful as the very first smoke test after building the workspace: it only
needs robot_state_publisher, joint_state_publisher_gui and RViz, so it is the
fastest way to confirm the xacro/meshes are valid before bringing up MoveIt.

    ros2 launch irb2600_description display.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory("irb2600_description")

    xacro_path = PathJoinSubstitution(
        [description_pkg, "urdf", "irb2600.urdf.xacro"]
    )
    default_rviz_config = os.path.join(description_pkg, "rviz", "view_robot.rviz")

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Path to the RViz config file to use.",
    )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_path]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
