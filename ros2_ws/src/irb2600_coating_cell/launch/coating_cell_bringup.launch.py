"""Full Phase-1 demo: robot + MoveIt 2 (mock hardware, RViz) + static target
structure/obstacle in the planning scene + simulated perception + spray_on
control service.

    ros2 launch irb2600_coating_cell coating_cell_bringup.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config_pkg = get_package_share_directory("irb2600_moveit_config")
    coating_cell_pkg = get_package_share_directory("irb2600_coating_cell")
    scene_objects_yaml = os.path.join(coating_cell_pkg, "config", "scene_objects.yaml")

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, "launch", "demo.launch.py")
        )
    )

    scene_setup_node = Node(
        package="irb2600_coating_cell",
        executable="scene_setup_node",
        name="scene_setup_node",
        output="screen",
        parameters=[scene_objects_yaml],
    )

    perception_sim_node = Node(
        package="irb2600_coating_cell",
        executable="perception_sim_node",
        name="perception_sim_node",
        output="screen",
        parameters=[scene_objects_yaml],
    )

    spray_controller_node = Node(
        package="irb2600_coating_cell",
        executable="spray_controller_node",
        name="spray_controller_node",
        output="screen",
    )

    return LaunchDescription(
        [
            moveit_demo,
            scene_setup_node,
            perception_sim_node,
            spray_controller_node,
        ]
    )
