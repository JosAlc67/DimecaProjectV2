import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def _moveit_config():
    urdf_xacro_path = os.path.join(
        get_package_share_directory("irb2600_description"),
        "urdf",
        "irb2600.urdf.xacro",
    )
    return (
        MoveItConfigsBuilder("irb2600", package_name="irb2600_moveit_config")
        .robot_description(file_path=urdf_xacro_path)
        .robot_description_semantic(file_path="config/irb2600.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .pilz_cartesian_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )


def generate_launch_description():
    return generate_move_group_launch(_moveit_config())
