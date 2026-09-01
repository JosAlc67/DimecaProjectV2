"""One-shot utility: sends the arm back to the "home" pose (all joints at 0,
matching config/irb2600.srdf's group_state "home") via a single MoveGroup
action call (plan + execute together), so you don't have to restart the
whole coating_cell_bringup.launch.py just to get a clean starting state
between manual test runs.

    ros2 run irb2600_coating_cell go_home_node
"""

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.node import Node

from irb2600_coating_cell.stoppable import StoppableActionNode

_ARM_JOINTS = ["track_prismatic_joint", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
_HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# Deliberately modest, collision-checked pose for mock-hardware simulation.
_DEMO_JOINTS = [0.30, -0.35, -0.55, 0.35, 0.0, 0.35, 0.0]


class GoHomeNode(StoppableActionNode, Node):

    def __init__(self, **kwargs):
        super().__init__("go_home_node", **kwargs)
        self._init_stoppable()
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("planning_time_s", 3.0)

        self._client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for /move_action (move_group)...")
        self._client.wait_for_server()

    def go_home(self):
        """Plan+execute a return to the "home" pose (all joints = 0). Public
        so callers other than main() (e.g. the Tkinter GUI's background
        thread) can reuse this node instance directly. Can be interrupted
        by request_stop() from another thread."""
        return self._move_to_joint_target("home", _HOME_JOINTS)

    def move_demo(self):
        """Run a short visible, collision-checked movement in simulation."""
        return self._move_to_joint_target("demo", _DEMO_JOINTS)

    def _move_to_joint_target(self, target_name, joint_positions):
        """Plan and execute one named joint target through MoveIt."""
        self._clear_stop()
        constraints = Constraints()
        for joint_name, joint_position in zip(_ARM_JOINTS, joint_positions):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(joint_position)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.get_parameter("group_name").value
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.request.allowed_planning_time = float(
            self.get_parameter("planning_time_s").value
        )
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.workspace_parameters.header.frame_id = "world"
        goal.request.workspace_parameters.min_corner.x = -3.0
        goal.request.workspace_parameters.min_corner.y = -3.0
        goal.request.workspace_parameters.min_corner.z = -3.0
        goal.request.workspace_parameters.max_corner.x = 3.0
        goal.request.workspace_parameters.max_corner.y = 3.0
        goal.request.workspace_parameters.max_corner.z = 3.0
        # plan_only=False: move_group plans AND executes in one action call,
        # no separate /execute_trajectory step needed for this simple case.
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True

        self.get_logger().info(f"Planning path to {target_name} joint target...")
        
        max_attempts = 5
        attempts = 0
        while rclpy.ok() and not self._stop_requested() and attempts < max_attempts:
            attempts += 1
            if attempts > 1:
                self.get_logger().info(
                    f"--- Attempt {attempts}/{max_attempts} to reach {target_name} ---"
                )
                
            result_response = self._send_goal_and_wait(self._client, goal)

            if result_response is None:
                self.get_logger().error("move_action goal rejected.")
                break
            elif self._succeeded(result_response):
                self.get_logger().info(f"{target_name.capitalize()} target reached.")
                return True
            elif self._cancelled(result_response):
                self.get_logger().warn(f"{target_name.capitalize()} motion cancelled (Stop).")
                return False
            else:
                error_code = result_response.result.error_code.val
                if error_code in [-26, -27]:
                    # START_STATE_INVALID or GOAL_STATE_INVALID
                    self.get_logger().error(
                        f"Failed to reach {target_name} (error_code={error_code}). "
                        "Make sure obstacles are not colliding with the robot's current or target position!"
                    )
                    break
                else:
                    self.get_logger().warn(
                    f"Failed to plan/execute path to {target_name} (error_code={error_code}). Retrying..."
                    )
                    
        self.get_logger().error(f"Could not reach {target_name} after multiple attempts.")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = GoHomeNode()
    node.go_home()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
