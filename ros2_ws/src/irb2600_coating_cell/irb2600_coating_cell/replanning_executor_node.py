"""Point-to-Point (Pick and Place) executor with reactive replanning.
Si el robot encuentra un obstáculo dinámico en su camino, detendrá la ejecución
y recalculará automáticamente una nueva ruta 3D para rodearlo hasta llegar al destino.
"""

import time
import math
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from rclpy.node import Node

from irb2600_coating_cell.geometry_utils import quaternion_with_z_axis, rotate_vector_by_quaternion
from irb2600_coating_cell.stoppable import StoppableActionNode

# MoveIt error codes
SUCCESS = 1
PREEMPTED = -2
CONTROL_FAILED = -4
NO_IK_SOLUTION = -31
TIMED_OUT = -3

class PickAndPlaceExecutorNode(StoppableActionNode, Node):

    def __init__(self, **kwargs):
        super().__init__("replanning_executor_node", **kwargs)
        self._init_stoppable()

        # Target point configuration
        self.declare_parameter("target_structure.frame_id", "world")
        self.declare_parameter("target_structure.position", [1.8, 0.0, 1.0])
        self.declare_parameter("target_structure.local_normal", [-1.0, 0.0, 0.0])
        self.declare_parameter("d_standoff", 0.20)
        
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("tcp_link", "nozzle_tip")
        self.declare_parameter("execute", False)
        self.declare_parameter("replanning_time_s", 10.0)
        self.declare_parameter("replanning_attempts", 10)

        self._latest_target_pose = None
        self.create_subscription(PoseStamped, "structure_pose", self._on_structure_pose, 10)

        self._move_group_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for /move_action (move_group)...")
        self._move_group_client.wait_for_server()

    def _on_structure_pose(self, msg):
        self._latest_target_pose = msg

    def run_to_target(self):
        self._clear_stop()
        p = self.get_parameter
        
        self.get_logger().info("Waiting for /structure_pose from perception_sim_node...")
        # Wait up to 2 seconds for a message from perception_sim_node
        elapsed = 0.0
        while self._latest_target_pose is None and rclpy.ok() and not self._stop_requested() and elapsed < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed += 0.1

        if self._latest_target_pose:
            frame_id = self._latest_target_pose.header.frame_id
            pos = [
                self._latest_target_pose.pose.position.x,
                self._latest_target_pose.pose.position.y,
                self._latest_target_pose.pose.position.z,
            ]
            target_quat = self._latest_target_pose.pose.orientation
            normal = p("target_structure.local_normal").value
            # Rotate normal if the target structure was rotated
            normal = rotate_vector_by_quaternion(normal, target_quat)
        else:
            self.get_logger().warn("Did not receive /structure_pose, falling back to static parameters.")
            frame_id = p("target_structure.frame_id").value
            pos = p("target_structure.position").value
            normal = p("target_structure.local_normal").value
            
        d_standoff = p("d_standoff").value
        
        target_pose = Pose()
        target_pose.position.x = pos[0] + normal[0] * d_standoff
        target_pose.position.y = pos[1] + normal[1] * d_standoff
        target_pose.position.z = pos[2] + normal[2] * d_standoff
        
        # Point the tool towards the target
        approach_direction = tuple(-c for c in normal)
        target_pose.orientation = quaternion_with_z_axis(approach_direction)
        
        self.get_logger().info(
            f"Target established at x={target_pose.position.x:.2f}, "
            f"y={target_pose.position.y:.2f}, z={target_pose.position.z:.2f}"
        )

        attempts = 0
        max_attempts = 10

        while rclpy.ok() and not self._stop_requested() and attempts < max_attempts:
            attempts += 1
            self.get_logger().info(f"--- Attempt {attempts}/{max_attempts} to reach target ---")
            
            goal = self._build_move_goal(target_pose, frame_id)
            
            self.get_logger().info("Planning and executing 3D joint-space path...")
            result = self._send_goal_and_wait(self._move_group_client, goal)
            
            if result is None:
                self.get_logger().error("Action server rejected the goal.")
                break
                
            error_code = result.result.error_code.val
            
            if error_code == SUCCESS:
                self.get_logger().info("SUCCESS! Target point reached safely.")
                return True
            
            elif error_code in [PREEMPTED, CONTROL_FAILED]:
                self.get_logger().warn(
                    f"Execution blocked by a dynamic obstacle (error_code={error_code})! "
                    "Replanning a new bypass route from current position..."
                )
                # Wait briefly before replanning to allow the Octomap to settle
                self._interruptible_sleep(1.0)
                continue
                
            elif error_code == NO_IK_SOLUTION:
                self.get_logger().error(
                    "Target point is completely unreachable (likely physically inside an obstacle). "
                    "Aborting."
                )
                break
                
            else:
                self.get_logger().warn(
                    f"Failed to plan/execute path (error_code={error_code}). Retrying..."
                )
                self._interruptible_sleep(1.0)
                continue

        self.get_logger().error("Could not reach the target after multiple attempts or fatal error.")
        return False

    def _build_move_goal(self, target_pose, frame_id):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.get_parameter("group_name").value
        goal.request.num_planning_attempts = int(self.get_parameter("replanning_attempts").value)
        goal.request.allowed_planning_time = float(self.get_parameter("replanning_time_s").value)
        
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        
        goal.request.workspace_parameters.header.frame_id = "world"
        goal.request.workspace_parameters.min_corner.x = -3.0
        goal.request.workspace_parameters.min_corner.y = -3.0
        goal.request.workspace_parameters.min_corner.z = -3.0
        goal.request.workspace_parameters.max_corner.x = 3.0
        goal.request.workspace_parameters.max_corner.y = 3.0
        goal.request.workspace_parameters.max_corner.z = 3.0
        
        # If execute:=false, just plan. If true, plan AND execute in one step!
        goal.planning_options.plan_only = not self.get_parameter("execute").value
        goal.planning_options.planning_scene_diff.is_diff = True
        
        # Setup constraints
        constraint = Constraints()
        
        pc = PositionConstraint()
        pc.header.frame_id = frame_id
        pc.link_name = self.get_parameter("tcp_link").value
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01] # 1cm tolerance
        bv.primitives.append(sp)
        bv.primitive_poses.append(target_pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        
        oc = OrientationConstraint()
        oc.header.frame_id = frame_id
        oc.link_name = self.get_parameter("tcp_link").value
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        
        constraint.position_constraints.append(pc)
        constraint.orientation_constraints.append(oc)
        
        goal.request.goal_constraints = [constraint]
        
        return goal

    def _interruptible_sleep(self, duration_s):
        elapsed = 0.0
        while elapsed < duration_s and not self._stop_requested():
            step = min(0.1, duration_s - elapsed)
            rclpy.spin_once(self, timeout_sec=step)
            elapsed += step

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceExecutorNode()
    try:
        node.run_to_target()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
