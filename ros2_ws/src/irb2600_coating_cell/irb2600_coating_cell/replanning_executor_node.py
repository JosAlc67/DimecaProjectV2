"""Coverage Path Planning Executor for 7-DOF Painting Robot.
Ejecuta un barrido de pintura (Raster Pattern) sobre la pieza objetivo,
moviéndose de manera inteligente y replanificando ante obstáculos.
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from irb2600_coating_cell.geometry_utils import quaternion_with_z_axis, rotate_vector_by_quaternion
from irb2600_coating_cell.stoppable import StoppableActionNode

# MoveIt error codes
SUCCESS = 1
PREEMPTED = -2
CONTROL_FAILED = -4
NO_IK_SOLUTION = -31
TIMED_OUT = -3

class CoveragePathExecutorNode(StoppableActionNode, Node):

    def __init__(self, **kwargs):
        super().__init__("replanning_executor_node", **kwargs)
        self._init_stoppable()

        # Target point configuration (will fallback to this if no sensor reading)
        self.declare_parameter("target_structure.frame_id", "world")
        self.declare_parameter("target_structure.position", [1.5, -2.0, 1.5])
        self.declare_parameter("target_structure.size", [0.05, 5.0, 2.0])
        self.declare_parameter("target_structure.local_normal", [-1.0, 0.0, 0.0])
        self.declare_parameter("d_standoff", 0.15)
        
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("tcp_link", "nozzle_tip")
        self.declare_parameter("execute", False)
        self.declare_parameter("replanning_time_s", 5.0)
        self.declare_parameter("replanning_attempts", 10)
        self.declare_parameter("raster_step_z", 0.2) # Distance between horizontal sweeps
        self.declare_parameter("raster_step_y", 0.5) # Resolution along the sweep

        self._latest_target_pose = None
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseStamped, "structure_pose", self._on_structure_pose, qos)

        self._move_group_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for /move_action (move_group)...")
        self._move_group_client.wait_for_server()

    def _on_structure_pose(self, msg):
        self._latest_target_pose = msg

    def _generate_raster_path(self, center_pos, size, normal, standoff):
        """Generates a list of Poses covering the YZ face of the panel."""
        p = self.get_parameter
        step_z = p("raster_step_z").value
        step_y = p("raster_step_y").value
        
        # Dimensions
        y_size = size[1]
        z_size = size[2]
        
        # We start from top-left
        start_y = center_pos[1] - y_size/2.0 + 0.1
        end_y = center_pos[1] + y_size/2.0 - 0.1
        start_z = center_pos[2] + z_size/2.0 - 0.1
        end_z = center_pos[2] - z_size/2.0 + 0.1
        
        # Fixed X based on normal
        paint_x = center_pos[0] + normal[0] * standoff
        
        path = []
        current_z = start_z
        direction = 1 # 1 for moving +Y, -1 for moving -Y
        
        # The tool orientation is opposite to the surface normal
        approach_direction = tuple(-c for c in normal)
        orientation = quaternion_with_z_axis(approach_direction)
        
        while current_z >= end_z:
            if direction == 1:
                y_points = self._frange(start_y, end_y, step_y)
            else:
                y_points = self._frange(end_y, start_y, -step_y)
                
            for current_y in y_points:
                pose = Pose()
                pose.position.x = paint_x
                pose.position.y = current_y
                pose.position.z = current_z
                pose.orientation = orientation
                path.append(pose)
                
            current_z -= step_z
            direction *= -1
            
        return path

    def _frange(self, start, stop, step):
        points = []
        val = start
        if step > 0:
            while val <= stop:
                points.append(val)
                val += step
        else:
            while val >= stop:
                points.append(val)
                val += step
        return points

    def run_coverage(self):
        self._clear_stop()
        p = self.get_parameter
        
        # Get target from parameters
        frame_id = p("target_structure.frame_id").value
        pos = p("target_structure.position").value
        size = p("target_structure.size").value
        normal = p("target_structure.local_normal").value
        d_standoff = p("d_standoff").value
        
        self.get_logger().info(f"Generating Coverage Path for Panel size {size} at {pos}")
        waypoints = self._generate_raster_path(pos, size, normal, d_standoff)
        self.get_logger().info(f"Generated {len(waypoints)} painting waypoints.")
        
        for idx, target_pose in enumerate(waypoints):
            if not rclpy.ok() or self._stop_requested():
                break
                
            self.get_logger().info(
                f"--- Painting Waypoint {idx+1}/{len(waypoints)}: "
                f"y={target_pose.position.y:.2f}, z={target_pose.position.z:.2f} ---"
            )
            
            # Try to reach the waypoint
            attempts = 0
            max_attempts = 3
            waypoint_reached = False
            
            while rclpy.ok() and not self._stop_requested() and attempts < max_attempts:
                attempts += 1
                goal = self._build_move_goal(target_pose, frame_id)
                result = self._send_goal_and_wait(self._move_group_client, goal)
                
                if result is None:
                    self.get_logger().error("Action server rejected the goal.")
                    break
                    
                error_code = result.result.error_code.val
                if error_code == SUCCESS:
                    waypoint_reached = True
                    break
                elif error_code in [PREEMPTED, CONTROL_FAILED]:
                    self.get_logger().warn(f"Dynamic obstacle encountered! Replanning (attempt {attempts})...")
                    self._interruptible_sleep(1.0)
                elif error_code == NO_IK_SOLUTION:
                    self.get_logger().error(f"Waypoint {idx+1} unreachable (Kinematic limits of 7-DOF?). Skipping waypoint.")
                    break
                else:
                    self.get_logger().warn(f"Failed to plan (code={error_code}). Retrying...")
                    self._interruptible_sleep(1.0)
            
            if not waypoint_reached:
                self.get_logger().warn(f"Skipping waypoint {idx+1} after failures.")

        self.get_logger().info("Coverage Path Execution Completed.")
        return True

    def _build_move_goal(self, target_pose, frame_id):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.get_parameter("group_name").value
        goal.request.num_planning_attempts = int(self.get_parameter("replanning_attempts").value)
        goal.request.allowed_planning_time = float(self.get_parameter("replanning_time_s").value)
        
        # Painting requires smooth, slow motions
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        
        goal.request.workspace_parameters.header.frame_id = "world"
        goal.request.workspace_parameters.min_corner.x = -5.0
        goal.request.workspace_parameters.min_corner.y = -5.0
        goal.request.workspace_parameters.min_corner.z = 0.0
        goal.request.workspace_parameters.max_corner.x = 5.0
        goal.request.workspace_parameters.max_corner.y = 10.0
        goal.request.workspace_parameters.max_corner.z = 5.0
        
        goal.planning_options.plan_only = not self.get_parameter("execute").value
        goal.planning_options.planning_scene_diff.is_diff = True
        
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
        sp.dimensions = [0.05] # 5cm tolerance for coverage path
        bv.primitives.append(sp)
        bv.primitive_poses.append(target_pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        
        oc = OrientationConstraint()
        oc.header.frame_id = frame_id
        oc.link_name = self.get_parameter("tcp_link").value
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
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
    node = CoveragePathExecutorNode()
    try:
        node.run_coverage()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
