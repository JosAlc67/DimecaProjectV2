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
from std_srvs.srv import SetBool

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
        self.declare_parameter("target_structure.position", [0.6, -0.55, 1.0])
        self.declare_parameter("target_structure.mesh_file", "package://irb2600_coating_cell/meshes/curved_panel.stl")
        self.declare_parameter("d_standoff", 0.3)
        
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
        
        from std_srvs.srv import SetBool
        self._spray_client = self.create_client(SetBool, "/spray_controller_node/set_spray_on")

    def _on_structure_pose(self, msg):
        self._latest_target_pose = msg

    def _generate_mesh_coverage_path(self, mesh_file, center_pos, standoff):
        """Generates a list of Poses by raycasting against the 3D mesh surface."""
        import trimesh
        import numpy as np
        import os
        from ament_index_python.packages import get_package_share_directory
        
        # Resolve path
        if mesh_file.startswith("package://"):
            pkg_name = mesh_file.split("/")[2]
            rel_path = "/".join(mesh_file.split("/")[3:])
            try:
                share_dir = get_package_share_directory(pkg_name)
                mesh_path = os.path.join(share_dir, rel_path)
            except Exception:
                mesh_path = ""
            
            # If not in install, fallback to src for development
            if not os.path.exists(mesh_path):
                mesh_path = f"/home/josuealcivar/DimecaProjectV2/ros2_ws/src/{pkg_name}/{rel_path}"
        else:
            mesh_path = mesh_file
            
        self.get_logger().info(f"Loading mesh for 3D CPP from {mesh_path}")
        mesh = trimesh.load(mesh_path)
        mesh.apply_translation(center_pos)
        
        p = self.get_parameter
        step_z = p("raster_step_z").value
        step_y = p("raster_step_y").value
        
        bounds = mesh.bounds
        z_min = bounds[0][2] + 0.1
        z_max = bounds[1][2] - 0.1
        y_min = bounds[0][1] + 0.1
        y_max = bounds[1][1] - 0.1
        
        path = []
        current_z = z_max
        direction = 1
        
        while current_z >= z_min:
            y_points = self._frange(y_min, y_max, step_y) if direction == 1 else self._frange(y_max, y_min, -step_y)
            
            for current_y in y_points:
                # Shoot a ray from +X towards the mesh to find the surface
                origin = np.array([[center_pos[0] + 1.0, current_y, current_z]])
                direction_vec = np.array([[-1.0, 0.0, 0.0]])
                
                locs, idx_ray, idx_tri = mesh.ray.intersects_location(
                    ray_origins=origin,
                    ray_directions=direction_vec,
                    multiple_hits=False
                )
                
                if len(locs) > 0:
                    hit_point = locs[0]
                    tri_idx = idx_tri[0]
                    
                    # Get exact 3D surface normal
                    normal = mesh.face_normals[tri_idx]
                    # Ensure normal points outwards (towards +X)
                    if normal[0] < 0:
                        normal = -normal
                        
                    # Calculate tool position with standoff along the normal
                    tool_x = hit_point[0] + normal[0] * standoff
                    tool_y = hit_point[1] + normal[1] * standoff
                    tool_z = hit_point[2] + normal[2] * standoff
                    
                    pose = Pose()
                    pose.position.x = float(tool_x)
                    pose.position.y = float(tool_y)
                    pose.position.z = float(tool_z)
                    
                    # Tool approaches AGAINST the normal
                    approach = [-float(normal[0]), -float(normal[1]), -float(normal[2])]
                    pose.orientation = quaternion_with_z_axis(approach)
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
        mesh_file = p("target_structure.mesh_file").value
        d_standoff = p("d_standoff").value
        
        self.get_logger().info(f"Generating 3D Coverage Path for mesh at {pos}")
        waypoints = self._generate_mesh_coverage_path(mesh_file, pos, d_standoff)
        self.get_logger().info(f"Generated {len(waypoints)} dynamic 3D painting waypoints.")
        
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
                # Stop spraying before replanning/moving to a new discontinuous point
                self._set_spray(False)
                
                goal = self._build_move_goal(target_pose, frame_id)
                result = self._send_goal_and_wait(self._move_group_client, goal)
                
                if result is None:
                    self.get_logger().error("Action server rejected the goal.")
                    break
                    
                error_code = result.result.error_code.val
                if error_code == SUCCESS:
                    waypoint_reached = True
                    # If we successfully moved, turn on spray for the NEXT continuous motion
                    # (In a real system, you'd trigger this via a trajectory controller,
                    # but for this simulation, triggering it after reaching a point starts
                    # the trail for the subsequent movement).
                    self._set_spray(True)
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

        self._set_spray(False)
        self.get_logger().info("Coverage Path Execution Completed.")
        return True

    def _set_spray(self, state):
        if not self._spray_client.wait_for_service(timeout_sec=1.0):
            return
        req = SetBool.Request()
        req.data = state
        self._spray_client.call_async(req)

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
        goal.request.workspace_parameters.min_corner.y = -10.0
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
        oc.absolute_x_axis_tolerance = 0.5
        oc.absolute_y_axis_tolerance = 0.5
        oc.absolute_z_axis_tolerance = 0.5
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
