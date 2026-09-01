"""Coverage Path Planning Executor for 7-DOF Painting Robot.
Ejecuta un barrido de pintura (Raster Pattern) sobre la pieza objetivo,
moviéndose de manera inteligente y replanificando ante obstáculos.
"""

import time
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from irb2600_coating_cell.geometry_utils import (
    quaternion_from_rpy,
    quaternion_with_z_axis,
    rotate_vector_by_quaternion,
    transform_point,
)
from irb2600_coating_cell.resource_utils import load_shared_cell_config, resolve_resource_uri
from irb2600_coating_cell.raster_path import resample_rows
from irb2600_coating_cell.run_metrics import RunMetrics
from irb2600_coating_cell.stoppable import StoppableActionNode

# MoveIt error codes
SUCCESS = 1

class CoveragePathExecutorNode(StoppableActionNode, Node):

    def __init__(self, **kwargs):
        super().__init__("replanning_executor_node", **kwargs)
        self._init_stoppable()

        try:
            shared_config = load_shared_cell_config()
        except (OSError, ValueError, LookupError) as exc:
            self.get_logger().warn(f"Shared configuration unavailable; using safe defaults: {exc}")
            shared_config = {}

        def configured(section, name, fallback):
            value = shared_config.get(section, {})
            return value.get(name, fallback) if isinstance(value, dict) else fallback

        # Target point configuration (will fallback to this if no sensor reading)
        self.declare_parameter("target_structure.frame_id", "world")
        target_config = shared_config.get("target_structure", {})
        self.declare_parameter("target_structure.position", target_config.get("position", [0.0, -1.2, 1.5]))
        self.declare_parameter("target_structure.orientation_rpy", target_config.get("orientation_rpy", [0.0, 0.0, 0.0]))
        self.declare_parameter("target_structure.local_normal", target_config.get("local_normal", [-1.0, 0.0, 0.0]))
        self.declare_parameter("target_structure.mesh_file", target_config.get("mesh_file", "package://irb2600_coating_cell/meshes/curved_panel.stl"))
        self.declare_parameter("trajectory.d_standoff", configured("trajectory", "d_standoff", 0.15))
        self.declare_parameter("trajectory.max_cartesian_step", configured("trajectory", "max_cartesian_step", 0.05))
        
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("tcp_link", "nozzle_tip")
        self.declare_parameter("execute", False)
        self.declare_parameter("replanning_time_s", 5.0)
        self.declare_parameter("replanning_attempts", 10)
        self.declare_parameter("trajectory.raster_step_z", configured("trajectory", "raster_step_z", 0.2))
        self.declare_parameter("trajectory.raster_step_y", configured("trajectory", "raster_step_y", 0.1))
        self.declare_parameter("trajectory.requested_waypoints", configured("trajectory", "requested_waypoints", 0))
        self.declare_parameter("trajectory.target_speed_mps", configured("trajectory", "target_speed_mps", 0.0))
        self.declare_parameter("trajectory.velocity_scaling_factor", configured("trajectory", "velocity_scaling_factor", 0.2))
        self.declare_parameter("safety.require_workspace_clear", configured("safety", "require_workspace_clear", False))
        self.declare_parameter("safety.stable_clear_time_s", configured("safety", "stable_clear_time_s", 0.5))
        self.declare_parameter("safety.workspace_wait_timeout_s", configured("safety", "workspace_wait_timeout_s", 30.0))
        self.declare_parameter("metrics.enabled", configured("metrics", "enabled", True))
        self.declare_parameter("metrics.output_directory", configured("metrics", "output_directory", ""))

        self._latest_target_pose = None
        self._workspace_clear = None
        self._workspace_clear_since = None
        self._metrics = None
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseStamped, "structure_pose", self._on_structure_pose, qos)
        self.create_subscription(Bool, "/workspace_clear", self._on_workspace_clear, qos)

        self._move_group_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for /move_action (move_group)...")
        self._move_group_client.wait_for_server()
        
        from std_srvs.srv import SetBool
        self._spray_client = self.create_client(SetBool, "/spray_controller_node/set_spray_on")
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

    def _on_structure_pose(self, msg):
        self._latest_target_pose = msg

    def _on_workspace_clear(self, msg):
        clear = bool(msg.data)
        if clear and not self._workspace_clear:
            self._workspace_clear_since = time.monotonic()
        elif not clear:
            self._workspace_clear_since = None
        self._workspace_clear = clear

    def _generate_mesh_coverage_path(self, mesh_file, target_pose, standoff):
        """Generate world-frame poses from a mesh expressed in its local frame.

        The mesh is sampled in local Y/Z so it can be placed at any 6D pose.
        Its working face must point along ``target_structure.local_normal``;
        this is the established convention for the coating-panel CAD assets.
        """
        import trimesh
        import numpy as np
        mesh_path = resolve_resource_uri(mesh_file)
            
        self.get_logger().info(f"Loading mesh for 3D CPP from {mesh_path}")
        mesh = trimesh.load(mesh_path)
        
        p = self.get_parameter
        step_z = p("trajectory.raster_step_z").value
        step_y = p("trajectory.raster_step_y").value
        
        bounds = mesh.bounds
        center_pos = (
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
        )
        target_quat = target_pose.orientation
        local_normal = tuple(
            float(value) for value in p("target_structure.local_normal").value
        )
        normal_length = float(np.linalg.norm(local_normal))
        if normal_length < 1e-9:
            raise ValueError("target_structure.local_normal must not be zero")
        local_normal = tuple(value / normal_length for value in local_normal)
        if abs(local_normal[0]) < 0.99:
            raise ValueError(
                "mesh coverage currently requires target_structure.local_normal "
                "to be aligned with the local X axis"
            )
        z_min = bounds[0][2] + 0.1
        z_max = bounds[1][2] - 0.1
        y_min = bounds[0][1] + 0.1
        y_max = bounds[1][1] - 0.1
        
        path = []
        current_z = z_max
        direction = 1
        
        while current_z >= z_min:
            current_row = []
            y_points = self._frange(y_min, y_max, step_y) if direction == 1 else self._frange(y_max, y_min, -step_y)
            
            for current_y in y_points:
                # Raycast in the CAD's local coordinates.  The world pose is
                # applied only after finding the exact surface point and normal.
                x_extent = (bounds[1][0] - bounds[0][0]) / 2.0
                origin = np.array([[
                    (bounds[0][0] + bounds[1][0]) / 2.0 + local_normal[0] * (x_extent + 1.0),
                    current_y,
                    current_z,
                ]])
                direction_vec = np.array([[-local_normal[0], 0.0, 0.0]])

                locs, idx_ray, idx_tri = mesh.ray.intersects_location(
                    ray_origins=origin,
                    ray_directions=direction_vec,
                    multiple_hits=False
                )

                if len(locs) > 0:
                    hit_point = locs[0]
                    tri_idx = idx_tri[0]
                    
                    # Orient the local normal toward the configured working face.
                    normal = mesh.face_normals[tri_idx]
                    if float(np.dot(normal, local_normal)) < 0.0:
                        normal = -normal

                    tool_point_local = hit_point + normal * standoff
                    tool_x, tool_y, tool_z = transform_point(
                        tool_point_local, center_pos, target_quat
                    )
                    
                    pose = Pose()
                    pose.position.x = float(tool_x)
                    pose.position.y = float(tool_y)
                    pose.position.z = float(tool_z)
                    
                    # Tool approaches the surface, against its outward normal.
                    normal_world = rotate_vector_by_quaternion(normal, target_quat)
                    approach = [-normal_world[0], -normal_world[1], -normal_world[2]]
                    pose.orientation = quaternion_with_z_axis(approach)
                    current_row.append(pose)
                    
            if current_row:
                path.append(current_row)

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

    def run_coverage(self, num_passes=1, resume=False):
        self._clear_stop()
        p = self.get_parameter
        self._metrics = RunMetrics(
            enabled=p("metrics.enabled").value,
            output_directory=p("metrics.output_directory").value,
        )
        self._metrics.event("run_started", f"passes={num_passes}, resume={resume}")
        
        # Get target from parameters or the perception simulation.  Keep the
        # orientation as well as translation; CAD assets are not world-aligned.
        frame_id = p("target_structure.frame_id").value
        target_pose = Pose()
        if self._latest_target_pose is not None:
            frame_id = self._latest_target_pose.header.frame_id or frame_id
            target_pose = self._latest_target_pose.pose
            self.get_logger().info("Using dynamic structure pose from perception sensor.")
        else:
            target_pose.position.x, target_pose.position.y, target_pose.position.z = (
                float(value) for value in p("target_structure.position").value
            )
            target_pose.orientation = quaternion_from_rpy(
                *[float(value) for value in p("target_structure.orientation_rpy").value]
            )
            self.get_logger().warn("No structure pose received, using fallback parameter.")
        mesh_file = p("target_structure.mesh_file").value
        d_standoff = p("trajectory.d_standoff").value
        
        self.get_logger().info(
            "Generating 3D Coverage Path for mesh at "
            f"({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})"
        )
        rows = self._generate_mesh_coverage_path(mesh_file, target_pose, d_standoff)
        requested_waypoints = int(p("trajectory.requested_waypoints").value)
        if requested_waypoints > 0:
            try:
                rows = resample_rows(rows, requested_waypoints)
            except ValueError as exc:
                self.get_logger().error(str(exc))
                self._finish_metrics("failed", num_passes, str(exc))
                return False
        self.get_logger().info(f"Generated {len(rows)} rows for dynamic 3D painting.")
        self._metrics.rows_generated = len(rows)
        self._metrics.waypoints_requested = sum(len(row) for row in rows) * num_passes
        self._metrics.event("path_generated", f"rows={len(rows)}, waypoints={sum(len(row) for row in rows)}")
        
        if not rows:
            self._finish_metrics("failed", num_passes, "no waypoints generated")
            return False
            
        start_pass = 0
        start_row = 0
        waypoints_to_execute = None
        
        if resume and self._resume_state is not None:
            start_pass, start_row, waypoints_to_execute = self._resume_state
            self.get_logger().info(f"Resuming from pass {start_pass+1}, row {start_row+1}...")
        else:
            self._resume_state = None
            
        for pass_idx in range(start_pass, num_passes):
            if not rclpy.ok() or self._stop_requested():
                break
                
            self.get_logger().info(f"--- Starting pass {pass_idx+1}/{num_passes} ---")
            
            row_start = start_row if pass_idx == start_pass else 0
            for row_idx in range(row_start, len(rows)):
                if not rclpy.ok() or self._stop_requested():
                    break
                
                if pass_idx == start_pass and row_idx == start_row and waypoints_to_execute is not None:
                    self.get_logger().info(f"Row {row_idx + 1}/{len(rows)}: Resuming chunk...")
                else:
                    self.get_logger().info(f"Row {row_idx + 1}/{len(rows)}: Moving to start position...")
                    waypoints_to_execute = list(rows[row_idx])

                if not self._wait_for_safe_workspace():
                    self._finish_metrics("stopped" if self._stop_requested() else "failed", num_passes, "workspace did not become safely clear")
                    return False

                if not self._set_spray(False):
                    self._finish_metrics("failed", num_passes, "spray-off command was not confirmed")
                    return False

                while waypoints_to_execute and rclpy.ok():
                    if self._stop_requested():
                        self._resume_state = (pass_idx, row_idx, waypoints_to_execute)
                        break
                    # Move to the first waypoint of this chunk
                    goal = self._build_move_goal(waypoints_to_execute[0], frame_id)
                    result = self._send_goal_and_wait(self._move_group_client, goal)
                    if not self._succeeded(result) or result.result.error_code.val != SUCCESS:
                        self.get_logger().error(f"Failed to reach start of chunk in Row {row_idx + 1}.")
                        self._finish_metrics("failed", num_passes, "failed to reach start of row chunk")
                        return False
                        
                    if len(waypoints_to_execute) == 1:
                        break # Only 1 point left, already reached it.
                        
                    # Compute Cartesian path for the chunk
                    self.get_logger().info("Computing smooth Cartesian path...")
                    req = GetCartesianPath.Request()
                    req.header.frame_id = frame_id
                    req.group_name = self.get_parameter("group_name").value
                    req.link_name = self.get_parameter("tcp_link").value
                    req.waypoints = waypoints_to_execute[1:]
                    req.max_step = float(self.get_parameter("trajectory.max_cartesian_step").value)
                    req.jump_threshold = 0.0
                    req.avoid_collisions = True
                    
                    if not self._cartesian_client.wait_for_service(timeout_sec=2.0):
                        self.get_logger().error("Cartesian service not available!")
                        self._finish_metrics("failed", num_passes, "Cartesian service unavailable")
                        return False
                        
                    future = self._cartesian_client.call_async(req)
                    if not self._wait_for_future(future):
                        self._finish_metrics("failed", num_passes, "Cartesian service call did not complete")
                        return False
                    res = future.result()
                    
                    if res.fraction > 0.0:
                        # Back off a bit if we hit an obstacle
                        if res.fraction < 0.99:
                            num_points = len(res.solution.joint_trajectory.points)
                            # Cut off the last 8 points (~40cm of TCP travel) so it visibly 
                            # stops safely away from the obstacle instead of getting too close.
                            safe_len = max(1, num_points - 8)
                            res.solution.joint_trajectory.points = res.solution.joint_trajectory.points[:safe_len]
                        
                        self.get_logger().info(f"Cartesian path computed (fraction: {res.fraction:.2f}). Executing...")
                        if self.get_parameter("execute").value:
                            exec_goal = ExecuteTrajectory.Goal()
                            exec_goal.trajectory = res.solution

                            if not self._wait_for_safe_workspace() or not self._set_spray(True):
                                self._finish_metrics("failed", num_passes, "workspace unsafe or spray-on command unconfirmed")
                                return False
                            self._execute_client.wait_for_server()
                            execution_started = time.monotonic()
                            exec_result = self._send_goal_and_wait(self._execute_client, exec_goal)
                            execution_time = time.monotonic() - execution_started
                            self._metrics.cartesian_segment(waypoints_to_execute[1:], res.fraction, execution_time)

                            if not self._succeeded(exec_result):
                                self.get_logger().error("Execution was rejected or cancelled.")
                                self._set_spray(False)
                                self._finish_metrics("stopped" if self._stop_requested() else "failed", num_passes, "trajectory execution rejected, cancelled, or failed")
                                return False

                            if not self._set_spray(False):
                                self._finish_metrics("failed", num_passes, "spray-off command was not confirmed after segment")
                                return False
                        else:
                            self.get_logger().info("execute:=false: recording the planned segment without moving or enabling spray.")
                            self._metrics.cartesian_segment(waypoints_to_execute[1:], res.fraction, 0.0)
                    else:
                        self.get_logger().error(f"Failed to compute Cartesian path. Error code: {res.error_code.val}")
                        self._finish_metrics("failed", num_passes, f"Cartesian planning error {res.error_code.val}")
                        return False
                        
                    if res.fraction >= 0.99:
                        break # Finished this row chunk successfully
                    else:
                        # Obstacle hit! Calculate where we stopped.
                        hit_idx = int(res.fraction * (len(waypoints_to_execute) - 1))
                        self.get_logger().warn(f"Row {row_idx + 1} blocked at {res.fraction:.0%}. Attempting bypass...")
                        self._metrics.event("partial_path", f"row={row_idx + 1}, fraction={res.fraction:.6f}")
                        
                        # Find a safe jump index. Try skipping 4, 7, 10, 15, 20 waypoints ahead.
                        bypass_success = False
                        for skip in [4, 7, 10, 15, 20]:
                            jump_idx = hit_idx + skip
                            if jump_idx >= len(waypoints_to_execute):
                                jump_idx = len(waypoints_to_execute) - 1
                                
                            self.get_logger().info(f"Row {row_idx + 1}: trying bypass jump to waypoint {jump_idx}/{len(waypoints_to_execute)-1}...")
                            self._metrics.event("bypass_attempt", f"row={row_idx + 1}, waypoint={jump_idx}")
                            jump_goal = self._build_move_goal(waypoints_to_execute[jump_idx], frame_id)
                            jump_res = self._send_goal_and_wait(self._move_group_client, jump_goal)
                            
                            if self._succeeded(jump_res) and jump_res.result.error_code.val == SUCCESS:
                                self.get_logger().info(f"Bypass successful! Resuming paint from waypoint {jump_idx}.")
                                waypoints_to_execute = waypoints_to_execute[jump_idx:]
                                bypass_success = True
                                break
                        
                        if not bypass_success:
                            self.get_logger().error(f"Row {row_idx + 1}: Could not find a safe bypass. Aborting pass.")
                            self._resume_state = (pass_idx, row_idx, waypoints_to_execute)
                            self._finish_metrics("failed", num_passes, "no safe obstacle bypass found")
                            return False

        if self._stop_requested():
            self._finish_metrics("stopped", num_passes, "operator stop requested")
            return False
        self.get_logger().info("Coverage Path Execution Completed.")
        self._finish_metrics("completed", num_passes)
        return True

    def _set_spray(self, state):
        if not self._spray_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Spray controller service is unavailable.")
            return False
        req = SetBool.Request()
        req.data = state
        future = self._spray_client.call_async(req)
        completed = self._wait_for_future(future, timeout_s=2.0)
        response = future.result() if completed else None
        confirmed = response is not None and response.success
        if confirmed and self._metrics is not None:
            self._metrics.event("spray_on" if state else "spray_off", response.message)
        if not confirmed:
            self.get_logger().error(f"Spray {'on' if state else 'off'} command was not confirmed.")
        return confirmed

    def _wait_for_safe_workspace(self):
        if not self.get_parameter("safety.require_workspace_clear").value:
            return not self._stop_requested()
        stable_s = float(self.get_parameter("safety.stable_clear_time_s").value)
        timeout_s = float(self.get_parameter("safety.workspace_wait_timeout_s").value)
        deadline = time.monotonic() + timeout_s
        announced = False
        if not self._set_spray(False):
            return False
        while rclpy.ok() and not self._stop_requested() and time.monotonic() < deadline:
            stable = (
                self._workspace_clear is True
                and self._workspace_clear_since is not None
                and time.monotonic() - self._workspace_clear_since >= stable_s
            )
            if stable:
                return True
            if not announced:
                self.get_logger().warn("Workspace unsafe or unknown; spray remains off while waiting for a stable clear signal.")
                if self._metrics is not None:
                    self._metrics.event("obstacle_wait", "waiting for stable workspace_clear=true")
                announced = True
            self._cooperative_pause(0.1)
        return False

    def _finish_metrics(self, outcome, passes_requested, failure_reason=""):
        if self._metrics is None:
            return
        paths = self._metrics.finish(
            outcome=outcome,
            passes_requested=passes_requested,
            configured_target_speed_mps=self.get_parameter("trajectory.target_speed_mps").value,
            configured_standoff_m=self.get_parameter("trajectory.d_standoff").value,
            failure_reason=failure_reason,
        )
        if paths:
            self.get_logger().info(f"Run metrics written to {paths[0]} and {paths[1]}")
        self._metrics = None

    def _build_move_goal(self, target_pose, frame_id):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.get_parameter("group_name").value
        goal.request.num_planning_attempts = int(self.get_parameter("replanning_attempts").value)
        goal.request.allowed_planning_time = float(self.get_parameter("replanning_time_s").value)
        
        # Painting requires smooth, slow motions
        velocity_scaling = float(self.get_parameter("trajectory.velocity_scaling_factor").value)
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = velocity_scaling
        
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
