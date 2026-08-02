"""Reactive replanning executor (report Table VI "Replan the trajectory";
Table XVII Case 3: obstacle changes position dynamically during operation).

Unlike trajectory_planner_node (Phase 1: plans the whole raster once against
a static obstacle), this node executes the coating raster ROW BY ROW. Before
each row it re-checks that row's Cartesian path against the *current*
planning scene:

  - If clear (fraction >= fraction_threshold): execute it directly, exactly
    like trajectory_planner_node does for the whole path.
  - If blocked: attempt to replan -- compute IK for the row's end pose,
    *seeded from the last joint state the blocked Cartesian path actually
    reached* (not the robot's current/starting state), then ask move_group
    for a full joint-space (OMPL) plan to that joint goal (which, unlike
    straight-line Cartesian interpolation, can route around the obstacle).
    The warm seed matters: KDL (this project's kinematics_solver) is a local
    numerical IK solver, and on the VM it consistently failed to converge
    for a row's end pose when seeded from a "cold"/far state -- both via a
    direct /compute_ik call and via OMPL's own internal goal-constraint
    sampler ("Unable to sample any valid states for goal tree") -- even
    though the Cartesian planner could already get ~80% of the way to that
    same pose by construction (warm-started, incremental steps). Seeding IK
    from that near-target state converges reliably where a cold seed did
    not. If that also fails, the row is unreachable: report it as a failed
    trajectory and stop (Table XVII Case 4), matching Sec. VI-B's "safe
    robot stop".

Between rows the node pauses for `segment_pause_s` seconds, specifically so
you can trigger a "controlled change in obstacle position" during that
window (Table IX: "Replanning type: Discrete/reactive") and watch the next
row react to it:

    ros2 param set /perception_sim_node scaffold_pole.position "[X, Y, Z]"

(or tool_cart/cable_reel -- the default obstacle names in
config/scene_objects.yaml; run `ros2 param list /perception_sim_node` to
see the actual names for your scene). scene_setup_node subscribes to
perception_sim_node's /obstacles/<name>/pose topics and re-applies the
planning scene automatically when any of them changes, so that one command
is enough -- no more manual refresh_scene call needed (that combination is
still available as a fallback if perception_sim_node isn't running, see
scene_setup_node's docstring).

    ros2 run irb2600_coating_cell replanning_executor_node --ros-args -p execute:=true
"""

import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import SetBool

from irb2600_coating_cell.raster_path import generate_raster_rows
from irb2600_coating_cell.stoppable import StoppableActionNode

# Only the arm's own joints go into the IK seed / replanning joint goal.
_ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# How often the between-rows pause and the stop check re-evaluate
# request_stop() -- keeps "Stop" responsive without busy-waiting.
_STOP_POLL_S = 0.1


class ReplanningExecutorNode(StoppableActionNode, Node):

    def __init__(self, **kwargs):
        super().__init__("replanning_executor_node", **kwargs)
        self._init_stoppable()

        self.declare_parameter("target_structure.frame_id", "world")
        self.declare_parameter("target_structure.position", [1.8, 0.0, 1.0])
        self.declare_parameter("target_structure.orientation_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("target_structure.size", [0.02, 1.0, 0.6])
        self.declare_parameter("target_structure.local_normal", [-1.0, 0.0, 0.0])

        self.declare_parameter("d_standoff", 0.20)
        self.declare_parameter("edge_margin", 0.05)
        self.declare_parameter("row_pitch", 0.10)
        self.declare_parameter("max_step", 0.01)
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("tcp_link", "nozzle_tip")
        self.declare_parameter("execute", False)

        # Minimum compute_cartesian_path fraction to accept a row as clear.
        self.declare_parameter("fraction_threshold", 0.99)
        # Pause between rows -- the window to manually move the obstacle to
        # exercise Case 3 (see module docstring).
        self.declare_parameter("segment_pause_s", 3.0)
        self.declare_parameter("replanning_time_s", 2.0)
        self.declare_parameter("replanning_attempts", 5)
        self.declare_parameter("ik_timeout_s", 1.0)
        # Diagnostic switch: set to false to ask /compute_ik to ignore
        # collisions for the replanning seed IK. If IK then succeeds, the
        # row's end pose collides with something (obstacle or self) even
        # from the warm seed; if it still fails, it is a pure numerical
        # convergence problem independent of collision checking.
        self.declare_parameter("replan_ik_avoid_collisions", True)

        self._cartesian_path_client = self.create_client(
            GetCartesianPath, "compute_cartesian_path"
        )
        self._ik_client = self.create_client(GetPositionIK, "compute_ik")
        self._move_group_client = ActionClient(self, MoveGroup, "move_action")
        self._execute_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self._spray_client = self.create_client(
            SetBool, "/spray_controller_node/set_spray_on"
        )

        self.get_logger().info("Waiting for /compute_cartesian_path (move_group)...")
        self._cartesian_path_client.wait_for_service()
        self._ik_client.wait_for_service()

    # -- top-level control loop ----------------------------------------------

    def run_route(self):
        """Plan+execute the whole raster, row by row, with reactive
        replanning. Public so callers other than main() (e.g. the Tkinter
        GUI's background thread) can reuse this node instance without
        re-running __init__'s service/action waits each time. Can be
        interrupted by request_stop() from another thread."""
        self._clear_stop()
        p = self.get_parameter
        rows, _normal_world = generate_raster_rows(
            position=p("target_structure.position").value,
            rpy=p("target_structure.orientation_rpy").value,
            size=p("target_structure.size").value,
            local_normal=p("target_structure.local_normal").value,
            d_standoff=p("d_standoff").value,
            edge_margin=p("edge_margin").value,
            row_pitch=p("row_pitch").value,
        )

        metrics = {"direct": 0, "replanned": 0, "failed": 0, "total_replan_time_s": 0.0}
        segment_pause_s = float(p("segment_pause_s").value)
        n_rows = len(rows)

        for idx, row in enumerate(rows):
            if self._stop_requested():
                self.get_logger().warn(
                    f"Stop requested before row {idx + 1}. Aborting remaining rows."
                )
                self._stop_spray_sync()
                break

            self.get_logger().info(f"--- Row {idx + 1}/{n_rows} ---")
            ok = self._process_row(row, idx, metrics)
            if self._stop_requested():
                self.get_logger().warn(
                    f"Row {idx + 1}: stopped by user request. Aborting remaining rows."
                )
                self._stop_spray_sync()
                break
            if not ok:
                self.get_logger().error(
                    f"Row {idx + 1} could not be reached even after replanning "
                    "(Table XVII Case 4: failed-trajectory report, safe robot "
                    "stop). Aborting remaining rows."
                )
                self._stop_spray_sync()
                break

            if idx < n_rows - 1:
                self.get_logger().info(
                    f"Row {idx + 1} done. Pausing {segment_pause_s:.1f}s before "
                    "the next row -- move an obstacle now to test Case 3, e.g.:\n"
                    '  ros2 param set /perception_sim_node scaffold_pole.position "[0.79, 0.0, 1.0]"'
                )
                self._interruptible_sleep(segment_pause_s)

        self.get_logger().info(
            "Summary: {direct} row(s) direct, {replanned} row(s) replanned, "
            "{failed} row(s) failed, total replanning time {total_replan_time_s:.3f} s"
            .format(**metrics)
        )

    def _interruptible_sleep(self, duration_s):
        """time.sleep() split into short slices so request_stop() (called
        from another thread, e.g. the GUI) takes effect within
        _STOP_POLL_S instead of only after the full pause elapses."""
        elapsed = 0.0
        while elapsed < duration_s and not self._stop_requested():
            step = min(_STOP_POLL_S, duration_s - elapsed)
            time.sleep(step)
            elapsed += step

    def _process_row(self, row, idx, metrics):
        t0 = time.time()
        fraction, trajectory = self._compute_cartesian_segment(row)
        t_plan = time.time() - t0

        if fraction >= float(self.get_parameter("fraction_threshold").value):
            self.get_logger().info(
                f"Row {idx + 1}: direct Cartesian path OK "
                f"(fraction={fraction:.3f}, t_plan={t_plan:.3f} s)."
            )
            metrics["direct"] += 1
            return self._execute_with_spray(trajectory)

        self.get_logger().warn(
            f"Row {idx + 1} blocked (fraction={fraction:.3f}, checked in "
            f"{t_plan:.3f} s). Replanning around it (Table VI 'Replan the "
            "trajectory')..."
        )
        seed = self._extract_last_joint_state(trajectory)
        t0 = time.time()
        replanned_trajectory = self._replan_row(row, idx, seed, fraction)
        t_replan = time.time() - t0
        metrics["total_replan_time_s"] += t_replan

        if replanned_trajectory is None:
            metrics["failed"] += 1
            return False

        self.get_logger().info(
            f"Row {idx + 1}: replanned successfully (t_replan={t_replan:.3f} s)."
        )
        metrics["replanned"] += 1
        return self._execute_with_spray(replanned_trajectory)

    # -- direct Cartesian segment ---------------------------------------------

    def _compute_cartesian_segment(self, row):
        request = GetCartesianPath.Request()
        request.header.frame_id = self.get_parameter("target_structure.frame_id").value
        request.group_name = self.get_parameter("group_name").value
        request.link_name = self.get_parameter("tcp_link").value
        request.waypoints = row
        request.max_step = float(self.get_parameter("max_step").value)
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.start_state.is_diff = True

        future = self._cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.fraction, response.solution

    @staticmethod
    def _extract_last_joint_state(trajectory):
        """Names/positions of the last point of a (possibly partial)
        RobotTrajectory, used as a warm IK seed. None if it has no points."""
        points = trajectory.joint_trajectory.points
        if not points:
            return None
        return list(trajectory.joint_trajectory.joint_names), list(points[-1].positions)

    # -- replanning fallback: IK (warm-seeded) + full joint-space (OMPL) plan --

    # Tried in order until one produces a reachable, collision-free target.
    # A single small margin isn't always enough: an obstacle can run
    # alongside a stretch of the row rather than just touching its tail, so
    # retry further back before giving up (report eq. 5 already frames a
    # shortened row as the measurable cost of avoidance -- trying harder to
    # find *some* safe point instead of failing on the first attempt is the
    # same idea taken further).
    _REPLAN_MARGINS = [0.05, 0.10, 0.20, 0.35]

    def _replan_row(self, row, idx, seed, fraction):
        for margin in self._REPLAN_MARGINS:
            target_t = max(0.0, min(fraction - margin, 0.95))
            if target_t <= 0.0:
                break
            self.get_logger().info(
                f"Row {idx + 1}: trying replan target at t={target_t:.3f} "
                f"(margin={margin:.2f})."
            )
            trajectory = self._attempt_replan_at(row, idx, seed, target_t)
            if trajectory is not None:
                return trajectory

        self.get_logger().error(
            f"Row {idx + 1}: no margin in {self._REPLAN_MARGINS} produced a "
            f"reachable, collision-free replan target (fraction={fraction:.3f})."
        )
        return None

    def _attempt_replan_at(self, row, idx, seed, target_t):
        start, end = row[0], row[-1]
        goal_pose = Pose()
        goal_pose.position.x = start.position.x + target_t * (
            end.position.x - start.position.x
        )
        goal_pose.position.y = start.position.y + target_t * (
            end.position.y - start.position.y
        )
        goal_pose.position.z = start.position.z + target_t * (
            end.position.z - start.position.z
        )
        # Orientation is constant across a row (and across the whole panel):
        # raster_path.py computes it once from the panel's surface normal.
        goal_pose.orientation = end.orientation

        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = self.get_parameter("group_name").value
        ik_request.ik_request.ik_link_name = self.get_parameter("tcp_link").value
        ik_request.ik_request.pose_stamped.header.frame_id = self.get_parameter(
            "target_structure.frame_id"
        ).value
        ik_request.ik_request.pose_stamped.pose = goal_pose
        ik_request.ik_request.avoid_collisions = bool(
            self.get_parameter("replan_ik_avoid_collisions").value
        )
        if seed is not None:
            names, positions = seed
            ik_request.ik_request.robot_state.joint_state.name = names
            ik_request.ik_request.robot_state.joint_state.position = positions
            ik_request.ik_request.robot_state.is_diff = False
        else:
            # No partial trajectory to seed from (blocked on the very first
            # interpolation step) -- fall back to the current state.
            ik_request.ik_request.robot_state.is_diff = True
        ik_timeout_s = float(self.get_parameter("ik_timeout_s").value)
        ik_request.ik_request.timeout.sec = int(ik_timeout_s)
        ik_request.ik_request.timeout.nanosec = int(
            (ik_timeout_s - int(ik_timeout_s)) * 1e9
        )

        future = self._ik_client.call_async(ik_request)
        rclpy.spin_until_future_complete(self, future)
        ik_response = future.result()

        if ik_response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                f"Row {idx + 1}: no collision-free IK solution at t={target_t:.3f} "
                f"(error_code={ik_response.error_code.val})."
            )
            return None

        joint_positions = dict(
            zip(ik_response.solution.joint_state.name, ik_response.solution.joint_state.position)
        )

        constraints = Constraints()
        for joint_name in _ARM_JOINTS:
            if joint_name not in joint_positions:
                continue
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = joint_positions[joint_name]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.get_parameter("group_name").value
        goal.request.start_state.is_diff = True
        goal.request.num_planning_attempts = int(
            self.get_parameter("replanning_attempts").value
        )
        goal.request.allowed_planning_time = float(
            self.get_parameter("replanning_time_s").value
        )
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.goal_constraints = [constraints]
        # Left unset, this defaults to a zero-volume (0,0,0)-(0,0,0) box,
        # which can make every candidate state look "out of bounds" to
        # planners/adapters that check it -- easy to miss since it has
        # nothing to do with the arm's actual joint limits (which come from
        # joint_limits.yaml/the URDF regardless of this field).
        frame_id = self.get_parameter("target_structure.frame_id").value
        goal.request.workspace_parameters.header.frame_id = frame_id
        goal.request.workspace_parameters.min_corner.x = -3.0
        goal.request.workspace_parameters.min_corner.y = -3.0
        goal.request.workspace_parameters.min_corner.z = -3.0
        goal.request.workspace_parameters.max_corner.x = 3.0
        goal.request.workspace_parameters.max_corner.y = 3.0
        goal.request.workspace_parameters.max_corner.z = 3.0

        # Plan only; we execute separately via /execute_trajectory, same as
        # the direct-Cartesian path, so both routes share one execution path.
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True

        self._move_group_client.wait_for_server()
        send_goal_future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"Row {idx + 1}: move_action goal rejected.")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                f"Row {idx + 1}: OMPL could not find a collision-free joint-space "
                f"path to t={target_t:.3f} (error_code={result.error_code.val})."
            )
            return None

        return result.planned_trajectory

    # -- execution + spray_on -------------------------------------------------

    def _execute_with_spray(self, trajectory):
        if not self.get_parameter("execute").value:
            self.get_logger().info(
                "execute:=false (default): row planned but not sent to the "
                "controller."
            )
            return True

        self._set_spray_sync(True)

        self._execute_client.wait_for_server()
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        result_response = self._send_goal_and_wait(self._execute_client, goal)

        self._set_spray_sync(False)

        if result_response is None:
            self.get_logger().error("execute_trajectory goal was rejected.")
            return False
        if self._cancelled(result_response):
            self.get_logger().warn("Trajectory execution cancelled (Stop).")
            return False
        succeeded = self._succeeded(result_response)
        if not succeeded:
            self.get_logger().error("Trajectory execution FAILED.")
        return succeeded

    def _set_spray_sync(self, on):
        if not self._spray_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "spray_controller_node not available; continuing without "
                "toggling spray_on."
            )
            return
        request = SetBool.Request()
        request.data = on
        future = self._spray_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def _stop_spray_sync(self):
        self._set_spray_sync(False)


def main(args=None):
    rclpy.init(args=args)
    node = ReplanningExecutorNode()
    node.run_route()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
