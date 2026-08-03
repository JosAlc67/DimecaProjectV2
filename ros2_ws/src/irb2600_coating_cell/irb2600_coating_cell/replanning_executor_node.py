"""Reactive replanning executor (report Table VI "Replan the trajectory";
Table XVII Case 3: obstacle changes position dynamically during operation).
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

_ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
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

        self.declare_parameter("fraction_threshold", 0.99)
        self.declare_parameter("segment_pause_s", 3.0)
        self.declare_parameter("replanning_time_s", 2.0)
        self.declare_parameter("replanning_attempts", 5)
        self.declare_parameter("ik_timeout_s", 1.0)
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

    def run_route(self):
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
                    "the next row..."
                )
                self._interruptible_sleep(segment_pause_s)

        self.get_logger().info(
            "Summary: {direct} row(s) direct, {replanned} row(s) replanned, "
            "{failed} row(s) failed, total replanning time {total_replan_time_s:.3f} s"
            .format(**metrics)
        )

    def _interruptible_sleep(self, duration_s):
        elapsed = 0.0
        while elapsed < duration_s and not self._stop_requested():
            step = min(_STOP_POLL_S, duration_s - elapsed)
            rclpy.spin_once(self, timeout_sec=step)
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
            f"{t_plan:.3f} s). Replanning around it..."
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
        points = trajectory.joint_trajectory.points
        if not points:
            return None
        return list(trajectory.joint_trajectory.joint_names), list(points[-1].positions)

    _REPLAN_MARGINS = [0.05, 0.10, 0.20, 0.35]

    def _replan_row(self, row, idx, seed, fraction):
        # 1. Attempt backing off from partial Cartesian path
        for margin in self._REPLAN_MARGINS:
            target_t = fraction - margin
            if target_t > 0.0:
                self.get_logger().info(
                    f"Row {idx + 1}: trying replan target at t={target_t:.3f} (margin={margin:.2f})."
                )
                trajectory = self._attempt_replan_at(row, idx, seed, target_t)
                if trajectory is not None:
                    return trajectory

        # 2. If obstacle is near the start of the row, attempt 3D joint-space bypass
        for target_t in [1.0, 0.75, 0.5]:
            self.get_logger().info(
                f"Row {idx + 1}: obstacle near start (fraction={fraction:.3f}), trying 3D joint-space bypass to t={target_t:.3f}."
            )
            trajectory = self._attempt_replan_at(row, idx, seed, target_t)
            if trajectory is not None:
                return trajectory

        self.get_logger().error(
            f"Row {idx + 1}: no safe replanning trajectory found (fraction={fraction:.3f})."
        )
        return None

    def _attempt_replan_at(self, row, idx, seed, target_t):
        start, end = row[0], row[-1]
        goal_pose = Pose()
        goal_pose.position.x = start.position.x + target_t * (end.position.x - start.position.x)
        goal_pose.position.y = start.position.y + target_t * (end.position.y - start.position.y)
        goal_pose.position.z = start.position.z + target_t * (end.position.z - start.position.z)
        goal_pose.orientation = end.orientation

        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = self.get_parameter("group_name").value
        ik_request.ik_request.ik_link_name = self.get_parameter("tcp_link").value
        ik_request.ik_request.pose_stamped.header.frame_id = (
            self.get_parameter("target_structure.frame_id").value
        )
        ik_request.ik_request.pose_stamped.pose = goal_pose
        ik_request.ik_request.timeout = rclpy.duration.Duration(
            seconds=float(self.get_parameter("ik_timeout_s").value)
        ).to_msg()
        ik_request.ik_request.avoid_collisions = bool(
            self.get_parameter("replan_ik_avoid_collisions").value
        )

        if seed is not None:
            ik_request.ik_request.robot_state.joint_state.name = seed[0]
            ik_request.ik_request.robot_state.joint_state.position = seed[1]

        future = self._ik_client.call_async(ik_request)
        rclpy.spin_until_future_complete(self, future)
        ik_response = future.result()

        if ik_response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                f"Row {idx + 1}: no collision-free IK solution at t={target_t:.3f} "
                f"(error_code={ik_response.error_code.val})."
            )
            return None

        joint_names = ik_response.solution.joint_state.name
        joint_positions = ik_response.solution.joint_state.position
        joint_map = dict(zip(joint_names, joint_positions))

        goal_constraints = Constraints()
        for joint in _ARM_JOINTS:
            if joint in joint_map:
                jc = JointConstraint()
                jc.joint_name = joint
                jc.position = float(joint_map[joint])
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                goal_constraints.joint_constraints.append(jc)

        move_goal = MoveGroup.Goal()
        move_goal.request.group_name = self.get_parameter("group_name").value
        move_goal.request.num_planning_attempts = int(
            self.get_parameter("replanning_attempts").value
        )
        move_goal.request.allowed_planning_time = float(
            self.get_parameter("replanning_time_s").value
        )
        move_goal.request.goal_constraints = [goal_constraints]
        
        move_goal.planning_options.plan_only = True
        move_goal.planning_options.planning_scene_diff.is_diff = True

        if not self._move_group_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("move_action action server not available.")
            return None

        send_future = self._move_group_client.send_goal_async(move_goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn(
                f"Row {idx + 1}: move_group goal rejected at t={target_t:.3f}."
            )
            return None

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        move_result = res_future.result()

        if move_result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                f"Row {idx + 1}: joint-space planning failed at t={target_t:.3f} "
                f"(error_code={move_result.result.error_code.val})."
            )
            return None

        return move_result.result.planned_trajectory

    def _execute_with_spray(self, trajectory):
        self._set_spray_sync(True)
        try:
            return self._execute_trajectory(trajectory)
        finally:
            self._set_spray_sync(False)

    def _execute_trajectory(self, trajectory):
        if not self.get_parameter("execute").value:
            self.get_logger().info("Planning only (execute:=false); skipping move execution.")
            return True

        if not self._execute_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("execute_trajectory action server not available.")
            return False

        # Ensure header stamp is 0 (start immediately) for ExecuteTrajectory
        trajectory.joint_trajectory.header.stamp.sec = 0
        trajectory.joint_trajectory.header.stamp.nanosec = 0

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("ExecuteTrajectory goal rejected by controller.")
            return False

        res_future = goal_handle.get_result_async()
        while not res_future.done():
            if self._stop_requested():
                self.get_logger().warn("Cancel requested while trajectory is executing.")
                goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, res_future)
                return False
            rclpy.spin_until_future_complete(self, res_future, timeout_sec=_STOP_POLL_S)

        result = res_future.result()
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Trajectory execution failed with error_code={result.result.error_code.val}."
            )
            return False

        return True

    def _set_spray_sync(self, enable: bool):
        if not self._spray_client.service_is_ready():
            self.get_logger().warn(
                "spray_controller_node not available; continuing without toggling spray_on."
            )
            return
        req = SetBool.Request()
        req.data = enable
        future = self._spray_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def _stop_spray_sync(self):
        self._set_spray_sync(False)


def main(args=None):
    rclpy.init(args=args)
    node = ReplanningExecutorNode()
    try:
        node.run_route()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
