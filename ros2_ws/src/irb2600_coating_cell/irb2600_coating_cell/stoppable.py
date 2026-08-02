"""Shared cooperative-cancellation helper for action-based nodes
(go_home_node, replanning_executor_node) so a "Stop" request from another
thread (e.g. the Tkinter GUI's button) can both interrupt a blocking wait
and cancel the in-flight MoveGroup/ExecuteTrajectory goal, instead of only
taking effect before the *next* motion starts.
"""

import threading

import rclpy
from action_msgs.msg import GoalStatus


class StoppableActionNode:
    """Mixin for rclpy Node subclasses that send action goals and want a
    request_stop() other threads can call safely. Must be mixed in with
    rclpy.node.Node (relies on self.get_logger()/spin)."""

    def _init_stoppable(self):
        self._stop_event = threading.Event()
        self._active_goal_handle = None

    def request_stop(self):
        """Thread-safe: ask any in-progress action goal to cancel and any
        blocking wait loop to return early. Safe to call even if nothing
        is running."""
        self._stop_event.set()
        goal_handle = self._active_goal_handle
        if goal_handle is not None:
            self.get_logger().warn("Stop requested: cancelling in-progress goal...")
            goal_handle.cancel_goal_async()

    def _clear_stop(self):
        self._stop_event.clear()

    def _stop_requested(self):
        return self._stop_event.is_set()

    def _send_goal_and_wait(self, action_client, goal, poll_timeout_s=0.2):
        """Send an action goal and block until it finishes, returning the
        result response, or None if rejected or cancelled via
        request_stop(). Polls in short slices (instead of one unbounded
        spin_until_future_complete) so a Stop request lands promptly."""
        send_goal_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            return None

        self._active_goal_handle = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            while rclpy.ok():
                rclpy.spin_until_future_complete(
                    self, result_future, timeout_sec=poll_timeout_s
                )
                if result_future.done():
                    return result_future.result()
                if self._stop_requested():
                    # cancel already sent by request_stop(); keep polling
                    # the same future so we still pick up the resulting
                    # CANCELED status instead of abandoning it.
                    continue
            return None
        finally:
            self._active_goal_handle = None

    @staticmethod
    def _succeeded(result_response):
        return result_response is not None and result_response.status == GoalStatus.STATUS_SUCCEEDED

    @staticmethod
    def _cancelled(result_response):
        return result_response is not None and result_response.status == GoalStatus.STATUS_CANCELED
