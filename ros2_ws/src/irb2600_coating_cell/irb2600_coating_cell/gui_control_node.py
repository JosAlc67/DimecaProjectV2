"""Simple Tkinter control panel with three buttons: "Go Home", "Start
Route" and "Stop". Wraps GoHomeNode.go_home(), ReplanningExecutorNode.
run_route() and their request_stop() so operating the cell doesn't require
typing CLI commands in a separate terminal each time.

An RViz panel plugin was considered instead but ruled out: this workspace
already ran into three separate moveit_rviz_plugin bugs on this ROS 2
Humble install (a nonexistent Panel class, a joint_limits type-mismatch
exception, PlanningScene never subscribing to /monitored_planning_scene --
see scene_setup_node.py and config/moveit.rviz for the workarounds), so a
plain Tkinter window (stdlib, no RViz plugin dependency) is more reliable
here.

    ros2 run irb2600_coating_cell gui_control_node

"Start Route" runs the same reactive replanning loop as
replanning_executor_node, but with execute:=true forced on (this GUI's
whole point is to actually move the robot, unlike that node's plan-only
CLI default).

"Stop" (only enabled while Go Home or Start Route is running) cancels the
in-progress MoveGroup/ExecuteTrajectory goal via request_stop() (see
stoppable.py) -- it interrupts whatever is currently moving, not just
whatever hasn't started yet.

Requires the python3-tk system package (not a rosdep/pip dependency):
    sudo apt install -y python3-tk
"""

import queue
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.parameter import Parameter

from irb2600_coating_cell.go_home_node import GoHomeNode
from irb2600_coating_cell.replanning_executor_node import ReplanningExecutorNode


class ControlPanelApp:

    def __init__(self, root, go_home_node, replanning_node):
        self._root = root
        self._go_home_node = go_home_node
        self._replanning_node = replanning_node
        self._events = queue.Queue()
        self._busy = False
        self._active_node = None

        root.title("IRB2600 Coating Cell - Control Panel")

        self._status_var = tk.StringVar(value="Ready.")
        ttk.Label(root, textvariable=self._status_var, padding=10).pack(fill="x")

        button_frame = ttk.Frame(root, padding=10)
        button_frame.pack(fill="x")

        self._home_button = ttk.Button(
            button_frame, text="Go Home", command=self._on_go_home
        )
        self._home_button.pack(side="left", padx=5, expand=True, fill="x")

        self._route_button = ttk.Button(
            button_frame, text="Start Route", command=self._on_start_route
        )
        self._route_button.pack(side="left", padx=5, expand=True, fill="x")

        self._stop_button = ttk.Button(
            button_frame, text="Stop", command=self._on_stop, state="disabled"
        )
        self._stop_button.pack(side="left", padx=5, expand=True, fill="x")

        self._poll_events()

    # -- button handlers ----------------------------------------------------

    def _on_go_home(self):
        self._run_in_background(
            "Going home...", self._go_home_node, self._go_home_node.go_home
        )

    def _on_start_route(self):
        self._run_in_background(
            "Running route...", self._replanning_node, self._replanning_node.run_route
        )

    def _on_stop(self):
        if self._active_node is not None:
            self._status_var.set("Stopping...")
            self._active_node.request_stop()

    def _run_in_background(self, status_text, node, target):
        # rclpy's spin_until_future_complete() calls inside go_home()/
        # run_route() block the calling thread, so they run off the Tk main
        # thread to keep the window responsive; Go Home/Start Route are
        # disabled for the duration to avoid two blocking calls overlapping
        # on the same underlying node, while Stop stays enabled so it can
        # reach the node that is actually running.
        if self._busy:
            return
        self._busy = True
        self._active_node = node
        self._set_buttons_enabled(running=True)
        self._status_var.set(status_text)

        def worker():
            try:
                target()
                self._events.put(("done", None))
            except Exception as exc:
                self._events.put(("error", str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def _set_buttons_enabled(self, running):
        self._home_button.configure(state="disabled" if running else "normal")
        self._route_button.configure(state="disabled" if running else "normal")
        self._stop_button.configure(state="normal" if running else "disabled")

    def _poll_events(self):
        try:
            while True:
                kind, payload = self._events.get_nowait()
                if kind == "done":
                    self._status_var.set("Ready.")
                elif kind == "error":
                    self._status_var.set(f"Error: {payload}")
                self._busy = False
                self._active_node = None
                self._set_buttons_enabled(running=False)
        except queue.Empty:
            pass
        self._root.after(100, self._poll_events)


def main(args=None):
    rclpy.init(args=args)

    go_home_node = GoHomeNode()
    replanning_node = ReplanningExecutorNode(
        parameter_overrides=[Parameter("execute", Parameter.Type.BOOL, True)]
    )

    root = tk.Tk()
    ControlPanelApp(root, go_home_node, replanning_node)
    try:
        root.mainloop()
    finally:
        go_home_node.destroy_node()
        replanning_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
