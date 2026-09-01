"""Tkinter control panel for the simulated coating cell.

It is launched by ``coating_cell_bringup.launch.py`` by default and wraps
the home, demonstration and coverage movements so operating the cell doesn't require
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
from irb2600_coating_cell.replanning_executor_node import CoveragePathExecutorNode


class ControlPanelApp:

    def __init__(self, root, go_home_node, replanning_node):
        self._root = root
        self._go_home_node = go_home_node
        self._replanning_node = replanning_node
        self._events = queue.Queue()
        self._busy = False
        self._active_node = None

        root.title("DIMECA | Celda de recubrimiento IRB2600")
        root.minsize(600, 220)

        ttk.Label(
            root,
            text="SIMULACIÓN ROS 2 / MOVEIT",
            font=("TkDefaultFont", 12, "bold"),
            padding=(12, 12, 12, 4),
        ).pack(fill="x")
        self._status_var = tk.StringVar(
            value="Simulación conectada. Ejecute primero Movimiento demo."
        )
        ttk.Label(root, textvariable=self._status_var, padding=(12, 4, 12, 10)).pack(fill="x")

        # Passes Configuration Frame
        config_frame = ttk.Frame(root, padding=10)
        config_frame.pack(fill="x")
        ttk.Label(config_frame, text="Número de pasadas:").pack(side="left", padx=5)
        self._passes_var = tk.IntVar(value=1)
        self._passes_spin = ttk.Spinbox(config_frame, from_=1, to=10, textvariable=self._passes_var, width=5)
        self._passes_spin.pack(side="left", padx=5)

        button_frame = ttk.Frame(root, padding=10)
        button_frame.pack(fill="x")

        self._demo_button = ttk.Button(
            button_frame, text="Movimiento demo", command=self._on_demo_move
        )
        self._demo_button.pack(side="left", padx=5, expand=True, fill="x")

        self._home_button = ttk.Button(
            button_frame, text="Ir a inicio", command=self._on_go_home
        )
        self._home_button.pack(side="left", padx=5, expand=True, fill="x")

        self._route_button = ttk.Button(
            button_frame, text="Iniciar ruta", command=self._on_start_route
        )
        self._route_button.pack(side="left", padx=5, expand=True, fill="x")

        self._resume_button = ttk.Button(
            button_frame, text="Reanudar", command=self._on_resume_route
        )
        self._resume_button.pack(side="left", padx=5, expand=True, fill="x")

        self._stop_button = ttk.Button(
            button_frame, text="DETENER", command=self._on_stop, state="disabled"
        )
        self._stop_button.pack(side="left", padx=5, expand=True, fill="x")

        self._poll_events()

    # -- button handlers ----------------------------------------------------

    def _on_go_home(self):
        self._run_in_background(
            "Moviendo a la posición de inicio...", self._go_home_node, self._go_home_node.go_home
        )

    def _on_demo_move(self):
        self._run_in_background(
            "Ejecutando movimiento demo...", self._go_home_node, self._go_home_node.move_demo
        )

    def _on_start_route(self):
        passes = self._passes_var.get()
        self._run_in_background(
            f"Ejecutando ruta ({passes} pasadas)...",
            self._replanning_node,
            lambda: self._replanning_node.run_coverage(num_passes=passes, resume=False)
        )
        
    def _on_resume_route(self):
        passes = self._passes_var.get()
        self._run_in_background(
            f"Reanudando ruta ({passes} pasadas)...",
            self._replanning_node,
            lambda: self._replanning_node.run_coverage(num_passes=passes, resume=True)
        )

    def _on_stop(self):
        if self._active_node is not None:
            self._status_var.set("Deteniendo movimiento...")
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
        self._demo_button.configure(state="disabled" if running else "normal")
        self._home_button.configure(state="disabled" if running else "normal")
        self._route_button.configure(state="disabled" if running else "normal")
        self._resume_button.configure(state="disabled" if running else "normal")
        self._stop_button.configure(state="normal" if running else "disabled")
        self._passes_spin.configure(state="disabled" if running else "normal")

    def _poll_events(self):
        try:
            while True:
                kind, payload = self._events.get_nowait()
                if kind == "done":
                    self._status_var.set("Listo. Movimiento finalizado.")
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
    replanning_node = CoveragePathExecutorNode(
        parameter_overrides=[Parameter("execute", Parameter.Type.BOOL, True)]
    )

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(go_home_node)
    executor.add_node(replanning_node)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    root = tk.Tk()
    ControlPanelApp(root, go_home_node, replanning_node)
    try:
        root.mainloop()
    finally:
        executor.shutdown()
        executor_thread.join(timeout=1.0)
        go_home_node.destroy_node()
        replanning_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
