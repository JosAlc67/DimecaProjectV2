"""CSV metrics for reproducible coating-cell simulation runs."""

import csv
import math
import os
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path


def pose_path_length(poses) -> float:
    """Return Cartesian length of a sequence of geometry_msgs/Pose objects."""
    total = 0.0
    for first, second in zip(poses[:-1], poses[1:]):
        dx = second.position.x - first.position.x
        dy = second.position.y - first.position.y
        dz = second.position.z - first.position.z
        total += math.sqrt(dx * dx + dy * dy + dz * dz)
    return total


class RunMetrics:
    """Collect run events and append one machine-readable summary row."""

    SUMMARY_FIELDS = (
        "run_id",
        "started_at_utc",
        "finished_at_utc",
        "outcome",
        "passes_requested",
        "rows_generated",
        "waypoints_requested",
        "waypoints_completed_estimate",
        "cartesian_requests",
        "cartesian_fraction_mean",
        "planned_path_length_m",
        "execution_time_s",
        "observed_average_tcp_speed_mps_estimate",
        "configured_target_speed_mps",
        "configured_standoff_m",
        "replans_or_bypasses",
        "obstacle_waits",
        "spray_on_events",
        "spray_off_events",
        "failure_reason",
        "data_quality_notes",
    )

    EVENT_FIELDS = ("run_id", "timestamp_utc", "elapsed_s", "event", "details")

    def __init__(self, enabled=True, output_directory=""):
        self.enabled = bool(enabled)
        ros_log_dir = os.environ.get("ROS_LOG_DIR")
        default_dir = (
            Path(ros_log_dir, "dimeca_metrics")
            if ros_log_dir
            else Path(os.path.expanduser("~/.ros/dimeca_metrics"))
        )
        self.output_directory = Path(output_directory).expanduser() if output_directory else default_dir
        self.run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
        self.started_at = datetime.now(timezone.utc)
        self.started_monotonic = time.monotonic()
        self.execution_time_s = 0.0
        self.rows_generated = 0
        self.waypoints_requested = 0
        self.waypoints_completed = 0
        self.cartesian_fractions = []
        self.planned_path_length_m = 0.0
        self.replans_or_bypasses = 0
        self.obstacle_waits = 0
        self.spray_on_events = 0
        self.spray_off_events = 0
        self._events = []

    def event(self, name, details=""):
        if not self.enabled:
            return
        if name == "spray_on":
            self.spray_on_events += 1
        elif name == "spray_off":
            self.spray_off_events += 1
        elif name == "obstacle_wait":
            self.obstacle_waits += 1
        elif name in ("partial_path", "bypass_attempt"):
            self.replans_or_bypasses += 1
        now = datetime.now(timezone.utc)
        self._events.append(
            {
                "run_id": self.run_id,
                "timestamp_utc": now.isoformat(),
                "elapsed_s": f"{time.monotonic() - self.started_monotonic:.6f}",
                "event": name,
                "details": str(details),
            }
        )

    def cartesian_segment(self, poses, fraction, execution_time_s):
        fraction = max(0.0, min(1.0, float(fraction)))
        requested_length = pose_path_length(poses)
        self.cartesian_fractions.append(fraction)
        self.planned_path_length_m += requested_length * fraction
        self.waypoints_completed += round(len(poses) * fraction)
        self.execution_time_s += max(0.0, float(execution_time_s))

    def finish(
        self,
        outcome,
        passes_requested,
        configured_target_speed_mps,
        configured_standoff_m,
        failure_reason="",
    ):
        if not self.enabled:
            return None
        self.event("run_finished", outcome)
        self.output_directory.mkdir(parents=True, exist_ok=True)
        events_path = self.output_directory / f"events_{self.run_id}.csv"
        with events_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.EVENT_FIELDS)
            writer.writeheader()
            writer.writerows(self._events)

        fraction_mean = (
            sum(self.cartesian_fractions) / len(self.cartesian_fractions)
            if self.cartesian_fractions
            else 0.0
        )
        average_speed = (
            self.planned_path_length_m / self.execution_time_s
            if self.execution_time_s > 0.0
            else 0.0
        )
        summary = {
            "run_id": self.run_id,
            "started_at_utc": self.started_at.isoformat(),
            "finished_at_utc": datetime.now(timezone.utc).isoformat(),
            "outcome": outcome,
            "passes_requested": passes_requested,
            "rows_generated": self.rows_generated,
            "waypoints_requested": self.waypoints_requested,
            "waypoints_completed_estimate": self.waypoints_completed,
            "cartesian_requests": len(self.cartesian_fractions),
            "cartesian_fraction_mean": f"{fraction_mean:.6f}",
            "planned_path_length_m": f"{self.planned_path_length_m:.6f}",
            "execution_time_s": f"{self.execution_time_s:.6f}",
            "observed_average_tcp_speed_mps_estimate": f"{average_speed:.6f}",
            "configured_target_speed_mps": configured_target_speed_mps,
            "configured_standoff_m": configured_standoff_m,
            "replans_or_bypasses": self.replans_or_bypasses,
            "obstacle_waits": self.obstacle_waits,
            "spray_on_events": self.spray_on_events,
            "spray_off_events": self.spray_off_events,
            "failure_reason": failure_reason,
            "data_quality_notes": (
                "TCP speed is estimated from requested Cartesian distance and controller elapsed time; "
                "waypoints completed are estimated from MoveIt fraction. Actual TCP samples and surface "
                "measurements require the final CAD/measurement source."
            ),
        }
        summary_path = self.output_directory / "run_summary.csv"
        needs_header = not summary_path.exists() or summary_path.stat().st_size == 0
        with summary_path.open("a", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.SUMMARY_FIELDS)
            if needs_header:
                writer.writeheader()
            writer.writerow(summary)
        return summary_path, events_path
