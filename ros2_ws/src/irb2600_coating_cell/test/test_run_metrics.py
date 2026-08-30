import csv

import pytest
from geometry_msgs.msg import Pose

from irb2600_coating_cell.run_metrics import RunMetrics, pose_path_length


def _pose(x):
    pose = Pose()
    pose.position.x = x
    pose.orientation.w = 1.0
    return pose


def test_pose_path_length():
    assert pose_path_length([_pose(0.0), _pose(1.5), _pose(2.0)]) == pytest.approx(2.0)


def test_metrics_write_events_and_append_summary(tmp_path):
    metrics = RunMetrics(enabled=True, output_directory=str(tmp_path))
    metrics.rows_generated = 1
    metrics.waypoints_requested = 3
    metrics.event("spray_on", "confirmed")
    metrics.cartesian_segment([_pose(0.0), _pose(2.0)], 0.5, 2.0)
    paths = metrics.finish("completed", 1, 0.0, 0.15)

    summary_path, events_path = paths
    assert summary_path.exists()
    assert events_path.exists()
    with summary_path.open(newline="", encoding="utf-8") as stream:
        summary = list(csv.DictReader(stream))
    assert summary[0]["outcome"] == "completed"
    assert float(summary[0]["planned_path_length_m"]) == pytest.approx(1.0)
    assert float(summary[0]["observed_average_tcp_speed_mps_estimate"]) == pytest.approx(0.5)
    assert summary[0]["spray_on_events"] == "1"
