import math

import pytest

from irb2600_coating_cell.geometry_utils import quaternion_from_rpy
from irb2600_coating_cell.sensor_geometry import (
    ray_plane_distance,
    visible_points_in_camera_frame,
)


def test_motioncam_filters_by_fov_and_range_and_returns_camera_coordinates():
    identity = quaternion_from_rpy(0.0, 0.0, 0.0)
    visible = visible_points_in_camera_frame(
        [(0.0, 0.0, 1.0), (1.1, 0.0, 1.0), (0.0, 0.0, 4.0)],
        (0.0, 0.0, 0.0),
        identity,
        0.5,
        3.0,
        math.pi / 2.0,
        math.pi / 2.0,
    )
    assert visible == [(0.0, 0.0, 1.0)]


def test_c3_plane_distance_uses_sensor_forward_axis():
    identity = quaternion_from_rpy(0.0, 0.0, 0.0)
    assert ray_plane_distance(
        (0.0, 0.0, 0.0), identity, (0.0, 0.0, 0.08), (0.0, 0.0, -1.0)
    ) == pytest.approx(0.08)
