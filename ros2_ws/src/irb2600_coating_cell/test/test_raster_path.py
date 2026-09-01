import math

import pytest
from geometry_msgs.msg import Pose

from irb2600_coating_cell.geometry_utils import (
    quaternion_with_z_axis,
    rotate_vector_by_quaternion,
)
from irb2600_coating_cell.raster_path import resample_rows


def _pose(x, y, z, approach=(1.0, 0.0, 0.0)):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    pose.orientation = quaternion_with_z_axis(approach)
    return pose


def test_resample_rows_produces_exact_requested_count_and_preserves_endpoints():
    rows = [
        [_pose(0.0, 0.0, 0.0), _pose(0.0, 2.0, 0.0)],
        [_pose(0.0, 2.0, 1.0), _pose(0.0, 0.0, 1.0)],
    ]
    result = resample_rows(rows, 10)

    assert sum(len(row) for row in result) == 10
    assert result[0][0].position.y == pytest.approx(0.0)
    assert result[0][-1].position.y == pytest.approx(2.0)
    assert result[1][0].position.y == pytest.approx(2.0)
    assert result[1][-1].position.y == pytest.approx(0.0)


def test_resample_rows_rejects_count_too_small_to_preserve_rows():
    rows = [[_pose(0, 0, 0), _pose(0, 1, 0)] for _ in range(3)]
    with pytest.raises(ValueError, match="at least 6"):
        resample_rows(rows, 5)


def test_tcp_quaternion_aligns_local_z_with_requested_approach():
    requested = (1.0, 2.0, -0.5)
    quaternion = quaternion_with_z_axis(requested)
    actual = rotate_vector_by_quaternion((0.0, 0.0, 1.0), quaternion)
    norm = math.sqrt(sum(value * value for value in requested))

    assert actual == pytest.approx(tuple(value / norm for value in requested), abs=1e-7)
