"""Geometry-only models shared by the simulated MotionCam and C3 sensors."""

import math

from irb2600_coating_cell.geometry_utils import (
    inverse_transform_point,
    rotate_vector_by_quaternion,
)


def visible_points_in_camera_frame(
    points_world,
    camera_position,
    camera_orientation,
    near_range_m,
    far_range_m,
    horizontal_fov_rad,
    vertical_fov_rad,
):
    """Return points visible by a +Z-forward optical camera frame.

    The MotionCam datasheet defines X right, Y down and Z toward the scene.
    This function keeps exactly that convention and returns camera-frame
    points ready for a PointCloud2 whose header is the optical frame.
    """
    visible = []
    half_horizontal = float(horizontal_fov_rad) / 2.0
    half_vertical = float(vertical_fov_rad) / 2.0
    for point in points_world:
        x, y, z = inverse_transform_point(point, camera_position, camera_orientation)
        distance = math.sqrt(x * x + y * y + z * z)
        if not float(near_range_m) <= distance <= float(far_range_m) or z <= 0.0:
            continue
        if abs(math.atan2(x, z)) > half_horizontal:
            continue
        if abs(math.atan2(y, z)) > half_vertical:
            continue
        visible.append((x, y, z))
    return visible


def ray_plane_distance(origin, orientation, plane_point, plane_normal):
    """Distance along the sensor +Z ray to a plane, or ``None`` if no hit."""
    direction = rotate_vector_by_quaternion((0.0, 0.0, 1.0), orientation)
    denominator = sum(a * b for a, b in zip(plane_normal, direction))
    if abs(denominator) < 1e-9:
        return None
    numerator = sum(
        plane_normal[index] * (plane_point[index] - origin[index])
        for index in range(3)
    )
    distance = numerator / denominator
    return distance if distance >= 0.0 else None
