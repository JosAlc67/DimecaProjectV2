"""Small self-contained geometry helpers (no tf_transformations dependency,
since it is not guaranteed to be installed alongside a bare ROS 2 Humble +
MoveIt 2 setup).
"""

import math

from geometry_msgs.msg import Quaternion


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Standard ZYX Euler -> quaternion conversion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def rotate_vector_by_quaternion(vector, q: Quaternion):
    """Rotate a 3-tuple/list `vector` by quaternion q, return (x, y, z)."""
    vx, vy, vz = vector
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    # v' = q * v * q_conj, expanded closed-form.
    uvx = qy * vz - qz * vy
    uvy = qz * vx - qx * vz
    uvz = qx * vy - qy * vx

    uuvx = qy * uvz - qz * uvy
    uuvy = qz * uvx - qx * uvz
    uuvz = qx * uvy - qy * uvx

    rx = vx + 2.0 * (qw * uvx + uuvx)
    ry = vy + 2.0 * (qw * uvy + uuvy)
    rz = vz + 2.0 * (qw * uvz + uuvz)
    return (rx, ry, rz)


def inverse_rotate_vector_by_quaternion(vector, q: Quaternion):
    """Rotate ``vector`` from a child frame into the parent inverse frame."""
    inverse = Quaternion()
    inverse.x, inverse.y, inverse.z, inverse.w = -q.x, -q.y, -q.z, q.w
    return rotate_vector_by_quaternion(vector, inverse)


def transform_point(point, position, orientation: Quaternion):
    """Transform a local 3D point by a pose, without depending on tf2."""
    rotated = rotate_vector_by_quaternion(point, orientation)
    return tuple(float(position[index]) + rotated[index] for index in range(3))


def inverse_transform_point(point, position, orientation: Quaternion):
    """Express a parent-frame point in the local frame of a pose."""
    translated = tuple(float(point[index]) - float(position[index]) for index in range(3))
    return inverse_rotate_vector_by_quaternion(translated, orientation)


def _vec_normalize(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n < 1e-9:
        raise ValueError("cannot normalize a zero-length vector")
    return (v[0] / n, v[1] / n, v[2] / n)


def _vec_cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def quaternion_with_z_axis(target_z_axis) -> Quaternion:
    """Build an orientation whose local +Z axis equals `target_z_axis`
    (world frame, need not be normalized). The rotation about that axis is
    left free and resolved by keeping the local +Y axis as close as possible
    to world +Z ("up"), which gives stable, non-twisted waypoints for a
    raster path swept across a roughly vertical panel. Falls back to world
    +X as the reference when target_z_axis is close to vertical.
    """
    z_axis = _vec_normalize(target_z_axis)

    world_up = (0.0, 0.0, 1.0)
    reference = world_up if abs(_vec_dot(z_axis, world_up)) < 0.99 else (1.0, 0.0, 0.0)

    x_axis = _vec_normalize(_vec_cross(reference, z_axis))
    y_axis = _vec_cross(z_axis, x_axis)

    # Rotation matrix with columns [x_axis | y_axis | z_axis] -> quaternion
    # (standard Shepperd's method).
    m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
    m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
    m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]

    trace = m00 + m11 + m22
    q = Quaternion()
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        q.w = 0.25 / s
        q.x = (m21 - m12) * s
        q.y = (m02 - m20) * s
        q.z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        q.w = (m21 - m12) / s
        q.x = 0.25 * s
        q.y = (m01 + m10) / s
        q.z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        q.w = (m02 - m20) / s
        q.x = (m01 + m10) / s
        q.y = 0.25 * s
        q.z = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        q.w = (m10 - m01) / s
        q.x = (m02 + m20) / s
        q.y = (m12 + m21) / s
        q.z = 0.25 * s
    return q


def point_segment_distance_2d(px, py, ax, ay, bx, by) -> float:
    """Shortest distance from point P to segment AB, in the XY plane."""
    abx, aby = (bx - ax), (by - ay)
    seg_len_sq = abx * abx + aby * aby
    if seg_len_sq < 1e-12:
        dx, dy = (px - ax), (py - ay)
        return math.hypot(dx, dy)

    t = ((px - ax) * abx + (py - ay) * aby) / seg_len_sq
    t = max(0.0, min(1.0, t))
    closest_x = ax + t * abx
    closest_y = ay + t * aby
    return math.hypot(px - closest_x, py - closest_y)
