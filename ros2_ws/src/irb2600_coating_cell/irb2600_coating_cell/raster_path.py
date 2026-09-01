"""Pure (no ROS node) generation and resampling of raster coating paths over
the target structure's face, shared by trajectory_planner_node (plans the
whole thing at once, static-obstacle Phase 1) and replanning_executor_node
(plans/executes it row by row, reactive-replanning Phase 2).

Assumes target_structure.local_normal is aligned with the target structure's
local X axis (true for the default config/scene_objects.yaml); the raster
sweep is generated across the object's local Y (width) and Z (height) axes.
"""

import math

from geometry_msgs.msg import Pose

from irb2600_coating_cell.geometry_utils import (
    quaternion_from_rpy,
    quaternion_with_z_axis,
    rotate_vector_by_quaternion,
)


def generate_raster_rows(
    position, rpy, size, local_normal, d_standoff, edge_margin, row_pitch
):
    """Returns (rows, normal_world), where `rows` is a list of rows and each
    row is a 2-element list of geometry_msgs/Pose [start, end] for one raster
    pass (waypoint order alternates direction row to row, boustrophedon-style).
    """
    position = [float(v) for v in position]
    rpy = [float(v) for v in rpy]
    size = [float(v) for v in size]
    local_normal = [float(v) for v in local_normal]
    d_standoff = float(d_standoff)
    edge_margin = float(edge_margin)
    row_pitch = float(row_pitch)

    structure_quat = quaternion_from_rpy(*rpy)
    normal_world = rotate_vector_by_quaternion(local_normal, structure_quat)

    width_eff = max(size[1] - 2.0 * edge_margin, 0.0)
    height_eff = max(size[2] - 2.0 * edge_margin, 0.0)
    n_rows = max(int(round(height_eff / row_pitch)) + 1, 1)

    rows = []
    for row in range(n_rows):
        local_z = -height_eff / 2.0 + row * (height_eff / max(n_rows - 1, 1))
        y_start, y_end = -width_eff / 2.0, width_eff / 2.0
        if row % 2 == 1:
            y_start, y_end = y_end, y_start

        row_poses = []
        for local_y in (y_start, y_end):
            # Point on the panel face, in the panel's local frame, offset by
            # half the thickness towards the working face (local_normal),
            # then pushed out by d_standoff.
            local_point = (
                local_normal[0] * (size[0] / 2.0),
                local_y,
                local_z,
            )
            surface_point_world = rotate_vector_by_quaternion(local_point, structure_quat)
            x = position[0] + surface_point_world[0] + normal_world[0] * d_standoff
            y = position[1] + surface_point_world[1] + normal_world[1] * d_standoff
            z = position[2] + surface_point_world[2] + normal_world[2] * d_standoff

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            # Physically, the nozzle's approach vector must point INTO the
            # surface (like any real spray/weld/machining tool), i.e.
            # antiparallel to the outward surface normal n_s used for the
            # standoff offset above -- NOT parallel to it, even though eq. 9
            # (theta_error = arccos(z_e . n_s) <= 10 deg) reads as if z_e
            # should equal n_s directly. Using +n_s (tool pointing away from
            # the panel, back over its own shoulder) was tested on hardware
            # and made nearly the entire raster kinematically unreachable
            # (fraction ~0 with IK/collision both failing regardless of any
            # obstacle). Whichever eventually computes eq. 9 as a reported
            # metric should measure the angle against -n_s (equivalently,
            # against the surface's inward normal) to match this convention.
            approach_direction = tuple(-c for c in normal_world)
            pose.orientation = quaternion_with_z_axis(approach_direction)
            row_poses.append(pose)

        rows.append(row_poses)

    return rows, normal_world


def flatten_rows(rows):
    return [pose for row in rows for pose in row]


def _distance(first, second):
    return math.sqrt(
        (second.position.x - first.position.x) ** 2
        + (second.position.y - first.position.y) ** 2
        + (second.position.z - first.position.z) ** 2
    )


def _interpolate_pose(first, second, ratio):
    pose = Pose()
    pose.position.x = first.position.x + ratio * (second.position.x - first.position.x)
    pose.position.y = first.position.y + ratio * (second.position.y - first.position.y)
    pose.position.z = first.position.z + ratio * (second.position.z - first.position.z)
    # Normalized linear interpolation is sufficient here because adjacent
    # mesh samples have close orientations. Flip sign to take the short arc.
    qa = (first.orientation.x, first.orientation.y, first.orientation.z, first.orientation.w)
    qb = (second.orientation.x, second.orientation.y, second.orientation.z, second.orientation.w)
    if sum(a * b for a, b in zip(qa, qb)) < 0.0:
        qb = tuple(-value for value in qb)
    values = [a + ratio * (b - a) for a, b in zip(qa, qb)]
    norm = math.sqrt(sum(value * value for value in values)) or 1.0
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = (value / norm for value in values)
    return pose


def resample_rows(rows, requested_waypoints):
    """Resample rows by arc length while preserving row boundaries exactly."""
    requested_waypoints = int(requested_waypoints)
    if requested_waypoints <= 0:
        return rows
    nonempty = [row for row in rows if row]
    minimum = 2 * len(nonempty)
    if requested_waypoints < minimum:
        raise ValueError(
            f"requested_waypoints={requested_waypoints} is too small for "
            f"{len(nonempty)} raster rows; at least {minimum} are required"
        )

    lengths = [sum(_distance(a, b) for a, b in zip(row[:-1], row[1:])) for row in nonempty]
    total_length = sum(lengths)
    remaining = requested_waypoints - minimum
    raw_extras = [remaining * length / total_length if total_length else 0.0 for length in lengths]
    extras = [int(value) for value in raw_extras]
    for index in sorted(
        range(len(extras)), key=lambda i: raw_extras[i] - extras[i], reverse=True
    )[: remaining - sum(extras)]:
        extras[index] += 1

    result = []
    for row, extra, row_length in zip(nonempty, extras, lengths):
        target_count = 2 + extra
        if len(row) == 1 or row_length <= 1e-12:
            result.append([_interpolate_pose(row[0], row[0], 0.0) for _ in range(target_count)])
            continue
        cumulative = [0.0]
        for first, second in zip(row[:-1], row[1:]):
            cumulative.append(cumulative[-1] + _distance(first, second))
        sampled = []
        segment = 0
        for sample_index in range(target_count):
            distance = row_length * sample_index / (target_count - 1)
            while segment < len(row) - 2 and cumulative[segment + 1] < distance:
                segment += 1
            segment_length = cumulative[segment + 1] - cumulative[segment]
            ratio = (distance - cumulative[segment]) / segment_length if segment_length else 0.0
            sampled.append(_interpolate_pose(row[segment], row[segment + 1], ratio))
        result.append(sampled)
    return result
