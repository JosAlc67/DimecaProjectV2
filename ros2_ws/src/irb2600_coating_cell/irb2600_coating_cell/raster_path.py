"""Pure (no ROS node) generation of a raster/boustrophedon coating path over
the target structure's face, shared by trajectory_planner_node (plans the
whole thing at once, static-obstacle Phase 1) and replanning_executor_node
(plans/executes it row by row, reactive-replanning Phase 2).

Assumes target_structure.local_normal is aligned with the target structure's
local X axis (true for the default config/scene_objects.yaml); the raster
sweep is generated across the object's local Y (width) and Z (height) axes.
"""

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
