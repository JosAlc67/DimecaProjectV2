"""Simulated RGB-D camera (report Sec. III-B / Table V / Table XVI):
Generates pose signals and synthetic 3D PointCloud2 messages (/camera/depth/color/points)
that feed MoveIt 2's Octomap 3D voxel updater in real time.

Published topics:
    structure_pose           (geometry_msgs/PoseStamped)
    surface_normal            (geometry_msgs/Vector3Stamped)
    obstacles/<name>/pose     (geometry_msgs/PoseStamped), one per obstacle
    workspace_clear           (std_msgs/Bool)
    /camera/depth/color/points (sensor_msgs/PointCloud2) for 3D Octomap occupancy
"""

import math
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2, PointField

from irb2600_coating_cell.geometry_utils import (
    point_segment_distance_2d,
    quaternion_from_rpy,
    rotate_vector_by_quaternion,
)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl


def create_point_cloud_2(header, points):
    """Helper to create sensor_msgs/PointCloud2 without external dependencies."""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    buffer = []
    for p in points:
        buffer.append(struct.pack("fff", float(p[0]), float(p[1]), float(p[2])))
    msg.data = b"".join(buffer)
    return msg


class PerceptionSimNode(Node):

    def __init__(self):
        super().__init__("perception_sim_node")

        self.declare_parameter("target_structure.frame_id", "world")
        self.declare_parameter("target_structure.position", [1.0, 0.0, 1.0])
        self.declare_parameter("target_structure.orientation_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("target_structure.local_normal", [-1.0, 0.0, 0.0])

        self.declare_parameter("obstacles", ["temporary_obstacle"])
        self._obstacle_names = list(self.get_parameter("obstacles").value)

        for name in self._obstacle_names:
            self.declare_parameter(f"{name}.frame_id", "world")
            self.declare_parameter(f"{name}.type", "box")
            self.declare_parameter(f"{name}.position", [0.5, 0.3, 1.0])
            self.declare_parameter(f"{name}.orientation_rpy", [0.0, 0.0, 0.0])
            self.declare_parameter(f"{name}.size", [0.15, 0.15, 0.8])

        self.declare_parameter("robot_base_xy", [0.0, 0.0])
        self.declare_parameter("d_safe", 0.05)
        self.declare_parameter("workspace_clear_margin", 0.15)
        self.declare_parameter("publish_rate_hz", 5.0)

        self._structure_pub = self.create_publisher(PoseStamped, "structure_pose", 10)
        self._normal_pub = self.create_publisher(Vector3Stamped, "surface_normal", 10)
        self._obstacle_pubs = {
            name: self.create_publisher(PoseStamped, f"obstacles/{name}/pose", 10)
            for name in self._obstacle_names
        }
        self._clear_pub = self.create_publisher(Bool, "workspace_clear", 10)
        self._pointcloud_pub = self.create_publisher(
            PointCloud2, "/camera/depth/color/points", 10
        )

        self._dynamic_poses = {}
        self._im_server = InteractiveMarkerServer(self, "perception_markers")
        self._init_interactive_markers()

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

    def _init_interactive_markers(self):
        # Create a marker for each obstacle
        for name in self._obstacle_names:
            pos = [float(v) for v in self.get_parameter(f"{name}.position").value]
            rpy = [float(v) for v in self.get_parameter(f"{name}.orientation_rpy").value]
            self._create_6dof_marker(name, self.get_parameter(f"{name}.frame_id").value, pos, rpy)
            
        # Create a marker for the target structure
        pos = [float(v) for v in self.get_parameter("target_structure.position").value]
        rpy = [float(v) for v in self.get_parameter("target_structure.orientation_rpy").value]
        self._create_6dof_marker("target_structure", self.get_parameter("target_structure.frame_id").value, pos, rpy)
        
        self._im_server.applyChanges()

    def _create_6dof_marker(self, name, frame_id, position, rpy):
        im = InteractiveMarker()
        im.header.frame_id = frame_id
        im.name = name
        im.description = f"Drag {name}"
        im.scale = 0.4
        
        im.pose.position.x = position[0]
        im.pose.position.y = position[1]
        im.pose.position.z = position[2]
        im.pose.orientation = quaternion_from_rpy(*rpy)

        # Add a visual cube
        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        
        from visualization_msgs.msg import Marker
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        visual_control.markers.append(marker)
        
        im.controls.append(visual_control)

        # Move 3D control
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        im.controls.append(control)

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        im.controls.append(control)

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        im.controls.append(control)

        self._im_server.insert(im, feedback_callback=self._process_feedback)

    def _process_feedback(self, feedback):
        name = feedback.marker_name
        self._dynamic_poses[name] = feedback.pose

    def _on_timer(self):
        now = self.get_clock().now().to_msg()
        p = self.get_parameter

        structure_frame = p("target_structure.frame_id").value
        structure_rpy = [float(v) for v in p("target_structure.orientation_rpy").value]

        if "target_structure" in self._dynamic_poses:
            dynamic_pose = self._dynamic_poses["target_structure"]
            structure_pos = [dynamic_pose.position.x, dynamic_pose.position.y, dynamic_pose.position.z]
            structure_quat = dynamic_pose.orientation
        else:
            structure_pos = [float(v) for v in p("target_structure.position").value]
            structure_quat = quaternion_from_rpy(*structure_rpy)

        structure_msg = PoseStamped()
        structure_msg.header.stamp = now
        structure_msg.header.frame_id = structure_frame
        (
            structure_msg.pose.position.x,
            structure_msg.pose.position.y,
            structure_msg.pose.position.z,
        ) = (float(v) for v in structure_pos)
        structure_msg.pose.orientation = structure_quat
        self._structure_pub.publish(structure_msg)

        local_normal = [float(v) for v in p("target_structure.local_normal").value]
        nx, ny, nz = rotate_vector_by_quaternion(local_normal, structure_quat)
        normal_msg = Vector3Stamped()
        normal_msg.header.stamp = now
        normal_msg.header.frame_id = structure_frame
        normal_msg.vector.x, normal_msg.vector.y, normal_msg.vector.z = nx, ny, nz
        self._normal_pub.publish(normal_msg)

        all_points = []
        workspace_clear = True

        for name in self._obstacle_names:
            obstacle_frame = p(f"{name}.frame_id").value
            obstacle_size = [float(v) for v in p(f"{name}.size").value]
            
            if name in self._dynamic_poses:
                dynamic_pose = self._dynamic_poses[name]
                obstacle_pos = [dynamic_pose.position.x, dynamic_pose.position.y, dynamic_pose.position.z]
                obs_quat = dynamic_pose.orientation
            else:
                obstacle_pos = [float(v) for v in p(f"{name}.position").value]
                obstacle_rpy = [float(v) for v in p(f"{name}.orientation_rpy").value]
                obs_quat = quaternion_from_rpy(*obstacle_rpy)

            obstacle_msg = PoseStamped()
            obstacle_msg.header.stamp = now
            obstacle_msg.header.frame_id = obstacle_frame
            (
                obstacle_msg.pose.position.x,
                obstacle_msg.pose.position.y,
                obstacle_msg.pose.position.z,
            ) = obstacle_pos
            obstacle_msg.pose.orientation = obs_quat
            self._obstacle_pubs[name].publish(obstacle_msg)

            # Sample surface 3D points for this obstacle to feed Octomap
            obstacle_type = self.get_parameter(f"{name}.type").value
            if obstacle_type == "cylinder":
                obs_pts = self._sample_cylinder_surface_points(obstacle_pos, obs_quat, obstacle_size)
            else:
                obs_pts = self._sample_box_surface_points(obstacle_pos, obs_quat, obstacle_size)
            all_points.extend(obs_pts)

            if not self._is_obstacle_clear(name, obstacle_pos, structure_pos):
                workspace_clear = False

        clear_msg = Bool()
        clear_msg.data = workspace_clear
        self._clear_pub.publish(clear_msg)

        # Publish PointCloud2 if points exist
        if all_points:
            header = structure_msg.header
            pc2_msg = create_point_cloud_2(header, all_points)
            self._pointcloud_pub.publish(pc2_msg)

    def _sample_box_surface_points(self, pos, quat, size):
        """Generates a grid of 3D surface points in world frame representing the box volume."""
        points = []
        dx, dy, dz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
        step = 0.05  # 5 cm point spacing

        nx_steps = max(int(size[0] / step), 2)
        ny_steps = max(int(size[1] / step), 2)
        nz_steps = max(int(size[2] / step), 2)

        for ix in range(nx_steps):
            x = -dx + ix * (size[0] / max(nx_steps - 1, 1))
            for iy in range(ny_steps):
                y = -dy + iy * (size[1] / max(ny_steps - 1, 1))
                for iz in range(nz_steps):
                    z = -dz + iz * (size[2] / max(nz_steps - 1, 1))
                    if ix in (0, nx_steps - 1) or iy in (0, ny_steps - 1) or iz in (0, nz_steps - 1):
                        rot_x, rot_y, rot_z = rotate_vector_by_quaternion((x, y, z), quat)
                        points.append((pos[0] + rot_x, pos[1] + rot_y, pos[2] + rot_z))
        return points

    def _sample_cylinder_surface_points(self, pos, quat, size):
        """Generates a grid of 3D surface points in world frame representing the cylinder volume.
        size = [height, radius, _unused]"""
        points = []
        height, radius = size[0], size[1]
        step = 0.05
        
        nz_steps = max(int(height / step), 2)
        ntheta_steps = max(int((2 * math.pi * radius) / step), 8)
        
        for iz in range(nz_steps):
            z = -height/2.0 + iz * (height / max(nz_steps - 1, 1))
            for itheta in range(ntheta_steps):
                theta = itheta * (2 * math.pi / ntheta_steps)
                x = radius * math.cos(theta)
                y = radius * math.sin(theta)
                rot_x, rot_y, rot_z = rotate_vector_by_quaternion((x, y, z), quat)
                points.append((pos[0] + rot_x, pos[1] + rot_y, pos[2] + rot_z))
                
        # Top and bottom caps
        nrad_steps = max(int(radius / step), 2)
        for iz in [0, nz_steps - 1]:
            z = -height/2.0 + iz * (height / max(nz_steps - 1, 1))
            for ir in range(1, nrad_steps):
                r = ir * (radius / max(nrad_steps - 1, 1))
                circ_steps = max(int((2 * math.pi * r) / step), 4)
                for itheta in range(circ_steps):
                    theta = itheta * (2 * math.pi / circ_steps)
                    x = r * math.cos(theta)
                    y = r * math.sin(theta)
                    rot_x, rot_y, rot_z = rotate_vector_by_quaternion((x, y, z), quat)
                    points.append((pos[0] + rot_x, pos[1] + rot_y, pos[2] + rot_z))
                    
        return points

    def _is_obstacle_clear(self, name, obstacle_pos, structure_pos):
        base_x, base_y = (float(v) for v in self.get_parameter("robot_base_xy").value)
        obstacle_size = [float(v) for v in self.get_parameter(f"{name}.size").value]
        d_safe = float(self.get_parameter("d_safe").value)
        margin = float(self.get_parameter("workspace_clear_margin").value)

        obstacle_bounding_radius = 0.5 * max(obstacle_size[0], obstacle_size[1])
        distance = point_segment_distance_2d(
            obstacle_pos[0], obstacle_pos[1], base_x, base_y, structure_pos[0], structure_pos[1]
        )
        return distance > (obstacle_bounding_radius + d_safe + margin)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
