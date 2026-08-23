"""Inserts the target structure and the temporary obstacles (report Table VIII)
into the MoveIt planning scene as collision objects, via the standard
/apply_planning_scene service exposed by move_group. Using that service
instead of moveit_py/pymoveit2 keeps this node dependency-light and
independent of which Python MoveIt binding ends up being available.

The obstacle set is a named list (the `obstacles` parameter, e.g.
["scaffold_pole", "tool_cart", "cable_reel"] in the default
config/scene_objects.yaml) with each name having its own
<name>.type/frame_id/position/orientation_rpy/size parameters -- the
standard ROS 2 pattern for a dynamic list of named sub-configs. One
CollisionObject is inserted per name (id = name).

Subscribes to perception_sim_node's /obstacles/<name>/pose and
/workspace_clear topics so the "simulated sensor" actually drives the
planning scene instead of just running in parallel unused: previously,
moving an obstacle meant manually calling `ros2 param set` on THIS node
plus `~/refresh_scene`, completely bypassing perception_sim_node. Now,
changing an obstacle's position on perception_sim_node's own parameters
(simulating "the camera observed the obstacle move") is enough -- this
node picks up the new pose per obstacle and re-applies the planning scene
automatically, no manual refresh needed. The <name>.* parameters remain as
the fallback used before any sensor reading has arrived for that obstacle
(e.g. when running this node standalone without perception_sim_node).

Also publishes the target + all obstacles as a plain
visualization_msgs/MarkerArray on ~/scene_markers. This project hit three
separate, unrelated bugs in moveit_rviz_plugin on ROS 2 Humble while
testing this (a nonexistent Panel class, a joint_limits type-mismatch
exception in its robot-model loader, and finally
moveit_rviz_plugin/PlanningScene never actually subscribing to
/monitored_planning_scene despite move_group publishing it -- confirmed
with `ros2 topic info /monitored_planning_scene` showing 1 publisher, 0
subscribers). Rather than keep chasing bugs in that plugin, the markers use
only rviz_default_plugins/MarkerArray, which has no dependency on MoveIt's
RViz integration. Published with TRANSIENT_LOCAL durability so RViz gets
them immediately on connecting, regardless of launch ordering.

    ros2 run irb2600_coating_cell scene_setup_node

To "move an obstacle" with perception_sim_node running (the scene updates
automatically, no refresh needed):

    ros2 param set /perception_sim_node scaffold_pole.position "[0.5, 0.1, 1.0]"

Without perception_sim_node running, fall back to setting this node's own
parameters directly and re-applying by hand:

    ros2 param set /scene_setup_node scaffold_pole.position "[0.5, 0.1, 1.0]"
    ros2 service call /scene_setup_node/refresh_scene std_srvs/srv/Trigger
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from irb2600_coating_cell.geometry_utils import quaternion_from_rpy

_TARGET_COLOR = ColorRGBA(r=0.2, g=0.5, b=1.0, a=1.0)

# Cycled by obstacle index so an arbitrary-length obstacle list always gets
# a distinct, readable color instead of everything defaulting to the same
# shade.
_OBSTACLE_PALETTE = [
    ColorRGBA(r=1.0, g=0.25, b=0.1, a=0.8),  # red-orange
    ColorRGBA(r=0.9, g=0.65, b=0.05, a=0.8),  # amber
    ColorRGBA(r=0.6, g=0.2, b=0.85, a=0.8),  # purple
    ColorRGBA(r=0.1, g=0.7, b=0.4, a=0.8),  # green
    ColorRGBA(r=0.9, g=0.2, b=0.55, a=0.8),  # magenta
]


class SceneSetupNode(Node):

    def __init__(self):
        super().__init__("scene_setup_node")

        self.declare_parameter("target_structure.frame_id", "world")
        self.declare_parameter("target_structure.position", [1.5, -2.0, 1.5])
        self.declare_parameter("target_structure.orientation_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("target_structure.size", [0.05, 5.0, 2.0])

        # Fallback single-obstacle name if the "obstacles" parameter isn't
        # provided by a params file (e.g. running this node standalone).
        self.declare_parameter("obstacles", [])
        self._obstacle_names = list(self.get_parameter("obstacles").value)

        for name in self._obstacle_names:
            self.declare_parameter(f"{name}.frame_id", "world")
            self.declare_parameter(f"{name}.type", "box")
            self.declare_parameter(f"{name}.position", [0.5, 0.3, 1.0])
            self.declare_parameter(f"{name}.orientation_rpy", [0.0, 0.0, 0.0])
            self.declare_parameter(f"{name}.size", [0.15, 0.15, 0.8])

        # Last pose received per obstacle from perception_sim_node's
        # /obstacles/<name>/pose; a name missing from this dict means no
        # sensor reading has arrived yet, so its <name>.* parameters above
        # are used instead.
        self._sensor_obstacle_poses = {}
        self._sensor_target_pose = None
        self._workspace_clear = None

        self._apply_scene_client = self.create_client(
            ApplyPlanningScene, "apply_planning_scene"
        )
        self.create_service(Trigger, "~/refresh_scene", self._on_refresh_scene)
        
        self.create_subscription(PoseStamped, "structure_pose", self._on_structure_pose, 10)
        
        for name in self._obstacle_names:
            self.create_subscription(
                PoseStamped,
                f"/obstacles/{name}/pose",
                self._make_obstacle_pose_callback(name),
                10,
            )
        self.create_subscription(Bool, "/workspace_clear", self._on_workspace_clear, 10)

        transient_local_qos = QoSProfile(depth=1)
        transient_local_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._marker_pub = self.create_publisher(
            MarkerArray, "~/scene_markers", transient_local_qos
        )
        self.create_timer(2.0, self._publish_markers)

        self.get_logger().info(
            f"Waiting for /apply_planning_scene (provided by move_group)... "
            f"obstacles={self._obstacle_names}"
        )
        if self._apply_scene_client.wait_for_service(timeout_sec=5.0):
            self._apply_scene()
        else:
            self.get_logger().error("Service /apply_planning_scene not available, proceeding to spin anyway so markers can be published.")
            # We still publish markers even if move_group isn't up
            self._publish_markers()

    def _make_obstacle_pose_callback(self, name):
        def callback(msg):
            self._on_sensor_obstacle_pose(name, msg)

        return callback

    def _make_box_object(self, object_id, frame_id, position, orientation, size):
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
        pose.orientation = orientation

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        return obj

    def _make_cylinder_object(self, object_id, frame_id, position, orientation, size):
        # size = [height, radius, _unused]
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [float(size[0]), float(size[1])]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
        pose.orientation = orientation

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        return obj

    def _target_pose(self):
        p = self.get_parameter
        frame_id = p("target_structure.frame_id").value
        size = p("target_structure.size").value
        
        if self._sensor_target_pose is not None:
            position = [self._sensor_target_pose.position.x, self._sensor_target_pose.position.y, self._sensor_target_pose.position.z]
            orientation = self._sensor_target_pose.orientation
        else:
            position = [float(v) for v in p("target_structure.position").value]
            orientation = quaternion_from_rpy(
                *[float(v) for v in p("target_structure.orientation_rpy").value]
            )
        return frame_id, position, orientation, size

    def _obstacle_pose(self, name):
        """(frame_id, position, orientation, size, type) for one obstacle,
        preferring the latest /obstacles/<name>/pose reading over its
        <name>.* parameters once one has arrived."""
        p = self.get_parameter
        frame_id = p(f"{name}.frame_id").value
        size = p(f"{name}.size").value
        obstacle_type = p(f"{name}.type").value

        sensor_pose = self._sensor_obstacle_poses.get(name)
        if sensor_pose is not None:
            position = [sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z]
            orientation = sensor_pose.orientation
        else:
            position = [float(v) for v in p(f"{name}.position").value]
            orientation = quaternion_from_rpy(
                *[float(v) for v in p(f"{name}.orientation_rpy").value]
            )

        return frame_id, position, orientation, size, obstacle_type

    def _build_collision_objects(self):
        objects = []
        
        # Add the target structure as a collision object so it renders in RViz
        target_frame, target_pos, target_quat, target_size = self._target_pose()
        objects.append(
            self._make_box_object("target_structure", target_frame, target_pos, target_quat, target_size)
        )

        for name in self._obstacle_names:
            frame_id, position, orientation, size, obstacle_type = self._obstacle_pose(name)
            if obstacle_type == "cylinder":
                objects.append(
                    self._make_cylinder_object(name, frame_id, position, orientation, size)
                )
            else:
                objects.append(
                    self._make_box_object(name, frame_id, position, orientation, size)
                )

        return objects

    def _object_colors(self):
        colors = {"target_structure": _TARGET_COLOR}
        for i, name in enumerate(self._obstacle_names):
            colors[name] = _OBSTACLE_PALETTE[i % len(_OBSTACLE_PALETTE)]
        return colors

    def _make_marker(self, marker_id, object_id, frame_id, position, orientation, size, shape_type, color):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "coating_cell"
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            float(v) for v in position
        )
        marker.pose.orientation = orientation
        marker.color = color

        if shape_type == "cylinder":
            # size = [height, radius, _unused] (SolidPrimitive convention);
            # Marker.CYLINDER scale is [diameter_x, diameter_y, height].
            marker.type = Marker.CYLINDER
            marker.scale.x = 2.0 * float(size[1])
            marker.scale.y = 2.0 * float(size[1])
            marker.scale.z = float(size[0])
        else:
            marker.type = Marker.CUBE
            marker.scale.x, marker.scale.y, marker.scale.z = (float(v) for v in size)

        return marker

    def _make_point_marker(self, marker_id, frame_id, position, color, scale=0.05):
        """A small sphere marking a single reference point (e.g. the target
        structure's center, used as the origin for the raster coating path
        in raster_path.py) -- distinct from the box/cylinder object markers."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "coating_cell_points"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            float(v) for v in position
        )
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color = color
        return marker

    def _build_markers(self):
        colors = self._object_colors()
        target_frame, target_position, target_orientation, target_size = self._target_pose()
        markers = [
            self._make_marker(
                999, "target_structure", target_frame, target_position, target_orientation, target_size, "box", colors["target_structure"]
            ),
        ]

        for i, name in enumerate(self._obstacle_names):
            frame_id, position, orientation, size, obstacle_type = self._obstacle_pose(name)
            markers.append(
                self._make_marker(
                    i + 1, name, frame_id, position, orientation, size, obstacle_type, colors[name]
                )
            )

        return MarkerArray(markers=markers)

    def _apply_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = self._build_collision_objects()
        # Distinct colors so every object reads clearly at a glance in
        # RViz. Collision-object primitives have no color of their own in
        # moveit_msgs/CollisionObject; PlanningScene.object_colors is the
        # mechanism moveit_rviz_plugin/PlanningScene reads (see
        # _build_markers/_object_colors above for the actually-reliable
        # rviz_default_plugins/MarkerArray equivalent).
        scene.object_colors = [
            ObjectColor(id=object_id, color=color)
            for object_id, color in self._object_colors().items()
        ]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._apply_scene_client.call_async(request)
        future.add_done_callback(self._on_apply_scene_done)

        self._publish_markers()

    def _publish_markers(self):
        self._marker_pub.publish(self._build_markers())

    def _on_apply_scene_done(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - just logging, not recovering
            self.get_logger().error(f"apply_planning_scene call failed: {exc}")
            return
        if response.success:
            self.get_logger().info(
                "Planning scene updated: target_structure + "
                f"{', '.join(self._obstacle_names)}."
            )
        else:
            self.get_logger().error("move_group rejected the planning scene update.")

    def _on_refresh_scene(self, request, response):
        self._apply_scene()
        response.success = True
        response.message = "Planning scene re-applied from current parameters."
        return response

    def _on_workspace_clear(self, msg):
        if self._workspace_clear != msg.data:
            state = "clear" if msg.data else "NOT clear (obstacle nearby)"
            self.get_logger().info(f"Perception: workspace_clear -> {state}.")
        self._workspace_clear = msg.data

    def _on_structure_pose(self, msg):
        new_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        previous = self._sensor_target_pose
        first_reading = previous is None
        moved = True
        if not first_reading:
            old = previous.position
            moved = max(abs(n - o) for n, o in zip(new_position, (old.x, old.y, old.z))) > 0.01

        self._sensor_target_pose = msg.pose

        if first_reading or moved:
            x, y, z = new_position
            if first_reading:
                msg_text = f"Perception: first reading for 'target_structure' at ({x:.3f}, {y:.3f}, {z:.3f})"
            else:
                msg_text = f"Perception: 'target_structure' moved to ({x:.3f}, {y:.3f}, {z:.3f})"
            self.get_logger().info(f"{msg_text} -- refreshing planning scene.")
            self._apply_scene()

    def _on_sensor_obstacle_pose(self, name, msg):
        new_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        previous = self._sensor_obstacle_poses.get(name)
        first_reading = previous is None
        moved = True
        if not first_reading:
            old = previous.position
            moved = max(abs(n - o) for n, o in zip(new_position, (old.x, old.y, old.z))) > 0.01

        self._sensor_obstacle_poses[name] = msg.pose

        if first_reading or moved:
            x, y, z = new_position
            if first_reading:
                msg_text = f"Perception: first reading for '{name}' at ({x:.3f}, {y:.3f}, {z:.3f})"
            else:
                msg_text = f"Perception: '{name}' moved to ({x:.3f}, {y:.3f}, {z:.3f})"
            self.get_logger().info(f"{msg_text} -- refreshing planning scene.")
            self._apply_scene()


def main(args=None):
    rclpy.init(args=args)
    node = SceneSetupNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
