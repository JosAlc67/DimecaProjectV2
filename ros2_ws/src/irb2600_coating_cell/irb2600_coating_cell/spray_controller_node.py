"""Logical control subsystem for the spray_on signal (report Sec. VII-F,
eq. 6-7 and Table XVI). Implements only the logical state itself; deciding
*when* to turn it on/off during a trajectory is the trajectory-execution
script's job (Task 5: it should call set_spray_on(true) at the start of the
coating pass and set_spray_on(false) at the end, or immediately on a
replanning/collision abort per report Sec. VI-B steps 7-9).

    ss = 1 if spray_on == true else 0        (eq. 6)
    ss == 1  =>  coating in progress          (eq. 7)

Interface:
    ~/set_spray_on (std_srvs/srv/SetBool)  - request the signal on/off
    spray_on        (std_msgs/msg/Bool)     - latched-style state publication
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros

from irb2600_coating_cell.geometry_utils import rotate_vector_by_quaternion
from irb2600_coating_cell.resource_utils import load_shared_cell_config


class SprayControllerNode(Node):

    def __init__(self):
        super().__init__("spray_controller_node")

        try:
            shared_config = load_shared_cell_config()
            default_standoff = float(shared_config.get("trajectory", {}).get("d_standoff", 0.15))
        except (OSError, ValueError, LookupError):
            default_standoff = 0.15
        self.declare_parameter("trajectory.d_standoff", default_standoff)

        transient_local_qos = QoSProfile(depth=1)
        transient_local_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._spray_on = False
        self._publisher = self.create_publisher(Bool, "spray_on", transient_local_qos)
        self.create_service(SetBool, "~/set_spray_on", self._on_set_spray_on)

        # Paint visualization
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._paint_pub = self.create_publisher(Marker, "~/paint_splatters", 10)
        self._paint_marker = Marker()
        self._paint_marker.header.frame_id = "world"
        self._paint_marker.type = Marker.SPHERE_LIST
        self._paint_marker.action = Marker.ADD
        self._paint_marker.scale.x = 0.08  # 8cm paint spray
        self._paint_marker.scale.y = 0.08
        self._paint_marker.scale.z = 0.08
        self._paint_marker.color.r = 0.0
        self._paint_marker.color.g = 0.5
        self._paint_marker.color.b = 1.0
        self._paint_marker.color.a = 0.8
        self._paint_marker.pose.orientation.w = 1.0

        # Run timer at 10Hz to sample paint
        self.create_timer(0.1, self._paint_tick)

        self._publish_state()
        self.get_logger().info("spray_controller_node ready, spray_on = false")

    def _paint_tick(self):
        if not self._spray_on:
            return

        try:
            # Lookup where the nozzle tip is right now
            trans = self._tf_buffer.lookup_transform(
                "world",
                "nozzle_tip",
                rclpy.time.Time()
            )
            
            p = Point()
            # nozzle_tip +Z is the tool approach vector. Project the configured
            # standoff along that orientation instead of assuming world +X.
            standoff = float(self.get_parameter("trajectory.d_standoff").value)
            direction = rotate_vector_by_quaternion(
                (0.0, 0.0, standoff), trans.transform.rotation
            )
            p.x = trans.transform.translation.x + direction[0]
            p.y = trans.transform.translation.y + direction[1]
            p.z = trans.transform.translation.z + direction[2]
            
            self._paint_marker.points.append(p)
            self._paint_marker.header.stamp = self.get_clock().now().to_msg()
            self._paint_pub.publish(self._paint_marker)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # TF not ready yet
            pass

    def _publish_state(self):
        msg = Bool()
        msg.data = self._spray_on
        self._publisher.publish(msg)

    def _on_set_spray_on(self, request, response):
        previous = self._spray_on
        self._spray_on = bool(request.data)
        self._publish_state()

        if self._spray_on:
            response.message = "spray_on = true: coating in progress (eq. 7)."
        else:
            response.message = "spray_on = false: coating trajectory cancelled/paused."

        response.success = True
        if previous != self._spray_on:
            self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SprayControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
