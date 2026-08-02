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


class SprayControllerNode(Node):

    def __init__(self):
        super().__init__("spray_controller_node")

        transient_local_qos = QoSProfile(depth=1)
        transient_local_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._spray_on = False
        self._publisher = self.create_publisher(Bool, "spray_on", transient_local_qos)
        self.create_service(SetBool, "~/set_spray_on", self._on_set_spray_on)

        self._publish_state()
        self.get_logger().info("spray_controller_node ready, spray_on = false")

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
