"""Demo node: take off, move 2 m forward and land."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class TakeoffForwardLandNode(Node):
    """Take off, fly forward and land."""

    def __init__(self) -> None:
        super().__init__('takeoff_forward_land_node')
        self.local_pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        self.state = 0
        self.counter = 0
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.z = 2.0
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self) -> None:
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.local_pos_pub.publish(self.target_pose)
        if self.state == 0:
            self.counter += 1
            if self.counter > 30:
                self.arm(True)
                self.state = 1
                self.counter = 0
        elif self.state == 1:
            self.counter += 1
            if self.counter > 20:
                self.set_offboard()
                self.state = 2
                self.counter = 0
        elif self.state == 2:
            self.counter += 1
            if self.counter > 100:
                self.target_pose.pose.position.x = 2.0
                self.state = 3
                self.counter = 0
        elif self.state == 3:
            self.counter += 1
            if self.counter > 100:
                self.land()
                self.state = 4
                self.counter = 0
        elif self.state == 4:
            self.counter += 1
            if self.counter > 50:
                self.arm(False)
                self.state = 5
        elif self.state == 5:
            self.get_logger().info('Mission complete.')
            rclpy.shutdown()

    def arm(self, value: bool) -> None:
        req = CommandBool.Request()
        req.value = value
        self.arming_client.call_async(req)

    def set_offboard(self) -> None:
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        self.set_mode_client.call_async(req)

    def land(self) -> None:
        req = CommandTOL.Request()
        req.altitude = 0.0
        self.land_client.call_async(req)


def main() -> None:
    rclpy.init()
    node = TakeoffForwardLandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()
