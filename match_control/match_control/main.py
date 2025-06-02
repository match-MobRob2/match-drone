import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped

class SimpleDroneNode(Node):
    def __init__(self):
        super().__init__('simple_drone_node')

        # Publisher
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        # Interner Status
        self.state = 0
        self.counter = 0

        # Zielposition
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 2.0

        # Loop-Timer 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.local_pos_pub.publish(self.target_pose)

        if self.state == 0:
            self.get_logger().info('Sending initial setpoints...')
            self.counter += 1
            if self.counter > 30: 
                self.arm()
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
            if self.counter > 20:
                self.get_logger().info('Reaching takeoff altitude...')
                self.state = 3
                self.counter = 0

        elif self.state == 3:
            self.counter += 1
            if self.counter > 100:  
                self.get_logger().info('Flying forward...')
                self.target_pose.pose.position.x = 2.0
                self.state = 4
                self.counter = 0

        elif self.state == 4:
            self.counter += 1
            if self.counter > 100:  
                self.land()
                self.state = 5
                self.counter = 0

        elif self.state == 5:
            self.counter += 1
            if self.counter > 50:
                self.disarm()
                self.state = 6

        elif self.state == 6:
            self.get_logger().info('Mission complete.')
            rclpy.shutdown()

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info('Arming result: {}'.format(f.result().success)))

    def disarm(self):
        req = CommandBool.Request()
        req.value = False
        future = self.arming_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info('Disarming result: {}'.format(f.result().success)))

    def set_offboard(self):
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info('SetMode result: {}'.format(f.result().mode_sent)))

    def land(self):
        req = CommandTOL.Request()
        req.altitude = 0.0
        future = self.land_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info('Land result: {}'.format(f.result().success)))


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDroneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
