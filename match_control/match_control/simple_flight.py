import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped

YELLOW = "\033[93m"
RED = "\033[91m"
GREEN = "\033[92m"
RESET = "\033[0m"

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
            self.get_logger().info(f"{YELLOW}'Sending initial setpoints...{RESET}")
            self.counter += 1
            if self.counter > 30: 
                self.arm()

        elif self.state == 1:
            self.counter += 1
            if self.counter > 20:
                self.set_offboard()

        elif self.state == 2:
            self.counter += 1
            if self.counter > 20:
                self.get_logger().info(f"{YELLOW}Reaching takeoff altitude...{RESET}")
                self.state = 3
                self.counter = 0

        elif self.state == 3:
            self.counter += 1
            if self.counter > 100:  
                self.get_logger().info(f"{YELLOW}Flying forward...{RESET}")
                self.target_pose.pose.position.x = 2.0
                self.state = 4
                self.counter = 0

        elif self.state == 4:
            self.counter += 1
            if self.counter > 100:  
                self.land()

        elif self.state == 5:
            self.counter += 1
            if self.counter > 50:
                self.disarm()
                self.state = 6

        elif self.state == 6:
            self.get_logger().info(f"{GREEN}Mission complete{RESET}")
            rclpy.shutdown()

    def arm(self):
        if self.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            future.add_done_callback(self.arm_response_callback)
        else:
            self.get_logger().warn(f"{YELLOW}Arming service not ready yet{RESET}")
            self.counter = 0  # optional: zurücksetzen

    def arm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{GREEN}Armed successfully{RESET}")
                self.state = 1
                self.counter = 0
            else:
                self.get_logger().warn(f"{RED}Arming failed, retrying...{RESET}")
                self.state = 0  # Wieder zurück zu vorherigem Zustand
                self.counter = 0
        except Exception as e:
            self.get_logger().error(f"{RED}Arming service call failed: {e}{RESET}")
            self.state = 0
            self.counter = 0

    def set_offboard(self):
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(self.set_mode_response_callback)
        else:
            self.get_logger().warn(f"{YELLOW}SetMode service not ready yet{RESET}")

    def set_mode_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f"{GREEN}OFFBOARD mode set successfully{RESET}")
                self.state = 2
                self.counter = 0
            else:
                self.get_logger().warn(f"{RED}Failed to set OFFBOARD mode, retrying...{RESET}")
                self.state = 1
                self.counter = 0
        except Exception as e:
            self.get_logger().error(f"{RED}SetMode service call failed: {e}{RESET}")
            self.state = 1
            self.counter = 0

    def land(self):
        if self.land_client.service_is_ready():
            req = CommandTOL.Request()
            req.altitude = 0.0
            future = self.land_client.call_async(req)
            future.add_done_callback(self.land_response_callback)
        else:
            self.get_logger().warn(f"{YELLOW}Land service not ready yet{RESET}")

    def land_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{GREEN}Landing initiated successfully{RESET}")
                self.state = 5  # Proceed to disarming
                self.counter = 0
            else:
                self.get_logger().warn(f"{RED}Landing failed, retrying...{RESET}")
                self.state = 4
                self.counter = 0
        except Exception as e:
            self.get_logger().error(f"{RED}Landing service call failed: {e}{RESET}")
            self.state = 4
            self.counter = 0

    def disarm(self):
        if self.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = False
            future = self.arming_client.call_async(req)
            future.add_done_callback(self.disarm_response_callback)
        else:
            self.get_logger().warn(f"{YELLOW}Disarming service not ready yet{RESET}")

    def disarm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{GREEN}Disarmed successfully{RESET}")
                self.state = 6  # Next state after disarming
            else:
                self.get_logger().warn(f"{RED}Disarming failed, retrying...{RESET}")
                self.state = 5
                self.counter = 0
        except Exception as e:
            self.get_logger().error(f"{RED}Disarming service call failed: {e}{RESET}")
            self.state = 5
            self.counter = 0


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDroneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
