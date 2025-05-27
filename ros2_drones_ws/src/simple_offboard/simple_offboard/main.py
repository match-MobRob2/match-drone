import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from rclpy.qos import QoSProfile
import time

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        qos = QoSProfile(depth=10)

        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.counter = 0
        self.state = 0

    def arm(self):
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
        self.get_logger().info("Arming...")

    def set_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # custom
        msg.param2 = 6.0  # offboard
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
        self.get_logger().info("Switching to OFFBOARD mode...")

    def send_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def send_position(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        self.setpoint_pub.publish(msg)

    def timer_callback(self):
        self.send_offboard_control_mode()

        if self.counter == 10:
            self.set_offboard_mode()
        elif self.counter == 15:
            self.arm()
        elif 20 <= self.counter < 70:
            self.send_position(0.0, 5.0, -5.0)  # vorwärts
        elif 70 <= self.counter < 120:
            self.send_position(0.0, -5.0, -5.0)  # rückwärts
        elif self.counter == 130:
            self.get_logger().info("Mission complete. Disarming.")
            self.disarm()
        elif self.counter > 140:
            rclpy.shutdown()

        self.counter += 1

    def disarm(self):
        msg = VehicleCommand()
        msg.param1 = 0.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
