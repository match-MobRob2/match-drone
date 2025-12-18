#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class TimestampRewriter(Node):
    def __init__(self):
        super().__init__('imu_timemachine')

        # Parameter
        self.declare_parameter('topic_in', '/mavros/imu/data')
        self.declare_parameter('topic_out', '/mavros/imu/data_sim')
        self.declare_parameter('msg_type', 'sensor_msgs/msg/Imu')
        # WICHTIG: use_sim_time NICHT deklarieren, das macht ROS selbst!

        topic_in = self.get_parameter('topic_in').value
        topic_out = self.get_parameter('topic_out').value
        msg_type_str = self.get_parameter('msg_type').value

        # Message-Typ dynamisch laden
        pkg, msg = msg_type_str.split('/msg/')
        self.msg_type = __import__(pkg + ".msg", fromlist=[msg]).__dict__[msg]

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.sub = self.create_subscription(
            self.msg_type,
            topic_in,
            self.callback,
            sensor_qos
        )

        self.pub = self.create_publisher(
            self.msg_type,
            topic_out,
            sensor_qos
        )

        self.get_logger().info(f"Rewriting timestamps: {topic_in} → {topic_out}")

    def callback(self, msg):
        # Node-Clock verwenden → respektiert use_sim_time und /clock
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TimestampRewriter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
