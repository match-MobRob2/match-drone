import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu

class ImuFrameFix(Node):
    def __init__(self):
        super().__init__('imu_frame_fix')
        # params
        self.declare_parameter('target_frame', 'lidar_link')
        self.declare_parameter('in_topic', '/livox/imu')
        self.declare_parameter('out_topic', '/livox/imu_fixed')
        target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        in_topic     = self.get_parameter('in_topic').get_parameter_value().string_value
        out_topic    = self.get_parameter('out_topic').get_parameter_value().string_value

        # SensorData QoS (matches bridges/sensors, avoids dropped msgs)
        qos = QoSProfile(depth=100)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST
        # self.pub = self.create_publisher(Imu, out_topic, qos)
        # self.sub = self.create_subscription(Imu, in_topic, self.cb, qos)

        pub_qos = QoSProfile(depth=10);  pub_qos.reliability  = QoSReliabilityPolicy.RELIABLE
        sub_qos = QoSProfile(depth=100); sub_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(Imu, out_topic, pub_qos)
        self.sub = self.create_subscription(Imu, in_topic, self.cb, sub_qos)

        self.target_frame = target_frame
        self.get_logger().info(f"IMU frame fix: {in_topic} -> {out_topic}, frame_id={target_frame}")

    def cb(self, msg: Imu):
        msg.header.frame_id = self.target_frame  # clean frame
        # (optional) ensure covariances are not all zeros if your consumer dislikes that:
        # if all(v == 0.0 for v in msg.angular_velocity_covariance):
        #     msg.angular_velocity_covariance[0] = 1e-4
        # if all(v == 0.0 for v in msg.linear_acceleration_covariance):
        #     msg.linear_acceleration_covariance[0] = 1e-3
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ImuFrameFix())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
