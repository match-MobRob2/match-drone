# imu_propagation_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation

class ImuPropagationNode(Node):
    def __init__(self):
        super().__init__('imu_propagation_node')
        
        # Letzter bekannter Zustand von FastLIO
        self.last_odom = None
        self.last_odom_time = None
        
        # IMU Bias (aus FastLIO State übernehmen falls verfügbar)
        self.accel_bias = np.zeros(3)
        self.gyro_bias  = np.zeros(3)
        
        # Aktueller propagierter Zustand
        self.pos   = np.zeros(3)
        self.vel   = np.zeros(3)
        self.rot   = Rotation.identity()
        
        self.last_imu_time = None
        self.initialized   = False

        # QoS passend zu FastLIO (BEST_EFFORT)
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions
        self.create_subscription(Odometry, '/Odometry', self.odom_cb, best_effort_qos)
        self.create_subscription(Imu, '/mavros/imu/data_sim', self.imu_cb, 100)

        # Output-Timer @ 50Hz
        self.pub = self.create_publisher(Odometry, '/odom_highrate', 10)
        self.create_timer(1.0 / 50.0, self.publish_cb)

        self.get_logger().info("ImuPropagationNode initialized. Waiting for first odometry message...")

    def odom_cb(self, msg: Odometry):
        """FastLIO-Update → Zustand übernehmen und Reset der Integration"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear

        self.pos = np.array([p.x, p.y, p.z])
        self.vel = np.array([v.x, v.y, v.z])
        self.rot = Rotation.from_quat([q.x, q.y, q.z, q.w])

        self.last_odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_imu_time  = self.last_odom_time
        was_initialized = self.initialized
        self.initialized    = True

        if not was_initialized:
            self.get_logger().debug(
                f"First odometry received — node is now initialized. "
                f"pos=[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}], "
                f"vel=[{v.x:.3f}, {v.y:.3f}, {v.z:.3f}]"
            )
        else:
            self.get_logger().debug(
                f"Odometry update (FastLIO reset): "
                f"pos=[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}], "
                f"vel=[{v.x:.3f}, {v.y:.3f}, {v.z:.3f}], "
                f"quat=[{q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f}]"
            )

    def imu_cb(self, msg: Imu):
        """IMU-Daten integrieren"""
        if not self.initialized:
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu_time is None:
            self.last_imu_time = t
            return
        
        dt = t - self.last_imu_time
        if dt <= 0 or dt > 0.1:   # Sanity-Check
            self.get_logger().debug(
                f"IMU sample skipped: dt={dt:.6f}s (must be in (0, 0.1]). Resetting IMU time."
            )
            self.last_imu_time = t
            return

        # Gyro-Integration (Bias-korrigiert)
        omega = np.array([
            msg.angular_velocity.x - self.gyro_bias[0],
            msg.angular_velocity.y - self.gyro_bias[1],
            msg.angular_velocity.z - self.gyro_bias[2],
        ])
        delta_rot = Rotation.from_rotvec(omega * dt)
        self.rot  = self.rot * delta_rot

        # Accel-Integration im World-Frame (Bias + Gravity korrigiert)
        gravity   = np.array([0, 0, 9.81])
        accel_body = np.array([
            msg.linear_acceleration.x - self.accel_bias[0],
            msg.linear_acceleration.y - self.accel_bias[1],
            msg.linear_acceleration.z - self.accel_bias[2],
        ])
        accel_world = self.rot.apply(accel_body) - gravity
        
        self.pos += self.vel * dt + 0.5 * accel_world * dt**2
        self.vel += accel_world * dt

        self.get_logger().debug(
            f"IMU propagation: dt={dt:.4f}s | "
            f"omega=[{omega[0]:.4f}, {omega[1]:.4f}, {omega[2]:.4f}] rad/s | "
            f"accel_body=[{accel_body[0]:.4f}, {accel_body[1]:.4f}, {accel_body[2]:.4f}] m/s² | "
            f"accel_world=[{accel_world[0]:.4f}, {accel_world[1]:.4f}, {accel_world[2]:.4f}] m/s² | "
            f"pos=[{self.pos[0]:.3f}, {self.pos[1]:.3f}, {self.pos[2]:.3f}] | "
            f"vel=[{self.vel[0]:.3f}, {self.vel[1]:.3f}, {self.vel[2]:.3f}]"
        )

        self.last_imu_time = t

    def publish_cb(self):
        if not self.initialized:
            return

        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_init'
        msg.child_frame_id  = 'body'

        msg.pose.pose.position.x = self.pos[0]
        msg.pose.pose.position.y = self.pos[1]
        msg.pose.pose.position.z = self.pos[2]

        q = self.rot.as_quat()
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        msg.twist.twist.linear.x = self.vel[0]
        msg.twist.twist.linear.y = self.vel[1]
        msg.twist.twist.linear.z = self.vel[2]

        self.get_logger().info(f"Publishing propagated odometry: pos={self.pos}, vel={self.vel}")

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuPropagationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()