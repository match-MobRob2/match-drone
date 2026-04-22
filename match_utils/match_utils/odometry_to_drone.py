#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class FastlioToPx4(Node):
    def __init__(self):
        super().__init__('fastlio_to_px4')
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_highrate', self.callback, 10)
        
        # WICHTIG: Nutze /mavros/odometry/out für korrektes ENU Frame!
        self.odom_pub = self.create_publisher(
            Odometry,
            '/mavros/odometry/out',
            10
        )
        
        self.counter = 0
        self.get_logger().info('Bridge started - publishing from /odom_highrate to /mavros/odometry/out (ENU frame)')
    
    def callback(self, msg):
        out = Odometry()
        
        # Header - WICHTIG für Frame-Identifikation
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "odom"      # ENU world frame
        out.child_frame_id = "base_link"  # Body frame
        
        # Position und Orientation kopieren
        out.pose.pose = msg.pose.pose
        
        # OPTIONAL: Z-Achse invertieren falls FastLIO NED nutzt
        # Teste erst ohne Inversion!
        # out.pose.pose.position.z = -msg.pose.pose.position.z
        
        # Covariance für Pose
        if np.any(msg.pose.covariance) and False:
            out.pose.covariance = list(msg.pose.covariance)
        else:
            # Default Covariance
            cov = [0.01] * 36
            cov[0] = 0.01    # x variance (10cm std dev)
            cov[7] = 0.01    # y variance
            cov[14] = 0.01   # z variance
            cov[21] = 0.01   # roll variance
            cov[28] = 0.01   # pitch variance
            cov[35] = 0.02   # yaw variance
            out.pose.covariance = cov
        
        # Twist (Velocity) - wichtig für bessere EKF2 Performance
        out.twist.twist = msg.twist.twist
        
        # Velocity Covariance
        if np.any(msg.twist.covariance) and False:
            out.twist.covariance = list(msg.twist.covariance)
        else:
            vel_cov = [0.01] * 36
            vel_cov[0] = 0.01   # vx
            vel_cov[7] = 0.01   # vy
            vel_cov[14] = 0.01  # vz
            out.twist.covariance = vel_cov
        
        self.odom_pub.publish(out)
        
        # Logging
        self.counter += 1
        if self.counter % 1 == 0:
            self.get_logger().info(
                f'FastLIO: x={msg.pose.pose.position.x:.3f}, '
                f'y={msg.pose.pose.position.y:.3f}, '
                f'z={msg.pose.pose.position.z:.3f}'
            )

def main():
    rclpy.init()
    rclpy.spin(FastlioToPx4())

if __name__ == '__main__':
    main()