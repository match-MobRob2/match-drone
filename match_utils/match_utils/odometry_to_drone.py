#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3, Twist, Pose
import numpy as np
import math

def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions (x, y, z, w).
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ], dtype=np.float64)

def quaternion_inverse(q):
    """
    Inverse of a quaternion (x, y, z, w).
    """
    x, y, z, w = q
    norm_sq = x*x + y*y + z*z + w*w
    return np.array([-x, -y, -z, w], dtype=np.float64) / norm_sq

def quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion (x, y, z, w) to a 3x3 rotation matrix.
    """
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,         2*x*z + 2*y*w],
        [2*x*y + 2*z*w,         1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [2*x*z - 2*y*w,         2*y*z + 2*x*w,         1 - 2*x*x - 2*y*y]
    ])

class OdometryToDrone(Node):
    def __init__(self):
        super().__init__('odometry_to_drone')
        
        # Parameters
        self.declare_parameter('enu_to_ned', True)
        self.declare_parameter('world_frame', 'camera_init')
        self.declare_parameter('body_frame', 'body')
        self.declare_parameter('max_dt', 0.2)
        
        self.enu_to_ned = self.get_parameter('enu_to_ned').value
        self.world_frame = self.get_parameter('world_frame').value
        self.body_frame = self.get_parameter('body_frame').value
        self.max_dt = self.get_parameter('max_dt').value
        
        # State
        self.prev_odom = None
        self.prev_time = None
        
        # Transformation matrices
        # ENU: East(X), North(Y), Up(Z)
        # NED: North(X), East(Y), Down(Z)
        # T_enu2ned transforms a vector in ENU to NED
        self.T_enu2ned = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])
        
        # Body Frame transformation: FLU to FRD
        # FLU: Front(X), Left(Y), Up(Z)
        # FRD: Front(X), Right(Y), Down(Z)
        self.T_flu2frd = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        
        # Publishers and Subscribers
        self.sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        self.pub = self.create_publisher(Odometry, '/mavros/odometry/out', 10)
        
        self.get_logger().info(f"Odometry to Drone node started. ENU->NED: {self.enu_to_ned}")

    def odom_callback(self, msg: Odometry):
        current_time = self.get_clock().now()
        
        if self.prev_odom is None:
            self.prev_odom = msg
            self.prev_time = current_time
            return

        # Calculate time difference
        dt_duration = current_time - self.prev_time
        dt = dt_duration.nanoseconds / 1e9
        
        # Simple health check for dt
        if dt <= 0 or dt > self.max_dt:
            self.prev_odom = msg
            self.prev_time = current_time
            return

        # Extract Pose
        pos_enu = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        q_enu = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        prev_pos_enu = np.array([
            self.prev_odom.pose.pose.position.x,
            self.prev_odom.pose.pose.position.y,
            self.prev_odom.pose.pose.position.z
        ])
        
        prev_q_enu = np.array([
            self.prev_odom.pose.pose.orientation.x,
            self.prev_odom.pose.pose.orientation.y,
            self.prev_odom.pose.pose.orientation.z,
            self.prev_odom.pose.pose.orientation.w
        ])

        # 1. Coordinate Transform (Global)
        if self.enu_to_ned:
            # Position
            pos_out = self.T_enu2ned @ pos_enu
            
            # Orientation: R_ned = T_enu2ned * R_enu * T_flu2frd.T
            R_enu = quaternion_to_rotation_matrix(q_enu)
            R_out = self.T_enu2ned @ R_enu @ self.T_flu2frd.T # Because T_flu2frd is its own transpose/inverse here but better to be explicit
            
            # Convert R_out back to quaternion
            q_out = self.matrix_to_quaternion(R_out)
            
            # Linear Velocity (World frame first)
            vel_world_enu = (pos_enu - prev_pos_enu) / dt
            vel_world_out = self.T_enu2ned @ vel_world_enu
            
            # Angular Velocity
            # Delta q in body frame: dq = q_prev^-1 * q_curr
            dq_body_flu = quaternion_multiply(quaternion_inverse(prev_q_enu), q_enu)
            # This dq represents rotation in FLU. We need it in FRD.
            # R_dq_frd = T_flu2frd * R_dq_flu * T_flu2frd^T
            omega_flu = self.q_to_omega(dq_body_flu, dt)
            omega_out = self.T_flu2frd @ omega_flu
            
            # Velocity Body Frame (FRD)
            # vel_body = R_out^T * vel_world_out
            vel_body_out = R_out.T @ vel_world_out
            
        else:
            # Keep as ENU
            pos_out = pos_enu
            q_out = q_enu
            
            vel_world_out = (pos_enu - prev_pos_enu) / dt
            R_enu = quaternion_to_rotation_matrix(q_enu)
            vel_body_out = R_enu.T @ vel_world_out
            
            dq_body = quaternion_multiply(quaternion_inverse(prev_q_enu), q_enu)
            omega_out = self.q_to_omega(dq_body, dt)

        # Create Output Message
        out_msg = Odometry()
        out_msg.header = msg.header
        out_msg.header.frame_id = self.world_frame
        out_msg.child_frame_id = self.body_frame
        
        # Set Pose
        out_msg.pose.pose.position.x = pos_out[0]
        out_msg.pose.pose.position.y = pos_out[1]
        out_msg.pose.pose.position.z = pos_out[2]
        out_msg.pose.pose.orientation.x = q_out[0]
        out_msg.pose.pose.orientation.y = q_out[1]
        out_msg.pose.pose.orientation.z = q_out[2]
        out_msg.pose.pose.orientation.w = q_out[3]
        
        # Set Twist (should be in child_frame_id, i.e., body frame)
        out_msg.twist.twist.linear.x = vel_body_out[0]
        out_msg.twist.twist.linear.y = vel_body_out[1]
        out_msg.twist.twist.linear.z = vel_body_out[2]
        out_msg.twist.twist.angular.x = omega_out[0]
        out_msg.twist.twist.angular.y = omega_out[1]
        out_msg.twist.twist.angular.z = omega_out[2]
        
        # Set Covariances
        # Realistic values (small but non-zero)
        # Position covariance
        pose_cov = [0.0] * 36
        for i in range(3): pose_cov[i*7] = 0.05 # x, y, z
        for i in range(3, 6): pose_cov[i*7] = 0.01 # orientation
        out_msg.pose.covariance = pose_cov
        
        # Velocity covariance
        twist_cov = [0.0] * 36
        for i in range(3): twist_cov[i*7] = 0.02 # lin vel
        for i in range(3, 6): twist_cov[i*7] = 0.01 # ang vel
        out_msg.twist.covariance = twist_cov
        
        self.pub.publish(out_msg)
        
        # Update state
        self.prev_odom = msg
        self.prev_time = current_time

    def q_to_omega(self, dq, dt):
        """
        Convert small delta quaternion to angular velocity.
        dq = [x, y, z, w]
        """
        # Ensure w is positive for the shortest path
        if dq[3] < 0:
            dq = -dq
            
        angle = 2.0 * math.acos(max(-1.0, min(1.0, dq[3])))
        if abs(angle) < 1e-8:
            return np.array([0.0, 0.0, 0.0])
        
        norm = math.sqrt(dq[0]**2 + dq[1]**2 + dq[2]**2)
        if norm < 1e-8:
            return np.array([0.0, 0.0, 0.0])
            
        axis = np.array([dq[0], dq[1], dq[2]]) / norm
        return (axis * angle) / dt

    def matrix_to_quaternion(self, R):
        """
        Convert 3x3 rotation matrix to quaternion (x, y, z, w).
        Uses Shepperd's algorithm or similar for stability.
        """
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
            
        return np.array([qx, qy, qz, qw])

def main(args=None):
    rclpy.init(args=args)
    node = OdometryToDrone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
