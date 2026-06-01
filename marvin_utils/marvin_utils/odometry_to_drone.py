#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


# --- Quaternion-Helfer (ROS-Konvention: x, y, z, w) ---

def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def q_rotate(q, v):
    """Rotiert Vektor v mit Quaternion q (v' = q * v * q^-1)."""
    qv = (v[0], v[1], v[2], 0.0)
    x, y, z, _ = q_mul(q_mul(q, qv), q_conj(q))
    return (x, y, z)


def q_from_rpy(roll, pitch, yaw):
    """q = Rz(yaw) * Ry(pitch) * Rx(roll), Achsen einzeln zusammengesetzt."""
    qx = (math.sin(roll / 2.0), 0.0, 0.0, math.cos(roll / 2.0))
    qy = (0.0, math.sin(pitch / 2.0), 0.0, math.cos(pitch / 2.0))
    qz = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    return q_mul(q_mul(qz, qy), qx)


class FastlioToPx4(Node):
    def __init__(self):
        super().__init__('fastlio_to_px4')

        # Topics / Frames
        self.declare_parameter('input_topic', '/Odometry')
        self.declare_parameter('output_topic', '/mavros/odometry/out')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        # Montage-Rotation der IMU (= FAST-LIO 'body') gegenueber base_link.
        # Beim MID360-fat_marvin sitzt die IMU 61 deg (1.0647 rad) um Y gekippt.
        # FAST-LIO setzt beim Init rot=Identitaet -> 'camera_init' erbt diese
        # Kippung. Diese eine Rotation levelt beides zugleich:
        #   q_out = q_mount * q_fastlio * q_mount^-1   (Orientierung)
        #   p_out = R(q_mount) * p_fastlio             (Position)
        # Voraussetzung: Drohne steht beim FAST-LIO-Init level.
        self.declare_parameter('mount_roll', 0.0)
        self.declare_parameter('mount_pitch', 1.0647)
        self.declare_parameter('mount_yaw', 0.0)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        roll = self.get_parameter('mount_roll').value
        pitch = self.get_parameter('mount_pitch').value
        yaw = self.get_parameter('mount_yaw').value
        self.q_mount = q_from_rpy(roll, pitch, yaw)
        self.q_mount_conj = q_conj(self.q_mount)

        self.odom_sub = self.create_subscription(Odometry, in_topic, self.callback, 10)
        # /mavros/odometry/out -> EKF2 fusioniert das als Vision-Odometrie (ENU)
        self.odom_pub = self.create_publisher(Odometry, out_topic, 10)

        self.counter = 0
        self.get_logger().info(
            f'Bridge gestartet: {in_topic} -> {out_topic} '
            f'(frame {self.frame_id} -> {self.child_frame_id}), '
            f'mount RPY=[{roll:.4f}, {pitch:.4f}, {yaw:.4f}] rad'
        )

    def callback(self, msg):
        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id          # gelevelter Welt-Frame (ENU)
        out.child_frame_id = self.child_frame_id     # base_link

        # --- Orientierung: Aehnlichkeitstransform um die Montage-Rotation ---
        o = msg.pose.pose.orientation
        q_msg = (o.x, o.y, o.z, o.w)
        n = math.sqrt(sum(c * c for c in q_msg)) or 1.0
        q_msg = tuple(c / n for c in q_msg)
        qx, qy, qz, qw = q_mul(q_mul(self.q_mount, q_msg), self.q_mount_conj)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        # --- Position: Vektor in den gelevelten Frame drehen ---
        p = msg.pose.pose.position
        px, py, pz = q_rotate(self.q_mount, (p.x, p.y, p.z))
        # Hinweis: Hebelarm IMU->base_link (~0.25 m) wird vernachlaessigt; bei
        # Bedarf p_out += R(q_mount*q_msg) * t_imu_to_base ergaenzen.
        out.pose.pose.position.x = px
        out.pose.pose.position.y = py
        out.pose.pose.position.z = pz

        # Pose-Covariance (Default ~10 cm / wenige Grad)
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = 0.01     # x, y, z
        cov[21] = cov[28] = 0.01             # roll, pitch
        cov[35] = 0.02                       # yaw
        out.pose.covariance = cov

        # --- Twist: FAST-LIO liefert KEINE Velocity (publish_odometry setzt
        #     twist nie). Als unbekannt markieren, damit EKF2 sie nicht
        #     faelschlich als "0" fusioniert. ---
        big = 1e6
        twist_cov = [0.0] * 36
        twist_cov[0] = twist_cov[7] = twist_cov[14] = big
        twist_cov[21] = twist_cov[28] = twist_cov[35] = big
        out.twist.covariance = twist_cov

        self.odom_pub.publish(out)

        self.counter += 1
        if self.counter % 20 == 0:
            self.get_logger().info(
                f'pos=({px:.2f}, {py:.2f}, {pz:.2f})  (geleveltes {self.frame_id})'
            )


def main():
    rclpy.init()
    rclpy.spin(FastlioToPx4())


if __name__ == '__main__':
    main()
