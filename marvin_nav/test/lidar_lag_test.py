#!/usr/bin/env python3
"""Misst den Zeitversatz Lidar-Scan <-> IMU in der Sim (von Hand, nicht CI).

Die Drohne dreht auf der Stelle (Sim laeuft, fake_gcs wie bei drift_flight).
Jeder Scan wird per 2D-ICP (gelevelte Punkte, nur Yaw) gegen einen
Referenzscan aus dem Stillstand ausgerichtet. Referenz ist der integrierte
Gyro der Lidar-IMU — genau die Zeitbasis, gegen die FAST-LIO fusioniert.
Gesucht wird tau mit scan_yaw(t) = imu_yaw(t - tau):
tau > 0 = der Scan zeigt die Welt von FRUEHER als sein Stempel sagt.
Setzt FAST-LIO ggf. als time_offset_lidar_to_imu = -tau.

Aufruf: python3 -s lidar_lag_test.py [mount_pitch] [imu_topic]
(-s: System-NumPy, scipy passt nicht zur NumPy 2 in ~/.local)
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

PITCH = float(sys.argv[1]) if len(sys.argv) > 1 else 0.52
IMU_TOPIC = sys.argv[2] if len(sys.argv) > 2 else '/rgl_lidar/imu'


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def level_slice(msg):
    p = np.array([[a, b, c] for a, b, c in point_cloud2.read_points(
        msg, field_names=('x', 'y', 'z'), skip_nans=True)])
    c, s = math.cos(PITCH), math.sin(PITCH)
    p = p @ np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]).T  # Lidar -> gelevelt
    r = np.hypot(p[:, 0], p[:, 1])
    m = (p[:, 2] > 0.3) & (p[:, 2] < 3.0) & (r > 1.0) & (r < 25.0)  # Waende, kein Boden
    return p[m, :2]


def icp_yaw(src, tree, ref, yaw0, iters=30):
    """Nur Rotation um den Ursprung (Drohne dreht auf der Stelle)."""
    yaw = yaw0
    for _ in range(iters):
        c, s = math.cos(yaw), math.sin(yaw)
        q = src @ np.array([[c, -s], [s, c]]).T
        d, idx = tree.query(q, distance_upper_bound=1.0)
        ok = np.isfinite(d)
        if ok.sum() < 50:
            return None
        a, b = q[ok], ref[idx[ok]]
        # optimale Zusatzrotation (2D-Kabsch um den Ursprung)
        dyaw = math.atan2(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]), np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
        yaw += dyaw
        if abs(dyaw) < 1e-5:
            break
    return yaw


class LagTest(Node):
    def __init__(self):
        super().__init__('lidar_lag_test', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.imu = []     # (t, wx, wy, wz) Lidar-IMU
        self.lio = []     # (t, qx, qy, qz, qw) FAST-LIO body in camera_init
        self.scans = []   # (t, msg)
        self.px4 = None
        self.record = False
        self.create_subscription(Imu, IMU_TOPIC, self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/Odometry', self.lio_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/rgl_lidar', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 lambda m: setattr(self, 'px4', m.pose), qos_profile_sensor_data)
        self.sp_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode = self.create_client(SetMode, '/mavros/set_mode')
        self.sp = None
        self.create_timer(0.05, self.stream)

    def lio_cb(self, msg):
        if self.record:
            q = msg.pose.pose.orientation
            self.lio.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, q.x, q.y, q.z, q.w))

    def imu_cb(self, msg):
        if self.record:
            w = msg.angular_velocity
            self.imu.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, w.x, w.y, w.z))

    def scan_cb(self, msg):
        if self.record:
            self.scans.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg))

    def stream(self):
        if self.sp is None:
            return
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.sp[:3]
        m.pose.orientation.z, m.pose.orientation.w = math.sin(self.sp[3] / 2), math.cos(self.sp[3] / 2)
        self.sp_pub.publish(m)

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, client, req):
        client.wait_for_service(timeout_sec=10)
        fut = client.call_async(req)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result()

    def run(self):
        while self.px4 is None:
            self.spin_for(0.5)
        p = self.px4.position
        x0, y0, yaw0 = p.x, p.y, yaw_of(self.px4.orientation)
        self.sp = (x0, y0, p.z, yaw0)
        self.spin_for(2.0)
        self.call(self.mode, SetMode.Request(custom_mode='OFFBOARD'))
        for _ in range(20):
            if self.call(self.arm, CommandBool.Request(value=True)).success:
                break
            self.spin_for(3.0)
        self.sp = (x0, y0, 1.5, yaw0)
        self.spin_for(12.0)
        self.record = True
        self.spin_for(3.0)                       # Referenz im Stillstand
        for k in range(1, 9):                    # 2 Umdrehungen in 90°-Schritten
            self.sp = (x0, y0, 1.5, yaw0 + k * math.pi / 2)
            self.spin_for(3.0)
        self.record = False
        self.call(self.mode, SetMode.Request(custom_mode='AUTO.LAND'))
        self.spin_for(3.0)


def main():
    rclpy.init()
    n = LagTest()
    n.run()
    imu = np.array(n.imu)
    imu = imu[np.argsort(imu[:, 0])]
    # Drehrate um die Hochachse: Gyro der gekippten IMU levelt wie die Punkte
    c, s = math.cos(PITCH), math.sin(PITCH)
    wz = (np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ imu[:, 1:4].T)[2]
    t_imu = imu[:, 0]
    yaw_imu = np.concatenate([[0.0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * np.diff(t_imu))])

    t_ref = n.scans[0][0]
    ref = level_slice(n.scans[0][1])
    tree = cKDTree(ref)
    y0 = np.interp(t_ref, t_imu, yaw_imu)
    ts, ys = [], []
    for t, msg in n.scans[1:]:
        guess = np.interp(t, t_imu, yaw_imu) - y0
        y = icp_yaw(level_slice(msg), tree, ref, guess)
        if y is not None:
            ts.append(t)
            ys.append(guess + wrap(y - guess))  # ICP-Ergebnis nahe Schaetzung auspacken
    ts, ys = np.array(ts), np.array(ys)
    np.savez('lidar_lag_raw.npz', t_imu=t_imu, yaw_imu=yaw_imu, t_scan=ts, yaw_scan=ys)

    rate = np.interp(ts, t_imu, np.gradient(yaw_imu, t_imu))
    moving = np.abs(rate) > 0.3
    taus = np.arange(-0.2, 0.2001, 0.001)
    cost = [np.mean((ys - (np.interp(ts - tau, t_imu, yaw_imu) - y0)) ** 2) for tau in taus]
    tau = taus[int(np.argmin(cost))]
    res0 = ys - (np.interp(ts, t_imu, yaw_imu) - y0)
    res = ys - (np.interp(ts - tau, t_imu, yaw_imu) - y0)
    print(f'{len(ts)} Scans, {moving.sum()} in Drehung, Drehrate max {np.degrees(np.abs(rate).max()):.0f}°/s')
    print(f'Gesamtdrehung: IMU {np.degrees(np.interp(ts[-1], t_imu, yaw_imu) - y0):.1f}°, '
          f'Scan {np.degrees(ys[-1]):.1f}°  (Differenz = Gyro-Skala/Drift)')
    print(f'ohne Versatz: Yaw-Residuum in Drehung {np.degrees(np.sqrt(np.mean(res0[moving] ** 2))):.2f}° RMS, '
          f'im Stand {np.degrees(np.sqrt(np.mean(res0[~moving] ** 2))):.2f}° RMS')
    # FAST-LIO: body-Orientierung gelevelt (map = Ry(p) camera_init, base = body Ry(p)^T)
    if n.lio:
        L = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        lt, lyaw = [], []
        for t, qx, qy, qz, qw in n.lio:
            R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                          [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                          [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
            B = L @ R @ L.T
            lt.append(t)
            lyaw.append(math.atan2(B[1, 0], B[0, 0]))
        lt, lyaw = np.array(lt), np.unwrap(np.array(lyaw))
        lyaw -= np.interp(t_ref, lt, lyaw)
        li = np.interp(lt, t_imu, yaw_imu) - y0
        m = lt >= t_ref
        print(f'FAST-LIO: Gesamtdrehung {np.degrees(lyaw[m][-1]):.1f}° (IMU {np.degrees(li[m][-1]):.1f}°), '
              f'max. Nachlauf in Drehung {np.degrees(np.max(np.abs(lyaw[m] - li[m]))):.1f}°')
    print(f'=> bester Versatz tau = {tau * 1000:+.0f} ms, Residuum in Drehung dann '
          f'{np.degrees(np.sqrt(np.mean(res[moving] ** 2))):.2f}° RMS')


if __name__ == '__main__':
    main()
