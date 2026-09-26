#!/usr/bin/env python3
"""Schreibt waehrend des Mapping-Flugs Keyframes fuer die Pose-Graph-Optimierung.

Je Keyframe (alle `dist` m oder `yaw` rad): Scan im FAST-LIO-body-Frame als
<out_dir>/kf_NNNNN.pcd und eine Zeile in <out_dir>/poses.csv mit der ROHEN
FAST-LIO-Pose (camera_init, unkorrigiert — die Korrektur macht der
map_optimizer). Wird sofort geschrieben: ein Absturz verliert nichts.
"""
import math
import os

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def write_pcd(path, pts):
    pts = np.ascontiguousarray(pts, dtype=np.float32)
    header = (f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(pts.tobytes())


def voxel_down(pts, leaf):
    keys = np.floor(pts / leaf).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


class KeyframeRecorder(Node):
    def __init__(self):
        super().__init__('keyframe_recorder')
        self.out = os.path.expanduser(self.declare_parameter('out_dir', '').value)
        self.dist = self.declare_parameter('dist', 1.0).value
        self.yaw = self.declare_parameter('yaw', 0.35).value      # ~20°
        self.leaf = self.declare_parameter('voxel', 0.1).value    # Speicher/Platte sparen
        # Montage-Pitch der FAST-LIO-IMU: der map_optimizer levelt damit die Karte
        mount_pitch = self.declare_parameter('mount_pitch', 0.0).value
        if not self.out:
            raise RuntimeError('out_dir muss gesetzt sein')
        os.makedirs(self.out, exist_ok=True)
        with open(os.path.join(self.out, 'mount_pitch'), 'w') as f:
            f.write(f'{mount_pitch}\n')
        self.poses = open(os.path.join(self.out, 'poses.csv'), 'w', buffering=1)
        self.poses.write('id,stamp,x,y,z,qx,qy,qz,qw\n')
        self.n = 0
        self.last = None  # (pos, R) des letzten Keyframes
        subs = [Subscriber(self, Odometry, '/Odometry', qos_profile=qos_profile_sensor_data),
                Subscriber(self, PointCloud2, '/cloud_registered_body', qos_profile=qos_profile_sensor_data)]
        self.sync = ApproximateTimeSynchronizer(subs, 10, 0.02)
        self.sync.registerCallback(self.cb)
        self.get_logger().info(f'Keyframes -> {self.out} (alle {self.dist} m / {math.degrees(self.yaw):.0f}°)')

    def cb(self, odom, cloud):
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        pos = np.array([p.x, p.y, p.z])
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        if self.last is not None:
            dpos = np.linalg.norm(pos - self.last[0])
            # Drehwinkel zwischen den Keyframes (body ist gekippt -> kein reiner Yaw)
            dang = math.acos(max(-1.0, min(1.0, (np.trace(self.last[1].T @ R) - 1) / 2)))
            if dpos < self.dist and dang < self.yaw:
                return
        pts = np.array([[a, b, c] for a, b, c in point_cloud2.read_points(
            cloud, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=np.float32)
        if len(pts) < 100:
            return
        write_pcd(os.path.join(self.out, f'kf_{self.n:05d}.pcd'), voxel_down(pts, self.leaf))
        stamp = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        self.poses.write(f'{self.n},{stamp:.6f},{p.x},{p.y},{p.z},{x},{y},{z},{w}\n')
        self.last = (pos, R)
        self.n += 1
        if self.n % 20 == 0:
            self.get_logger().info(f'{self.n} Keyframes')


def main():
    rclpy.init()
    rclpy.spin(KeyframeRecorder())


if __name__ == '__main__':
    main()
