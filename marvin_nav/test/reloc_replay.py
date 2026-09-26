#!/usr/bin/env python3
"""Relokalisierung offline gegen eine Bag testen (von Hand, nicht CI).

Die Bag (ros2 bag record --use-sim-time ... /cloud_registered /Odometry
/gt_poses /clock) liefert FAST-LIOs Ausgaben und Ground Truth. Hier laeuft nur
lidar_relocalization_node mit einstellbaren Parametern; bewertet wird die
Pose, mit der Planer + pursuit fliegen (map -> base_link), gegen Gazebo.

    python3 -s reloc_replay.py <bag> <map.pcd> [param:=wert ...]

Annahmen (Sim): Karten-Ursprung = Mapping-Start bei Welt (0,0), Yaw 0;
Lidar-Montage 0.52 rad / (0.1, 0, 0.07) wie in nav_fastlio.
"""
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
import rosbag2_py
from tf2_msgs.msg import TFMessage
import tf2_ros

PITCH = 0.52
LIDAR_T = np.array([0.10, 0.0, 0.07])


def rot(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def bag_gt(bag):
    """Ground Truth mit Bag-Zeitstempeln (= Sim-Empfangszeit bei --use-sim-time)."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/gt_poses']))
    t, p = [], []
    while r.has_next():
        _, data, stamp = r.read_next()
        tr = deserialize_message(data, TFMessage).transforms[0].transform
        q = tr.rotation
        t.append(stamp * 1e-9)
        p.append((tr.translation.x, tr.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                                  1 - 2 * (q.y * q.y + q.z * q.z))))
    return np.array(t), np.array(p)


def main():
    bag, map_pcd, overrides = sys.argv[1], sys.argv[2], sys.argv[3:]
    t_gt, gt = bag_gt(bag)

    rclpy.init()
    n = Node('reloc_replay', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    est = []  # (t, x, y, yaw) von base_link in map

    R_body_base = Ry(PITCH).T
    t_body_base = -R_body_base @ LIDAR_T

    def odom_cb(m):
        try:
            c = buf.lookup_transform('map', 'camera_init', rclpy.time.Time()).transform
        except Exception:  # noqa: B902
            return
        Rc = rot(c.rotation.x, c.rotation.y, c.rotation.z, c.rotation.w)
        tc = np.array([c.translation.x, c.translation.y, c.translation.z])
        q, p = m.pose.pose.orientation, m.pose.pose.position
        Rb = Rc @ rot(q.x, q.y, q.z, q.w)
        pb = Rc @ np.array([p.x, p.y, p.z]) + tc
        pos = pb + Rb @ t_body_base
        Rbase = Rb @ R_body_base
        est.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, pos[0], pos[1],
                    math.atan2(Rbase[1, 0], Rbase[0, 0])))

    n.create_subscription(Odometry, '/Odometry', odom_cb, qos_profile_sensor_data)

    exe = os.path.join(get_package_prefix('marvin_nav'), 'lib', 'marvin_nav', 'lidar_relocalization_node')
    args = [exe, '--ros-args', '-p', 'use_sim_time:=true', '-p', f'map_pcd:={map_pcd}',
            '-p', f'initial_pitch:={PITCH}']
    for o in ' '.join(overrides).split():
        args += ['-p', o]
    reloc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3.0)
    play = subprocess.Popen(['ros2', 'bag', 'play', bag, '--clock', '--topics', '/cloud_registered',
                             '/Odometry', '/clock'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    while play.poll() is None:
        rclpy.spin_once(n, timeout_sec=0.05)
    reloc.kill()
    reloc.wait()

    if not est:
        print('keine Posen (TF map->camera_init fehlt?)')
        return
    e = np.array(est)
    # Welt <- map: Mapping-Start (Welt 0,0, Yaw 0) + Lidar-Hebelarm
    gx = np.interp(e[:, 0], t_gt, gt[:, 0])
    gy = np.interp(e[:, 0], t_gt, gt[:, 1])
    gyaw = np.arctan2(np.interp(e[:, 0], t_gt, np.sin(gt[:, 2])), np.interp(e[:, 0], t_gt, np.cos(gt[:, 2])))
    exy = np.hypot(e[:, 1] + LIDAR_T[0] - gx, e[:, 2] - gy)  # map-Ursprung liegt um den Hebelarm vor Welt-0
    eyaw = np.degrees(np.abs((e[:, 3] - gyaw + np.pi) % (2 * np.pi) - np.pi))
    print(f'{" ".join(overrides) or "(Basis)":55} xy Median {np.median(exy):.2f} p90 {np.percentile(exy, 90):.2f} '
          f'max {exy.max():.2f} m | Yaw Median {np.median(eyaw):.1f} max {eyaw.max():.1f}°')


if __name__ == '__main__':
    main()
