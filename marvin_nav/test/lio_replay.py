#!/usr/bin/env python3
"""FAST-LIO offline gegen eine Rosbag testen (von Hand, nicht CI).

Startet fastlio_mapping mit der Sim-Config plus Overrides, spielt die Bag
(/rgl_lidar, /rgl_lidar/imu, /clock) ab und vergleicht FAST-LIOs Drehung um
die Hochachse mit dem integrierten Gyro aus der Bag. Kein Gazebo noetig —
jedes Experiment sieht exakt dieselben Daten.

    python3 -s lio_replay.py <bag> [mapping.max_iteration:=10 ...]
"""
import math
import os
import subprocess
import sys
import time

import numpy as np
from ament_index_python.packages import get_package_prefix
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import Imu

PITCH = 0.52
CFG = os.path.join(os.path.dirname(__file__), '..', '..', 'marvin_launch', 'config', 'fastlio_sim.yaml')


def rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def bag_imu_yaw(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/rgl_lidar/imu']))
    t, w = [], []
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, Imu)
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        w.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    t, w = np.array(t), np.array(w)
    c, s = math.cos(PITCH), math.sin(PITCH)
    wz = (np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ w.T)[2]
    return t, np.concatenate([[0.0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * np.diff(t))])


def main():
    bag, overrides = sys.argv[1], sys.argv[2:]
    t_imu, yaw_imu = bag_imu_yaw(bag)

    rclpy.init()
    n = Node('lio_replay', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    odo = []
    # (Bag-Uhr beim Empfang, Scan-Stempel): Bag-Uhr (--clock) laeuft mit der Wiedergabe,
    # dadurch faellt deren Tempo heraus; konstanter Versatz Bag-Uhr/Stempel wird unten abgezogen
    lat = []
    n.create_subscription(Odometry, '/Odometry', lambda m: lat.append(
        (n.get_clock().now().nanoseconds * 1e-9, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)),
        qos_profile_sensor_data)
    n.create_subscription(Odometry, '/Odometry', lambda m: odo.append(
        (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.orientation,
         (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z))), qos_profile_sensor_data)

    # Binary direkt (nicht 'ros2 run'): sonst trifft kill() nur den Wrapper
    # und alte FAST-LIO-Instanzen publizieren im naechsten Lauf weiter
    exe = os.path.join(get_package_prefix('fast_lio'), 'lib', 'fast_lio', 'fastlio_' + 'mapping')
    args = [exe, '--ros-args', '--params-file', CFG,
            '-p', 'use_sim_time:=true', '-p', 'pcd_save.pcd_save_en:=false']
    for o in ' '.join(overrides).split():
        args += ['-p', o]
    lio_log = open('lio_replay_fastlio.log', 'w')
    lio = subprocess.Popen(args, stdout=lio_log, stderr=subprocess.STDOUT)
    time.sleep(3.0)
    # Nur EINE Uhr: --clock aus den Bag-Stempeln; das aufgezeichnete /clock nicht
    # zusaetzlich abspielen (bei ohne --use-sim-time aufgenommenen Bags widerspraechen
    # sich die beiden -> FAST-LIO-Timer springen)
    t_play = time.time()
    play = subprocess.Popen(['ros2', 'bag', 'play', bag, '--clock',
                             '--topics', '/rgl_lidar', '/rgl_lidar/imu'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    while play.poll() is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    end = time.time() + 2.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    lio.kill()
    lio.wait()
    play.wait()
    lio_log.close()
    for line in open('lio_replay_fastlio.log'):
        if 'Rechenzeit' in line:
            print('   ', line.strip().split(']: ')[-1])

    if not odo:
        print('keine /Odometry empfangen')
        return
    L = np.array([[math.cos(PITCH), 0, math.sin(PITCH)], [0, 1, 0], [-math.sin(PITCH), 0, math.cos(PITCH)]])
    lt = np.array([t for t, _, _ in odo])
    pos = np.array([p for _, _, p in odo]) @ L.T  # gelevelt
    div = np.linalg.norm(pos, axis=1).max()
    la = np.array(lat)
    la = la[la[:, 0] > 1e3]  # vor dem ersten /clock steht die Node-Uhr auf 0
    rel = la[:, 0] - la[:, 1]
    rel -= np.percentile(rel, 1)  # konstanter Versatz raus: relativ zum schnellsten Scan
    np.savez('lio_replay_last.npz', t=lt, pos=pos)  # Verlauf zur Nachanalyse
    lyaw = np.unwrap([math.atan2(*(lambda B: (B[1, 0], B[0, 0]))(L @ rot((q.x, q.y, q.z, q.w)) @ L.T))
                      for _, q, _ in odo])
    li = np.interp(lt, t_imu, yaw_imu)
    err = (lyaw - lyaw[0]) - (li - li[0])
    print(f'{" ".join(overrides) or "(Basis)":45}  Drehung FAST-LIO {math.degrees(lyaw[-1] - lyaw[0]):6.1f}° '
          f'/ IMU {math.degrees(li[-1] - li[0]):6.1f}°  Endfehler {math.degrees(err[-1]):+6.1f}°  '
          f'max Nachlauf {math.degrees(np.max(np.abs(err))):5.1f}°  |  z [{pos[:, 2].min():.1f}, {pos[:, 2].max():.1f}] m'
          f'{"  DIVERGIERT (max |p| %.0f m)" % div if div > 100 else ""}'
          f'  |  Latenz Median {np.median(rel) * 1000:.0f} p95 {np.percentile(rel, 95) * 1000:.0f} '
          f'max {rel.max() * 1000:.0f} ms, '
          f'{len(odo)} Scans verarbeitet')


if __name__ == '__main__':
    main()
