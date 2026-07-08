#!/usr/bin/env python3
"""Smoke-Test für lidar_relocalization_node (manuell ausführen, kein CI-Test).

Fall 1 (PCD-Modus): synthetische Raum-Punktwolke als PCD-Karte, Scan um
bekannten Versatz verschoben (= Odom-Drift) — die publizierte Korrektur
map->odom muss dem Versatz entsprechen.

Fall 2 (Echtzeit-Modus, ohne map_pcd): Referenz entsteht online aus
Keyframes — erst undriftete Scans (Keyframe wird eingefroren), dann derselbe
Raum mit Versatz; die Korrektur muss den Drift wiederfinden.

Aufruf (Workspace gesourced, Node vorher NICHT starten — macht das Skript):
    python3 src/marvin_nav/test/reloc_smoke.py
"""
import math
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

# Bekannter Versatz map->odom (Translation + Yaw)
OFFSET = np.array([0.5, 0.3, 0.2])
YAW = 0.1


def room_points():
    """Punkte auf den Wänden eines 10x8x3-Raums."""
    rng = np.random.default_rng(42)
    pts = []
    for _ in range(4000):
        wall = rng.integers(0, 5)
        u, v = rng.uniform(0, 1, 2)
        if wall == 0:
            pts.append([u * 10, 0, v * 3])
        elif wall == 1:
            pts.append([u * 10, 8, v * 3])
        elif wall == 2:
            pts.append([0, u * 8, v * 3])
        elif wall == 3:
            pts.append([10, u * 8, v * 3])
        else:
            pts.append([u * 10, v * 8, 0])
    return np.array(pts)


def write_pcd(path, pts):
    with open(path, 'w') as f:
        f.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
                'TYPE F F F\nCOUNT 1 1 1\n'
                f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                f'POINTS {len(pts)}\nDATA ascii\n')
        for p in pts:
            f.write(f'{p[0]} {p[1]} {p[2]}\n')


def drifted(pts):
    """Karte um OFFSET/YAW 'zurück' verschoben (Punkte in Odom-Koordinaten)."""
    c, s = math.cos(YAW), math.sin(YAW)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return (pts - OFFSET) @ R  # inverse Transformation: R^T @ (p - t)


def run_case(name, node_args, phases, timeout=30.0):
    """phases: Liste (scan_pts, dauer_s); die letzte Phase läuft, bis die
    Korrektur dem Versatz entspricht oder timeout erreicht ist."""
    proc = subprocess.Popen(
        ['ros2', 'run', 'marvin_nav', 'lidar_relocalization_node', '--ros-args',
         '-p', 'match_period:=1.0', '-p', 'voxel_size:=0.2'] + node_args)

    rclpy.init()
    node = Node('reloc_smoke')
    last = {}

    def on_pose(msg: PoseStamped):
        last['pos'] = np.array([msg.pose.position.x, msg.pose.position.y,
                                msg.pose.position.z])
        q = msg.pose.orientation
        last['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))

    node.create_subscription(PoseStamped, '/relocalization/pose', on_pose, 10)
    scan_pub = node.create_publisher(point_cloud2.PointCloud2, '/cloud_registered', 5)
    # Odom nur fürs Keyframe-Gate im Echtzeit-Modus; Position 0 reicht
    odom_pub = node.create_publisher(Odometry, '/Odometry', 5)

    deadline = time.time() + timeout
    phase_idx = 0
    phase_end = time.time() + phases[0][1]
    while time.time() < deadline:
        if phase_idx < len(phases) - 1 and time.time() >= phase_end:
            phase_idx += 1
            phase_end = time.time() + phases[phase_idx][1]
        header = Header(frame_id='camera_init')
        header.stamp = node.get_clock().now().to_msg()
        scan_pub.publish(point_cloud2.create_cloud_xyz32(
            header, phases[phase_idx][0].tolist()))
        odom = Odometry()
        odom.header = header
        odom_pub.publish(odom)
        rclpy.spin_once(node, timeout_sec=0.5)
        if phase_idx == len(phases) - 1 and 'pos' in last:
            if (np.linalg.norm(last['pos'] - OFFSET) < 0.1
                    and abs(last['yaw'] - YAW) < 0.05):
                break

    proc.terminate()
    proc.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()

    assert 'pos' in last, f'{name}: keine Korrektur empfangen'
    err_t = np.linalg.norm(last['pos'] - OFFSET)
    err_yaw = abs(last['yaw'] - YAW)
    print(f"{name}: Korrektur {last['pos']}, Yaw {last['yaw']:.3f} "
          f"(erwartet {OFFSET}, {YAW}) — Fehler: {err_t:.3f} m, {err_yaw:.3f} rad")
    assert err_t < 0.1, f'{name}: Translationsfehler zu gross: {err_t:.3f} m'
    assert err_yaw < 0.05, f'{name}: Yaw-Fehler zu gross: {err_yaw:.3f} rad'


def main():
    map_pts = room_points()
    pcd = tempfile.NamedTemporaryFile(suffix='.pcd', delete=False)
    write_pcd(pcd.name, map_pts)

    run_case('PCD-Modus', ['-p', f'map_pcd:={pcd.name}'],
             [(drifted(map_pts), 0.0)])
    run_case('Echtzeit-Modus', [],
             [(map_pts, 4.0), (drifted(map_pts), 0.0)])
    print('RELOC SMOKE TEST OK')


if __name__ == '__main__':
    sys.exit(main())
