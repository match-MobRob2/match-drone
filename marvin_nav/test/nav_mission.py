#!/usr/bin/env python3
"""Autonomie-Test in der Sim: Ziele ueber den Planer anfliegen (von Hand, nicht CI).

Takeoff per Offboard-Setpoint, dann Uebergabe an Planer + pursuit: Ziele gehen
auf /goal_pose (map-Frame), angekommen = TF map->base_link < `tol` am Ziel.
Gegen Gazebo-Ground-Truth (Bridge /gt_poses wie bei drift_flight) wird der
minimale Abstand zu Hallenstrukturen gemessen (Mesh-Punkte auf Flughoehe +-1 m).

    python3 -s nav_mission.py <mesh.obj> "x,y;x,y;..." [csv_out] [box_x,box_y]

Optional wird nach dem Takeoff ein Block (1.5 x 1.5 x 4 m) an box_x,box_y (Welt)
gespawnt — nicht in der gespeicherten Karte, muss live erkannt werden.
"""
import subprocess
import csv
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from tf2_msgs.msg import TFMessage
import tf2_ros

ALT = 1.5
BOX = 1.5  # Kantenlaenge des Test-Hindernisses [m]


def spawn_box(x, y, z_floor):
    sdf = (f"<sdf version='1.9'><model name='hindernis'><static>true</static><link name='l'>"
           f"<collision name='c'><geometry><box><size>{BOX} {BOX} 4</size></box></geometry></collision>"
           f"<visual name='v'><geometry><box><size>{BOX} {BOX} 4</size></box></geometry>"
           f"<material><diffuse>1 0.3 0 1</diffuse></material></visual></link></model></sdf>")
    subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'scale3', '-string', sdf,
                    '-x', str(x), '-y', str(y), '-z', str(z_floor + 2.0)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def mesh_tree(path, zmin, zmax):
    """Mesh-Oberflaeche im Hoehenband als KD-Baum (Dreiecke mit ~0.1 m abgetastet)."""
    v, pts = [], []
    for line in open(path):
        if line.startswith('v '):
            v.append([float(x) for x in line.split()[1:4]])
    v = np.array(v)
    for line in open(path):
        if not line.startswith('f '):
            continue
        idx = [int(t.split('/')[0]) - 1 for t in line.split()[1:]]
        for k in range(1, len(idx) - 1):
            a, b, c = v[idx[0]], v[idx[k]], v[idx[k + 1]]
            if max(a[2], b[2], c[2]) < zmin or min(a[2], b[2], c[2]) > zmax:
                continue
            n = int(0.5 * np.linalg.norm(np.cross(b - a, c - a)) / 0.01) + 1
            r1, r2 = np.random.rand(n, 1), np.random.rand(n, 1)
            m = (r1 + r2) > 1
            r1[m], r2[m] = 1 - r1[m], 1 - r2[m]
            p = a + r1 * (b - a) + r2 * (c - a)
            pts.append(p[(p[:, 2] >= zmin) & (p[:, 2] <= zmax)])
    return cKDTree(np.vstack(pts))


class Mission(Node):
    def __init__(self):
        super().__init__('nav_mission', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.px4 = self.gt = None
        self.path_seen = False
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 lambda m: setattr(self, 'px4', m.pose), qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/gt_poses',
                                 lambda m: setattr(self, 'gt', m.transforms[0].transform), qos_profile_sensor_data)
        self.local_z = math.nan  # z des lokalen Pfads ~3 Wegpunkte voraus (Diagnose Reiseflughoehe)
        self.create_subscription(Path, '/local_planned_path', self.local_cb, 10)
        self.sp_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode = self.create_client(SetMode, '/mavros/set_mode')
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.sp = None
        self.create_timer(0.05, self.stream)
        self.log = []

    def local_cb(self, m):
        self.path_seen = self.path_seen or len(m.poses) > 0
        if m.poses:
            self.local_z = m.poses[min(3, len(m.poses) - 1)].pose.position.z

    def stream(self):
        # Eigene Setpoints nur bis der Planer uebernimmt (pursuit streamt dann)
        if self.sp is None or self.path_seen:
            return
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.sp[:3]
        m.pose.orientation.z, m.pose.orientation.w = math.sin(self.sp[3] / 2), math.cos(self.sp[3] / 2)
        self.sp_pub.publish(m)

    def pose_map(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception:  # noqa: B902
            return None

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.gt is not None:
                g = self.gt.translation
                pm = self.pose_map()
                try:
                    c = self.tf.lookup_transform('map', 'camera_init', rclpy.time.Time()).transform.translation
                    corr = (c.x, c.y)
                except Exception:  # noqa: B902
                    corr = (math.nan, math.nan)
                self.log.append((self.get_clock().now().nanoseconds * 1e-9, g.x, g.y, g.z,
                                 *(pm if pm is not None else (math.nan,) * 3), *corr, self.local_z))

    def call(self, client, req):
        client.wait_for_service(timeout_sec=10)
        fut = client.call_async(req)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result()

    def run(self, goals, tol=0.4, timeout=180.0, box=None):
        while self.px4 is None or self.gt is None or self.pose_map() is None:
            self.spin_for(0.5)
        p = self.px4.position
        self.sp = (p.x, p.y, p.z, yaw_of(self.px4.orientation))
        self.spin_for(2.0)
        self.call(self.mode, SetMode.Request(custom_mode='OFFBOARD'))
        for _ in range(20):
            if self.call(self.arm, CommandBool.Request(value=True)).success:
                break
            self.spin_for(3.0)
        self.sp = (p.x, p.y, ALT, self.sp[3])
        self.spin_for(10.0)
        if box:
            # Boden ~0.25 m unter base_link beim Start (gelandete Drohne)
            spawn_box(box[0], box[1], self.log[0][3] - 0.25)
            self.get_logger().info(f'Hindernis bei {box} gespawnt')
            self.spin_for(2.0)
        results = []
        for gx, gy in goals:
            g = PoseStamped()
            g.header.frame_id = 'map'
            g.pose.position.x, g.pose.position.y, g.pose.position.z = gx, gy, ALT
            g.pose.orientation.w = 1.0
            # Setpoint-Stream weiterfuehren, bis pursuit den ersten Pfad hat
            p = self.px4.position
            self.sp = (p.x, p.y, p.z, yaw_of(self.px4.orientation))
            self.goal_pub.publish(g)
            t0 = time.time()
            t0_sim = self.get_clock().now().nanoseconds * 1e-9
            ok = False
            while time.time() - t0 < timeout:
                self.spin_for(0.5)
                pm = self.pose_map()
                if pm is not None and math.hypot(pm[0] - gx, pm[1] - gy) < tol:
                    ok = True
                    break
            dt = time.time() - t0
            results.append((gx, gy, ok, dt, t0_sim, self.get_clock().now().nanoseconds * 1e-9))
            self.get_logger().info(f'Ziel ({gx}, {gy}): {"erreicht" if ok else "TIMEOUT"} nach {dt:.0f} s')
            self.spin_for(3.0)
        self.call(self.mode, SetMode.Request(custom_mode='AUTO.LAND'))
        self.spin_for(10.0)
        return results


def quality(log, results):
    """Flugqualitaet je Ziel aus der Ground Truth (20 Hz resampelt, geglaettet)."""
    r = np.array([row[:4] for row in log])
    _, keep = np.unique(r[:, 0], return_index=True)
    r = r[keep]
    t = np.arange(r[0, 0], r[-1, 0], 0.05)
    p = np.stack([np.interp(t, r[:, 0], r[:, k]) for k in (1, 2, 3)], 1)
    k = np.ones(5) / 5  # 0.25 s Gleitmittel gegen Stufen im GT-Takt
    p = np.stack([np.convolve(p[:, i], k, mode='same') for i in range(3)], 1)
    v = np.gradient(p, t, axis=0)
    a = np.gradient(v, t, axis=0)
    j = np.gradient(a, t, axis=0)
    sp, an, jn = (np.linalg.norm(x, axis=1) for x in (v, a, j))
    print(f'{"Ziel":>12} {"Zeit":>6} {"Weg":>6} {"Umweg":>6} {"v mit/max":>10} {"a p95":>6} {"Jerk p95":>8} {"Stopps":>6}')
    for gx, gy, ok, dt, ts, te in results:
        m = (t >= ts) & (t <= te)
        if m.sum() < 10:
            continue
        seg = p[m]
        length = np.linalg.norm(np.diff(seg, axis=0), axis=1).sum()
        direct = max(np.linalg.norm(seg[-1, :2] - seg[0, :2]), 0.1)
        slow = sp[m] < 0.1
        # Stopps: zusammenhaengende Stillstaende > 1 s, nicht am Anfang/Ende
        stops, run = 0, 0
        for s_ in slow[20:-20]:
            run = run + 1 if s_ else 0
            stops += run == 20
        print(f'({gx:4.0f},{gy:4.0f}) {"" if ok else "X"}{te - ts:5.0f}s {length:5.1f}m {length / direct:5.2f}x '
              f'{sp[m].mean():4.2f}/{sp[m].max():4.2f} {np.percentile(an[m], 95):6.2f} {np.percentile(jn[m], 95):8.1f} {stops:6d}')


def main():
    mesh, route = sys.argv[1], sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else 'nav_mission.csv'
    box = tuple(map(float, sys.argv[4].split(','))) if len(sys.argv) > 4 else None
    goals = [tuple(map(float, wp.split(','))) for wp in route.split(';')]
    rclpy.init()
    n = Mission()
    try:
        results = n.run(goals, box=box, timeout=float(os.environ.get('NAV_TIMEOUT', '180')))
    finally:
        with open(out, 'w', newline='') as f:
            csv.writer(f).writerows([('t', 'gt_x', 'gt_y', 'gt_z', 'map_x', 'map_y', 'map_z', 'corr_x', 'corr_y',
                                      'local_path_z')]
                                    + n.log)
    traj = np.array([r[1:4] for r in n.log])
    m = np.array([r[4:6] for r in n.log])
    ok = ~np.isnan(m).any(1)
    # map-Ursprung = Sensor beim Mapping-Start = Welt (0,0) + Lidar-Hebelarm 0.1 m in x
    err = np.hypot(*(m[ok] + [0.1, 0.0] - traj[ok, :2]).T)
    print(f'Planer-Pose vs. Ground Truth (xy): Median {np.median(err):.2f} m, max {err.max():.2f} m')
    fly = traj[traj[:, 2] > traj[0, 2] + 1.0]  # Flugphase (Start/Landung zaehlen nicht)
    # Hoehenband aus der geflogenen Hoehe; Boden ausklammern (liegt in Gazebo nicht
    # bei z=0 — Startpunkt + 0.3 m)
    tree = mesh_tree(mesh, max(fly[:, 2].min() - 0.5, traj[0, 2] + 0.3), fly[:, 2].max() + 0.5)
    d, _ = tree.query(fly)
    if box:
        # horizontaler Abstand zur Block-Aussenflaeche
        dx = np.maximum(np.abs(fly[:, 0] - box[0]) - BOX / 2, 0)
        dy = np.maximum(np.abs(fly[:, 1] - box[1]) - BOX / 2, 0)
        print(f'min. Abstand zum Test-Hindernis {np.hypot(dx, dy).min():.2f} m')
    quality(n.log, results)
    print(f'{sum(r[2] for r in results)}/{len(results)} Ziele erreicht, '
          f'min. Abstand zu Strukturen {d.min():.2f} m (Median {np.median(d):.2f} m), '
          f'max. Hoehe {traj[:, 2].max() - traj[0, 2]:.1f} m ueber Start')


if __name__ == '__main__':
    main()
