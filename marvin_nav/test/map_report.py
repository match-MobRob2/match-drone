#!/usr/bin/env python3
"""Kartenguete einer Sim-Aufnahme gegen Gazebo-Ground-Truth (von Hand, nicht CI).

    python3 map_report.py <map_dir> <flight.csv> [mesh.obj]

- Keyframe-Posen roh (FAST-LIO, gelevelt) vs. optimiert (poses_opt.csv) gegen
  die Ground Truth aus dem drift_flight-CSV (Stempel = Sim-Zeit)
- optional: Abstand map.pcd -> Hallen-Mesh (Punkte auf Dreiecken abgetastet)

Frames: map = Ry(pitch) * camera_init, camera_init = body beim Start. Daraus
world <- map = GT-Pose von base_link beim ersten Keyframe, verschoben um den
Lidar-Hebelarm t (in base_link) — Rotation kuerzt sich heraus.
"""
import csv
import math
import sys

import numpy as np

LIDAR_T = np.array([0.10, 0.0, 0.07])  # Lidar in base_link (Sim, model.sdf)


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_csv(path):
    return list(csv.DictReader(open(path)))


def main(map_dir, flight_csv, mesh=None):
    pitch = float(open(f'{map_dir}/keyframes/mount_pitch').read())
    raw = load_csv(f'{map_dir}/keyframes/poses.csv')
    opt = load_csv(f'{map_dir}/poses_opt.csv')
    fl = load_csv(flight_csv)
    ft = np.array([float(r['t']) for r in fl])
    gt = {k: np.array([float(r[f'gt_{k}']) for r in fl]) for k in ('x', 'y', 'z', 'yaw')}

    def gt_at(t):
        p = np.array([np.interp(t, ft, gt[k]) for k in 'xyz'])
        # Yaw ueber sin/cos interpolieren (Sprung bei ±180°)
        yaw = math.atan2(np.interp(t, ft, np.sin(gt['yaw'])), np.interp(t, ft, np.cos(gt['yaw'])))
        return p, yaw

    L = Ry(pitch)
    R_body_base = Ry(pitch).T               # base_link in body
    t_body_base = -R_body_base @ LIDAR_T

    # world <- map aus dem ersten Keyframe
    t0 = float(raw[0]['stamp'])
    g0, gy0 = gt_at(t0)
    Rw = np.array([[math.cos(gy0), -math.sin(gy0), 0], [math.sin(gy0), math.cos(gy0), 0], [0, 0, 1]])
    tw = g0 + Rw @ LIDAR_T

    def err(pos_map, R_map_body, t):
        # base_link in map -> world, gegen GT
        pb = pos_map + R_map_body @ t_body_base
        Rb = R_map_body @ R_body_base
        pw = Rw @ pb + tw
        g, gy = gt_at(t)
        return np.hypot(*(pw - g)[:2]), (pw - g)[2], math.degrees(wrap(yaw_of(Rw @ Rb) - gy))

    rows = []
    for r, o in zip(raw, opt):
        t = float(r['stamp'])
        if t > ft[-1]:
            break
        q = [float(r[k]) for k in ('qx', 'qy', 'qz', 'qw')]
        pr = L @ np.array([float(r[k]) for k in 'xyz'])
        Rr = L @ quat_to_R(*q)
        po = np.array([float(o[k]) for k in 'xyz'])
        Ro = quat_to_R(*[float(o[k]) for k in ('qx', 'qy', 'qz', 'qw')])
        rows.append(err(pr, Rr, t) + err(po, Ro, t))
    e = np.array(rows)
    print(f'{len(e)} Keyframes mit Ground Truth')
    for name, off in (('roh (FAST-LIO)', 0), ('optimiert', 3)):
        xy, z, yw = np.abs(e[:, off]), np.abs(e[:, off + 1]), np.abs(e[:, off + 2])
        print(f'  {name:15} xy mittel {xy.mean():.3f} max {xy.max():.3f} m | '
              f'z mittel {z.mean():.3f} max {z.max():.3f} m | yaw mittel {yw.mean():.2f} max {yw.max():.2f}°')

    if mesh:
        mesh_to_map(map_dir, mesh, Rw, tw)


def mesh_to_map(map_dir, mesh, Rw, tw):
    from scipy.spatial import cKDTree
    v, faces = [], []
    for line in open(mesh):
        if line.startswith('v '):
            v.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):
            faces.append([int(t.split('/')[0]) for t in line.split()[1:]])
    v = np.array(v)
    # Dreiecke gleichmaessig abtasten (~5 cm)
    samples = []
    for f in faces:
        idx = [i - 1 if i > 0 else len(v) + i for i in f]
        for k in range(1, len(idx) - 1):
            a, b, c = v[idx[0]], v[idx[k]], v[idx[k + 1]]
            area = 0.5 * np.linalg.norm(np.cross(b - a, c - a))
            n = int(area / 0.0025) + 1
            r1, r2 = np.random.rand(n, 1), np.random.rand(n, 1)
            m = (r1 + r2) > 1
            r1[m], r2[m] = 1 - r1[m], 1 - r2[m]
            samples.append(a + r1 * (b - a) + r2 * (c - a))
    tree = cKDTree(np.vstack(samples))
    pts = read_pcd(f'{map_dir}/map.pcd') @ Rw.T + tw
    d, _ = tree.query(pts)
    print(f'map.pcd -> Mesh ({len(pts)} Punkte): Median {np.median(d):.3f} m, '
          f'90% {np.percentile(d, 90):.3f} m, Anteil < 0.1 m: {np.mean(d < 0.1) * 100:.0f}%')


def read_pcd(path):
    with open(path, 'rb') as f:
        n = 0
        while True:
            line = f.readline().decode()
            if line.startswith('POINTS'):
                n = int(line.split()[1])
            if line.startswith('DATA'):
                assert 'binary' in line, 'nur binaere PCD'
                return np.frombuffer(f.read(n * 12), dtype=np.float32).reshape(n, 3).astype(np.float64)


if __name__ == '__main__':
    main(*sys.argv[1:])
