#!/usr/bin/env python3
"""Auswertung von drift_flight.py: Fehler je Quelle gegen Gazebo-Ground-Truth.

Jede Quelle wird zu Beginn (erste gueltige Zeile) per Translation + Yaw auf
die Ground Truth ausgerichtet — gemessen wird also nur der Drift danach.
Aufruf: python3 drift_report.py drift.csv
"""
import csv
import math
import sys

import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def main(path):
    rows = list(csv.DictReader(open(path)))
    f = lambda k: np.array([float(r[k]) for r in rows])  # noqa: E731
    phase = [r['phase'] for r in rows]
    gt = np.stack([f('gt_x'), f('gt_y'), f('gt_z')], 1)
    gyaw = f('gt_yaw')

    print(f'{len(rows)} Samples, {rows[-1]["t"] and float(rows[-1]["t"]) - float(rows[0]["t"]):.0f} s')
    print(f'{"Quelle":6} {"Phase":10} {"xy-Fehler [m]":>14} {"z [m]":>7} {"Yaw [°]":>8}')
    for src in ('px4', 'lio', 'map'):
        p = np.stack([f(f'{src}_x'), f(f'{src}_y'), f(f'{src}_z')], 1)
        yaw = f(f'{src}_yaw')
        ok = ~np.isnan(p).any(1)
        if not ok.any():
            print(f'{src}: keine Daten')
            continue
        i0 = int(np.argmax(ok))
        # Ausrichtung am ersten gueltigen Sample: Quelle -> GT
        dyaw = gyaw[i0] - yaw[i0]
        c, s = math.cos(dyaw), math.sin(dyaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        aligned = (p - p[i0]) @ R.T + gt[i0]
        err = aligned - gt
        eyaw = np.degrees(wrap(yaw + dyaw - gyaw))
        # je Phase der Fehler am Phasenende (= nach dem Einschwingen)
        last = {}
        for i, ph in enumerate(phase):
            if ok[i]:
                last[ph] = i
        for ph, i in last.items():
            print(f'{src:6} {ph:10} {np.hypot(*err[i, :2]):14.3f} {err[i, 2]:7.3f} {eyaw[i]:8.2f}')
        print(f'{src:6} {"MAX":10} {np.nanmax(np.hypot(err[ok, 0], err[ok, 1])):14.3f} '
              f'{np.nanmax(np.abs(err[ok, 2])):7.3f} {np.nanmax(np.abs(eyaw[ok])):8.2f}')
        print()

    corr = np.stack([f('corr_x'), f('corr_y'), f('corr_z')], 1)
    cyaw = f('corr_yaw')
    ok = ~np.isnan(corr).any(1)
    if ok.any():
        d = corr[ok] - corr[ok][0]
        print(f'Relokalisierungs-Korrektur map->camera_init: max |dxy| {np.max(np.hypot(d[:, 0], d[:, 1])):.3f} m, '
              f'max |dz| {np.max(np.abs(d[:, 2])):.3f} m, '
              f'max |dyaw| {np.degrees(np.max(np.abs(wrap(cyaw[ok] - cyaw[ok][0])))):.2f}°')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'drift.csv')
