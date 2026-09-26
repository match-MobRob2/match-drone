"""View Planning fuer die Punkte eines Objekts (Aufruf durch den Supervisor).

    python -m marvin_view_planning.plan_box --defaults
    python -m marvin_view_planning.plan_box <punkte.npy> <ergebnis.json> [optionen.json]

--defaults       config.toml + Planer-Optionen als JSON auf stdout (Startwerte der UI).
<punkte.npy>     float32 Nx3 im map-Frame, vom Supervisor auf die Box zugeschnitten
                 (Karte + Live-Lidar). Daraus wird die Oberflaeche rekonstruiert
                 (Poisson, scene.from_file) und die Scan-Posen geplant.
<optionen.json>  {"config": {Config-Feld: Wert}, "method", "k", "time_limit", "refine",
                 "repair_overlap", "voxel", "poisson_depth"} -- fehlende Werte = Defaults.
<ergebnis.json>  Vorschau fuer die UI: Posen in Flugreihenfolge (auch unerreichbare,
                 reachable=false), Facetten mit Abdeckungsstatus, ausgeduennte Punkte, Kennzahlen.
Laeuft in einer eigenen venv (System-numpy/scipy vertragen sich nicht, README).
"""
import dataclasses
import json
import math
import sys
import tempfile
import time

import numpy as np

from .config import Config, load_config
from .pipeline import plan_scan_route

# Hoehenband des Planers (marvin_nav nav_node.hpp min/max_flight_z_) -- Posen
# ausserhalb waeren unerreichbar (Planer klemmt das Ziel -> Wegpunkt-Timeout)
FLIGHT_Z = (0.3, 3.0)
OPTS = {'method': 'greedy', 'k': 1, 'time_limit': 60.0, 'refine': False, 'repair_overlap': True, 'voxel': 0.1,
        'poisson_depth': 8}
# abgeleitet (d_opt = Mitte des Arbeitsabstands, Dedup-Raster 0.15 * d_opt), nicht direkt einstellbar
DERIVED = ('d_opt', 'voxel')


def defaults():
    cfg = dataclasses.asdict(load_config())
    return {'config': {k: v for k, v in cfg.items() if k not in DERIVED}, **OPTS}


def _config(over):
    c = {**defaults()['config'], **(over or {})}
    c['cone_fractions'] = tuple(c['cone_fractions'])
    for f in dataclasses.fields(Config):  # Browser-JSON: 8.0 statt 8 -> linspace bricht
        cast = {'int': int, 'float': float, 'bool': bool}.get(f.type)
        if cast and f.name in c:
            c[f.name] = cast(c[f.name])
    if not 0 < c['d_min'] < c['d_max']:
        raise ValueError('Arbeitsabstand: 0 < d_min < d_max')
    if c['resolution'] < 0.05:  # Facettenzahl ~ 1/res^2 -> Rechenzeit explodiert
        raise ValueError('Facetten-Aufloesung mindestens 0.05 m')
    c['d_opt'] = (c['d_min'] + c['d_max']) / 2
    c['voxel'] = 0.15 * c['d_opt']
    return c


def _r(a, nd=2):
    return np.round(np.asarray(a, float), nd).ravel().tolist()


def plan(points, opts=None):
    import open3d as o3d
    t0 = time.time()
    o = {**OPTS, **(opts or {})}
    cfg = _config(o.get('config'))
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(points, float).reshape(-1, 3)))
    pcd = pcd.voxel_down_sample(float(o['voxel']))
    if len(pcd.points) < 100:
        raise ValueError(f'nur {len(pcd.points)} Punkte in der Box')
    with tempfile.NamedTemporaryFile(suffix='.ply') as f:
        o3d.io.write_point_cloud(f.name, pcd)
        r = plan_scan_route(mesh_path=f.name, with_tracking=False, method=o['method'], k=int(o['k']),
                            time_limit=float(o['time_limit']), refine=bool(o['refine']),
                            repair_overlap=bool(o['repair_overlap']), overrides=cfg,
                            poisson_depth=int(o['poisson_depth']))

    idx = r.selected[r.route.order]
    pos = r.poses.positions[idx]
    reach = (pos[:, 2] >= FLIGHT_Z[0]) & (pos[:, 2] <= FLIGHT_Z[1])
    # ponytail: nach refine ist V veraltet (Posen verschoben) -> Abdeckung dann nur Naeherung
    V = r.visibility.V.tocsr()
    coverable = np.asarray(V.sum(axis=0)).ravel() > 0
    covered = np.asarray(V[idx[reach]].sum(axis=0)).ravel() > 0
    state = np.where(covered, 2, np.where(coverable, 1, 0))  # 2 erfasst, 1 erfassbar, 0 unerreichbar
    area = r.scene.facets.areas
    rp = pos[reach]
    return {
        'poses': [{'x': round(float(p[0]), 2), 'y': round(float(p[1]), 2), 'z': round(float(p[2]), 2),
                   'yaw_deg': round(math.degrees(float(y)), 1), 'pitch_deg': round(math.degrees(float(pt)), 1),
                   'reachable': bool(ok), 'n_facets': int(V[i].nnz)}
                  for i, p, y, pt, ok in zip(idx, pos, r.poses.yaws[idx], r.poses.pitches[idx], reach)],
        'facets': {'centers': _r(r.scene.facets.centers), 'normals': _r(r.scene.facets.normals),
                   'state': state.tolist()},
        'points': _r(np.asarray(pcd.points)),
        'config': {k: v for k, v in cfg.items() if k != 'cone_fractions'},
        'stats': {
            'points': len(pcd.points), 'facets': len(state), 'area_m2': round(float(area.sum()), 1),
            'coverage': round(float(area[state == 2].sum() / area.sum()), 3),
            'coverable': round(float(area[state >= 1].sum() / area.sum()), 3),
            'candidates': len(r.poses), 'selected': len(idx), 'reachable': int(reach.sum()),
            'length_m': round(float(np.linalg.norm(np.diff(rp, axis=0), axis=1).sum()) if len(rp) > 1 else 0.0, 1),
            'runtime_s': round(time.time() - t0, 1),
        },
    }


def main():
    if sys.argv[1:] == ['--defaults']:
        print(json.dumps(defaults()))
        return
    if len(sys.argv) not in (3, 4):
        sys.exit(__doc__)
    opts = json.load(open(sys.argv[3])) if len(sys.argv) == 4 else {}
    try:
        out = plan(np.load(sys.argv[1]), opts)
    except (ValueError, TypeError) as e:  # TypeError: unbekanntes Config-Feld
        sys.exit(str(e))
    with open(sys.argv[2], 'w') as f:
        json.dump(out, f)


if __name__ == '__main__':
    main()
