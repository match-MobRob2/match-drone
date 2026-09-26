from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .config import Config
from .scene import Scene
from .visibility import ray_clear


@dataclass
class TrackingPlan:
    stations: np.ndarray
    assignment: np.ndarray
    pose_positions: np.ndarray
    n_untrackable: int = 0
    extra: dict = field(default_factory=dict)

    def summary(self) -> str:
        txt = (
            f"Tracking (LoS): {len(self.stations):,} Standorte "
            f"(= {max(0, len(self.stations) - 1):,} Repositionierungen)"
        )
        if self.n_untrackable:
            txt += (f"\n  WARNUNG: {self.n_untrackable:,} Posen nicht trackbar "
                    f"(ausserhalb Reichweite oder rundum verdeckt)!")
        return txt


def plan_tracking(scene: Scene, positions: np.ndarray, cfg: Config,
                  ray_tolerance: float | None = None) -> TrackingPlan:
    import open3d as o3d

    positions = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    S = len(positions)
    if S == 0:
        return TrackingPlan(np.empty((0, 3)), np.empty(0, np.int64),
                            positions, 0)

    if ray_tolerance is None:
        ray_tolerance = max(0.02, 0.5 * cfg.resolution)

    ground = float(scene.bounds[0][2])
    lo = positions[:, :2].min(axis=0) - cfg.track_margin
    hi = positions[:, :2].max(axis=0) + cfg.track_margin
    xs = np.arange(lo[0], hi[0] + cfg.track_grid_step, cfg.track_grid_step)
    ys = np.arange(lo[1], hi[1] + cfg.track_grid_step, cfg.track_grid_step)
    gx, gy = np.meshgrid(xs, ys)
    cand = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, ground)])
    T = len(cand)

    d3 = np.linalg.norm(cand[:, None, :] - positions[None, :, :], axis=2)
    within = (d3 >= cfg.track_range_min) & (d3 <= cfg.track_range_max)

    rscene = o3d.t.geometry.RaycastingScene()
    rscene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(scene.mesh))
    covers = np.zeros((T, S), dtype=bool)
    ti, pj = np.nonzero(within)
    if len(ti):
        los = ray_clear(rscene, cand[ti], positions[pj], ray_tolerance)
        covers[ti[los], pj[los]] = True

    trackable = covers.any(axis=0)

    deficit = trackable.copy()
    chosen: list[int] = []
    while deficit.any():
        gain = covers[:, deficit].sum(axis=1)
        t = int(np.argmax(gain))
        if gain[t] == 0:
            break
        chosen.append(t)
        deficit &= ~covers[t]

    assignment = np.full(S, -1, dtype=np.int64)
    if chosen:
        d_masked = np.where(covers[chosen], d3[chosen], np.inf)
        nearest = np.argmin(d_masked, axis=0)
        ok = np.isfinite(d_masked[nearest, np.arange(S)])
        assignment[ok] = nearest[ok]

    return TrackingPlan(
        stations=cand[chosen] if chosen else np.empty((0, 3)),
        assignment=assignment,
        pose_positions=positions,
        n_untrackable=int((~trackable).sum()),
        extra={"n_candidates": T},
    )
