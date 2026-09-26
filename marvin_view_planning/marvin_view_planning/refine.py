from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree

from .candidates import Poses
from .config import Config
from .scene import Facets

_EPS = 1e-9
_PENALTY = 1.0e6


def _aim(position: np.ndarray, target: np.ndarray, cfg: Config):
    look = target - position
    horiz = float(np.hypot(look[0], look[1]))
    yaw = float(np.arctan2(look[1], look[0])) if horiz > _EPS else 0.0
    pitch = float(np.clip(np.arctan2(look[2], max(horiz, _EPS)),
                          cfg.pitch_min_rad, cfg.pitch_max_rad))
    cp, sp = np.cos(pitch), np.sin(pitch)
    return yaw, pitch, np.array([cp * np.cos(yaw), cp * np.sin(yaw), sp])


def pose_quality(
    position: np.ndarray,
    facet_pos: np.ndarray,
    facet_norm: np.ndarray,
    cfg: Config,
    *,
    weights: np.ndarray | None = None,
    sigma_scale: float = 1.5,
    return_detail: bool = False,
):
    position = np.asarray(position, dtype=np.float64).reshape(3)
    n = len(facet_pos)
    wt = np.ones(n) if weights is None else np.asarray(weights, dtype=np.float64)
    w = facet_pos - position
    d = np.linalg.norm(w, axis=1)
    u = w / np.maximum(d, _EPS)[:, None]

    cos_inc = -np.einsum("ij,ij->i", facet_norm, u)
    cos_tmax = np.cos(cfg.theta_max_rad)
    gate_di = (d >= cfg.d_min) & (d <= cfg.d_max) & (cos_inc >= cos_tmax)

    sig_near = max((cfg.d_opt - cfg.d_min) / sigma_scale, _EPS)
    sig_far = max((cfg.d_max - cfg.d_opt) / sigma_scale, _EPS)
    sigma = np.where(d < cfg.d_opt, sig_near, sig_far)
    q_dist = np.exp(-0.5 * ((d - cfg.d_opt) / sigma) ** 2)

    c = np.clip((cos_inc - cos_tmax) / max(1.0 - cos_tmax, _EPS), 0.0, 1.0)
    q_inc = c * c * (3.0 - 2.0 * c)

    w_di = np.where(gate_di, q_dist * q_inc * wt, 0.0)
    detail = {"yaw": 0.0, "pitch": 0.0, "forward": np.array([1.0, 0.0, 0.0]),
              "q": np.zeros(n)}
    if not w_di.any():
        return (0.0, detail) if return_detail else 0.0

    target = (w_di[:, None] * facet_pos).sum(axis=0) / w_di.sum()
    yaw, pitch, fwd = _aim(position, target, cfg)

    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    nr = np.linalg.norm(right)
    right = right / nr if nr > _EPS else np.array([0.0, 1.0, 0.0])
    up = np.cross(right, fwd)
    x = (u @ fwd) * d
    y = (u @ right) * d
    z = (u @ up) * d
    x_safe = np.where(x > _EPS, x, _EPS)
    ry = y / (x_safe * np.tan(0.5 * cfg.fov_h_rad))
    rz = z / (x_safe * np.tan(0.5 * cfg.fov_v_rad))
    gate_f = (x > _EPS) & (np.abs(ry) <= 1.0) & (np.abs(rz) <= 1.0)
    q_frust = np.clip(1.0 - ry ** 2, 0.0, 1.0) * np.clip(1.0 - rz ** 2, 0.0, 1.0)

    q = np.where(gate_di & gate_f, w_di * q_frust, 0.0)
    J = float(q.sum())
    if return_detail:
        return J, {"yaw": yaw, "pitch": pitch, "forward": fwd, "q": q}
    return J


@dataclass
class RefineInfo:
    j_before: np.ndarray
    j_after: np.ndarray
    displacement: np.ndarray
    soft_cov_before: np.ndarray
    soft_cov_after: np.ndarray
    mean_inc_before: np.ndarray
    mean_inc_after: np.ndarray
    d_err_before: np.ndarray
    d_err_after: np.ndarray
    n_iter: np.ndarray = field(default_factory=lambda: np.empty(0))

    def summary(self) -> str:
        def pct(a, b):
            a, b = float(np.nanmean(a)), float(np.nanmean(b))
            return f"{a:.3g} -> {b:.3g}  ({(b - a) / max(abs(a), 1e-9):+.1%})"
        ang_b = float(np.degrees(np.nanmean(self.mean_inc_before)))
        ang_a = float(np.degrees(np.nanmean(self.mean_inc_after)))
        return (
            f"Lokale Verfeinerung: {len(self.j_before):,} Posen\n"
            f"  Score J         : {pct(self.j_before, self.j_after)}\n"
            f"  Weiche Abdeckung: {pct(self.soft_cov_before, self.soft_cov_after)} Facetten/Pose\n"
            f"  Einfallswinkel  : {ang_b:.1f}° -> {ang_a:.1f}° (kleiner = frontaler)\n"
            f"  |d - d_opt|     : {pct(self.d_err_before, self.d_err_after)} m\n"
            f"  Verschiebung    : Ø {np.mean(self.displacement):.3g} m, "
            f"max {np.max(self.displacement, initial=0.0):.3g} m"
        )


def _pose_metrics(position, facet_pos, facet_norm, cfg):
    _, det = pose_quality(position, facet_pos, facet_norm, cfg, return_detail=True)
    vis = det["q"] > 0.0
    n = int(vis.sum())
    if n == 0:
        return 0, float("nan"), float("nan")
    w = facet_pos[vis] - position
    d = np.linalg.norm(w, axis=1)
    u = w / np.maximum(d, _EPS)[:, None]
    cos_inc = np.clip(-np.einsum("ij,ij->i", facet_norm[vis], u), -1.0, 1.0)
    return n, float(np.mean(np.arccos(cos_inc))), float(np.mean(np.abs(d - cfg.d_opt)))


def refine_poses(
    poses: Poses,
    facets: Facets,
    cfg: Config,
    *,
    target_facets: list[np.ndarray] | None = None,
    max_offset: float | None = None,
    init_step: float | None = None,
    sigma_scale: float = 1.5,
    coverage_weight: float = 4.0,
    gain_weight: float = 0.0,
    z_min: float | None = None,
    max_iter: int = 120,
) -> tuple[Poses, RefineInfo]:
    max_offset = 0.5 * cfg.d_opt if max_offset is None else float(max_offset)
    init_step = 0.15 * cfg.d_opt if init_step is None else float(init_step)
    P, Nn = facets.centers, facets.normals
    tree = cKDTree(P)
    search_r = cfg.d_max + max_offset

    S = len(poses)
    new_pos = poses.positions.copy()
    new_yaw = poses.yaws.copy()
    new_pitch = poses.pitches.copy()

    j_before = np.zeros(S)
    j_after = np.zeros(S)
    disp = np.zeros(S)
    sc_b = np.zeros(S, dtype=np.int64)
    sc_a = np.zeros(S, dtype=np.int64)
    inc_b = np.full(S, np.nan)
    inc_a = np.full(S, np.nan)
    de_b = np.full(S, np.nan)
    de_a = np.full(S, np.nan)
    n_iter = np.zeros(S, dtype=np.int64)

    for j in range(S):
        base = poses.positions[j]
        if target_facets is not None:
            assigned = np.asarray(target_facets[j], dtype=np.int64)
            n_assigned = len(assigned)
            if gain_weight > 0.0 and n_assigned:
                nearby = np.asarray(tree.query_ball_point(base, r=search_r),
                                    dtype=np.int64)
                extra = np.setdiff1d(nearby, assigned, assume_unique=False)
                idx = np.concatenate([assigned, extra])
                wt = np.concatenate([np.ones(n_assigned),
                                     np.full(len(extra), float(gain_weight))])
            else:
                idx, wt = assigned, None
        else:
            idx = np.asarray(tree.query_ball_point(base, r=search_r), dtype=np.int64)
            wt = None
            n_assigned = 0
        if len(idx) == 0:
            j_before[j] = j_after[j] = 0.0
            new_pos[j] = base
            continue
        nbr_pos, nbr_norm = P[idx], Nn[idx]

        guard = (target_facets is not None and coverage_weight > 0.0
                 and n_assigned > 0)

        def objective(offset, base=base, nbr_pos=nbr_pos, nbr_norm=nbr_norm,
                      wt=wt, guard=guard, n_assigned=n_assigned):
            pos = base + offset
            J, det = pose_quality(pos, nbr_pos, nbr_norm, cfg, weights=wt,
                                  sigma_scale=sigma_scale, return_detail=True)
            pen = 0.0
            r = float(np.linalg.norm(offset))
            if r > max_offset:
                pen += (r - max_offset)
            clear = float(tree.query(pos, k=1)[0])
            if clear < cfg.safety_distance:
                pen += (cfg.safety_distance - clear)
            if z_min is not None and pos[2] < z_min:
                pen += (z_min - pos[2])
            cov_pen = (coverage_weight * int(np.count_nonzero(det["q"][:n_assigned] <= 0.0))
                       if guard else 0.0)
            return -J + cov_pen + _PENALTY * pen

        sc_b[j], inc_b[j], de_b[j] = _pose_metrics(base, nbr_pos, nbr_norm, cfg)
        j_before[j] = -objective(np.zeros(3))

        res = minimize(
            objective, np.zeros(3), method="Nelder-Mead",
            options={"initial_simplex": np.vstack([np.zeros(3), init_step * np.eye(3)]),
                     "maxiter": max_iter, "xatol": 1e-3, "fatol": 1e-4},
        )
        n_iter[j] = int(res.nit)

        if -res.fun > j_before[j]:
            pos = base + res.x
            j_after[j] = -res.fun
        else:
            pos = base.copy()
            j_after[j] = j_before[j]
        new_pos[j] = pos
        disp[j] = float(np.linalg.norm(pos - base))
        sc_a[j], inc_a[j], de_a[j] = _pose_metrics(pos, nbr_pos, nbr_norm, cfg)

        _, det = pose_quality(pos, nbr_pos, nbr_norm, cfg, weights=wt,
                              return_detail=True)
        new_yaw[j], new_pitch[j] = det["yaw"], det["pitch"]

    refined = Poses(
        positions=new_pos,
        yaws=new_yaw,
        pitches=new_pitch,
        ids=poses.ids.copy(),
        seed_facet=poses.seed_facet.copy(),
    )
    info = RefineInfo(
        j_before=j_before, j_after=j_after, displacement=disp,
        soft_cov_before=sc_b, soft_cov_after=sc_a,
        mean_inc_before=inc_b, mean_inc_after=inc_a,
        d_err_before=de_b, d_err_after=de_a, n_iter=n_iter,
    )
    return refined, info


def target_facets_from_visibility(V, rows: np.ndarray) -> list[np.ndarray]:
    import scipy.sparse as sp

    Vsel = sp.csr_matrix(V)[rows]
    return [Vsel.indices[Vsel.indptr[k]:Vsel.indptr[k + 1]]
            for k in range(Vsel.shape[0])]


def subset_poses(poses: Poses, rows: np.ndarray) -> Poses:
    rows = np.asarray(rows, dtype=np.int64)
    return Poses(
        positions=poses.positions[rows],
        yaws=poses.yaws[rows],
        pitches=poses.pitches[rows],
        ids=poses.ids[rows],
        seed_facet=(poses.seed_facet[rows] if len(poses.seed_facet)
                    else poses.seed_facet),
    )


def refine_selected_poses(
    vis, poses: Poses, scene, selected: np.ndarray, cfg: Config,
    *,
    free: bool = False,
    gain_weight: float = 0.0,
    max_offset: float | None = None,
    sigma_scale: float = 1.5,
    coverage_weight: float = 4.0,
    z_min: float | None = None,
) -> tuple[Poses, Poses, RefineInfo]:
    sel = np.asarray(selected, dtype=np.int64)
    before = subset_poses(poses, sel)
    targets = None if free else target_facets_from_visibility(vis.V, sel)

    refined, info = refine_poses(
        before, scene.facets, cfg,
        target_facets=targets,
        max_offset=max_offset, sigma_scale=sigma_scale,
        coverage_weight=coverage_weight, gain_weight=gain_weight, z_min=z_min,
    )

    poses.positions[sel] = refined.positions
    poses.yaws[sel] = refined.yaws
    poses.pitches[sel] = refined.pitches
    return before, refined, info
