from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
import scipy.sparse as sp
from scipy.spatial import cKDTree

from .candidates import Poses
from .config import Config
from .scene import Scene

_EPS = 1e-12


@dataclass
class VisibilityMatrix:
    V: sp.csr_matrix
    stats: dict = field(default_factory=dict)
    runtime: float = 0.0

    @property
    def shape(self) -> tuple[int, int]:
        return self.V.shape

    def coverage_per_facet(self) -> np.ndarray:
        return np.asarray(self.V.sum(axis=0)).ravel()

    def summary(self) -> str:
        cov = self.coverage_per_facet()
        reach = int((cov > 0).sum())
        n = self.V.shape[1]
        lines = [
            f"Coverage-Matrix V: {self.V.shape[0]:,} Posen × {n:,} Facetten, "
            f"{self.V.nnz:,} Sichtbarkeiten",
            f"  erreichbare Facetten: {reach}/{n} ({reach / max(1, n):.1%})",
            f"  Rechenzeit          : {self.runtime:.2f} s",
        ]
        if self.stats:
            lines.append("  Filterstufen        : " + " -> ".join(
                f"{k}={v:,}" for k, v in self.stats.items()))
        return "\n".join(lines)


def ray_clear(rscene, origins: np.ndarray, targets: np.ndarray,
              tol: float, batch: int = 1_000_000) -> np.ndarray:
    import open3d as o3d

    d = targets - origins
    dd = np.linalg.norm(d, axis=1)
    dirs = d / np.maximum(dd[:, None], _EPS)
    ok = np.empty(len(origins), dtype=bool)
    for s in range(0, len(origins), batch):
        e = min(s + batch, len(origins))
        rays = o3d.core.Tensor(
            np.hstack([origins[s:e], dirs[s:e]]).astype(np.float32))
        t_hit = rscene.cast_rays(rays)["t_hit"].numpy().astype(np.float64)
        ok[s:e] = t_hit >= dd[s:e] - tol
    return ok


def _frames(poses: Poses) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    fwd = poses.view_dirs
    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    right /= np.maximum(np.linalg.norm(right, axis=1, keepdims=True), _EPS)
    return fwd, right, np.cross(right, fwd)


def _in_frustum(diff: np.ndarray, fwd: np.ndarray, right: np.ndarray,
                up: np.ndarray, tan_h: float, tan_v: float) -> np.ndarray:
    x = np.einsum("ij,ij->i", diff, fwd)
    y = np.einsum("ij,ij->i", diff, right)
    z = np.einsum("ij,ij->i", diff, up)
    return (x > 0.0) & (np.abs(y) <= x * tan_h) & (np.abs(z) <= x * tan_v)


def compute_visibility(scene: Scene, poses: Poses, cfg: Config,
                       ray_tolerance: float | None = None,
                       ray_batch: int = 1_000_000) -> VisibilityMatrix:
    import open3d as o3d

    t0 = time.perf_counter()
    C = scene.facets.centers
    Nn = scene.facets.normals
    M, N = len(poses), len(scene.facets)
    if M == 0 or N == 0:
        return VisibilityMatrix(sp.csr_matrix((M, N), dtype=bool),
                                runtime=time.perf_counter() - t0)

    if ray_tolerance is None:
        ray_tolerance = max(0.02, 0.5 * cfg.resolution)
    tan_h = np.tan(0.5 * cfg.fov_h_rad)
    tan_v = np.tan(0.5 * cfg.fov_v_rad)
    cos_theta = np.cos(cfg.theta_max_rad)

    fwd, right, up = _frames(poses)
    p_proj = poses.positions
    p_cam = p_proj + cfg.baseline * right

    neigh = cKDTree(C).query_ball_point(p_proj, r=cfg.d_max)
    counts = np.fromiter((len(l) for l in neigh), dtype=np.int64, count=M)
    pose_idx = np.repeat(np.arange(M, dtype=np.int64), counts)
    patch_idx = (np.concatenate([np.asarray(l, np.int64) for l in neigh if l])
                 if counts.sum() else np.empty(0, np.int64))
    stats = {"d_max": int(len(pose_idx))}

    diff = C[patch_idx] - p_proj[pose_idx]
    dist = np.linalg.norm(diff, axis=1)
    keep = dist >= cfg.d_min
    pose_idx, patch_idx, diff, dist = pose_idx[keep], patch_idx[keep], diff[keep], dist[keep]
    stats["d_min"] = int(len(pose_idx))

    u = diff / np.maximum(dist[:, None], _EPS)
    keep = -np.einsum("ij,ij->i", Nn[patch_idx], u) >= cos_theta
    pose_idx, patch_idx, diff, dist = pose_idx[keep], patch_idx[keep], diff[keep], dist[keep]
    stats["incidence"] = int(len(pose_idx))

    keep = _in_frustum(diff, fwd[pose_idx], right[pose_idx], up[pose_idx], tan_h, tan_v)
    pose_idx, patch_idx, diff, dist = pose_idx[keep], patch_idx[keep], diff[keep], dist[keep]
    stats["frustum_proj"] = int(len(pose_idx))

    diff_c = C[patch_idx] - p_cam[pose_idx]
    keep = _in_frustum(diff_c, fwd[pose_idx], right[pose_idx], up[pose_idx], tan_h, tan_v)
    pose_idx, patch_idx = pose_idx[keep], patch_idx[keep]
    stats["frustum_cam"] = int(len(pose_idx))

    rscene = o3d.t.geometry.RaycastingScene()
    rscene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(scene.mesh))

    if len(pose_idx):
        clear_p = ray_clear(rscene, p_proj[pose_idx], C[patch_idx],
                            ray_tolerance, ray_batch)
        pose_idx, patch_idx = pose_idx[clear_p], patch_idx[clear_p]
        stats["occlusion_proj"] = int(len(pose_idx))
        if cfg.baseline > 0 and len(pose_idx):
            clear_c = ray_clear(rscene, p_cam[pose_idx], C[patch_idx],
                                ray_tolerance, ray_batch)
            pose_idx, patch_idx = pose_idx[clear_c], patch_idx[clear_c]
        stats["dual"] = int(len(pose_idx))
    else:
        stats["occlusion_proj"] = 0
        stats["dual"] = 0

    V = sp.csr_matrix(
        (np.ones(len(pose_idx), dtype=bool), (pose_idx, patch_idx)),
        shape=(M, N))
    return VisibilityMatrix(V=V, stats=stats, runtime=time.perf_counter() - t0)
