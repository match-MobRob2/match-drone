from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from .config import Config
from .scene import Scene


@dataclass
class Poses:
    positions: np.ndarray
    yaws: np.ndarray
    pitches: np.ndarray
    ids: np.ndarray
    seed_facet: np.ndarray = field(default_factory=lambda: np.empty(0, np.int64))

    def __len__(self) -> int:
        return len(self.ids)

    @property
    def view_dirs(self) -> np.ndarray:
        cp = np.cos(self.pitches)
        return np.column_stack([
            cp * np.cos(self.yaws),
            cp * np.sin(self.yaws),
            np.sin(self.pitches),
        ])


def _tangent_basis(n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ref = np.tile(np.array([0.0, 0.0, 1.0]), (len(n), 1))
    ref[np.abs(n[:, 2]) > 0.9] = np.array([1.0, 0.0, 0.0])
    t1 = np.cross(n, ref)
    t1 /= np.maximum(np.linalg.norm(t1, axis=1, keepdims=True), 1e-12)
    return t1, np.cross(n, t1)


def _cone_directions(Nn: np.ndarray, t1: np.ndarray, t2: np.ndarray,
                     cfg: Config) -> np.ndarray:
    rings = []
    for frac in cfg.cone_fractions:
        alpha = frac * cfg.theta_max_rad
        if frac == 0.0:
            rings.append(Nn[None, :, :])
            continue
        az = np.linspace(0.0, 2 * np.pi, cfg.n_azimuth, endpoint=False)
        ring = (np.cos(alpha) * Nn[None, :, :]
                + np.sin(alpha) * (np.cos(az)[:, None, None] * t1[None, :, :]
                                   + np.sin(az)[:, None, None] * t2[None, :, :]))
        rings.append(ring)
    return np.concatenate(rings, axis=0)


def sample_pose_regions(scene: Scene, cfg: Config) -> Poses:
    C = scene.facets.centers
    Nn = scene.facets.normals
    t1, t2 = _tangent_basis(Nn)
    dists = (np.array([cfg.d_opt]) if cfg.n_distance == 1
             else np.linspace(cfg.d_min, cfg.d_max, cfg.n_distance))

    dirs = _cone_directions(Nn, t1, t2, cfg)
    D, N, _ = dirs.shape
    T = len(dists)

    positions = (C[None, None, :, :]
                 + dirs[:, None, :, :] * dists[None, :, None, None]).reshape(-1, 3)
    vd = np.broadcast_to(-dirs[:, None, :, :], (D, T, N, 3)).reshape(-1, 3)
    yaws = np.arctan2(vd[:, 1], vd[:, 0])
    pitches = np.arcsin(np.clip(vd[:, 2], -1.0, 1.0))
    seed = np.broadcast_to(scene.facets.ids[None, None, :], (D, T, N)).reshape(-1).copy()

    return Poses(positions, yaws, pitches,
                 np.arange(len(positions), dtype=np.int64), seed)


def _clearance_mask(positions: np.ndarray, scene: Scene, safety: np.ndarray) -> np.ndarray:
    dist, _ = cKDTree(scene.facets.centers).query(positions, k=1)
    return dist >= safety


def _vertical_fov_ok(positions: np.ndarray, yaws: np.ndarray, pitches: np.ndarray,
                     seed_centers: np.ndarray, cfg: Config) -> np.ndarray:
    cp = np.cos(pitches)
    fwd = np.column_stack([cp * np.cos(yaws), cp * np.sin(yaws), np.sin(pitches)])
    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    right /= np.maximum(np.linalg.norm(right, axis=1, keepdims=True), 1e-12)
    up = np.cross(right, fwd)
    u = seed_centers - positions
    x = np.einsum("ij,ij->i", u, fwd)
    z = np.einsum("ij,ij->i", u, up)
    return (x > 0.0) & (np.abs(z) <= x * np.tan(0.5 * cfg.fov_v_rad))


def dedup_keys(pos: np.ndarray, yaw: np.ndarray, pitch: np.ndarray,
               cfg: Config) -> np.ndarray:
    return np.column_stack([
        np.floor((pos - pos.min(axis=0)) / cfg.voxel).astype(np.int64),
        np.floor(np.mod(yaw, 2 * np.pi) / cfg.yaw_bin_rad).astype(np.int64),
        np.floor(pitch / cfg.pitch_bin_rad).astype(np.int64),
    ])


def project_and_dedup(raw: Poses, scene: Scene, cfg: Config) -> tuple[Poses, dict]:
    keep = _clearance_mask(raw.positions, scene, cfg.safety_distance)
    pos, yaw, pitch, seed = (raw.positions[keep], raw.yaws[keep],
                             raw.pitches[keep], raw.seed_facet[keep])
    n_after_clear = len(pos)

    if cfg.use_ground_plane:
        gok = pos[:, 2] >= scene.bounds[0][2] + cfg.ground_clearance
        pos, yaw, pitch, seed = pos[gok], yaw[gok], pitch[gok], seed[gok]
    n_after_ground = len(pos)

    pitch = np.clip(pitch, cfg.pitch_min_rad, cfg.pitch_max_rad)
    feas = _vertical_fov_ok(pos, yaw, pitch, scene.facets.centers[seed], cfg)
    pos, yaw, pitch, seed = pos[feas], yaw[feas], pitch[feas], seed[feas]
    n_after_feas = len(pos)

    if n_after_feas == 0:
        empty = Poses(np.empty((0, 3)), np.empty(0), np.empty(0),
                      np.empty(0, np.int64), np.empty(0, np.int64))
        return empty, {"n_raw": len(raw), "n_after_clearance": n_after_clear,
                       "n_after_ground": n_after_ground,
                       "n_after_feasibility": 0, "n_after_dedup": 0, "dedup_ratio": 0.0}

    _, first_idx = np.unique(dedup_keys(pos, yaw, pitch, cfg), axis=0,
                             return_index=True)
    first_idx = np.sort(first_idx)

    poses = Poses(
        positions=pos[first_idx],
        yaws=yaw[first_idx],
        pitches=pitch[first_idx],
        ids=np.arange(len(first_idx), dtype=np.int64),
        seed_facet=seed[first_idx],
    )
    stats = {
        "n_raw": len(raw),
        "n_after_clearance": n_after_clear,
        "n_after_ground": n_after_ground,
        "n_after_feasibility": n_after_feas,
        "n_after_dedup": len(poses),
        "dedup_ratio": len(poses) / max(1, n_after_feas),
    }
    return poses, stats
