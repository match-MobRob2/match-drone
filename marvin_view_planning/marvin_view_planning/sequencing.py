from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _nearest_neighbor(D: np.ndarray, start: int) -> np.ndarray:
    n = len(D)
    visited = np.zeros(n, dtype=bool)
    order = np.empty(n, dtype=np.int64)
    order[0] = start
    visited[start] = True
    for i in range(1, n):
        d = D[order[i - 1]].copy()
        d[visited] = np.inf
        order[i] = int(np.argmin(d))
        visited[order[i]] = True
    return order


def _two_opt(order: np.ndarray, D: np.ndarray, max_rounds: int = 12) -> np.ndarray:
    order = order.copy()
    n = len(order)
    for _ in range(max_rounds):
        improved = False
        for i in range(n - 2):
            for j in range(i + 2, n):
                a, b, c = order[i], order[i + 1], order[j]
                if j + 1 < n:
                    d = order[j + 1]
                    d_old = D[a, b] + D[c, d]
                    d_new = D[a, c] + D[b, d]
                else:
                    d_old = D[a, b]
                    d_new = D[a, c]
                if d_new + 1e-12 < d_old:
                    order[i + 1 : j + 1] = order[i + 1 : j + 1][::-1]
                    improved = True
        if not improved:
            break
    return order


def solve_tsp_path(points: np.ndarray, start: int = 0) -> np.ndarray:
    n = len(points)
    if n <= 2:
        others = [i for i in range(n) if i != start]
        return np.asarray([start] + others, dtype=np.int64)
    D = np.linalg.norm(points[:, None] - points[None, :], axis=2)
    return _two_opt(_nearest_neighbor(D, start), D)


@dataclass
class Route:
    order: np.ndarray
    length: float
    station_ids: np.ndarray = None
    n_repositions: int = 0

    def __post_init__(self) -> None:
        if self.station_ids is None:
            self.station_ids = np.full(len(self.order), -1, dtype=np.int64)

    def summary(self) -> str:
        txt = f"Route: {len(self.order)} Posen, Flugweg {self.length:.1f} m"
        if self.n_repositions or (self.station_ids >= 0).any():
            n_seg = int(self.station_ids.max()) + 1 if (self.station_ids >= 0).any() else 0
            txt += (f", {n_seg} Tracking-Segmente "
                    f"({self.n_repositions} Repositionierungen)")
        return txt


def sequence_route(pose_positions: np.ndarray, tracking=None) -> Route:
    positions = np.asarray(pose_positions, dtype=np.float64).reshape(-1, 3)
    P = len(positions)
    if P == 0:
        return Route(np.empty(0, np.int64), 0.0, np.empty(0, np.int64), 0)

    stations = (np.asarray(tracking.stations, dtype=np.float64).reshape(-1, 3)
                if tracking is not None else np.empty((0, 3)))

    if len(stations) == 0:
        order = solve_tsp_path(positions, start=0)
        length = float(np.linalg.norm(np.diff(positions[order], axis=0), axis=1).sum())
        return Route(order, length, np.full(P, -1, np.int64), 0)

    assignment = np.asarray(tracking.assignment, dtype=np.int64).reshape(-1).copy()

    untracked = assignment < 0
    if untracked.any():
        d = np.linalg.norm(
            stations[:, None, :2] - positions[None, untracked, :2], axis=2)
        assignment[untracked] = np.argmin(d, axis=0)

    counts = np.bincount(assignment, minlength=len(stations))
    st_order = solve_tsp_path(stations, start=int(np.argmax(counts)))

    route_idx: list[int] = []
    station_per_pose: list[int] = []
    visited = 0
    prev_end: np.ndarray | None = None
    for st in st_order:
        members = np.flatnonzero(assignment == st)
        if len(members) == 0:
            continue
        pts = positions[members]
        start = 0 if prev_end is None else int(
            np.argmin(np.linalg.norm(pts - prev_end, axis=1)))
        local = solve_tsp_path(pts, start=start)
        route_idx.extend(members[local].tolist())
        station_per_pose.extend([visited] * len(members))
        visited += 1
        prev_end = pts[local[-1]]

    order = np.asarray(route_idx, dtype=np.int64)
    length = float(np.linalg.norm(np.diff(positions[order], axis=0), axis=1).sum())

    station_ids = np.asarray(station_per_pose, dtype=np.int64)
    station_ids[untracked[order]] = -1

    return Route(order, length, station_ids, max(0, visited - 1))
