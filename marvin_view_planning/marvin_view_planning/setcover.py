from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
import scipy.sparse as sp
from scipy.sparse.csgraph import connected_components, breadth_first_order


@dataclass
class CoverResult:
    poses: np.ndarray
    method: str
    k: int = 1
    n_coverable: int = 0
    n_covered: int = 0
    runtime: float = 0.0
    optimal: bool = False
    extra: dict = field(default_factory=dict)

    def summary(self) -> str:
        opt = " (Optimum bewiesen)" if self.optimal else ""
        kstr = f", k={self.k}" if self.k > 1 else ""
        return (
            f"Set Cover [{self.method}{opt}{kstr}]:\n"
            f"  Drohnenposen (Scans)            : {len(self.poses)}\n"
            f"  Abdeckung erreichbarer Facetten : "
            f"{self.n_covered}/{self.n_coverable} "
            f"({self.n_covered / max(1, self.n_coverable):.1%})\n"
            f"  Rechenzeit                      : {self.runtime:.2f} s"
        )


def _demand(V: sp.csr_matrix, k: int) -> np.ndarray:
    avail = np.asarray(V.sum(axis=0)).ravel().astype(np.int64)
    return np.minimum(int(k), avail)


def greedy_set_cover(
    V: sp.csr_matrix, k: int = 1, trace: list | None = None
) -> CoverResult:
    t0 = time.perf_counter()
    Vc = V.tocsr().astype(np.int64)
    demand = _demand(Vc, k)
    coverable = demand > 0
    deficit = demand.copy()
    chosen = np.zeros(Vc.shape[0], dtype=bool)
    selected: list[int] = []

    while deficit.any():
        gain = np.asarray(Vc @ (deficit > 0).astype(np.int64)).ravel()
        gain[chosen] = 0
        j = int(np.argmax(gain))
        if gain[j] == 0:
            break
        seen = Vc.indices[Vc.indptr[j]:Vc.indptr[j + 1]]
        if trace is not None:
            open_seen = deficit[seen] > 0
            trace.append({
                "pose": int(j),
                "covered_before": (coverable & (deficit == 0)).copy(),
                "new_facets": seen[open_seen].copy(),
                "overlap_facets": seen[~open_seen].copy(),
            })
        chosen[j] = True
        selected.append(j)
        deficit[seen] = np.maximum(deficit[seen] - 1, 0)

    return CoverResult(
        poses=np.asarray(selected, dtype=np.int64),
        method="greedy",
        k=int(k),
        n_coverable=int(coverable.sum()),
        n_covered=int((coverable & (deficit == 0)).sum()),
        runtime=time.perf_counter() - t0,
    )


def ilp_set_cover(
    V: sp.csr_matrix, k: int = 1, time_limit: float = 120.0
) -> CoverResult:
    from ortools.sat.python import cp_model

    t0 = time.perf_counter()
    Vcsc = V.tocsc()
    M, N = V.shape
    demand = _demand(V, k)

    model = cp_model.CpModel()
    x = [model.new_bool_var(f"x{j}") for j in range(M)]
    for i in np.flatnonzero(demand):
        rows = Vcsc.indices[Vcsc.indptr[i]:Vcsc.indptr[i + 1]]
        model.add(sum(x[j] for j in rows) >= int(demand[i]))
    model.minimize(sum(x))

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = float(time_limit)
    solver.parameters.num_workers = 8
    status = solver.solve(model)
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        raise RuntimeError(f"CP-SAT ohne Loesung ({solver.status_name(status)})")

    sel = np.asarray([j for j in range(M) if solver.value(x[j])], dtype=np.int64)
    times = np.asarray(V.tocsr()[sel].sum(axis=0)).ravel() if sel.size else np.zeros(N)
    return CoverResult(
        poses=sel,
        method="ilp",
        k=int(k),
        n_coverable=int((demand > 0).sum()),
        n_covered=int(((demand > 0) & (times >= demand)).sum()),
        runtime=time.perf_counter() - t0,
        optimal=status == cp_model.OPTIMAL,
        extra={"solver_status": solver.status_name(status)},
    )


def _sparsify_topk(adj: sp.csr_matrix, max_degree: int) -> sp.csr_matrix:
    A = adj.tocsr()
    n = A.shape[0]
    rows: list[np.ndarray] = []
    cols: list[np.ndarray] = []
    for i in range(n):
        s, e = A.indptr[i], A.indptr[i + 1]
        idx, dat = A.indices[s:e], A.data[s:e]
        if len(idx) > max_degree:
            idx = idx[np.argpartition(dat, -max_degree)[-max_degree:]]
        rows.append(np.full(len(idx), i, dtype=np.int64))
        cols.append(idx.astype(np.int64))
    r = np.concatenate(rows) if rows else np.empty(0, np.int64)
    c = np.concatenate(cols) if cols else np.empty(0, np.int64)
    m = sp.csr_matrix((np.ones(len(r), dtype=bool), (r, c)), shape=(n, n))
    return (m + m.T).astype(bool)


def _hint_from_warm(model, warm_local, adj_s, x, y, g, arc_var, n) -> None:
    w = len(warm_local)
    x_h = np.zeros(n, np.int64)
    y_h = np.zeros(n, np.int64)
    g_h = np.zeros(n, np.int64)
    arc_h = {key: 0 for key in arc_var}
    if w >= 1:
        x_h[warm_local] = 1
    if w == 1:
        r = int(warm_local[0]); y_h[r] = 1; g_h[r] = 1
    elif w >= 2:
        sub = adj_s[warm_local][:, warm_local]
        order, preds = breadth_first_order(sub, 0, directed=False,
                                           return_predecessors=True)
        subtree = np.ones(w, np.int64)
        for node in order[::-1]:
            p = preds[node]
            if p >= 0:
                subtree[p] += subtree[node]
        root = int(warm_local[0]); y_h[root] = 1; g_h[root] = int(w)
        for node in order:
            p = preds[node]
            if p < 0:
                continue
            key = (int(warm_local[p]), int(warm_local[node]))
            if key in arc_h:
                arc_h[key] = int(subtree[node])
    for j in range(n):
        model.add_hint(x[j], int(x_h[j]))
        model.add_hint(y[j], int(y_h[j]))
        model.add_hint(g[j], int(g_h[j]))
    for key, var in arc_var.items():
        model.add_hint(var, int(arc_h[key]))


def connected_set_cover_ilp(
    V: sp.csr_matrix, min_overlap: float, k: int = 1, time_limit: float = 120.0,
    max_degree: int = 8,
) -> CoverResult:
    from ortools.sat.python import cp_model

    from .overlap import overlap_adjacency, ensure_connected
    from .visibility import VisibilityMatrix

    t0 = time.perf_counter()
    Vcsr = V.tocsr()
    N = Vcsr.shape[1]
    demand = _demand(Vcsr, k)

    nodes, adj = overlap_adjacency(Vcsr, min_overlap)
    n = len(nodes)
    if n == 0:
        return CoverResult(poses=np.empty(0, np.int64), method="connected",
                           k=int(k), runtime=time.perf_counter() - t0)

    Vn = Vcsr[nodes].tocsc()
    g2l = {int(gid): li for li, gid in enumerate(nodes)}

    warm = ensure_connected(VisibilityMatrix(V=Vcsr),
                            greedy_set_cover(Vcsr, k=k).poses, min_overlap)
    warm_local = np.array([g2l[int(p)] for p in warm.selected], dtype=np.int64)
    cap = max(1, len(warm_local))

    adj_s = _sparsify_topk(adj, max_degree)
    if len(warm_local) > 1:
        sub = adj[warm_local][:, warm_local].tocoo()
        wr = warm_local[sub.row]; wc = warm_local[sub.col]
        extra = sp.csr_matrix((np.ones(len(wr), bool), (wr, wc)), shape=(n, n))
        adj_s = (adj_s + extra + extra.T).astype(bool)
    n_comp, comp = connected_components(adj_s, directed=False)

    model = cp_model.CpModel()
    x = [model.new_bool_var(f"x{j}") for j in range(n)]
    y = [model.new_bool_var(f"y{j}") for j in range(n)]
    g = [model.new_int_var(0, cap, f"g{j}") for j in range(n)]

    for i in np.flatnonzero(demand):
        rows_i = Vn.indices[Vn.indptr[i]:Vn.indptr[i + 1]]
        model.add(sum(x[j] for j in rows_i) >= int(demand[i]))

    for j in range(n):
        model.add(y[j] <= x[j])
        model.add(g[j] <= cap * y[j])
    for c in range(n_comp):
        members = np.flatnonzero(comp == c)
        model.add(sum(y[int(j)] for j in members) <= 1)

    edges = sp.triu(adj_s, k=1).tocoo()
    inflow: list[list] = [[] for _ in range(n)]
    outflow: list[list] = [[] for _ in range(n)]
    arc_var: dict[tuple[int, int], object] = {}
    for u, v in zip(edges.row.tolist(), edges.col.tolist()):
        f_uv = model.new_int_var(0, cap, f"f_{u}_{v}")
        f_vu = model.new_int_var(0, cap, f"f_{v}_{u}")
        model.add(f_uv <= cap * x[u]); model.add(f_uv <= cap * x[v])
        model.add(f_vu <= cap * x[u]); model.add(f_vu <= cap * x[v])
        outflow[u].append(f_uv); inflow[v].append(f_uv)
        outflow[v].append(f_vu); inflow[u].append(f_vu)
        arc_var[(u, v)] = f_uv; arc_var[(v, u)] = f_vu

    for j in range(n):
        model.add(g[j] + sum(inflow[j]) - sum(outflow[j]) == x[j])

    model.minimize(sum(x))
    model.add(sum(x) <= cap)

    _hint_from_warm(model, warm_local, adj_s, x, y, g, arc_var, n)

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = float(time_limit)
    solver.parameters.num_workers = 8
    status = solver.solve(model)
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        raise RuntimeError(f"CP-SAT ohne Loesung ({solver.status_name(status)})")

    poses = nodes[[j for j in range(n) if solver.value(x[j])]]
    times = np.asarray(Vcsr[poses].sum(axis=0)).ravel() if len(poses) else np.zeros(N)
    return CoverResult(
        poses=poses.astype(np.int64),
        method="connected",
        k=int(k),
        n_coverable=int((demand > 0).sum()),
        n_covered=int(((demand > 0) & (times >= demand)).sum()),
        runtime=time.perf_counter() - t0,
        optimal=status == cp_model.OPTIMAL,
        extra={"solver_status": solver.status_name(status),
               "graph_components": int(n_comp),
               "warm_poses": int(len(warm_local)),
               "objective_bound": float(solver.best_objective_bound)},
    )
