from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import scipy.sparse as sp
from scipy.sparse.csgraph import connected_components

from .visibility import VisibilityMatrix


@dataclass
class OverlapResult:
    selected: np.ndarray
    added: np.ndarray
    n_components: int
    overlap_facets: np.ndarray
    min_overlap: float
    extra: dict = field(default_factory=dict)

    def summary(self) -> str:
        ok = "zusammenhaengend" if self.n_components == 1 else (
            f"NICHT zusammenhaengend ({self.n_components} Komponenten)"
        )
        return (
            f"Registrierungsgraph (min_overlap={self.min_overlap:.0%} Flaeche, "
            f"kleinerer Scan):\n"
            f"  Posen                    : {len(self.selected):,} "
            f"(+{len(self.added):,} Bruecken ergaenzt)\n"
            f"  Ueberlappungs-Facetten   : {len(self.overlap_facets):,} "
            f"(von >= 2 Scans gesehen)\n"
            f"  Graph                    : {ok}"
        )


def overlap_fractions(V: sp.csr_matrix, rows: np.ndarray) -> np.ndarray:
    Vs = V[rows].astype(np.int64)
    inter = np.asarray((Vs @ Vs.T).todense())
    sizes = np.asarray(Vs.sum(axis=1)).ravel()
    return inter / np.maximum(np.minimum.outer(sizes, sizes), 1)


def registration_graph(
    V: sp.csr_matrix, rows: np.ndarray, min_overlap: float
) -> np.ndarray:
    adj = overlap_fractions(V, rows) >= min_overlap
    np.fill_diagonal(adj, False)
    return adj


def overlap_adjacency(
    V: sp.csr_matrix, min_overlap: float
) -> tuple[np.ndarray, sp.csr_matrix]:
    Vc = V.tocsr().astype(np.int64)
    sizes_all = np.asarray(Vc.sum(axis=1)).ravel()
    nodes = np.flatnonzero(sizes_all > 0).astype(np.int64)
    n = len(nodes)
    if n == 0:
        return nodes, sp.csr_matrix((0, 0), dtype=np.float32)

    Vn = Vc[nodes]
    inter = (Vn @ Vn.T).tocoo()
    sizes = sizes_all[nodes]

    upper = inter.row < inter.col
    a, b, val = inter.row[upper], inter.col[upper], inter.data[upper]
    frac = val / np.maximum(np.minimum(sizes[a], sizes[b]), 1)
    edge = frac >= min_overlap
    a, b, frac = a[edge], b[edge], frac[edge].astype(np.float32)

    adj = sp.csr_matrix((frac, (a, b)), shape=(n, n), dtype=np.float32)
    return nodes, adj + adj.T


def _overlap_facets(V: sp.csr_matrix, rows: np.ndarray) -> np.ndarray:
    if len(rows) == 0:
        return np.empty(0, dtype=np.int64)
    cov = np.asarray(V[rows].sum(axis=0)).ravel()
    return np.flatnonzero(cov >= 2).astype(np.int64)


def ensure_connected(
    vis: VisibilityMatrix,
    selected: np.ndarray,
    min_overlap: float,
) -> OverlapResult:
    V = vis.V.tocsr()
    sel = list(np.asarray(selected, dtype=np.int64))
    added: list[int] = []

    while True:
        rows = np.asarray(sel, dtype=np.int64)
        adj = registration_graph(V, rows, min_overlap)
        n_comp, labels = connected_components(sp.csr_matrix(adj), directed=False)
        if n_comp <= 1:
            break

        pool = np.setdiff1d(np.arange(V.shape[0]), rows, assume_unique=False)
        if len(pool) == 0:
            break

        Vp = V[pool].astype(np.int64)
        Vs = V[rows].astype(np.int64)
        inter = np.asarray((Vp @ Vs.T).todense())
        size_p = np.asarray(Vp.sum(axis=1)).ravel()
        size_s = np.asarray(Vs.sum(axis=1)).ravel()
        frac = inter / np.maximum(np.minimum.outer(size_p, size_s), 1)
        link = frac >= min_overlap

        comp_onehot = np.eye(n_comp, dtype=bool)[labels]
        comps_touched = (link @ comp_onehot.astype(np.int64)) > 0
        n_touched = comps_touched.sum(axis=1)

        best = int(np.lexsort((-frac.sum(axis=1), -n_touched))[0])
        if n_touched[best] < 2:
            break
        sel.append(int(pool[best]))
        added.append(int(pool[best]))

    rows = np.asarray(sel, dtype=np.int64)
    adj = registration_graph(V, rows, min_overlap)
    n_comp, _ = connected_components(sp.csr_matrix(adj), directed=False)
    return OverlapResult(
        selected=rows,
        added=np.asarray(added, dtype=np.int64),
        n_components=int(n_comp),
        overlap_facets=_overlap_facets(V, rows),
        min_overlap=float(min_overlap),
    )
