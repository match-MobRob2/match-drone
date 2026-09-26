from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Facets:
    centers: np.ndarray
    normals: np.ndarray
    areas: np.ndarray
    ids: np.ndarray

    def __len__(self) -> int:
        return len(self.ids)


@dataclass
class Scene:
    mesh: object
    facets: Facets
    name: str = "scene"
    facet_verts: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    facet_tris: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.int64))

    @property
    def vertices(self) -> np.ndarray:
        return np.asarray(self.mesh.vertices)

    @property
    def triangles(self) -> np.ndarray:
        return np.asarray(self.mesh.triangles)

    @property
    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        v = self.vertices
        return v.min(axis=0), v.max(axis=0)

    def summary(self) -> str:
        lo, hi = self.bounds
        ext = hi - lo
        return (
            f"Scene '{self.name}': {len(self.triangles)} Occluder-Dreiecke, "
            f"{len(self.facets)} Facetten\n"
            f"  Ausdehnung: {ext[0]:.1f} × {ext[1]:.1f} × {ext[2]:.1f} m"
        )


def _edge_points_2d(pa: np.ndarray, pb: np.ndarray, res: float) -> np.ndarray:
    seg = max(1, int(round(np.linalg.norm(pb - pa) / res)))
    if seg < 2:
        return np.empty((0, 2))
    t = np.arange(1, seg)[:, None] / seg
    return pa[None] + t * (pb - pa)[None]


def _facets_from_triangles(verts: np.ndarray, tris: np.ndarray,
                           resolution: float,
                           assume_convex: bool = False) -> tuple:
    from scipy.spatial import Delaunay, cKDTree

    all_centers: list[np.ndarray] = []
    all_normals: list[np.ndarray] = []
    all_areas:   list[np.ndarray] = []
    all_fv:      list[np.ndarray] = []
    all_ft:      list[np.ndarray] = []
    vtx_off = 0

    for i in range(len(tris)):
        a = verts[tris[i, 0]].astype(float)
        b = verts[tris[i, 1]].astype(float)
        c = verts[tris[i, 2]].astype(float)

        n_raw = np.cross(b - a, c - a)
        ln = np.linalg.norm(n_raw)
        if ln < 1e-12:
            continue
        n_hat = n_raw / ln

        u_ax = (b - a) / np.linalg.norm(b - a)
        v_ax = np.cross(n_hat, u_ax)
        v_ax = v_ax / np.linalg.norm(v_ax)

        pa = np.zeros(2)
        pb = np.array([np.dot(b - a, u_ax), np.dot(b - a, v_ax)])
        pc = np.array([np.dot(c - a, u_ax), np.dot(c - a, v_ax)])

        boundary = [pa[None], pb[None], pc[None],
                    _edge_points_2d(pa, pb, resolution),
                    _edge_points_2d(pb, pc, resolution),
                    _edge_points_2d(pc, pa, resolution)]
        bpts = np.vstack([p for p in boundary if len(p)])

        p2 = np.stack([pa, pb, pc])
        interior = np.empty((0, 2))
        us = np.arange(p2[:, 0].min(), p2[:, 0].max() + resolution, resolution)
        vs = np.arange(p2[:, 1].min(), p2[:, 1].max() + resolution, resolution)
        if len(us) and len(vs):
            ug, vg = np.meshgrid(us, vs)
            cand = np.stack([ug.ravel(), vg.ravel()], axis=1)
            T = np.array([[pb[0] - pa[0], pc[0] - pa[0]],
                          [pb[1] - pa[1], pc[1] - pa[1]]])
            lam = np.linalg.solve(T, (cand - pa).T).T
            l1 = 1.0 - lam[:, 0] - lam[:, 1]
            cand = cand[(l1 > 0) & (lam[:, 0] > 0) & (lam[:, 1] > 0)]
            if len(cand):
                d, _ = cKDTree(bpts).query(cand)
                interior = cand[d > 0.5 * resolution]

        pts2d = np.vstack([bpts, interior]) if len(interior) else bpts

        if len(pts2d) < 3:
            ctrs2d = ((pa + pb + pc) / 3.0)[None]
            ctrs3d = a + ctrs2d[:, 0:1] * u_ax + ctrs2d[:, 1:2] * v_ax
            all_fv.append(np.stack([a, b, c]))
            all_ft.append(np.array([[0, 1, 2]], dtype=np.int64) + vtx_off)
            vtx_off += 3
            all_centers.append(ctrs3d)
            all_normals.append(n_hat[None])
            all_areas.append(np.array([0.5 * ln]))
            continue

        simp = Delaunay(pts2d).simplices
        d01 = pts2d[simp[:, 1]] - pts2d[simp[:, 0]]
        d02 = pts2d[simp[:, 2]] - pts2d[simp[:, 0]]
        cw = (d01[:, 0] * d02[:, 1] - d01[:, 1] * d02[:, 0]) < 0
        simp[cw] = simp[cw][:, [0, 2, 1]]
        pts3d = a + pts2d[:, 0:1] * u_ax + pts2d[:, 1:2] * v_ax

        tri3 = pts3d[simp]
        cr = np.cross(tri3[:, 1] - tri3[:, 0], tri3[:, 2] - tri3[:, 0])

        all_fv.append(pts3d)
        all_ft.append(simp.astype(np.int64) + vtx_off)
        vtx_off += len(pts3d)

        all_centers.append(tri3.mean(axis=1))
        all_normals.append(np.tile(n_hat, (len(simp), 1)))
        all_areas.append(0.5 * np.linalg.norm(cr, axis=1))

    centers = np.vstack(all_centers)
    normals = np.vstack(all_normals)

    if assume_convex:
        centroid = centers.mean(axis=0)
        flip = np.einsum("ij,ij->i", normals, centers - centroid) < 0
        normals[flip] *= -1.0

    facets = Facets(
        centers=centers,
        normals=normals,
        areas=np.concatenate(all_areas),
        ids=np.arange(len(centers), dtype=np.int64),
    )
    fv = np.vstack(all_fv) if all_fv else np.empty((0, 3))
    ft = np.vstack(all_ft).astype(np.int64) if all_ft else np.empty((0, 3), dtype=np.int64)
    return facets, fv, ft


def _make_mesh(verts: np.ndarray, tris: np.ndarray):
    import open3d as o3d
    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(np.asarray(verts, float)),
        o3d.utility.Vector3iVector(np.asarray(tris, np.int32)),
    )
    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()
    return mesh


def build_scene(verts: np.ndarray, tris: np.ndarray, resolution: float,
                name: str = "scene", assume_convex: bool = False) -> Scene:
    facets, V, T = _facets_from_triangles(verts, tris, resolution, assume_convex)
    return Scene(mesh=_make_mesh(verts, tris), facets=facets, name=name,
                 facet_verts=V, facet_tris=T.astype(np.int64))


def _prism(poly2d: np.ndarray, z0: float, z1: float) -> tuple[np.ndarray, np.ndarray]:
    m = len(poly2d)
    bottom = np.column_stack([poly2d, np.full(m, z0)])
    top = np.column_stack([poly2d, np.full(m, z1)])
    verts = np.vstack([bottom, top])
    tris = []
    for i in range(m):
        j = (i + 1) % m
        tris.append([i, j, j + m])
        tris.append([i, j + m, i + m])
    for i in range(1, m - 1):
        tris.append([0, i + 1, i])
    for i in range(1, m - 1):
        tris.append([m, m + i, m + i + 1])
    return verts, np.asarray(tris, dtype=np.int64)


def box(resolution: float, size: float = 4.0, name: str = "box") -> Scene:
    h = size / 2.0
    poly = np.array([[-h, -h], [h, -h], [h, h], [-h, h]], dtype=float)
    return build_scene(*_prism(poly, -h, h), resolution, name)


def notched_box(resolution: float, w: float = 4.4, h: float = 2.0, d: float = 3.1,
                notch: float = 1.2, name: str = "notched_box") -> Scene:
    cy = h / 2.0
    poly = np.array([
        [0.0, 0.0],
        [w, 0.0],
        [w, cy - notch / 2],
        [w - notch, cy - notch / 2],
        [w - notch, cy + notch / 2],
        [w, cy + notch / 2],
        [w, h],
        [0.0, h],
    ], dtype=float)
    return build_scene(*_prism(poly, 0.0, d), resolution, name)


def from_file(path: str, resolution: float, assume_convex: bool = False,
              poisson_depth: int = 8, name: str | None = None) -> Scene:
    import open3d as o3d

    stem = path.replace("\\", "/").split("/")[-1].rsplit(".", 1)[0]
    nm = name or stem

    mesh = o3d.io.read_triangle_mesh(path)
    if len(mesh.triangles) > 0:
        return build_scene(np.asarray(mesh.vertices), np.asarray(mesh.triangles),
                           resolution, nm, assume_convex)

    pcd = o3d.io.read_point_cloud(path)
    nn = np.asarray(pcd.compute_nearest_neighbor_distance())
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=3.0 * float(np.median(nn)), max_nn=30))
    if assume_convex:
        pcd.orient_normals_towards_camera_location(pcd.get_center())
        pcd.normals = o3d.utility.Vector3dVector(-np.asarray(pcd.normals))
    else:
        pcd.orient_normals_consistent_tangent_plane(30)
        # konsistent, aber Vorzeichen beliebig -> Mehrheit soll vom Schwerpunkt weg zeigen
        # ponytail: Mehrheitsentscheid, versagt bei Hohlkoerpern (Innenscan) -- dann Sensorposen nutzen
        pts, nrm = np.asarray(pcd.points), np.asarray(pcd.normals)
        if np.einsum("ij,ij->i", nrm, pts - pts.mean(axis=0)).mean() < 0:
            pcd.normals = o3d.utility.Vector3dVector(-nrm)
    rec, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=poisson_depth)
    dens = np.asarray(densities)
    rec.remove_vertices_by_mask(dens < np.quantile(dens, 0.02))
    return build_scene(np.asarray(rec.vertices), np.asarray(rec.triangles),
                       resolution, nm, assume_convex=False)


SCENES = {
    "box": box,
    "notched_box": notched_box,
}


def scene_from_args(args, cfg) -> Scene:
    if getattr(args, "mesh", None):
        return from_file(args.mesh, cfg.resolution, assume_convex=args.assume_convex)
    return SCENES[args.scene](cfg.resolution)
