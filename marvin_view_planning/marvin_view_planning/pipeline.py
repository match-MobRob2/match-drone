"""Nicht-visuelle Pipeline-Orchestrierung: Szene/Mesh -> ScanRoute.

Buendelt die Schritte [1]-[7] aus vpp3d/run.py (siehe Studienarbeit-Repo) zu
einer einzigen Bibliotheksfunktion, ohne Matplotlib/Open3D-Anzeige, ohne CLI
und ohne Lauf-Logging (RunLog) -- als Ausgangspunkt fuer die spaetere
Anbindung an einen ROS-2-Node (Ausgabe: Posen + Tracking-Standorte + Route).
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .config import Config, load_config
from .scene import Scene, SCENES, from_file
from .candidates import Poses, sample_pose_regions, project_and_dedup
from .visibility import VisibilityMatrix, compute_visibility
from .setcover import (
    CoverResult, greedy_set_cover, ilp_set_cover, connected_set_cover_ilp,
)
from .overlap import OverlapResult, ensure_connected
from .refine import refine_selected_poses
from .tracking import TrackingPlan, plan_tracking
from .sequencing import Route, sequence_route


@dataclass
class PlanResult:
    scene: Scene
    poses: Poses                    # alle Kandidatenposen (Kegel-Sampling, [2]/[3])
    visibility: VisibilityMatrix     # volle Sichtbarkeitsmatrix ueber `poses`
    cover: CoverResult
    selected: np.ndarray             # Zeilenindizes in `poses`, die der Plan waehlt
    overlap: OverlapResult | None
    tracking: TrackingPlan | None
    route: Route


def plan_scan_route(
    *,
    mesh_path: str | None = None,
    scene_name: str = "box",
    assume_convex: bool = False,
    config_path: str | None = None,
    resolution: float | None = None,
    method: str = "greedy",
    k: int = 1,
    time_limit: float = 60.0,
    repair_overlap: bool = True,
    with_tracking: bool = True,
    refine: bool = False,
    refine_kwargs: dict | None = None,
    overrides: dict | None = None,
    poisson_depth: int = 8,
) -> PlanResult:
    """Fuehrt Schritt [1]-[7] der vpp3d-Pipeline ohne Visualisierung aus.

    `mesh_path` (STL/PLY) ersetzt `scene_name` (synthetische Testszene aus
    `SCENES`, z.B. "box"). Rueckgabe ist die vollstaendige `PlanResult`
    inklusive aller Zwischenobjekte (Kandidatenposen, Sichtbarkeitsmatrix,
    Set-Cover-Ergebnis), damit ein aufrufender ROS-Node beliebig
    weiterverarbeiten kann, z.B. `poses.positions[selected]`, `route.order`,
    `tracking.stations`.
    """
    cfg = load_config(config_path)
    if overrides:  # Config-Felder, z.B. aus der UI; Unbekanntes -> TypeError
        cfg = replace(cfg, **overrides)
    if resolution is not None:
        cfg = replace(cfg, resolution=resolution)

    # Punktwolke: Poisson-Tiefe bestimmt die Facettengroesse (Dreiecke < resolution)
    scene = (from_file(mesh_path, cfg.resolution, assume_convex=assume_convex, poisson_depth=poisson_depth)
              if mesh_path else SCENES[scene_name](cfg.resolution))

    raw = sample_pose_regions(scene, cfg)
    poses, _ = project_and_dedup(raw, scene, cfg)

    vis = compute_visibility(scene, poses, cfg)

    if method == "greedy":
        cover = greedy_set_cover(vis.V, k=k)
    elif method == "ilp":
        cover = ilp_set_cover(vis.V, k=k, time_limit=time_limit)
    elif method == "connected":
        cover = connected_set_cover_ilp(vis.V, cfg.min_overlap, k=k,
                                        time_limit=time_limit)
    else:
        raise ValueError(f"unbekannte Methode: {method!r}")

    selected = cover.poses
    overlap = None
    if repair_overlap:
        overlap = ensure_connected(vis, selected, cfg.min_overlap)
        selected = overlap.selected

    if refine:
        refine_selected_poses(vis, poses, scene, selected, cfg,
                              **(refine_kwargs or {}))

    tracking = None
    if with_tracking:
        tracking = plan_tracking(scene, poses.positions[selected], cfg)

    route = sequence_route(poses.positions[selected], tracking)

    return PlanResult(scene, poses, vis, cover, selected, overlap, tracking, route)
