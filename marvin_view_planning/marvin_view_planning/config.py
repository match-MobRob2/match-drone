from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:
    import tomli as tomllib  # ROS 2 Humble / Ubuntu 22.04 -> Python 3.10

_DEFAULT_PATH = Path(__file__).with_name("config.toml")


def _rad(deg_field: str):
    return property(lambda self: math.radians(getattr(self, deg_field)))


@dataclass(frozen=True)
class Config:
    d_min: float
    d_max: float
    d_opt: float

    fov_h_deg: float
    fov_v_deg: float

    baseline: float

    theta_max_deg: float

    pitch_min_deg: float
    pitch_max_deg: float

    safety_distance: float
    use_ground_plane: bool
    ground_clearance: float

    min_overlap: float
    erosion: float

    track_range_min: float
    track_range_max: float
    track_grid_step: float
    track_margin: float

    n_azimuth: int
    cone_fractions: tuple[float, ...]
    n_distance: int
    voxel: float
    yaw_bin_deg: float
    pitch_bin_deg: float
    resolution: float

    fov_h_rad = _rad("fov_h_deg")
    fov_v_rad = _rad("fov_v_deg")
    theta_max_rad = _rad("theta_max_deg")
    pitch_min_rad = _rad("pitch_min_deg")
    pitch_max_rad = _rad("pitch_max_deg")
    yaw_bin_rad = _rad("yaw_bin_deg")
    pitch_bin_rad = _rad("pitch_bin_deg")

    def summary(self) -> str:
        return (
            "Config(3D):\n"
            f"  Arbeitsabstand : [{self.d_min}, {self.d_max}] m (d_opt={self.d_opt})\n"
            f"  FoV (h x v)    : {self.fov_h_deg}° x {self.fov_v_deg}° / Inzidenz ≤{self.theta_max_deg}°\n"
            f"  Pitch-Klemmung : [{self.pitch_min_deg}°, {self.pitch_max_deg}°] (Roll gesperrt)\n"
            f"  Basislinie     : {self.baseline} m "
            f"({'duale Sicht' if self.baseline > 0 else 'Einzelsicht'})\n"
            f"  Overlap (Reg.) : {self.min_overlap:.0%} gemeinsame Flaeche (kleinerer Scan)"
            + (f" | Erosion {self.erosion:.2f} m" if self.erosion > 0 else "")
            + "\n"
            f"  Bodenfilter    : {'an' if self.use_ground_plane else 'aus'} "
            f"(z_boden + {self.ground_clearance} m Bodenabstand)\n"
            f"  Tracking       : LoS-Standorte, Reichweite [{self.track_range_min}, "
            f"{self.track_range_max}] m, Raster {self.track_grid_step} m\n"
            f"  Sampling       : {len(self.cone_fractions)}×{self.n_azimuth} Kegel-Richtungen "
            f"× {self.n_distance}×Distanz, Dedup {self.voxel:.2f} m / "
            f"{self.yaw_bin_deg}° / {self.pitch_bin_deg}°"
        )


def load_config(path: str | Path | None = None) -> Config:
    path = Path(path) if path is not None else _DEFAULT_PATH
    with open(path, "rb") as fh:
        raw = tomllib.load(fh)

    wd = raw["working_distance"]
    d_min = float(wd["d_min"])
    d_max = float(wd["d_max"])
    d_opt = float(wd.get("d_opt", (d_min + d_max) / 2.0))

    smp = raw.get("sampling", {})
    orient = raw.get("orientation", {})
    trk = raw.get("tracking", {})

    return Config(
        d_min=d_min,
        d_max=d_max,
        d_opt=d_opt,
        fov_h_deg=float(raw["fov"]["fov_h_deg"]),
        fov_v_deg=float(raw["fov"]["fov_v_deg"]),
        baseline=float(raw.get("sensor", {}).get("baseline", 0.0)),
        theta_max_deg=float(raw["incidence"]["theta_max_deg"]),
        pitch_min_deg=float(orient.get("pitch_min_deg", -10.0)),
        pitch_max_deg=float(orient.get("pitch_max_deg", 10.0)),
        safety_distance=float(raw["drone"]["safety_distance"]),
        use_ground_plane=bool(raw["drone"].get("use_ground_plane", True)),
        ground_clearance=float(raw["drone"].get("ground_clearance",
                                                raw["drone"]["safety_distance"])),
        min_overlap=float(raw["registration"]["min_overlap"]),
        erosion=float(raw["registration"].get("erosion", 0.0)),
        track_range_min=float(trk.get("range_min", 1.0)),
        track_range_max=float(trk.get("range_max", 8.0)),
        track_grid_step=float(trk.get("grid_step", 0.5)),
        track_margin=float(trk.get("margin", 2.0)),
        n_azimuth=int(smp.get("n_azimuth", 8)),
        cone_fractions=tuple(float(x) for x in smp.get("cone_fractions", [0.0, 0.5, 0.85])),
        n_distance=int(smp.get("n_distance", 2)),
        voxel=float(smp.get("voxel", 0.15 * d_opt)),
        yaw_bin_deg=float(smp.get("yaw_bin_deg", 15.0)),
        pitch_bin_deg=float(smp.get("pitch_bin_deg", 10.0)),
        resolution=float(smp.get("resolution", 0.25)),
    )


def with_overrides(cfg: Config, args) -> Config:
    if getattr(args, "baseline", None) is not None:
        cfg = replace(cfg, baseline=args.baseline)
    if getattr(args, "resolution", None) is not None:
        cfg = replace(cfg, resolution=args.resolution)
    if getattr(args, "no_ground_plane", False):
        cfg = replace(cfg, use_ground_plane=False)
    return cfg
