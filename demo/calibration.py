"""Calibration produced by a twin (camera + LiDAR) session.

Holds what the training run measured about the physical installation:

* ``camera_axis_deg`` - the LiDAR bearing the camera looks along, so the demo
  can aim its detection sector at what the camera shows;
* ``degrees_per_pixel`` / ``frame_width`` - the bearing of any image column;
* ``blind_sectors`` - bearings permanently blocked by the mount, cabling or
  whatever the LiDAR is standing against. Returns inside them are self-
  obstruction, not obstacles, and must not drive the nearest-obstacle result.

Obstacle detection also lives here: grouping returns into clusters before
picking the nearest one keeps a single stray sample from raising a warning.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Optional, Sequence

CALIBRATION_FILENAME = "calibration.json"


def wrap180(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def angular_difference(first_deg: float, second_deg: float) -> float:
    return abs(wrap180(first_deg - second_deg))


@dataclass(frozen=True)
class BlindSector:
    start_deg: float
    end_deg: float
    median_m: float = 0.0

    def contains(self, angle_deg: float) -> bool:
        centre = wrap180(self.start_deg + wrap180(self.end_deg - self.start_deg) / 2.0)
        half = abs(wrap180(self.end_deg - self.start_deg)) / 2.0
        return angular_difference(angle_deg, centre) <= half


@dataclass
class Calibration:
    camera_axis_deg: float = 0.0
    degrees_per_pixel: float = 0.0
    frame_width: int = 0
    horizontal_fov_deg: float = 0.0
    residual_rms_deg: float = 0.0
    samples: int = 0
    blind_sectors: list[BlindSector] = field(default_factory=list)
    source: str = ""

    @classmethod
    def load(cls, path: Path | str) -> "Calibration":
        location = Path(path)
        if location.is_dir():
            location = location / CALIBRATION_FILENAME
        payload = json.loads(location.read_text(encoding="utf-8"))
        return cls(
            camera_axis_deg=float(payload.get("camera_axis_deg", 0.0)),
            degrees_per_pixel=float(payload.get("degrees_per_pixel", 0.0)),
            frame_width=int(payload.get("frame_width", 0)),
            horizontal_fov_deg=float(payload.get("horizontal_fov_deg", 0.0)),
            residual_rms_deg=float(payload.get("residual_rms_deg", 0.0)),
            samples=int(payload.get("samples", 0)),
            blind_sectors=[
                BlindSector(
                    start_deg=float(entry["start_deg"]),
                    end_deg=float(entry["end_deg"]),
                    median_m=float(entry.get("median_m", 0.0)),
                )
                for entry in payload.get("blind_sectors", [])
            ],
            source=str(payload.get("source", str(location))),
        )

    def save(self, path: Path | str) -> Path:
        location = Path(path)
        if location.is_dir():
            location = location / CALIBRATION_FILENAME
        location.write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")
        return location

    def to_dict(self) -> dict:
        return {
            "camera_axis_deg": round(self.camera_axis_deg, 2),
            "degrees_per_pixel": round(self.degrees_per_pixel, 5),
            "frame_width": self.frame_width,
            "horizontal_fov_deg": round(self.horizontal_fov_deg, 2),
            "residual_rms_deg": round(self.residual_rms_deg, 2),
            "samples": self.samples,
            "blind_sectors": [
                {
                    "start_deg": round(sector.start_deg, 1),
                    "end_deg": round(sector.end_deg, 1),
                    "median_m": round(sector.median_m, 3),
                }
                for sector in self.blind_sectors
            ],
            "source": self.source,
        }

    def pixel_to_angle(self, pixel_x: float) -> float:
        return wrap180(self.camera_axis_deg + self.degrees_per_pixel * (pixel_x - self.frame_width / 2.0))

    def angle_to_pixel(self, angle_deg: float) -> Optional[float]:
        if self.degrees_per_pixel == 0.0:
            return None
        offset = wrap180(angle_deg - self.camera_axis_deg) / self.degrees_per_pixel
        pixel = offset + self.frame_width / 2.0
        return pixel if 0.0 <= pixel <= self.frame_width else None

    def is_blind(self, angle_deg: float) -> bool:
        return any(sector.contains(angle_deg) for sector in self.blind_sectors)

    def camera_sector(self) -> tuple[float, float]:
        """Centre and width of the LiDAR sector the camera covers."""
        return self.camera_axis_deg, self.horizontal_fov_deg or 60.0


@dataclass(frozen=True)
class Obstacle:
    angle_deg: float
    distance_m: float
    width_deg: float
    points: int

    @property
    def half_width_deg(self) -> float:
        return self.width_deg / 2.0


def cluster_obstacles(
    points: Sequence[tuple[float, float]],
    gap_deg: float = 6.0,
    radial_gap_m: float = 0.30,
    min_points: int = 3,
    calibration: Optional[Calibration] = None,
) -> list[Obstacle]:
    """Group returns into obstacles, ignoring bearings the mount blocks.

    ``points`` are ``(angle_deg, distance_m)`` pairs. Groups break on an angular
    gap or a radial step, so a person standing in front of a wall does not merge
    with it.
    """
    usable = [
        (wrap180(angle), distance)
        for angle, distance in points
        if distance > 0.0 and (calibration is None or not calibration.is_blind(angle))
    ]
    if not usable:
        return []
    usable.sort(key=lambda item: item[0])

    groups: list[list[tuple[float, float]]] = []
    current = [usable[0]]
    for angle, distance in usable[1:]:
        previous_angle, previous_distance = current[-1]
        if angle - previous_angle > gap_deg or abs(distance - previous_distance) > radial_gap_m:
            groups.append(current)
            current = []
        current.append((angle, distance))
    groups.append(current)

    # The scan is a circle: a cluster may straddle the +/-180 seam.
    if len(groups) > 1:
        first_angle, first_distance = groups[0][0]
        last_angle, last_distance = groups[-1][-1]
        if (
            angular_difference(first_angle, last_angle) <= gap_deg
            and abs(first_distance - last_distance) <= radial_gap_m
        ):
            groups[0] = groups[-1] + groups[0]
            groups.pop()

    obstacles = []
    for group in groups:
        if len(group) < min_points:
            continue
        angles = [angle for angle, _ in group]
        distances = [distance for _, distance in group]
        span = abs(wrap180(angles[-1] - angles[0]))
        centre = wrap180(angles[0] + wrap180(angles[-1] - angles[0]) / 2.0)
        obstacles.append(
            Obstacle(angle_deg=centre, distance_m=min(distances), width_deg=span, points=len(group))
        )
    obstacles.sort(key=lambda obstacle: obstacle.distance_m)
    return obstacles


def nearest_obstacle(
    points: Sequence[tuple[float, float]],
    sector_centre_deg: Optional[float] = None,
    sector_width_deg: float = 360.0,
    calibration: Optional[Calibration] = None,
    **kwargs,
) -> Optional[Obstacle]:
    """Nearest clustered obstacle, optionally restricted to one sector."""
    obstacles = cluster_obstacles(points, calibration=calibration, **kwargs)
    if sector_centre_deg is not None and sector_width_deg < 360.0:
        half = sector_width_deg / 2.0
        obstacles = [
            obstacle
            for obstacle in obstacles
            if angular_difference(obstacle.angle_deg, sector_centre_deg) <= half + obstacle.half_width_deg
        ]
    return obstacles[0] if obstacles else None


def merge_sectors(
    angles_deg: Iterable[float],
    bin_width_deg: float,
    medians: dict[float, float],
    gap_bins: float = 2.5,
    pad_deg: float = 5.0,
) -> list[BlindSector]:
    """Merge neighbouring blocked bins into sectors.

    ``gap_bins`` bridges bins the mount only blocks intermittently, and
    ``pad_deg`` widens each sector so a return grazing its edge is still treated
    as self-obstruction.
    """
    ordered = sorted(angles_deg)
    if not ordered:
        return []
    sectors: list[BlindSector] = []
    run = [ordered[0]]
    for angle in ordered[1:]:
        if angle - run[-1] > bin_width_deg * gap_bins:
            sectors.append(_sector_from_run(run, bin_width_deg, medians, pad_deg))
            run = []
        run.append(angle)
    sectors.append(_sector_from_run(run, bin_width_deg, medians, pad_deg))
    if len(sectors) > 1 and angular_difference(sectors[0].start_deg, sectors[-1].end_deg) <= bin_width_deg * gap_bins:
        merged = BlindSector(
            start_deg=sectors[-1].start_deg,
            end_deg=sectors[0].end_deg,
            median_m=min(sectors[0].median_m, sectors[-1].median_m),
        )
        sectors = [merged] + sectors[1:-1]
    return sectors


def _sector_from_run(
    run: list[float], bin_width_deg: float, medians: dict[float, float], pad_deg: float = 0.0
) -> BlindSector:
    return BlindSector(
        start_deg=wrap180(run[0] - bin_width_deg / 2.0 - pad_deg),
        end_deg=wrap180(run[-1] + bin_width_deg / 2.0 + pad_deg),
        median_m=min(medians.get(angle, 0.0) for angle in run),
    )


def find_calibration(explicit: Optional[str] = None) -> Optional[Calibration]:
    """Load an explicit calibration path, else the newest recorded one."""
    if explicit:
        return Calibration.load(explicit)
    candidates = sorted(
        Path("recordings").glob(f"*/{CALIBRATION_FILENAME}"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    for candidate in candidates:
        try:
            return Calibration.load(candidate)
        except (OSError, ValueError, KeyError):
            continue
    return None
