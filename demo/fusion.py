"""Pairing a camera detection with the LiDAR cluster that explains it.

The camera says *something new is there and this is where it is in the image*;
the LiDAR says *something is there and this is how far*. The twin calibration
turns the first into a bearing, which is the only unit the two share, so the
association is a bearing comparison and nothing more.

A match is deliberately generous: the camera box is the visible outline and the
LiDAR cluster is the part of the object at scanner height, so the two centres
rarely coincide. The tolerance is widened by half the cluster's own width, the
same rule `nearest_obstacle` already uses for its sector test.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

from demo.calibration import Calibration, Obstacle, angular_difference, cluster_obstacles, wrap180
from demo.rgb_obstacle import CameraObstacle


# Both sensors agreeing is the only state that carries a distance.
AGREEMENT_BOTH = "both"
AGREEMENT_CAMERA_ONLY = "camera_only"
AGREEMENT_LIDAR_ONLY = "lidar_only"
AGREEMENT_CLEAR = "clear"

DEFAULT_MATCH_TOLERANCE_DEG = 12.0

# Which sensor the reported bearing came from. In a lit room the camera rarely
# supplies one: the tracker's box comes from its dark-blob pass, and a lit
# object is not dark, so the LiDAR names the direction and the camera only says
# that something is new there.
SOURCE_CAMERA = "camera"
SOURCE_LIDAR = "lidar"


@dataclass(frozen=True)
class FusedDetection:
    """What the two sensors together say about one obstacle."""

    agreement: str
    camera: Optional[CameraObstacle] = None
    bearing_deg: Optional[float] = None
    width_deg: Optional[float] = None
    lidar: Optional[Obstacle] = None
    match_error_deg: Optional[float] = None
    bearing_source: Optional[str] = None

    @property
    def distance_m(self) -> Optional[float]:
        return self.lidar.distance_m if self.lidar else None

    @property
    def confirmed(self) -> bool:
        return self.agreement == AGREEMENT_BOTH

    def describe(self) -> str:
        if self.agreement == AGREEMENT_BOTH and self.lidar is not None:
            aimed = (
                f"camera {self.bearing_deg:+.1f} deg"
                if self.bearing_source == SOURCE_CAMERA and self.bearing_deg is not None
                else "camera has no bearing"
            )
            return (
                f"confirmed {self.lidar.distance_m:.2f} m at "
                f"{self.lidar.angle_deg:+.1f} deg ({aimed})"
            )
        if self.agreement == AGREEMENT_CAMERA_ONLY:
            where = f" at {self.bearing_deg:+.1f} deg" if self.bearing_deg is not None else ""
            return f"camera only{where}, no LiDAR return"
        if self.agreement == AGREEMENT_LIDAR_ONLY and self.lidar is not None:
            return (
                f"LiDAR only {self.lidar.distance_m:.2f} m at "
                f"{self.lidar.angle_deg:+.1f} deg, camera sees nothing new"
            )
        return "clear"

    def to_dict(self) -> dict:
        return {
            "agreement": self.agreement,
            "bearing_deg": round(self.bearing_deg, 2) if self.bearing_deg is not None else None,
            "width_deg": round(self.width_deg, 1) if self.width_deg is not None else None,
            "distance_m": round(self.lidar.distance_m, 4) if self.lidar else None,
            "lidar_angle_deg": round(self.lidar.angle_deg, 2) if self.lidar else None,
            "lidar_points": self.lidar.points if self.lidar else None,
            "match_error_deg": round(self.match_error_deg, 2) if self.match_error_deg is not None else None,
            "bearing_source": self.bearing_source,
            "camera": self.camera.to_dict() if self.camera else None,
        }


def camera_bearing(
    detection: CameraObstacle,
    calibration: Optional[Calibration],
) -> tuple[Optional[float], Optional[float]]:
    """Bearing and angular width of the camera box, or ``(None, None)``.

    The box comes from the tracker's dark-blob pass. A frame that only trips
    the difference threshold has no box, so a new object that is *brighter*
    than the background still triggers but cannot be aimed.
    """
    if calibration is None or detection.box is None:
        return None, None
    if calibration.degrees_per_pixel == 0.0:
        return None, None
    centre = detection.centre_x
    if centre is None:
        return None, None
    _x, _y, width, _height = detection.box
    return calibration.pixel_to_angle(centre), abs(width * calibration.degrees_per_pixel)


def match_lidar(
    bearing_deg: float,
    obstacles: Sequence[Obstacle],
    tolerance_deg: float = DEFAULT_MATCH_TOLERANCE_DEG,
) -> tuple[Optional[Obstacle], Optional[float]]:
    """The obstacle whose bearing best explains the camera detection."""
    best: Optional[Obstacle] = None
    best_error: Optional[float] = None
    for obstacle in obstacles:
        error = angular_difference(obstacle.angle_deg, bearing_deg)
        if error > tolerance_deg + obstacle.half_width_deg:
            continue
        if best_error is None or error < best_error:
            best, best_error = obstacle, error
    return best, best_error


def fuse(
    detection: Optional[CameraObstacle],
    points: Sequence[tuple[float, float]],
    calibration: Optional[Calibration] = None,
    min_points: int = 3,
    tolerance_deg: float = DEFAULT_MATCH_TOLERANCE_DEG,
    sector_centre_deg: Optional[float] = None,
    sector_width_deg: Optional[float] = None,
) -> FusedDetection:
    """Combine one camera verdict with one LiDAR revolution.

    ``points`` are ``(angle_deg, distance_m)`` pairs. The sector, when given,
    limits which clusters may be reported on their own; a camera detection is
    matched against the whole scan, because the camera already restricts it.
    """
    obstacles = cluster_obstacles(points, calibration=calibration, min_points=min_points)
    in_sector = obstacles
    if sector_centre_deg is not None and sector_width_deg is not None and sector_width_deg < 360.0:
        half = sector_width_deg / 2.0
        in_sector = [
            obstacle
            for obstacle in obstacles
            if angular_difference(obstacle.angle_deg, sector_centre_deg) <= half + obstacle.half_width_deg
        ]

    if detection is not None and detection.triggered:
        bearing, width = camera_bearing(detection, calibration)
        if bearing is None:
            # Triggered without a box: report it, and offer the nearest cluster
            # inside the camera's own field as the best available range.
            nearest = in_sector[0] if in_sector else None
            return FusedDetection(
                agreement=AGREEMENT_BOTH if nearest else AGREEMENT_CAMERA_ONLY,
                camera=detection,
                bearing_deg=nearest.angle_deg if nearest else None,
                width_deg=nearest.width_deg if nearest else None,
                lidar=nearest,
                match_error_deg=None,
                bearing_source=SOURCE_LIDAR if nearest else None,
            )
        matched, error = match_lidar(bearing, obstacles, tolerance_deg)
        return FusedDetection(
            agreement=AGREEMENT_BOTH if matched else AGREEMENT_CAMERA_ONLY,
            camera=detection,
            bearing_deg=wrap180(bearing),
            width_deg=width,
            lidar=matched,
            match_error_deg=error,
            bearing_source=SOURCE_CAMERA,
        )

    if in_sector:
        nearest = in_sector[0]
        return FusedDetection(
            agreement=AGREEMENT_LIDAR_ONLY,
            camera=detection,
            bearing_deg=nearest.angle_deg,
            width_deg=nearest.width_deg,
            lidar=nearest,
            bearing_source=SOURCE_LIDAR,
        )
    return FusedDetection(agreement=AGREEMENT_CLEAR, camera=detection)
