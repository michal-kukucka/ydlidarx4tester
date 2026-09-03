"""Camera obstacle detection through Rozeta's C ABI.

The detector itself is `rozeta::perception::RgbObstacleTracker`, the same
implementation the robot uses; this module only binds it with `ctypes` and
feeds it the packed rgb24 frames `camera_stream` already produces. Nothing is
reimplemented in Python, so a threshold tuned in the field applies here too.

`update_ref` is the "new object" mode: the tracker compares the live frame with
a reference background and trips when enough of the region of interest changed,
with a hysteresis of `trigger_streak` frames on and `clear_streak` frames off.
`update` is the reference-free fallback and only sees dark blobs.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from typing import Optional

from demo.camera_stream import CameraFrame
from demo.x4_driver import RozetaSdk, YdlidarError


CLEAR = 0
PENDING = 1
TRIGGERED = 2

STATE_NAMES = {CLEAR: "clear", PENDING: "pending", TRIGGERED: "triggered"}


class RgbObstacleError(RuntimeError):
    """Raised when the loaded Rozeta build cannot run the RGB detector."""


class RozetaRgbObstacleConfig(ctypes.Structure):
    _fields_ = [
        ("roi_left_fraction", ctypes.c_double),
        ("roi_right_fraction", ctypes.c_double),
        ("roi_top_fraction", ctypes.c_double),
        ("roi_bottom_fraction", ctypes.c_double),
        ("dark_max_value", ctypes.c_double),
        ("coverage_threshold", ctypes.c_double),
        ("diff_threshold", ctypes.c_double),
        ("diff_coverage_threshold", ctypes.c_double),
        ("min_obstacle_area_fraction", ctypes.c_double),
        ("max_obstacles", ctypes.c_int),
        ("trigger_streak", ctypes.c_int),
        ("clear_streak", ctypes.c_int),
    ]


class RozetaRgbObstacleResult(ctypes.Structure):
    _fields_ = [
        ("state", ctypes.c_int),
        ("dark_coverage", ctypes.c_double),
        ("diff_coverage", ctypes.c_double),
        ("obstacle_count", ctypes.c_int),
        ("largest_obstacle_area_fraction", ctypes.c_double),
        ("largest_obstacle_x", ctypes.c_int),
        ("largest_obstacle_y", ctypes.c_int),
        ("largest_obstacle_width", ctypes.c_int),
        ("largest_obstacle_height", ctypes.c_int),
        ("streak_count", ctypes.c_int),
        ("ok", ctypes.c_int),
        ("source", ctypes.c_char * 16),
    ]


@dataclass(frozen=True)
class CameraObstacle:
    """One tracker verdict, in the units the demo displays."""

    state: int
    dark_coverage: float
    diff_coverage: float
    obstacle_count: int
    area_fraction: float
    box: Optional[tuple[int, int, int, int]]
    streak: int
    source: str
    ok: bool

    @property
    def triggered(self) -> bool:
        return self.state == TRIGGERED

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.state, "unknown")

    @property
    def centre_x(self) -> Optional[float]:
        """Horizontal centre of the detection, in pixels."""
        if self.box is None:
            return None
        x, _y, width, _height = self.box
        return x + width / 2.0

    def to_dict(self) -> dict:
        return {
            "state": self.state_name,
            "dark_coverage": round(self.dark_coverage, 4),
            "diff_coverage": round(self.diff_coverage, 4),
            "obstacle_count": self.obstacle_count,
            "area_fraction": round(self.area_fraction, 4),
            "box": list(self.box) if self.box else None,
            "streak": self.streak,
            "source": self.source,
        }


def _declare(lib: ctypes.CDLL) -> None:
    if getattr(lib, "_rozeta_rgb_obstacle_declared", False):
        return
    try:
        lib.rozeta_rgb_obstacle_default_config.argtypes = []
    except AttributeError as error:
        raise RgbObstacleError(
            "This Rozeta build has no RGB obstacle C ABI "
            "(rozeta_rgb_obstacle_tracker_create is missing). Rebuild the "
            "sibling Rozeta checkout, then rerun."
        ) from error
    lib.rozeta_rgb_obstacle_default_config.restype = RozetaRgbObstacleConfig
    lib.rozeta_rgb_obstacle_tracker_create.argtypes = [RozetaRgbObstacleConfig]
    lib.rozeta_rgb_obstacle_tracker_create.restype = ctypes.c_void_p
    lib.rozeta_rgb_obstacle_tracker_destroy.argtypes = [ctypes.c_void_p]
    lib.rozeta_rgb_obstacle_tracker_destroy.restype = None
    lib.rozeta_rgb_obstacle_tracker_reset.argtypes = [ctypes.c_void_p]
    lib.rozeta_rgb_obstacle_tracker_reset.restype = None
    lib.rozeta_rgb_obstacle_tracker_update.argtypes = [
        ctypes.c_void_p,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_int,
    ]
    lib.rozeta_rgb_obstacle_tracker_update.restype = ctypes.c_int
    lib.rozeta_rgb_obstacle_tracker_update_ref.argtypes = [
        ctypes.c_void_p,
        ctypes.c_char_p,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_int,
    ]
    lib.rozeta_rgb_obstacle_tracker_update_ref.restype = ctypes.c_int
    lib.rozeta_rgb_obstacle_tracker_result.argtypes = [ctypes.c_void_p]
    lib.rozeta_rgb_obstacle_tracker_result.restype = RozetaRgbObstacleResult
    lib._rozeta_rgb_obstacle_declared = True


def _check_size(frame: CameraFrame) -> None:
    """The C side reads width * height * 3 bytes and trusts the caller."""
    expected = frame.width * frame.height * 3
    if frame.width <= 0 or frame.height <= 0 or len(frame.rgb) != expected:
        raise RgbObstacleError(
            f"frame carries {len(frame.rgb)} bytes, expected {expected} "
            f"for {frame.width}x{frame.height} rgb24"
        )


def default_config(sdk: RozetaSdk) -> RozetaRgbObstacleConfig:
    _declare(sdk.lib)
    return sdk.lib.rozeta_rgb_obstacle_default_config()


class RgbObstacleTracker:
    """Owns one native tracker handle."""

    def __init__(
        self,
        sdk: RozetaSdk,
        config: Optional[RozetaRgbObstacleConfig] = None,
        **overrides: float,
    ) -> None:
        self.sdk = sdk
        _declare(sdk.lib)
        self.config = config if config is not None else default_config(sdk)
        for name, value in overrides.items():
            if value is None:
                continue
            if not hasattr(self.config, name):
                raise RgbObstacleError(f"unknown RGB obstacle setting: {name}")
            setattr(self.config, name, value)
        handle = sdk.lib.rozeta_rgb_obstacle_tracker_create(self.config)
        if not handle:
            raise RgbObstacleError(
                "Rozeta rejected the RGB obstacle configuration; every fraction "
                "must be within 0..1 and the streaks at least 1."
            )
        self.handle = ctypes.c_void_p(handle)
        self._result = CameraObstacle(
            state=CLEAR,
            dark_coverage=0.0,
            diff_coverage=-1.0,
            obstacle_count=0,
            area_fraction=0.0,
            box=None,
            streak=0,
            source="none",
            ok=True,
        )

    @classmethod
    def open(
        cls,
        library_path: Optional[str] = None,
        **overrides: float,
    ) -> "RgbObstacleTracker":
        """Loads the library itself, for callers with no LiDAR of their own."""
        try:
            sdk = RozetaSdk(library_path)
        except YdlidarError as error:
            raise RgbObstacleError(str(error)) from error
        return cls(sdk, **overrides)

    def update(self, frame: CameraFrame) -> CameraObstacle:
        """Dark-blob detection only, for a run with no reference background."""
        _check_size(frame)
        status = self.sdk.lib.rozeta_rgb_obstacle_tracker_update(
            self.handle, frame.rgb, frame.width, frame.height
        )
        return self._collect(status)

    def update_ref(self, frame: CameraFrame, reference: CameraFrame) -> CameraObstacle:
        """New-object detection: the live frame against a reference background."""
        if (frame.width, frame.height) != (reference.width, reference.height):
            raise RgbObstacleError("frame and reference must have the same size")
        _check_size(frame)
        _check_size(reference)
        status = self.sdk.lib.rozeta_rgb_obstacle_tracker_update_ref(
            self.handle, frame.rgb, reference.rgb, frame.width, frame.height
        )
        return self._collect(status)

    def reset(self) -> None:
        self.sdk.lib.rozeta_rgb_obstacle_tracker_reset(self.handle)
        self._result = self._collect(0)

    @property
    def result(self) -> CameraObstacle:
        return self._result

    def _collect(self, status: int) -> CameraObstacle:
        if status != 0:
            raise RgbObstacleError("the RGB obstacle tracker rejected the frame")
        raw = self.sdk.lib.rozeta_rgb_obstacle_tracker_result(self.handle)
        box = None
        if raw.largest_obstacle_width > 0 and raw.largest_obstacle_height > 0:
            box = (
                raw.largest_obstacle_x,
                raw.largest_obstacle_y,
                raw.largest_obstacle_width,
                raw.largest_obstacle_height,
            )
        self._result = CameraObstacle(
            state=int(raw.state),
            dark_coverage=float(raw.dark_coverage),
            diff_coverage=float(raw.diff_coverage),
            obstacle_count=int(raw.obstacle_count),
            area_fraction=float(raw.largest_obstacle_area_fraction),
            box=box,
            streak=int(raw.streak_count),
            source=raw.source.decode(errors="replace"),
            ok=bool(raw.ok),
        )
        return self._result

    def close(self) -> None:
        handle = getattr(self, "handle", None)
        if handle:
            self.sdk.lib.rozeta_rgb_obstacle_tracker_destroy(handle)
            self.handle = ctypes.c_void_p(None)

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def __enter__(self) -> "RgbObstacleTracker":
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()
