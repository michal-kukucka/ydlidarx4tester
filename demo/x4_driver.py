"""Typed Python wrapper around the official YDLIDAR SDK C API.

The X4 profile is intentionally explicit: 128000 baud, triangle protocol,
5 kHz sample rate, dual-channel communication, and DTR motor control.
"""

from __future__ import annotations

import ctypes
import math
import os
import random
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple


YDLIDAR_TYPE_SERIAL = 0
TYPE_TRIANGLE = 1

LIDAR_PROP_SERIAL_PORT = 0
LIDAR_PROP_IGNORE_ARRAY = 1
LIDAR_PROP_SERIAL_BAUDRATE = 10
LIDAR_PROP_LIDAR_TYPE = 11
LIDAR_PROP_DEVICE_TYPE = 12
LIDAR_PROP_SAMPLE_RATE = 13
LIDAR_PROP_ABNORMAL_CHECK_COUNT = 14
LIDAR_PROP_INTENSITY_BIT = 15
LIDAR_PROP_MAX_RANGE = 20
LIDAR_PROP_MIN_RANGE = 21
LIDAR_PROP_MAX_ANGLE = 22
LIDAR_PROP_MIN_ANGLE = 23
LIDAR_PROP_SCAN_FREQUENCY = 24
LIDAR_PROP_FIXED_RESOLUTION = 30
LIDAR_PROP_REVERSION = 31
LIDAR_PROP_INVERTED = 32
LIDAR_PROP_AUTO_RECONNECT = 33
LIDAR_PROP_SINGLE_CHANNEL = 34
LIDAR_PROP_INTENSITY = 35
LIDAR_PROP_SUPPORT_MOTOR_DTR = 36
LIDAR_PROP_SUPPORT_HEARTBEAT = 37

X4_BAUDRATE = 128000
X4_SAMPLE_RATE_KHZ = 5
X4_MIN_RANGE_M = 0.12
X4_MAX_RANGE_M = 10.0
X4_MIN_FREQUENCY_HZ = 5.0
X4_MAX_FREQUENCY_HZ = 12.0


class LaserPoint(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("angle", ctypes.c_float),
        ("range", ctypes.c_float),
        ("intensity", ctypes.c_float),
    ]


class LaserConfig(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("min_angle", ctypes.c_float),
        ("max_angle", ctypes.c_float),
        ("angle_increment", ctypes.c_float),
        ("time_increment", ctypes.c_float),
        ("scan_time", ctypes.c_float),
        ("min_range", ctypes.c_float),
        ("max_range", ctypes.c_float),
    ]


class LaserFan(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("stamp", ctypes.c_uint64),
        ("npoints", ctypes.c_uint32),
        ("points", ctypes.POINTER(LaserPoint)),
        ("config", LaserConfig),
    ]


class LidarVersion(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("hardware", ctypes.c_uint8),
        ("soft_major", ctypes.c_uint8),
        ("soft_minor", ctypes.c_uint8),
        ("soft_patch", ctypes.c_uint8),
        ("sn", ctypes.c_uint8 * 16),
    ]


class String50(ctypes.Structure):
    _fields_ = [("data", ctypes.c_char * 50)]


class LidarPort(ctypes.Structure):
    _fields_ = [("port", String50 * 8)]


class YdlidarError(RuntimeError):
    """Raised when the native SDK or the X4 reports an error."""


@dataclass(frozen=True)
class LidarSettings:
    port: Optional[str] = None
    scan_frequency_hz: float = 8.0
    min_range_m: float = X4_MIN_RANGE_M
    max_range_m: float = X4_MAX_RANGE_M
    min_angle_deg: float = -180.0
    max_angle_deg: float = 180.0
    ignore_array: str = ""
    fixed_resolution: bool = False
    reversion: bool = False
    inverted: bool = False
    auto_reconnect: bool = True
    sun_filter: bool = False
    glass_filter: bool = False
    debug: bool = False

    def validate(self) -> None:
        if not X4_MIN_FREQUENCY_HZ <= self.scan_frequency_hz <= X4_MAX_FREQUENCY_HZ:
            raise ValueError("X4 scan frequency must be between 5 and 12 Hz")
        if self.min_range_m < X4_MIN_RANGE_M:
            raise ValueError("X4 minimum range cannot be below 0.12 m")
        if self.max_range_m > X4_MAX_RANGE_M or self.max_range_m <= self.min_range_m:
            raise ValueError("X4 range must satisfy min < max <= 10.0 m")
        if not -180.0 <= self.min_angle_deg < self.max_angle_deg <= 180.0:
            raise ValueError("angle window must satisfy -180 <= min < max <= 180 degrees")


@dataclass(frozen=True)
class ScanFrame:
    stamp_ns: int
    angles_rad: Tuple[float, ...]
    ranges_m: Tuple[float, ...]
    intensities: Tuple[float, ...]
    scan_frequency_hz: float
    angle_increment_rad: float
    time_increment_s: float

    @property
    def point_count(self) -> int:
        return len(self.ranges_m)


@dataclass(frozen=True)
class DeviceVersion:
    hardware: int
    firmware: str
    serial_number: str


class YdlidarSdk:
    """Loads and declares the ABI of ``ydlidar_sdk.dll``."""

    def __init__(self, library_path: Optional[os.PathLike[str] | str] = None):
        self.path = self._resolve_library(library_path)
        self._dll_directory = None
        if os.name == "nt" and hasattr(os, "add_dll_directory"):
            self._dll_directory = os.add_dll_directory(str(self.path.parent))
        try:
            self.lib = ctypes.CDLL(str(self.path))
        except OSError as exc:
            raise YdlidarError(
                f"Could not load {self.path}: {exc}. Run scripts\\setup.ps1 first."
            ) from exc
        self._declare_api()

    @staticmethod
    def _resolve_library(library_path: Optional[os.PathLike[str] | str]) -> Path:
        repo = Path(__file__).resolve().parents[1]
        supplied = Path(library_path).expanduser() if library_path else None
        env_path = os.environ.get("YDLIDAR_SDK_DLL")
        candidates = [
            supplied,
            Path(env_path).expanduser() if env_path else None,
            repo / "build-x4" / "ydlidar_sdk.dll",
            repo / "build" / "ydlidar_sdk.dll",
            repo / "build" / "Release" / "ydlidar_sdk.dll",
            repo / "build" / "libydlidar_sdk.dll",
            repo / "ydlidar_sdk.dll",
        ]
        for candidate in candidates:
            if candidate and candidate.is_file():
                return candidate.resolve()
        checked = "\n  ".join(str(p) for p in candidates if p)
        raise YdlidarError(
            "ydlidar_sdk.dll was not found. Run scripts\\setup.ps1 first. "
            f"Checked:\n  {checked}"
        )

    def _declare_api(self) -> None:
        lib = self.lib
        lib.lidarCreate.argtypes = []
        lib.lidarCreate.restype = ctypes.c_void_p
        lib.lidarDestroy.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        lib.lidarDestroy.restype = None
        lib.setlidaropt.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int]
        lib.setlidaropt.restype = ctypes.c_bool
        lib.initialize.argtypes = [ctypes.c_void_p]
        lib.initialize.restype = ctypes.c_bool
        lib.turnOn.argtypes = [ctypes.c_void_p]
        lib.turnOn.restype = ctypes.c_bool
        lib.doProcessSimple.argtypes = [ctypes.c_void_p, ctypes.POINTER(LaserFan)]
        lib.doProcessSimple.restype = ctypes.c_bool
        lib.turnOff.argtypes = [ctypes.c_void_p]
        lib.turnOff.restype = ctypes.c_bool
        lib.disconnecting.argtypes = [ctypes.c_void_p]
        lib.disconnecting.restype = None
        lib.DescribeError.argtypes = [ctypes.c_void_p]
        lib.DescribeError.restype = ctypes.c_char_p
        lib.GetSdkVersion.argtypes = [ctypes.c_char_p]
        lib.GetSdkVersion.restype = None
        lib.GetLidarVersion.argtypes = [ctypes.c_void_p, ctypes.POINTER(LidarVersion)]
        lib.GetLidarVersion.restype = None
        lib.LaserFanInit.argtypes = [ctypes.POINTER(LaserFan)]
        lib.LaserFanInit.restype = None
        lib.LaserFanDestroy.argtypes = [ctypes.POINTER(LaserFan)]
        lib.LaserFanDestroy.restype = None
        lib.lidarPortList.argtypes = [ctypes.POINTER(LidarPort)]
        lib.lidarPortList.restype = ctypes.c_int
        for name in (
            "enableSunNoise",
            "enableGlassNoise",
            "setBottomPriority",
            "setAutoIntensity",
            "setEnableDebug",
        ):
            function = getattr(lib, name)
            function.argtypes = [ctypes.c_void_p, ctypes.c_bool]
            function.restype = None
        lib.os_init.argtypes = []
        lib.os_init.restype = None
        lib.os_shutdown.argtypes = []
        lib.os_shutdown.restype = None

    @property
    def version(self) -> str:
        result = ctypes.create_string_buffer(64)
        self.lib.GetSdkVersion(result)
        return result.value.decode("ascii", errors="replace")

    def list_ports(self) -> Tuple[str, ...]:
        result = LidarPort()
        count = max(0, min(8, self.lib.lidarPortList(ctypes.byref(result))))
        ports = []
        for index in range(count):
            value = bytes(result.port[index].data).split(b"\0", 1)[0]
            if value:
                ports.append(value.decode(errors="replace"))
        return tuple(ports)


class X4Lidar:
    """Owns one live YDLIDAR X4 connection."""

    def __init__(self, sdk: YdlidarSdk, settings: LidarSettings):
        settings.validate()
        self.sdk = sdk
        self.settings = settings
        self.handle = ctypes.c_void_p()
        self.scan = LaserFan()
        self.started = False
        self.port: Optional[str] = None
        self.device_version: Optional[DeviceVersion] = None

    def _error(self, prefix: str) -> YdlidarError:
        raw = self.sdk.lib.DescribeError(self.handle)
        detail = raw.decode(errors="replace") if raw else "unknown SDK error"
        return YdlidarError(f"{prefix}: {detail}")

    def _set_bytes(self, prop: int, value: str) -> None:
        encoded = value.encode()
        buffer = ctypes.create_string_buffer(encoded)
        if not self.sdk.lib.setlidaropt(self.handle, prop, buffer, len(encoded)):
            raise self._error(f"could not set string property {prop}")

    def _set_scalar(self, prop: int, value: object, ctype: type) -> None:
        scalar = ctype(value)
        if not self.sdk.lib.setlidaropt(
            self.handle, prop, ctypes.byref(scalar), ctypes.sizeof(scalar)
        ):
            raise self._error(f"could not set property {prop}")

    def open(self) -> "X4Lidar":
        if self.handle.value:
            return self
        port = self.settings.port
        if not port:
            ports = self.sdk.list_ports()
            if not ports:
                raise YdlidarError(
                    "No serial ports found. Connect the X4 data USB and install the "
                    "Silicon Labs CP210x VCP driver, then retry."
                )
            if len(ports) != 1:
                joined = ", ".join(ports)
                raise YdlidarError(f"Multiple serial ports found ({joined}); pass --port COMx")
            port = ports[0]

        self.sdk.lib.os_init()
        self.handle = ctypes.c_void_p(self.sdk.lib.lidarCreate())
        if not self.handle.value:
            raise YdlidarError("the SDK could not allocate a LiDAR instance")
        self.sdk.lib.LaserFanInit(ctypes.byref(self.scan))
        self.port = port

        try:
            self._set_bytes(LIDAR_PROP_SERIAL_PORT, port)
            self._set_bytes(LIDAR_PROP_IGNORE_ARRAY, self.settings.ignore_array)
            for prop, value in (
                (LIDAR_PROP_SERIAL_BAUDRATE, X4_BAUDRATE),
                (LIDAR_PROP_LIDAR_TYPE, TYPE_TRIANGLE),
                (LIDAR_PROP_DEVICE_TYPE, YDLIDAR_TYPE_SERIAL),
                (LIDAR_PROP_SAMPLE_RATE, X4_SAMPLE_RATE_KHZ),
                (LIDAR_PROP_ABNORMAL_CHECK_COUNT, 4),
                # X4 packets contain distance only; treating a byte as
                # intensity shifts every following sample and breaks packet
                # checksums.
                (LIDAR_PROP_INTENSITY_BIT, 0),
            ):
                self._set_scalar(prop, value, ctypes.c_int)
            for prop, value in (
                (LIDAR_PROP_MIN_RANGE, self.settings.min_range_m),
                (LIDAR_PROP_MAX_RANGE, self.settings.max_range_m),
                (LIDAR_PROP_MIN_ANGLE, self.settings.min_angle_deg),
                (LIDAR_PROP_MAX_ANGLE, self.settings.max_angle_deg),
                (LIDAR_PROP_SCAN_FREQUENCY, self.settings.scan_frequency_hz),
            ):
                self._set_scalar(prop, value, ctypes.c_float)
            for prop, value in (
                (LIDAR_PROP_FIXED_RESOLUTION, self.settings.fixed_resolution),
                (LIDAR_PROP_REVERSION, self.settings.reversion),
                (LIDAR_PROP_INVERTED, self.settings.inverted),
                (LIDAR_PROP_AUTO_RECONNECT, self.settings.auto_reconnect),
                (LIDAR_PROP_SINGLE_CHANNEL, False),
                (LIDAR_PROP_INTENSITY, False),
                (LIDAR_PROP_SUPPORT_MOTOR_DTR, True),
                (LIDAR_PROP_SUPPORT_HEARTBEAT, False),
            ):
                self._set_scalar(prop, value, ctypes.c_bool)

            self.sdk.lib.enableSunNoise(self.handle, self.settings.sun_filter)
            self.sdk.lib.enableGlassNoise(self.handle, self.settings.glass_filter)
            self.sdk.lib.setBottomPriority(self.handle, True)
            self.sdk.lib.setAutoIntensity(self.handle, False)
            self.sdk.lib.setEnableDebug(self.handle, self.settings.debug)

            if not self.sdk.lib.initialize(self.handle):
                raise self._error(f"could not initialize X4 on {port}")
            version = LidarVersion()
            self.sdk.lib.GetLidarVersion(self.handle, ctypes.byref(version))
            serial_bytes = bytes(version.sn).rstrip(b"\0")
            if serial_bytes and all(32 <= value < 127 for value in serial_bytes):
                serial = serial_bytes.decode("ascii")
            else:
                serial = serial_bytes.hex().upper()
            self.device_version = DeviceVersion(
                hardware=int(version.hardware),
                firmware=f"{version.soft_major}.{version.soft_minor}.{version.soft_patch}",
                serial_number=serial or "unknown",
            )
            if not self.sdk.lib.turnOn(self.handle):
                raise self._error(f"could not start X4 motor/scan on {port}")
            self.started = True
            return self
        except BaseException:
            self.close()
            raise

    def read_scan(self) -> ScanFrame:
        if not self.started:
            raise YdlidarError("X4 is not open")
        if not self.sdk.lib.doProcessSimple(self.handle, ctypes.byref(self.scan)):
            raise self._error("failed to receive a complete scan")
        count = int(self.scan.npoints)
        if count <= 0 or not self.scan.points:
            raise YdlidarError("the SDK returned an empty scan")
        angles = tuple(float(self.scan.points[index].angle) for index in range(count))
        ranges = tuple(float(self.scan.points[index].range) for index in range(count))
        intensities = tuple(float(self.scan.points[index].intensity) for index in range(count))
        scan_time = float(self.scan.config.scan_time)
        return ScanFrame(
            stamp_ns=int(self.scan.stamp),
            angles_rad=angles,
            ranges_m=ranges,
            intensities=intensities,
            scan_frequency_hz=(1.0 / scan_time if scan_time > 0.0 else 0.0),
            angle_increment_rad=float(self.scan.config.angle_increment),
            time_increment_s=float(self.scan.config.time_increment),
        )

    def close(self) -> None:
        if self.handle.value:
            if self.started:
                self.sdk.lib.turnOff(self.handle)
                self.started = False
            self.sdk.lib.disconnecting(self.handle)
            self.sdk.lib.LaserFanDestroy(ctypes.byref(self.scan))
            self.sdk.lib.lidarDestroy(ctypes.byref(self.handle))
            self.handle = ctypes.c_void_p()

    def __enter__(self) -> "X4Lidar":
        return self.open()

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()


class SimulatedX4:
    """Deterministic 2-D room simulator for testing the complete UI offline."""

    def __init__(self, settings: LidarSettings, realtime: bool = True):
        settings.validate()
        self.settings = settings
        self.realtime = realtime
        self.port = "SIMULATED"
        self.device_version = DeviceVersion(1, "simulator", "SIM-X4")
        self._frame = 0
        self._next_frame = time.perf_counter()
        self._rng = random.Random(4)

    def open(self) -> "SimulatedX4":
        self._next_frame = time.perf_counter()
        return self

    @staticmethod
    def _circle_distance(
        angle: float, center_x: float, center_y: float, radius: float
    ) -> float:
        dx = math.sin(angle)
        dy = math.cos(angle)
        projection = dx * center_x + dy * center_y
        discriminant = projection * projection - (
            center_x * center_x + center_y * center_y - radius * radius
        )
        if discriminant < 0.0:
            return math.inf
        root = projection - math.sqrt(discriminant)
        return root if root > 0.0 else math.inf

    def read_scan(self) -> ScanFrame:
        period = 1.0 / self.settings.scan_frequency_hz
        if self.realtime:
            delay = self._next_frame - time.perf_counter()
            if delay > 0.0:
                time.sleep(delay)
        self._next_frame = max(self._next_frame + period, time.perf_counter())

        point_count = max(180, int(5000 / self.settings.scan_frequency_hz))
        phase = self._frame * 0.055
        minimum_angle = math.radians(self.settings.min_angle_deg)
        angle_step = math.radians(
            (self.settings.max_angle_deg - self.settings.min_angle_deg) / point_count
        )
        angles = []
        ranges = []
        for index in range(point_count):
            angle = minimum_angle + index * angle_step
            dx = math.sin(angle)
            dy = math.cos(angle)
            wall_x = 4.0 / abs(dx) if abs(dx) > 1e-9 else math.inf
            wall_y = 3.0 / abs(dy) if abs(dy) > 1e-9 else math.inf
            distance = min(
                wall_x,
                wall_y,
                self._circle_distance(angle, 1.20, 0.55, 0.35),
                self._circle_distance(angle, -1.10, 1.25, 0.28),
                self._circle_distance(angle, 1.6 * math.sin(phase), -1.1, 0.24),
            )
            distance += self._rng.gauss(0.0, 0.008)
            if self._rng.random() < 0.012 or not (
                self.settings.min_range_m <= distance <= self.settings.max_range_m
            ):
                distance = 0.0
            if self.settings.reversion:
                angle = (angle + 2.0 * math.pi) % (2.0 * math.pi) - math.pi
            if self.settings.inverted:
                angle = -angle
            angles.append(angle)
            ranges.append(distance)
        self._frame += 1
        return ScanFrame(
            stamp_ns=time.time_ns(),
            angles_rad=tuple(angles),
            ranges_m=tuple(ranges),
            intensities=(0.0,) * point_count,
            scan_frequency_hz=self.settings.scan_frequency_hz,
            angle_increment_rad=angle_step,
            time_increment_s=period / point_count,
        )

    def close(self) -> None:
        pass

    def __enter__(self) -> "SimulatedX4":
        return self.open()

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()


def open_source(
    settings: LidarSettings,
    simulate: bool = False,
    library_path: Optional[os.PathLike[str] | str] = None,
    realtime: bool = True,
) -> X4Lidar | SimulatedX4:
    if simulate:
        return SimulatedX4(settings, realtime=realtime)
    return X4Lidar(YdlidarSdk(library_path), settings)
