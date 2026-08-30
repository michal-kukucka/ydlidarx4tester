"""Webcam capture for the LiDAR/camera twin, built on ffmpeg only.

The demo avoids PyPI dependencies. ``ffmpeg`` delivers uncompressed ``rgb24``
frames, which keeps latency low (no encoder buffering in the pipe) and lets the
frames be shown by Tk as PPM data and stored as PNG with a small pure-Python
encoder.
"""

from __future__ import annotations

import platform
import shutil
import struct
import subprocess
import threading
import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple


class CameraError(RuntimeError):
    """Raised when the camera cannot be opened or stops delivering frames."""


@dataclass(frozen=True)
class CameraFrame:
    stamp_ns: int
    rgb: bytes
    width: int
    height: int
    index: int

    def to_ppm(self) -> bytes:
        """Binary PPM, the format Tk's PhotoImage accepts without extras."""
        return b"P6\n%d %d\n255\n" % (self.width, self.height) + self.rgb

    def to_png(self, compression: int = 6) -> bytes:
        return encode_png(self.rgb, self.width, self.height, compression)


def encode_png(rgb: bytes, width: int, height: int, compression: int = 6) -> bytes:
    """Minimal PNG encoder for 8-bit RGB, filter type 0 on every row."""
    stride = width * 3
    raw = bytearray()
    for row in range(height):
        raw.append(0)
        raw += rgb[row * stride:(row + 1) * stride]

    def chunk(tag: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + tag
            + payload
            + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
        )

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(bytes(raw), compression))
        + chunk(b"IEND", b"")
    )


def _input_arguments(device: str, width: int, height: int, fps: int) -> list[str]:
    system = platform.system()
    if system == "Darwin":
        return [
            "-f", "avfoundation",
            "-framerate", str(fps),
            "-video_size", f"{width}x{height}",
            "-i", device,
        ]
    if system == "Linux":
        return [
            "-f", "v4l2",
            "-framerate", str(fps),
            "-video_size", f"{width}x{height}",
            "-i", device,
        ]
    if system == "Windows":
        return [
            "-f", "dshow",
            "-framerate", str(fps),
            "-video_size", f"{width}x{height}",
            "-i", f"video={device}",
        ]
    raise CameraError(f"unsupported platform for camera capture: {system}")


def list_cameras() -> Tuple[str, ...]:
    """Return the camera device names ffmpeg reports for this platform."""
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise CameraError("ffmpeg was not found on PATH")
    system = platform.system()
    if system == "Darwin":
        command = [ffmpeg, "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""]
    elif system == "Windows":
        command = [ffmpeg, "-hide_banner", "-f", "dshow", "-list_devices", "true", "-i", "dummy"]
    else:
        return tuple(sorted(str(path) for path in Path("/dev").glob("video*")))
    process = subprocess.run(command, capture_output=True, text=True)
    names = []
    for line in process.stderr.splitlines():
        if system == "Darwin":
            if "AVFoundation audio devices" in line:
                break
            if "AVFoundation video devices" in line:
                continue
            # "[AVFoundation indev @ 0x...] [0] HD Webcam C525"
            marker = line.find("] [")
            if marker < 0:
                continue
            tail = line[marker + 2:].strip()
            close = tail.find("]")
            if close > 0:
                names.append(tail[close + 1:].strip())
        elif '"' in line and "Alternative name" not in line:
            start = line.find('"')
            end = line.rfind('"')
            if end > start:
                names.append(line[start + 1:end])
    return tuple(names)


class CameraStream:
    """Keeps the newest raw RGB frame from a webcam available to other threads."""

    def __init__(
        self,
        device: str = "0",
        width: int = 640,
        height: int = 480,
        capture_fps: int = 30,
        output_fps: int = 10,
    ) -> None:
        self.device = device
        self.width = width
        self.height = height
        self.capture_fps = capture_fps
        self.output_fps = output_fps
        self.frame_bytes = width * height * 3
        self._process: Optional[subprocess.Popen[bytes]] = None
        self._reader: Optional[threading.Thread] = None
        self._stderr_reader: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._frame: Optional[CameraFrame] = None
        self._count = 0
        self._stop = threading.Event()
        self._stderr_tail: list[str] = []

    def open(self) -> "CameraStream":
        if self._process is not None:
            return self
        ffmpeg = shutil.which("ffmpeg")
        if not ffmpeg:
            raise CameraError("ffmpeg was not found on PATH; install it to use the camera twin")
        command = [ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin"]
        command += _input_arguments(self.device, self.width, self.height, self.capture_fps)
        command += [
            "-r", str(self.output_fps),
            "-pix_fmt", "rgb24",
            "-f", "rawvideo",
            "-fflags", "nobuffer",
            "-",
        ]
        self._process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        self._reader = threading.Thread(target=self._read_frames, name="camera-reader", daemon=True)
        self._reader.start()
        self._stderr_reader = threading.Thread(target=self._read_stderr, name="camera-stderr", daemon=True)
        self._stderr_reader.start()
        return self

    def _read_stderr(self) -> None:
        process = self._process
        if process is None or process.stderr is None:
            return
        for raw in iter(process.stderr.readline, b""):
            line = raw.decode("utf-8", "replace").strip()
            if line:
                self._stderr_tail = (self._stderr_tail + [line])[-10:]

    def _read_frames(self) -> None:
        process = self._process
        if process is None or process.stdout is None:
            return
        stream = process.stdout
        while not self._stop.is_set():
            buffer = bytearray()
            while len(buffer) < self.frame_bytes:
                chunk = stream.read(self.frame_bytes - len(buffer))
                if not chunk:
                    return
                buffer += chunk
            with self._lock:
                self._count += 1
                self._frame = CameraFrame(
                    stamp_ns=time.time_ns(),
                    rgb=bytes(buffer),
                    width=self.width,
                    height=self.height,
                    index=self._count,
                )

    def latest(self) -> Optional[CameraFrame]:
        with self._lock:
            return self._frame

    def wait_for_frame(self, timeout_s: float = 8.0) -> CameraFrame:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self.latest()
            if frame is not None:
                return frame
            if self._process is not None and self._process.poll() is not None:
                break
            time.sleep(0.05)
        detail = "; ".join(self._stderr_tail[-3:]) or "no frames arrived"
        raise CameraError(f"camera {self.device} produced no frames: {detail}")

    def close(self) -> None:
        self._stop.set()
        process = self._process
        self._process = None
        if process is not None:
            try:
                process.terminate()
                process.wait(timeout=3.0)
            except Exception:
                process.kill()
            for pipe in (process.stdout, process.stderr):
                if pipe is not None:
                    try:
                        pipe.close()
                    except Exception:
                        pass
        if self._reader is not None:
            self._reader.join(timeout=2.0)
            self._reader = None
        self._stderr_reader = None

    def __enter__(self) -> "CameraStream":
        return self.open()

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()
