"""Synchronized YDLIDAR X4 + webcam capture ("twin mode").

Records time-aligned camera frames and LiDAR revolutions into a session
directory so the LiDAR bearing scale can be calibrated against what the camera
actually sees. Optionally shows both live side by side.

Session layout::

    <session>/session.json      run metadata
    <session>/frames/*.png      camera frames, one per recorded sample
    <session>/samples.jsonl     one JSON object per recorded sample
    <session>/labels.jsonl      operator annotations (GUI "Mark" button)

Each sample carries the full scan as ``points`` ([angle_deg, distance_m]),
15-degree sector minima, and the camera/LiDAR timestamp difference.
"""

from __future__ import annotations

import argparse
import json
import shutil
import math
import statistics
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from demo.calibration import Calibration, find_calibration, nearest_obstacle
from demo.camera_stream import CameraError, CameraFrame, CameraStream, list_cameras
from demo.fusion import FusedDetection, fuse
from demo.rgb_obstacle import CameraObstacle, RgbObstacleError, RgbObstacleTracker
from demo.x4_driver import LidarSettings, ScanFrame, YdlidarError, open_source

SECTOR_WIDTH_DEG = 15.0


@dataclass
class Sample:
    index: int
    scan: ScanFrame
    frame: CameraFrame
    fusion: Optional[FusedDetection] = None


def sector_summary(angles_deg: list[float], ranges_m: list[float]) -> list[dict]:
    buckets: dict[int, list[float]] = {}
    for angle, distance in zip(angles_deg, ranges_m):
        if distance <= 0.0:
            continue
        key = int(math.floor(angle / SECTOR_WIDTH_DEG))
        buckets.setdefault(key, []).append(distance)
    summary = []
    for key in sorted(buckets):
        values = buckets[key]
        summary.append(
            {
                "start_deg": key * SECTOR_WIDTH_DEG,
                "end_deg": (key + 1) * SECTOR_WIDTH_DEG,
                "count": len(values),
                "min_m": round(min(values), 4),
                "median_m": round(statistics.median(values), 4),
            }
        )
    return summary


def scan_arrays(scan: ScanFrame) -> tuple[list[float], list[float]]:
    angles = [math.degrees(value) for value in scan.angles_rad]
    return angles, list(scan.ranges_m)


def nearest_point(
    angles_deg: list[float],
    ranges_m: list[float],
    calibration: Optional[Calibration] = None,
    min_points: int = 3,
) -> Optional[dict]:
    """Nearest clustered obstacle, with mount returns removed when calibrated."""
    obstacle = nearest_obstacle(
        list(zip(angles_deg, ranges_m)),
        calibration=calibration,
        min_points=min_points,
    )
    if obstacle is None:
        return None
    return {
        "angle_deg": round(obstacle.angle_deg, 2),
        "distance_m": round(obstacle.distance_m, 4),
        "width_deg": round(obstacle.width_deg, 1),
        "points": obstacle.points,
    }


def _raw_nearest(angles_deg: list[float], ranges_m: list[float]) -> Optional[dict]:
    """Unfiltered closest return, kept so a session can be re-calibrated later."""
    best = None
    for angle, distance in zip(angles_deg, ranges_m):
        if distance > 0.0 and (best is None or distance < best[1]):
            best = (angle, distance)
    if best is None:
        return None
    return {"angle_deg": round(best[0], 2), "distance_m": round(best[1], 4)}


class SessionWriter:
    """Writes frames and sample metadata into a session directory."""

    def __init__(
        self,
        directory: Path,
        metadata: dict,
        calibration: Optional[Calibration] = None,
        budget_bytes: int = 0,
        min_free_bytes: int = 512 * 1024 * 1024,
    ) -> None:
        self.calibration = calibration
        self.budget_bytes = budget_bytes
        self.min_free_bytes = min_free_bytes
        self.bytes_written = 0
        self.stopped_reason: Optional[str] = None
        self.directory = directory
        self.frames_dir = directory / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.samples_path = directory / "samples.jsonl"
        self.labels_path = directory / "labels.jsonl"
        (directory / "session.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        self._lock = threading.Lock()
        self.count = 0

    def has_room(self) -> bool:
        """Refuse to fill the disk: a long unattended session must not run it dry."""
        if self.stopped_reason is not None:
            return False
        if self.budget_bytes and self.bytes_written >= self.budget_bytes:
            self.stopped_reason = f"session budget of {self.budget_bytes // (1024 * 1024)} MB reached"
            return False
        try:
            free = shutil.disk_usage(self.directory).free
        except OSError:
            return True
        if free < self.min_free_bytes:
            self.stopped_reason = f"only {free // (1024 * 1024)} MB free on the session volume"
            return False
        return True

    def write_sample(self, sample: Sample) -> dict:
        angles, ranges = scan_arrays(sample.scan)
        frame_name = f"frame_{sample.index:05d}.png"
        # Level 1 keeps the per-sample encode near 60 ms; the files stay readable.
        png = sample.frame.to_png(compression=1)
        try:
            (self.frames_dir / frame_name).write_bytes(png)
        except OSError as error:
            self.stopped_reason = f"could not write {frame_name}: {error}"
            return summarize(sample, self.calibration)
        self.bytes_written += len(png)
        record = {
            "index": sample.index,
            "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "scan_stamp_ns": sample.scan.stamp_ns,
            "camera_stamp_ns": sample.frame.stamp_ns,
            "camera_frame_index": sample.frame.index,
            "sync_delta_ms": round((sample.scan.stamp_ns - sample.frame.stamp_ns) / 1e6, 2),
            "scan_frequency_hz": round(sample.scan.scan_frequency_hz, 3),
            "point_count": sample.scan.point_count,
            "frame": f"frames/{frame_name}",
            "nearest": nearest_point(angles, ranges, self.calibration),
            "nearest_raw": _raw_nearest(angles, ranges),
            "sectors": sector_summary(angles, ranges),
            "fusion": sample.fusion.to_dict() if sample.fusion else None,
            "points": [[round(a, 3), round(d, 4)] for a, d in zip(angles, ranges) if d > 0.0],
        }
        line = json.dumps(record) + "\n"
        with self._lock:
            try:
                with self.samples_path.open("a", encoding="utf-8") as handle:
                    handle.write(line)
            except OSError as error:
                self.stopped_reason = f"could not append to samples.jsonl: {error}"
                return record
            self.count += 1
            self.bytes_written += len(line)
        return record

    def write_label(self, index: int, text: str) -> None:
        record = {
            "sample_index": index,
            "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "stamp_ns": time.time_ns(),
            "text": text,
        }
        with self._lock:
            with self.labels_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record) + "\n")


class MotionGate:
    """Decides whether a sample is worth storing.

    A session left running while nobody is there produced 2.7 GB of identical
    frames and filled the disk. Idle samples are still summarised and shown,
    they are just not written, apart from a periodic keepalive so the session
    still records that the scene was quiet.
    """

    def __init__(
        self,
        luma_delta: float,
        range_delta_m: float,
        keepalive_s: float,
        min_cells: int = 12,
        min_bins: int = 2,
    ) -> None:
        self.luma_delta = luma_delta
        self.range_delta_m = range_delta_m
        self.keepalive_s = keepalive_s
        # Webcam grain and X4 edge jitter both trip a single-cell test, so a
        # sample only counts as motion when several places change at once.
        self.min_cells = min_cells
        self.min_bins = min_bins
        self._luma: Optional[list[int]] = None
        self._ranges: Optional[dict[int, float]] = None
        self._last_write = 0.0
        self.skipped = 0

    @staticmethod
    def _luma_signature(frame: CameraFrame, cells: int = 24) -> list[int]:
        """Mean luma of a coarse grid, cheap enough to run every sample."""
        step_x = max(frame.width // cells, 1)
        step_y = max(frame.height // cells, 1)
        signature = []
        stride = frame.width * 3
        for y in range(0, frame.height, step_y):
            for x in range(0, frame.width, step_x):
                index = y * stride + x * 3
                signature.append(
                    (frame.rgb[index] * 299 + frame.rgb[index + 1] * 587 + frame.rgb[index + 2] * 114) // 1000
                )
        return signature

    @staticmethod
    def _range_signature(scan: ScanFrame) -> dict[int, float]:
        minima: dict[int, float] = {}
        for angle_rad, distance in zip(scan.angles_rad, scan.ranges_m):
            if distance <= 0.0:
                continue
            key = int(math.degrees(angle_rad) // SECTOR_WIDTH_DEG)
            if key not in minima or distance < minima[key]:
                minima[key] = distance
        return minima

    def should_write(self, sample: Sample, now: float) -> bool:
        luma = self._luma_signature(sample.frame)
        ranges = self._range_signature(sample.scan)
        moved = True
        if self._luma is not None and self._ranges is not None and len(luma) == len(self._luma):
            changed_cells = sum(1 for a, b in zip(luma, self._luma) if abs(a - b) >= self.luma_delta)
            shared = set(ranges) & set(self._ranges)
            changed_bins = sum(
                1 for key in shared if abs(ranges[key] - self._ranges[key]) >= self.range_delta_m
            )
            moved = changed_cells >= self.min_cells or changed_bins >= self.min_bins
        keepalive = self.keepalive_s > 0.0 and (now - self._last_write) >= self.keepalive_s
        if moved or keepalive:
            self._luma = luma
            self._ranges = ranges
            self._last_write = now
            return True
        self._luma = luma
        self.skipped += 1
        return False


def summarize(sample: Sample, calibration: Optional[Calibration] = None) -> dict:
    """The metadata a recorded sample would carry, without writing anything."""
    angles, ranges = scan_arrays(sample.scan)
    return {
        "index": sample.index,
        "scan_stamp_ns": sample.scan.stamp_ns,
        "camera_stamp_ns": sample.frame.stamp_ns,
        "sync_delta_ms": round((sample.scan.stamp_ns - sample.frame.stamp_ns) / 1e6, 2),
        "scan_frequency_hz": round(sample.scan.scan_frequency_hz, 3),
        "point_count": sample.scan.point_count,
        "nearest": nearest_point(angles, ranges, calibration),
        "fusion": sample.fusion.to_dict() if sample.fusion else None,
    }


class LidarStream:
    """Reads scans in a background thread and keeps the newest one."""

    def __init__(self, source) -> None:
        self.source = source
        self._lock = threading.Lock()
        self._scan: Optional[ScanFrame] = None
        self._error: Optional[BaseException] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="lidar-reader", daemon=True)

    def start(self) -> "LidarStream":
        self._thread.start()
        return self

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                scan = self.source.read_scan()
            except YdlidarError as error:
                self._error = error
                time.sleep(0.05)
                continue
            except BaseException as error:  # pragma: no cover - defensive
                self._error = error
                return
            with self._lock:
                self._scan = scan
                self._error = None

    def latest(self) -> Optional[ScanFrame]:
        with self._lock:
            return self._scan

    @property
    def error(self) -> Optional[BaseException]:
        return self._error

    def wait_for_scan(self, timeout_s: float = 10.0) -> ScanFrame:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            scan = self.latest()
            if scan is not None:
                return scan
            time.sleep(0.05)
        raise YdlidarError(f"no LiDAR scan within {timeout_s:.0f} s: {self._error}")

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)


class CameraDetector:
    """Feeds camera frames to Rozeta's tracker at the rate they arrive.

    The tracker's hysteresis counts frames, not seconds, so it has to see every
    decoded frame and not only the recorded samples: at the default 2 samples
    per second a five-frame trigger would otherwise take two and a half seconds.
    """

    def __init__(
        self,
        tracker: RgbObstacleTracker,
        use_reference: bool = True,
        settle_s: float = 2.0,
    ) -> None:
        self.tracker = tracker
        self.use_reference = use_reference
        self.settle_s = settle_s
        self.reference: Optional[CameraFrame] = None
        self.reference_index: Optional[int] = None
        self._deadline = time.monotonic() + settle_s
        self._last_index: Optional[int] = None
        self.detection: Optional[CameraObstacle] = None

    def capture_reference(self, frame: CameraFrame) -> None:
        """Adopts this frame as the empty scene every later frame is judged against."""
        self.reference = frame
        self.reference_index = frame.index
        self.tracker.reset()
        self.detection = None

    def observe(self, frame: CameraFrame) -> Optional[CameraObstacle]:
        """Runs the tracker once per new frame; repeats are ignored."""
        if frame.index == self._last_index:
            return self.detection
        self._last_index = frame.index
        if self.use_reference and self.reference is None:
            if time.monotonic() < self._deadline:
                return None
            self.capture_reference(frame)
            return None
        if self.reference is not None:
            self.detection = self.tracker.update_ref(frame, self.reference)
        else:
            self.detection = self.tracker.update(frame)
        return self.detection

    @property
    def waiting(self) -> bool:
        return self.use_reference and self.reference is None

    def close(self) -> None:
        self.tracker.close()


class TwinWindow:
    """Tk view: camera on the left, polar LiDAR plot on the right."""

    def __init__(
        self,
        args: argparse.Namespace,
        writer: Optional[SessionWriter],
        calibration: Optional[Calibration] = None,
        detector: Optional["CameraDetector"] = None,
    ) -> None:
        import tkinter as tk

        self.tk = tk
        self.args = args
        self.writer = writer
        self.calibration = calibration
        self.detector = detector
        self.root = tk.Tk()
        self.root.title("YDLIDAR X4 + camera twin")
        self.root.configure(bg="#101014")
        self.image_size = (args.width, args.height)
        self.canvas_camera = tk.Canvas(
            self.root, width=args.width, height=args.height, bg="#000000", highlightthickness=0
        )
        self.canvas_camera.grid(row=0, column=0, padx=8, pady=8)
        self.plot_size = max(args.height, 480)
        self.canvas_lidar = tk.Canvas(
            self.root, width=self.plot_size, height=self.plot_size, bg="#08080c", highlightthickness=0
        )
        self.canvas_lidar.grid(row=0, column=1, padx=8, pady=8)
        self.status = tk.StringVar(value="starting")
        tk.Label(
            self.root, textvariable=self.status, fg="#d8d8e0", bg="#101014", anchor="w", font=("TkDefaultFont", 12)
        ).grid(row=1, column=0, columnspan=2, sticky="we", padx=10)
        controls = tk.Frame(self.root, bg="#101014")
        controls.grid(row=2, column=0, columnspan=2, sticky="we", padx=8, pady=6)
        tk.Label(controls, text="Label", fg="#d8d8e0", bg="#101014").pack(side="left")
        self.label_entry = tk.Entry(controls, width=48)
        self.label_entry.pack(side="left", padx=6)
        self.label_entry.bind("<Return>", lambda _event: self.mark())
        tk.Button(controls, text="Mark", command=self.mark).pack(side="left")
        self.recording = tk.BooleanVar(value=not args.no_record)
        tk.Checkbutton(
            controls,
            text="Record",
            variable=self.recording,
            fg="#d8d8e0",
            bg="#101014",
            selectcolor="#202028",
            activebackground="#101014",
            activeforeground="#d8d8e0",
        ).pack(side="left", padx=10)
        if detector is not None and detector.use_reference:
            tk.Button(controls, text="Reference", command=self.retake_reference).pack(side="left")
        self._photo = None
        self.last_sample_index = 0
        self._frame = None

    def retake_reference(self) -> None:
        """Adopts the current view as empty: use it after the scene settles."""
        if self.detector is None or self._frame is None:
            return
        self.detector.capture_reference(self._frame)
        self.status.set(f"reference taken from frame {self._frame.index}")

    def mark(self) -> None:
        text = self.label_entry.get().strip()
        if not text or self.writer is None:
            return
        self.writer.write_label(self.last_sample_index, text)
        self.label_entry.delete(0, self.tk.END)
        self.status.set(f"labelled sample {self.last_sample_index}: {text}")

    def show_camera(self, frame: CameraFrame, detection: Optional[CameraObstacle] = None) -> None:
        # Level 0 costs ~7 ms per frame; Tk only needs a container, not small files.
        self._frame = frame
        self._photo = self.tk.PhotoImage(data=frame.to_png(compression=0))
        self.canvas_camera.delete("all")
        self.canvas_camera.create_image(0, 0, image=self._photo, anchor="nw")
        if detection is None or detection.box is None:
            return
        x, y, width, height = detection.box
        colour = "#ff6b4a" if detection.triggered else "#e0c04a"
        self.canvas_camera.create_rectangle(x, y, x + width, y + height, outline=colour, width=2)
        self.canvas_camera.create_text(
            x + 2,
            max(8, y - 8),
            text=f"{detection.state_name} {detection.area_fraction * 100:.1f}%",
            fill=colour,
            anchor="w",
        )

    def show_scan(self, scan: ScanFrame, fused: Optional[FusedDetection] = None) -> None:
        canvas = self.canvas_lidar
        canvas.delete("all")
        size = self.plot_size
        cx = cy = size / 2.0
        radius = size / 2.0 - 16.0
        scale = radius / self.args.plot_range
        for ring in range(1, int(self.args.plot_range) + 1):
            r = ring * scale
            canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#22303a")
            canvas.create_text(cx + r - 12, cy - 8, text=f"{ring}m", fill="#3c5060", anchor="e")
        canvas.create_line(cx - radius, cy, cx + radius, cy, fill="#22303a")
        canvas.create_line(cx, cy - radius, cx, cy + radius, fill="#22303a")
        canvas.create_text(cx, cy - radius + 10, text="0deg", fill="#4c6070")
        canvas.create_text(cx + radius - 24, cy, text="+90", fill="#4c6070")
        canvas.create_text(cx - radius + 24, cy, text="-90", fill="#4c6070")
        half = self.args.field_of_view / 2.0
        start = self.args.forward_angle - half
        canvas.create_arc(
            cx - radius,
            cy - radius,
            cx + radius,
            cy + radius,
            start=90.0 - (start + self.args.field_of_view),
            extent=self.args.field_of_view,
            outline="#2f6f4f",
            fill="#12291f",
            style="pieslice",
        )
        if self.calibration is not None:
            for sector in self.calibration.blind_sectors:
                width = (sector.end_deg - sector.start_deg) % 360.0
                canvas.create_arc(
                    cx - radius,
                    cy - radius,
                    cx + radius,
                    cy + radius,
                    start=90.0 - (sector.start_deg + width),
                    extent=width,
                    outline="",
                    fill="#2a1c1c",
                    style="pieslice",
                )
        for angle_rad, distance in zip(scan.angles_rad, scan.ranges_m):
            if distance <= 0.0 or distance > self.args.plot_range:
                continue
            angle_deg = math.degrees(angle_rad)
            if self.calibration is not None and self.calibration.is_blind(angle_deg):
                canvas.create_oval(
                    cx + math.sin(angle_rad) * distance * scale - 1,
                    cy - math.cos(angle_rad) * distance * scale - 1,
                    cx + math.sin(angle_rad) * distance * scale + 1,
                    cy - math.cos(angle_rad) * distance * scale + 1,
                    outline="",
                    fill="#6b3b3b",
                )
                continue
            inside = abs(((angle_deg - self.args.forward_angle + 180.0) % 360.0) - 180.0) <= half
            x = cx + math.sin(angle_rad) * distance * scale
            y = cy - math.cos(angle_rad) * distance * scale
            colour = "#7fe0a0" if inside else "#4a5a66"
            canvas.create_oval(x - 1.5, y - 1.5, x + 1.5, y + 1.5, outline="", fill=colour)
        if fused is not None:
            self._draw_fusion(canvas, cx, cy, radius, scale, fused)
        canvas.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill="#e05050", outline="")

    def _draw_fusion(self, canvas, cx: float, cy: float, radius: float, scale: float, fused) -> None:
        """A ray along the camera bearing, and a ring on the matched cluster."""
        from demo.fusion import AGREEMENT_BOTH, AGREEMENT_CAMERA_ONLY

        colours = {AGREEMENT_BOTH: "#ff5c3a", AGREEMENT_CAMERA_ONLY: "#ffc14a"}
        colour = colours.get(fused.agreement)
        if colour is None or fused.bearing_deg is None:
            return
        bearing = math.radians(fused.bearing_deg)
        canvas.create_line(
            cx,
            cy,
            cx + math.sin(bearing) * radius,
            cy - math.cos(bearing) * radius,
            fill=colour,
            dash=(4, 4),
        )
        if fused.width_deg:
            half = fused.width_deg / 2.0
            canvas.create_arc(
                cx - radius,
                cy - radius,
                cx + radius,
                cy + radius,
                start=90.0 - (fused.bearing_deg + half),
                extent=fused.width_deg,
                outline=colour,
                style="arc",
            )
        if fused.lidar is not None and fused.lidar.distance_m <= self.args.plot_range:
            r = fused.lidar.distance_m * scale
            angle = math.radians(fused.lidar.angle_deg)
            x = cx + math.sin(angle) * r
            y = cy - math.cos(angle) * r
            canvas.create_oval(x - 7, y - 7, x + 7, y + 7, outline=colour, width=2)
            canvas.create_text(
                x + 10, y, text=f"{fused.lidar.distance_m:.2f}m", fill=colour, anchor="w"
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Synchronized YDLIDAR X4 + camera capture")
    parser.add_argument("--port", default=None, help="serial port of the X4 data USB")
    parser.add_argument("--simulate", action="store_true", help="use the built-in LiDAR simulator")
    parser.add_argument("--rozeta-lib", default=None, help="path to librozeta.dylib/.so/.dll")
    parser.add_argument("--camera", default="0", help="camera device (avfoundation index, /dev/videoN, dshow name)")
    parser.add_argument("--list-cameras", action="store_true", help="list camera devices and exit")
    parser.add_argument("--width", type=int, default=640, help="camera frame width")
    parser.add_argument("--height", type=int, default=480, help="camera frame height")
    parser.add_argument("--camera-fps", type=int, default=30, help="camera capture rate requested from the device")
    parser.add_argument("--stream-fps", type=int, default=10, help="decoded frame rate delivered to the demo")
    parser.add_argument("--session", default=None, help="session directory (default recordings/twin_<timestamp>)")
    parser.add_argument("--rate", type=float, default=2.0, help="recorded samples per second")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds; 0 runs until closed")
    parser.add_argument("--samples", type=int, default=0, help="stop after N recorded samples; 0 means unlimited")
    parser.add_argument("--no-record", action="store_true", help="show the twin without writing a session")
    parser.add_argument(
        "--max-session-mb",
        type=int,
        default=512,
        help="stop recording once the session reaches this size; 0 removes the limit",
    )
    parser.add_argument(
        "--min-free-mb",
        type=int,
        default=512,
        help="stop recording when the volume has less free space than this",
    )
    parser.add_argument(
        "--record-idle",
        action="store_true",
        help="store every sample, even when neither camera nor LiDAR changed",
    )
    parser.add_argument("--motion-luma", type=float, default=18.0, help="luma step that counts as camera motion")
    parser.add_argument("--motion-range", type=float, default=0.15, help="range step in metres that counts as motion")
    parser.add_argument("--motion-cells", type=int, default=12, help="changed image cells needed to call it motion")
    parser.add_argument("--motion-bins", type=int, default=2, help="changed LiDAR sectors needed to call it motion")
    parser.add_argument(
        "--keepalive",
        type=float,
        default=30.0,
        help="store an idle sample at least this often in seconds; 0 disables",
    )
    parser.add_argument(
        "--min-points",
        type=int,
        default=200,
        help="ignore partial revolutions with fewer points (motor spin-up)",
    )
    parser.add_argument("--headless", action="store_true", help="record without the GUI")
    parser.add_argument(
        "--forward-angle",
        type=float,
        default=None,
        help="centre of the highlighted sector (default: the calibrated camera axis)",
    )
    parser.add_argument("--calibration", default=None, help="calibration.json to load (default: newest recorded)")
    parser.add_argument("--no-calibration", action="store_true", help="ignore any recorded calibration")
    parser.add_argument(
        "--field-of-view",
        type=float,
        default=None,
        help="width of the highlighted sector (default: the calibrated camera FOV)",
    )
    parser.add_argument(
        "--no-camera-detection",
        action="store_true",
        help="skip Rozeta's RGB obstacle tracker and show the LiDAR alone",
    )
    parser.add_argument(
        "--no-reference",
        action="store_true",
        help="detect dark blobs only, instead of new objects against a background",
    )
    parser.add_argument(
        "--reference-delay",
        type=float,
        default=2.0,
        help="seconds of settling before the background reference is captured",
    )
    parser.add_argument(
        "--diff-threshold",
        type=float,
        default=None,
        help="per-channel step that counts as changed against the reference (0-255)",
    )
    parser.add_argument(
        "--diff-coverage",
        type=float,
        default=None,
        help="fraction of the region of interest that must change to call it an obstacle",
    )
    parser.add_argument(
        "--dark-coverage",
        type=float,
        default=None,
        help=(
            "fraction of dark pixels that alone counts as an obstacle; the "
            "default disables it while a reference background is in use"
        ),
    )
    parser.add_argument(
        "--dark-max-value",
        type=float,
        default=None,
        help="brightness below which a pixel is dark, 0-1; this is what shapes the box",
    )
    parser.add_argument(
        "--trigger-streak",
        type=int,
        default=None,
        help="consecutive detecting frames before the tracker triggers",
    )
    parser.add_argument(
        "--clear-streak",
        type=int,
        default=None,
        help="consecutive clear frames before the tracker clears",
    )
    parser.add_argument(
        "--match-tolerance",
        type=float,
        default=12.0,
        help="bearing difference in degrees still counted as the same object",
    )
    parser.add_argument("--plot-range", type=float, default=5.0, help="plot radius in metres")
    parser.add_argument("--min-range", type=float, default=0.12, help="minimum accepted distance in metres")
    parser.add_argument("--max-range", type=float, default=10.0, help="maximum accepted distance in metres")
    return parser


def _load_sdk(library_path: Optional[str]):
    """Loads Rozeta for the camera alone, when the LiDAR is simulated."""
    from demo.x4_driver import RozetaSdk

    try:
        return RozetaSdk(library_path)
    except YdlidarError as error:
        raise RgbObstacleError(str(error)) from error


def default_session_dir() -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return Path("recordings") / f"twin_{stamp}"


def run(args: argparse.Namespace) -> int:
    calibration: Optional[Calibration] = None
    if not args.no_calibration:
        try:
            calibration = find_calibration(args.calibration)
        except (OSError, ValueError, KeyError) as error:
            print(f"warning: could not read calibration: {error}", file=sys.stderr)
    if calibration is not None:
        print(
            f"calibration {calibration.source}: camera axis {calibration.camera_axis_deg:+.1f} deg, "
            f"FOV {calibration.horizontal_fov_deg:.0f} deg, {len(calibration.blind_sectors)} blind sectors"
        )
    if args.forward_angle is None:
        args.forward_angle = calibration.camera_axis_deg if calibration else 0.0
    if args.field_of_view is None:
        args.field_of_view = (
            calibration.horizontal_fov_deg if calibration and calibration.horizontal_fov_deg > 0 else 70.0
        )

    settings = LidarSettings(
        port=args.port,
        min_range_m=args.min_range,
        max_range_m=args.max_range,
    )
    settings.validate()

    camera = CameraStream(
        device=args.camera,
        width=args.width,
        height=args.height,
        capture_fps=args.camera_fps,
        output_fps=args.stream_fps,
    )
    source = open_source(settings, simulate=args.simulate, library_path=args.rozeta_lib)

    writer: Optional[SessionWriter] = None
    gate: Optional[MotionGate] = None
    detector: Optional[CameraDetector] = None
    lidar: Optional[LidarStream] = None
    window: Optional[TwinWindow] = None
    stop_reason = "finished"

    try:
        camera.open()
        first_frame = camera.wait_for_frame()
        source.open()
        lidar = LidarStream(source).start()
        first_scan = lidar.wait_for_scan()
        version = getattr(source, "device_version", None)
        print(
            f"camera={args.camera} {args.width}x{args.height} frames_ok; "
            f"lidar={getattr(source, 'port', 'SIMULATED')} "
            f"firmware={getattr(version, 'firmware', 'n/a')} points={first_scan.point_count}"
        )

        if not args.no_camera_detection:
            try:
                # The simulated source has no library of its own to borrow.
                sdk = getattr(source, "sdk", None)
                # With a reference background the question is what is *new*, so
                # the dark-pixel path must not trigger on its own: a dim room is
                # half dark and would leave the tracker latched on for good.
                dark_coverage = args.dark_coverage
                if dark_coverage is None and not args.no_reference:
                    dark_coverage = 1.0
                tracker = RgbObstacleTracker(
                    sdk if sdk is not None else _load_sdk(args.rozeta_lib),
                    diff_threshold=args.diff_threshold,
                    diff_coverage_threshold=args.diff_coverage,
                    coverage_threshold=dark_coverage,
                    dark_max_value=args.dark_max_value,
                    trigger_streak=args.trigger_streak,
                    clear_streak=args.clear_streak,
                )
            except RgbObstacleError as error:
                print(f"warning: camera detection disabled: {error}", file=sys.stderr)
            else:
                detector = CameraDetector(
                    tracker,
                    use_reference=not args.no_reference,
                    settle_s=args.reference_delay,
                )
                mode = "new objects against a reference background" if detector.use_reference else "dark blobs"
                print(
                    f"camera detection: {mode}, trigger {tracker.config.trigger_streak} frames / "
                    f"clear {tracker.config.clear_streak}, match within {args.match_tolerance:.0f} deg"
                )
                if calibration is None:
                    print(
                        "warning: no calibration, so a camera detection has no bearing "
                        "and cannot be matched to a LiDAR cluster",
                        file=sys.stderr,
                    )

        if not args.no_record:
            directory = Path(args.session) if args.session else default_session_dir()
            metadata = {
                "started": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "camera": {"device": args.camera, "width": args.width, "height": args.height},
                "lidar": {
                    "port": getattr(source, "port", None),
                    "simulated": bool(args.simulate),
                    "firmware": getattr(version, "firmware", None),
                    "min_range_m": args.min_range,
                    "max_range_m": args.max_range,
                },
                "sample_rate_hz": args.rate,
                "sector": {"forward_angle_deg": args.forward_angle, "field_of_view_deg": args.field_of_view},
                "calibration": calibration.to_dict() if calibration else None,
                "notes": "angle_deg is the raw LiDAR bearing; nearest excludes calibrated blind sectors",
            }
            writer = SessionWriter(
                directory,
                metadata,
                calibration,
                budget_bytes=args.max_session_mb * 1024 * 1024,
                min_free_bytes=args.min_free_mb * 1024 * 1024,
            )
            print(f"session: {directory}")

        started = time.monotonic()
        state = {"index": 0, "next_sample": started, "running": True}
        if not args.record_idle:
            gate = MotionGate(
                args.motion_luma,
                args.motion_range,
                args.keepalive,
                min_cells=args.motion_cells,
                min_bins=args.motion_bins,
            )

        def tick() -> Optional[dict]:
            nonlocal detector
            frame = camera.latest()
            scan = lidar.latest() if lidar else None
            if frame is None or scan is None:
                return None
            if scan.point_count < args.min_points:
                return None
            if detector is not None:
                try:
                    detector.observe(frame)
                except RgbObstacleError as error:
                    print(f"camera detection stopped: {error}", file=sys.stderr)
                    detector.close()
                    detector = None
            angles, ranges = scan_arrays(scan)
            fused = fuse(
                detector.detection if detector is not None else None,
                list(zip(angles, ranges)),
                calibration=calibration,
                tolerance_deg=args.match_tolerance,
                sector_centre_deg=args.forward_angle,
                sector_width_deg=args.field_of_view,
            )
            state["fusion"] = fused
            record = None
            now = time.monotonic()
            if now >= state["next_sample"]:
                state["next_sample"] = now + (1.0 / args.rate if args.rate > 0 else 0.5)
                state["index"] += 1
                sample = Sample(index=state["index"], scan=scan, frame=frame, fusion=fused)
                recording = (
                    writer is not None
                    and (window is None or window.recording.get())
                    and writer.has_room()
                    and (gate is None or gate.should_write(sample, now))
                )
                # Without a session the sample is still summarised, so --no-record
                # and skipped idle samples stay a live view with the same counters.
                record = writer.write_sample(sample) if recording else summarize(sample, calibration)
                if writer is not None and writer.stopped_reason and not state.get("warned"):
                    state["warned"] = True
                    print(f"recording stopped: {writer.stopped_reason}", file=sys.stderr)
                    if window is not None:
                        window.recording.set(False)
                        window.status.set(f"recording stopped: {writer.stopped_reason}")
            if args.duration > 0 and now - started >= args.duration:
                state["running"] = False
            if args.samples > 0 and state["index"] >= args.samples:
                state["running"] = False
            return record

        if args.headless:
            while state["running"]:
                record = tick()
                if record is not None:
                    nearest = record["nearest"]
                    near_text = (
                        f"{nearest['distance_m']:.3f} m at {nearest['angle_deg']:+.1f} deg "
                        f"({nearest['points']} pts)"
                        if nearest
                        else "n/a"
                    )
                    fused = state.get("fusion")
                    fusion_text = f" {fused.describe()}" if fused is not None else ""
                    print(
                        f"sample {record['index']:>4} points={record['point_count']:>4} "
                        f"scan={record['scan_frequency_hz']:.2f}Hz sync={record['sync_delta_ms']:+.0f}ms "
                        f"nearest={near_text}{fusion_text}"
                    )
                time.sleep(0.02)
        else:
            window = TwinWindow(args, writer, calibration, detector)

            def refresh() -> None:
                if not state["running"]:
                    window.root.destroy()
                    return
                try:
                    record = tick()
                except Exception as error:  # a failed sample must not freeze the window
                    record = None
                    state["running"] = False
                    print(f"error: {error}", file=sys.stderr)
                frame = camera.latest()
                scan = lidar.latest() if lidar else None
                if frame is not None:
                    window.show_camera(frame, detector.detection if detector else None)
                if scan is not None:
                    window.show_scan(scan, state.get("fusion"))
                    near = nearest_point(*scan_arrays(scan), calibration)
                    near_text = (
                        f"{near['distance_m']:.2f} m at {near['angle_deg']:+.1f} deg ({near['points']} pts)"
                        if near
                        else "n/a"
                    )
                    saved = writer.count if writer else 0
                    if record is not None:
                        window.last_sample_index = record["index"]
                    fused = state.get("fusion")
                    if detector is not None and detector.waiting:
                        fusion_text = "  camera: waiting for the reference frame"
                    elif fused is not None:
                        fusion_text = f"  {fused.describe()}"
                    else:
                        fusion_text = ""
                    window.status.set(
                        f"points={scan.point_count}  scan={scan.scan_frequency_hz:.2f} Hz  "
                        f"nearest={near_text}  samples={saved}{fusion_text}"
                    )
                window.root.after(max(30, int(1000 / max(args.stream_fps, 1))), refresh)

            window.root.after(50, refresh)
            window.root.protocol("WM_DELETE_WINDOW", lambda: (state.__setitem__("running", False)))
            window.root.mainloop()

    except KeyboardInterrupt:
        stop_reason = "interrupted"
    except (CameraError, YdlidarError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    finally:
        if detector is not None:
            detector.close()
        if lidar is not None:
            lidar.stop()
        try:
            source.close()
        except Exception:
            pass
        camera.close()

    if writer is not None:
        skipped = f", {gate.skipped} idle samples skipped" if gate is not None else ""
        size_mb = writer.bytes_written / (1024 * 1024)
        print(f"{stop_reason}: {writer.count} samples ({size_mb:.0f} MB) in {writer.directory}{skipped}")
        if writer.stopped_reason:
            print(f"recording had stopped early: {writer.stopped_reason}")
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.list_cameras:
        for index, name in enumerate(list_cameras()):
            print(f"[{index}] {name}")
        return 0
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
