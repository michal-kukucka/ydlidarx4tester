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
from demo.x4_driver import LidarSettings, ScanFrame, YdlidarError, open_source

SECTOR_WIDTH_DEG = 15.0


@dataclass
class Sample:
    index: int
    scan: ScanFrame
    frame: CameraFrame


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

    def __init__(self, directory: Path, metadata: dict, calibration: Optional[Calibration] = None) -> None:
        self.calibration = calibration
        self.directory = directory
        self.frames_dir = directory / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.samples_path = directory / "samples.jsonl"
        self.labels_path = directory / "labels.jsonl"
        (directory / "session.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        self._lock = threading.Lock()
        self.count = 0

    def write_sample(self, sample: Sample) -> dict:
        angles, ranges = scan_arrays(sample.scan)
        frame_name = f"frame_{sample.index:05d}.png"
        # Level 1 keeps the per-sample encode near 60 ms; the files stay readable.
        (self.frames_dir / frame_name).write_bytes(sample.frame.to_png(compression=1))
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
            "points": [[round(a, 3), round(d, 4)] for a, d in zip(angles, ranges) if d > 0.0],
        }
        with self._lock:
            with self.samples_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record) + "\n")
            self.count += 1
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


class TwinWindow:
    """Tk view: camera on the left, polar LiDAR plot on the right."""

    def __init__(
        self,
        args: argparse.Namespace,
        writer: Optional[SessionWriter],
        calibration: Optional[Calibration] = None,
    ) -> None:
        import tkinter as tk

        self.tk = tk
        self.args = args
        self.writer = writer
        self.calibration = calibration
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
        self._photo = None
        self.last_sample_index = 0

    def mark(self) -> None:
        text = self.label_entry.get().strip()
        if not text or self.writer is None:
            return
        self.writer.write_label(self.last_sample_index, text)
        self.label_entry.delete(0, self.tk.END)
        self.status.set(f"labelled sample {self.last_sample_index}: {text}")

    def show_camera(self, frame: CameraFrame) -> None:
        # Level 0 costs ~7 ms per frame; Tk only needs a container, not small files.
        self._photo = self.tk.PhotoImage(data=frame.to_png(compression=0))
        self.canvas_camera.delete("all")
        self.canvas_camera.create_image(0, 0, image=self._photo, anchor="nw")

    def show_scan(self, scan: ScanFrame) -> None:
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
        canvas.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill="#e05050", outline="")


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
    parser.add_argument("--plot-range", type=float, default=5.0, help="plot radius in metres")
    parser.add_argument("--min-range", type=float, default=0.12, help="minimum accepted distance in metres")
    parser.add_argument("--max-range", type=float, default=10.0, help="maximum accepted distance in metres")
    return parser


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
            writer = SessionWriter(directory, metadata, calibration)
            print(f"session: {directory}")

        started = time.monotonic()
        state = {"index": 0, "next_sample": started, "running": True}

        def tick() -> Optional[dict]:
            frame = camera.latest()
            scan = lidar.latest() if lidar else None
            if frame is None or scan is None:
                return None
            if scan.point_count < args.min_points:
                return None
            record = None
            now = time.monotonic()
            if now >= state["next_sample"]:
                state["next_sample"] = now + (1.0 / args.rate if args.rate > 0 else 0.5)
                state["index"] += 1
                sample = Sample(index=state["index"], scan=scan, frame=frame)
                recording = writer is not None and (window is None or window.recording.get())
                # Without a session the sample is still summarised, so --no-record
                # stays a live view with the same counters and stop conditions.
                record = writer.write_sample(sample) if recording else summarize(sample, calibration)
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
                    print(
                        f"sample {record['index']:>4} points={record['point_count']:>4} "
                        f"scan={record['scan_frequency_hz']:.2f}Hz sync={record['sync_delta_ms']:+.0f}ms "
                        f"nearest={near_text}"
                    )
                time.sleep(0.02)
        else:
            window = TwinWindow(args, writer, calibration)

            def refresh() -> None:
                if not state["running"]:
                    window.root.destroy()
                    return
                record = tick()
                frame = camera.latest()
                scan = lidar.latest() if lidar else None
                if frame is not None:
                    window.show_camera(frame)
                if scan is not None:
                    window.show_scan(scan)
                    near = nearest_point(*scan_arrays(scan), calibration)
                    near_text = (
                        f"{near['distance_m']:.2f} m at {near['angle_deg']:+.1f} deg ({near['points']} pts)"
                        if near
                        else "n/a"
                    )
                    saved = writer.count if writer else 0
                    if record is not None:
                        window.last_sample_index = record["index"]
                    window.status.set(
                        f"points={scan.point_count}  scan={scan.scan_frequency_hz:.2f} Hz  "
                        f"nearest={near_text}  samples={saved}"
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
        if lidar is not None:
            lidar.stop()
        try:
            source.close()
        except Exception:
            pass
        camera.close()

    if writer is not None:
        print(f"{stop_reason}: {writer.count} samples in {writer.directory}")
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
