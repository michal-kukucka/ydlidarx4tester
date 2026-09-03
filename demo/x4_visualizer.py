"""Live YDLIDAR X4 visualizer using Rozeta's native C++ driver."""

from __future__ import annotations

import argparse
import csv
import math
import queue
import sys
import threading
from pathlib import Path
from typing import Optional, Sequence

try:
    from .x4_driver import (
        LidarSettings,
        PORT_EXAMPLE,
        ROZETA_LIBRARY_NAMES,
        ScanFrame,
        YdlidarError,
        RozetaSdk,
        open_source,
    )
    from .calibration import Calibration, cluster_obstacles, find_calibration, nearest_obstacle
except ImportError:
    from calibration import Calibration, cluster_obstacles, find_calibration, nearest_obstacle  # type: ignore
    from x4_driver import (  # type: ignore
        LidarSettings,
        PORT_EXAMPLE,
        ROZETA_LIBRARY_NAMES,
        ScanFrame,
        YdlidarError,
        RozetaSdk,
        open_source,
    )


class CsvRecorder:
    def __init__(self, path: Optional[Path]):
        self.path = path
        self._file = None
        self._writer = None
        self._scan_index = 0

    def __enter__(self) -> "CsvRecorder":
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._file = self.path.open("w", newline="", encoding="utf-8")
            self._writer = csv.writer(self._file)
            self._writer.writerow(
                ("timestamp_ns", "scan_index", "angle_rad", "distance_m", "intensity")
            )
        return self

    def write(self, frame: ScanFrame) -> None:
        if self._writer:
            self._writer.writerows(
                (
                    frame.stamp_ns,
                    self._scan_index,
                    f"{angle:.8f}",
                    f"{distance:.5f}",
                    f"{intensity:.1f}",
                )
                for angle, distance, intensity in zip(
                    frame.angles_rad, frame.ranges_m, frame.intensities
                )
            )
            self._file.flush()
        self._scan_index += 1

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        if self._file:
            self._file.close()


class AcquisitionWorker(threading.Thread):
    def __init__(
        self,
        settings: LidarSettings,
        simulate: bool,
        native_library: Optional[Path],
        record: Optional[Path],
    ):
        super().__init__(name="x4-acquisition", daemon=True)
        self.settings = settings
        self.simulate = simulate
        self.native_library = native_library
        self.record = record
        self.frames: queue.Queue[ScanFrame] = queue.Queue(maxsize=2)
        self.errors: queue.Queue[BaseException] = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.source_description = "starting..."
        self.frame_count = 0

    def run(self) -> None:
        try:
            with CsvRecorder(self.record) as recorder:
                with open_source(self.settings, self.simulate, self.native_library) as source:
                    version = source.device_version
                    self.source_description = (
                        f"{source.port} | firmware {version.firmware} | S/N {version.serial_number}"
                    )
                    consecutive_failures = 0
                    while not self.stop_event.is_set():
                        try:
                            frame = source.read_scan()
                            consecutive_failures = 0
                        except YdlidarError:
                            consecutive_failures += 1
                            if consecutive_failures >= 5:
                                raise
                            continue
                        recorder.write(frame)
                        self.frame_count += 1
                        if self.frames.full():
                            try:
                                self.frames.get_nowait()
                            except queue.Empty:
                                pass
                        self.frames.put_nowait(frame)
        except BaseException as exc:
            try:
                self.errors.put_nowait(exc)
            except queue.Full:
                pass


def apply_calibration(args: argparse.Namespace, argv: Sequence[str]) -> Optional[Calibration]:
    """Aim the sector and mask the mount from a recorded twin calibration.

    Explicit command-line values always win; the calibration only fills in what
    the operator did not state.
    """
    if args.no_calibration:
        return None
    try:
        calibration = find_calibration(args.calibration)
    except (OSError, ValueError, KeyError) as error:
        print(f"warning: could not read calibration: {error}", file=sys.stderr)
        return None
    if calibration is None:
        return None
    given = set(argv)
    if not any(flag.startswith("--forward-angle") for flag in given):
        args.forward_angle = calibration.camera_axis_deg
    if not any(flag.startswith("--field-of-view") for flag in given) and calibration.horizontal_fov_deg > 0.0:
        args.field_of_view = calibration.horizontal_fov_deg
    return calibration


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Live polar/Cartesian visualization for a YDLIDAR X4",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", help=f"serial data port, for example {PORT_EXAMPLE}")
    parser.add_argument(
        "--rozeta-lib",
        "--rozeta-dll",
        "--sdk-dll",
        dest="rozeta_lib",
        type=Path,
        help=f"path to Rozeta's {ROZETA_LIBRARY_NAMES[0]}",
    )
    parser.add_argument("--list-ports", action="store_true", help="list serial ports visible to this machine")
    parser.add_argument("--simulate", action="store_true", help="run the complete demo without hardware")
    parser.add_argument("--scan-frequency", type=float, default=8.0, help="motor scan frequency in Hz")
    parser.add_argument("--min-range", type=float, default=0.12, help="minimum accepted distance in metres")
    parser.add_argument("--max-range", type=float, default=10.0, help="maximum accepted distance in metres")
    parser.add_argument("--min-angle", type=float, default=-180.0, help="minimum accepted angle in degrees")
    parser.add_argument("--max-angle", type=float, default=180.0, help="maximum accepted angle in degrees")
    parser.add_argument(
        "--forward-angle",
        type=float,
        default=-125.0,
        help="physical forward direction in LiDAR degrees",
    )
    parser.add_argument(
        "--field-of-view",
        type=float,
        default=70.0,
        help="displayed forward sector width in degrees",
    )
    parser.add_argument(
        "--show-all",
        action="store_true",
        help="disable the forward sector and display all directions",
    )
    parser.add_argument(
        "--ignore-array",
        default="",
        help='SDK angle exclusion list, for example "-15,15,170,180"',
    )
    parser.add_argument("--fixed-resolution", action="store_true", help="resample scans to a fixed angular grid")
    parser.add_argument("--reversion", action="store_true", help="rotate reported scans by 180 degrees")
    parser.add_argument("--inverted", action="store_true", help="reverse reported scan direction")
    parser.add_argument("--no-auto-reconnect", action="store_true", help="disable SDK hot-plug reconnection")
    parser.add_argument("--sun-filter", action="store_true", help="enable strong-sunlight noise filtering")
    parser.add_argument("--glass-filter", action="store_true", help="enable glass-reflection filtering")
    parser.add_argument("--debug", action="store_true", help="enable verbose native SDK diagnostics")
    parser.add_argument("--danger-distance", type=float, default=0.65, help="nearest-obstacle warning distance in metres")
    parser.add_argument(
        "--calibration",
        help="calibration.json from a twin session (default: newest under recordings/)",
    )
    parser.add_argument("--no-calibration", action="store_true", help="ignore any recorded calibration")
    parser.add_argument(
        "--cluster-points",
        type=int,
        default=3,
        help="returns that must agree before a cluster counts as an obstacle",
    )
    parser.add_argument("--record", type=Path, help="write every received point to a CSV file")
    parser.add_argument("--headless", action="store_true", help="print scan summaries instead of opening a GUI")
    parser.add_argument("--frames", type=int, default=0, help="stop after N scans; 0 runs forever")
    return parser


def settings_from_args(args: argparse.Namespace) -> LidarSettings:
    settings = LidarSettings(
        port=args.port,
        scan_frequency_hz=args.scan_frequency,
        min_range_m=args.min_range,
        max_range_m=args.max_range,
        min_angle_deg=args.min_angle,
        max_angle_deg=args.max_angle,
        ignore_array=args.ignore_array,
        fixed_resolution=args.fixed_resolution,
        reversion=args.reversion,
        inverted=args.inverted,
        auto_reconnect=not args.no_auto_reconnect,
        sun_filter=args.sun_filter,
        glass_filter=args.glass_filter,
        debug=args.debug,
    )
    settings.validate()
    if args.danger_distance <= 0.0:
        raise ValueError("danger distance must be positive")
    if not -180.0 <= args.forward_angle <= 180.0:
        raise ValueError("forward angle must be between -180 and 180 degrees")
    if not 5.0 <= args.field_of_view <= 360.0:
        raise ValueError("field of view must be between 5 and 360 degrees")
    if args.frames < 0:
        raise ValueError("frames cannot be negative")
    return settings


def normalize_angle_deg(angle_deg: float) -> float:
    """Normalize an angle to the half-open interval [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def angle_in_sector(angle_rad: float, center_deg: float, width_deg: float) -> bool:
    """Return whether an angle lies in a possibly wrap-around sector."""
    if width_deg >= 360.0:
        return True
    delta = normalize_angle_deg(math.degrees(angle_rad) - center_deg)
    return abs(delta) <= width_deg / 2.0


def requested_roi(args: argparse.Namespace) -> tuple[float, float]:
    return args.forward_angle, 360.0 if args.show_all else args.field_of_view


def valid_points(
    frame: ScanFrame,
    settings: LidarSettings,
    roi_center_deg: float = 0.0,
    roi_width_deg: float = 360.0,
    calibration: Optional[Calibration] = None,
) -> list[tuple[float, float]]:
    return [
        (angle, distance)
        for angle, distance in zip(frame.angles_rad, frame.ranges_m)
        if math.isfinite(angle)
        and math.isfinite(distance)
        and settings.min_range_m <= distance <= settings.max_range_m
        and angle_in_sector(angle, roi_center_deg, roi_width_deg)
        and not (calibration is not None and calibration.is_blind(math.degrees(angle)))
    ]


def obstacles_from_points(
    points: Sequence[tuple[float, float]],
    calibration: Optional[Calibration] = None,
    min_points: int = 3,
):
    """Cluster valid points (radians) into obstacles reported in degrees."""
    return cluster_obstacles(
        [(math.degrees(angle), distance) for angle, distance in points],
        min_points=min_points,
        calibration=calibration,
    )


def describe_frame(
    frame: ScanFrame,
    settings: LidarSettings,
    roi_center_deg: float = 0.0,
    roi_width_deg: float = 360.0,
    calibration: Optional[Calibration] = None,
    min_points: int = 3,
) -> str:
    points = valid_points(frame, settings, roi_center_deg, roi_width_deg, calibration)
    nearest = None
    if points:
        nearest = obstacles_from_points(points, calibration, min_points)
    if nearest:
        closest = nearest[0]
        obstacle = (
            f"nearest={closest.distance_m:.3f}m at {closest.angle_deg:+.1f}deg "
            f"width={closest.width_deg:.0f}deg n={closest.points}"
        )
    else:
        obstacle = "nearest=n/a"
    return (
        f"stamp={frame.stamp_ns} points={len(points)}/{frame.point_count} "
        f"scan={frame.scan_frequency_hz:.2f}Hz {obstacle}"
    )


def run_headless(
    args: argparse.Namespace, settings: LidarSettings, calibration: Optional[Calibration] = None
) -> int:
    count = 0
    roi_center, roi_width = requested_roi(args)
    with CsvRecorder(args.record) as recorder:
        with open_source(settings, args.simulate, args.rozeta_lib) as source:
            version = source.device_version
            print(
                f"Connected: {source.port}; firmware={version.firmware}; "
                f"serial={version.serial_number}"
            )
            consecutive_failures = 0
            while args.frames == 0 or count < args.frames:
                try:
                    frame = source.read_scan()
                    consecutive_failures = 0
                except YdlidarError as exc:
                    consecutive_failures += 1
                    if consecutive_failures >= 5:
                        raise
                    print(f"Transient scan failure ({consecutive_failures}/5): {exc}", file=sys.stderr)
                    continue
                recorder.write(frame)
                print(
                    describe_frame(frame, settings, roi_center, roi_width, calibration, args.cluster_points),
                    flush=True,
                )
                count += 1
    return 0


def ui_font_family(root: "object") -> str:
    """Pick a readable UI font that actually exists on this platform."""
    import tkinter.font as tkfont

    preferred = {
        "darwin": ("SF Pro Text", "Helvetica Neue", "Lucida Grande"),
        "win32": ("Segoe UI",),
    }.get(sys.platform, ("DejaVu Sans", "Liberation Sans"))
    available = set(tkfont.families(root))
    for family in preferred:
        if family in available:
            return family
    return tkfont.nametofont("TkDefaultFont").actual("family")


def run_gui(
    args: argparse.Namespace, settings: LidarSettings, calibration: Optional[Calibration] = None
) -> int:
    import tkinter as tk
    from tkinter import ttk

    worker = AcquisitionWorker(settings, args.simulate, args.rozeta_lib, args.record)
    worker.start()

    root = tk.Tk()
    root.title("YDLIDAR X4 live detection")
    root.geometry("1280x760")
    root.minsize(900, 520)
    style = ttk.Style(root)
    for theme in ("aqua", "vista", "clam"):
        if theme in style.theme_names():
            style.theme_use(theme)
            break
    ui_font = ui_font_family(root)
    root.columnconfigure(0, weight=1)
    root.columnconfigure(1, weight=1)
    root.rowconfigure(1, weight=1)

    title = ttk.Label(
        root,
        text="YDLIDAR X4 — waiting for first scan...",
        anchor="center",
        font=(ui_font, 11, "bold"),
    )
    title.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=(8, 4))
    polar_canvas = tk.Canvas(root, background="#07131f", highlightthickness=0)
    cart_canvas = tk.Canvas(root, background="#0b1218", highlightthickness=0)
    polar_canvas.grid(row=1, column=0, sticky="nsew", padx=(8, 4), pady=4)
    cart_canvas.grid(row=1, column=1, sticky="nsew", padx=(4, 8), pady=4)
    controls = ttk.Frame(root)
    controls.grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=(2, 0))
    controls.columnconfigure(2, weight=1)
    controls.columnconfigure(5, weight=1)

    roi_enabled = tk.BooleanVar(value=not args.show_all)
    roi_center = tk.DoubleVar(value=args.forward_angle)
    roi_width = tk.DoubleVar(value=args.field_of_view)
    roi_center_text = tk.StringVar(value=f"{args.forward_angle:+.0f}°")
    roi_width_text = tk.StringVar(value=f"{args.field_of_view:.0f}°")
    roi_check = ttk.Checkbutton(controls, text="Forward ROI only", variable=roi_enabled)
    roi_check.grid(row=0, column=0, padx=(0, 12))
    ttk.Label(controls, text="Direction").grid(row=0, column=1, padx=(0, 4))
    center_scale = ttk.Scale(controls, from_=-180.0, to=180.0, variable=roi_center)
    center_scale.grid(row=0, column=2, sticky="ew", padx=(0, 4))
    ttk.Label(controls, textvariable=roi_center_text, width=6).grid(row=0, column=3, padx=(0, 12))
    ttk.Label(controls, text="FOV width").grid(row=0, column=4, padx=(0, 4))
    width_scale = ttk.Scale(controls, from_=5.0, to=360.0, variable=roi_width)
    width_scale.grid(row=0, column=5, sticky="ew", padx=(0, 4))
    ttk.Label(controls, textvariable=roi_width_text, width=6).grid(row=0, column=6)

    legend = ttk.Label(
        root,
        text=(
            "Click the polar plot to select forward direction    |    green wedge = active ROI    |    "
            "red/orange = near, cyan/blue = far"
        ),
        anchor="center",
    )
    legend.grid(row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=(2, 8))

    last_frame: Optional[ScanFrame] = None

    def current_roi() -> tuple[float, float]:
        center = normalize_angle_deg(roi_center.get())
        width = max(5.0, min(360.0, roi_width.get()))
        return center, width if roi_enabled.get() else 360.0

    def geometry(canvas: tk.Canvas) -> tuple[float, float, float]:
        width = max(100, canvas.winfo_width())
        height = max(100, canvas.winfo_height())
        return width / 2.0, height / 2.0, min(width, height) * 0.45 / settings.max_range_m

    def draw_static() -> None:
        polar_canvas.delete("static")
        cart_canvas.delete("static")
        pcx, pcy, pscale = geometry(polar_canvas)
        ccx, ccy, cscale = geometry(cart_canvas)
        center_deg, width_deg = current_roi()
        if width_deg < 360.0:
            polar_radius = settings.max_range_m * pscale
            cart_radius = settings.max_range_m * cscale
            arc_start = 90.0 - (center_deg + width_deg / 2.0)
            polar_canvas.create_arc(
                pcx - polar_radius,
                pcy - polar_radius,
                pcx + polar_radius,
                pcy + polar_radius,
                start=arc_start,
                extent=width_deg,
                style=tk.PIESLICE,
                fill="#0b2b2b",
                outline="#4ee6a8",
                width=2,
                tags="static",
            )
            cart_canvas.create_arc(
                ccx - cart_radius,
                ccy - cart_radius,
                ccx + cart_radius,
                ccy + cart_radius,
                start=arc_start,
                extent=width_deg,
                style=tk.ARC,
                outline="#4ee6a8",
                width=2,
                tags="static",
            )
            for boundary_deg in (center_deg - width_deg / 2.0, center_deg + width_deg / 2.0):
                boundary = math.radians(boundary_deg)
                cart_canvas.create_line(
                    ccx,
                    ccy,
                    ccx + cart_radius * math.sin(boundary),
                    ccy - cart_radius * math.cos(boundary),
                    fill="#4ee6a8",
                    width=2,
                    tags="static",
                )
        for index in range(1, 6):
            distance = settings.max_range_m * index / 5.0
            radius = distance * pscale
            polar_canvas.create_oval(
                pcx - radius, pcy - radius, pcx + radius, pcy + radius,
                outline="#284458", width=1, tags="static",
            )
            polar_canvas.create_text(
                pcx + 5, pcy - radius + 9, text=f"{distance:g} m",
                fill="#8fa8b7", anchor="w", font=(ui_font, 8), tags="static",
            )
        for angle_deg in range(0, 360, 30):
            angle = math.radians(angle_deg)
            radius = settings.max_range_m * pscale
            x = pcx + radius * math.sin(angle)
            y = pcy - radius * math.cos(angle)
            polar_canvas.create_line(pcx, pcy, x, y, fill="#1e3444", tags="static")
        polar_canvas.create_text(
            12, 12, text="POLAR — LiDAR angles", fill="#dcecf5",
            anchor="nw", font=(ui_font, 10, "bold"), tags="static",
        )

        grid_step = 1.0 if settings.max_range_m <= 10.0 else 2.0
        grid_count = int(settings.max_range_m / grid_step)
        for index in range(-grid_count, grid_count + 1):
            offset = index * grid_step * cscale
            cart_canvas.create_line(
                ccx + offset, 0, ccx + offset, cart_canvas.winfo_height(),
                fill="#20303a", tags="static",
            )
            cart_canvas.create_line(
                0, ccy + offset, cart_canvas.winfo_width(), ccy + offset,
                fill="#20303a", tags="static",
            )
        cart_canvas.create_line(ccx, 0, ccx, cart_canvas.winfo_height(), fill="#607887", tags="static")
        cart_canvas.create_line(0, ccy, cart_canvas.winfo_width(), ccy, fill="#607887", tags="static")
        danger = args.danger_distance * cscale
        cart_canvas.create_oval(
            ccx - danger, ccy - danger, ccx + danger, ccy + danger,
            outline="#ff485a", dash=(5, 4), width=2, tags="static",
        )
        cart_canvas.create_polygon(
            ccx, ccy - 9, ccx - 7, ccy + 8, ccx + 7, ccy + 8,
            fill="#f5f7fa", tags="static",
        )
        cart_canvas.create_text(
            12, 12, text="CARTESIAN — metres", fill="#dcecf5",
            anchor="nw", font=(ui_font, 10, "bold"), tags="static",
        )

    def point_colour(distance: float) -> str:
        ratio = max(0.0, min(1.0, distance / settings.max_range_m))
        red = int(255 * (1.0 - ratio))
        green = int(105 + 120 * ratio)
        blue = int(55 + 200 * ratio)
        return f"#{red:02x}{green:02x}{blue:02x}"

    def draw_scan(frame: ScanFrame) -> None:
        center_deg, width_deg = current_roi()
        points = valid_points(frame, settings, center_deg, width_deg, calibration)
        polar_canvas.delete("scan")
        cart_canvas.delete("scan")
        pcx, pcy, pscale = geometry(polar_canvas)
        ccx, ccy, cscale = geometry(cart_canvas)
        for angle, distance in points:
            sin_angle = math.sin(angle)
            cos_angle = math.cos(angle)
            colour = point_colour(distance)
            px = pcx + distance * pscale * sin_angle
            py = pcy - distance * pscale * cos_angle
            cx = ccx + distance * cscale * sin_angle
            cy = ccy - distance * cscale * cos_angle
            polar_canvas.create_oval(px - 1.5, py - 1.5, px + 1.5, py + 1.5, fill=colour, outline="", tags="scan")
            cart_canvas.create_oval(cx - 1.5, cy - 1.5, cx + 1.5, cy + 1.5, fill=colour, outline="", tags="scan")
        obstacles = obstacles_from_points(points, calibration, args.cluster_points) if points else []
        if obstacles:
            closest = obstacles[0]
            nearest_angle = math.radians(closest.angle_deg)
            nearest_range = closest.distance_m
            nx = math.sin(nearest_angle)
            ny = math.cos(nearest_angle)
            for canvas, center_x, center_y, scale in (
                (polar_canvas, pcx, pcy, pscale),
                (cart_canvas, ccx, ccy, cscale),
            ):
                x = center_x + nearest_range * scale * nx
                y = center_y - nearest_range * scale * ny
                canvas.create_line(x - 7, y - 7, x + 7, y + 7, fill="#ff253a", width=3, tags="scan")
                canvas.create_line(x - 7, y + 7, x + 7, y - 7, fill="#ff253a", width=3, tags="scan")
            warning = "  —  WARNING: obstacle close" if nearest_range < args.danger_distance else ""
            title.configure(
                text=(
                    f"{worker.source_description}  |  {len(points)} valid points  |  "
                    f"{frame.scan_frequency_hz:.2f} Hz  |  nearest {nearest_range:.3f} m "
                    f"at {closest.angle_deg:+.1f} degrees ({closest.points} pts, "
                    f"{closest.width_deg:.0f}° wide)  |  "
                    f"ROI {center_deg:+.0f}° / {width_deg:.0f}°{warning}"
                ),
                foreground="#c5162e" if warning else "#17222b",
            )
        else:
            detail = "no obstacle cluster" if points else "no valid points"
            title.configure(text=f"{worker.source_description} | {detail}", foreground="#17222b")

    def on_roi_change(_value: object = None) -> None:
        center_deg, width_deg = current_roi()
        roi_center_text.set(f"{center_deg:+.0f}°")
        roi_width_text.set(f"{roi_width.get():.0f}°")
        draw_static()
        if last_frame is not None:
            draw_scan(last_frame)

    def on_polar_click(event: tk.Event) -> None:
        pcx, pcy, _ = geometry(polar_canvas)
        dx = event.x - pcx
        dy = event.y - pcy
        if math.hypot(dx, dy) < 10.0:
            return
        roi_center.set(math.degrees(math.atan2(dx, -dy)))
        roi_enabled.set(True)
        on_roi_change()

    roi_check.configure(command=on_roi_change)
    center_scale.configure(command=on_roi_change)
    width_scale.configure(command=on_roi_change)

    closing = False
    error_displayed = False

    def poll() -> None:
        nonlocal error_displayed, last_frame
        if closing:
            return
        if not worker.errors.empty() and not error_displayed:
            error_displayed = True
            title.configure(text=f"ERROR: {worker.errors.get_nowait()}", foreground="#c5162e")
        latest = None
        while True:
            try:
                latest = worker.frames.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            last_frame = latest
            draw_scan(latest)
            if args.frames and worker.frame_count >= args.frames:
                root.after(80, on_close)
        root.after(40, poll)

    def on_resize(_event: object) -> None:
        draw_static()

    def on_close() -> None:
        nonlocal closing
        closing = True
        worker.stop_event.set()
        root.destroy()

    polar_canvas.bind("<Configure>", on_resize)
    polar_canvas.bind("<Button-1>", on_polar_click)
    cart_canvas.bind("<Configure>", on_resize)
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(40, poll)
    try:
        root.mainloop()
    finally:
        worker.stop_event.set()
        worker.join(timeout=4.0)
    if not worker.errors.empty():
        raise worker.errors.get_nowait()
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.list_ports:
            sdk = RozetaSdk(args.rozeta_lib)
            ports = sdk.list_ports()
            print(f"Rozeta {sdk.version}")
            if ports:
                for port in ports:
                    print(port)
                return 0
            print("No serial ports detected.")
            return 2
        calibration = apply_calibration(args, list(argv) if argv is not None else sys.argv[1:])
        settings = settings_from_args(args)
        if calibration is not None:
            blind = ", ".join(
                f"{sector.start_deg:+.0f}..{sector.end_deg:+.0f}" for sector in calibration.blind_sectors
            )
            print(
                f"calibration {calibration.source}: camera axis {calibration.camera_axis_deg:+.1f} deg, "
                f"FOV {calibration.horizontal_fov_deg:.0f} deg"
                + (f", blind {blind}" if blind else "")
            )
        if args.headless:
            return run_headless(args, settings, calibration)
        return run_gui(args, settings, calibration)
    except (OSError, ValueError, YdlidarError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Stopped.")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
