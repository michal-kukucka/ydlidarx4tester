"""Estimate the camera-to-LiDAR bearing mapping from a twin session.

The session recorded by ``twin_capture.py`` holds, per sample, one camera frame
and one LiDAR revolution. This tool finds what moved in each of them:

* camera: the horizontal centre of the pixels that differ from the static
  background (per-pixel median over the session);
* LiDAR: the angular centre of the returns that came closer than the static
  background (per-bin median over the session).

Fitting ``lidar_angle = offset + scale * (x_px - width/2)`` over the samples
where both agree on a single moving object yields the LiDAR bearing that the
camera looks along, plus the degrees-per-pixel scale (the camera's horizontal
field of view).

Angles are handled on the circle, so a mapping that straddles +/-180 degrees is
fitted the same way as one around 0.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import struct
import sys
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from demo.calibration import CALIBRATION_FILENAME, Calibration, merge_sectors

BIN_DEG = 5.0


def decode_png_rgb(data: bytes) -> tuple[int, int, bytes]:
    """Decode the 8-bit RGB, filter-0 PNGs written by ``camera_stream``."""
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    pos = 8
    width = height = 0
    idat = bytearray()
    while pos < len(data):
        length = struct.unpack(">I", data[pos:pos + 4])[0]
        tag = data[pos + 4:pos + 8]
        payload = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if tag == b"IHDR":
            width, height, depth, colour = struct.unpack(">IIBB", payload[:10])
            if depth != 8 or colour != 2:
                raise ValueError("only 8-bit RGB PNGs are supported")
        elif tag == b"IDAT":
            idat += payload
        elif tag == b"IEND":
            break
    raw = zlib.decompress(bytes(idat))
    stride = width * 3
    out = bytearray(height * stride)
    for row in range(height):
        start = row * (stride + 1)
        if raw[start] != 0:
            raise ValueError("only filter type 0 is supported")
        out[row * stride:(row + 1) * stride] = raw[start + 1:start + 1 + stride]
    return width, height, bytes(out)


def frame_luma_columns(rgb: bytes, width: int, height: int, step: int = 4) -> list[list[int]]:
    """Sub-sampled luma rows, kept as ints to stay cheap without numpy."""
    rows = []
    stride = width * 3
    for y in range(0, height, step):
        base = y * stride
        row = []
        for x in range(0, width, step):
            index = base + x * 3
            row.append((rgb[index] * 299 + rgb[index + 1] * 587 + rgb[index + 2] * 114) // 1000)
        rows.append(row)
    return rows


@dataclass
class Observation:
    index: int
    pixel_x: float
    pixel_fraction: float
    changed_pixels: int
    lidar_angle_deg: float
    lidar_distance_m: float
    lidar_bins: int


def circular_mean(angles_deg: Iterable[float]) -> float:
    angles = list(angles_deg)
    sx = sum(math.sin(math.radians(a)) for a in angles)
    sy = sum(math.cos(math.radians(a)) for a in angles)
    return math.degrees(math.atan2(sx, sy))


def wrap180(value: float) -> float:
    return ((value + 180.0) % 360.0) - 180.0


def lidar_clusters(bin_min: dict[int, float], background: dict[int, float], drop_m: float, max_m: float):
    hits = sorted(
        key for key, value in bin_min.items()
        if background.get(key, 0.0) > drop_m and value < background[key] - drop_m and value < max_m
    )
    if not hits:
        return []
    groups: list[list[int]] = []
    current = [hits[0]]
    for key in hits[1:]:
        if key - current[-1] > 1:
            groups.append(current)
            current = []
        current.append(key)
    groups.append(current)
    if len(groups) > 1:
        lowest, highest = min(bin_min), max(bin_min)
        if groups[0][0] == lowest and groups[-1][-1] == highest:
            groups[0] = groups[-1] + groups[0]
            groups.pop()
    clusters = []
    for group in groups:
        angles = [key * BIN_DEG + BIN_DEG / 2.0 for key in group]
        clusters.append(
            {
                "angle_deg": circular_mean(angles),
                "distance_m": min(bin_min[key] for key in group),
                "bins": len(group),
            }
        )
    clusters.sort(key=lambda item: item["distance_m"])
    return clusters


def load_samples(session: Path) -> list[dict]:
    return [json.loads(line) for line in (session / "samples.jsonl").read_text().splitlines() if line.strip()]


def lidar_bin_minima(sample: dict) -> dict[int, float]:
    result: dict[int, float] = {}
    for angle, distance in sample["points"]:
        if distance <= 0.0:
            continue
        key = int(math.floor(angle / BIN_DEG))
        if key not in result or distance < result[key]:
            result[key] = distance
    return result


def grid_search(candidates: list[list[Observation]], width: int, tolerance_deg: float) -> list[Observation]:
    """Pick one LiDAR cluster per sample with a RANSAC-style vote.

    A moving person is often not the nearest return (the LiDAR may sit against
    a wall or a mount), and in half the samples they stand behind the camera and
    have no pixel evidence at all. So every sample offers several candidate
    clusters, and the mapping that explains the most of them wins.
    """
    best: list[Observation] = []
    for axis_deg in range(-180, 180, 2):
        for scale in [s / 10000.0 for s in range(-1600, 1601, 25)]:
            if abs(scale) < 0.02:
                continue
            chosen: list[Observation] = []
            for options in candidates:
                pick = None
                for observation in options:
                    predicted = axis_deg + scale * (observation.pixel_x - width / 2.0)
                    error = abs(wrap180(observation.lidar_angle_deg - predicted))
                    if error <= tolerance_deg and (pick is None or error < pick[0]):
                        pick = (error, observation)
                if pick is not None:
                    chosen.append(pick[1])
            if len(chosen) > len(best):
                best = chosen
    return best


def fit_mapping(observations: list[Observation], width: int) -> Optional[dict]:
    """Least squares on the unwrapped angle, anchored at the circular mean."""
    if len(observations) < 4:
        return None
    anchor = circular_mean(obs.lidar_angle_deg for obs in observations)
    xs = [obs.pixel_x - width / 2.0 for obs in observations]
    ys = [wrap180(obs.lidar_angle_deg - anchor) for obs in observations]
    mean_x = statistics.fmean(xs)
    mean_y = statistics.fmean(ys)
    denominator = sum((x - mean_x) ** 2 for x in xs)
    if denominator == 0.0:
        return None
    scale = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denominator
    intercept = mean_y - scale * mean_x
    residuals = [y - (intercept + scale * x) for x, y in zip(xs, ys)]
    rms = math.sqrt(statistics.fmean(r * r for r in residuals))
    return {
        "forward_angle_deg": round(wrap180(anchor + intercept), 2),
        "degrees_per_pixel": round(scale, 5),
        "horizontal_fov_deg": round(abs(scale) * width, 2),
        "flipped": scale < 0.0,
        "samples": len(observations),
        "residual_rms_deg": round(rms, 2),
        "max_residual_deg": round(max(abs(r) for r in residuals), 2),
    }


def analyse(session: Path, args: argparse.Namespace) -> dict:
    samples = load_samples(session)
    if not samples:
        raise SystemExit(f"no samples in {session}")

    frames = []
    width = height = 0
    for sample in samples:
        width, height, rgb = decode_png_rgb((session / sample["frame"]).read_bytes())
        frames.append(frame_luma_columns(rgb, width, height, args.pixel_step))

    rows = len(frames[0])
    columns = len(frames[0][0])
    background_px = [
        [statistics.median(frame[y][x] for frame in frames) for x in range(columns)]
        for y in range(rows)
    ]

    minima = [lidar_bin_minima(sample) for sample in samples]
    keys = sorted({key for entry in minima for key in entry})
    background_lidar = {
        key: statistics.median([entry[key] for entry in minima if key in entry]) for key in keys
    }

    candidates: list[list[Observation]] = []
    per_sample = []
    for sample, frame, bin_min in zip(samples, frames, minima):
        changed = [
            (x, abs(frame[y][x] - background_px[y][x]))
            for y in range(rows)
            for x in range(columns)
            if abs(frame[y][x] - background_px[y][x]) >= args.pixel_threshold
        ]
        clusters = lidar_clusters(bin_min, background_lidar, args.drop_m, args.max_range_m)
        clusters = [c for c in clusters if c["distance_m"] >= args.min_object_m]
        entry = {
            "index": sample["index"],
            "changed_pixels": len(changed),
            "pixel_fraction": round(len(changed) / (rows * columns), 4),
            "clusters": [
                {
                    "angle_deg": round(cluster["angle_deg"], 1),
                    "distance_m": round(cluster["distance_m"], 2),
                    "bins": cluster["bins"],
                }
                for cluster in clusters[:3]
            ],
        }
        if changed:
            weight = sum(value for _, value in changed)
            centre = sum(x * value for x, value in changed) / weight
            entry["pixel_x"] = round(centre * args.pixel_step, 1)
        per_sample.append(entry)

        if "pixel_x" in entry and entry["pixel_fraction"] >= args.min_pixel_fraction:
            options = [
                Observation(
                    index=sample["index"],
                    pixel_x=entry["pixel_x"],
                    pixel_fraction=entry["pixel_fraction"],
                    changed_pixels=len(changed),
                    lidar_angle_deg=cluster["angle_deg"],
                    lidar_distance_m=cluster["distance_m"],
                    lidar_bins=cluster["bins"],
                )
                for cluster in clusters
                if cluster["bins"] >= args.min_bins
                and args.min_object_m <= cluster["distance_m"] <= args.max_range_m
            ]
            if options:
                candidates.append(options)

    observations = grid_search(candidates, width, args.outlier_deg)
    fit = fit_mapping(observations, width)
    if fit is not None and args.reject_outliers and len(observations) >= 8:
        anchor = circular_mean(obs.lidar_angle_deg for obs in observations)
        keep = []
        for obs in observations:
            predicted = fit["forward_angle_deg"] + fit["degrees_per_pixel"] * (obs.pixel_x - width / 2.0)
            if abs(wrap180(obs.lidar_angle_deg - predicted)) <= args.outlier_deg:
                keep.append(obs)
        if len(keep) >= 4 and len(keep) < len(observations):
            refit = fit_mapping(keep, width)
            if refit is not None:
                refit["rejected"] = len(observations) - len(keep)
                fit = refit
                observations = keep

    # Bins that stay closer than the session median are static self-obstruction.
    static = []
    for key in keys:
        values = [entry[key] for entry in minima if key in entry]
        if len(values) < len(samples) * 0.6:
            continue
        if statistics.median(values) <= args.static_range_m:
            static.append(
                {
                    "angle_deg": key * BIN_DEG + BIN_DEG / 2.0,
                    "median_m": round(statistics.median(values), 3),
                    "seen_fraction": round(len(values) / len(samples), 2),
                }
            )

    return {
        "session": str(session),
        "frame_size": [width, height],
        "samples": len(samples),
        "used_observations": len(observations),
        "fit": fit,
        "static_returns": static,
        "per_sample": per_sample,
        "observations": [obs.__dict__ for obs in observations],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fit the camera/LiDAR bearing mapping of a twin session")
    parser.add_argument("session", help="session directory written by twin_capture.py")
    parser.add_argument("--pixel-step", type=int, default=4, help="pixel sub-sampling stride")
    parser.add_argument("--pixel-threshold", type=int, default=25, help="luma difference that counts as changed")
    parser.add_argument("--min-pixel-fraction", type=float, default=0.04, help="minimum changed area to use a sample")
    parser.add_argument("--drop-m", type=float, default=0.4, help="distance drop that counts as a LiDAR detection")
    parser.add_argument("--max-range-m", type=float, default=4.0, help="ignore detections beyond this range")
    parser.add_argument("--min-bins", type=int, default=2, help="minimum 5-degree bins in a LiDAR cluster")
    parser.add_argument("--min-object-m", type=float, default=0.5, help="ignore returns closer than this (mount, cables)")
    parser.add_argument("--static-range-m", type=float, default=0.6, help="range under which a fixed return is self-obstruction")
    parser.add_argument("--reject-outliers", action="store_true", default=True, help="refit after dropping outliers")
    parser.add_argument("--outlier-deg", type=float, default=25.0, help="residual above which a sample is dropped")
    parser.add_argument("--json", default=None, help="write the full report to this file")
    parser.add_argument(
        "--calibration-out",
        default=None,
        help=f"where to write the calibration (default <session>/{CALIBRATION_FILENAME})",
    )
    parser.add_argument("--no-calibration", action="store_true", help="only report, do not write a calibration")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    report = analyse(Path(args.session), args)
    fit = report["fit"]
    print(f"session {report['session']}: {report['samples']} samples, {report['used_observations']} usable")
    if fit is None:
        print("not enough paired camera/LiDAR motion to fit a mapping")
    else:
        print(
            f"camera axis  = {fit['forward_angle_deg']:+.1f} deg LiDAR bearing\n"
            f"scale        = {fit['degrees_per_pixel']:+.4f} deg/px "
            f"(horizontal FOV {fit['horizontal_fov_deg']:.1f} deg, "
            f"{'mirrored' if fit['flipped'] else 'same handedness'})\n"
            f"fit quality  = {fit['residual_rms_deg']:.1f} deg RMS, "
            f"max {fit['max_residual_deg']:.1f} deg over {fit['samples']} samples"
        )
    if report["static_returns"]:
        print("static close returns (self-obstruction candidates):")
        for entry in report["static_returns"]:
            print(f"  {entry['angle_deg']:+7.1f} deg  {entry['median_m']:.2f} m  seen {entry['seen_fraction']*100:.0f}%")
    if args.json:
        Path(args.json).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"report written to {args.json}")

    if fit is not None and not args.no_calibration:
        medians = {entry["angle_deg"]: entry["median_m"] for entry in report["static_returns"]}
        calibration = Calibration(
            camera_axis_deg=fit["forward_angle_deg"],
            degrees_per_pixel=fit["degrees_per_pixel"],
            frame_width=report["frame_size"][0],
            horizontal_fov_deg=fit["horizontal_fov_deg"],
            residual_rms_deg=fit["residual_rms_deg"],
            samples=fit["samples"],
            blind_sectors=merge_sectors(medians.keys(), BIN_DEG, medians),
            source=report["session"],
        )
        written = calibration.save(args.calibration_out or Path(report["session"]))
        print(f"calibration written to {written}")
        for sector in calibration.blind_sectors:
            print(f"  blind sector {sector.start_deg:+7.1f}..{sector.end_deg:+7.1f} deg at {sector.median_m:.2f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
