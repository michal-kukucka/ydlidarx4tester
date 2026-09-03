from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from demo.calibration import (
    BlindSector,
    Calibration,
    cluster_obstacles,
    merge_sectors,
    nearest_obstacle,
    wrap180,
)
from demo.calibrate_twin import decode_png_rgb, fit_mapping, Observation
from demo.camera_stream import CameraFrame, encode_png, list_cameras
from demo.twin_capture import MotionGate, Sample, SessionWriter
from demo.x4_driver import ScanFrame


class PngRoundTripTests(unittest.TestCase):
    def test_encoded_frame_decodes_to_the_same_pixels(self) -> None:
        width, height = 7, 5
        rgb = bytes((x * 3 + y) % 256 for y in range(height) for x in range(width * 3))
        decoded_width, decoded_height, decoded = decode_png_rgb(encode_png(rgb, width, height))
        self.assertEqual((decoded_width, decoded_height), (width, height))
        self.assertEqual(decoded, rgb)

    def test_compression_level_does_not_change_pixels(self) -> None:
        rgb = bytes(range(48))
        fast = decode_png_rgb(encode_png(rgb, 4, 4, compression=0))[2]
        small = decode_png_rgb(encode_png(rgb, 4, 4, compression=9))[2]
        self.assertEqual(fast, small)


class BlindSectorTests(unittest.TestCase):
    def test_sector_across_the_seam_contains_180(self) -> None:
        sector = BlindSector(170.0, -170.0)
        self.assertTrue(sector.contains(180.0))
        self.assertTrue(sector.contains(-175.0))
        self.assertFalse(sector.contains(0.0))

    def test_merge_joins_neighbouring_bins_and_pads(self) -> None:
        medians = {27.5: 0.15, 32.5: 0.15, 37.5: 0.14}
        sectors = merge_sectors(medians.keys(), 5.0, medians, pad_deg=5.0)
        self.assertEqual(len(sectors), 1)
        self.assertAlmostEqual(sectors[0].start_deg, 20.0)
        self.assertAlmostEqual(sectors[0].end_deg, 45.0)

    def test_merge_bridges_an_intermittent_gap(self) -> None:
        medians = {10.0: 0.2, 15.0: 0.2, 25.0: 0.2}
        sectors = merge_sectors(medians.keys(), 5.0, medians, gap_bins=2.5, pad_deg=0.0)
        self.assertEqual(len(sectors), 1)


class ClusterTests(unittest.TestCase):
    def test_single_stray_return_is_not_an_obstacle(self) -> None:
        points = [(0.0, 0.3)] + [(angle, 3.0) for angle in range(10, 40, 2)]
        obstacles = cluster_obstacles(points, min_points=3)
        self.assertTrue(all(obstacle.distance_m > 1.0 for obstacle in obstacles))

    def test_person_in_front_of_a_wall_stays_separate(self) -> None:
        # One return per bearing, as a real revolution reports: the person
        # occludes the wall over the bearings they cover.
        wall = [(float(angle), 3.0) for angle in range(-60, 61, 2) if abs(angle) > 6]
        person = [(float(angle), 1.0) for angle in range(-6, 7, 2)]
        obstacles = cluster_obstacles(wall + person, min_points=3)
        self.assertGreaterEqual(len(obstacles), 2)
        self.assertAlmostEqual(obstacles[0].distance_m, 1.0)
        self.assertLessEqual(abs(obstacles[0].angle_deg), 2.0)

    def test_cluster_across_the_seam_is_one_obstacle(self) -> None:
        points = [(float(angle), 1.0) for angle in (174, 176, 178, -178, -176, -174)]
        obstacles = cluster_obstacles(points, min_points=3)
        self.assertEqual(len(obstacles), 1)
        self.assertGreater(abs(obstacles[0].angle_deg), 170.0)

    def test_blind_sector_returns_are_ignored(self) -> None:
        calibration = Calibration(blind_sectors=[BlindSector(20.0, 120.0, 0.15)])
        mount = [(float(angle), 0.15) for angle in range(30, 100, 2)]
        target = [(float(angle), 2.0) for angle in range(-10, 11, 2)]
        obstacle = nearest_obstacle(mount + target, calibration=calibration)
        self.assertIsNotNone(obstacle)
        self.assertAlmostEqual(obstacle.distance_m, 2.0)

    def test_sector_restriction_keeps_overlapping_obstacles(self) -> None:
        points = [(float(angle), 1.5) for angle in range(80, 101, 2)]
        self.assertIsNotNone(nearest_obstacle(points, sector_centre_deg=90.0, sector_width_deg=40.0))
        self.assertIsNone(nearest_obstacle(points, sector_centre_deg=-90.0, sector_width_deg=40.0))


class CalibrationFileTests(unittest.TestCase):
    def test_round_trip_through_json(self) -> None:
        calibration = Calibration(
            camera_axis_deg=179.19,
            degrees_per_pixel=0.0784,
            frame_width=640,
            horizontal_fov_deg=50.2,
            blind_sectors=[BlindSector(20.0, 120.0, 0.15)],
            source="recordings/train_01",
        )
        with tempfile.TemporaryDirectory() as directory:
            path = calibration.save(Path(directory))
            self.assertEqual(json.loads(path.read_text())["frame_width"], 640)
            loaded = Calibration.load(Path(directory))
        self.assertAlmostEqual(loaded.camera_axis_deg, 179.19)
        self.assertEqual(len(loaded.blind_sectors), 1)
        self.assertTrue(loaded.is_blind(60.0))

    def test_pixel_and_angle_are_inverse(self) -> None:
        calibration = Calibration(camera_axis_deg=179.19, degrees_per_pixel=0.0784, frame_width=640)
        angle = calibration.pixel_to_angle(500.0)
        self.assertAlmostEqual(calibration.angle_to_pixel(angle), 500.0, places=3)

    def test_axis_near_the_seam_maps_both_edges(self) -> None:
        calibration = Calibration(camera_axis_deg=179.19, degrees_per_pixel=0.0784, frame_width=640)
        left = calibration.pixel_to_angle(0.0)
        right = calibration.pixel_to_angle(640.0)
        self.assertGreater(left, 150.0)
        self.assertLess(right, -150.0)


class FitTests(unittest.TestCase):
    def test_fit_recovers_a_mapping_that_straddles_180(self) -> None:
        axis, scale, width = 179.0, 0.08, 640
        observations = [
            Observation(
                index=index,
                pixel_x=float(pixel),
                pixel_fraction=0.1,
                changed_pixels=100,
                lidar_angle_deg=wrap180(axis + scale * (pixel - width / 2.0)),
                lidar_distance_m=1.5,
                lidar_bins=4,
            )
            for index, pixel in enumerate(range(40, 620, 40))
        ]
        fit = fit_mapping(observations, width)
        self.assertIsNotNone(fit)
        self.assertLess(abs(wrap180(fit["forward_angle_deg"] - axis)), 0.5)
        self.assertAlmostEqual(fit["degrees_per_pixel"], scale, places=3)
        self.assertLess(fit["residual_rms_deg"], 0.5)

    def test_fit_needs_enough_observations(self) -> None:
        self.assertIsNone(fit_mapping([], 640))


def make_frame(value: int, width: int = 64, height: int = 48, patch: int = 0) -> CameraFrame:
    rgb = bytearray([value]) * (width * height * 3)
    for y in range(patch):
        for x in range(patch):
            index = (y * width + x) * 3
            rgb[index:index + 3] = b"\xff\xff\xff"
    return CameraFrame(stamp_ns=0, rgb=bytes(rgb), width=width, height=height, index=1)


def make_scan(distance: float, points: int = 360) -> ScanFrame:
    angles = tuple(math.radians(-180.0 + 360.0 * index / points) for index in range(points))
    return ScanFrame(
        stamp_ns=0,
        angles_rad=angles,
        ranges_m=(distance,) * points,
        intensities=(0.0,) * points,
        scan_frequency_hz=8.0,
        angle_increment_rad=2.0 * math.pi / points,
        time_increment_s=0.0,
    )


class MotionGateTests(unittest.TestCase):
    def test_first_sample_is_always_written(self) -> None:
        gate = MotionGate(18.0, 0.15, 0.0)
        self.assertTrue(gate.should_write(Sample(1, make_scan(2.0), make_frame(100)), 0.0))

    def test_a_still_scene_is_skipped(self) -> None:
        gate = MotionGate(18.0, 0.15, 0.0)
        gate.should_write(Sample(1, make_scan(2.0), make_frame(100)), 0.0)
        for index in range(2, 6):
            written = gate.should_write(Sample(index, make_scan(2.0), make_frame(100)), float(index))
            self.assertFalse(written)
        self.assertEqual(gate.skipped, 4)

    def test_sensor_noise_below_the_cell_count_is_not_motion(self) -> None:
        gate = MotionGate(18.0, 0.15, 0.0, min_cells=12)
        gate.should_write(Sample(1, make_scan(2.0), make_frame(100)), 0.0)
        noisy = make_frame(100, patch=2)  # a handful of cells, as webcam grain looks
        self.assertFalse(gate.should_write(Sample(2, make_scan(2.0), noisy), 1.0))

    def test_camera_motion_is_written(self) -> None:
        gate = MotionGate(18.0, 0.15, 0.0)
        gate.should_write(Sample(1, make_scan(2.0), make_frame(100)), 0.0)
        self.assertTrue(gate.should_write(Sample(2, make_scan(2.0), make_frame(200)), 1.0))

    def test_lidar_motion_alone_is_written(self) -> None:
        gate = MotionGate(18.0, 0.15, 0.0)
        gate.should_write(Sample(1, make_scan(2.0), make_frame(100)), 0.0)
        self.assertTrue(gate.should_write(Sample(2, make_scan(1.0), make_frame(100)), 1.0))

    def test_keepalive_stores_an_idle_sample(self) -> None:
        gate = MotionGate(18.0, 0.15, keepalive_s=10.0)
        gate.should_write(Sample(1, make_scan(2.0), make_frame(100)), 0.0)
        self.assertFalse(gate.should_write(Sample(2, make_scan(2.0), make_frame(100)), 5.0))
        self.assertTrue(gate.should_write(Sample(3, make_scan(2.0), make_frame(100)), 10.0))


class SessionBudgetTests(unittest.TestCase):
    def test_writer_stops_at_its_budget(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            writer = SessionWriter(Path(directory), {}, budget_bytes=4096, min_free_bytes=0)
            self.assertTrue(writer.has_room())
            writer.write_sample(Sample(1, make_scan(2.0), make_frame(100)))
            self.assertFalse(writer.has_room())
            self.assertIn("budget", writer.stopped_reason)

    def test_writer_stops_when_the_volume_is_nearly_full(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            writer = SessionWriter(Path(directory), {}, min_free_bytes=1 << 62)
            self.assertFalse(writer.has_room())
            self.assertIn("free", writer.stopped_reason)

    def test_unbudgeted_writer_keeps_room(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            writer = SessionWriter(Path(directory), {}, budget_bytes=0, min_free_bytes=0)
            writer.write_sample(Sample(1, make_scan(2.0), make_frame(100)))
            self.assertTrue(writer.has_room())
            self.assertEqual(writer.count, 1)


class CameraListingTests(unittest.TestCase):
    def test_listing_cameras_never_raises_on_a_missing_device(self) -> None:
        try:
            names = list_cameras()
        except Exception as error:  # pragma: no cover - ffmpeg may be absent in CI
            self.skipTest(f"camera listing unavailable: {error}")
        self.assertIsInstance(names, tuple)
        for name in names:
            self.assertFalse(name.startswith("["), f"index prefix leaked into {name!r}")


if __name__ == "__main__":
    unittest.main()
