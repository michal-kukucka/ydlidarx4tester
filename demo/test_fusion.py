"""Camera detection through the Rozeta C ABI, and its pairing with the LiDAR.

The detector tests need the built library; they skip when it is missing, which
is the same rule the rest of the suite follows for hardware-backed pieces. The
fusion tests are pure arithmetic and always run.
"""

from __future__ import annotations

import unittest

from demo.calibration import BlindSector, Calibration, wrap180
from demo.camera_stream import CameraFrame
from demo.fusion import (
    AGREEMENT_BOTH,
    AGREEMENT_CAMERA_ONLY,
    AGREEMENT_CLEAR,
    AGREEMENT_LIDAR_ONLY,
    SOURCE_CAMERA,
    SOURCE_LIDAR,
    camera_bearing,
    fuse,
    match_lidar,
)
from demo.rgb_obstacle import CLEAR, TRIGGERED, CameraObstacle, RgbObstacleError, RgbObstacleTracker


def calibration() -> Calibration:
    """The recorded X4/webcam geometry, rounded: axis behind, 50-degree view."""
    return Calibration(
        camera_axis_deg=179.2,
        degrees_per_pixel=0.0784,
        frame_width=640,
        horizontal_fov_deg=50.2,
        blind_sectors=[BlindSector(-25.0, -5.0)],
    )


def detection(
    state: int = TRIGGERED,
    box: tuple[int, int, int, int] | None = (300, 200, 40, 120),
) -> CameraObstacle:
    return CameraObstacle(
        state=state,
        dark_coverage=0.2,
        diff_coverage=0.3,
        obstacle_count=1,
        area_fraction=0.05,
        box=box,
        streak=5,
        source="diff",
        ok=True,
    )


def frame(width: int, height: int, fill: int, index: int = 0) -> CameraFrame:
    return CameraFrame(
        stamp_ns=index,
        rgb=bytes([fill]) * (width * height * 3),
        width=width,
        height=height,
        index=index,
    )


def frame_with_patch(width: int, height: int, background: int, patch: int, index: int = 0) -> CameraFrame:
    """A background with a solid block over the middle third of the frame."""
    pixels = bytearray([background]) * (width * height * 3)
    for y in range(height // 3, 2 * height // 3):
        for x in range(width // 3, 2 * width // 3):
            base = (y * width + x) * 3
            pixels[base : base + 3] = bytes([patch, patch, patch])
    return CameraFrame(stamp_ns=index, rgb=bytes(pixels), width=width, height=height, index=index)


class CameraBearingTests(unittest.TestCase):
    def test_a_box_at_the_frame_centre_sits_on_the_camera_axis(self) -> None:
        bearing, width = camera_bearing(detection(box=(300, 0, 40, 100)), calibration())
        self.assertAlmostEqual(bearing, 179.2, places=3)
        self.assertAlmostEqual(width, 40 * 0.0784, places=4)

    def test_a_box_on_the_right_moves_the_bearing_clockwise(self) -> None:
        # Past +180 the bearing wraps, so the offset from the axis is the test.
        bearing, _width = camera_bearing(detection(box=(500, 0, 40, 100)), calibration())
        self.assertAlmostEqual(wrap180(bearing - 179.2), 200 * 0.0784, places=3)

    def test_a_detection_without_a_box_has_no_bearing(self) -> None:
        self.assertEqual(camera_bearing(detection(box=None), calibration()), (None, None))

    def test_without_a_calibration_there_is_no_bearing(self) -> None:
        self.assertEqual(camera_bearing(detection(), None), (None, None))


class MatchTests(unittest.TestCase):
    def test_the_closest_bearing_within_tolerance_wins(self) -> None:
        from demo.calibration import Obstacle

        far = Obstacle(angle_deg=160.0, distance_m=1.0, width_deg=4.0, points=8)
        near = Obstacle(angle_deg=176.0, distance_m=3.0, width_deg=4.0, points=8)
        matched, error = match_lidar(179.0, [far, near], tolerance_deg=12.0)
        self.assertIs(matched, near)
        self.assertAlmostEqual(error, 3.0, places=3)

    def test_a_wide_cluster_is_matched_from_further_away(self) -> None:
        from demo.calibration import Obstacle

        wide = Obstacle(angle_deg=160.0, distance_m=1.0, width_deg=30.0, points=40)
        self.assertIsNotNone(match_lidar(179.0, [wide], tolerance_deg=5.0)[0])

    def test_nothing_within_tolerance_is_no_match(self) -> None:
        from demo.calibration import Obstacle

        away = Obstacle(angle_deg=90.0, distance_m=1.0, width_deg=2.0, points=8)
        self.assertEqual(match_lidar(179.0, [away], tolerance_deg=12.0), (None, None))


class FuseTests(unittest.TestCase):
    @staticmethod
    def cluster_at(angle_deg: float, distance_m: float, count: int = 6) -> list[tuple[float, float]]:
        return [(angle_deg + step * 0.5, distance_m) for step in range(count)]

    def test_camera_and_lidar_agreeing_gives_a_distance(self) -> None:
        result = fuse(detection(), self.cluster_at(178.0, 2.4), calibration())
        self.assertEqual(result.agreement, AGREEMENT_BOTH)
        self.assertTrue(result.confirmed)
        self.assertAlmostEqual(result.distance_m, 2.4, places=3)
        self.assertEqual(result.bearing_source, SOURCE_CAMERA)
        self.assertIn("camera +", result.describe())

    def test_a_camera_detection_with_no_return_stays_unranged(self) -> None:
        result = fuse(detection(), self.cluster_at(20.0, 2.4), calibration())
        self.assertEqual(result.agreement, AGREEMENT_CAMERA_ONLY)
        self.assertIsNone(result.distance_m)

    def test_a_quiet_camera_still_reports_the_lidar_cluster(self) -> None:
        result = fuse(
            detection(state=CLEAR),
            self.cluster_at(178.0, 1.1),
            calibration(),
            sector_centre_deg=179.2,
            sector_width_deg=50.0,
        )
        self.assertEqual(result.agreement, AGREEMENT_LIDAR_ONLY)
        self.assertAlmostEqual(result.distance_m, 1.1, places=3)

    def test_a_cluster_outside_the_sector_is_not_reported_alone(self) -> None:
        result = fuse(
            detection(state=CLEAR),
            self.cluster_at(20.0, 1.1),
            calibration(),
            sector_centre_deg=179.2,
            sector_width_deg=50.0,
        )
        self.assertEqual(result.agreement, AGREEMENT_CLEAR)

    def test_blind_sector_returns_never_become_an_obstacle(self) -> None:
        result = fuse(
            detection(state=CLEAR),
            self.cluster_at(-20.0, 0.3),
            calibration(),
            sector_centre_deg=-15.0,
            sector_width_deg=60.0,
        )
        self.assertEqual(result.agreement, AGREEMENT_CLEAR)

    def test_a_trigger_without_a_box_falls_back_to_the_sector(self) -> None:
        result = fuse(
            detection(box=None),
            self.cluster_at(178.0, 2.0),
            calibration(),
            sector_centre_deg=179.2,
            sector_width_deg=50.0,
        )
        self.assertEqual(result.agreement, AGREEMENT_BOTH)
        self.assertAlmostEqual(result.distance_m, 2.0, places=3)
        # The bearing is the LiDAR's, and must not be reported as the camera's.
        self.assertEqual(result.bearing_source, SOURCE_LIDAR)
        self.assertIn("camera has no bearing", result.describe())

    def test_without_the_camera_the_lidar_alone_still_reports(self) -> None:
        result = fuse(None, self.cluster_at(178.0, 2.0), calibration())
        self.assertEqual(result.agreement, AGREEMENT_LIDAR_ONLY)
        self.assertEqual(result.bearing_source, SOURCE_LIDAR)

    def test_the_record_is_json_ready(self) -> None:
        import json

        payload = fuse(detection(), self.cluster_at(178.0, 2.4), calibration()).to_dict()
        self.assertEqual(json.loads(json.dumps(payload))["agreement"], AGREEMENT_BOTH)


def tracker_or_skip(**overrides) -> RgbObstacleTracker:
    try:
        return RgbObstacleTracker.open(**overrides)
    except RgbObstacleError as error:
        raise unittest.SkipTest(f"Rozeta RGB obstacle ABI unavailable: {error}")


class RgbObstacleTrackerTests(unittest.TestCase):
    """The detector is Rozeta's; these check the binding and the hysteresis."""

    def test_defaults_come_from_the_library(self) -> None:
        with tracker_or_skip() as tracker:
            self.assertEqual(tracker.config.trigger_streak, 5)
            self.assertEqual(tracker.config.clear_streak, 3)

    def test_an_unchanged_frame_stays_clear(self) -> None:
        with tracker_or_skip() as tracker:
            background = frame(64, 48, 200)
            for _ in range(10):
                result = tracker.update_ref(background, background)
            self.assertEqual(result.state, CLEAR)
            self.assertAlmostEqual(result.diff_coverage, 0.0, places=6)

    def test_a_new_object_triggers_after_the_streak(self) -> None:
        with tracker_or_skip() as tracker:
            background = frame(64, 48, 200)
            occupied = frame_with_patch(64, 48, 200, 20)
            states = [tracker.update_ref(occupied, background).state for _ in range(5)]
            self.assertEqual(states[0], CLEAR)
            self.assertEqual(states[-1], TRIGGERED)
            self.assertGreater(tracker.result.diff_coverage, 0.0)

    def test_the_object_leaving_clears_the_tracker(self) -> None:
        with tracker_or_skip() as tracker:
            background = frame(64, 48, 200)
            occupied = frame_with_patch(64, 48, 200, 20)
            for _ in range(6):
                tracker.update_ref(occupied, background)
            self.assertEqual(tracker.result.state, TRIGGERED)
            for _ in range(4):
                tracker.update_ref(background, background)
            self.assertEqual(tracker.result.state, CLEAR)

    def test_a_dark_object_reports_a_box(self) -> None:
        with tracker_or_skip() as tracker:
            background = frame(64, 48, 200)
            occupied = frame_with_patch(64, 48, 200, 5)
            for _ in range(5):
                result = tracker.update_ref(occupied, background)
            self.assertIsNotNone(result.box)
            self.assertGreater(result.area_fraction, 0.0)

    def test_a_new_reference_forgets_the_old_streak(self) -> None:
        with tracker_or_skip() as tracker:
            background = frame(64, 48, 200)
            occupied = frame_with_patch(64, 48, 200, 20)
            for _ in range(6):
                tracker.update_ref(occupied, background)
            self.assertEqual(tracker.result.state, TRIGGERED)
            tracker.reset()
            self.assertEqual(tracker.result.state, CLEAR)

    def test_mismatched_reference_size_is_refused(self) -> None:
        with tracker_or_skip() as tracker:
            with self.assertRaises(RgbObstacleError):
                tracker.update_ref(frame(64, 48, 200), frame(32, 24, 200))

    def test_an_out_of_range_setting_is_refused(self) -> None:
        try:
            with self.assertRaises(RgbObstacleError):
                RgbObstacleTracker.open(coverage_threshold=5.0)
        except RgbObstacleError as error:
            raise unittest.SkipTest(f"Rozeta RGB obstacle ABI unavailable: {error}")

    def test_a_detection_from_the_library_can_be_fused(self) -> None:
        with tracker_or_skip() as tracker:
            background = frame(640, 480, 200)
            occupied = frame_with_patch(640, 480, 200, 5)
            for _ in range(5):
                result = tracker.update_ref(occupied, background)
            fused = fuse(result, [(179.0 + i * 0.5, 2.0) for i in range(6)], calibration())
            self.assertEqual(fused.agreement, AGREEMENT_BOTH)
            self.assertAlmostEqual(fused.distance_m, 2.0, places=3)


if __name__ == "__main__":
    unittest.main()
