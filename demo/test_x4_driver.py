from __future__ import annotations

import ctypes
import math
import os
import unittest
from pathlib import Path

from demo.x4_driver import (
    LaserConfig,
    LaserFan,
    LaserPoint,
    LidarSettings,
    ScanFrame,
    SimulatedX4,
    YdlidarSdk,
)
from demo.x4_visualizer import angle_in_sector, valid_points


class AbiLayoutTests(unittest.TestCase):
    def test_packed_c_structures_match_sdk(self) -> None:
        self.assertEqual(ctypes.sizeof(LaserPoint), 12)
        self.assertEqual(ctypes.sizeof(LaserConfig), 28)
        self.assertEqual(ctypes.sizeof(LaserFan), 40 + ctypes.sizeof(ctypes.c_void_p))


class SettingsTests(unittest.TestCase):
    def test_x4_profile_accepts_documented_limits(self) -> None:
        LidarSettings(scan_frequency_hz=5.0, min_range_m=0.12, max_range_m=10.0).validate()
        LidarSettings(scan_frequency_hz=12.0).validate()

    def test_invalid_frequency_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            LidarSettings(scan_frequency_hz=12.1).validate()


class SimulatorTests(unittest.TestCase):
    def test_simulator_produces_realistic_scan(self) -> None:
        settings = LidarSettings(scan_frequency_hz=8.0)
        with SimulatedX4(settings, realtime=False) as source:
            frame = source.read_scan()
        valid = [distance for distance in frame.ranges_m if distance > 0.0]
        self.assertGreater(frame.point_count, 500)
        self.assertGreater(len(valid), 450)
        self.assertLess(min(valid), 2.0)
        self.assertLessEqual(max(valid), settings.max_range_m)
        self.assertAlmostEqual(frame.scan_frequency_hz, 8.0)


class RegionOfInterestTests(unittest.TestCase):
    def test_sector_handles_wraparound(self) -> None:
        self.assertTrue(angle_in_sector(math.radians(179.0), -179.0, 10.0))
        self.assertFalse(angle_in_sector(math.radians(160.0), -179.0, 10.0))

    def test_valid_points_filters_to_selected_sector(self) -> None:
        frame = ScanFrame(
            stamp_ns=1,
            angles_rad=(math.radians(-125.0), math.radians(20.0)),
            ranges_m=(4.0, 0.2),
            intensities=(0.0, 0.0),
            scan_frequency_hz=8.0,
            angle_increment_rad=0.01,
            time_increment_s=0.0002,
        )
        points = valid_points(frame, LidarSettings(), -125.0, 70.0)
        self.assertEqual(points, [(math.radians(-125.0), 4.0)])


@unittest.skipUnless(os.name == "nt", "Windows DLL integration test")
class NativeSdkTests(unittest.TestCase):
    def test_built_sdk_loads_and_exports_c_api(self) -> None:
        dll = Path(__file__).resolve().parents[1] / "build-x4" / "ydlidar_sdk.dll"
        if not dll.is_file():
            self.skipTest("native SDK has not been built")
        sdk = YdlidarSdk(dll)
        self.assertEqual(sdk.version, "1.2.20")
        self.assertIsInstance(sdk.list_ports(), tuple)


if __name__ == "__main__":
    unittest.main()
