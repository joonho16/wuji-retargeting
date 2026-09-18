"""Compile and check actual C++ reference limits without ROS or robot hardware."""
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


class AccelerationLimiterTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("g++"), "C++ compiler required")
    def test_position_reference_velocity_and_acceleration_bounds(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "test_limiter"
            subprocess.run([
                "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-O2",
                "-I", str(root / "docker/gen3-vive/servo-overlay"),
                str(root / "tests/cpp/test_gen3_acceleration_limiter.cpp"), "-o", str(executable),
            ], check=True, capture_output=True, text=True)
            result = subprocess.run([str(executable)], check=True, capture_output=True, text=True)
            self.assertIn("15000 reference steps", result.stdout)
