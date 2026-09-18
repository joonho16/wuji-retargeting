"""Inject the observed SDK exception types without any SDK connection."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class CyclicFailureGuardTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("g++"), "C++ compiler required")
    def test_failed_session_never_retries_or_resumes(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "test_cyclic_guard"
            subprocess.run([
                "g++", "-std=c++14", "-Wall", "-Wextra", "-Werror", "-O2",
                "-I", str(root / "docker/gen3-vive/kortex-overlay"),
                str(root / "tests/cpp/test_gen3_cyclic_failure_guard.cpp"), "-o", str(executable),
            ], check=True, capture_output=True, text=True)
            result = subprocess.run([str(executable)], check=True, capture_output=True, text=True)
            self.assertIn("without retrying passed", result.stdout)
