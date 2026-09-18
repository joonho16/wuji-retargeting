"""Actual tmux argument regression on a private socket; no Docker or hardware."""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("gen3_shutdown_runtime", ROOT / "scripts/gen3_teleop_shutdown.py")
shutdown = importlib.util.module_from_spec(spec)
spec.loader.exec_module(shutdown)


@unittest.skipUnless(shutil.which("tmux"), "tmux is not installed")
class TmuxRuntimeTests(unittest.TestCase):
    def test_launcher_and_shutdown_use_valid_exact_session_targets(self):
        real_tmux = shutil.which("tmux")
        with tempfile.TemporaryDirectory(prefix="gen3-tmux-test-") as directory:
            task = Path(directory)
            repo, bin_dir = task / "checkout", task / "bin"
            (repo / "scripts").mkdir(parents=True)
            bin_dir.mkdir()
            shutil.copy2(ROOT / "scripts/gen3_manus_teleop.sh", repo / "scripts/gen3_manus_teleop.sh")
            calibration = task / "calibration"
            calibration.mkdir()
            (calibration / "test.yaml").write_text("real_robot_calibration_reviewed: true\n")
            socket = task / "private.sock"
            # Never allow any real container command. Replace every launched
            # window process with a bounded sleep; leave tmux's own CLI intact.
            shim = bin_dir / "tmux"
            shim.write_text("#!/usr/bin/python3\nimport os, subprocess, sys\n"
                            f"prefix = {[real_tmux, '-S', str(socket), '-f', '/dev/null']!r}\n"
                            "args = sys.argv[1:]\n"
                            "if args[0] in ('attach', 'attach-session'): sys.exit(0)\n"
                            "if args[0] in ('new-session', 'new-window'): args[-1] = 'sleep 60'\n"
                            "sys.exit(subprocess.call(prefix + args))\n")
            shim.chmod(0o755)
            docker = bin_dir / "docker"
            docker.write_text("#!/bin/sh\necho 'Unexpected Docker invocation in tmux test' >&2\nexit 99\n")
            docker.chmod(0o755)
            operator_dir = task / "operator"
            env = dict(os.environ, PATH=f"{bin_dir}:/usr/bin:/bin", TMUX="", TMUX_PANE="",
                       GEN3_ROBOT_IP="127.0.0.1", MANUS_CORE_IP="127.0.0.1", DISPLAY=":99",
                       CONFIRM_REAL_WH110="I_UNDERSTAND_WH110_MOVES",
                       GEN3_OPERATOR_DIR=str(operator_dir),
                       GEN3_VIVE_CALIBRATION_DIR=str(calibration), GEN3_VIVE_CALIBRATION="/calib/test.yaml")

            def tmux(args, **kwargs):
                return subprocess.run([real_tmux, "-S", str(socket), "-f", "/dev/null", *args],
                                      env=env, text=True, capture_output=True, **kwargs)

            def private_run(args, **kwargs):
                self.assertEqual(args[0], "tmux", "This test must never invoke Docker")
                return tmux(args[1:], **kwargs)

            try:
                result = subprocess.run(["bash", str(repo / "scripts/gen3_manus_teleop.sh"),
                                         "full-real", "--session", "gen3-manus-teleop"],
                                        env=env, text=True, capture_output=True, timeout=8)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                session_rows = tmux(["list-sessions", "-F", "#{session_id}\t#{session_name}"],
                                    check=True).stdout.splitlines()
                session = next(row.partition("\t")[0] for row in session_rows
                               if row.partition("\t")[2] == "gen3-manus-teleop")
                self.assertEqual(set(tmux(["list-windows", "-t", session, "-F", "#{window_name}"],
                                          check=True).stdout.splitlines()), {"gen3", "deadman", "manus", "hand"})
                tmux(["new-session", "-d", "-s", "unrelated", "sleep 60"], check=True)
                with patch.object(shutdown, "run", side_effect=private_run):
                    self.assertEqual(shutdown.resolve_session("gen3-manus-teleop"), session)
                    self.assertEqual(shutdown.resolve_session(session), session)
                    self.assertEqual(shutdown.resolve_session("gen3-manus"), "")
                    self.assertTrue(shutdown.marked_session(session, repo, operator_dir))
                    self.assertFalse(shutdown.marked_session("unrelated", repo, operator_dir))
                    shutdown.cancel_stack_windows(session)
                self.assertEqual(tmux(["list-windows", "-t", session, "-F", "#{window_name}"],
                                      check=True).stdout.strip(), "deadman")
                self.assertEqual(tmux(["has-session", "-t", "=unrelated"]).returncode, 0)
            finally:
                # The private socket is mandatory even during cleanup.
                tmux(["kill-server"])


if __name__ == "__main__":
    unittest.main()
