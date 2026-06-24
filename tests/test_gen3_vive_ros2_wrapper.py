"""Tests for Gen3 VIVE ROS2 wrapper safety boundaries."""

from __future__ import annotations

import contextlib
import io
import json
import subprocess
import sys
import unittest
from pathlib import Path

from example.arm_teleop.gen3_vive_ros2_wrapper import (
    DEFAULT_PHYSICAL_DEADMAN_TOPIC,
    DEFAULT_MAPPER_FACTORY,
    DEFAULT_TWIST_TOPIC,
    DIAGNOSTIC_KORTEX_TWIST_BACKEND,
    MOVEIT_SERVO_TWIST_BACKEND,
    Gen3ViveRos2WrapperConfig,
    Gen3ViveRos2WrapperConfigError,
    TwistCommand,
    build_mapper_runtime_config,
    build_twist_stamped_payload,
    construct_mapper_with_factory,
    load_mapper_factory,
    main,
)


class Gen3ViveRos2WrapperConfigTest(unittest.TestCase):
    def test_default_config_is_dry_run_moveit_servo_twist(self) -> None:
        config = Gen3ViveRos2WrapperConfig()

        config.validate()

        self.assertTrue(config.dry_run)
        self.assertFalse(config.real_robot)
        self.assertFalse(config.real_robot_latch)
        self.assertEqual(config.backend, MOVEIT_SERVO_TWIST_BACKEND)
        self.assertEqual(config.twist_topic, DEFAULT_TWIST_TOPIC)

    def test_real_robot_requires_physical_deadman_and_latch(self) -> None:
        with self.assertRaisesRegex(Gen3ViveRos2WrapperConfigError, "physical_topic"):
            Gen3ViveRos2WrapperConfig(
                dry_run=False,
                real_robot=True,
                real_robot_latch=True,
                deadman_source="synthetic_cli",
            ).validate()

        with self.assertRaisesRegex(Gen3ViveRos2WrapperConfigError, "real_robot_latch"):
            Gen3ViveRos2WrapperConfig(
                dry_run=False,
                real_robot=True,
                deadman_source="physical_topic",
                physical_deadman_topic=DEFAULT_PHYSICAL_DEADMAN_TOPIC,
            ).validate()

        config = Gen3ViveRos2WrapperConfig(
            dry_run=False,
            real_robot=True,
            real_robot_latch=True,
            deadman_source="physical_topic",
            physical_deadman_topic=DEFAULT_PHYSICAL_DEADMAN_TOPIC,
            calibration_path="/tmp/gen3_vive_calibration.yaml",
        )
        config.validate()

    def test_synthetic_deadman_is_dry_run_only(self) -> None:
        with self.assertRaisesRegex(Gen3ViveRos2WrapperConfigError, "dry-run"):
            Gen3ViveRos2WrapperConfig(
                dry_run=False,
                deadman_source="synthetic_cli",
            ).validate()

    def test_diagnostic_kortex_backend_is_not_a_ros2_wrapper_branch(self) -> None:
        with self.assertRaisesRegex(Gen3ViveRos2WrapperConfigError, "not implemented"):
            Gen3ViveRos2WrapperConfig(
                backend=DIAGNOSTIC_KORTEX_TWIST_BACKEND,
            ).validate()

        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            with self.assertRaises(SystemExit) as raised:
                main(["--backend", DIAGNOSTIC_KORTEX_TWIST_BACKEND, "--print-config"])
        self.assertNotEqual(raised.exception.code, 0)

    def test_mapper_runtime_config_propagates_real_robot_gates(self) -> None:
        config = Gen3ViveRos2WrapperConfig(
            dry_run=False,
            real_robot=True,
            real_robot_latch=True,
            deadman_source="physical_topic",
            physical_deadman_topic=DEFAULT_PHYSICAL_DEADMAN_TOPIC,
            calibration_path="/tmp/gen3_vive_calibration.yaml",
        )

        runtime = build_mapper_runtime_config(config)

        self.assertFalse(runtime.dry_run)
        self.assertTrue(runtime.real_robot)
        self.assertEqual(runtime.command_backend, MOVEIT_SERVO_TWIST_BACKEND)

    def test_construct_mapper_passes_wrapper_runtime_to_factory(self) -> None:
        seen: dict[str, object] = {}
        config = Gen3ViveRos2WrapperConfig(
            dry_run=False,
            real_robot=True,
            real_robot_latch=True,
            deadman_source="physical_topic",
            physical_deadman_topic=DEFAULT_PHYSICAL_DEADMAN_TOPIC,
            calibration_path="/tmp/gen3_vive_calibration.yaml",
        )

        def factory(path: str, runtime: object) -> object:
            seen["path"] = path
            seen["runtime"] = runtime
            return object()

        mapper = construct_mapper_with_factory(config, factory)

        self.assertIsNotNone(mapper)
        self.assertEqual(seen["path"], "/tmp/gen3_vive_calibration.yaml")
        self.assertTrue(getattr(seen["runtime"], "real_robot"))

    def test_default_mapper_factory_imports_from_documented_filename_launch_model(self) -> None:
        repo_root = Path(__file__).resolve().parents[1]
        script_dir = repo_root / "example" / "arm_teleop"
        code = """
import importlib.util
import pathlib
import sys
script_dir = pathlib.Path(sys.argv[1])
repo_root = pathlib.Path(sys.argv[2])
sys.path = [
    str(script_dir),
    *[
        path for path in sys.path
        if path not in ("", str(repo_root), str(repo_root.resolve()))
    ],
]
spec = importlib.util.spec_from_file_location(
    "gen3_vive_ros2_wrapper_direct", script_dir / "gen3_vive_ros2_wrapper.py"
)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = module
spec.loader.exec_module(module)
factory = module.load_mapper_factory(module.DEFAULT_MAPPER_FACTORY)
print(callable(factory))
"""
        completed = subprocess.run(
            [sys.executable, "-c", code, str(script_dir), str(repo_root)],
            check=True,
            text=True,
            capture_output=True,
        )

        self.assertEqual(completed.stdout.strip(), "True")


class Gen3ViveTwistStampedPayloadTest(unittest.TestCase):
    def test_twist_command_builds_moveit_servo_twist_stamped_shape(self) -> None:
        payload = build_twist_stamped_payload(
            TwistCommand(
                linear_xyz=(0.01, -0.02, 0.03),
                angular_xyz=(0.1, 0.0, -0.1),
                planning_frame="base_link",
            ),
            stamp_sec=123,
            stamp_nanosec=456,
        )

        self.assertEqual(payload["header"]["frame_id"], "base_link")
        self.assertEqual(payload["header"]["stamp"], {"sec": 123, "nanosec": 456})
        self.assertEqual(payload["twist"]["linear"], {"x": 0.01, "y": -0.02, "z": 0.03})
        self.assertEqual(payload["twist"]["angular"], {"x": 0.1, "y": 0.0, "z": -0.1})
        self.assertTrue(payload["command_enabled"])

    def test_disabled_command_publishes_zero_stop_payload(self) -> None:
        payload = build_twist_stamped_payload(
            {
                "linear_xyz": (1.0, 2.0, 3.0),
                "angular_xyz": (4.0, 5.0, 6.0),
                "planning_frame": "base_link",
                "command_enabled": False,
                "stop_reason": "deadman_released",
            },
            stamp_sec=0,
        )

        self.assertEqual(payload["twist"]["linear"], {"x": 0.0, "y": 0.0, "z": 0.0})
        self.assertEqual(payload["twist"]["angular"], {"x": 0.0, "y": 0.0, "z": 0.0})
        self.assertFalse(payload["command_enabled"])
        self.assertEqual(payload["stop_reason"], "deadman_released")

    def test_non_finite_command_is_rejected_before_ros_publish(self) -> None:
        with self.assertRaisesRegex(Gen3ViveRos2WrapperConfigError, "finite"):
            build_twist_stamped_payload({"linear_xyz": (0.0, float("nan"), 0.0)})


class Gen3ViveWrapperCliTest(unittest.TestCase):
    def run_main(self, argv: list[str]) -> tuple[int, str]:
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            code = main(argv)
        return code, stdout.getvalue().strip()

    def test_print_config_uses_safe_defaults_without_ros_imports(self) -> None:
        code, output = self.run_main(["--print-config"])

        self.assertEqual(code, 0)
        config = json.loads(output)
        self.assertTrue(config["dry_run"])
        self.assertFalse(config["real_robot"])
        self.assertFalse(config["real_robot_latch"])
        self.assertEqual(config["backend"], MOVEIT_SERVO_TWIST_BACKEND)
        self.assertNotIn("pose_topic", config)
        self.assertEqual(config["twist_topic"], DEFAULT_TWIST_TOPIC)

    def test_cli_does_not_expose_pose_topic_override(self) -> None:
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            with self.assertRaises(SystemExit) as raised:
                main(["--pose-topic", "/vive/left_hand/pose", "--print-config"])
        self.assertNotEqual(raised.exception.code, 0)

    def test_real_robot_cli_fails_closed_without_latch(self) -> None:
        code, output = self.run_main(
            [
                "--no-dry-run",
                "--real-robot",
                "--deadman-source",
                "physical_topic",
                "--print-config",
            ]
        )

        self.assertEqual(code, 2)
        self.assertIn("real_robot_latch", output)

    def test_cli_can_emit_twist_stamped_payload_for_mapper_command_json(self) -> None:
        code, output = self.run_main(
            [
                "--twist-command-json",
                '{"linear_xyz":[0.01,0.0,0.0],"angular_xyz":[0.0,0.0,0.0],"planning_frame":"base_link"}',
            ]
        )

        self.assertEqual(code, 0)
        payload = json.loads(output)
        self.assertEqual(payload["header"]["frame_id"], "base_link")
        self.assertEqual(payload["twist"]["linear"]["x"], 0.01)
        self.assertTrue(payload["command_enabled"])


if __name__ == "__main__":
    unittest.main()
