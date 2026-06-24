"""Tests for Gen3 VIVE ROS2 wrapper safety boundaries."""

from __future__ import annotations

import contextlib
import io
import json
import unittest

from example.arm_teleop.gen3_vive_ros2_wrapper import (
    DEFAULT_PHYSICAL_DEADMAN_TOPIC,
    DEFAULT_POSE_TOPIC,
    DEFAULT_TWIST_TOPIC,
    DIAGNOSTIC_KORTEX_TWIST_BACKEND,
    MOVEIT_SERVO_TWIST_BACKEND,
    Gen3ViveRos2WrapperConfig,
    Gen3ViveRos2WrapperConfigError,
    TwistCommand,
    build_twist_stamped_payload,
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
        self.assertEqual(config.pose_topic, DEFAULT_POSE_TOPIC)
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
        )
        config.validate()

    def test_synthetic_deadman_is_dry_run_only(self) -> None:
        with self.assertRaisesRegex(Gen3ViveRos2WrapperConfigError, "dry-run"):
            Gen3ViveRos2WrapperConfig(
                dry_run=False,
                deadman_source="synthetic_cli",
            ).validate()

    def test_diagnostic_kortex_backend_requires_explicit_enable(self) -> None:
        with self.assertRaisesRegex(Gen3ViveRos2WrapperConfigError, "disabled by default"):
            Gen3ViveRos2WrapperConfig(
                backend=DIAGNOSTIC_KORTEX_TWIST_BACKEND,
            ).validate()

        config = Gen3ViveRos2WrapperConfig(
            backend=DIAGNOSTIC_KORTEX_TWIST_BACKEND,
            diagnostic_kortex_enabled=True,
        )
        config.validate()


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
        self.assertEqual(config["twist_topic"], DEFAULT_TWIST_TOPIC)

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
