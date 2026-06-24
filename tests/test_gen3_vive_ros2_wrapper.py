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
    DEFAULT_WATCHDOG_PERIOD_S,
    DIAGNOSTIC_KORTEX_TWIST_BACKEND,
    MOVEIT_SERVO_TWIST_BACKEND,
    Gen3ViveSafetyLoop,
    Gen3ViveRos2WrapperConfig,
    Gen3ViveRos2WrapperConfigError,
    TwistCommand,
    build_mapper_runtime_config,
    build_twist_stamped_payload,
    construct_mapper_with_factory,
    load_mapper_factory,
    main,
)
from example.arm_teleop.gen3_vive_mapper import (
    Gen3ViveCalibration,
    Gen3ViveMapper,
    MapperRuntimeConfig,
    StopReason,
)


def calibration() -> Gen3ViveCalibration:
    return Gen3ViveCalibration.from_mapping(
        {
            "source_frame": "steamvr_world",
            "planning_frame": "base_link",
            "tracker_role": "gen3_tracker",
            "tracker_mount_frame": "gen3_tool_tracker",
            "T_planning_source": {
                "translation": [0.0, 0.0, 0.0],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "T_tool_tracker_mount": {
                "translation": [0.0, 0.0, 0.0],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "linear_scale": 1.0,
            "angular_scale": 1.0,
            "workspace_limits": {
                "x": [-0.05, 0.05],
                "y": [-0.05, 0.05],
                "z": [-0.05, 0.05],
            },
            "max_linear_mps": 0.2,
            "max_angular_radps": 0.5,
            "stale_timeout_s": 0.1,
        }
    )


def pose_msg(
    x: float = 0.0,
    *,
    frame: str = "steamvr_world",
    quat_w: float = 1.0,
) -> dict[str, object]:
    return {
        "header": {"frame_id": frame},
        "pose": {
            "position": {"x": x, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": quat_w},
        },
    }


class FakeMapper:
    def handle_pose(self, *args: object, **kwargs: object) -> TwistCommand:
        return TwistCommand(linear_xyz=(0.0, 0.0, 0.0))

    def poll_safety(self, *args: object, **kwargs: object) -> TwistCommand:
        return TwistCommand(
            linear_xyz=(0.0, 0.0, 0.0),
            command_enabled=False,
            stop_reason="fake_stop",
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
        self.assertEqual(config.watchdog_period_s, DEFAULT_WATCHDOG_PERIOD_S)

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
            return FakeMapper()

        mapper = construct_mapper_with_factory(config, factory)

        self.assertIsNotNone(mapper)
        self.assertEqual(seen["path"], "/tmp/gen3_vive_calibration.yaml")
        self.assertTrue(getattr(seen["runtime"], "real_robot"))

    def test_construct_mapper_requires_watchdog_capable_mapper(self) -> None:
        config = Gen3ViveRos2WrapperConfig(calibration_path="/tmp/calibration.yaml")

        def factory(path: str, runtime: object) -> object:
            return object()

        with self.assertRaisesRegex(Gen3ViveRos2WrapperConfigError, "poll_safety"):
            construct_mapper_with_factory(config, factory)

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


class Gen3ViveSafetyLoopTest(unittest.TestCase):
    def test_watchdog_enforces_stale_stop_without_new_pose(self) -> None:
        mapper = Gen3ViveMapper.from_calibration(
            calibration(),
            MapperRuntimeConfig(dry_run=True),
        )
        loop = Gen3ViveSafetyLoop(
            Gen3ViveRos2WrapperConfig(synthetic_deadman_pressed=True),
            mapper,
        )

        neutral = loop.handle_pose(
            pose_msg(0.0),
            real_robot_latch=False,
            now_monotonic_s=10.0,
        )
        self.assertTrue(neutral.command_enabled)
        self.assertEqual(neutral.stop_reason, StopReason.NEUTRAL_CAPTURED.value)

        active = loop.handle_pose(
            pose_msg(0.02),
            real_robot_latch=False,
            now_monotonic_s=10.01,
        )
        self.assertTrue(active.command_enabled)

        stale = loop.poll_safety(
            real_robot_latch=False,
            now_monotonic_s=10.2,
        )
        self.assertFalse(stale.command_enabled)
        self.assertEqual(stale.stop_reason, StopReason.STALE.value)

        still_held = loop.handle_pose(
            pose_msg(0.02),
            real_robot_latch=False,
            now_monotonic_s=10.21,
        )
        self.assertFalse(still_held.command_enabled)
        self.assertEqual(
            still_held.stop_reason,
            StopReason.WAITING_FOR_NEUTRAL_RECAPTURE.value,
        )

    def test_deadman_release_publishes_immediate_zero_stop_and_requires_recapture(self) -> None:
        mapper = Gen3ViveMapper.from_calibration(
            calibration(),
            MapperRuntimeConfig(dry_run=True),
        )
        loop = Gen3ViveSafetyLoop(
            Gen3ViveRos2WrapperConfig(synthetic_deadman_pressed=True),
            mapper,
        )
        loop.handle_pose(pose_msg(0.0), real_robot_latch=False, now_monotonic_s=20.0)
        active = loop.handle_pose(
            pose_msg(0.02),
            real_robot_latch=False,
            now_monotonic_s=20.01,
        )
        self.assertTrue(active.command_enabled)

        stopped = loop.update_deadman(
            False,
            real_robot_latch=False,
            now_monotonic_s=20.02,
        )
        self.assertIsNotNone(stopped)
        assert stopped is not None
        self.assertFalse(stopped.command_enabled)
        self.assertEqual(stopped.stop_reason, StopReason.DEADMAN_RELEASED.value)

        waiting = loop.handle_pose(
            pose_msg(0.02),
            real_robot_latch=False,
            now_monotonic_s=20.03,
        )
        self.assertFalse(waiting.command_enabled)

        self.assertIsNone(loop.update_deadman(True, real_robot_latch=False))
        no_new_pose = loop.poll_safety(
            real_robot_latch=False,
            now_monotonic_s=20.035,
        )
        self.assertFalse(no_new_pose.command_enabled)
        self.assertEqual(
            no_new_pose.stop_reason,
            StopReason.WAITING_FOR_NEUTRAL_RECAPTURE.value,
        )
        recaptured = loop.handle_pose(
            pose_msg(0.02),
            real_robot_latch=False,
            now_monotonic_s=20.04,
        )
        self.assertTrue(recaptured.command_enabled)
        self.assertEqual(recaptured.stop_reason, StopReason.NEUTRAL_CAPTURED.value)

    def test_invalid_pose_stop_requires_release_and_repress(self) -> None:
        mapper = Gen3ViveMapper.from_calibration(
            calibration(),
            MapperRuntimeConfig(dry_run=True),
        )
        loop = Gen3ViveSafetyLoop(
            Gen3ViveRos2WrapperConfig(synthetic_deadman_pressed=True),
            mapper,
        )
        loop.handle_pose(pose_msg(0.0), real_robot_latch=False, now_monotonic_s=30.0)
        active = loop.handle_pose(
            pose_msg(0.02),
            real_robot_latch=False,
            now_monotonic_s=30.01,
        )
        self.assertTrue(active.command_enabled)

        invalid = loop.handle_pose(
            pose_msg(0.02, quat_w=2.0),
            real_robot_latch=False,
            now_monotonic_s=30.02,
        )
        self.assertFalse(invalid.command_enabled)
        self.assertEqual(invalid.stop_reason, StopReason.INVALID_POSE.value)

        still_held = loop.handle_pose(
            pose_msg(0.03),
            real_robot_latch=False,
            now_monotonic_s=30.03,
        )
        self.assertFalse(still_held.command_enabled)
        self.assertEqual(
            still_held.stop_reason,
            StopReason.WAITING_FOR_NEUTRAL_RECAPTURE.value,
        )

        loop.update_deadman(False, real_robot_latch=False, now_monotonic_s=30.04)
        self.assertIsNone(loop.update_deadman(True, real_robot_latch=False))
        no_new_pose = loop.poll_safety(
            real_robot_latch=False,
            now_monotonic_s=30.045,
        )
        self.assertFalse(no_new_pose.command_enabled)
        self.assertEqual(
            no_new_pose.stop_reason,
            StopReason.WAITING_FOR_NEUTRAL_RECAPTURE.value,
        )
        recaptured = loop.handle_pose(
            pose_msg(0.03),
            real_robot_latch=False,
            now_monotonic_s=30.05,
        )
        self.assertTrue(recaptured.command_enabled)
        self.assertEqual(recaptured.stop_reason, StopReason.NEUTRAL_CAPTURED.value)


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
