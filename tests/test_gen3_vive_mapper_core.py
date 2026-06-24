"""Safety regression tests for the dependency-light Gen3 VIVE mapper core."""

from __future__ import annotations

import unittest
from inspect import signature
from types import SimpleNamespace

from example.arm_teleop.gen3_vive_mapper import (
    DEFAULT_BACKEND,
    DIAGNOSTIC_KORTEX_BACKEND,
    DeadmanInput,
    Gen3ViveCalibration,
    Gen3ViveMapper,
    Gen3ViveMapperCore,
    MapperError,
    MapperRuntimeConfig,
    StopReason,
    ToolPose,
    ViveTrackerPose,
    build_mapper_from_yaml,
)
from example.arm_teleop.gen3_vive_ros2_wrapper import (
    DEFAULT_MAPPER_FACTORY,
    Gen3ViveRos2WrapperConfig,
    Gen3ViveRos2WrapperConfigError,
    build_twist_stamped_payload,
    load_mapper_factory,
    mapper_command_to_twist_command,
)


def calibration_dict(**overrides: object) -> dict[str, object]:
    data: dict[str, object] = {
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
    data.update(overrides)
    return data


def calibration(**overrides: object) -> Gen3ViveCalibration:
    return Gen3ViveCalibration.from_mapping(calibration_dict(**overrides))


def pose(
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    *,
    t: float = 1.0,
    role: str = "gen3_tracker",
    frame: str = "steamvr_world",
    quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> ViveTrackerPose:
    return ViveTrackerPose(
        role=role,
        frame=frame,
        position_xyz=(x, y, z),
        quaternion_xyzw=quat,
        received_monotonic_s=t,
        serial="LHR-TEST",
    )


def deadman(
    pressed: bool = True,
    *,
    physical: bool = True,
) -> DeadmanInput:
    return DeadmanInput(pressed=pressed, is_physical=physical)


class Gen3ViveMapperCalibrationTest(unittest.TestCase):
    def test_valid_calibration_contract(self) -> None:
        cal = calibration()

        self.assertEqual(cal.source_frame, "steamvr_world")
        self.assertEqual(cal.tracker_role, "gen3_tracker")
        self.assertEqual(cal.planning_frame, "base_link")

    def test_calibration_rejects_missing_and_invalid_fields(self) -> None:
        bad = calibration_dict()
        bad.pop("T_planning_source")
        with self.assertRaisesRegex(MapperError, "missing"):
            Gen3ViveCalibration.from_mapping(bad)

        with self.assertRaisesRegex(MapperError, "quaternion"):
            Gen3ViveCalibration.from_mapping(
                calibration_dict(
                    T_planning_source={
                        "translation": [0.0, 0.0, 0.0],
                        "rotation_xyzw": [0.0, 0.0, 0.0, 2.0],
                    }
                )
            )

        with self.assertRaisesRegex(MapperError, "min exceeds max"):
            Gen3ViveCalibration.from_mapping(
                calibration_dict(workspace_limits={"x": [1.0, -1.0], "y": [-1, 1], "z": [-1, 1]})
            )


class Gen3ViveMapperStateMachineTest(unittest.TestCase):
    def setUp(self) -> None:
        self.core = Gen3ViveMapperCore(
            calibration(),
            MapperRuntimeConfig(dry_run=True, low_pass_alpha=1.0),
        )

    def test_deadman_rising_edge_captures_neutral_then_outputs_delta(self) -> None:
        first = self.core.update(
            pose=pose(0.0, 0.0, 0.0, t=10.0),
            deadman=deadman(True),
            now_monotonic_s=10.0,
            current_tool_pose=ToolPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        )
        self.assertTrue(first.enabled)
        self.assertEqual(first.stop_reason, StopReason.NEUTRAL_CAPTURED.value)
        self.assertFalse(self.core.requires_neutral_recapture)

        command = self.core.update(
            pose=pose(0.02, -0.01, 0.03, t=10.01),
            deadman=deadman(True),
            now_monotonic_s=10.01,
        )

        self.assertTrue(command.enabled)
        self.assertIsNone(command.stop_reason)
        self.assertEqual(command.linear_xyz, (0.02, -0.01, 0.03))
        self.assertTrue(command.dry_run)
        self.assertEqual(command.backend, DEFAULT_BACKEND)
        self.assertEqual(command.command_type, "geometry_msgs/msg/TwistStamped")

    def test_workspace_clamp_and_low_pass_filter(self) -> None:
        core = Gen3ViveMapperCore(
            calibration(),
            MapperRuntimeConfig(dry_run=True, low_pass_alpha=0.5),
        )
        core.update(
            pose=pose(0.0, 0.0, 0.0, t=20.0),
            deadman=deadman(True),
            now_monotonic_s=20.0,
            current_tool_pose=ToolPose((0.04, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        )

        command = core.update(
            pose=pose(0.2, 0.0, 0.0, t=20.01),
            deadman=deadman(True),
            now_monotonic_s=20.01,
        )

        # Tool x=0.04 plus delta is clamped to workspace max x=0.05, so
        # command delta becomes 0.01; alpha=0.5 filters first output to 0.005.
        self.assertAlmostEqual(command.linear_xyz[0], 0.005)
        self.assertEqual(command.linear_xyz[1:], (0.0, 0.0))

    def test_stale_stop_requires_release_and_new_deadman_rising_edge(self) -> None:
        self.core.update(
            pose=pose(0.0, 0.0, 0.0, t=30.0),
            deadman=deadman(True),
            now_monotonic_s=30.0,
        )

        stale = self.core.update(
            pose=None,
            deadman=deadman(True),
            now_monotonic_s=30.2,
        )
        self.assertFalse(stale.enabled)
        self.assertEqual(stale.stop_reason, StopReason.STALE.value)
        self.assertTrue(stale.needs_neutral_recapture)

        still_pressed = self.core.update(
            pose=pose(0.0, 0.0, 0.0, t=30.21),
            deadman=deadman(True),
            now_monotonic_s=30.21,
        )
        self.assertFalse(still_pressed.enabled)
        self.assertEqual(still_pressed.stop_reason, StopReason.WAITING_FOR_NEUTRAL_RECAPTURE.value)

        self.core.update(
            pose=pose(0.0, 0.0, 0.0, t=30.22),
            deadman=deadman(False),
            now_monotonic_s=30.22,
        )
        recaptured = self.core.update(
            pose=pose(0.0, 0.0, 0.0, t=30.23),
            deadman=deadman(True),
            now_monotonic_s=30.23,
        )
        self.assertTrue(recaptured.enabled)
        self.assertEqual(recaptured.stop_reason, StopReason.NEUTRAL_CAPTURED.value)

    def test_frame_role_and_deadman_fail_closed(self) -> None:
        wrong_frame = self.core.update(
            pose=pose(frame="base_link"),
            deadman=deadman(True),
            now_monotonic_s=1.0,
        )
        self.assertFalse(wrong_frame.enabled)
        self.assertEqual(wrong_frame.stop_reason, StopReason.FRAME_MISMATCH.value)

        wrong_role = self.core.update(
            pose=pose(role="left_hand", t=1.01),
            deadman=deadman(True),
            now_monotonic_s=1.01,
        )
        self.assertFalse(wrong_role.enabled)
        self.assertEqual(wrong_role.stop_reason, StopReason.ROLE_MISMATCH.value)

        released = self.core.update(
            pose=pose(t=1.02),
            deadman=deadman(False),
            now_monotonic_s=1.02,
        )
        self.assertFalse(released.enabled)
        self.assertEqual(released.stop_reason, StopReason.DEADMAN_RELEASED.value)

    def test_real_robot_rejects_synthetic_deadman(self) -> None:
        core = Gen3ViveMapperCore(
            calibration(),
            MapperRuntimeConfig(dry_run=False, real_robot=True),
        )
        command = core.update(
            pose=pose(t=40.0),
            deadman=deadman(True, physical=False),
            now_monotonic_s=40.0,
            real_robot_latch=True,
        )

        self.assertFalse(command.enabled)
        self.assertEqual(command.stop_reason, StopReason.SYNTHETIC_DEADMAN_REJECTED.value)

    def test_real_robot_requires_dynamic_latch_even_with_physical_deadman(self) -> None:
        mapper = Gen3ViveMapper.from_calibration(
            calibration(),
            MapperRuntimeConfig(dry_run=False, real_robot=True),
        )
        msg = {
            "header": {"frame_id": "steamvr_world"},
            "pose": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }

        command = mapper.handle_pose(
            msg,
            deadman_pressed=True,
            deadman_is_physical=True,
            real_robot_latch=False,
            now_monotonic_s=40.0,
        )

        self.assertFalse(command.enabled)
        self.assertEqual(command.stop_reason, StopReason.REAL_ROBOT_LATCH.value)

    def test_diagnostic_kortex_backend_requires_explicit_opt_in(self) -> None:
        with self.assertRaisesRegex(MapperError, "diagnostic-only"):
            MapperRuntimeConfig(command_backend=DIAGNOSTIC_KORTEX_BACKEND)

        runtime = MapperRuntimeConfig(
            command_backend=DIAGNOSTIC_KORTEX_BACKEND,
            allow_diagnostic_kortex=True,
        )
        self.assertEqual(runtime.command_backend, DIAGNOSTIC_KORTEX_BACKEND)


class Gen3ViveMapperWrapperIntegrationTest(unittest.TestCase):
    def test_default_mapper_factory_points_to_yaml_factory(self) -> None:
        factory = load_mapper_factory(DEFAULT_MAPPER_FACTORY)
        self.assertTrue(callable(factory))
        self.assertIn("runtime", signature(build_mapper_from_yaml).parameters)

    def test_real_robot_wrapper_requires_calibration_path(self) -> None:
        with self.assertRaisesRegex(Gen3ViveRos2WrapperConfigError, "calibration_path"):
            Gen3ViveRos2WrapperConfig(
                dry_run=False,
                real_robot=True,
                real_robot_latch=True,
                deadman_source="physical_topic",
            ).validate()

    def test_mapper_command_converts_to_twist_stamped_payload(self) -> None:
        core = Gen3ViveMapperCore(calibration(), MapperRuntimeConfig(dry_run=True))
        core.update(
            pose=pose(0.0, 0.0, 0.0, t=50.0),
            deadman=deadman(True),
            now_monotonic_s=50.0,
        )
        command = core.update(
            pose=pose(0.01, 0.0, 0.0, t=50.01),
            deadman=deadman(True),
            now_monotonic_s=50.01,
        )

        twist = mapper_command_to_twist_command(command)
        payload = build_twist_stamped_payload(twist, stamp_sec=1)

        self.assertTrue(payload["command_enabled"])
        self.assertEqual(payload["header"]["frame_id"], "base_link")
        self.assertEqual(payload["twist"]["linear"]["x"], 0.01)

    def test_pose_stamped_like_adapter_smoke(self) -> None:
        mapper = Gen3ViveMapper.from_calibration(calibration())
        msg = {
            "header": {"frame_id": "steamvr_world"},
            "pose": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }

        command = mapper.handle_pose(
            msg,
            deadman_pressed=True,
            real_robot_latch=False,
            now_monotonic_s=60.0,
        )

        self.assertTrue(command.enabled)
        self.assertEqual(command.stop_reason, StopReason.NEUTRAL_CAPTURED.value)

    def test_pose_stamped_like_adapter_invalid_pose_forces_recapture(self) -> None:
        mapper = Gen3ViveMapper.from_calibration(calibration())
        valid = {
            "header": {"frame_id": "steamvr_world"},
            "pose": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }
        active = {
            "header": {"frame_id": "steamvr_world"},
            "pose": {
                "position": {"x": 0.02, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }
        msg = SimpleNamespace(
            header=SimpleNamespace(frame_id="steamvr_world"),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=2.0),
            ),
        )

        mapper.handle_pose(
            valid,
            deadman_pressed=True,
            real_robot_latch=False,
            now_monotonic_s=61.0,
        )
        enabled = mapper.handle_pose(
            active,
            deadman_pressed=True,
            real_robot_latch=False,
            now_monotonic_s=61.01,
        )
        self.assertTrue(enabled.enabled)

        stopped = mapper.handle_pose(
            msg,
            deadman_pressed=True,
            real_robot_latch=False,
            now_monotonic_s=61.02,
        )
        self.assertFalse(stopped.enabled)
        self.assertEqual(stopped.stop_reason, StopReason.INVALID_POSE.value)
        self.assertTrue(mapper.core.requires_neutral_recapture)

        still_held = mapper.handle_pose(
            active,
            deadman_pressed=True,
            real_robot_latch=False,
            now_monotonic_s=61.03,
        )
        self.assertFalse(still_held.enabled)
        self.assertEqual(
            still_held.stop_reason,
            StopReason.WAITING_FOR_NEUTRAL_RECAPTURE.value,
        )


if __name__ == "__main__":
    unittest.main()
