"""Velocity/acceleration ceilings and read-only Kortex discovery boundaries."""

from __future__ import annotations

import copy
import json
import math
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from example.arm_teleop import gen3_joint_limits as limits


def model_fixture(dof=7):
    urdf = '<robot name="gen3">' + "".join(
        f'<joint name="joint_{i}" type="continuous"><limit velocity="1.0"/></joint>'
        for i in range(1, dof + 1)
    ) + '</robot>'
    planning = {
        "default_velocity_scaling_factor": 0.1,
        "joint_limits": {
            name: dict(has_velocity_limits=True, max_velocity=0.8,
                       has_acceleration_limits=True, max_acceleration=8.6)
            for name in limits.arm_joint_names(dof)
        },
    }
    planning["joint_limits"]["gripper"] = {"max_velocity": 0.5}
    return urdf, planning


class JointLimitConfigTests(unittest.TestCase):
    def load(self, text, dof=7):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "limits.yaml"
            path.write_text(text)
            return limits.load_settings(path, dof)

    def test_default_config_supports_both_robots_without_extra_caps(self):
        path = Path(__file__).resolve().parents[1] / "example/config/gen3_joint_limits.yaml"
        for dof in (6, 7):
            settings = limits.load_settings(path, dof)
            self.assertEqual(settings.robot_soft_limit_mode, "ANGULAR_TRAJECTORY")
            self.assertEqual(settings.velocities, {})
            self.assertEqual(settings.accelerations, {})

    def test_partial_overrides_and_explicit_mode(self):
        settings = self.load("robot_soft_limit_mode: CARTESIAN_JOYSTICK\njoint_limits:\n  joint_2: {max_velocity: 0.2}")
        self.assertEqual(settings.robot_soft_limit_mode, "CARTESIAN_JOYSTICK")
        self.assertEqual(settings.velocities, {"joint_2": 0.2})

    def test_acceleration_can_be_configured_independently(self):
        settings = self.load("joint_limits: {joint_1: {max_acceleration: 0.8}, joint_2: {max_velocity: 0.2, max_acceleration: 1.2}}")
        self.assertEqual(settings.velocities, {"joint_2": 0.2})
        self.assertEqual(settings.accelerations, {"joint_1": 0.8, "joint_2": 1.2})

    def test_invalid_or_ambiguous_config_is_rejected(self):
        for text in (
            "", "[]", "joint_limits: null", "joint_limit: {}",
            "robot_soft_limit_mode: IDLE\njoint_limits: {}",
            "joint_limits: {joint_8: {max_velocity: 0.2}}",
            "joint_limits: {joint_1: {max_velocty: 0.2}}",
            "joint_limits: {joint_1: {max_velocity: 0.2, has_velocity_limits: false}}",
            "joint_limits: {}\njoint_limits: {}",
            "joint_limits: {joint_1: {max_velocity: 0.2, max_velocity: 1.0}}",
            "joint_limits: {joint_1: {max_velocity: 0.2}, joint_1: {max_velocity: 1.0}}",
            "joint_limits: {joint_1: {}}",
            "joint_limits: {joint_1: {has_acceleration_limits: true}}",
            "joint_limits: {joint_1: {max_acceleration: 1.0, has_acceleration_limits: false}}",
        ):
            with self.subTest(text=text), self.assertRaises(limits.JointLimitsError):
                self.load(text)
        with self.assertRaises(limits.JointLimitsError):
            self.load("joint_limits: {joint_7: {max_velocity: 0.2}}", dof=6)

    def test_invalid_velocity_values_are_rejected(self):
        for value in ("0", "-1", ".nan", ".inf", "true", "null", "'0.3'"):
            with self.subTest(value=value), self.assertRaises(limits.JointLimitsError):
                self.load(f"joint_limits: {{joint_1: {{max_velocity: {value}}}}}")
            with self.subTest(acceleration=value), self.assertRaises(limits.JointLimitsError):
                self.load(f"joint_limits: {{joint_1: {{max_acceleration: {value}}}}}")

    def test_missing_file_is_rejected(self):
        with self.assertRaises(limits.JointLimitsError):
            limits.load_settings(Path("/nonexistent/gen3-limit-config.yaml"), 7)


class JointLimitResolutionTests(unittest.TestCase):
    def test_minimum_of_all_sources_and_preservation_of_other_parameters(self):
        urdf, planning = model_fixture()
        before = copy.deepcopy(planning)
        robot = {
            "hard": {name: 0.7 for name in limits.arm_joint_names(7)},
            "soft": {name: 0.6 for name in limits.arm_joint_names(7)},
        }
        robot["hard"]["joint_3"] = 0.4
        settings = limits.JointLimitSettings("ANGULAR_TRAJECTORY", {"joint_1": 0.2, "joint_2": 5.0})
        resolved, messages = limits.resolve_velocity_limits(
            planning, limits.model_velocity_limits(urdf, planning, 7), settings, robot
        )
        self.assertEqual(resolved["joint_limits"]["joint_1"]["max_velocity"], 0.2)
        self.assertEqual(resolved["joint_limits"]["joint_2"]["max_velocity"], 0.6)
        self.assertEqual(resolved["joint_limits"]["joint_3"]["max_velocity"], 0.4)
        self.assertEqual(resolved["joint_limits"]["joint_7"]["max_velocity"], 0.6)
        self.assertEqual(resolved["joint_limits"]["joint_1"]["max_acceleration"], 8.6)
        self.assertEqual(resolved["joint_limits"]["gripper"], planning["joint_limits"]["gripper"])
        self.assertEqual(planning, before)
        self.assertEqual(len(messages), 7)
        self.assertIn("robot_soft=0.6", messages[0])
        self.assertIn("file=0.2", messages[0])

    def test_fake_hardware_uses_model_and_optional_file(self):
        urdf, planning = model_fixture(6)
        resolved, _ = limits.resolve_velocity_limits(
            planning, limits.model_velocity_limits(urdf, planning, 6),
            limits.JointLimitSettings("ANGULAR_TRAJECTORY", {"joint_1": 0.1}), None,
        )
        self.assertEqual(resolved["joint_limits"]["joint_1"]["max_velocity"], 0.1)
        self.assertEqual(resolved["joint_limits"]["joint_6"]["max_velocity"], 0.8)
        self.assertNotIn("joint_7", resolved["joint_limits"])

    def test_urdf_ceiling_cannot_be_raised_by_moveit_or_file(self):
        urdf, planning = model_fixture()
        planning["joint_limits"]["joint_1"]["max_velocity"] = 10.0
        model = limits.model_velocity_limits(urdf, planning, 7)
        self.assertEqual(model["joint_1"], 1.0)
        result, _ = limits.resolve_velocity_limits(
            planning, model, limits.JointLimitSettings("ANGULAR_TRAJECTORY", {"joint_1": 5.0}), None
        )
        self.assertEqual(result["joint_limits"]["joint_1"]["max_velocity"], 1.0)

    def test_model_missing_bound_or_joint_is_rejected(self):
        urdf, planning = model_fixture()
        for invalid in (urdf.replace('velocity="1.0"', 'velocity="nan"', 1),
                        urdf.replace('velocity="1.0"', '', 1),
                        urdf.replace('name="joint_1"', 'name="wrong"', 1), "<robot"):
            with self.subTest(invalid=invalid), self.assertRaises(limits.JointLimitsError):
                limits.model_velocity_limits(invalid, planning, 7)


class RobotLimitQueryTests(unittest.TestCase):
    def query(self, **overrides):
        args = dict(robot_ip="192.0.2.1", username="test", password="private-test-value",
                    mode="ANGULAR_TRAJECTORY", dof=7)
        args.update(overrides)
        return limits.query_robot_limits(**args)

    def response(self, **overrides):
        data = dict(mode="ANGULAR_TRAJECTORY", hard=[90.0] * 7, soft=[30.0] * 7,
                    hard_acceleration=[180.0] * 7, soft_acceleration=[60.0] * 7)
        data.update(overrides)
        return subprocess.CompletedProcess([], 0, stdout=json.dumps(data), stderr="")

    @patch.object(limits.subprocess, "run")
    def test_degrees_converted_and_credentials_not_passed_in_argv(self, run):
        run.return_value = self.response()
        result = self.query()
        self.assertAlmostEqual(result["hard"]["joint_1"], math.pi / 2)
        self.assertAlmostEqual(result["soft"]["joint_7"], math.pi / 6)
        self.assertAlmostEqual(result["hard_acceleration"]["joint_1"], math.pi)
        self.assertAlmostEqual(result["soft_acceleration"]["joint_7"], math.pi / 3)
        argv = run.call_args.args[0]
        self.assertNotIn("private-test-value", " ".join(argv))
        self.assertEqual(json.loads(run.call_args.kwargs["input"])["password"], "private-test-value")
        self.assertEqual(run.call_args.kwargs["timeout"], 15.0)

    @patch.object(limits.subprocess, "run")
    def test_timeout_or_sdk_failure_cannot_fall_back_to_model(self, run):
        run.side_effect = subprocess.TimeoutExpired("query", 15)
        with self.assertRaisesRegex(limits.JointLimitsError, "timed out"):
            self.query()
        run.side_effect = None
        run.return_value = subprocess.CompletedProcess([], 1, "", "private-test-value")
        with self.assertRaises(limits.JointLimitsError) as caught:
            self.query()
        self.assertNotIn("private-test-value", str(caught.exception))

    @patch.object(limits.subprocess, "run")
    def test_bad_counts_modes_or_values_are_rejected(self, run):
        for response in (
            self.response(hard=[90.0] * 6), self.response(soft=[]),
            self.response(mode="CARTESIAN_JOYSTICK"),
            self.response(hard=[float("nan")] * 7),
            self.response(soft=[0.0] * 7), self.response(hard=[-1.0] * 7),
            self.response(soft=[True] * 7),
            self.response(hard_acceleration=[0.0] * 7),
            self.response(soft_acceleration=[]), self.response(soft_acceleration=[float("nan")] * 7),
            subprocess.CompletedProcess([], 0, "not json", ""),
        ):
            with self.subTest(response=response), self.assertRaises(limits.JointLimitsError):
                run.return_value = response
                self.query()

    @patch.object(limits.subprocess, "run")
    def test_invalid_connection_settings_fail_before_connecting(self, run):
        for overrides in ({"robot_ip": "0.0.0.0"}, {"username": ""}, {"password": ""},
                          {"timeout_s": 0.0}, {"timeout_s": float("nan")}, {"mode": "IDLE"}):
            with self.subTest(overrides=overrides), self.assertRaises(limits.JointLimitsError):
                self.query(**overrides)
        run.assert_not_called()

    def test_sdk_uses_only_limit_reads_and_closes_session_on_read_failure(self):
        transport, session, control = Mock(), Mock(), Mock()
        control.GetKinematicHardLimits.return_value = SimpleNamespace(joint_speed_limits=[90.0] * 7,
                                                                    joint_acceleration_limits=[180.0] * 7)
        control.GetKinematicSoftLimits.return_value = SimpleNamespace(control_mode=4, joint_speed_limits=[30.0] * 7,
                                                                    joint_acceleration_limits=[60.0] * 7)
        control_pb = SimpleNamespace(
            ControlMode=SimpleNamespace(Value=Mock(return_value=4)),
            ControlModeInformation=lambda **kw: SimpleNamespace(**kw),
        )
        modules = {
            "kortex_api": Mock(),
            "kortex_api.TCPTransport": SimpleNamespace(TCPTransport=Mock(return_value=transport)),
            "kortex_api.RouterClient": SimpleNamespace(RouterClient=Mock(), RouterClientSendOptions=SimpleNamespace),
            "kortex_api.SessionManager": SimpleNamespace(SessionManager=Mock(return_value=session)),
            "kortex_api.autogen": Mock(),
            "kortex_api.autogen.client_stubs": Mock(),
            "kortex_api.autogen.client_stubs.ControlConfigClientRpc": SimpleNamespace(ControlConfigClient=Mock(return_value=control)),
            "kortex_api.autogen.messages": SimpleNamespace(
                ControlConfig_pb2=control_pb, Session_pb2=SimpleNamespace(CreateSessionInfo=lambda **kw: SimpleNamespace(**kw)),
            ),
        }
        request = dict(robot_ip="192.0.2.1", username="test", password="test", mode="ANGULAR_TRAJECTORY")
        with patch.dict("sys.modules", modules):
            result = limits._query_kortex(request)
            self.assertEqual(result["soft"], [30.0] * 7)
            self.assertEqual(result["soft_acceleration"], [60.0] * 7)
            self.assertEqual([call[0] for call in control.method_calls],
                             ["GetKinematicHardLimits", "GetKinematicSoftLimits"])
            self.assertEqual(control.GetKinematicSoftLimits.call_args.args[0].control_mode, 4)
            session.CloseSession.assert_called_once()
            transport.disconnect.assert_called_once()
            control.GetKinematicSoftLimits.side_effect = RuntimeError("read failed")
            with self.assertRaises(RuntimeError):
                limits._query_kortex(request)
            self.assertEqual(session.CloseSession.call_count, 2)
            self.assertEqual(transport.disconnect.call_count, 2)


class AccelerationResolutionTests(unittest.TestCase):
    def test_minimum_model_robot_and_file_acceleration(self):
        _, planning = model_fixture()
        original = copy.deepcopy(planning)
        robot = {f"{kind}_acceleration": {name: value for name in limits.arm_joint_names(7)}
                 for kind, value in (("hard", 10.0), ("soft", 5.0))}
        robot["hard_acceleration"]["joint_3"] = 3.0
        robot["soft_acceleration"]["joint_4"] = 20.0
        result, messages = limits.resolve_acceleration_limits(
            planning, limits.JointLimitSettings("ANGULAR_TRAJECTORY", {}, {"joint_1": 1.0, "joint_2": 20.0}), robot, 7)
        self.assertEqual([result["joint_limits"][f"joint_{i}"]["max_acceleration"] for i in range(1, 5)],
                         [1.0, 5.0, 3.0, 8.6])
        self.assertEqual(result["joint_limits"]["joint_1"]["max_velocity"], 0.8)
        self.assertEqual(planning, original)
        self.assertIn("robot_soft=5", messages[0])

    def test_fake_caps_and_missing_bounds(self):
        _, planning = model_fixture(6)
        settings = limits.JointLimitSettings("ANGULAR_TRAJECTORY", {}, {"joint_1": 0.7})
        result, _ = limits.resolve_acceleration_limits(planning, settings, None, 6)
        self.assertEqual(result["joint_limits"]["joint_1"]["max_acceleration"], 0.7)
        self.assertEqual(result["joint_limits"]["joint_6"]["max_acceleration"], 8.6)
        planning["joint_limits"]["joint_2"]["has_acceleration_limits"] = False
        with self.assertRaises(limits.JointLimitsError):
            limits.resolve_acceleration_limits(planning, settings, None, 6)
        with self.assertRaises(limits.JointLimitsError):
            limits.resolve_acceleration_limits(planning, settings, {}, 6)


if __name__ == "__main__":
    unittest.main()
