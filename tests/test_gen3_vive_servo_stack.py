"""Launch parameter integration tests, enabled in the ROS2 Compose image."""

import contextlib
import io
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

try:
    import ament_index_python  # noqa: F401
except ModuleNotFoundError:
    stack = None
else:
    from example.arm_teleop import gen3_vive_servo_stack as stack


@unittest.skipIf(stack is None, "ROS2 launch integration requires the Gen3 Compose image")
class ServoStackTests(unittest.TestCase):
    def setUp(self):
        # Keep the sourced ROS environment while isolating operator overrides.
        self.environment = {key: value for key, value in os.environ.items() if not key.startswith("GEN3_")}

    def test_fake_and_real_limits_reach_servo_parameters(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "limits.yaml"
            config.write_text("joint_limits: {joint_1: {max_velocity: 0.1, max_acceleration: 0.8}}")
            for dof in (6, 7):
                for fake in (True, False):
                    env = {**self.environment, "GEN3_DOF": str(dof), "GEN3_USE_FAKE_HARDWARE": str(fake),
                           "GEN3_JOINT_LIMITS_CONFIG": str(config), "GEN3_ROBOT_IP": "192.0.2.1"}
                    robot = {source: {f"joint_{i}": value for i in range(1, dof + 1)}
                             for source, value in (("hard", 0.5), ("soft", 0.2),
                                                   ("hard_acceleration", 4.0), ("soft_acceleration", 2.0))}
                    with self.subTest(dof=dof, fake=fake), patch.dict("os.environ", env, clear=True), \
                            patch.object(stack, "query_robot_limits", return_value=robot) as query, \
                            patch.object(stack, "Node", wraps=stack.Node) as node, \
                            contextlib.redirect_stdout(io.StringIO()):
                        stack._generate_launch_description()
                        params = node.call_args.kwargs["parameters"]
                        joints = params[0]["robot_description_planning"]["joint_limits"]
                        self.assertEqual(joints["joint_1"]["max_velocity"], 0.1)
                        self.assertEqual(joints["joint_1"]["max_acceleration"], 0.8)
                        self.assertTrue(params[2]["gen3_acceleration_limiting"])
                        self.assertNotIn("gen3_max_joint_tracking_error", params[1])
                        self.assertEqual(params[1]["moveit_servo"]["override_velocity_scaling_factor"], 0.0)
                        if fake:
                            query.assert_not_called()
                        else:
                            query.assert_called_once()
                            self.assertEqual(joints[f"joint_{dof}"]["max_velocity"], 0.2)
                            self.assertEqual(joints[f"joint_{dof}"]["max_acceleration"], 2.0)

    def test_discovery_failure_prevents_all_launch_actions(self):
        with patch.dict("os.environ", {**self.environment, "GEN3_USE_FAKE_HARDWARE": "false"}, clear=True), \
                patch.object(stack, "query_robot_limits", side_effect=stack.JointLimitsError("test failure")), \
                patch.object(stack, "IncludeLaunchDescription") as include, \
                patch.object(stack, "LaunchService") as service, \
                contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(stack.main(), 2)
            include.assert_not_called()
            service.assert_not_called()

    def test_servo_override_cannot_bypass_joint_limiting(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "servo.yaml"
            config.write_text("override_velocity_scaling_factor: 0.5")
            with self.assertRaises(stack.JointLimitsError):
                stack._load_servo_params(config)
            config.write_text("low_latency_mode: true")
            with self.assertRaises(stack.JointLimitsError):
                stack._load_servo_params(config)

    def test_legacy_tracking_error_setting_is_not_forwarded(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "servo.yaml"
            config.write_text("gen3_max_joint_tracking_error: 0.08")
            params = stack._load_servo_params(config)
            self.assertNotIn("gen3_max_joint_tracking_error", params)
            self.assertNotIn("gen3_max_joint_tracking_error", params["moveit_servo"])

    def test_missing_overlay_prevents_starting_any_driver(self):
        with patch.dict("os.environ", self.environment, clear=True), \
                patch.object(stack.Path, "is_file", return_value=False), \
                patch.object(stack, "query_robot_limits") as query, \
                patch.object(stack, "IncludeLaunchDescription") as include, \
                contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(stack.main(), 2)
            query.assert_not_called()
            include.assert_not_called()

    def test_old_overlay_prevents_starting_any_driver(self):
        read_text = Path.read_text

        def old_version(path, *args, **kwargs):
            if path.name == "gen3_acceleration_limiting.version":
                return "3\n"
            return read_text(path, *args, **kwargs)

        with patch.dict("os.environ", self.environment, clear=True), \
                patch.object(stack.Path, "read_text", old_version), \
                patch.object(stack, "query_robot_limits") as query, \
                patch.object(stack, "IncludeLaunchDescription") as include, \
                contextlib.redirect_stderr(io.StringIO()) as error:
            self.assertEqual(stack.main(), 2)
            self.assertIn("outdated", error.getvalue())
            query.assert_not_called()
            include.assert_not_called()

    def test_missing_driver_guard_prevents_real_hardware_start(self):
        is_file = Path.is_file

        def missing_guard(path):
            return False if path.name == "cyclic_failure_guard.version" else is_file(path)

        with patch.dict("os.environ", {**self.environment, "GEN3_USE_FAKE_HARDWARE": "false"}, clear=True), \
                patch.object(stack.Path, "is_file", missing_guard), \
                patch.object(stack, "query_robot_limits") as query, \
                patch.object(stack, "IncludeLaunchDescription") as include, \
                contextlib.redirect_stderr(io.StringIO()) as error:
            self.assertEqual(stack.main(), 2)
            self.assertIn("cyclic failure guard is missing", error.getvalue())
            query.assert_not_called()
            include.assert_not_called()


if __name__ == "__main__":
    unittest.main()
