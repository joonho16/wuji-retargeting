"""Safety checks for following a hand control point with a TCP Twist command."""
from dataclasses import replace
import math
import unittest

from example.arm_teleop.gen3_native_core import NativeSession, ToolPose, ViveTrackerPose
from tests.test_gen3_native_core import calibration, multiply, quat


class ControlPointTests(unittest.TestCase):
    def setUp(self):
        self.tool = ToolPose((0., 0., 0.), (0., 0., 0., 1.))
        self.pose = ViveTrackerPose("gen3_tracker", "steamvr_world", (0., 0., 0.),
                                    (0., 0., 0., 1.), 0.)

    def config(self, offset=(.2, 0., 0.), mount_q=(0., 0., 0., 1.), **limits):
        original = calibration()
        return replace(original, T_tool_tracker_mount=replace(
            original.T_tool_tracker_mount, translation=offset, rotation_xyzw=mount_q), **limits)

    def step(self, session, now, *, tool=None, q=None, position=None, **gates):
        pose = replace(self.pose, received_monotonic_s=now,
                       position_xyz=self.pose.position_xyz if position is None else position,
                       quaternion_xyzw=self.pose.quaternion_xyzw if q is None else q)
        return session.step(now=now, pose=pose, tool=self.tool if tool is None else tool,
                            pressed=gates.pop("pressed", True), **gates)

    def assertVector(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=9)

    def assertZero(self, command, *, exact=False):
        for vector in (command.linear_xyz, command.angular_xyz):
            if exact:
                self.assertEqual(vector, (0., 0., 0.))
            else:
                self.assertVector(vector, (0., 0., 0.))

    def test_nonzero_mount_neutral_is_stationary_and_uses_rigid_transform(self):
        tool_q = quat((0, 1, 0), math.pi / 2)
        mount_q = quat((1, 0, 0), math.pi / 6)
        tool = ToolPose((.4, -.3, .6), tool_q)
        session = NativeSession(self.config((.2, .03, .04), mount_q))
        control = session.control_pose(tool)
        self.assertVector(control.position_xyz, (.44, -.27, .4))
        self.assertVector(control.quaternion_xyzw, multiply(tool_q, mount_q))
        captured = self.step(session, 0., tool=tool)
        self.assertEqual(captured.stop_reason, "NEUTRAL_CAPTURED")
        self.assertZero(captured, exact=True)
        self.assertZero(self.step(session, .02, tool=tool))

    def test_achieved_ninety_degree_rotation_at_fixed_control_point_stops(self):
        session = NativeSession(self.config(mount_q=quat((1, 0, 0), .3)))
        neutral = ToolPose((.4, -.3, .6), (0., 0., 0., 1.))
        self.step(session, 0., tool=neutral)
        delta = quat((0, 0, 1), math.pi / 2)
        # Control point stays at (.6, -.3, .6); the TCP moves around it.
        reached = ToolPose((.6, -.5, .6), delta)
        self.assertVector(session.control_pose(reached).position_xyz, (.6, -.3, .6))
        command = self.step(session, .02, q=delta, tool=reached)
        self.assertTrue(command.enabled)
        self.assertZero(command)

    def test_rotation_compensation_has_correct_sign_and_uses_current_tcp_axes(self):
        cases = ((self.tool, (0, 0, 1), (0., -.08, 0.)),
                 (replace(self.tool, quaternion_xyzw=quat((0, 1, 0), math.pi / 2)),
                  (0, 1, 0), (.08, 0., 0.)))
        for motion in ("both", "rotation"):
            for tool, axis, expected_linear in cases:
                with self.subTest(motion=motion, tool_q=tool.quaternion_xyzw):
                    session = NativeSession(self.config(), motion=motion)
                    self.step(session, 0., tool=tool)
                    command = self.step(session, .02, tool=tool, q=quat(axis, .4))
                    self.assertVector(command.linear_xyz, expected_linear)
                    self.assertVector(command.angular_xyz, tuple(.4 * component for component in axis))
        translation = NativeSession(self.config(), motion="translation")
        self.step(translation, 0.)
        command = self.step(translation, .02, q=quat((0, 0, 1), .4), position=(.1, 0., 0.))
        self.assertVector(command.linear_xyz, (.1, 0., 0.))
        self.assertVector(command.angular_xyz, (0., 0., 0.))

    def test_tcp_speed_cap_scales_both_vectors_to_preserve_pivot_and_calibration_caps(self):
        # An intentionally large synthetic lever arm makes saturation unambiguous.
        for limits, expected_linear, expected_angular in (
            ({}, .5, .25),
            ({"max_linear_mps": .08, "max_angular_radps": .2}, .08, .04),
            ({"max_linear_mps": .08, "max_angular_radps": .02}, .04, .02),
        ):
            with self.subTest(limits=limits):
                session = NativeSession(self.config((2., 0., 0.), **limits))
                self.step(session, 0.)
                command = self.step(session, .02, q=quat((0, 0, 1), 1.))
                self.assertVector(command.linear_xyz, (0., -expected_linear, 0.))
                self.assertVector(command.angular_xyz, (0., 0., expected_angular))
                # v_control = v_TCP + omega x r must remain zero after capping.
                self.assertVector((command.linear_xyz[0],
                                   command.linear_xyz[1] + 2. * command.angular_xyz[2],
                                   command.linear_xyz[2]), (0., 0., 0.))

    def test_release_and_stale_feedback_zero_offset_compensation_and_require_recapture(self):
        for failure in ({"pressed": False}, {"feedback_fresh": False}):
            with self.subTest(failure=failure):
                session = NativeSession(self.config())
                delta = quat((0, 0, 1), .2)
                self.step(session, 0.)
                moving = self.step(session, .02, q=delta)
                self.assertNotEqual(moving.linear_xyz, (0., 0., 0.))
                stopped = self.step(session, .04, q=delta, **failure)
                self.assertFalse(stopped.enabled)
                self.assertZero(stopped, exact=True)
                if "feedback_fresh" in failure:
                    blocked = self.step(session, .06, q=delta)
                    self.assertFalse(blocked.enabled)
                    self.assertZero(blocked, exact=True)
                self.step(session, .08, q=delta, pressed=False)
                moved_tool = ToolPose((.7, .1, .3), quat((1, 0, 0), .3))
                recaptured = self.step(session, .1, q=delta, tool=moved_tool)
                self.assertEqual(recaptured.stop_reason, "NEUTRAL_CAPTURED")
                self.assertZero(recaptured, exact=True)
                self.assertZero(self.step(session, .12, q=delta, tool=moved_tool))

    def test_invalid_mount_is_rejected_before_session_can_command_motion(self):
        for offset, mount_q in (
            ((float("nan"), 0., 0.), (0., 0., 0., 1.)),
            ((.1, .2), (0., 0., 0., 1.)),
            ((.1, 0., 0.), (0., 0., 0., 0.)),
            ((.1, 0., 0.), (0., 0., 0., 2.)),
            ((.1, 0., 0.), (0., float("inf"), 0., 1.)),
        ):
            with self.subTest(offset=offset, mount_q=mount_q), self.assertRaises(ValueError):
                NativeSession(self.config(offset, mount_q))


if __name__ == "__main__":
    unittest.main()
