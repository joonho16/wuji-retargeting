from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

from example.arm_teleop.gen3_native_core import (NativeSession, Gen3ViveCalibration, ToolPose, ViveTrackerPose,
                  native_vectors, base_to_tool)
from example.arm_teleop.gen3_native_stack import native_launch_command


def quat(axis, angle):
    s = math.sin(angle/2)
    return (*[x*s for x in axis], math.cos(angle/2))


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def calibration():
    return Gen3ViveCalibration.from_mapping(dict(
        source_frame="steamvr_world", planning_frame="base_link", tracker_role="gen3_tracker",
        tracker_mount_frame="tracker", T_planning_source=dict(translation=[0, 0, 0], rotation_xyzw=[0, 0, 0, 1]),
        T_tool_tracker_mount=dict(translation=[0, 0, 0], rotation_xyzw=[0, 0, 0, 1]),
        linear_scale=1., angular_scale=1., max_linear_mps=1., max_angular_radps=2., stale_timeout_s=.1,
    ))


class NativeTests(unittest.TestCase):
    def setUp(self):
        self.session = NativeSession(calibration(), motion="both", max_linear=1., max_angular=2.)
        self.tool = ToolPose((0., 0., 0.), (0., 0., 0., 1.))
        self.pose = ViveTrackerPose("gen3_tracker", "steamvr_world", (0., 0., 0.), (0., 0., 0., 1.), 0.)

    def step(self, t, *, q=None, tool=None, pressed=True, **gates):
        pose = replace(self.pose, received_monotonic_s=t,
                       quaternion_xyzw=q if q is not None else self.pose.quaternion_xyzw)
        return self.session.step(now=t, pose=pose, tool=tool or self.tool, pressed=pressed, **gates)

    def assertVector(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=8)

    def test_ten_degree_error_is_ten_deg_per_second_native(self):
        self.step(0.)
        command = self.step(.02, q=quat((0, 0, 1), math.radians(10)))
        self.assertVector(command.angular_xyz, (0., 0., math.radians(10)))
        self.assertVector(native_vectors(command, self.tool)[1], (0., 0., 10.))

    def test_tool_rotation_changes_both_velocity_axes(self):
        q = quat((0, 0, 1), math.pi/2)
        self.assertVector(base_to_tool((1, 0, 0), q), (0, -1, 0))
        self.step(0.)
        command = replace(self.step(.02), linear_xyz=(.1, 0, 0), angular_xyz=(.2, 0, 0))
        linear, angular = native_vectors(command, replace(self.tool, quaternion_xyzw=q))
        self.assertVector(linear, (0, -.1, 0))
        self.assertVector(angular, (0, -math.degrees(.2), 0))

    def test_rotation_at_tilted_neutral_is_spatial_error_not_euler_subtraction(self):
        neutral = quat((1, 0, 0), math.pi/2)
        tool = replace(self.tool, quaternion_xyzw=neutral)
        self.step(0., tool=tool)
        delta = quat((0, 1, 0), .15)
        command = self.step(.02, q=delta, tool=tool)
        self.assertVector(command.angular_xyz, (0, .15, 0))
        self.assertVector(native_vectors(command, tool)[1], (0, 0, -math.degrees(.15)))
        reached = replace(tool, quaternion_xyzw=multiply(delta, neutral))
        stopped = self.step(.04, q=delta, tool=reached)
        self.assertVector(stopped.angular_xyz, (0, 0, 0))

    def test_shortest_rotation_across_180_degrees(self):
        self.tool = replace(self.tool, quaternion_xyzw=quat((0, 0, 1), math.radians(179)))
        self.step(0.)
        current = replace(self.tool, quaternion_xyzw=quat((0, 0, 1), math.radians(-179)))
        command = self.step(.02, tool=current)
        self.assertVector(native_vectors(command, current)[1], (0, 0, -2))

    def test_quaternion_sign_does_not_change_command(self):
        self.step(0.)
        q = quat((0, 1, 0), .1)
        a = self.step(.02, q=q)
        b = self.step(.04, q=tuple(-x for x in q))
        self.assertVector(a.angular_xyz, b.angular_xyz)

    def test_calibration_axis_mapping_applies_to_rotation(self):
        c = replace(calibration(), T_planning_source=replace(
            calibration().T_planning_source, rotation_xyzw=(.5, -.5, -.5, .5)))
        self.session = NativeSession(c, max_angular=2.)
        self.step(0.)
        command = self.step(.02, q=quat((1, 0, 0), .1))
        self.assertVector(command.angular_xyz, (0, -.1, 0))

    def test_magnitude_cap_before_degree_conversion(self):
        self.session = NativeSession(calibration(), max_angular=.2)
        self.step(0.)
        axis = (1/math.sqrt(3),)*3
        command = self.step(.02, q=quat(axis, 1.))
        self.assertAlmostEqual(math.sqrt(sum(x*x for x in command.angular_xyz)), .2)
        self.assertAlmostEqual(math.sqrt(sum(x*x for x in native_vectors(command, self.tool)[1])), math.degrees(.2))

    def test_test_caps_cannot_raise_calibration_caps(self):
        session = NativeSession(replace(calibration(), max_linear_mps=.01, max_angular_radps=.05))
        self.assertEqual(session.calibration.max_linear_mps, .01)
        self.assertEqual(session.calibration.max_angular_radps, .05)

    def test_translation_and_rotation_isolation(self):
        for motion in ("rotation", "translation"):
            self.session = NativeSession(calibration(), motion=motion)
            self.pose = replace(self.pose, position_xyz=(0., 0., 0.))
            self.step(0.)
            self.pose = replace(self.pose, position_xyz=(.1, 0., 0.))
            command = self.step(.02, q=quat((0, 0, 1), .1))
            suppressed = command.linear_xyz if motion == "rotation" else command.angular_xyz
            self.assertVector(suppressed, (0, 0, 0))

    def test_translation_scale_preserves_rotation_and_calibrated_axes(self):
        c = calibration()
        c = replace(c, linear_scale=1.5, T_planning_source=replace(
            c.T_planning_source, rotation_xyzw=(.5, -.5, -.5, .5)))
        self.session = NativeSession(c)
        self.step(0.)
        self.pose = replace(self.pose, position_xyz=(.1, 0., 0.))
        command = self.step(.02, q=quat((1, 0, 0), .1))
        self.assertVector(command.linear_xyz, (0., -.15, 0.))
        self.assertVector(command.angular_xyz, (0., -.1, 0.))

    def test_combined_pose_error_caps_both_vectors_and_stops_when_reached(self):
        self.session = NativeSession(calibration())
        neutral_q = quat((0, 0, 1), math.pi/2)
        self.tool = replace(self.tool, quaternion_xyzw=neutral_q)
        self.step(0.)
        self.pose = replace(self.pose, position_xyz=(1.2, 1.6, 0.))
        delta = quat((0, 1, 0), 2.)
        command = self.step(.02, q=delta)
        self.assertVector(command.linear_xyz, (.3, .4, 0.))
        linear, angular = native_vectors(command, self.tool)
        self.assertVector(linear, (.4, -.3, 0.))
        self.assertVector(angular, (100., 0., 0.))
        reached = replace(self.tool, position_xyz=self.pose.position_xyz,
                          quaternion_xyzw=multiply(delta, neutral_q))
        stopped = self.step(.04, q=delta, tool=reached)
        self.assertVector(stopped.linear_xyz, (0., 0., 0.))
        self.assertVector(stopped.angular_xyz, (0., 0., 0.))
        released = self.step(.06, q=delta, pressed=False)
        self.assertFalse(released.enabled)
        self.assertVector(released.linear_xyz, (0., 0., 0.))
        self.assertVector(released.angular_xyz, (0., 0., 0.))

    def test_stale_inputs_and_controller_failure_require_new_deadman_edge(self):
        for failure in ({"deadman_fresh": False}, {"feedback_fresh": False},
                        {"controller_ready": False}, {"input_error": "invalid"}):
            with self.subTest(failure=failure):
                self.session = NativeSession(calibration())
                self.step(0.)
                stopped = self.step(.02, **failure)
                self.assertFalse(stopped.enabled)
                self.assertVector(native_vectors(stopped, self.tool)[1], (0, 0, 0))
                self.assertFalse(self.step(.04).enabled)
                self.step(.06, pressed=False)
                recaptured = self.step(.08)
                self.assertEqual(recaptured.stop_reason, "NEUTRAL_CAPTURED")

    def test_tracker_dropout_stops_on_timer_without_new_pose(self):
        self.step(0.)
        self.step(.02, q=quat((0, 0, 1), .1))
        command = self.session.step(now=.2, pose=None, tool=self.tool, pressed=True)
        self.assertEqual(command.stop_reason, "STALE_STOP")
        self.assertFalse(command.enabled)

    def test_invalid_tcp_never_reaches_native_conversion(self):
        for q in ((0, 0, 0, 0), (0, 0, float("nan"), 1)):
            command = self.step(0., tool=replace(self.tool, quaternion_xyzw=q))
            self.assertFalse(command.enabled)

    def test_driver_spawns_different_active_and_inactive_controllers(self):
        command = native_launch_command("192.0.2.1")
        self.assertIn("robot_controller:=twist_controller", command)
        self.assertIn("robot_pos_controller:=joint_trajectory_controller", command)
        self.assertIn("use_internal_bus_gripper_comm:=false", command)
        self.assertFalse(any(arg.startswith("gripper:=") for arg in command))
        self.assertNotIn("moveit_servo", " ".join(command))


if __name__ == "__main__":
    unittest.main()
