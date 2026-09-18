from pathlib import Path
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from example.operator_hold import Intent, IntentWriter, STOP, TELEOP, HOME
from example.wh110_hold import HandMotion, HandTarget, HeldWh110Backend, load_open_pose

ROOT = Path(__file__).resolve().parents[1]
OPEN = ROOT / 'example/config/wh110_open_pose.yaml'


class HandMotionTests(unittest.TestCase):
    def setUp(self):
        self.actual = np.full((5, 4), .8)
        self.motion = HandMotion(load_open_pose(OPEN, 'left'), np.full((5, 4), -1.), np.full((5, 4), 2.))
        self.off = Intent(STOP, 'stop', True)
        self.a = Intent(TELEOP, 'a1', True)
        self.b = Intent(HOME, 'b1', True)
        self.motion.step(self.off, self.actual, None, now=0.)

    def test_b_opens_without_manus_and_release_holds_feedback(self):
        target, enabled = self.motion.step(self.b, self.actual, None, now=.02)
        self.assertTrue(enabled)
        self.assertGreater(np.max(np.abs(target-self.actual)), 0.)
        self.assertLessEqual(np.max(np.abs(target-self.actual)), .0120001)
        measured = np.full((5, 4), .75)
        stopped, enabled = self.motion.step(self.off, measured, None, now=.04)
        self.assertFalse(enabled)
        np.testing.assert_array_equal(stopped, measured)
        later, _ = self.motion.step(self.off, measured+.02, None, now=.06)
        np.testing.assert_array_equal(later, measured)  # hold, not tracking disturbances

    def test_a_requires_fresh_matching_target_and_stops_on_lease_loss(self):
        packet = HandTarget(np.zeros((5, 4)), .01, self.a.token)
        target, enabled = self.motion.step(self.a, self.actual, packet, now=.02)
        self.assertTrue(enabled)
        np.testing.assert_array_equal(target, self.actual)
        target, _ = self.motion.step(self.a, self.actual, packet, now=.03)
        self.assertTrue(np.all(target < self.actual))
        stopped, _ = self.motion.step(Intent(), self.actual, packet, now=.04)
        np.testing.assert_array_equal(stopped, self.actual)
        for intent, now in ((Intent(TELEOP, 'a2', True), .05), (self.a, .2)):
            target, enabled = self.motion.step(intent, self.actual, packet, now=now)
            self.assertFalse(enabled)
            np.testing.assert_array_equal(target, self.actual)

    def test_teleop_soft_start_blends_for_half_second_then_follows_normally(self):
        history = []
        for i in range(51):
            now = .02 + i*.01
            packet = HandTarget(np.zeros((5, 4)), now, self.a.token)
            target, enabled = self.motion.step(self.a, self.actual, packet, now=now)
            self.assertTrue(enabled)
            history.append(target)
        np.testing.assert_array_equal(history[0], self.actual)
        # Gentle initial motion, halfway around 250 ms, no extra startup lag
        # remaining after 500 ms for a stationary glove target.
        self.assertTrue(np.all(history[10] > .7))
        self.assertTrue(np.all((history[25] > .39) & (history[25] < .41)))
        np.testing.assert_allclose(history[50], 0., atol=1e-6)
        self.assertTrue(np.all(np.diff(np.asarray(history), axis=0) <= 0.))
        # Subsequent movement uses the ordinary filter, without a fresh ramp.
        target, _ = self.motion.step(self.a, self.actual,
                                    HandTarget(np.ones((5, 4)), .54, self.a.token), now=.54)
        self.assertTrue(np.all((target > .4) & (target < .5)))

    def test_soft_start_waits_for_first_valid_glove_target_and_uses_live_targets(self):
        for now in (.02, .4, 1.0):
            target, enabled = self.motion.step(self.a, self.actual, None, now=now)
            self.assertFalse(enabled)
        target, _ = self.motion.step(self.a, self.actual,
                                    HandTarget(np.zeros((5, 4)), 1.02, self.a.token), now=1.02)
        np.testing.assert_array_equal(target, self.actual)
        for i in range(1, 51):
            now = 1.02+i*.01
            # Glove changes during startup: do not finish at a captured old pose.
            goal = np.full((5, 4), 1.2 if i >= 20 else 0.)
            target, _ = self.motion.step(self.a, self.actual,
                                        HandTarget(goal, now, self.a.token), now=now)
        np.testing.assert_allclose(target, 1.2, atol=1e-4)

    def test_release_mid_start_stops_and_new_hold_starts_at_measured_pose(self):
        for i in range(11):
            now = .02+i*.02
            self.motion.step(self.a, self.actual,
                             HandTarget(np.zeros((5, 4)), now, self.a.token), now=now)
        measured = np.full((5, 4), .63)
        target, enabled = self.motion.step(self.off, measured, None, now=.24)
        self.assertFalse(enabled)
        np.testing.assert_array_equal(target, measured)
        a2 = Intent(TELEOP, 'a2', True)
        target, _ = self.motion.step(a2, measured,
                                    HandTarget(np.zeros((5, 4)), .26, a2.token), now=.26)
        np.testing.assert_array_equal(target, measured)
        # Even a release shorter than a worker tick has a new generation.
        a3 = Intent(TELEOP, 'a3', True)
        target, _ = self.motion.step(a3, measured+.1,
                                    HandTarget(np.zeros((5, 4)), .28, a3.token), now=.28)
        np.testing.assert_array_equal(target, measured+.1)

    def test_stale_input_and_return_from_b_restart_soft_start(self):
        self.motion.step(self.a, self.actual,
                         HandTarget(np.zeros((5, 4)), .02, self.a.token), now=.02)
        target, enabled = self.motion.step(self.a, self.actual,
                                          HandTarget(np.zeros((5, 4)), .02, self.a.token), now=.2)
        self.assertFalse(enabled)
        measured = self.actual-.1
        target, _ = self.motion.step(self.a, measured,
                                    HandTarget(np.zeros((5, 4)), .22, self.a.token), now=.22)
        np.testing.assert_array_equal(target, measured)
        self.motion.step(self.b, measured, None, now=.24)
        a2 = Intent(TELEOP, 'a2', True)
        target, _ = self.motion.step(a2, measured-.1,
                                    HandTarget(np.zeros((5, 4)), .26, a2.token), now=.26)
        np.testing.assert_array_equal(target, measured-.1)

    def test_invalid_targets_and_device_limits_cannot_move(self):
        packet = HandTarget(np.full((5, 4), np.nan), .01, self.a.token)
        _, enabled = self.motion.step(self.a, self.actual, packet, now=.02)
        self.assertFalse(enabled)
        with self.assertRaises(ValueError):
            HandMotion(np.full((5, 4), 3.), np.full((5, 4), -1.), np.full((5, 4), 2.))

    def test_open_pose_fits_model_and_four_fingers_point_along_palm(self):
        from scipy.spatial.transform import Rotation
        root = ET.parse(ROOT / 'wuji_retargeting/wuji-description/hand/body/urdf/left.urdf')
        opened = load_open_pose(OPEN, 'left')
        for f in range(1, 6):
            orientation = np.eye(3)
            for j in range(1, 5):
                joint = root.find(f"joint[@name='left_finger{f}_joint{j}']")
                q = opened[f-1, j-1]
                self.assertGreaterEqual(q, float(joint.find('limit').get('lower')))
                self.assertLessEqual(q, float(joint.find('limit').get('upper')))
                fixed = Rotation.from_euler('xyz', [float(v) for v in joint.find('origin').get('rpy').split()])
                axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
                orientation = orientation @ fixed.as_matrix() @ Rotation.from_rotvec(q*axis).as_matrix()
                if f > 1 and j >= 2:
                    np.testing.assert_allclose(orientation[:, 2], (0., 0., 1.), atol=1e-6)
            if f == 1:
                self.assertLess(orientation[1, 2], -.9)  # thumb extended outwards


class FakeHand:
    def __init__(self):
        self.actual = np.full((5, 4), .8)
        self.enabled = []
        self.targets = []
        self.closed = False
    def write_joint_enabled(self, value, timeout=.5): self.enabled.append(value)
    def read_joint_actual_position(self, timeout): return self.actual.copy()
    def read_joint_lower_limit(self, timeout): return np.full((5, 4), -1.)
    def read_joint_upper_limit(self, timeout): return np.full((5, 4), 2.)
    def realtime_controller(self, **kwargs): return self
    def set_joint_target_position(self, value): self.targets.append(np.asarray(value).copy())
    def close(self): self.closed = True


class OutputWorkerTests(unittest.TestCase):
    def test_independent_worker_stops_when_writer_disappears(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'intent.json'
            writer = IntentWriter(path)
            hand = FakeHand()
            backend = HeldWh110Backend(state_path=path, open_pose=load_open_pose(OPEN, 'left'),
                                       hand_factory=lambda: hand)
            try:
                def publish(mode, duration):
                    until = time.monotonic() + duration
                    while time.monotonic() < until:
                        writer.publish(mode)
                        time.sleep(.01)
                publish(STOP, .08)
                self.assertNotIn(True, hand.enabled)
                publish(HOME, .15)  # no MANUS or main-loop send calls
                self.assertIn(True, hand.enabled)
                self.assertTrue(any(np.any(t < hand.actual) for t in hand.targets))
                time.sleep(.17)  # writer/GUI vanished while B was held
                np.testing.assert_array_equal(hand.targets[-1], hand.actual)
                count = len(hand.targets)
                publish(HOME, .08)  # stale hold cannot automatically resume
                for t in hand.targets[count:]: np.testing.assert_array_equal(t, hand.actual)
            finally:
                writer.close()
                backend.close()
            self.assertFalse(hand.enabled[-1])
            self.assertTrue(hand.closed)


if __name__ == '__main__': unittest.main()
