from concurrent.futures import Future
from types import SimpleNamespace as NS
import unittest

from example.operator_hold import Intent, HOME, STOP
from example.arm_teleop.gen3_home_control import HomeControl


class HomeHandoverTests(unittest.TestCase):
    def setUp(self):
        self.switches, self.commands = [], []
        def switch(active):
            future = Future()
            self.switches.append((active, future))
            return future
        self.home = HomeControl(action_name='ready', switch=switch,
                                publish=self.commands.append, log=lambda _: None)
        self.b = Intent(HOME, 'b1', True, .0)
        self.off = Intent(STOP, 'off', True)

    def tick(self, intent, now=0.):
        self.home.tick(intent, controller_ready=True, now=now)

    def finish_switch(self, ok=True):
        self.switches[-1][1].set_result(NS(ok=ok))

    def start(self):
        self.tick(self.b)
        self.assertFalse(self.commands)
        self.finish_switch()
        self.tick(self.b)
        self.assertTrue(self.commands[-1].startswith('H\nb1\n'))

    def test_release_during_switch_never_executes(self):
        self.tick(self.b)
        self.tick(self.off)
        self.finish_switch()
        self.tick(self.off)
        self.assertFalse(self.commands)
        self.assertTrue(self.switches[-1][0])

    def test_release_stops_before_twist_and_requires_matching_ack(self):
        self.start()
        self.tick(self.off, .01)
        self.assertTrue(self.commands[-1].startswith('S\nb1\n'))
        self.home.on_status('old\nstopped', now=.02)
        self.tick(self.off, .02)
        self.assertEqual(len(self.switches), 1)
        self.home.on_status('b1\nstopped', now=.03)
        self.tick(self.off, .03)
        self.assertTrue(self.switches[-1][0])

    def test_stale_input_and_missing_stop_ack_never_restore_twist(self):
        self.start()
        self.tick(Intent(), .11)
        self.tick(self.off, 4.2)
        self.assertEqual(self.home.state, 'fault')
        self.assertEqual(len(self.switches), 1)
        self.assertFalse(self.home.teleop_allowed)

    def test_completion_does_not_switch_or_start_again_while_held(self):
        self.start()
        self.home.on_status('b1\ncomplete', now=.01)
        self.tick(self.b, .01)
        for index in range(5):
            self.tick(self.b, .02 + index * .01)
        self.assertEqual(self.home.state, 'held')
        self.assertEqual(len(self.switches), 1)
        self.assertFalse(self.home.teleop_allowed)
        self.assertTrue(all(command.split('\n')[1] == 'b1' for command in self.commands))

    def test_failed_switch_never_publishes_hold(self):
        self.tick(self.b)
        self.finish_switch(False)
        self.tick(self.b)
        self.assertEqual(self.home.state, 'fault')
        self.assertFalse(any(command.startswith('H') for command in self.commands))

    def test_close_stops_and_original_operator_stamp_is_preserved(self):
        self.start()
        self.tick(self.b, .08)
        self.assertEqual(self.commands[-1].split('\n')[2], '0')
        self.home.close()
        self.assertTrue(self.commands[-1].startswith('S\nb1\n'))
        self.assertEqual(len(self.switches), 1)


if __name__ == '__main__':
    unittest.main()
