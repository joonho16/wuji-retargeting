import json
from pathlib import Path
import tempfile
import unittest

from example.operator_hold import HoldKeys, IntentWriter, IntentReader, read_intent, STOP, TELEOP, HOME


class HoldTests(unittest.TestCase):
    def test_actual_key_hold_release_focus_and_conflict(self):
        keys = HoldKeys()
        update = lambda a=False, b=False, focused=True: keys.update(a=a, b=b, focused=focused)
        self.assertEqual(update(a=True), STOP)  # held at startup
        self.assertEqual(update(), STOP)
        self.assertEqual(update(a=True), TELEOP)
        self.assertEqual(update(a=True), TELEOP)  # no OS key repeat necessary
        self.assertEqual(update(), STOP)
        self.assertEqual(update(b=True), HOME)
        self.assertEqual(update(b=True, focused=False), STOP)
        self.assertEqual(update(b=True), STOP)  # focus regain cannot resume
        update()
        self.assertEqual(update(a=True, b=True), STOP)
        self.assertEqual(update(a=True), STOP)  # both must be released
        update()
        self.assertEqual(update(a=True), TELEOP)
        self.assertEqual(update(b=True), STOP)  # no neutral sample between modes

    def test_lease_timeout_restart_and_new_press_identity(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "intent.json"
            reader = IntentReader(path)
            self.assertFalse(reader.read(now=10).fresh)
            writer = IntentWriter(path)
            try:
                writer.publish(TELEOP, now=10)
                self.assertEqual(reader.read(now=10).mode, STOP)
                writer.publish(STOP, now=10.01)
                reader.read(now=10.01)
                writer.publish(TELEOP, now=10.02)
                first = reader.read(now=10.02)
                self.assertEqual(first.mode, TELEOP)
                self.assertEqual(reader.read(now=10.2).mode, STOP)
                writer.publish(TELEOP, now=10.21)
                self.assertEqual(reader.read(now=10.21).mode, STOP)
                writer.publish(STOP, now=10.22)
                reader.read(now=10.22)
                writer.publish(TELEOP, now=10.23)
                self.assertNotEqual(first.token, reader.read(now=10.23).token)
                self.assertEqual(read_intent(path, now=9).mode, STOP)
            finally:
                writer.close()
            self.assertEqual(read_intent(path).mode, STOP)

    def test_new_writer_and_malformed_state_fail_closed(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "intent.json"
            reader = IntentReader(path)
            first = IntentWriter(path)
            reader.read()
            with self.assertRaisesRegex(RuntimeError, "already running"):
                IntentWriter(path)
            first.close()
            second = IntentWriter(path)
            try:
                second.publish(HOME)
                self.assertEqual(reader.read().mode, STOP)
                for malformed in ("{}", "{", "null", "[]", json.dumps({"monotonic": float('nan')})):
                    path.write_text(malformed)
                    self.assertFalse(read_intent(path).fresh)
            finally:
                second.close()


if __name__ == "__main__":
    unittest.main()
