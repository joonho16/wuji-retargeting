"""Tests for the interactive Gen3 VIVE Enter deadman helper."""

from __future__ import annotations

import unittest

from example.arm_teleop.gen3_vive_enter_deadman import (
    DEFAULT_DEADMAN_TOPIC,
    DEFAULT_RATE_HZ,
    EnterDeadmanConfig,
    build_parser,
    config_from_args,
)


class EnterDeadmanConfigTest(unittest.TestCase):
    def test_default_cli_starts_false_and_publishes_to_physical_deadman_topic(self) -> None:
        args = build_parser().parse_args([])
        config = config_from_args(args)

        config.validate()
        self.assertEqual(config.topic, DEFAULT_DEADMAN_TOPIC)
        self.assertEqual(config.rate_hz, DEFAULT_RATE_HZ)
        self.assertFalse(config.initial_pressed)
        self.assertTrue(config.true_on_first_enter)

    def test_cli_can_start_true_for_testing(self) -> None:
        args = build_parser().parse_args(
            ["--topic", "/test/deadman", "--rate-hz", "10", "--start-true"]
        )
        config = config_from_args(args)

        config.validate()
        self.assertEqual(config.topic, "/test/deadman")
        self.assertEqual(config.rate_hz, 10.0)
        self.assertTrue(config.initial_pressed)

    def test_config_rejects_empty_topic_or_bad_rate(self) -> None:
        with self.assertRaisesRegex(ValueError, "topic"):
            EnterDeadmanConfig(topic=" ").validate()
        with self.assertRaisesRegex(ValueError, "rate_hz"):
            EnterDeadmanConfig(rate_hz=0.0).validate()


if __name__ == "__main__":
    unittest.main()
