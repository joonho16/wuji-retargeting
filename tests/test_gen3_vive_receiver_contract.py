"""Contract tests for the Linux Gen3 VIVE receiver mock path."""

from __future__ import annotations

import unittest

from tests.helpers.gen3_vive_receiver_contract import (
    EXPECTED_FRAME,
    EXPECTED_ROLE,
    ReceiverContractError,
    role_serial_selection_key,
    validate_role_frame_contract,
)


class Gen3ViveReceiverRoleFrameContractTest(unittest.TestCase):
    def valid_packet(self) -> dict[str, object]:
        return {
            "role": EXPECTED_ROLE,
            "frame": EXPECTED_FRAME,
            "serial": "LHR-UNIT-001",
            "pos": [0.1, 0.2, 0.3],
            "quat": [0.0, 0.0, 0.0, 1.0],
            "t": 1_782_287_700.0,
        }

    def test_valid_gen3_tracker_packet_uses_expected_linux_topic(self) -> None:
        status = validate_role_frame_contract(self.valid_packet())

        self.assertEqual(status.role, "gen3_tracker")
        self.assertEqual(status.frame, "steamvr_world")
        self.assertEqual(status.topic, "/vive/gen3_tracker/pose")

    def test_wrong_role_is_rejected_on_linux(self) -> None:
        packet = self.valid_packet() | {"role": "left_hand"}

        with self.assertRaisesRegex(ReceiverContractError, "role"):
            validate_role_frame_contract(packet)

    def test_wrong_frame_is_rejected_on_linux(self) -> None:
        packet = self.valid_packet() | {"frame": "base_link"}

        with self.assertRaisesRegex(ReceiverContractError, "frame"):
            validate_role_frame_contract(packet)

    def test_missing_role_or_frame_is_rejected_on_linux(self) -> None:
        for field in ("role", "frame"):
            packet = self.valid_packet()
            packet.pop(field)
            with self.subTest(field=field):
                with self.assertRaisesRegex(ReceiverContractError, field):
                    validate_role_frame_contract(packet)


class Gen3ViveReceiverRoleSerialSelectionTest(unittest.TestCase):
    def valid_packet(self) -> dict[str, object]:
        return {
            "role": EXPECTED_ROLE,
            "frame": EXPECTED_FRAME,
            "serial": "LHR-UNIT-001",
            "pos": [0.1, 0.2, 0.3],
            "quat": [0.0, 0.0, 0.0, 1.0],
            "t": 1_782_287_700.0,
        }

    def test_selection_key_is_field_ordered_not_json_ordered(self) -> None:
        packet_a = self.valid_packet()
        packet_b = {
            "t": 1_782_287_700.0,
            "quat": [0.0, 0.0, 0.0, 1.0],
            "pos": [0.1, 0.2, 0.3],
            "serial": "LHR-UNIT-001",
            "frame": EXPECTED_FRAME,
            "role": EXPECTED_ROLE,
        }

        self.assertEqual(
            role_serial_selection_key(packet_a),
            role_serial_selection_key(packet_b),
        )
        self.assertEqual(
            role_serial_selection_key(packet_a),
            "role=gen3_tracker;serial=LHR-UNIT-001",
        )

    def test_role_topic_is_stable_while_serial_remains_diagnostic_key(self) -> None:
        packet = self.valid_packet()

        status = validate_role_frame_contract(packet)

        self.assertEqual(status.topic, "/vive/gen3_tracker/pose")
        self.assertEqual(status.selection_key, "role=gen3_tracker;serial=LHR-UNIT-001")

    def test_expected_serial_is_enforced_when_configured(self) -> None:
        packet = self.valid_packet()

        status = validate_role_frame_contract(
            packet, expected_serial="LHR-UNIT-001"
        )
        self.assertEqual(status.selection_key, "role=gen3_tracker;serial=LHR-UNIT-001")

        with self.assertRaisesRegex(ReceiverContractError, "serial"):
            validate_role_frame_contract(packet, expected_serial="LHR-OTHER")

    def test_missing_serial_is_rejected_only_when_serial_is_pinned(self) -> None:
        packet = self.valid_packet()
        packet.pop("serial")

        self.assertEqual(role_serial_selection_key(packet), "role=gen3_tracker")
        with self.assertRaisesRegex(ReceiverContractError, "serial"):
            validate_role_frame_contract(packet, expected_serial="LHR-UNIT-001")


if __name__ == "__main__":
    unittest.main()
