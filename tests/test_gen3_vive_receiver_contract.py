"""Contract tests for the Linux Gen3 VIVE receiver mock path."""

from __future__ import annotations

import unittest

from tests.helpers.gen3_vive_receiver_contract import (
    EXPECTED_FRAME,
    EXPECTED_ROLE,
    ReceiverContractError,
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


if __name__ == "__main__":
    unittest.main()
