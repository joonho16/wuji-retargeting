#!/usr/bin/env python3
"""No-hardware regression tests for the MANUS UDP bridge input."""

from __future__ import annotations

import json
import socket
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "example"
for path in (ROOT, EXAMPLE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from input_devices.base import InputDeviceBase  # noqa: E402
from input_devices.manus_quantum import ManusQuantumGlove, parse_manus_payload  # noqa: E402
from scripts.manus_udp_mock import make_keypoints  # noqa: E402


def _assert(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def test_device_contract_and_safe_defaults() -> None:
    _assert(issubclass(ManusQuantumGlove, InputDeviceBase), "MANUS device should implement InputDeviceBase")
    receiver = ManusQuantumGlove(port=0, verbose=False)
    try:
        _assert(receiver.host == "127.0.0.1", "direct MANUS listener default should be loopback-only")
    finally:
        receiver.close()


def test_parse_payload_shapes_and_units() -> None:
    left_mm = make_keypoints(unit="mm")
    parsed = parse_manus_payload({"left_fingers": left_mm}, unit="mm")
    _assert(set(parsed) == {"left"}, f"unexpected sides: {parsed.keys()}")
    _assert(parsed["left"].shape == (21, 3), "left keypoints shape mismatch")
    _assert(np.isclose(parsed["left"][0, 2], 0.020), "mm -> m conversion failed")

    right_cm = make_keypoints(unit="cm")
    parsed = ManusQuantumGlove.parse_payload(
        json.dumps({"hand": "right", "keypoints": right_cm}),
        unit="cm",
        hand_side="right",
    )
    _assert(set(parsed) == {"right"}, f"unexpected right-only parse: {parsed.keys()}")
    _assert(np.isclose(parsed["right"][0, 2], 0.020), "cm -> m conversion failed")

    filtered = parse_manus_payload(
        {"hand": "right", "keypoints": right_cm},
        unit="cm",
        hand_side="left",
    )
    _assert(filtered == {}, "hand_side filter should ignore wrong side")

    try:
        parse_manus_payload({"left": [[0, 0, 0]]}, unit="m")
    except ValueError:
        pass
    else:
        raise AssertionError("invalid keypoint shape should raise ValueError")


def test_udp_receipt_and_stale_expiry() -> None:
    receiver = ManusQuantumGlove(
        host="127.0.0.1",
        port=0,
        hand_side="left",
        unit="m",
        timeout=0.15,
        verbose=False,
    )
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        payload = json.dumps({"hand": "left", "keypoints": make_keypoints(unit="m")}).encode("utf-8")
        sock.sendto(payload, ("127.0.0.1", receiver.port))

        deadline = time.monotonic() + 1.0
        fresh = None
        while time.monotonic() < deadline:
            data = receiver.get_fingers_data()
            if not np.allclose(data["left_fingers"], 0):
                fresh = data
                break
            time.sleep(0.01)
        _assert(fresh is not None, "receiver did not expose a fresh left frame")
        _assert(np.allclose(fresh["right_fingers"], 0), "right side should remain zero")

        time.sleep(0.25)
        stale = receiver.get_fingers_data()
        _assert(np.allclose(stale["left_fingers"], 0), "left frame should expire to zeros")
        _assert(np.allclose(stale["right_fingers"], 0), "right side should stay zero")
    finally:
        sock.close()
        receiver.close()


def main() -> None:
    test_device_contract_and_safe_defaults()
    print("ok: device contract and loopback default")
    test_parse_payload_shapes_and_units()
    print("ok: parse payload, units, side filtering")
    test_udp_receipt_and_stale_expiry()
    print("ok: UDP receipt and stale expiry")
    print("MANUS Quantum bridge tests passed")


if __name__ == "__main__":
    main()
