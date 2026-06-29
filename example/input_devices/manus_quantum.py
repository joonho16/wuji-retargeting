"""MANUS Quantum/Metaglove UDP bridge input.

This module intentionally does *not* import the proprietary MANUS SDK.  It
listens for UDP JSON frames produced by a small external bridge (MANUS C++ SDK,
MANUS ROS2 package, or any other process) and exposes the repository's standard
input-device contract::

    get_fingers_data() -> {
        "left_fingers": np.ndarray shape (21, 3),
        "right_fingers": np.ndarray shape (21, 3),
    }

Expected keypoint order is MediaPipe Hands order and coordinates are meters by
default.  Accepted payloads include::

    {"left_fingers": [[x, y, z], ...], "right_fingers": [[x, y, z], ...]}
    {"left": [[x, y, z], ...], "right": [[x, y, z], ...]}
    {"hand": "left", "keypoints": [[x, y, z], ...]}

Frames expire back to zeros after ``timeout`` seconds without a fresh update, so
teleop loops skip stale MANUS poses instead of repeatedly commanding old qpos.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time
from typing import Any, Mapping

import numpy as np

try:
    from .base import InputDeviceBase
except ImportError:  # allow `python input_devices/manus_quantum.py` from example/
    from base import InputDeviceBase

_HANDS = ("left", "right")
_FINGER_KEYS = ("left_fingers", "right_fingers")
_UNIT_SCALE = {
    "m": 1.0,
    "meter": 1.0,
    "meters": 1.0,
    "cm": 0.01,
    "centimeter": 0.01,
    "centimeters": 0.01,
    "mm": 0.001,
    "millimeter": 0.001,
    "millimeters": 0.001,
}
_EMPTY = np.zeros((21, 3), dtype=np.float32)


def _normalize_hand_side(hand_side: str | None) -> str | None:
    if hand_side is None or hand_side == "":
        return None
    side = str(hand_side).lower()
    if side not in _HANDS:
        raise ValueError(f"hand_side must be one of {_HANDS}, got {hand_side!r}")
    return side


def _unit_scale(unit: str) -> float:
    key = str(unit).lower()
    if key not in _UNIT_SCALE:
        raise ValueError(
            f"Unsupported MANUS unit {unit!r}; expected one of "
            f"{', '.join(sorted(_UNIT_SCALE))}"
        )
    return _UNIT_SCALE[key]


def _coerce_keypoints(value: Any, scale: float) -> np.ndarray:
    """Validate and normalize a MediaPipe-order 21x3 keypoint array."""
    arr = np.asarray(value, dtype=np.float32)
    if arr.shape == (63,):
        arr = arr.reshape(21, 3)
    if arr.shape != (21, 3):
        raise ValueError(f"MANUS keypoints must have shape (21, 3), got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise ValueError("MANUS keypoints contain NaN or infinite values")
    return arr * np.float32(scale)


def parse_manus_payload(
    payload: bytes | str | Mapping[str, Any],
    *,
    unit: str = "m",
    hand_side: str | None = None,
) -> dict[str, np.ndarray]:
    """Parse one MANUS bridge payload into side-keyed MediaPipe keypoints.

    Returns only sides present in the payload and allowed by ``hand_side``.
    Invalid shapes raise ``ValueError`` so callers/tests can catch bad bridge
    output early.
    """
    if isinstance(payload, (bytes, bytearray)):
        data = json.loads(payload.decode("utf-8"))
    elif isinstance(payload, str):
        data = json.loads(payload)
    elif isinstance(payload, Mapping):
        data = dict(payload)
    else:
        raise TypeError(f"Unsupported MANUS payload type: {type(payload).__name__}")

    wanted_side = _normalize_hand_side(hand_side)
    scale = _unit_scale(unit)
    frames: dict[str, np.ndarray] = {}

    def add(side: str, value: Any) -> None:
        if value is None:
            return
        if wanted_side is not None and side != wanted_side:
            return
        frames[side] = _coerce_keypoints(value, scale)

    # Explicit repository contract names.
    add("left", data.get("left_fingers"))
    add("right", data.get("right_fingers"))

    # Short aliases commonly used by small bridge scripts.
    add("left", data.get("left"))
    add("right", data.get("right"))

    # Single-hand message: {"hand":"left", "keypoints":[...]}.
    hand = data.get("hand") or data.get("side")
    if hand is not None:
        side = str(hand).lower()
        if side not in _HANDS:
            raise ValueError(f"MANUS hand must be 'left' or 'right', got {hand!r}")
        keypoints = data.get("keypoints", data.get("fingers", data.get("landmarks")))
        add(side, keypoints)

    # Optional nested form for bridge authors that naturally group hands.
    hands = data.get("hands")
    if isinstance(hands, Mapping):
        for side in _HANDS:
            item = hands.get(side)
            if isinstance(item, Mapping):
                item = item.get("keypoints", item.get("fingers", item.get("landmarks")))
            add(side, item)

    return frames


class ManusQuantumGlove(InputDeviceBase):
    """UDP receiver for MANUS bridge frames."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 15020,
        hand_side: str | None = None,
        unit: str = "m",
        timeout: float = 0.5,
        buffer_size: int = 65535,
        verbose: bool = True,
    ):
        self.host = host
        self.port = int(port)
        self.hand_side = _normalize_hand_side(hand_side)
        self.unit = unit
        self.timeout = float(timeout)
        if self.timeout <= 0:
            raise ValueError("timeout must be > 0")
        self.buffer_size = int(buffer_size)
        self.verbose = verbose

        self._frames = {side: _EMPTY.copy() for side in _HANDS}
        self._last_seen = {side: 0.0 for side in _HANDS}
        self._last_error: str | None = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(0.1)
        self.port = int(self._sock.getsockname()[1])

        self._thread = threading.Thread(target=self._recv_loop, name="manus-udp-recv", daemon=True)
        self._thread.start()

        if self.verbose:
            side_msg = self.hand_side or "left/right"
            print(
                "MANUS UDP bridge initialized\n"
                f"  Listen: {self.host}:{self.port}/udp\n"
                f"  Hand side: {side_msg}\n"
                f"  Unit: {self.unit}, stale timeout: {self.timeout:.3f}s"
            )

    @property
    def last_error(self) -> str | None:
        return self._last_error

    @staticmethod
    def parse_payload(
        payload: bytes | str | Mapping[str, Any],
        *,
        unit: str = "m",
        hand_side: str | None = None,
    ) -> dict[str, np.ndarray]:
        return parse_manus_payload(payload, unit=unit, hand_side=hand_side)

    def _recv_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                packet, _addr = self._sock.recvfrom(self.buffer_size)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                frames = parse_manus_payload(packet, unit=self.unit, hand_side=self.hand_side)
            except Exception as exc:  # keep listener alive on malformed bridge frames
                self._last_error = f"{type(exc).__name__}: {exc}"
                if self.verbose:
                    print(f"Ignoring invalid MANUS frame: {self._last_error}", file=sys.stderr)
                continue

            if not frames:
                continue

            now = time.monotonic()
            with self._lock:
                for side, keypoints in frames.items():
                    self._frames[side] = keypoints.astype(np.float32, copy=True)
                    self._last_seen[side] = now
                self._last_error = None

    def _fresh_or_empty(self, side: str, now: float) -> np.ndarray:
        last_seen = self._last_seen[side]
        if last_seen <= 0 or now - last_seen > self.timeout:
            return _EMPTY.copy()
        return self._frames[side].copy()

    def get_fingers_data(self) -> dict[str, np.ndarray]:
        """Return fresh left/right keypoints; stale or missing sides are zeros."""
        now = time.monotonic()
        with self._lock:
            left = self._fresh_or_empty("left", now)
            right = self._fresh_or_empty("right", now)
        return {"left_fingers": left, "right_fingers": right}

    def cleanup(self) -> None:
        self.close()

    def stop(self) -> None:
        self.close()

    def close(self) -> None:
        if self._stop_event.is_set():
            return
        self._stop_event.set()
        try:
            self._sock.close()
        except OSError:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=0.5)


# Backward-friendly alias for bridge scripts that prefer the device-style name.
ManusQuantum = ManusQuantumGlove


def _main() -> int:
    parser = argparse.ArgumentParser(description="Listen for MANUS UDP JSON keypoint frames")
    parser.add_argument("--host", default="127.0.0.1", help="UDP bind host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=15020, help="UDP bind port (default: 15020)")
    parser.add_argument("--hand", choices=["left", "right"], default=None,
                        help="Only accept frames for this side")
    parser.add_argument("--unit", default="m", choices=["m", "cm", "mm"],
                        help="Incoming coordinate unit (default: m)")
    parser.add_argument("--timeout", type=float, default=0.5,
                        help="Seconds before a side expires back to zeros (default: 0.5)")
    parser.add_argument("--once", action="store_true",
                        help="Exit after receiving one fresh frame for --hand or any side")
    parser.add_argument("--max-wait", type=float, default=10.0,
                        help="Maximum seconds to wait with --once (default: 10)")
    args = parser.parse_args()

    device = ManusQuantumGlove(
        host=args.host,
        port=args.port,
        hand_side=args.hand,
        unit=args.unit,
        timeout=args.timeout,
    )
    try:
        if args.once:
            deadline = time.monotonic() + args.max_wait
            wanted_keys = [f"{args.hand}_fingers"] if args.hand else list(_FINGER_KEYS)
            while time.monotonic() < deadline:
                data = device.get_fingers_data()
                for key in wanted_keys:
                    frame = data[key]
                    if frame is not None and not np.allclose(frame, 0):
                        print(f"received fresh MANUS frame: {key}, wrist={frame[0].tolist()}")
                        return 0
                time.sleep(0.02)
            print("timed out waiting for a fresh MANUS frame", file=sys.stderr)
            return 2

        print("Press Ctrl+C to stop.")
        while True:
            data = device.get_fingers_data()
            fresh = [key for key, frame in data.items() if not np.allclose(frame, 0)]
            if fresh:
                print(f"fresh sides: {', '.join(fresh)}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping MANUS UDP bridge listener...")
        return 0
    finally:
        device.close()


if __name__ == "__main__":
    raise SystemExit(_main())
