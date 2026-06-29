#!/usr/bin/env python3
"""Send deterministic MANUS-style UDP JSON keypoint frames.

Use this as a stand-in for a MANUS SDK/ROS2 bridge while testing Docker ports,
tuning, or simulation:

    python scripts/manus_udp_mock.py --hand left --host 127.0.0.1 --port 15020
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from typing import Iterable


def make_keypoints(frame_index: int = 0, *, unit: str = "m") -> list[list[float]]:
    """Create a simple non-zero MediaPipe-order 21x3 hand skeleton."""
    # Base shape in meters. It is not anatomically perfect; it is intentionally
    # stable and non-zero for bridge/connectivity validation.
    scale_out = {"m": 1.0, "cm": 100.0, "mm": 1000.0}[unit]
    phase = frame_index * 0.05
    points: list[list[float]] = []
    for idx in range(21):
        finger = 0 if idx == 0 else (idx - 1) // 4
        joint = 0 if idx == 0 else (idx - 1) % 4 + 1
        x = (finger - 2) * 0.018 + 0.001 * math.sin(phase + idx)
        y = joint * 0.028
        z = 0.020 + 0.002 * math.cos(phase + idx * 0.5)
        if idx == 0:
            x, y, z = 0.001, 0.001, 0.020
        points.append([x * scale_out, y * scale_out, z * scale_out])
    return points


def build_payload(hand: str, keypoints: Iterable[Iterable[float]]) -> bytes:
    return json.dumps({"hand": hand, "keypoints": list(keypoints)}, separators=(",", ":")).encode("utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Send mock MANUS UDP JSON frames")
    parser.add_argument("--host", default="127.0.0.1", help="Destination host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=15020, help="Destination UDP port (default: 15020)")
    parser.add_argument("--hand", choices=["left", "right"], default="left", help="Hand side to send")
    parser.add_argument("--unit", choices=["m", "cm", "mm"], default="m", help="Output coordinate unit")
    parser.add_argument("--hz", type=float, default=30.0, help="Send frequency (default: 30 Hz)")
    parser.add_argument("--frames", type=int, default=0, help="Number of frames; 0 means until Ctrl+C")
    args = parser.parse_args()

    period = 1.0 / args.hz if args.hz > 0 else 0.0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sent = 0
    print(
        f"Sending mock MANUS {args.hand} frames to {args.host}:{args.port}/udp "
        f"unit={args.unit}, hz={args.hz}, frames={'∞' if args.frames == 0 else args.frames}"
    )
    try:
        while args.frames == 0 or sent < args.frames:
            sock.sendto(build_payload(args.hand, make_keypoints(sent, unit=args.unit)), (args.host, args.port))
            sent += 1
            if period:
                time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopped mock MANUS sender")
    finally:
        sock.close()
    print(f"Sent {sent} frame(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
