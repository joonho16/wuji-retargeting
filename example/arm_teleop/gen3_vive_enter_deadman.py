#!/usr/bin/env python3
"""Interactive keyboard deadman publisher for Gen3 VIVE bring-up.

This is a bring-up/testing helper, not a replacement for a real physical
E-stop/deadman.  It publishes std_msgs/msg/Bool on the configured deadman topic:

* default mode: press Enter to toggle false/true,
* type q/quit/exit then Enter to publish false and exit,
* Ctrl-C/EOF publishes false before shutdown.
"""

from __future__ import annotations

import argparse
import select
import sys
import time
from collections.abc import Iterable
from dataclasses import dataclass

DEFAULT_DEADMAN_TOPIC = "/gen3_vive/physical_deadman"
DEFAULT_RATE_HZ = 20.0


@dataclass(frozen=True)
class EnterDeadmanConfig:
    topic: str = DEFAULT_DEADMAN_TOPIC
    rate_hz: float = DEFAULT_RATE_HZ
    initial_pressed: bool = False
    true_on_first_enter: bool = True

    def validate(self) -> None:
        if not isinstance(self.topic, str) or not self.topic.strip():
            raise ValueError("topic must be non-empty")
        if self.rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Publish /gen3_vive/physical_deadman interactively. "
            "Enter toggles Bool true/false; q exits false."
        )
    )
    parser.add_argument("--topic", default=DEFAULT_DEADMAN_TOPIC)
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument(
        "--start-true",
        action="store_true",
        help="start by publishing true immediately; default starts false",
    )
    parser.add_argument(
        "--first-enter-false",
        action="store_true",
        help="if --start-true is used, make the first Enter toggle to false",
    )
    return parser


def config_from_args(args: argparse.Namespace) -> EnterDeadmanConfig:
    return EnterDeadmanConfig(
        topic=args.topic,
        rate_hz=float(args.rate_hz),
        initial_pressed=bool(args.start_true),
        true_on_first_enter=not bool(args.first_enter_false),
    )


def _stdin_line_ready(timeout_s: float) -> bool:
    readable, _, _ = select.select([sys.stdin], [], [], timeout_s)
    return bool(readable)


def _print_state(pressed: bool, topic: str) -> None:
    state = "TRUE / ENABLED" if pressed else "false / stopped"
    print(f"[enter-deadman] {topic} = {state}", flush=True)


def _publish_false_burst(pub: object, msg_type: type, *, count: int = 5, delay_s: float = 0.02) -> None:
    msg = msg_type()
    msg.data = False
    for _ in range(count):
        pub.publish(msg)
        time.sleep(delay_s)


def run(config: EnterDeadmanConfig) -> int:
    config.validate()

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool

    rclpy.init(args=None)

    class EnterDeadmanNode(Node):
        def __init__(self) -> None:
            super().__init__("gen3_vive_enter_deadman")
            self.pub = self.create_publisher(Bool, config.topic, 10)

    node = EnterDeadmanNode()
    msg = Bool()
    pressed = bool(config.initial_pressed)
    first_enter = True
    period_s = 1.0 / float(config.rate_hz)

    print(
        "[enter-deadman] Bring-up helper only; keep real E-stop reachable.\n"
        "[enter-deadman] Press Enter to toggle deadman. Type q + Enter to exit false.\n"
        f"[enter-deadman] Publishing std_msgs/Bool on {config.topic} at {config.rate_hz:g} Hz.",
        flush=True,
    )
    _print_state(pressed, config.topic)

    try:
        while rclpy.ok():
            msg.data = pressed
            node.pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)

            if _stdin_line_ready(period_s):
                line = sys.stdin.readline()
                if line == "":  # EOF: fail closed.
                    break
                token = line.strip().lower()
                if token in {"q", "quit", "exit", "off", "false", "0"}:
                    pressed = False
                    _print_state(pressed, config.topic)
                    break
                if token in {"on", "true", "1"}:
                    pressed = True
                elif token == "":
                    if first_enter and config.true_on_first_enter:
                        pressed = True
                    else:
                        pressed = not pressed
                    first_enter = False
                else:
                    print(
                        "[enter-deadman] Unknown input. Use Enter toggle, on, off, or q.",
                        flush=True,
                    )
                    continue
                _print_state(pressed, config.topic)
    except KeyboardInterrupt:
        print("\n[enter-deadman] Ctrl-C: publishing false before exit.", flush=True)
    finally:
        try:
            _publish_false_burst(node.pub, Bool)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    return 0


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run(config_from_args(args))
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
