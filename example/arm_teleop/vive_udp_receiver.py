#!/usr/bin/env python3
"""Linux-side VIVE UDP JSON receiver/parser for Gen3 arm teleop.

The Windows/OpenVR sender is externally managed and out of scope for this repo.
This module verifies the sender contract on the Linux side only:

* role must be ``gen3_tracker``
* frame must be ``steamvr_world``
* ``pos`` must be three finite numbers
* ``quat`` must be four finite numbers
* optional sender timestamp ``t`` is preserved in diagnostics

The parser is dependency-free so it can be unit/smoke tested without ROS2.  When
ROS2 is available, run this file as a node to publish only validated poses to
``/vive/gen3_tracker/pose`` plus a JSON diagnostics string on
``/vive/gen3_tracker/status``.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, cast

EXPECTED_ROLE = "gen3_tracker"
EXPECTED_FRAME = "steamvr_world"
DEFAULT_POSE_TOPIC = f"/vive/{EXPECTED_ROLE}/pose"
DEFAULT_STATUS_TOPIC = f"/vive/{EXPECTED_ROLE}/status"
QUATERNION_NORM_TOLERANCE = 1.0e-3


class VivePacketError(ValueError):
    """Raised when an incoming sender packet violates the Linux contract."""

    def __init__(self, reason: str, message: str):
        super().__init__(message)
        self.reason = reason


@dataclass(frozen=True)
class VivePosePacket:
    """Validated sender packet ready for ROS2 publishing or mapper input."""

    role: str
    frame: str
    serial: str | None
    position_xyz: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    sender_timestamp_s: float | None
    received_monotonic_s: float
    selected_key: str
    pose_topic: str
    source_address: str | None = None


@dataclass
class ViveReceiverDiagnostics:
    """Receiver counters and last-packet metadata for logs/status publication."""

    total_packets: int = 0
    accepted_packets: int = 0
    invalid_packets: int = 0
    invalid_json_packets: int = 0
    missing_field_packets: int = 0
    non_finite_packets: int = 0
    non_unit_quaternion_packets: int = 0
    role_mismatch_packets: int = 0
    frame_mismatch_packets: int = 0
    serial_mismatch_packets: int = 0
    invalid_timestamp_packets: int = 0
    last_receive_monotonic_s: float | None = None
    last_sender_timestamp_s: float | None = None
    last_selected_key: str | None = None
    last_error_reason: str | None = None
    last_error: str | None = None

    def record_accept(self, packet: VivePosePacket) -> None:
        self.total_packets += 1
        self.accepted_packets += 1
        self.last_receive_monotonic_s = packet.received_monotonic_s
        self.last_sender_timestamp_s = packet.sender_timestamp_s
        self.last_selected_key = packet.selected_key
        self.last_error_reason = None
        self.last_error = None

    def record_reject(
        self,
        reason: str,
        message: str,
        received_monotonic_s: float | None = None,
    ) -> None:
        self.total_packets += 1
        self.invalid_packets += 1
        self.last_receive_monotonic_s = received_monotonic_s
        self.last_error_reason = reason
        self.last_error = message
        if reason == "invalid_json":
            self.invalid_json_packets += 1
        elif reason == "missing_field":
            self.missing_field_packets += 1
        elif reason == "non_finite":
            self.non_finite_packets += 1
        elif reason == "non_unit_quaternion":
            self.non_unit_quaternion_packets += 1
        elif reason == "role_mismatch":
            self.role_mismatch_packets += 1
        elif reason == "frame_mismatch":
            self.frame_mismatch_packets += 1
        elif reason == "serial_mismatch":
            self.serial_mismatch_packets += 1
        elif reason == "invalid_timestamp":
            self.invalid_timestamp_packets += 1

    def as_dict(self) -> dict[str, Any]:
        return {
            "total_packets": self.total_packets,
            "accepted_packets": self.accepted_packets,
            "invalid_packets": self.invalid_packets,
            "invalid_json_packets": self.invalid_json_packets,
            "missing_field_packets": self.missing_field_packets,
            "non_finite_packets": self.non_finite_packets,
            "non_unit_quaternion_packets": self.non_unit_quaternion_packets,
            "role_mismatch_packets": self.role_mismatch_packets,
            "frame_mismatch_packets": self.frame_mismatch_packets,
            "serial_mismatch_packets": self.serial_mismatch_packets,
            "invalid_timestamp_packets": self.invalid_timestamp_packets,
            "last_receive_monotonic_s": self.last_receive_monotonic_s,
            "last_sender_timestamp_s": self.last_sender_timestamp_s,
            "last_selected_key": self.last_selected_key,
            "last_error_reason": self.last_error_reason,
            "last_error": self.last_error,
        }


@dataclass
class VivePacketParser:
    """Validate VIVE UDP JSON packets against the Linux receiver contract."""

    expected_serial: str | None = None
    quaternion_norm_tolerance: float = QUATERNION_NORM_TOLERANCE
    diagnostics: ViveReceiverDiagnostics = field(default_factory=ViveReceiverDiagnostics)

    def parse_datagram(
        self,
        datagram: bytes | str,
        *,
        received_monotonic_s: float | None = None,
        source_address: str | None = None,
    ) -> VivePosePacket:
        """Parse one UDP datagram and update diagnostics.

        Invalid packets raise :class:`VivePacketError` and are counted; valid
        packets return a deterministic role/serial key and fixed pose topic.
        """

        recv_time = time.monotonic() if received_monotonic_s is None else received_monotonic_s
        try:
            packet = self._parse_datagram(datagram, recv_time, source_address)
        except VivePacketError as exc:
            self.diagnostics.record_reject(exc.reason, str(exc), recv_time)
            raise
        self.diagnostics.record_accept(packet)
        return packet

    def _parse_datagram(
        self,
        datagram: bytes | str,
        received_monotonic_s: float,
        source_address: str | None,
    ) -> VivePosePacket:
        if isinstance(datagram, bytes):
            try:
                text = datagram.decode("utf-8")
            except UnicodeDecodeError as exc:
                raise VivePacketError("invalid_json", f"packet is not UTF-8 JSON: {exc}") from exc
        else:
            text = datagram

        try:
            raw = json.loads(text)
        except json.JSONDecodeError as exc:
            raise VivePacketError("invalid_json", f"packet is not valid JSON: {exc}") from exc
        if not isinstance(raw, Mapping):
            raise VivePacketError("invalid_json", "packet JSON root must be an object")

        role = _required_string(raw, "role")
        if role != EXPECTED_ROLE:
            raise VivePacketError(
                "role_mismatch",
                f"expected role={EXPECTED_ROLE!r}, got {role!r}",
            )

        frame = _required_string(raw, "frame")
        if frame != EXPECTED_FRAME:
            raise VivePacketError(
                "frame_mismatch",
                f"expected frame={EXPECTED_FRAME!r}, got {frame!r}",
            )

        serial = _optional_string(raw, "serial")
        if self.expected_serial is not None and serial != self.expected_serial:
            raise VivePacketError(
                "serial_mismatch",
                f"expected serial={self.expected_serial!r}, got {serial!r}",
            )

        position_xyz = cast(
            tuple[float, float, float],
            _finite_float_tuple(raw, "pos", 3),
        )
        quaternion_xyzw = cast(
            tuple[float, float, float, float],
            _finite_float_tuple(raw, "quat", 4),
        )
        _validate_unit_quaternion(
            quaternion_xyzw,
            tolerance=self.quaternion_norm_tolerance,
        )
        sender_timestamp_s = _optional_finite_float(raw, "t")

        selected_key = deterministic_selected_key(role=role, serial=serial)
        return VivePosePacket(
            role=role,
            frame=frame,
            serial=serial,
            position_xyz=position_xyz,
            quaternion_xyzw=quaternion_xyzw,
            sender_timestamp_s=sender_timestamp_s,
            received_monotonic_s=received_monotonic_s,
            selected_key=selected_key,
            pose_topic=DEFAULT_POSE_TOPIC,
            source_address=source_address,
        )


def deterministic_selected_key(role: str, serial: str | None) -> str:
    """Return the stable role/serial key used in diagnostics and filtering."""

    parts = [f"role={role}"]
    if serial is not None:
        parts.append(f"serial={serial}")
    return ";".join(parts)


def _required_string(raw: Mapping[str, Any], field_name: str) -> str:
    value = raw.get(field_name)
    if not isinstance(value, str) or not value:
        raise VivePacketError("missing_field", f"field {field_name!r} must be a non-empty string")
    return value


def _optional_string(raw: Mapping[str, Any], field_name: str) -> str | None:
    value = raw.get(field_name)
    if value is None:
        return None
    if not isinstance(value, str) or not value:
        raise VivePacketError(
            "missing_field",
            f"field {field_name!r} must be a non-empty string when present",
        )
    return value


def _finite_float_tuple(
    raw: Mapping[str, Any],
    field_name: str,
    expected_len: int,
) -> tuple[float, ...]:
    if field_name not in raw:
        raise VivePacketError("missing_field", f"missing required field {field_name!r}")
    value = raw[field_name]
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise VivePacketError(
            "missing_field",
            f"field {field_name!r} must be a sequence of {expected_len} numbers",
        )
    if len(value) != expected_len:
        raise VivePacketError(
            "missing_field",
            f"field {field_name!r} must have length {expected_len}",
        )
    floats = tuple(_finite_float(item, field_name) for item in value)
    return floats


def _validate_unit_quaternion(
    quaternion_xyzw: tuple[float, float, float, float],
    *,
    tolerance: float,
) -> None:
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise VivePacketError("non_unit_quaternion", "quaternion tolerance must be finite")
    norm = math.sqrt(sum(component * component for component in quaternion_xyzw))
    if abs(norm - 1.0) > tolerance:
        raise VivePacketError(
            "non_unit_quaternion",
            f"field 'quat' must be unit length; norm={norm:.6f}",
        )


def _optional_finite_float(raw: Mapping[str, Any], field_name: str) -> float | None:
    if field_name not in raw or raw[field_name] is None:
        return None
    try:
        return _finite_float(raw[field_name], field_name)
    except VivePacketError as exc:
        # Sender timestamp diagnostics should not make a geometrically valid
        # pose publishable as stale/invalid data, but we preserve the failure in
        # counters and continue with a None sender timestamp.
        if exc.reason == "non_finite":
            raise VivePacketError("invalid_timestamp", str(exc)) from exc
        raise


def _finite_float(value: Any, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise VivePacketError("non_finite", f"field {field_name!r} must contain finite numbers")
    float_value = float(value)
    if not math.isfinite(float_value):
        raise VivePacketError("non_finite", f"field {field_name!r} contains a non-finite value")
    return float_value


def status_json(diagnostics: ViveReceiverDiagnostics) -> str:
    """Serialize diagnostics for logs or a ROS2 companion status topic."""

    return json.dumps(diagnostics.as_dict(), sort_keys=True, separators=(",", ":"))


class Ros2ViveUdpReceiver:
    """Optional ROS2 bridge; imports ROS2 packages only when instantiated."""

    def __init__(
        self,
        *,
        bind_host: str,
        bind_port: int,
        expected_serial: str | None,
        status_topic: str,
        poll_period_s: float,
    ):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from std_msgs.msg import String

        class _ReceiverNode(Node):
            def __init__(self) -> None:
                super().__init__("vive_udp_json_to_ros2")
                self._pose_type = PoseStamped
                self._status_type = String
                self._parser = VivePacketParser(
                    expected_serial=expected_serial,
                )
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._sock.bind((bind_host, bind_port))
                self._sock.setblocking(False)
                self._pose_pub = self.create_publisher(PoseStamped, DEFAULT_POSE_TOPIC, 10)
                self._status_pub = self.create_publisher(String, status_topic, 10)
                self._timer = self.create_timer(poll_period_s, self._poll)
                self.get_logger().info(
                    "Listening for VIVE UDP JSON on "
                    f"{bind_host}:{bind_port}; publishing validated poses to {DEFAULT_POSE_TOPIC}"
                )

            def _poll(self) -> None:
                while True:
                    try:
                        data, addr = self._sock.recvfrom(65535)
                    except BlockingIOError:
                        break
                    source = f"{addr[0]}:{addr[1]}"
                    try:
                        packet = self._parser.parse_datagram(data, source_address=source)
                    except VivePacketError as exc:
                        self.get_logger().warn(f"Rejected VIVE packet from {source}: {exc}")
                        self._publish_status()
                        continue
                    self._publish_pose(packet)
                    self._publish_status()

            def _publish_pose(self, packet: VivePosePacket) -> None:
                msg = self._pose_type()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = packet.frame
                msg.pose.position.x = packet.position_xyz[0]
                msg.pose.position.y = packet.position_xyz[1]
                msg.pose.position.z = packet.position_xyz[2]
                msg.pose.orientation.x = packet.quaternion_xyzw[0]
                msg.pose.orientation.y = packet.quaternion_xyzw[1]
                msg.pose.orientation.z = packet.quaternion_xyzw[2]
                msg.pose.orientation.w = packet.quaternion_xyzw[3]
                self._pose_pub.publish(msg)

            def _publish_status(self) -> None:
                msg = self._status_type()
                msg.data = status_json(self._parser.diagnostics)
                self._status_pub.publish(msg)

            def destroy_node(self) -> bool:
                self._sock.close()
                return super().destroy_node()

        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self._node = _ReceiverNode()

    def spin(self) -> None:
        try:
            self._rclpy.spin(self._node)
        finally:
            self._node.destroy_node()
            self._rclpy.shutdown()


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate external VIVE UDP JSON and publish Linux-side ROS2 PoseStamped.",
    )
    parser.add_argument("--bind-host", default="0.0.0.0", help="UDP bind host")
    parser.add_argument("--bind-port", type=int, default=39539, help="UDP bind port")
    parser.add_argument("--expected-serial", default=None)
    parser.add_argument("--status-topic", default=DEFAULT_STATUS_TOPIC)
    parser.add_argument("--poll-period-s", type=float, default=0.002)
    parser.add_argument(
        "--check-packet",
        help="Validate one JSON packet and print parser diagnostics instead of starting ROS2.",
    )
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    if args.check_packet is not None:
        parser = VivePacketParser(
            expected_serial=args.expected_serial,
        )
        try:
            packet = parser.parse_datagram(args.check_packet)
        except VivePacketError as exc:
            print(f"REJECTED: {exc.reason}: {exc}")
            print(status_json(parser.diagnostics))
            return 2
        print(f"ACCEPTED: {packet.selected_key} -> {packet.pose_topic}")
        print(status_json(parser.diagnostics))
        return 0

    receiver = Ros2ViveUdpReceiver(
        bind_host=args.bind_host,
        bind_port=args.bind_port,
        expected_serial=args.expected_serial,
        status_topic=args.status_topic,
        poll_period_s=args.poll_period_s,
    )
    receiver.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
