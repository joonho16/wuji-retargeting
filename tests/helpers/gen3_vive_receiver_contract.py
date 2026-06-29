"""Test helpers for the Linux Gen3 VIVE receiver contract.

These helpers intentionally live under ``tests`` so worker-owned receiver tests and
mock packets can validate the externally managed sender contract without editing
sender code or taking ownership of the production receiver implementation.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

EXPECTED_ROLE = "gen3_tracker"
EXPECTED_FRAME = "steamvr_world"


class ReceiverContractError(ValueError):
    """Raised when a mock packet violates the Linux receiver input contract."""


@dataclass(frozen=True)
class ReceiverContractStatus:
    """Validated Linux receiver contract state for one mock sender packet."""

    role: str
    frame: str
    topic: str
    selection_key: str


def _required_text(packet: Mapping[str, Any], field: str) -> str:
    value = packet.get(field)
    if not isinstance(value, str) or not value.strip():
        raise ReceiverContractError(f"missing or invalid {field!r}")
    return value.strip()


def _optional_text(packet: Mapping[str, Any], field: str) -> str | None:
    value = packet.get(field)
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise ReceiverContractError(f"invalid {field!r}")
    return value.strip()


def role_serial_selection_key(
    packet: Mapping[str, Any],
    *,
    expected_serial: str | None = None,
) -> str:
    """Return a deterministic diagnostic selection key for a tracker packet.

    The key is intentionally field-name ordered (``role`` before ``serial``) so
    it cannot depend on JSON object order or Python dict insertion order.
    ``serial`` is optional unless Linux configuration pins an expected tracker
    serial, in which case a missing or mismatched serial is a refusal condition.
    """

    role = _required_text(packet, "role")
    serial = _optional_text(packet, "serial")

    if expected_serial is not None:
        expected_serial = expected_serial.strip()
        if not expected_serial:
            raise ReceiverContractError("invalid expected serial")
        if serial is None:
            raise ReceiverContractError("missing required 'serial'")
        if serial != expected_serial:
            raise ReceiverContractError(
                f"serial {serial!r} != expected serial {expected_serial!r}"
            )

    parts = [f"role={role}"]
    if serial is not None:
        parts.append(f"serial={serial}")
    return ";".join(parts)


def validate_role_frame_contract(
    packet: Mapping[str, Any],
    *,
    expected_role: str = EXPECTED_ROLE,
    expected_frame: str = EXPECTED_FRAME,
    expected_serial: str | None = None,
) -> ReceiverContractStatus:
    """Validate Task-2 role/frame constraints for a mock Linux receiver packet.

    The Linux side treats ``role=gen3_tracker`` and ``frame=steamvr_world`` as a
    strict upstream contract. Any mismatch must be refused locally instead of
    being auto-corrected or pushed back into sender-side code/configuration.
    """

    role = _required_text(packet, "role")
    frame = _required_text(packet, "frame")

    errors: list[str] = []
    if role != expected_role:
        errors.append(f"role {role!r} != {expected_role!r}")
    if frame != expected_frame:
        errors.append(f"frame {frame!r} != {expected_frame!r}")
    if errors:
        raise ReceiverContractError("; ".join(errors))

    return ReceiverContractStatus(
        role=role,
        frame=frame,
        topic=f"/vive/{role}/pose",
        selection_key=role_serial_selection_key(
            packet, expected_serial=expected_serial
        ),
    )
