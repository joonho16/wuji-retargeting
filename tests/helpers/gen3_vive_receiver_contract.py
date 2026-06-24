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


def _required_text(packet: Mapping[str, Any], field: str) -> str:
    value = packet.get(field)
    if not isinstance(value, str) or not value.strip():
        raise ReceiverContractError(f"missing or invalid {field!r}")
    return value.strip()


def validate_role_frame_contract(
    packet: Mapping[str, Any],
    *,
    expected_role: str = EXPECTED_ROLE,
    expected_frame: str = EXPECTED_FRAME,
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
    )
