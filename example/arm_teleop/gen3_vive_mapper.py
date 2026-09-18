#!/usr/bin/env python3
"""Dependency-light Gen3 VIVE tracker to MoveIt Servo mapper core.

This module is intentionally ROS-optional and sender-agnostic.  It consumes the
Linux-side receiver contract from :mod:`example.arm_teleop.vive_udp_receiver`
(``role=gen3_tracker``, ``frame=steamvr_world``) and turns a fresh, valid,
relative tracker pose into a bounded command object that a ROS2 wrapper can
publish as a MoveIt Servo ``geometry_msgs/msg/TwistStamped``.

Safety boundaries implemented here:

* calibration contract validation before command enable,
* relative neutral capture on deadman rising edge,
* stale/invalid/deadman stop with neutral recapture required for recovery,
* fresh pose + new deadman rising edge required after every stop,
* finite pose/quaternion, role, and frame refusal,
* target-tool-pose following when wrappers provide current TF feedback,
* workspace/velocity clamp plus optional deadband and low-pass filtering,
* direct Kortex twist backend quarantined behind explicit diagnostic opt-in.

The Windows/OpenVR/VIVE sender and Wuji Hand input-device/teleop entrypoints are
out of scope for this module.
"""

from __future__ import annotations

import math
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Literal, cast

from example.arm_teleop.vive_udp_receiver import EXPECTED_FRAME, EXPECTED_ROLE

Vector3 = tuple[float, float, float]
QuaternionXYZW = tuple[float, float, float, float]
BackendName = Literal["moveit_servo_twist", "diagnostic_kortex_twist"]

DEFAULT_BACKEND: BackendName = "moveit_servo_twist"
DIAGNOSTIC_KORTEX_BACKEND: BackendName = "diagnostic_kortex_twist"
DEFAULT_COMMAND_TYPE = "geometry_msgs/msg/TwistStamped"
DIAGNOSTIC_KORTEX_COMMAND_TYPE = "diagnostic/kortex_twist"


class MapperError(ValueError):
    """Raised when mapper configuration or input violates a safety contract."""

    def __init__(self, reason: str, message: str):
        super().__init__(message)
        self.reason = reason


class StopReason(str, Enum):
    """Stable stop/diagnostic reasons emitted by the mapper core."""

    UNCALIBRATED = "UNCALIBRATED_STOP"
    FRAME_MISMATCH = "FRAME_MISMATCH_STOP"
    ROLE_MISMATCH = "ROLE_MISMATCH_STOP"
    INVALID_POSE = "INVALID_POSE_STOP"
    STALE = "STALE_STOP"
    NO_FRESH_POSE = "NO_FRESH_POSE_STOP"
    DEADMAN_RELEASED = "DEADMAN_RELEASED_STOP"
    WAITING_FOR_NEUTRAL_RECAPTURE = "WAITING_FOR_NEUTRAL_RECAPTURE"
    REAL_ROBOT_LATCH = "REAL_ROBOT_LATCH_STOP"
    SYNTHETIC_DEADMAN_REJECTED = "SYNTHETIC_DEADMAN_REJECTED_STOP"
    SERVO_NOT_READY = "SERVO_NOT_READY_STOP"
    NEUTRAL_CAPTURED = "NEUTRAL_CAPTURED"
    TOOL_POSE_UNAVAILABLE = "TOOL_POSE_UNAVAILABLE_STOP"


@dataclass(frozen=True)
class CalibrationTransform:
    """Rigid transform entry from the calibration artifact."""

    translation: Vector3
    rotation_xyzw: QuaternionXYZW

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any], *, field_name: str) -> "CalibrationTransform":
        return cls(
            translation=_finite_float_tuple(raw, "translation", 3, parent=field_name),
            rotation_xyzw=_unit_quaternion_tuple(raw, "rotation_xyzw", parent=field_name),
        )


@dataclass(frozen=True)
class WorkspaceLimits:
    """Axis-aligned planning-frame bounds for tracker-driven tool offsets.

    The limits are relative to the neutral tool pose captured on the deadman
    rising edge, not absolute base-link coordinates.  A zero tracker delta must
    therefore command zero tool motion even if the robot's starting TCP pose is
    outside this relative offset box.
    """

    x: tuple[float, float]
    y: tuple[float, float]
    z: tuple[float, float]

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "WorkspaceLimits":
        return cls(
            x=_limit_pair(raw, "x"),
            y=_limit_pair(raw, "y"),
            z=_limit_pair(raw, "z"),
        )

    def clamp_position(self, position_xyz: Vector3) -> Vector3:
        return (
            _clamp(position_xyz[0], self.x[0], self.x[1]),
            _clamp(position_xyz[1], self.y[0], self.y[1]),
            _clamp(position_xyz[2], self.z[0], self.z[1]),
        )


@dataclass(frozen=True)
class Gen3ViveCalibration:
    """Validated calibration artifact required before mapper command output.

    Field names intentionally mirror the approved plan's YAML contract so a ROS2
    wrapper can load the same mapping without extra translation.
    """

    source_frame: str
    planning_frame: str
    tracker_role: str
    tracker_mount_frame: str
    T_planning_source: CalibrationTransform
    # NativeSession consumes ^tool T_control; the legacy Servo mapper ignores it.
    T_tool_tracker_mount: CalibrationTransform
    linear_scale: float
    angular_scale: float
    workspace_limits: WorkspaceLimits | None
    max_linear_mps: float
    max_angular_radps: float
    stale_timeout_s: float
    linear_deadband_m: float = 0.0
    angular_deadband_rad: float = 0.0

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "Gen3ViveCalibration":
        required_transform_names = ("T_planning_source", "T_tool_tracker_mount")
        missing = [name for name in required_transform_names if name not in raw]
        if missing:
            raise MapperError("missing_calibration_field", f"missing calibration fields: {missing}")
        workspace_limits = None
        if raw.get("workspace_limits") is not None:
            workspace_limits = WorkspaceLimits.from_mapping(_required_mapping(raw, "workspace_limits"))
        calibration = cls(
            source_frame=_required_text(raw, "source_frame"),
            planning_frame=_required_text(raw, "planning_frame"),
            tracker_role=_required_text(raw, "tracker_role"),
            tracker_mount_frame=_required_text(raw, "tracker_mount_frame"),
            T_planning_source=CalibrationTransform.from_mapping(
                _required_mapping(raw, "T_planning_source"),
                field_name="T_planning_source",
            ),
            T_tool_tracker_mount=CalibrationTransform.from_mapping(
                _required_mapping(raw, "T_tool_tracker_mount"),
                field_name="T_tool_tracker_mount",
            ),
            linear_scale=_positive_finite_float(raw, "linear_scale"),
            angular_scale=_positive_finite_float(raw, "angular_scale"),
            workspace_limits=workspace_limits,
            max_linear_mps=_positive_finite_float(raw, "max_linear_mps"),
            max_angular_radps=_positive_finite_float(raw, "max_angular_radps"),
            stale_timeout_s=_positive_finite_float(raw, "stale_timeout_s"),
            linear_deadband_m=_nonnegative_finite_float(
                raw,
                "linear_deadband_m",
                default=0.0,
            ),
            angular_deadband_rad=_nonnegative_finite_float(
                raw,
                "angular_deadband_rad",
                default=0.0,
            ),
        )
        return calibration

    @classmethod
    def from_yaml_file(cls, path: str | Path) -> "Gen3ViveCalibration":
        """Load the calibration YAML with PyYAML only at this optional boundary."""

        try:
            import yaml
        except ModuleNotFoundError as exc:  # pragma: no cover - depends on env.
            raise MapperError(
                "missing_yaml_dependency",
                "PyYAML is required to load calibration YAML files",
            ) from exc
        with Path(path).open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
        if not isinstance(raw, Mapping):
            raise MapperError("invalid_calibration", "calibration YAML root must be a mapping")
        return cls.from_mapping(raw)


@dataclass(frozen=True)
class ViveTrackerPose:
    """Dependency-free validated pose input for the mapper core."""

    role: str
    frame: str
    position_xyz: Vector3
    quaternion_xyzw: QuaternionXYZW
    received_monotonic_s: float
    serial: str | None = None

    @classmethod
    def from_receiver_packet(cls, packet: Any) -> "ViveTrackerPose":
        """Convert a ``VivePosePacket``-like object without importing ROS2."""

        return cls(
            role=str(packet.role),
            frame=str(packet.frame),
            serial=getattr(packet, "serial", None),
            position_xyz=_coerce_vector3(packet.position_xyz, "position_xyz"),
            quaternion_xyzw=_coerce_unit_quaternion(packet.quaternion_xyzw, "quaternion_xyzw"),
            received_monotonic_s=_coerce_finite_float(
                packet.received_monotonic_s,
                "received_monotonic_s",
            ),
        )


@dataclass(frozen=True)
class ToolPose:
    """Current Gen3 tool pose used when a wrapper can provide it for neutral capture."""

    position_xyz: Vector3
    quaternion_xyzw: QuaternionXYZW


@dataclass(frozen=True)
class DeadmanInput:
    """Deadman state for one mapper update."""

    pressed: bool
    is_physical: bool


@dataclass(frozen=True)
class MapperRuntimeConfig:
    """Runtime gates that are independent from calibration geometry."""

    dry_run: bool = True
    real_robot: bool = False
    command_backend: BackendName = DEFAULT_BACKEND
    allow_diagnostic_kortex: bool = False
    low_pass_alpha: float = 1.0

    def __post_init__(self) -> None:
        if self.command_backend not in (DEFAULT_BACKEND, DIAGNOSTIC_KORTEX_BACKEND):
            raise MapperError(
                "invalid_backend",
                f"unsupported command backend {self.command_backend!r}",
            )
        if self.command_backend == DIAGNOSTIC_KORTEX_BACKEND:
            if not self.allow_diagnostic_kortex:
                raise MapperError(
                    "diagnostic_backend_requires_opt_in",
                    "direct Kortex twist is diagnostic-only and must be explicitly opted in",
                )
            if not self.dry_run and not self.real_robot:
                raise MapperError(
                    "diagnostic_backend_requires_real_robot_latch_path",
                    "non-dry-run direct Kortex diagnostics must run through "
                    "real-robot latch gating",
                )
        if not math.isfinite(self.low_pass_alpha) or not 0.0 < self.low_pass_alpha <= 1.0:
            raise MapperError("invalid_filter", "low_pass_alpha must be in (0, 1]")


@dataclass(frozen=True)
class MapperCommand:
    """Dry-run/testable command object produced by the mapper core."""

    backend: BackendName
    command_type: str
    frame_id: str
    linear_xyz: Vector3
    angular_xyz: Vector3
    enabled: bool
    dry_run: bool
    stop_reason: str | None
    generated_monotonic_s: float
    needs_neutral_recapture: bool
    diagnostic_only_backend: bool = False
    role: str = EXPECTED_ROLE
    source_frame: str = EXPECTED_FRAME
    zero_command: bool = False

    def as_twist_stamped_dict(self) -> dict[str, Any]:
        """Return a ROS-message-shaped dict for wrappers/tests, not a ROS object."""

        return {
            "header": {
                "frame_id": self.frame_id,
                "stamp_monotonic_s": self.generated_monotonic_s,
            },
            "twist": {
                "linear": {
                    "x": self.linear_xyz[0],
                    "y": self.linear_xyz[1],
                    "z": self.linear_xyz[2],
                },
                "angular": {
                    "x": self.angular_xyz[0],
                    "y": self.angular_xyz[1],
                    "z": self.angular_xyz[2],
                },
            },
            "enabled": self.enabled,
            "dry_run": self.dry_run,
            "stop_reason": self.stop_reason,
            "backend": self.backend,
            "command_type": self.command_type,
        }


@dataclass
class Gen3ViveMapperCore:
    """Stateful mapper enforcing the Gen3 VIVE safety/recovery contract."""

    calibration: Gen3ViveCalibration
    runtime: MapperRuntimeConfig = field(default_factory=MapperRuntimeConfig)
    _last_pose: ViveTrackerPose | None = field(default=None, init=False, repr=False)
    _last_deadman_pressed: bool = field(default=False, init=False, repr=False)
    _neutral_tracker_pose: ViveTrackerPose | None = field(default=None, init=False, repr=False)
    _neutral_tool_pose: ToolPose | None = field(default=None, init=False, repr=False)
    _requires_neutral_recapture: bool = field(default=True, init=False, repr=False)
    _filtered_linear_xyz: Vector3 = field(default=(0.0, 0.0, 0.0), init=False, repr=False)
    _filtered_angular_xyz: Vector3 = field(default=(0.0, 0.0, 0.0), init=False, repr=False)

    def update(
        self,
        *,
        pose: ViveTrackerPose | None,
        deadman: DeadmanInput,
        now_monotonic_s: float | None = None,
        real_robot_latch: bool = False,
        servo_ready: bool = True,
        current_tool_pose: ToolPose | None = None,
    ) -> MapperCommand:
        """Advance the mapper and return a bounded command or explicit stop.

        ``pose`` may be ``None`` when no new ROS/receiver message arrived.  In
        that case the mapper checks the last valid pose age and emits stale stop
        once ``calibration.stale_timeout_s`` elapses, preventing held-last-command
        behavior when the receiver stops publishing during tracking loss.
        """

        now = time.monotonic() if now_monotonic_s is None else _coerce_finite_float(
            now_monotonic_s,
            "now_monotonic_s",
        )
        rising_edge = deadman.pressed and not self._last_deadman_pressed
        new_pose_received = pose is not None

        if pose is not None:
            try:
                self._validate_pose_contract(pose)
            except MapperError as exc:
                reason = _input_error_to_stop_reason(exc.reason)
                return self._stop(
                    reason,
                    now,
                    deadman_pressed=deadman.pressed,
                    require_recapture=True,
                    role=pose.role,
                    source_frame=pose.frame,
                )
            self._last_pose = pose

        fresh_pose = self._fresh_pose(now)
        if fresh_pose is None:
            reason = StopReason.NO_FRESH_POSE if self._last_pose is None else StopReason.STALE
            return self._stop(reason, now, deadman_pressed=deadman.pressed, require_recapture=True)

        if self.runtime.real_robot and not real_robot_latch:
            return self._stop(
                StopReason.REAL_ROBOT_LATCH,
                now,
                deadman_pressed=deadman.pressed,
                require_recapture=True,
                role=fresh_pose.role,
                source_frame=fresh_pose.frame,
            )
        if self.runtime.real_robot and not deadman.is_physical:
            return self._stop(
                StopReason.SYNTHETIC_DEADMAN_REJECTED,
                now,
                deadman_pressed=deadman.pressed,
                require_recapture=True,
                role=fresh_pose.role,
                source_frame=fresh_pose.frame,
            )
        if not servo_ready:
            return self._stop(
                StopReason.SERVO_NOT_READY,
                now,
                deadman_pressed=deadman.pressed,
                require_recapture=True,
                role=fresh_pose.role,
                source_frame=fresh_pose.frame,
            )
        if not deadman.pressed:
            return self._stop(
                StopReason.DEADMAN_RELEASED,
                now,
                deadman_pressed=False,
                require_recapture=True,
                role=fresh_pose.role,
                source_frame=fresh_pose.frame,
            )
        if self.runtime.real_robot and current_tool_pose is None:
            return self._stop(
                StopReason.TOOL_POSE_UNAVAILABLE,
                now,
                deadman_pressed=deadman.pressed,
                require_recapture=True,
                role=fresh_pose.role,
                source_frame=fresh_pose.frame,
            )

        if self._requires_neutral_recapture or self._neutral_tracker_pose is None:
            if not rising_edge:
                return self._stop(
                    StopReason.WAITING_FOR_NEUTRAL_RECAPTURE,
                    now,
                    deadman_pressed=deadman.pressed,
                    require_recapture=True,
                    role=fresh_pose.role,
                    source_frame=fresh_pose.frame,
                )
            if not new_pose_received:
                return self._wait_for_fresh_pose_before_recapture(
                    now,
                    role=fresh_pose.role,
                    source_frame=fresh_pose.frame,
                )
            self._capture_neutral(fresh_pose, current_tool_pose)
            self._last_deadman_pressed = deadman.pressed
            return self._zero_command(
                StopReason.NEUTRAL_CAPTURED.value,
                now,
                enabled=True,
                needs_neutral_recapture=False,
                role=fresh_pose.role,
                source_frame=fresh_pose.frame,
            )

        command = self._active_command(fresh_pose, now, current_tool_pose=current_tool_pose)
        self._last_deadman_pressed = deadman.pressed
        return command

    @property
    def requires_neutral_recapture(self) -> bool:
        return self._requires_neutral_recapture

    @property
    def neutral_tracker_pose(self) -> ViveTrackerPose | None:
        return self._neutral_tracker_pose

    def force_stop(
        self,
        reason: StopReason = StopReason.DEADMAN_RELEASED,
        *,
        deadman_pressed: bool = False,
        now_monotonic_s: float | None = None,
    ) -> MapperCommand:
        """Explicit stop hook for wrappers that receive an external safety event."""

        now = time.monotonic() if now_monotonic_s is None else _coerce_finite_float(
            now_monotonic_s,
            "now_monotonic_s",
        )
        return self._stop(
            reason,
            now,
            deadman_pressed=bool(deadman_pressed),
            require_recapture=True,
        )

    def _validate_pose_contract(self, pose: ViveTrackerPose) -> None:
        if pose.role != self.calibration.tracker_role:
            raise MapperError(
                "role_mismatch",
                f"expected tracker role {self.calibration.tracker_role!r}, got {pose.role!r}",
            )
        if pose.frame != self.calibration.source_frame:
            raise MapperError(
                "frame_mismatch",
                f"expected source frame {self.calibration.source_frame!r}, got {pose.frame!r}",
            )
        _coerce_vector3(pose.position_xyz, "position_xyz")
        _coerce_unit_quaternion(pose.quaternion_xyzw, "quaternion_xyzw")
        _coerce_finite_float(pose.received_monotonic_s, "received_monotonic_s")

    def _fresh_pose(self, now: float) -> ViveTrackerPose | None:
        if self._last_pose is None:
            return None
        age_s = now - self._last_pose.received_monotonic_s
        if age_s < 0.0 or age_s > self.calibration.stale_timeout_s:
            return None
        return self._last_pose

    def _capture_neutral(self, pose: ViveTrackerPose, tool_pose: ToolPose | None) -> None:
        self._neutral_tracker_pose = pose
        self._neutral_tool_pose = tool_pose
        self._requires_neutral_recapture = False
        self._filtered_linear_xyz = (0.0, 0.0, 0.0)
        self._filtered_angular_xyz = (0.0, 0.0, 0.0)

    def _active_command(
        self,
        pose: ViveTrackerPose,
        now: float,
        *,
        current_tool_pose: ToolPose | None = None,
    ) -> MapperCommand:
        neutral = self._neutral_tracker_pose
        if neutral is None:  # Defensive; update() should route this to recapture.
            return self._stop(
                StopReason.WAITING_FOR_NEUTRAL_RECAPTURE,
                now,
                deadman_pressed=True,
                require_recapture=True,
                role=pose.role,
                source_frame=pose.frame,
            )

        source_delta = _sub3(pose.position_xyz, neutral.position_xyz)
        planning_delta = _rotate_vector(
            self.calibration.T_planning_source.rotation_xyzw,
            source_delta,
        )
        target_offset = _scale3(planning_delta, self.calibration.linear_scale)
        if self.calibration.workspace_limits is not None:
            target_offset = self.calibration.workspace_limits.clamp_position(target_offset)
        if self._neutral_tool_pose is not None:
            target_position = _add3(self._neutral_tool_pose.position_xyz, target_offset)
            if current_tool_pose is not None:
                # Pose-follow mode: tracker displacement defines a target tool
                # position.  The Twist command is proportional to remaining
                # target error, so the robot stops once it reaches the target
                # even if the tracker stays offset from neutral.
                linear = _sub3(target_position, current_tool_pose.position_xyz)
            else:
                # Compatibility/rate mode for dependency-light dry-run tests or
                # diagnostic launches without TF feedback.  Real robot mode is
                # blocked earlier if current_tool_pose is unavailable.
                linear = _sub3(target_position, self._neutral_tool_pose.position_xyz)
        else:
            linear = target_offset
        linear = _deadband3(linear, self.calibration.linear_deadband_m)
        linear = _limit_magnitude(linear, self.calibration.max_linear_mps)

        delta_quat = _quat_multiply(pose.quaternion_xyzw, _quat_conjugate(neutral.quaternion_xyzw))
        angular_source = _quat_to_rotvec(delta_quat)
        angular_planning = _rotate_vector(
            self.calibration.T_planning_source.rotation_xyzw,
            angular_source,
        )
        target_rotvec = _scale3(angular_planning, self.calibration.angular_scale)
        if self._neutral_tool_pose is not None and current_tool_pose is not None:
            target_delta_quat = _rotvec_to_quat(target_rotvec)
            target_orientation = _quat_multiply(
                target_delta_quat,
                self._neutral_tool_pose.quaternion_xyzw,
            )
            orientation_error = _quat_multiply(
                target_orientation,
                _quat_conjugate(current_tool_pose.quaternion_xyzw),
            )
            angular = _quat_to_rotvec(orientation_error)
        else:
            angular = target_rotvec
        angular = _deadband3(angular, self.calibration.angular_deadband_rad)
        angular = _limit_magnitude(angular, self.calibration.max_angular_radps)

        alpha = self.runtime.low_pass_alpha
        self._filtered_linear_xyz = _blend3(self._filtered_linear_xyz, linear, alpha)
        self._filtered_angular_xyz = _blend3(self._filtered_angular_xyz, angular, alpha)

        return MapperCommand(
            backend=self.runtime.command_backend,
            command_type=_command_type(self.runtime.command_backend),
            frame_id=self.calibration.planning_frame,
            linear_xyz=self._filtered_linear_xyz,
            angular_xyz=self._filtered_angular_xyz,
            enabled=True,
            dry_run=self.runtime.dry_run,
            stop_reason=None,
            generated_monotonic_s=now,
            needs_neutral_recapture=False,
            diagnostic_only_backend=self.runtime.command_backend == DIAGNOSTIC_KORTEX_BACKEND,
            role=pose.role,
            source_frame=pose.frame,
            zero_command=(
                _is_zero3(self._filtered_linear_xyz)
                and _is_zero3(self._filtered_angular_xyz)
            ),
        )

    def _stop(
        self,
        reason: StopReason,
        now: float,
        *,
        deadman_pressed: bool,
        require_recapture: bool,
        role: str = EXPECTED_ROLE,
        source_frame: str = EXPECTED_FRAME,
    ) -> MapperCommand:
        if require_recapture:
            self._requires_neutral_recapture = True
            self._neutral_tracker_pose = None
            self._neutral_tool_pose = None
        self._filtered_linear_xyz = (0.0, 0.0, 0.0)
        self._filtered_angular_xyz = (0.0, 0.0, 0.0)
        self._last_deadman_pressed = deadman_pressed
        return self._zero_command(
            reason.value,
            now,
            enabled=False,
            needs_neutral_recapture=self._requires_neutral_recapture,
            role=role,
            source_frame=source_frame,
        )

    def _wait_for_fresh_pose_before_recapture(
        self,
        now: float,
        *,
        role: str,
        source_frame: str,
    ) -> MapperCommand:
        """Return a zero stop without consuming the pending deadman rising edge."""

        self._requires_neutral_recapture = True
        self._neutral_tracker_pose = None
        self._neutral_tool_pose = None
        self._filtered_linear_xyz = (0.0, 0.0, 0.0)
        self._filtered_angular_xyz = (0.0, 0.0, 0.0)
        return self._zero_command(
            StopReason.WAITING_FOR_NEUTRAL_RECAPTURE.value,
            now,
            enabled=False,
            needs_neutral_recapture=True,
            role=role,
            source_frame=source_frame,
        )

    def _zero_command(
        self,
        reason: str,
        now: float,
        *,
        enabled: bool,
        needs_neutral_recapture: bool,
        role: str,
        source_frame: str,
    ) -> MapperCommand:
        return MapperCommand(
            backend=self.runtime.command_backend,
            command_type=_command_type(self.runtime.command_backend),
            frame_id=self.calibration.planning_frame,
            linear_xyz=(0.0, 0.0, 0.0),
            angular_xyz=(0.0, 0.0, 0.0),
            enabled=enabled,
            dry_run=self.runtime.dry_run,
            stop_reason=reason,
            generated_monotonic_s=now,
            needs_neutral_recapture=needs_neutral_recapture,
            diagnostic_only_backend=self.runtime.command_backend == DIAGNOSTIC_KORTEX_BACKEND,
            role=role,
            source_frame=source_frame,
            zero_command=True,
        )


@dataclass
class Gen3ViveMapper:
    """Small adapter from ROS2 ``PoseStamped``-like inputs to the mapper core.

    The ROS2 wrapper owns subscriptions/publishers and safety configuration.  The
    adapter here only converts a validated ``/vive/gen3_tracker/pose`` message
    into the dependency-light :class:`ViveTrackerPose` shape and delegates to the
    core state machine.
    """

    core: Gen3ViveMapperCore
    servo_ready: bool = True

    @classmethod
    def from_calibration(
        cls,
        calibration: Gen3ViveCalibration,
        runtime: MapperRuntimeConfig | None = None,
    ) -> "Gen3ViveMapper":
        return cls(core=Gen3ViveMapperCore(calibration, runtime or MapperRuntimeConfig()))

    @classmethod
    def from_yaml_file(
        cls,
        path: str | Path,
        runtime: MapperRuntimeConfig | None = None,
    ) -> "Gen3ViveMapper":
        return cls.from_calibration(Gen3ViveCalibration.from_yaml_file(path), runtime)

    def handle_pose(
        self,
        pose_msg: Any,
        *,
        deadman_pressed: bool,
        real_robot_latch: bool,
        deadman_is_physical: bool = False,
        now_monotonic_s: float | None = None,
        current_tool_pose: ToolPose | None = None,
    ) -> MapperCommand:
        try:
            pose = pose_stamped_like_to_tracker_pose(
                pose_msg,
                tracker_role=self.core.calibration.tracker_role,
                received_monotonic_s=now_monotonic_s,
            )
        except MapperError:
            return self.core.force_stop(
                StopReason.INVALID_POSE,
                deadman_pressed=bool(deadman_pressed),
                now_monotonic_s=now_monotonic_s,
            )
        return self.core.update(
            pose=pose,
            deadman=DeadmanInput(
                pressed=bool(deadman_pressed),
                is_physical=bool(deadman_is_physical),
            ),
            now_monotonic_s=now_monotonic_s,
            real_robot_latch=bool(real_robot_latch),
            servo_ready=self.servo_ready,
            current_tool_pose=current_tool_pose,
        )

    def poll_safety(
        self,
        *,
        deadman_pressed: bool,
        real_robot_latch: bool,
        deadman_is_physical: bool = False,
        now_monotonic_s: float | None = None,
        current_tool_pose: ToolPose | None = None,
    ) -> MapperCommand:
        """Run safety checks without a new pose message.

        ROS2 pose subscriptions are event-driven, so sender/tracking loss must
        still call into the mapper periodically to turn stale poses into an
        explicit zero command and force neutral recapture.
        """

        return self.core.update(
            pose=None,
            deadman=DeadmanInput(
                pressed=bool(deadman_pressed),
                is_physical=bool(deadman_is_physical),
            ),
            now_monotonic_s=now_monotonic_s,
            real_robot_latch=bool(real_robot_latch),
            servo_ready=self.servo_ready,
            current_tool_pose=current_tool_pose,
        )


def build_mapper_from_yaml(
    path: str | Path,
    runtime: MapperRuntimeConfig | None = None,
) -> Gen3ViveMapper:
    """Factory used by the ROS2 wrapper default ``--mapper-factory`` setting."""

    return Gen3ViveMapper.from_yaml_file(path, runtime)


def pose_stamped_like_to_tracker_pose(
    pose_msg: Any,
    *,
    tracker_role: str,
    received_monotonic_s: float | None = None,
) -> ViveTrackerPose:
    """Convert a ROS2 ``PoseStamped`` object or equivalent mapping to tracker pose."""

    header = _child(pose_msg, "header")
    pose = _child(pose_msg, "pose")
    position = _child(pose, "position")
    orientation = _child(pose, "orientation")
    frame_id = _child(header, "frame_id")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise MapperError("invalid_pose", "PoseStamped header.frame_id must be non-empty")
    return ViveTrackerPose(
        role=tracker_role,
        frame=frame_id.strip(),
        position_xyz=(
            _coerce_finite_float(_child(position, "x"), "pose.position.x"),
            _coerce_finite_float(_child(position, "y"), "pose.position.y"),
            _coerce_finite_float(_child(position, "z"), "pose.position.z"),
        ),
        quaternion_xyzw=_coerce_unit_quaternion(
            (
                _coerce_finite_float(_child(orientation, "x"), "pose.orientation.x"),
                _coerce_finite_float(_child(orientation, "y"), "pose.orientation.y"),
                _coerce_finite_float(_child(orientation, "z"), "pose.orientation.z"),
                _coerce_finite_float(_child(orientation, "w"), "pose.orientation.w"),
            ),
            "pose.orientation",
        ),
        received_monotonic_s=(
            time.monotonic()
            if received_monotonic_s is None
            else _coerce_finite_float(received_monotonic_s, "received_monotonic_s")
        ),
    )


def _command_type(backend: BackendName) -> str:
    if backend == DEFAULT_BACKEND:
        return DEFAULT_COMMAND_TYPE
    if backend == DIAGNOSTIC_KORTEX_BACKEND:
        return DIAGNOSTIC_KORTEX_COMMAND_TYPE
    raise MapperError("invalid_backend", f"unsupported command backend {backend!r}")


def _input_error_to_stop_reason(reason: str) -> StopReason:
    if reason == "frame_mismatch":
        return StopReason.FRAME_MISMATCH
    if reason == "role_mismatch":
        return StopReason.ROLE_MISMATCH
    return StopReason.INVALID_POSE


def _required_text(raw: Mapping[str, Any], field_name: str) -> str:
    value = raw.get(field_name)
    if not isinstance(value, str) or not value.strip():
        raise MapperError("missing_calibration_field", f"{field_name!r} must be non-empty text")
    return value.strip()


def _required_mapping(raw: Mapping[str, Any], field_name: str) -> Mapping[str, Any]:
    value = raw.get(field_name)
    if not isinstance(value, Mapping):
        raise MapperError("missing_calibration_field", f"{field_name!r} must be a mapping")
    return value


def _child(value: Any, field_name: str) -> Any:
    if isinstance(value, Mapping):
        if field_name not in value:
            raise MapperError("invalid_pose", f"missing PoseStamped field {field_name!r}")
        return value[field_name]
    try:
        return getattr(value, field_name)
    except AttributeError as exc:
        raise MapperError("invalid_pose", f"missing PoseStamped field {field_name!r}") from exc


def _finite_float_tuple(
    raw: Mapping[str, Any],
    field_name: str,
    expected_len: int,
    *,
    parent: str,
) -> tuple[float, ...]:
    value = raw.get(field_name)
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise MapperError(
            "invalid_calibration",
            f"{parent}.{field_name} must be a sequence of {expected_len} numbers",
        )
    if len(value) != expected_len:
        raise MapperError(
            "invalid_calibration",
            f"{parent}.{field_name} must have length {expected_len}",
        )
    return tuple(_coerce_finite_float(item, f"{parent}.{field_name}") for item in value)


def _unit_quaternion_tuple(
    raw: Mapping[str, Any],
    field_name: str,
    *,
    parent: str,
) -> QuaternionXYZW:
    quat = cast(
        QuaternionXYZW,
        _finite_float_tuple(raw, field_name, 4, parent=parent),
    )
    return _coerce_unit_quaternion(quat, f"{parent}.{field_name}")


def _limit_pair(raw: Mapping[str, Any], axis: str) -> tuple[float, float]:
    pair = _finite_float_tuple(raw, axis, 2, parent="workspace_limits")
    low, high = pair
    if low > high:
        raise MapperError("invalid_calibration", f"workspace_limits.{axis} min exceeds max")
    return (low, high)


def _positive_finite_float(raw: Mapping[str, Any], field_name: str) -> float:
    value = _coerce_finite_float(raw.get(field_name), field_name)
    if value <= 0.0:
        raise MapperError("invalid_calibration", f"{field_name!r} must be > 0")
    return value


def _nonnegative_finite_float(
    raw: Mapping[str, Any],
    field_name: str,
    *,
    default: float,
) -> float:
    value = _coerce_finite_float(raw.get(field_name, default), field_name)
    if value < 0.0:
        raise MapperError("invalid_calibration", f"{field_name!r} must be >= 0")
    return value


def _coerce_vector3(value: Any, field_name: str) -> Vector3:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise MapperError("invalid_pose", f"{field_name} must be a length-3 sequence")
    if len(value) != 3:
        raise MapperError("invalid_pose", f"{field_name} must have length 3")
    return cast(Vector3, tuple(_coerce_finite_float(item, field_name) for item in value))


def _coerce_unit_quaternion(value: Any, field_name: str) -> QuaternionXYZW:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise MapperError("invalid_pose", f"{field_name} must be a length-4 sequence")
    if len(value) != 4:
        raise MapperError("invalid_pose", f"{field_name} must have length 4")
    quat = cast(QuaternionXYZW, tuple(_coerce_finite_float(item, field_name) for item in value))
    norm = math.sqrt(sum(item * item for item in quat))
    if abs(norm - 1.0) > 1.0e-3:
        raise MapperError("invalid_pose", f"{field_name} quaternion must be normalized")
    return quat


def _coerce_finite_float(value: Any, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MapperError("non_finite", f"{field_name} must be a finite number")
    float_value = float(value)
    if not math.isfinite(float_value):
        raise MapperError("non_finite", f"{field_name} must be finite")
    return float_value


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def _add3(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub3(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale3(a: Vector3, scale: float) -> Vector3:
    return (a[0] * scale, a[1] * scale, a[2] * scale)


def _blend3(previous: Vector3, current: Vector3, alpha: float) -> Vector3:
    beta = 1.0 - alpha
    return (
        beta * previous[0] + alpha * current[0],
        beta * previous[1] + alpha * current[1],
        beta * previous[2] + alpha * current[2],
    )


def _norm3(a: Vector3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _deadband3(vector: Vector3, deadband: float) -> Vector3:
    if deadband <= 0.0 or _norm3(vector) > deadband:
        return vector
    return (0.0, 0.0, 0.0)


def _limit_magnitude(vector: Vector3, max_magnitude: float) -> Vector3:
    magnitude = _norm3(vector)
    if magnitude <= max_magnitude or magnitude == 0.0:
        return vector
    return _scale3(vector, max_magnitude / magnitude)


def _is_zero3(vector: Vector3, *, eps: float = 1.0e-12) -> bool:
    return abs(vector[0]) <= eps and abs(vector[1]) <= eps and abs(vector[2]) <= eps


def _quat_conjugate(quat: QuaternionXYZW) -> QuaternionXYZW:
    x, y, z, w = quat
    return (-x, -y, -z, w)


def _quat_multiply(a: QuaternionXYZW, b: QuaternionXYZW) -> QuaternionXYZW:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(quat: QuaternionXYZW, vector: Vector3) -> Vector3:
    vx, vy, vz = vector
    rotated = _quat_multiply(
        _quat_multiply(quat, (vx, vy, vz, 0.0)),
        _quat_conjugate(quat),
    )
    return (rotated[0], rotated[1], rotated[2])



def _rotvec_to_quat(rotvec: Vector3) -> QuaternionXYZW:
    angle = _norm3(rotvec)
    if angle < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    half_angle = 0.5 * angle
    scale = math.sin(half_angle) / angle
    return (
        rotvec[0] * scale,
        rotvec[1] * scale,
        rotvec[2] * scale,
        math.cos(half_angle),
    )

def _quat_to_rotvec(quat: QuaternionXYZW) -> Vector3:
    x, y, z, w = quat
    if w < 0.0:  # shortest-path orientation error
        x, y, z, w = -x, -y, -z, -w
    vector_norm = math.sqrt(x * x + y * y + z * z)
    if vector_norm < 1.0e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vector_norm, w)
    return (x / vector_norm * angle, y / vector_norm * angle, z / vector_norm * angle)


__all__ = [
    "DEFAULT_BACKEND",
    "DEFAULT_COMMAND_TYPE",
    "DIAGNOSTIC_KORTEX_BACKEND",
    "DIAGNOSTIC_KORTEX_COMMAND_TYPE",
    "BackendName",
    "CalibrationTransform",
    "DeadmanInput",
    "Gen3ViveCalibration",
    "Gen3ViveMapper",
    "Gen3ViveMapperCore",
    "MapperCommand",
    "MapperError",
    "MapperRuntimeConfig",
    "StopReason",
    "ToolPose",
    "ViveTrackerPose",
    "WorkspaceLimits",
    "build_mapper_from_yaml",
    "pose_stamped_like_to_tracker_pose",
]
