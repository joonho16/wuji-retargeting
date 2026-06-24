#!/usr/bin/env python3
"""ROS2 wrapper and CLI boundary for the Gen3 VIVE mapper safety path.

This module owns ROS2-facing integration policy around the mapper core without
owning the mapper algorithm itself.  It intentionally keeps configuration,
backend selection, real-robot gating, and TwistStamped payload construction
unit-testable without ROS2 so safety defaults can be verified in CI.

The default and only ROS2 wrapper command path is MoveIt Servo
``TwistStamped``.  Direct Kortex twist remains quarantined in the dependency-light
mapper core for diagnostic tests, but this ROS2 wrapper does not expose that
backend as a runnable node branch.
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import sys
import time
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Literal, Protocol, cast

DEFAULT_POSE_TOPIC = "/vive/gen3_tracker/pose"
DEFAULT_TWIST_TOPIC = "/servo_node/delta_twist_cmds"
DEFAULT_STATUS_TOPIC = "/gen3_vive/status"
DEFAULT_PHYSICAL_DEADMAN_TOPIC = "/gen3_vive/physical_deadman"
DEFAULT_PLANNING_FRAME = "base_link"
DEFAULT_TOOL_FRAME = "end_effector_link"
DEFAULT_MAPPER_FACTORY = "example.arm_teleop.gen3_vive_mapper:build_mapper_from_yaml"
DEFAULT_WATCHDOG_PERIOD_S = 0.02

MOVEIT_SERVO_TWIST_BACKEND = "moveit_servo_twist"
DIAGNOSTIC_KORTEX_TWIST_BACKEND = "diagnostic_kortex_twist"
BackendName = Literal["moveit_servo_twist", "diagnostic_kortex_twist"]
DeadmanSource = Literal["synthetic_cli", "physical_topic"]


class Gen3ViveRos2WrapperConfigError(ValueError):
    """Raised when wrapper configuration would violate the safety contract."""


@dataclass(frozen=True)
class TwistCommand:
    """Mapper command shape consumed by the ROS2 wrapper.

    The mapper core may return this dataclass directly or a mapping with matching
    fields.  Disabled commands are converted to zero TwistStamped payloads so
    MoveIt Servo can receive an explicit stop when the safety state requires it.
    """

    linear_xyz: tuple[float, float, float]
    angular_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    planning_frame: str = DEFAULT_PLANNING_FRAME
    command_enabled: bool = True
    stop_reason: str | None = None


@dataclass(frozen=True)
class Gen3ViveRos2WrapperConfig:
    """Safety-relevant ROS2 wrapper and CLI configuration."""

    twist_topic: str = DEFAULT_TWIST_TOPIC
    status_topic: str = DEFAULT_STATUS_TOPIC
    physical_deadman_topic: str = DEFAULT_PHYSICAL_DEADMAN_TOPIC
    planning_frame: str = DEFAULT_PLANNING_FRAME
    tool_frame: str = DEFAULT_TOOL_FRAME
    mapper_factory: str = DEFAULT_MAPPER_FACTORY
    calibration_path: str | None = None
    watchdog_period_s: float = DEFAULT_WATCHDOG_PERIOD_S
    backend: BackendName = MOVEIT_SERVO_TWIST_BACKEND
    deadman_source: DeadmanSource = "synthetic_cli"
    dry_run: bool = True
    real_robot: bool = False
    real_robot_latch: bool = False
    synthetic_deadman_pressed: bool = False
    low_pass_alpha: float = 1.0

    def validate(self) -> None:
        """Validate safety defaults before publishing or spinning a ROS2 node."""

        _require_non_empty(self.status_topic, "status_topic")
        _require_non_empty(self.planning_frame, "planning_frame")
        _require_non_empty(self.tool_frame, "tool_frame")
        _require_non_empty(self.mapper_factory, "mapper_factory")
        _require_positive_finite(self.watchdog_period_s, "watchdog_period_s")
        _require_unit_interval_open_closed(self.low_pass_alpha, "low_pass_alpha")

        if self.backend == MOVEIT_SERVO_TWIST_BACKEND:
            _require_non_empty(self.twist_topic, "twist_topic")
        elif self.backend == DIAGNOSTIC_KORTEX_TWIST_BACKEND:
            raise Gen3ViveRos2WrapperConfigError(
                "diagnostic Kortex backend is not implemented by this ROS2 "
                "wrapper; use the mapper-core diagnostic path instead"
            )
        else:  # pragma: no cover - typing prevents this unless callers cast.
            raise Gen3ViveRos2WrapperConfigError(f"unsupported backend {self.backend!r}")

        if self.deadman_source == "physical_topic":
            _require_non_empty(self.physical_deadman_topic, "physical_deadman_topic")
        elif self.deadman_source != "synthetic_cli":  # pragma: no cover - typing prevents this unless callers cast.
            raise Gen3ViveRos2WrapperConfigError(
                f"unsupported deadman_source {self.deadman_source!r}"
            )

        if self.real_robot:
            if self.dry_run:
                raise Gen3ViveRos2WrapperConfigError(
                    "real_robot mode cannot run with dry_run enabled"
                )
            if self.deadman_source != "physical_topic":
                raise Gen3ViveRos2WrapperConfigError(
                    "real_robot mode requires deadman_source='physical_topic'"
                )
            if not self.real_robot_latch:
                raise Gen3ViveRos2WrapperConfigError(
                    "real_robot mode requires the explicit real_robot_latch gate"
                )
            if self.synthetic_deadman_pressed:
                raise Gen3ViveRos2WrapperConfigError(
                    "synthetic deadman must not be accepted in real_robot mode"
                )
            if not self.calibration_path:
                raise Gen3ViveRos2WrapperConfigError(
                    "real_robot mode requires an explicit calibration_path"
                )
        elif self.deadman_source == "synthetic_cli" and not self.dry_run:
            raise Gen3ViveRos2WrapperConfigError(
                "synthetic deadman is allowed only in dry-run mode"
            )

    def as_json_dict(self) -> dict[str, Any]:
        """Return a stable JSON-serializable config dictionary."""

        return asdict(self)


class MapperProtocol(Protocol):
    """Minimal protocol the worker-1 mapper core can satisfy."""

    def handle_pose(
        self,
        pose_msg: Any,
        *,
        deadman_pressed: bool,
        real_robot_latch: bool,
        deadman_is_physical: bool = False,
        now_monotonic_s: float | None = None,
        current_tool_pose: Any | None = None,
    ) -> Any:
        """Convert one ROS2 pose message into a wrapper command or stop."""

    def poll_safety(
        self,
        *,
        deadman_pressed: bool,
        real_robot_latch: bool,
        deadman_is_physical: bool = False,
        now_monotonic_s: float | None = None,
        current_tool_pose: Any | None = None,
    ) -> Any:
        """Run stale/deadman safety checks without a new pose message."""


def _require_non_empty(value: str, field_name: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise Gen3ViveRos2WrapperConfigError(f"{field_name} must be non-empty")


def _require_positive_finite(value: float, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Gen3ViveRos2WrapperConfigError(f"{field_name} must be a positive number")
    if not math.isfinite(float(value)) or float(value) <= 0.0:
        raise Gen3ViveRos2WrapperConfigError(f"{field_name} must be positive and finite")


def _require_unit_interval_open_closed(value: float, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Gen3ViveRos2WrapperConfigError(f"{field_name} must be in (0, 1]")
    value_f = float(value)
    if not math.isfinite(value_f) or not 0.0 < value_f <= 1.0:
        raise Gen3ViveRos2WrapperConfigError(f"{field_name} must be in (0, 1]")


def _finite_xyz(values: Any, field_name: str) -> tuple[float, float, float]:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes, bytearray)):
        raise Gen3ViveRos2WrapperConfigError(f"{field_name} must contain exactly 3 values")
    if len(values) != 3:
        raise Gen3ViveRos2WrapperConfigError(f"{field_name} must contain exactly 3 values")
    out: list[float] = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise Gen3ViveRos2WrapperConfigError(f"{field_name} must contain numbers")
        f = float(value)
        if not math.isfinite(f):
            raise Gen3ViveRos2WrapperConfigError(f"{field_name} must be finite")
        out.append(f)
    return cast(tuple[float, float, float], tuple(out))


def coerce_twist_command(command: TwistCommand | Mapping[str, Any] | None) -> TwistCommand:
    """Normalize mapper output into a :class:`TwistCommand`.

    ``None`` means no enabled mapper command and is converted to an explicit stop.
    A mapping form keeps this wrapper independent from the worker-1 mapper class.
    """

    if command is None:
        return TwistCommand(
            linear_xyz=(0.0, 0.0, 0.0),
            angular_xyz=(0.0, 0.0, 0.0),
            command_enabled=False,
            stop_reason="mapper_returned_no_command",
        )
    if isinstance(command, TwistCommand):
        return command

    linear = _finite_xyz(command.get("linear_xyz", (0.0, 0.0, 0.0)), "linear_xyz")
    angular = _finite_xyz(command.get("angular_xyz", (0.0, 0.0, 0.0)), "angular_xyz")
    planning_frame = command.get("planning_frame", DEFAULT_PLANNING_FRAME)
    if not isinstance(planning_frame, str) or not planning_frame.strip():
        raise Gen3ViveRos2WrapperConfigError("planning_frame must be a non-empty string")
    command_enabled = bool(command.get("command_enabled", True))
    stop_reason_raw = command.get("stop_reason")
    stop_reason = str(stop_reason_raw) if stop_reason_raw is not None else None
    return TwistCommand(
        linear_xyz=linear,
        angular_xyz=angular,
        planning_frame=planning_frame,
        command_enabled=command_enabled,
        stop_reason=stop_reason,
    )


def mapper_command_to_twist_command(command: Any) -> TwistCommand:
    """Convert mapper-core ``MapperCommand`` objects to wrapper ``TwistCommand``.

    The mapper core intentionally stays ROS-free and exposes ``linear_xyz``,
    ``angular_xyz``, ``frame_id``, ``enabled``, and ``stop_reason``.  Keeping the
    conversion explicit prevents accidental truthiness/coercion bugs at the
    safety boundary.
    """

    if isinstance(command, TwistCommand) or command is None or isinstance(command, Mapping):
        return coerce_twist_command(command)
    try:
        linear_xyz = getattr(command, "linear_xyz")
        angular_xyz = getattr(command, "angular_xyz")
        planning_frame = getattr(command, "frame_id")
        enabled = getattr(command, "enabled")
        stop_reason = getattr(command, "stop_reason")
    except AttributeError as exc:
        raise Gen3ViveRos2WrapperConfigError(
            "mapper command must be TwistCommand, mapping, None, or MapperCommand-like"
        ) from exc
    return coerce_twist_command(
        {
            "linear_xyz": linear_xyz,
            "angular_xyz": angular_xyz,
            "planning_frame": planning_frame,
            "command_enabled": enabled,
            "stop_reason": stop_reason,
        }
    )


def build_twist_stamped_payload(
    command: TwistCommand | Mapping[str, Any] | None,
    *,
    stamp_sec: int | None = None,
    stamp_nanosec: int = 0,
) -> dict[str, Any]:
    """Build a dependency-free dictionary matching ROS2 ``TwistStamped`` fields."""

    normalized = coerce_twist_command(command)
    if stamp_sec is None:
        stamp_sec = int(time.time())
    linear = normalized.linear_xyz if normalized.command_enabled else (0.0, 0.0, 0.0)
    angular = normalized.angular_xyz if normalized.command_enabled else (0.0, 0.0, 0.0)
    return {
        "header": {
            "stamp": {"sec": int(stamp_sec), "nanosec": int(stamp_nanosec)},
            "frame_id": normalized.planning_frame,
        },
        "twist": {
            "linear": {"x": linear[0], "y": linear[1], "z": linear[2]},
            "angular": {"x": angular[0], "y": angular[1], "z": angular[2]},
        },
        "command_enabled": normalized.command_enabled,
        "stop_reason": normalized.stop_reason,
    }


def load_mapper_factory(import_path: str = DEFAULT_MAPPER_FACTORY) -> Any:
    """Load the worker-1 mapper factory lazily by ``module:attribute`` path."""

    if ":" not in import_path:
        raise Gen3ViveRos2WrapperConfigError(
            "mapper_factory must use 'module:attribute' syntax"
        )
    module_name, attr_name = import_path.split(":", 1)
    _ensure_repo_root_importable_for_direct_script(module_name)
    try:
        module = importlib.import_module(module_name)
    except ImportError as exc:
        raise Gen3ViveRos2WrapperConfigError(
            f"mapper factory module {module_name!r} could not be imported"
        ) from exc
    try:
        return getattr(module, attr_name)
    except AttributeError as exc:
        raise Gen3ViveRos2WrapperConfigError(
            f"mapper factory {attr_name!r} not found in {module_name!r}"
        ) from exc


def _ensure_repo_root_importable_for_direct_script(module_name: str) -> None:
    """Make documented filename launches work for repo-local package imports."""

    if not module_name.startswith("example."):
        return
    repo_root = Path(__file__).resolve().parents[2]
    repo_root_text = str(repo_root)
    if repo_root_text not in sys.path:
        sys.path.insert(0, repo_root_text)


def build_mapper_runtime_config(config: Gen3ViveRos2WrapperConfig) -> Any:
    """Translate wrapper safety gates into the mapper-core runtime config."""

    _ensure_repo_root_importable_for_direct_script("example.arm_teleop.gen3_vive_mapper")
    from example.arm_teleop.gen3_vive_mapper import MapperRuntimeConfig

    return MapperRuntimeConfig(
        dry_run=config.dry_run,
        real_robot=config.real_robot,
        command_backend=config.backend,
        allow_diagnostic_kortex=False,
        low_pass_alpha=config.low_pass_alpha,
    )


def construct_mapper_with_factory(
    config: Gen3ViveRos2WrapperConfig,
    mapper_factory: Any,
) -> MapperProtocol:
    """Construct the mapper while preserving wrapper runtime safety gates."""

    if not config.calibration_path:
        raise Gen3ViveRos2WrapperConfigError(
            "--spin requires --calibration-path for mapper construction"
        )
    runtime = build_mapper_runtime_config(config)
    mapper = mapper_factory(config.calibration_path, runtime)
    _validate_mapper_protocol(mapper)
    return cast(MapperProtocol, mapper)


def _validate_mapper_protocol(mapper: Any) -> None:
    missing = [
        method_name
        for method_name in ("handle_pose", "poll_safety")
        if not callable(getattr(mapper, method_name, None))
    ]
    if missing:
        raise Gen3ViveRos2WrapperConfigError(
            f"mapper missing required method(s): {', '.join(missing)}"
        )


@dataclass
class Gen3ViveSafetyLoop:
    """Dependency-light wrapper safety loop used by the ROS2 node and tests."""

    config: Gen3ViveRos2WrapperConfig
    mapper: MapperProtocol

    def __post_init__(self) -> None:
        self._deadman_pressed = bool(self.config.synthetic_deadman_pressed)

    def update_deadman(
        self,
        pressed: bool,
        *,
        real_robot_latch: bool,
        now_monotonic_s: float | None = None,
    ) -> TwistCommand | None:
        was_pressed = self._deadman_pressed
        self._deadman_pressed = bool(pressed)
        if was_pressed and not self._deadman_pressed:
            return self.poll_safety(
                real_robot_latch=real_robot_latch,
                now_monotonic_s=now_monotonic_s,
            )
        return None

    def handle_pose(
        self,
        pose_msg: Any,
        *,
        real_robot_latch: bool,
        now_monotonic_s: float | None = None,
        current_tool_pose: Any | None = None,
    ) -> TwistCommand:
        command = self.mapper.handle_pose(
            pose_msg,
            deadman_pressed=self._deadman_pressed,
            real_robot_latch=bool(real_robot_latch),
            deadman_is_physical=self.config.deadman_source == "physical_topic",
            now_monotonic_s=now_monotonic_s,
            current_tool_pose=current_tool_pose,
        )
        return mapper_command_to_twist_command(command)

    def poll_safety(
        self,
        *,
        real_robot_latch: bool,
        now_monotonic_s: float | None = None,
        current_tool_pose: Any | None = None,
    ) -> TwistCommand:
        command = self.mapper.poll_safety(
            deadman_pressed=self._deadman_pressed,
            real_robot_latch=bool(real_robot_latch),
            deadman_is_physical=self.config.deadman_source == "physical_topic",
            now_monotonic_s=now_monotonic_s,
            current_tool_pose=current_tool_pose,
        )
        return mapper_command_to_twist_command(command)


class Ros2Gen3ViveMapperWrapper:
    """Optional ROS2 node glue for an injected mapper core.

    ROS2 imports occur only when this class is instantiated.  That keeps default
    CLI/config/test paths runnable on development machines without ROS2 while
    preserving a concrete node boundary for Linux robot hosts.
    """

    def __init__(self, config: Gen3ViveRos2WrapperConfig, mapper: MapperProtocol):
        config.validate()
        if config.backend != MOVEIT_SERVO_TWIST_BACKEND:
            raise Gen3ViveRos2WrapperConfigError(
                "Ros2Gen3ViveMapperWrapper publishes only MoveIt Servo TwistStamped; "
                "diagnostic Kortex is intentionally not implemented here"
            )

        import rclpy
        from example.arm_teleop.gen3_vive_mapper import ToolPose
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from rclpy.node import Node
        from std_msgs.msg import Bool, String
        from tf2_ros import Buffer, TransformListener

        class _WrapperNode(Node):
            def __init__(self) -> None:
                super().__init__("gen3_vive_mapper_wrapper")
                self._config = config
                self._safety_loop = Gen3ViveSafetyLoop(config, mapper)
                self._twist_type = TwistStamped
                self._status_type = String
                self._tf_buffer = Buffer()
                self._tf_listener = TransformListener(self._tf_buffer, self)
                self._last_tool_pose_available = False
                self.declare_parameter("real_robot_latch", bool(config.real_robot_latch))
                self._twist_pub = self.create_publisher(TwistStamped, config.twist_topic, 10)
                self._status_pub = self.create_publisher(String, config.status_topic, 10)
                self.create_subscription(PoseStamped, DEFAULT_POSE_TOPIC, self._on_pose, 10)
                self._watchdog_timer = self.create_timer(
                    config.watchdog_period_s,
                    self._on_watchdog,
                )
                if config.deadman_source == "physical_topic":
                    self.create_subscription(
                        Bool,
                        config.physical_deadman_topic,
                        self._on_deadman,
                        10,
                    )
                self.get_logger().info(
                    "Gen3 VIVE wrapper publishing MoveIt Servo TwistStamped to "
                    f"{config.twist_topic}; dry_run={config.dry_run}; "
                    f"real_robot={config.real_robot}"
                )

            def _on_deadman(self, msg: Any) -> None:
                try:
                    command = self._safety_loop.update_deadman(
                        bool(msg.data),
                        real_robot_latch=self._real_robot_latch(),
                    )
                    if command is not None:
                        self._publish_command(command)
                except Exception as exc:  # defensive ROS callback boundary: fail closed.
                    self._publish_exception_stop(exc)

            def _real_robot_latch(self) -> bool:
                return bool(self.get_parameter("real_robot_latch").value)

            def _current_tool_pose(self) -> Any | None:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        config.planning_frame,
                        config.tool_frame,
                        rclpy.time.Time(),
                    )
                except Exception:
                    self._last_tool_pose_available = False
                    return None
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self._last_tool_pose_available = True
                return ToolPose(
                    position_xyz=(
                        float(translation.x),
                        float(translation.y),
                        float(translation.z),
                    ),
                    quaternion_xyzw=(
                        float(rotation.x),
                        float(rotation.y),
                        float(rotation.z),
                        float(rotation.w),
                    ),
                )

            def _on_pose(self, pose_msg: Any) -> None:
                try:
                    command = self._safety_loop.handle_pose(
                        pose_msg,
                        real_robot_latch=self._real_robot_latch(),
                        current_tool_pose=self._current_tool_pose(),
                    )
                    self._publish_command(command)
                except Exception as exc:  # defensive: fail closed with zero twist.
                    self._publish_exception_stop(exc)

            def _on_watchdog(self) -> None:
                try:
                    self._publish_command(
                        self._safety_loop.poll_safety(
                            real_robot_latch=self._real_robot_latch(),
                            current_tool_pose=self._current_tool_pose(),
                        )
                    )
                except Exception as exc:  # defensive ROS callback boundary: fail closed.
                    self._publish_exception_stop(exc)

            def _publish_command(self, command: TwistCommand) -> None:
                payload = build_twist_stamped_payload(command)
                self._publish_twist_payload(payload)
                self._publish_status(payload)

            def _publish_exception_stop(self, exc: Exception) -> None:
                payload = build_twist_stamped_payload(
                    TwistCommand(
                        linear_xyz=(0.0, 0.0, 0.0),
                        angular_xyz=(0.0, 0.0, 0.0),
                        planning_frame=config.planning_frame,
                        command_enabled=False,
                        stop_reason=f"wrapper_exception:{type(exc).__name__}",
                    )
                )
                self._publish_twist_payload(payload)
                self._publish_status(payload)
                self.get_logger().error(f"Mapper wrapper stopped command output: {exc}")

            def _publish_twist_payload(self, payload: Mapping[str, Any]) -> None:
                msg = self._twist_type()
                msg.header.frame_id = str(payload["header"]["frame_id"])
                msg.header.stamp = self.get_clock().now().to_msg()
                twist = payload["twist"]
                msg.twist.linear.x = float(twist["linear"]["x"])
                msg.twist.linear.y = float(twist["linear"]["y"])
                msg.twist.linear.z = float(twist["linear"]["z"])
                msg.twist.angular.x = float(twist["angular"]["x"])
                msg.twist.angular.y = float(twist["angular"]["y"])
                msg.twist.angular.z = float(twist["angular"]["z"])
                self._twist_pub.publish(msg)

            def _publish_status(self, payload: Mapping[str, Any]) -> None:
                msg = self._status_type()
                msg.data = json.dumps(
                    {
                        "backend": config.backend,
                        "deadman_source": config.deadman_source,
                        "dry_run": config.dry_run,
                        "real_robot": config.real_robot,
                        "real_robot_latch": self._real_robot_latch(),
                        "tool_frame": config.tool_frame,
                        "tool_pose_available": self._last_tool_pose_available,
                        "command_enabled": payload["command_enabled"],
                        "stop_reason": payload["stop_reason"],
                    },
                    sort_keys=True,
                    separators=(",", ":"),
                )
                self._status_pub.publish(msg)

        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self._node = _WrapperNode()

    def spin(self) -> None:
        try:
            self._rclpy.spin(self._node)
        finally:
            self._node.destroy_node()
            self._rclpy.shutdown()


def _json_arg(value: str) -> Any:
    try:
        return json.loads(value)
    except json.JSONDecodeError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Safe ROS2 wrapper/CLI boundary for Gen3 VIVE mapper commands.",
    )
    parser.add_argument("--twist-topic", default=DEFAULT_TWIST_TOPIC)
    parser.add_argument("--status-topic", default=DEFAULT_STATUS_TOPIC)
    parser.add_argument("--physical-deadman-topic", default=DEFAULT_PHYSICAL_DEADMAN_TOPIC)
    parser.add_argument("--planning-frame", default=DEFAULT_PLANNING_FRAME)
    parser.add_argument("--tool-frame", default=DEFAULT_TOOL_FRAME)
    parser.add_argument("--mapper-factory", default=DEFAULT_MAPPER_FACTORY)
    parser.add_argument("--calibration-path", default=None)
    parser.add_argument("--watchdog-period-s", type=float, default=DEFAULT_WATCHDOG_PERIOD_S)
    parser.add_argument(
        "--low-pass-alpha",
        type=float,
        default=1.0,
        help=(
            "Mapper command low-pass alpha in (0, 1]. 1 disables smoothing; "
            "smaller values reduce VIVE/TF jitter but add lag."
        ),
    )
    parser.add_argument(
        "--backend",
        choices=[MOVEIT_SERVO_TWIST_BACKEND],
        default=MOVEIT_SERVO_TWIST_BACKEND,
    )
    parser.add_argument(
        "--deadman-source",
        choices=["synthetic_cli", "physical_topic"],
        default="synthetic_cli",
    )
    parser.add_argument("--no-dry-run", dest="dry_run", action="store_false")
    parser.set_defaults(dry_run=True)
    parser.add_argument("--real-robot", action="store_true", default=False)
    parser.add_argument("--real-robot-latch", action="store_true", default=False)
    parser.add_argument("--synthetic-deadman-pressed", action="store_true", default=False)
    parser.add_argument(
        "--print-config",
        action="store_true",
        help="Validate and print wrapper config, then exit without ROS2 imports.",
    )
    parser.add_argument(
        "--twist-command-json",
        type=_json_arg,
        help="Validate one mapper command JSON and print a TwistStamped-shaped payload.",
    )
    parser.add_argument(
        "--spin",
        action="store_true",
        help="Instantiate ROS2 node and spin. Requires ROS2 and the mapper factory.",
    )
    return parser


def config_from_args(args: argparse.Namespace) -> Gen3ViveRos2WrapperConfig:
    return Gen3ViveRos2WrapperConfig(
        twist_topic=args.twist_topic,
        status_topic=args.status_topic,
        physical_deadman_topic=args.physical_deadman_topic,
        planning_frame=args.planning_frame,
        tool_frame=args.tool_frame,
        mapper_factory=args.mapper_factory,
        calibration_path=args.calibration_path,
        watchdog_period_s=float(args.watchdog_period_s),
        backend=cast(BackendName, args.backend),
        deadman_source=cast(DeadmanSource, args.deadman_source),
        dry_run=bool(args.dry_run),
        real_robot=bool(args.real_robot),
        real_robot_latch=bool(args.real_robot_latch),
        synthetic_deadman_pressed=bool(args.synthetic_deadman_pressed),
        low_pass_alpha=float(args.low_pass_alpha),
    )


def main(argv: Iterable[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    config = config_from_args(args)
    try:
        config.validate()
        if args.twist_command_json is not None:
            print(
                json.dumps(
                    build_twist_stamped_payload(args.twist_command_json, stamp_sec=0),
                    sort_keys=True,
                )
            )
            return 0
        if args.print_config or not args.spin:
            print(json.dumps(config.as_json_dict(), sort_keys=True))
            return 0

        mapper_factory = load_mapper_factory(config.mapper_factory)
        mapper = construct_mapper_with_factory(config, mapper_factory)
        Ros2Gen3ViveMapperWrapper(config, mapper).spin()
        return 0
    except Gen3ViveRos2WrapperConfigError as exc:
        print(f"ERROR: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
