"""Follow a robot-mounted control point; command native Twist at the actual TCP."""
from __future__ import annotations

from dataclasses import replace
import math
from example.arm_teleop.gen3_vive_mapper import (  # noqa: E402
    DeadmanInput, Gen3ViveCalibration, Gen3ViveMapperCore, MapperCommand,
    MapperRuntimeConfig, StopReason, ToolPose, ViveTrackerPose,
    _quat_multiply, _rotate_vector,
)

DEFAULT_MAX_LINEAR_MPS = 0.5
DEFAULT_MAX_ANGULAR_RADPS = math.radians(100.0)


def positive(value: float) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError("Expected a finite positive value")
    return value


def validate_tool(tool: ToolPose) -> None:
    if len(tool.position_xyz) != 3 or len(tool.quaternion_xyzw) != 4:
        raise ValueError("Invalid TCP pose dimensions")
    if not all(math.isfinite(x) for x in (*tool.position_xyz, *tool.quaternion_xyzw)):
        raise ValueError("Non-finite TCP pose")
    if abs(sum(x*x for x in tool.quaternion_xyzw) - 1.0) > 0.002:
        raise ValueError("TCP orientation must be a unit quaternion")


def base_to_tool(vector, q_base_tool):
    """Rotate velocity axes at the SAME TCP origin: R_base_tool.T @ vector."""
    x, y, z, w = q_base_tool
    # Inverse quaternion rotates a planning/base vector into current tool axes.
    x, y, z = -x, -y, -z
    vx, vy, vz = vector
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return (vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx)


def native_vectors(command: MapperCommand, tool: ToolPose):
    """Kortex 0.2.3 + PickNik 0.0.3: TOOL axes, m/s and DEG/s on Twist."""
    validate_tool(tool)
    values = (*command.linear_xyz, *command.angular_xyz)
    if len(values) != 6 or not all(math.isfinite(x) for x in values):
        raise ValueError("Invalid mapper velocity")
    if not command.enabled:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    linear = base_to_tool(command.linear_xyz, tool.quaternion_xyzw)
    angular_rad = base_to_tool(command.angular_xyz, tool.quaternion_xyzw)
    return linear, tuple(math.degrees(x) for x in angular_rad)


class NativeSession:
    """Capture/follow the control point, then shift its Twist to the actual TCP.

    T_tool_tracker_mount is ^tool T_control: the virtual tracker/control frame
    fixed to the robot hand, expressed in the actual TCP frame. Its translation
    is in metres and is never multiplied by the user's displacement scale.
    """

    def __init__(self, calibration: Gen3ViveCalibration, *, motion="both",
                 max_linear=DEFAULT_MAX_LINEAR_MPS, max_angular=DEFAULT_MAX_ANGULAR_RADPS,
                 low_pass_alpha=1.0):
        if motion not in {"rotation", "translation", "both"}:
            raise ValueError("Unknown motion selection")
        mount = calibration.T_tool_tracker_mount
        validate_tool(ToolPose(mount.translation, mount.rotation_xyzw))
        self.motion = motion
        self.calibration = replace(
            calibration,
            max_linear_mps=min(calibration.max_linear_mps, positive(max_linear)),
            max_angular_radps=min(calibration.max_angular_radps, positive(max_angular)),
        )
        # Apply exactly the same gates and gain (1/s) for native and Servo.
        self.mapper = Gen3ViveMapperCore(
            self.calibration, MapperRuntimeConfig(dry_run=False, real_robot=True, low_pass_alpha=low_pass_alpha)
        )

    def control_pose(self, tool: ToolPose) -> ToolPose:
        """^base T_control = ^base T_tool * ^tool T_control."""
        validate_tool(tool)
        mount = self.calibration.T_tool_tracker_mount
        offset = _rotate_vector(tool.quaternion_xyzw, mount.translation)
        control = ToolPose(
            tuple(p + r for p, r in zip(tool.position_xyz, offset)),
            _quat_multiply(tool.quaternion_xyzw, mount.rotation_xyzw),
        )
        validate_tool(control)
        return control

    def _tcp_command(self, command: MapperCommand, tool: ToolPose) -> MapperCommand:
        if not command.enabled:
            return replace(command, linear_xyz=(0.0, 0.0, 0.0), angular_xyz=(0.0, 0.0, 0.0))
        linear = (0.0, 0.0, 0.0) if self.motion == "rotation" else command.linear_xyz
        angular = (0.0, 0.0, 0.0) if self.motion == "translation" else command.angular_xyz
        rx, ry, rz = _rotate_vector(tool.quaternion_xyzw, self.calibration.T_tool_tracker_mount.translation)
        wx, wy, wz = angular
        cross = (wy*rz-wz*ry, wz*rx-wx*rz, wx*ry-wy*rx)
        # Velocity at C = velocity at TCP + omega x (TCP -> C), all in base axes.
        tcp_linear = tuple(v - c for v, c in zip(linear, cross))
        if not all(math.isfinite(v) for v in (*tcp_linear, *angular)):
            raise ValueError("Non-finite control-point Twist")
        # A lever-arm correction can exceed the TCP linear cap. Scale both
        # parts together so a stationary control point stays stationary.
        factor = min(1.0, self.calibration.max_linear_mps / max(math.hypot(*tcp_linear), 1e-30),
                     self.calibration.max_angular_radps / max(math.hypot(*angular), 1e-30))
        return replace(command, linear_xyz=tuple(factor*v for v in tcp_linear),
                       angular_xyz=tuple(factor*w for w in angular))

    def step(self, *, now, pose: ViveTrackerPose | None, tool: ToolPose | None,
             pressed: bool, deadman_fresh=True, feedback_fresh=True,
             controller_ready=True, input_error=None):
        reason = input_error
        if not deadman_fresh:
            reason = "DEADMAN_TIMEOUT"
        elif not feedback_fresh or tool is None:
            reason = "TCP_FEEDBACK_STALE_OR_MISSING"
        elif not controller_ready:
            reason = "CONTROLLER_NOT_READY"
        control = None
        if tool is not None:
            try:
                control = self.control_pose(tool)
            except ValueError:
                reason = "INVALID_TCP_POSE"
        if reason:
            command = self.mapper.force_stop(
                StopReason.INVALID_POSE, deadman_pressed=pressed, now_monotonic_s=now
            )
            return replace(command, stop_reason=reason)
        command = self.mapper.update(
            pose=pose, current_tool_pose=control,
            deadman=DeadmanInput(pressed=pressed, is_physical=True),
            now_monotonic_s=now, real_robot_latch=True,
        )
        try:
            return self._tcp_command(command, tool)
        except ValueError:
            command = self.mapper.force_stop(
                StopReason.INVALID_POSE, deadman_pressed=pressed, now_monotonic_s=now
            )
            return replace(command, stop_reason="INVALID_CONTROL_POINT_TWIST")
