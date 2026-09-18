"""Resolve Gen3 Servo velocity/acceleration limits before robot launch actions.

The Kortex SDK is imported only in a short-lived query subprocess. This keeps
fake hardware independent of the SDK and bounds even a stalled SDK connection
or cleanup. The subprocess only opens a session and reads ControlConfig limits.
"""

from __future__ import annotations

import copy
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


SOFT_LIMIT_MODES = (
    "ANGULAR_TRAJECTORY",
    "CARTESIAN_TRAJECTORY",
    "ANGULAR_JOYSTICK",
    "CARTESIAN_JOYSTICK",
)


class JointLimitsError(ValueError):
    """A required limit could not be read or validated."""


def _positive(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise JointLimitsError(f"{label} must be a positive finite number")
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise JointLimitsError(f"{label} must be a positive finite number")
    return number


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _unique_mapping(loader: _UniqueKeyLoader, node: Any) -> dict:
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=True)
        if not isinstance(key, str):
            raise JointLimitsError("Joint limit YAML keys must be strings")
        if key in result:
            raise JointLimitsError(f"Duplicate joint limit YAML key: {key}")
        result[key] = loader.construct_object(value_node, deep=True)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping
)


@dataclass(frozen=True)
class JointLimitSettings:
    robot_soft_limit_mode: str
    velocities: dict[str, float]
    accelerations: dict[str, float] = field(default_factory=dict)


def arm_joint_names(dof: int) -> tuple[str, ...]:
    if dof not in (6, 7):
        raise JointLimitsError("Gen3 joint limits require 6 or 7 DOF")
    return tuple(f"joint_{i}" for i in range(1, dof + 1))


def load_settings(path: Path, dof: int) -> JointLimitSettings:
    try:
        raw = yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
    except (OSError, yaml.YAMLError) as exc:
        raise JointLimitsError(f"Cannot load joint limit config: {path}") from exc
    if not isinstance(raw, dict) or set(raw) - {"robot_soft_limit_mode", "joint_limits"}:
        raise JointLimitsError("Expected robot_soft_limit_mode and joint_limits mapping")
    mode = raw.get("robot_soft_limit_mode", "ANGULAR_TRAJECTORY")
    if mode not in SOFT_LIMIT_MODES:
        raise JointLimitsError(f"robot_soft_limit_mode must be one of {SOFT_LIMIT_MODES}")
    entries = raw.get("joint_limits")
    if not isinstance(entries, dict):
        raise JointLimitsError("joint_limits must be a mapping (use {} for no extra caps)")
    names = arm_joint_names(dof)
    velocities, accelerations = {}, {}
    for name, entry in entries.items():
        if name not in names:
            raise JointLimitsError(f"Unknown arm joint for {dof} DOF: {name}")
        allowed = {"has_velocity_limits", "max_velocity", "has_acceleration_limits", "max_acceleration"}
        if not isinstance(entry, dict) or not entry or set(entry) - allowed:
            raise JointLimitsError(f"{name}: expected velocity and/or acceleration limits")
        for kind, destination in (("velocity", velocities), ("acceleration", accelerations)):
            if f"has_{kind}_limits" in entry or f"max_{kind}" in entry:
                if entry.get(f"has_{kind}_limits", True) is not True:
                    raise JointLimitsError(f"{name}: {kind} limits cannot be disabled")
                destination[name] = _positive(entry.get(f"max_{kind}"), f"{name}.max_{kind}")
    return JointLimitSettings(mode, velocities, accelerations)


def model_velocity_limits(
    robot_description: str, planning: dict[str, Any], dof: int
) -> dict[str, float]:
    """Use both URDF and MoveIt caps, so a YAML override cannot raise the URDF cap."""
    try:
        root = ET.fromstring(robot_description)
    except (ET.ParseError, TypeError) as exc:
        raise JointLimitsError("Cannot read the expanded Gen3 URDF") from exc
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    overrides = planning.get("joint_limits", {})
    limits = {}
    for name in arm_joint_names(dof):
        joint = joints.get(name)
        if joint is None or joint.get("type") not in {"revolute", "continuous"}:
            raise JointLimitsError(f"URDF is missing rotary arm joint {name}")
        limit = joint.find("limit")
        try:
            velocity = float(limit.attrib["velocity"]) if limit is not None else None
        except (ValueError, KeyError) as exc:
            raise JointLimitsError(f"Invalid URDF velocity for {name}") from exc
        caps = [_positive(velocity, f"URDF {name}.velocity")]
        entry = overrides.get(name, {})
        if entry.get("has_velocity_limits", False):
            caps.append(_positive(entry.get("max_velocity"), f"MoveIt {name}.max_velocity"))
        limits[name] = min(caps)
    return limits


def _robot_vectors(raw: dict[str, Any], dof: int) -> dict[str, dict[str, float]]:
    result = {}
    for source in ("hard", "soft", "hard_acceleration", "soft_acceleration"):
        values = raw.get(source)
        if not isinstance(values, list) or len(values) != dof:
            raise JointLimitsError(f"Robot {source} limits must contain exactly {dof} joints")
        unit = "deg/s^2" if "acceleration" in source else "deg/s"
        result[source] = {
            name: math.radians(_positive(value, f"Robot {source} {name} ({unit})"))
            for name, value in zip(arm_joint_names(dof), values)
        }
    return result


def resolve_acceleration_limits(
    planning: dict[str, Any], settings: JointLimitSettings,
    robot: dict[str, dict[str, float]] | None, dof: int,
) -> tuple[dict[str, Any], list[str]]:
    """Resolve rad/s² bounds; never silently run without an acceleration bound."""
    result = copy.deepcopy(planning)
    entries = result.setdefault("joint_limits", {})
    messages = []
    for name in arm_joint_names(dof):
        entry = entries.setdefault(name, {})
        sources = {}
        if entry.get("has_acceleration_limits", False):
            sources["model"] = _positive(entry.get("max_acceleration"), f"MoveIt {name}.max_acceleration")
        if robot is not None:
            for source in ("hard", "soft"):
                try:
                    sources[f"robot_{source}"] = _positive(
                        robot[f"{source}_acceleration"][name], f"Robot {source} {name} acceleration"
                    )
                except KeyError as exc:
                    raise JointLimitsError(f"Missing robot {source} acceleration limit for {name}") from exc
        if name in settings.accelerations:
            sources["file"] = _positive(settings.accelerations[name], f"File {name} acceleration")
        if not sources:
            raise JointLimitsError(f"No acceleration limit for {name}; configure max_acceleration")
        effective = min(sources.values())
        entry.update(has_acceleration_limits=True, max_acceleration=effective)
        detail = ", ".join(f"{key}={value:.6g}" for key, value in sources.items())
        messages.append(f"{name}: max_acceleration={effective:.6g} rad/s^2 ({detail})")
    return result, messages


def query_robot_limits(
    *, robot_ip: str, username: str, password: str, mode: str, dof: int,
    timeout_s: float = 15.0,
) -> dict[str, dict[str, float]]:
    if not robot_ip or robot_ip == "0.0.0.0":
        raise JointLimitsError("GEN3_ROBOT_IP must be set for robot limit discovery")
    if not username or not password:
        raise JointLimitsError("Gen3 limit discovery requires a username and password")
    _positive(timeout_s, "GEN3_LIMIT_QUERY_TIMEOUT_S")
    if mode not in SOFT_LIMIT_MODES:
        raise JointLimitsError(f"Unsupported robot soft limit mode: {mode}")
    # Credentials go through stdin, never through process arguments or logs.
    request = dict(robot_ip=robot_ip, username=username, password=password, mode=mode)
    try:
        completed = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), "--query-robot"],
            input=json.dumps(request), text=True, capture_output=True,
            timeout=timeout_s, check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise JointLimitsError(f"Robot limit query timed out after {timeout_s:g}s; Servo will not start") from exc
    if completed.returncode:
        # SDK errors may contain session data. Do not forward arbitrary stderr.
        raise JointLimitsError(
            "Robot limit query failed; check Kortex SDK installation, robot connection, "
            "credentials and soft limit mode. Servo will not start"
        )
    try:
        raw = json.loads(completed.stdout)
    except (ValueError, TypeError) as exc:
        raise JointLimitsError("Invalid response from robot limit query") from exc
    if not isinstance(raw, dict) or raw.get("mode") != mode:
        raise JointLimitsError("Robot limit query returned a different soft limit mode")
    return _robot_vectors(raw, dof)


def resolve_velocity_limits(
    planning: dict[str, Any], model: dict[str, float],
    settings: JointLimitSettings, robot: dict[str, dict[str, float]] | None,
) -> tuple[dict[str, Any], list[str]]:
    """Return new planning parameters and auditable per-joint startup messages."""
    result = copy.deepcopy(planning)
    entries = result.setdefault("joint_limits", {})
    messages = []
    for name, model_limit in model.items():
        sources = {"model": _positive(model_limit, f"Model {name}")}
        if robot is not None:
            for source in ("hard", "soft"):
                try:
                    sources[f"robot_{source}"] = _positive(robot[source][name], f"Robot {source} {name}")
                except KeyError as exc:
                    raise JointLimitsError(f"Missing robot {source} limit for {name}") from exc
        if name in settings.velocities:
            sources["file"] = _positive(settings.velocities[name], f"File {name}")
        effective = min(sources.values())
        entries.setdefault(name, {}).update(has_velocity_limits=True, max_velocity=effective)
        detail = ", ".join(f"{key}={value:.6g}" for key, value in sources.items())
        messages.append(f"{name}: max_velocity={effective:.6g} rad/s ({detail})")
    return result, messages


def _query_kortex(request: dict[str, str]) -> dict[str, Any]:
    # Official Kortex 2.x SDK; no dependency on ROS and no robot writes.
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
    from kortex_api.SessionManager import SessionManager
    from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient
    from kortex_api.autogen.messages import ControlConfig_pb2, Session_pb2

    mode = ControlConfig_pb2.ControlMode.Value(request["mode"])
    transport = TCPTransport()
    router = RouterClient(transport, lambda _error: None)
    session = SessionManager(router)
    options = RouterClientSendOptions()
    options.timeout_ms = 3000
    connected = False
    authenticated = False
    try:
        transport.connect(request["robot_ip"], 10000)
        connected = True
        info = Session_pb2.CreateSessionInfo(
            username=request["username"], password=request["password"],
            session_inactivity_timeout=10000, connection_inactivity_timeout=2000,
        )
        session.CreateSession(info, options=options)
        authenticated = True
        control = ControlConfigClient(router)
        hard = control.GetKinematicHardLimits(options=options)
        soft = control.GetKinematicSoftLimits(
            ControlConfig_pb2.ControlModeInformation(control_mode=mode), options=options
        )
        if soft.control_mode != mode:
            raise JointLimitsError("Robot returned limits for a different control mode")
        return dict(
            mode=request["mode"], hard=list(hard.joint_speed_limits), soft=list(soft.joint_speed_limits),
            hard_acceleration=list(hard.joint_acceleration_limits),
            soft_acceleration=list(soft.joint_acceleration_limits),
        )
    finally:
        try:
            if authenticated:
                session.CloseSession(options=options)
        finally:
            if connected:
                transport.disconnect()


def _query_main() -> int:
    import contextlib

    try:
        request = json.load(sys.stdin)
        # Keep the machine-readable response separate from any SDK output.
        with contextlib.redirect_stdout(sys.stderr):
            result = _query_kortex(request)
        print(json.dumps(result))
    except Exception as exc:
        print(f"Kortex limit query failed ({type(exc).__name__})", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    if sys.argv[1:] != ["--query-robot"]:
        raise SystemExit("This module is used by gen3_vive_servo_stack.py")
    raise SystemExit(_query_main())
