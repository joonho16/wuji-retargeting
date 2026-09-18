#!/usr/bin/env python3
"""Launch a Docker-contained Kinova Gen3 MoveIt + Servo stack for VIVE teleop.

This script intentionally lives outside a ROS package so the compose stack can
run from a plain checked-out repository image.  It launches the binary Kinova
MoveIt config plus a MoveIt Servo node using the repo's Servo YAML override.
"""

from __future__ import annotations

import os
import signal
import sys
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

if __package__:
    from .gen3_joint_limits import (
        JointLimitsError, load_settings, model_velocity_limits,
        query_robot_limits, resolve_velocity_limits, resolve_acceleration_limits,
    )
else:
    from gen3_joint_limits import (
        JointLimitsError, load_settings, model_velocity_limits,
        query_robot_limits, resolve_velocity_limits, resolve_acceleration_limits,
    )


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SERVO_YAML = REPO_ROOT / "example" / "config" / "gen3_vive_moveit_servo.yaml"
DEFAULT_JOINT_LIMITS_YAML = REPO_ROOT / "example" / "config" / "gen3_joint_limits.yaml"


def _env(name: str, default: str) -> str:
    value = os.environ.get(name)
    return default if value is None or value == "" else value


def _bool_env(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _moveit_package_for_dof(dof: str) -> str:
    if dof == "6":
        return "kinova_gen3_6dof_robotiq_2f_85_moveit_config"
    if dof == "7":
        return "kinova_gen3_7dof_robotiq_2f_85_moveit_config"
    raise SystemExit(f"GEN3_DOF must be 6 or 7, got {dof!r}")


def _load_servo_params(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        params = yaml.safe_load(handle) or {}
    if not isinstance(params, dict):
        raise SystemExit(f"Servo config must be a YAML mapping: {path}")
    # Humble uses a nonzero override IN PLACE OF its joint-bound scaling.
    if params.get("override_velocity_scaling_factor", 0.0) != 0.0:
        raise JointLimitsError("Servo override_velocity_scaling_factor must be 0.0 to enforce joint limits")
    params["override_velocity_scaling_factor"] = 0.0
    if params.get("low_latency_mode", False):
        raise JointLimitsError("Gen3 acceleration limiting requires low_latency_mode: false")
    # Accept older operator YAML files without forwarding the removed guard.
    params.pop("gen3_max_joint_tracking_error", None)
    return {"moveit_servo": params}


def _generate_launch_description() -> LaunchDescription:
    dof = _env("GEN3_DOF", "7")
    robot_ip = _env("GEN3_ROBOT_IP", "0.0.0.0")
    use_fake_hardware = "true" if _bool_env("GEN3_USE_FAKE_HARDWARE", True) else "false"
    launch_rviz = "true" if _bool_env("GEN3_LAUNCH_RVIZ", False) else "false"
    robot_type = _env("GEN3_ROBOT_TYPE", "gen3")
    gripper = _env("GEN3_GRIPPER", "robotiq_2f_85")
    gripper_joint_name = _env("GEN3_GRIPPER_JOINT_NAME", "robotiq_85_left_knuckle_joint")
    gripper_max_velocity = _env("GEN3_GRIPPER_MAX_VELOCITY", "100.0")
    gripper_max_force = _env("GEN3_GRIPPER_MAX_FORCE", "100.0")
    use_internal_bus_gripper_comm = _env("GEN3_USE_INTERNAL_BUS_GRIPPER_COMM", "true")
    use_sim_time = _env("GEN3_USE_SIM_TIME", "false")
    servo_yaml = Path(_env("GEN3_SERVO_CONFIG", str(DEFAULT_SERVO_YAML)))
    joint_limits_yaml = Path(_env("GEN3_JOINT_LIMITS_CONFIG", str(DEFAULT_JOINT_LIMITS_YAML)))
    servo_start_delay_s = float(_env("GEN3_SERVO_START_DELAY_S", "8.0"))
    servo_autostart = _bool_env("GEN3_SERVO_AUTOSTART", True)

    moveit_package = _moveit_package_for_dof(dof)
    moveit_share = Path(get_package_share_directory(moveit_package))
    robot_launch = moveit_share / "launch" / "robot.launch.py"
    if not robot_launch.exists():
        raise SystemExit(f"Kinova MoveIt launch file not found: {robot_launch}")
    if not servo_yaml.exists():
        raise SystemExit(f"Servo config file not found: {servo_yaml}")
    servo_params = _load_servo_params(servo_yaml)
    servo_share = Path(get_package_share_directory("moveit_servo"))
    marker = servo_share / "config/gen3_acceleration_limiting.version"
    if not marker.is_file() or marker.read_text().strip() != "4":
        raise JointLimitsError("Gen3 Servo overlay is missing or outdated (requires version 4); rebuild the Gen3 Docker image")
    if use_fake_hardware == "false":
        driver_marker = Path(get_package_share_directory("kortex_driver")) / "cyclic_failure_guard.version"
        if not driver_marker.is_file() or driver_marker.read_text().strip() != "1":
            raise JointLimitsError("Gen3 Kortex cyclic failure guard is missing; rebuild the Gen3 Docker image")
    limit_settings = load_settings(joint_limits_yaml, int(dof))

    # Match the launch arguments consumed by Kinova's generated MoveIt config.
    mappings = {
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "gripper": gripper,
        "gripper_joint_name": gripper_joint_name,
        "dof": dof,
        "gripper_max_velocity": gripper_max_velocity,
        "gripper_max_force": gripper_max_force,
        "use_internal_bus_gripper_comm": use_internal_bus_gripper_comm,
    }

    moveit_config = (
        MoveItConfigsBuilder(robot_type, package_name=moveit_package)
        .robot_description(mappings=mappings)
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    planning = moveit_config.joint_limits["robot_description_planning"]
    model_limits = model_velocity_limits(
        moveit_config.robot_description["robot_description"], planning, int(dof)
    )
    robot_limits = None
    if use_fake_hardware == "false":
        print(f"Reading Gen3 hard/soft velocity and acceleration limits ({limit_settings.robot_soft_limit_mode})", flush=True)
        robot_limits = query_robot_limits(
            robot_ip=robot_ip,
            username=_env("GEN3_LIMIT_QUERY_USERNAME", "admin"),
            password=_env("GEN3_LIMIT_QUERY_PASSWORD", "admin"),
            mode=limit_settings.robot_soft_limit_mode,
            dof=int(dof),
            timeout_s=float(_env("GEN3_LIMIT_QUERY_TIMEOUT_S", "15.0")),
        )
    resolved_planning, limit_messages = resolve_velocity_limits(
        planning, model_limits, limit_settings, robot_limits
    )
    resolved_planning, acceleration_messages = resolve_acceleration_limits(
        resolved_planning, limit_settings, robot_limits, int(dof)
    )
    moveit_config.joint_limits["robot_description_planning"] = resolved_planning
    source = "model + robot + file" if robot_limits is not None else "model + file (fake hardware)"
    print(f"Gen3 Servo joint limits: {source}; config={joint_limits_yaml}", flush=True)
    for message in limit_messages + acceleration_messages:
        print(f"  {message}", flush=True)

    actions: list[Any] = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(robot_launch)),
            launch_arguments={
                "robot_ip": robot_ip,
                "use_fake_hardware": use_fake_hardware,
                "gripper_max_velocity": gripper_max_velocity,
                "gripper_max_force": gripper_max_force,
                "use_internal_bus_gripper_comm": use_internal_bus_gripper_comm,
                "use_sim_time": use_sim_time,
                "launch_rviz": launch_rviz,
            }.items(),
        ),
        TimerAction(
            period=servo_start_delay_s,
            actions=[
                Node(
                    package="moveit_servo",
                    executable="servo_node_main",
                    name="servo_node",
                    output="screen",
                    parameters=[
                        moveit_config.to_dict(),
                        servo_params,
                        {"gen3_acceleration_limiting": True},
                    ],
                )
            ],
        ),
    ]

    if servo_autostart:
        actions.append(
            TimerAction(
                period=servo_start_delay_s + 4.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "bash",
                            "-lc",
                            "for i in $(seq 1 30); do "
                            "ros2 service call /servo_node/start_servo std_srvs/srv/Trigger '{}' && exit 0; "
                            "sleep 1; "
                            "done; "
                            "echo 'failed to call /servo_node/start_servo' >&2; exit 1",
                        ],
                        output="screen",
                    )
                ],
            )
        )

    return LaunchDescription(actions)


def main() -> int:
    # Resolve all limits before starting the driver, Servo, or autostart timer.
    try:
        launch_description = _generate_launch_description()
    except (JointLimitsError, ValueError) as exc:
        print(f"Gen3 Servo startup refused: {exc}", file=sys.stderr)
        return 2
    launch_service = LaunchService(argv=sys.argv[1:])
    launch_service.include_launch_description(launch_description)

    def _shutdown(_signum: int, _frame: Any) -> None:
        launch_service.shutdown(force_sync=True)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    return int(launch_service.run())


if __name__ == "__main__":
    raise SystemExit(main())
