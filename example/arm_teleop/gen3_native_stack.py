"""Arm-only native Kortex stack for existing Gen3/MANUS launcher profiles."""
import argparse
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET



def verify_native_installation():
    from ament_index_python.packages import get_package_share_directory
    for package, expected in (("kortex_driver", "0.2.3"), ("picknik_twist_controller", "0.0.3")):
        share = Path(get_package_share_directory(package))
        actual = ET.parse(share / "package.xml").getroot().findtext("version")
        if actual != expected:
            raise RuntimeError(f"Recheck native frame/units for {package} {actual}; tested version is {expected}")
    for package, name in (("kortex_driver", "cyclic_failure_guard.version"),
                          ("kortex_driver", "native_safety.version"),
                          ("kortex_driver", "saved_action_bridge.version"),
                          ("picknik_twist_controller", "command_watchdog.version")):
        marker = Path(get_package_share_directory(package)) / name
        if not marker.is_file() or marker.read_text().strip() != "1":
            raise RuntimeError(f"Missing {name}; rebuild the Gen3 Docker image before native control")


def native_launch_command(robot_ip, dof=7):
    if not robot_ip or robot_ip in {"0.0.0.0", "127.0.0.1"}:
        raise ValueError("Set GEN3_ROBOT_IP or --robot-ip to the intended robot")
    return [
        "ros2", "launch", "kortex_bringup", "kortex_control.launch.py",
        "robot_type:=gen3", f"dof:={dof}", f"robot_ip:={robot_ip}",
        "use_fake_hardware:=false", "launch_rviz:=false",
        "robot_controller:=twist_controller",
        # The upstream launcher also starts robot_pos_controller INACTIVE.
        # Override its default (twist_controller) to avoid spawning it twice.
        "robot_pos_controller:=joint_trajectory_controller",
        # The launch default is no gripper. Do not load an absent external
        # Robotiq plugin when internal-bus communication is disabled.
        "gripper_joint_name:=robotiq_85_left_knuckle_joint",
        # Wuji hand is controlled separately; no internal-bus gripper commands.
        "use_internal_bus_gripper_comm:=false",
    ]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default=os.environ.get("GEN3_ROBOT_IP"))
    parser.add_argument("--dof", type=int, choices=(6, 7), default=int(os.environ.get("GEN3_DOF", "7")))
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    command = native_launch_command(args.robot_ip, args.dof)
    if args.check:
        print(json.dumps({"command": command, "connects": False}, indent=2))
        return 0
    verify_native_installation()
    print("Starting native Gen3: controller watchdog 100ms, session watchdog at most 500ms. "
          "Kortex startup clears faults and changes servoing mode.", flush=True)
    os.execvp(command[0], command)


if __name__ == "__main__":
    raise SystemExit(main())
