"""Load the actual Twist plugin against RAM interfaces; network-none only."""
from pathlib import Path
import subprocess
import tempfile


def main():
    if {p.name for p in Path('/sys/class/net').iterdir()} != {'lo'}:
        raise SystemExit('Run only in Docker --network none')
    from example.arm_teleop.gen3_native_stack import verify_native_installation
    verify_native_installation()
    root = Path(__file__).resolve().parents[2]
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        (directory / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(gen3_twist_watchdog_runtime)
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
add_executable(watchdog "''' + str(root / 'tests/cpp/test_gen3_twist_watchdog_runtime.cpp') + '''")
ament_target_dependencies(watchdog controller_interface hardware_interface pluginlib rclcpp geometry_msgs)
''')
        for command in (["cmake", "-S", str(directory), "-B", str(directory / "build")],
                        ["cmake", "--build", str(directory / "build"), "-j2"]):
            result = subprocess.run(command, capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError(result.stdout + result.stderr)
        subprocess.run([str(directory / 'build/watchdog')], check=True, timeout=15)


if __name__ == '__main__':
    main()
