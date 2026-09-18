"""Exercise the actual patched driver/SDK ONLY with Docker --network none."""
from pathlib import Path
import subprocess
import tempfile


def main():
    if {path.name for path in Path("/sys/class/net").iterdir()} != {"lo"}:
        raise SystemExit("Refusing driver test: run in Docker with --network none")
    from ament_index_python.packages import get_package_prefix
    driver_prefix = Path(get_package_prefix("kortex_driver"))
    assert (driver_prefix / "share/kortex_driver/cyclic_failure_guard.version").read_text().strip() == "1"
    root = Path(__file__).resolve().parents[2]
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        (directory / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.14)
project(gen3_kortex_cyclic_runtime)
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
add_executable(cyclic_runtime "''' + str(root / "tests/cpp/test_gen3_kortex_cyclic_runtime.cpp") + '''")
ament_target_dependencies(cyclic_runtime hardware_interface pluginlib rclcpp)
''')
        for command in (["cmake", "-S", str(directory), "-B", str(directory / "build")],
                        ["cmake", "--build", str(directory / "build"), "-j2"]):
            result = subprocess.run(command, text=True, capture_output=True)
            if result.returncode:
                raise RuntimeError(result.stdout + result.stderr)
        subprocess.run([str(directory / "build/cyclic_runtime")], check=True, timeout=15)


if __name__ == "__main__":
    main()
