"""Fail-closed cyclic error handling for the pinned Kortex 0.2.3 driver."""
from pathlib import Path
import shutil
import sys
import textwrap


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise RuntimeError(f"Pinned Kortex source mismatch: {before[:100]!r}")
    return text.replace(before, after, 1)


def guard_function(source, start, end, phase):
    body = source.split(start, 1)[1].split(end, 1)[0]
    wrapped = '''
  return cyclic_failure_guard_.run(return_type::ERROR, [&]() -> return_type
  {BODY
  }, [&](const char* message)
  {
    if (saved_action_bridge_) saved_action_bridge_->abort_on_fault();
    in_fault_ = 1.0;
    RCLCPP_ERROR(LOGGER, "Gen3 Kortex cyclic PHASE failed: %s; communication latched, restart required. "
                 "Last feedback: base_fault_a=%u base_fault_b=%u active_state=%d",
                 message, feedback_.base().fault_bank_a(), feedback_.base().fault_bank_b(),
                 static_cast<int>(feedback_.base().active_state()));
  });
'''.replace("BODY", textwrap.indent(body, "  ")).replace("PHASE", phase)
    return replace_once(source, start + body + end, start + wrapped + end)


def apply(root):
    header = root / "include/kortex_driver/hardware_interface.hpp"
    source = root / "src/hardware_interface.cpp"
    h = replace_once(header.read_text(), '#include <atomic>',
                     '#include <atomic>\n#include "kortex_driver/cyclic_failure_guard.hpp"\n#include "kortex_driver/saved_action_bridge.hpp"')
    h = replace_once(h, "private:\n", "private:\n  gen3_kortex::CyclicFailureGuard cyclic_failure_guard_;\n  std::unique_ptr<gen3_kortex::SavedActionBridge> saved_action_bridge_;\n")
    c = source.read_text()
    # Both pointers borrow children owned by the surrounding protobuf messages.
    # Deleting them here makes the parent destructors free them a second time.
    c = replace_once(c, '  delete k_api_twist_;\n  delete gripper_motor_command_;',
                    '  k_api_twist_ = nullptr;\n  gripper_motor_command_ = nullptr;')
    # Also stop motion if the entire PC/controller process or network disappears.
    # The firmware owns this session inactivity watchdog, unlike Twist.duration
    # (documented by Kinova as unimplemented).
    c = replace_once(c, '  if (connection_inactivity_timeout <= 0)',
                    '  connection_inactivity_timeout = std::min(connection_inactivity_timeout, 500);\n  if (connection_inactivity_timeout <= 0)')
    c = replace_once(c, '#include <chrono>', '#include <chrono>\n#include <algorithm>')
    # Arm-only/Wuji setup must not read a nonexistent internal gripper motor.
    start = '  // Initialize gripper\n'
    end = '  // Send a first frame\n'
    body = c.split(start, 1)[1].split(end, 1)[0]
    wrapped = '''  // Commands are protobuf-owned; do not retain old activation entries.
  base_command_.clear_interconnect();
  gripper_motor_command_ = nullptr;
  if (use_internal_bus_gripper_comm_)
  {
    if (base_feedback.interconnect().gripper_feedback().motor_size() == 0)
    {
      RCLCPP_ERROR(LOGGER, "Internal gripper enabled but no motor feedback available");
      return CallbackReturn::ERROR;
    }
BODY  }

'''.replace('BODY', textwrap.indent(body, '  '))
    c = replace_once(c, start + body + end, wrapped + end)
    c = replace_once(c, '  // Add each actuator to the base_command_ and set the command to its current position',
                    '  base_command_.clear_actuators();\n  // Add each actuator to the base_command_ and set the command to its current position')
    # A controller deactivation must send a stop before dropping its interface.
    c = replace_once(c, '  if (stop_twist_controller_)\n  {\n    twist_controller_running_ = false;',
                    '''  if (stop_twist_controller_)
  {
    twist_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    base_.Stop(0, {false, 0, 250});
    twist_controller_running_ = false;''')
    # One action bridge uses the exact TCP session that already controls Twist.
    c = replace_once(c, '  // initialize kortex api twist commandd',
                    '  saved_action_bridge_ = std::make_unique<gen3_kortex::SavedActionBridge>(base_, [this]() { transport_tcp_.disconnect(); });\n\n  // initialize kortex api twist commandd')
    c = replace_once(c, '  if (block_write)\n', '''  if (saved_action_bridge_)
    saved_action_bridge_->tick(!block_write && in_fault_ == 0.0 &&
      !twist_controller_running_ && !joint_based_controller_running_ &&
      arm_mode_ == k_api::Base::SINGLE_LEVEL_SERVOING);
  if (block_write)
''')
    # An independent controller activation must stop an action before mode changes.
    c = replace_once(c, '  if (stop_joint_based_controller_)\n',
                    '  if (saved_action_bridge_) saved_action_bridge_->stop();\n  if (stop_joint_based_controller_)\n')
    # Keep action stop checks bounded even if cyclic feedback disappears.
    # Cleanup must complete even if an RPC fails after a cable/robot disconnect.
    start = '  auto servoing_mode = k_api::Base::ServoingModeInformation();\n  // Set back'
    end = '  // memory handling\n'
    body = c.split(start, 1)[1].split(end, 1)[0]
    cleanup = '''  auto cleanup = [&](const char* phase, auto action)
  {
    try { action(); }
    catch (const std::exception& ex) { RCLCPP_WARN(LOGGER, "Shutdown %s: %s", phase, ex.what()); }
    catch (...) { RCLCPP_WARN(LOGGER, "Shutdown %s failed", phase); }
  };
  cleanup("saved action", [&]() { if (saved_action_bridge_) saved_action_bridge_->stop(); });
  cleanup("single-level mode", [&]() {
    auto mode = k_api::Base::ServoingModeInformation();
    mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base_.SetServoingMode(mode, 0, {false, 0, 250});
  });
  cleanup("stop", [&]() { base_.Stop(0, {false, 0, 250}); });
  cleanup("TCP session", [&]() { session_manager_.CloseSession(); });
  cleanup("UDP session", [&]() { session_manager_real_time_.CloseSession(); });
  cleanup("TCP router", [&]() { router_tcp_.SetActivationStatus(false); });
  cleanup("TCP transport", [&]() { transport_tcp_.disconnect(); });
  cleanup("UDP router", [&]() { router_udp_realtime_.SetActivationStatus(false); });
  cleanup("UDP transport", [&]() { transport_udp_realtime_.disconnect(); });

'''
    c = replace_once(c, start + body + end, cleanup + end)
    # Avoid forcing noisy DEBUG output on the 1 kHz cyclic thread.
    c = replace_once(c, '"Setting severity threshold to DEBUG"', '"Setting severity threshold to INFO"')
    c = replace_once(c, "RCUTILS_LOG_SEVERITY_DEBUG", "RCUTILS_LOG_SEVERITY_INFO")
    c = guard_function(c,
        "return_type KortexMultiInterfaceHardware::read(\n"
        "  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)\n{",
        "}\n\nvoid KortexMultiInterfaceHardware::readGripperPosition()", "read")
    c = guard_function(c,
        "return_type KortexMultiInterfaceHardware::write(\n"
        "  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)\n{",
        "}\n\nvoid KortexMultiInterfaceHardware::prepareCommands()", "write")
    start = "void KortexMultiInterfaceHardware::sendJointCommands()\n{"
    end = "}\n\nvoid KortexMultiInterfaceHardware::incrementId()"
    original = c.split(start, 1)[1].split(end, 1)[0]
    if original.count("    feedback_ = base_cyclic_.RefreshFeedback();") != 4:
        raise RuntimeError("Pinned Kortex recovery paths mismatch")
    # Each handler logs the original exception before handing it to the cyclic
    # guard. A second RPC could fail and previously escaped these handlers.
    guarded = original.replace("    feedback_ = base_cyclic_.RefreshFeedback();\n", "")
    for anchor in (
        "                k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));",
        '    RCLCPP_ERROR_STREAM(LOGGER, "Runtime error: " << ex_runtime.what());',
        '    RCLCPP_ERROR_STREAM(LOGGER, "Future error: " << ex_future.what());',
        '    RCLCPP_ERROR_STREAM(LOGGER, "Standard exception: " << ex_std.what());',
    ):
        guarded = replace_once(guarded, anchor, anchor + "\n    throw;")
    c = replace_once(c, start + original + end, start + guarded + end)
    c = replace_once(c, '  RCLCPP_INFO(LOGGER, "Activating KortexMultiInterfaceHardware...");',
                    '''  if (cyclic_failure_guard_.failed())
    return CallbackReturn::ERROR;
  RCLCPP_INFO(LOGGER, "Activating KortexMultiInterfaceHardware...");''')
    c = replace_once(c, "  if (stop_joint_based_controller_)\n", '''  if (cyclic_failure_guard_.failed())
    return return_type::ERROR;
  if (stop_joint_based_controller_)
''')
    # Stop/mode-switch RPCs can also fail after a disconnect. Never let these
    # escape into controller_manager; preserve the same restart-required latch.
    c = guard_function(c,
        "return_type KortexMultiInterfaceHardware::perform_command_mode_switch(\n"
        "  const vector<std::string> & /*start_interfaces*/, const vector<std::string> & /*stop_interfaces*/)\n{",
        "}\n\nCallbackReturn KortexMultiInterfaceHardware::on_activate(", "switch")
    c = c.replace('base_cyclic_.RefreshFeedback()', 'base_cyclic_.RefreshFeedback(0, {false, 0, 100})')
    cmake = root / "CMakeLists.txt"
    build = replace_once(cmake.read_text(), "ament_package()",
                         'install(FILES cyclic_failure_guard.version native_safety.version saved_action_bridge.version DESTINATION share/${PROJECT_NAME})\nament_package()')
    build = replace_once(build, 'find_package(rclcpp REQUIRED)',
                         'find_package(rclcpp REQUIRED)\nfind_package(std_msgs REQUIRED)')
    build = build.replace('  rclcpp\n', '  rclcpp\n  std_msgs\n')
    package = root / "package.xml"
    package.write_text(replace_once(package.read_text(), '<depend>rclcpp</depend>',
                                   '<depend>rclcpp</depend>\n  <depend>std_msgs</depend>'))
    header.write_text(h)
    source.write_text(c)
    cmake.write_text(build)
    shutil.copyfile(Path(__file__).with_name("cyclic_failure_guard.hpp"), header.with_name("cyclic_failure_guard.hpp"))
    for name in ("saved_action_bridge.hpp", "saved_action_lease.hpp"):
        shutil.copyfile(Path(__file__).with_name(name), header.with_name(name))
    (root / "saved_action_bridge.version").write_text("1\n")
    (root / "cyclic_failure_guard.version").write_text("1\n")
    (root / "native_safety.version").write_text("1\n")


if __name__ == "__main__":
    apply(Path(sys.argv[1]))
