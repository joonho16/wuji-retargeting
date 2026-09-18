"""Steady-clock command watchdog for the pinned PickNik 0.0.3 controller."""
from pathlib import Path
import sys


def once(text, before, after):
    if text.count(before) != 1:
        raise RuntimeError(f"Pinned Twist source mismatch: {before[:100]!r}")
    return text.replace(before, after, 1)


def apply(root):
    header = root / 'include/picknik_twist_controller/picknik_twist_controller.hpp'
    source = root / 'src/picknik_twist_controller.cpp'
    h = once(header.read_text(), '#include <memory>', '#include <memory>\n#include <chrono>\n#include <array>')
    h = once(h, '  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;', '''  struct TimedCommand
  {
    std::array<double, 6> values;
    std::chrono::steady_clock::time_point received;
  };
  realtime_tools::RealtimeBuffer<std::shared_ptr<TimedCommand>> rt_command_ptr_;
  double command_timeout_s_ = 0.1;
  bool awaiting_zero_ = true;
  std::chrono::steady_clock::time_point activated_;
  void zero_commands()
  {
    for (auto & interface : command_interfaces_) interface.set_value(0.0);
  }''')
    c = once(source.read_text(), '#include <memory>', '#include <memory>\n#include <cmath>\n#include <algorithm>')
    c = once(c, '    auto_declare<std::string>("joint", "");',
             '    auto_declare<std::string>("joint", "");\n    auto_declare<double>("command_timeout", 0.1);')
    c = once(c, '  joint_name_ = get_node()->get_parameter("joint").as_string();', '''  command_timeout_s_ = get_node()->get_parameter("command_timeout").as_double();
  if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0 || command_timeout_s_ > 1.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "command_timeout must be finite and in (0, 1] seconds");
    return CallbackReturn::ERROR;
  }
  joint_name_ = get_node()->get_parameter("joint").as_string();''')
    c = once(c, '''    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });''', '''    "~/commands", rclcpp::QoS(1),
    [this](const CmdType::SharedPtr msg)
    {
      auto command = std::make_shared<TimedCommand>();
      command->received = std::chrono::steady_clock::now();
      command->values = {msg->linear.x, msg->linear.y, msg->linear.z,
                         msg->angular.x, msg->angular.y, msg->angular.z};
      rt_command_ptr_.writeFromNonRT(command);
    });''')
    start = 'CallbackReturn PicknikTwistController::on_activate('
    end = '\n}  // namespace picknik_twist_controller'
    old = c[c.index(start):c.index(end)]
    c = once(c, old, '''CallbackReturn PicknikTwistController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  zero_commands();
  activated_ = std::chrono::steady_clock::now();
  awaiting_zero_ = true;
  rt_command_ptr_.writeFromNonRT(nullptr);
  return command_interfaces_.size() == 6 ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn PicknikTwistController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  zero_commands();
  awaiting_zero_ = true;
  rt_command_ptr_.writeFromNonRT(nullptr);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type PicknikTwistController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (command_interfaces_.size() != 6)
  {
    zero_commands();
    return controller_interface::return_type::ERROR;
  }
  const auto command = *rt_command_ptr_.readFromRT();
  const auto now = std::chrono::steady_clock::now();
  if (!command || command->received < activated_ ||
      std::chrono::duration<double>(now - command->received).count() > command_timeout_s_ ||
      !std::all_of(command->values.begin(), command->values.end(),
                   [](double value) { return std::isfinite(value); }))
  {
    // A publisher crash/DDS outage must not keep the last command alive.
    // Require a fresh zero command before accepting motion after any timeout.
    awaiting_zero_ = true;
    zero_commands();
    return controller_interface::return_type::OK;
  }
  const bool zero = std::all_of(command->values.begin(), command->values.end(),
                                [](double value) { return value == 0.0; });
  if (zero) awaiting_zero_ = false;
  for (std::size_t i = 0; i < 6; ++i)
    command_interfaces_[i].set_value(awaiting_zero_ ? 0.0 : command->values[i]);
  return controller_interface::return_type::OK;
}
''')
    cmake = root / 'CMakeLists.txt'
    cmake.write_text(once(cmake.read_text(), 'ament_package()',
                         'install(FILES command_watchdog.version DESTINATION share/${PROJECT_NAME})\nament_package()'))
    header.write_text(h)
    source.write_text(c)
    (root / 'command_watchdog.version').write_text('1\n')


if __name__ == '__main__':
    apply(Path(sys.argv[1]))
