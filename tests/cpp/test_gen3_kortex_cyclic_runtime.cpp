#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  {
    // Deliberately never call on_init: no IP, session or hardware connection.
    // RefreshFeedback on this disconnected SDK must become ERROR, not abort.
    // Resolve the plugin exactly as ros2_control does, including overlay lookup.
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
        "hardware_interface", "hardware_interface::SystemInterface");
    auto driver = loader.createSharedInstance("kortex_driver/KortexMultiInterfaceHardware");
    const rclcpp::Time now(0);
    const auto period = rclcpp::Duration::from_seconds(0.001);
    if (driver->read(now, period) != hardware_interface::return_type::ERROR)
      throw std::runtime_error("Disconnected cyclic read was accepted");
    for (int i = 0; i < 1000; ++i)
      if (driver->write(now, period) != hardware_interface::return_type::ERROR ||
          driver->read(now, period) != hardware_interface::return_type::ERROR)
        throw std::runtime_error("Failed driver resumed cyclic communication");
    if (driver->on_activate(rclcpp_lifecycle::State()) !=
          rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR ||
        driver->perform_command_mode_switch({}, {}) != hardware_interface::return_type::ERROR)
      throw std::runtime_error("Failed driver accepted reactivation");
    // Even shutdown after a lost connection must finish cleanup without throwing.
    driver->on_deactivate(rclcpp_lifecycle::State());
  }
  rclcpp::shutdown();
  std::cout << "Actual Kortex SDK disconnected failure contained; read/write/reactivation stay blocked\n";
}
