// Exercise the installed controller plugin with real DDS and borrowed RAM
// command interfaces. No hardware driver/session is loaded by this test.
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using Callback = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  {
    pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
        "controller_interface", "controller_interface::ControllerInterface");
    auto controller = loader.createSharedInstance("picknik_twist_controller/PicknikTwistController");
    if (controller->init("twist_controller") != controller_interface::return_type::OK)
      throw std::runtime_error("init failed");
    const std::vector<std::string> axes{"linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"};
    controller->get_node()->set_parameters({rclcpp::Parameter("joint", "tcp"),
                                            rclcpp::Parameter("interface_names", axes)});
    if (controller->on_configure(rclcpp_lifecycle::State()) != Callback::SUCCESS)
      throw std::runtime_error("configure failed");
    std::array<double, 6> values;
    values.fill(99.0);
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (std::size_t i=0; i<6; ++i) interfaces.emplace_back("tcp", axes[i], &values[i]);
    std::vector<hardware_interface::LoanedCommandInterface> loans;
    for (auto& interface : interfaces) loans.emplace_back(interface);
    controller->assign_interfaces(std::move(loans), {});
    if (controller->on_activate(rclcpp_lifecycle::State()) != Callback::SUCCESS)
      throw std::runtime_error("activate failed");
    auto check_zero = [&]() {
      for (double value : values) if (value != 0.0) throw std::runtime_error("Expected zero interfaces");
    };
    check_zero();
    auto node = std::make_shared<rclcpp::Node>("twist_test_publisher");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/twist_controller/commands", 1);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller->get_node()->get_node_base_interface());
    executor.add_node(node);
    // Intentionally freeze ROS/update time: the watchdog must use a steady clock.
    auto tick = [&]() {
      executor.spin_some();
      controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.001));
    };
    auto send = [&](double x, double rz) {
      geometry_msgs::msg::Twist msg;
      msg.linear.x=x; msg.angular.z=rz;
      for (int i=0; i<8; ++i) { publisher->publish(msg); tick(); std::this_thread::sleep_for(5ms); }
      tick();
    };
    for (int i=0; i<100 && publisher->get_subscription_count()==0; ++i)
      { tick(); std::this_thread::sleep_for(10ms); }
    send(.04, 10.0); check_zero();  // Startup requires a fresh zero first.
    send(0., 0.);
    send(.04, 10.0);
    if (values[0] != .04 || values[5] != 10.0) throw std::runtime_error("Fresh command not forwarded");
    // Remove the publisher without a final zero, as on process death/DDS loss.
    publisher.reset();
    const auto start = std::chrono::steady_clock::now();
    while (values[0] != 0.0 && std::chrono::steady_clock::now()-start < 200ms)
      { tick(); std::this_thread::sleep_for(1ms); }
    check_zero();
    const auto stopped_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now()-start).count();
    publisher = node->create_publisher<geometry_msgs::msg::Twist>("/twist_controller/commands", 1);
    send(.04, 10.0); check_zero();  // Nonzero reconnect cannot rearm.
    send(0., 0.); send(.04, 10.0);
    if (values[0] != .04) throw std::runtime_error("Zero rearm failed");
    send(std::numeric_limits<double>::quiet_NaN(), 10.); check_zero();
    send(.04, 10.0); check_zero();
    send(0., 0.); send(.04, 10.0);
    controller->on_deactivate(rclcpp_lifecycle::State()); check_zero();
    controller->on_activate(rclcpp_lifecycle::State()); tick(); check_zero();
    controller->on_deactivate(rclcpp_lifecycle::State());
    controller->release_interfaces();
    std::cout << "Native controller DDS loss stop in " << stopped_ms
              << " ms; frozen clock, zero rearm, NaN and lifecycle stops passed\n";
  }
  rclcpp::shutdown();
}
