#pragma once
#include <algorithm>
#include <atomic>
#include <cctype>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "BaseClientRpc.h"
#include "kortex_driver/saved_action_lease.hpp"

namespace gen3_kortex
{
// spin_some() and every SDK command run on the driver's existing write thread.
// No competing session, SDK thread, or background action keepalive is created.
class SavedActionBridge
{
  using Base = Kinova::Api::Base::BaseClient;
public:
  SavedActionBridge(Base& base, std::function<void()> disconnect)
    : base_(base), disconnect_(std::move(disconnect)), event_(std::make_shared<std::atomic<uint64_t>>(0))
  {
    node_ = std::make_shared<rclcpp::Node>("gen3_saved_action_bridge",
      rclcpp::NodeOptions().use_global_arguments(false));
    publisher_ = node_->create_publisher<std_msgs::msg::String>("/gen3_vive/saved_action/status", 1);
    subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "/gen3_vive/saved_action/command", 1,
      [this](const std_msgs::msg::String::SharedPtr msg) { lease_.receive(msg->data, monotonic_ns()); });
    executor_.add_node(node_);
  }

  void tick(bool permitted)
  {
    executor_.spin_some();
    const auto now = monotonic_ns();
    if (moving_ && (!permitted || !lease_.held(token_, now))) stop();
    if (moving_)
    {
      const auto event = event_->load();
      const auto kind = uint32_t(event);
      if (uint32_t(event >> 32) == identifier_ &&
          (kind == Kinova::Api::Base::ACTION_END || kind == Kinova::Api::Base::ACTION_ABORT))
      {
        stop();
        state_ = kind == Kinova::Api::Base::ACTION_END ? "complete" : "failed";
      }
    }
    if (!lease_.token().empty() && !lease_.held(now))
    {
      token_ = lease_.token();
      state_ = "stopped";
    }
    if (!moving_ && lease_.claim(now))
    {
      token_ = lease_.token();
      if (!permitted) state_ = "failed";
      else execute(lease_.action());
    }
    if (!token_.empty() && now - published_ >= 20000000LL)
    {
      std_msgs::msg::String msg;
      msg.data = token_ + "\n" + state_;
      publisher_->publish(msg);
      published_ = now;
    }
  }

  void stop()
  {
    if (moving_)
    {
      try { base_.Stop(0, {false, 0, 150}); }
      catch (...) { disconnect_(); throw; }  // Stop failed: kill keepalive, allow firmware watchdog.
      moving_ = false;
    }
    state_ = "stopped";
  }

  void abort_on_fault() noexcept
  {
    if (!moving_) return;
    try { stop(); } catch (...) {}
    try { disconnect_(); } catch (...) {}
  }

private:
  static std::string normalized(std::string value)
  {
    auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    value = value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
    std::transform(value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return std::tolower(c); });
    return value;
  }

  bool still_held()
  {
    executor_.spin_some();
    return lease_.held(token_, monotonic_ns());
  }

  void execute(const std::string& name)
  {
    try
    {
      Kinova::Api::Base::ActionHandle handle;
      unsigned int matches = 0;
      for (auto kind : {Kinova::Api::Base::REACH_JOINT_ANGLES, Kinova::Api::Base::REACH_POSE})
      {
        Kinova::Api::Base::RequestedActionType requested;
        requested.set_action_type(kind);
        auto actions = base_.ReadAllActions(requested, 0, {false, 0, 1000});
        if (!still_held()) { state_ = "stopped"; return; }
        for (const auto& action : actions.action_list())
          if (normalized(action.name()) == normalized(name)) { handle = action.handle(); ++matches; }
      }
      if (matches != 1) throw std::runtime_error("Expected exactly one saved action named " + name);
      if (!subscribed_)
      {
        auto event = event_;
        base_.OnNotificationActionTopic([event](const Kinova::Api::Base::ActionNotification& note) {
          if (note.action_event() == Kinova::Api::Base::ACTION_END ||
              note.action_event() == Kinova::Api::Base::ACTION_ABORT)
            event->store((uint64_t(note.handle().identifier()) << 32) | uint32_t(note.action_event()));
        }, Kinova::Api::Common::NotificationOptions(), 0, {false, 0, 1000});
        subscribed_ = true;
      }
      if (!still_held()) { state_ = "stopped"; return; }
      identifier_ = handle.identifier();
      event_->store(0);
      // Even an execute timeout can mean the robot received the action.
      moving_ = true;
      base_.ExecuteActionFromReference(handle, 0, {false, 0, 150});
      state_ = "running";
      if (!still_held()) stop();
      RCLCPP_INFO(node_->get_logger(), "Saved action '%s' via existing Kortex session", name.c_str());
    }
    catch (const std::exception& error)
    {
      RCLCPP_ERROR(node_->get_logger(), "Saved action '%s' failed: %s", name.c_str(), error.what());
      stop();  // Logs primary failure first; stop failure disconnects then faults the driver.
      state_ = "failed";
    }
  }

  Base& base_;
  std::function<void()> disconnect_;
  std::shared_ptr<std::atomic<uint64_t>> event_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  SavedActionLease lease_;
  std::string token_, state_ = "stopped";
  uint32_t identifier_ = 0;
  int64_t published_ = 0;
  bool moving_ = false, subscribed_ = false;
};
}  // namespace gen3_kortex
