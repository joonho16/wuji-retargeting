"""Patch only the pinned Humble Servo package; fail on any upstream mismatch."""
from pathlib import Path
import shutil
import sys


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise RuntimeError(f"Pinned Servo source mismatch: {before[:100]!r}")
    return text.replace(before, after, 1)


def apply(root):
    header = root / "include/moveit_servo/servo_calcs.h"
    source = root / "src/servo_calcs.cpp"
    h = header.read_text()
    h = replace_once(h, "#include <atomic>", "#include <atomic>\n#include <moveit_servo/joint_acceleration_limiter.hpp>")
    h = replace_once(h, "  // Smoothing algorithm (loads a plugin)",
                     "  std::unique_ptr<gen3_servo::JointAccelerationLimiter> acceleration_limiter_;\n\n"
                     "  // Smoothing algorithm (loads a plugin)")
    c = source.read_text()
    c = replace_once(c, "#include <cassert>", "#include <cassert>\n#include <moveit/robot_model/revolute_joint_model.h>")
    c = replace_once(c, "  // Load the smoothing plugin", '''  // Opt-in Gen3 reference acceleration limits. Keep the original behavior for
  // other users of this Humble overlay.
  bool limit_acceleration = false;
  if (!node_->has_parameter("gen3_acceleration_limiting"))
    node_->declare_parameter("gen3_acceleration_limiting", false);
  node_->get_parameter("gen3_acceleration_limiting", limit_acceleration);
  if (limit_acceleration)
  {
    if (parameters_->low_latency_mode)
      throw std::invalid_argument("Gen3 acceleration limiting requires a fixed Servo publish_period");
    std::vector<double> velocity, acceleration;
    std::vector<bool> continuous;
    for (const auto* joint : joint_model_group_->getActiveJointModels())
    {
      if (joint->getVariableCount() != 1)
        throw std::invalid_argument("Gen3 acceleration limiting requires single-variable joints");
      const auto& bounds = joint->getVariableBounds(joint->getName());
      if (!bounds.velocity_bounded_ || !bounds.acceleration_bounded_)
        throw std::invalid_argument("Gen3 Servo requires velocity and acceleration bounds on every joint");
      velocity.push_back(std::min(bounds.max_velocity_, -bounds.min_velocity_));
      acceleration.push_back(std::min(bounds.max_acceleration_, -bounds.min_acceleration_));
      const auto* revolute = dynamic_cast<const moveit::core::RevoluteJointModel*>(joint);
      continuous.push_back(revolute && revolute->isContinuous());
    }
    acceleration_limiter_ = std::make_unique<gen3_servo::JointAccelerationLimiter>(
        parameters_->publish_period, velocity, acceleration, continuous);
    RCLCPP_INFO(LOGGER, "Gen3 per-joint reference acceleration limiting enabled (Butterworth bypassed)");
  }
  // Load the smoothing plugin''')
    c = replace_once(c, "        delta_theta_.coeffRef(i) = solution.at(i) - internal_joint_state_.position.at(i);",
                    '''        delta_theta_.coeffRef(i) =
            acceleration_limiter_ && acceleration_limiter_->continuous(i) ?
            gen3_servo::JointAccelerationLimiter::angleDelta(solution.at(i), internal_joint_state_.position.at(i)) :
            solution.at(i) - internal_joint_state_.position.at(i);''')
    c = replace_once(c,
                    "    if (ik_solver_->searchPositionIK(next_pose, internal_joint_state_.position, parameters_->publish_period / 2.0,",
                    '''    auto ik_seed = internal_joint_state_.position;
    if (acceleration_limiter_ && !acceleration_limiter_->normalizeIKSeed(ik_seed))
    {
      RCLCPP_ERROR_THROTTLE(LOGGER, *node_->get_clock(), 1000, "Gen3 Servo rejected non-finite IK feedback");
      return false;
    }
    if (ik_solver_->searchPositionIK(next_pose, ik_seed, parameters_->publish_period / 2.0,''')
    c = replace_once(c, "      // find the difference in joint positions that will get us to the desired pose", '''      if (acceleration_limiter_ &&
          (solution.size() != num_joints_ ||
           !std::all_of(solution.begin(), solution.end(), [](double value) { return std::isfinite(value); })))
      {
        // Approximate KDL success does not guarantee a finite solution. Keep
        // the last finite reference; never forward NaN or advance the integrator.
        RCLCPP_ERROR_THROTTLE(LOGGER, *node_->get_clock(), 1000,
                             "Gen3 Servo rejected a non-finite IK solution; holding last reference");
        joint_trajectory = *last_sent_command_;
        for (auto& point : joint_trajectory.points)
        {
          std::fill(point.velocities.begin(), point.velocities.end(), 0.0);
          std::fill(point.accelerations.begin(), point.accelerations.end(), 0.0);
        }
        if (!joint_trajectory.points.empty())
          acceleration_limiter_->reset(joint_trajectory.points.back().positions);
        updated_filters_ = true;
        return true;
      }
      // find the difference in joint positions that will get us to the desired pose''')
    c = replace_once(c, "  smoother_->doSmoothing(joint_state.position);",
                    "  if (!acceleration_limiter_)\n    smoother_->doSmoothing(joint_state.position);")
    c = replace_once(c, "  // Enforce SRDF position limits, might halt if needed, set prev_vel to 0", '''  if (acceleration_limiter_)
  {
    if (collision_scale == 0.0 || singularity_halt)
    {
      // Safety stops take precedence over acceleration bounds.
      internal_joint_state_ = original_joint_state_;
      internal_joint_state_.velocity.assign(num_joints_, 0.0);
      acceleration_limiter_->reset(internal_joint_state_.position);
    }
    else if (!acceleration_limiter_->update(internal_joint_state_.position, internal_joint_state_.velocity))
    {
      std::ostringstream diagnostic;
      diagnostic << "Gen3 acceleration limiter rejected an invalid joint reference: "
                 << acceleration_limiter_->validationError(internal_joint_state_.position, internal_joint_state_.velocity);
      for (std::size_t i = 0; i < internal_joint_state_.position.size(); ++i)
      {
        diagnostic << " [index=" << i << " target=" << internal_joint_state_.position[i];
        if (i < internal_joint_state_.velocity.size())
          diagnostic << " velocity=" << internal_joint_state_.velocity[i];
        diagnostic << "]";
      }
      RCLCPP_ERROR_STREAM_THROTTLE(LOGGER, *node_->get_clock(), 1000, diagnostic.str());
      return false;
    }
  }
  // Enforce SRDF position limits, might halt if needed, set prev_vel to 0''')
    c = replace_once(c, "  // Apply collision scaling", '''  const bool singularity_halt = status_ == StatusCode::HALT_FOR_SINGULARITY;
  // Apply collision scaling''')
    c = replace_once(c, "  // compose outgoing message", '''  if (acceleration_limiter_ && !joints_to_halt.empty())
    acceleration_limiter_->reset(internal_joint_state_.position);
  // compose outgoing message''')
    c = replace_once(c, "  smoother_->reset(joint_state.position);", '''  if (acceleration_limiter_)
    acceleration_limiter_->reset(joint_state.position);
  smoother_->reset(joint_state.position);''')
    c = replace_once(c, "  smoother_->doSmoothing(joint_trajectory.points[0].positions);", '''  if (acceleration_limiter_)
    acceleration_limiter_->reset(joint_trajectory.points[0].positions);
  else
    smoother_->doSmoothing(joint_trajectory.points[0].positions);''')
    c = replace_once(c, "  // Cache the original joints in case they need to be reset", '''  if (acceleration_limiter_)
    for (std::size_t i = 0; i < internal_joint_state_.position.size(); ++i)
      internal_joint_state_.position[i] = acceleration_limiter_->unwrap(i, internal_joint_state_.position[i]);
  // Cache the original joints in case they need to be reset''')
    c = replace_once(c, "  // Update from latest state", '''  if (acceleration_limiter_)
  {
    const auto feedback_age = node_->now() - planning_scene_monitor_->getStateMonitor()->getCurrentStateTime();
    if (feedback_age.seconds() < 0.0 || feedback_age.seconds() > parameters_->incoming_command_timeout)
    {
      // Never integrate an advancing reference if robot feedback has stopped.
      resetLowPassFilters(original_joint_state_);
      status_ = StatusCode::INVALID;
      RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 1000,
                           "Gen3 Servo holding: joint feedback is stale");
      return;
    }
  }
  // Update from latest state''')
    c = replace_once(c, "    // Joint trajectory is not populated with anything, so set it to the last positions and 0 velocity", '''    // Zero/deadman commands retain the existing immediate hold semantics.
    if (acceleration_limiter_ && !last_sent_command_->points.empty())
      acceleration_limiter_->reset(last_sent_command_->points.back().positions);
    // Joint trajectory is not populated with anything, so set it to the last positions and 0 velocity''')
    c = replace_once(c, "          joints_to_halt.push_back(joint);", '''          if (acceleration_limiter_)
          {
            RCLCPP_WARN_STREAM_THROTTLE(LOGGER, *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
                "Gen3 position limit: " << joint->getName()
                << " measured=" << original_joint_state_.position.at(joint_idx)
                << " reference=" << joint_angle
                << " velocity=" << joint_state.velocity.at(joint_idx)
                << " lower=" << limits[0].min_position << " upper=" << limits[0].max_position
                << " margin=" << parameters_->joint_limit_margin << " (rad, rad/s)");
          }
          joints_to_halt.push_back(joint);''')
    # Commit only after every expected source anchor was found.
    header.write_text(h)
    source.write_text(c)
    shutil.copyfile(Path(__file__).with_name("joint_acceleration_limiter.hpp"),
                    header.with_name("joint_acceleration_limiter.hpp"))
    marker = root / "config/gen3_acceleration_limiting.version"
    marker.write_text("4\n")


if __name__ == "__main__":
    apply(Path(sys.argv[1]))
