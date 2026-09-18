#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gen3_servo
{
// Limits the discrete position reference sent by Servo, not measured hardware
// acceleration. Safety holds deliberately reset the history instead of ramping.
class JointAccelerationLimiter
{
public:
  JointAccelerationLimiter(double period, std::vector<double> velocity_limits,
                           std::vector<double> acceleration_limits, std::vector<bool> continuous)
    : period_(period), vmax_(std::move(velocity_limits)), amax_(std::move(acceleration_limits)),
      continuous_(std::move(continuous))
  {
    if (!positive(period_) || vmax_.empty() || vmax_.size() != amax_.size() ||
        vmax_.size() != continuous_.size())
      throw std::invalid_argument("Invalid Gen3 joint acceleration limiter configuration");
    for (std::size_t i = 0; i < vmax_.size(); ++i)
      if (!positive(vmax_[i]) || !positive(amax_[i]))
        throw std::invalid_argument("Every Gen3 joint needs finite positive velocity and acceleration limits");
  }

  bool reset(const std::vector<double>& positions)
  {
    if (!valid(positions))
      return false;
    position_ = positions;
    velocity_.assign(positions.size(), 0.0);
    return true;
  }

  bool continuous(std::size_t i) const { return continuous_.at(i); }

  static double angleDelta(double target, double reference)
  {
    constexpr double TWO_PI = 6.28318530717958647692;
    return std::remainder(target - reference, TWO_PI);
  }

  // Keep feedback in the same revolution as the published reference. Without
  // this, +pi -> -pi can become a spurious full revolution in a position filter.
  double unwrap(std::size_t i, double measured) const
  {
    return !position_.empty() && continuous_[i] ? position_[i] + angleDelta(measured, position_[i]) : measured;
  }

  // KDL uses [-pi, pi] bounds even for continuous joints. The reference may
  // remain unwrapped, but using it directly as an IK seed can make KDL clip
  // against the opposite boundary and return a non-finite approximate solution.
  bool normalizeIKSeed(std::vector<double>& seed) const
  {
    if (!valid(seed))
      return false;
    for (std::size_t i = 0; i < seed.size(); ++i)
      if (continuous_[i])
        seed[i] = angleDelta(seed[i], 0.0);
    return true;
  }

  bool update(std::vector<double>& target, std::vector<double>& velocity)
  {
    if (validationError(target, velocity))
      return false;
    for (std::size_t i = 0; i < target.size(); ++i)
    {
      const double acceleration_step = amax_[i] * period_;
      // Servo has already converted Twist/JointJog to a velocity respecting
      // singularities, collisions and joint speed bounds. Ramp that velocity;
      // do not chase successive position samples with another position loop.
      velocity_[i] = std::clamp(velocity[i], std::max(-vmax_[i], velocity_[i] - acceleration_step),
                               std::min(vmax_[i], velocity_[i] + acceleration_step));
      position_[i] += velocity_[i] * period_;
      target[i] = position_[i];
    }
    velocity = velocity_;
    return true;
  }

  const char* validationError(const std::vector<double>& target, const std::vector<double>& velocity) const
  {
    if (position_.empty()) return "uninitialized limiter";
    if (target.size() != vmax_.size()) return "wrong position count";
    if (velocity.size() != vmax_.size()) return "wrong velocity count";
    if (!valid(target)) return "non-finite target position";
    if (!valid(velocity)) return "non-finite target velocity";
    if (!valid(position_) || !valid(velocity_)) return "invalid limiter history";
    return nullptr;
  }

private:
  static bool positive(double value) { return std::isfinite(value) && value > 0.0; }
  bool valid(const std::vector<double>& values) const
  {
    return values.size() == vmax_.size() &&
           std::all_of(values.begin(), values.end(), [](double x) { return std::isfinite(x); });
  }
  double period_;
  std::vector<double> vmax_, amax_, position_, velocity_;
  std::vector<bool> continuous_;
};
}  // namespace gen3_servo
