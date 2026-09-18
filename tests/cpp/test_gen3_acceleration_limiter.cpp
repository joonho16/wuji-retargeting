#include "joint_acceleration_limiter.hpp"
#include <iostream>
#include <limits>
#include <random>

void require(bool condition, const char* message)
{
  if (!condition)
    throw std::runtime_error(message);
}

int main()
{
  constexpr double dt = 0.01;
  const std::vector<double> vmax{ 1.3963, 1.2218, 0.3 };
  const std::vector<double> amax{ 8.6, 1.0, 0.4 };
  gen3_servo::JointAccelerationLimiter limiter(dt, vmax, amax, { true, true, false });
  std::vector<double> q{ 3.13, -3.13, 0.0 }, v(3, 0.0);
  require(limiter.reset(q), "reset failed");
  auto previous_q = q, previous_v = v;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> random(-10.0, 10.0);
  // Includes saturated steps, abrupt reversals, a zero target velocity while
  // other joints still move, and randomized requests well outside the caps.
  for (int tick = 0; tick < 15000; ++tick)
  {
    const double requested = tick < 500 ? 1.5 : tick < 1000 ? -1.5 : tick < 1500 ? 0.0 : random(rng);
    v = { requested, -requested, 0.5 * requested };
    q = { random(rng), random(rng), random(rng) };
    require(limiter.update(q, v), "valid update rejected");
    for (std::size_t i = 0; i < q.size(); ++i)
    {
      const double observed_v = (q[i] - previous_q[i]) / dt;
      require(std::abs(observed_v) <= vmax[i] + 1e-10, "position reference exceeds velocity cap");
      require(std::abs((observed_v - previous_v[i]) / dt) <= amax[i] + 1e-8,
              "position reference exceeds acceleration cap");
      require(std::abs(observed_v - v[i]) < 1e-10, "positions and velocities disagree");
    }
    previous_q = q;
    previous_v = v;
  }
  // Crossing +/- pi must stay in the same revolution, not jump toward zero.
  require(limiter.reset({ 3.13, -3.13, 0.0 }), "reset failed");
  require(std::abs(limiter.unwrap(0, -3.13) - 3.153185307179586) < 1e-12, "positive wrap discontinuity");
  require(std::abs(limiter.unwrap(1, 3.13) + 3.153185307179586) < 1e-12, "negative wrap discontinuity");
  require(limiter.unwrap(2, 3.13) == 3.13, "bounded joint must not unwrap");
  std::vector<double> seed{ -3.15421, 3.2 + 12.566370614359172, 3.4 };
  require(limiter.normalizeIKSeed(seed), "valid IK seed rejected");
  require(seed[0] > 3.0 && seed[0] <= 3.141592653589793, "negative boundary was not normalized for IK");
  require(seed[1] >= -3.141592653589793 && seed[1] < -3.0, "positive boundary was not normalized for IK");
  require(seed[2] == 3.4, "bounded joint IK seed was incorrectly wrapped");
  auto bad_seed = seed;
  bad_seed[1] = std::numeric_limits<double>::infinity();
  require(!limiter.normalizeIKSeed(bad_seed), "non-finite IK seed accepted");
  q = { 3.13, -3.13, 0.0 };
  v = { 1.0, 1.0, 0.2 };
  require(limiter.update(q, v), "restart rejected");
  require(v[0] <= amax[0] * dt + 1e-12, "safety reset retained old velocity");
  const auto saved_q = q;
  v = { 0.0, std::numeric_limits<double>::quiet_NaN(), 0.0 };
  require(!limiter.update(q, v), "NaN was accepted");
  require(q == saved_q, "invalid input partially changed reference");
  require(!limiter.reset({ 0.0 }), "wrong joint count was accepted");
  bool rejected = false;
  try { gen3_servo::JointAccelerationLimiter invalid(dt, { 1.0 }, { 0.0 }, { true }); }
  catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, "zero acceleration limit accepted");
  std::cout << "15000 reference steps, reversals, wraps, safety reset and invalid inputs passed\n";
}
