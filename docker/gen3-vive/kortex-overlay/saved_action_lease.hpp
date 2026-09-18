#pragma once
#include <cstdint>
#include <sstream>
#include <string>
#include <time.h>

namespace gen3_kortex
{
inline int64_t monotonic_ns()
{
  timespec value{};
  clock_gettime(CLOCK_MONOTONIC, &value);
  return int64_t(value.tv_sec) * 1000000000LL + value.tv_nsec;
}

// The sender and driver share the Linux monotonic clock across containers.
// Delivery delay must consume the lease, never renew an old request.
class SavedActionLease
{
public:
  bool receive(const std::string& wire, int64_t now)
  {
    std::istringstream stream(wire);
    std::string mode, token, stamp, action, extra;
    if (wire.size() > 1024 || !std::getline(stream, mode) || !std::getline(stream, token) ||
        !std::getline(stream, stamp) || !std::getline(stream, action) || std::getline(stream, extra) ||
        (mode != "H" && mode != "S") || token.empty() || token.size() > 128 || action.empty())
      return false;
    int64_t sent;
    try {
      size_t count = 0;
      sent = std::stoll(stamp, &count);
      if (count != stamp.size()) return false;
    } catch (...) { return false; }
    if (sent <= last_stamp_ || sent > now || now - sent > timeout_ns) return false;
    last_stamp_ = sent;
    token_ = token;
    action_ = action;
    hold_ = mode == "H";
    return true;
  }
  bool held(int64_t now) const
  { return hold_ && now >= last_stamp_ && now - last_stamp_ <= timeout_ns; }
  bool held(const std::string& token, int64_t now) const
  { return token_ == token && held(now); }
  bool claim(int64_t now)
  {
    if (!held(now) || token_ == consumed_) return false;
    consumed_ = token_;
    return true;
  }
  const std::string& token() const { return token_; }
  const std::string& action() const { return action_; }
  static constexpr int64_t timeout_ns = 100000000LL;
private:
  int64_t last_stamp_ = 0;
  bool hold_ = false;
  std::string token_, action_, consumed_;
};
}  // namespace gen3_kortex
