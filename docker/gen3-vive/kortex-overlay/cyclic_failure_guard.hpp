#pragma once

#include <exception>

namespace gen3_kortex
{
// A failed cyclic session must not be reused, or restarted by a fault-clear
// command. Keep the first exception, return ERROR to ros2_control, and require
// a new hardware instance. No recovery RPC is attempted from an exception path.
class CyclicFailureGuard
{
public:
  template <typename Result, typename Operation, typename Report>
  Result run(Result error, Operation operation, Report report)
  {
    if (failed_)
      return error;
    try
    {
      return operation();
    }
    catch (const std::exception& exception)
    {
      failed_ = true;
      report(exception.what());
    }
    catch (...)
    {
      failed_ = true;
      report("non-standard cyclic communication exception");
    }
    return error;
  }

  bool failed() const { return failed_; }

private:
  bool failed_ = false;
};
}  // namespace gen3_kortex
