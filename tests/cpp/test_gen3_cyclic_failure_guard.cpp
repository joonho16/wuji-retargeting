#include "cyclic_failure_guard.hpp"
#include <future>
#include <iostream>
#include <stdexcept>
#include <string>

enum class Result { OK, ERROR };

void require(bool condition, const char* message)
{
  if (!condition)
    throw std::runtime_error(message);
}

int main()
{
  for (int kind = 0; kind < 3; ++kind)
  {
    gen3_kortex::CyclicFailureGuard guard;
    int calls = 0, reports = 0;
    std::string cause;
    auto report = [&](const char* error) { ++reports; cause = error; };
    auto healthy = [&]() { ++calls; return Result::OK; };
    require(guard.run(Result::ERROR, healthy, report) == Result::OK, "healthy operation failed");
    auto failed = [&]() -> Result {
      ++calls;
      if (kind == 0) throw std::runtime_error("timeout detected: BaseCyclicClient::Refresh");
      if (kind == 1) throw std::future_error(std::future_errc::future_already_retrieved);
      throw 17;
    };
    require(guard.run(Result::ERROR, failed, report) == Result::ERROR, "exception was not converted to ERROR");
    require(guard.failed() && reports == 1 && !cause.empty(), "original error was lost");
    if (kind == 0) require(cause.find("Refresh") != std::string::npos, "original RPC cause missing");
    for (int cycle = 0; cycle < 10000; ++cycle)
      require(guard.run(Result::ERROR, healthy, report) == Result::ERROR, "failed session resumed");
    require(calls == 2 && reports == 1, "failed session retried communication or flooded logs");
  }
  std::cout << "cyclic runtime/future/unknown exceptions latch without retrying passed\n";
}
