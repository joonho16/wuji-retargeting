#include <cassert>
#include <string>
#include "saved_action_lease.hpp"
using gen3_kortex::SavedActionLease;
static std::string command(const char* mode, const char* token, int64_t stamp)
{ return std::string(mode) + "\n" + token + "\n" + std::to_string(stamp) + "\nready"; }
int main()
{
  SavedActionLease lease;
  const int64_t start = 1000000000;
  assert(!lease.receive(command("H", "b1", start), start + 100000001));
  assert(!lease.receive(command("H", "b1", start+1), start));
  assert(!lease.claim(start));
  assert(lease.receive(command("H", "b1", start), start));
  assert(lease.claim(start));
  assert(!lease.claim(start));
  assert(lease.held("b1", start + 99999999));
  assert(!lease.held("b1", start + 100000001));
  assert(lease.receive(command("H", "b1", start + 110000000), start + 110000000));
  assert(!lease.claim(start + 110000000));  // stale B never starts again
  assert(lease.receive(command("S", "b1", start + 120000000), start + 120000000));
  assert(!lease.held(start + 120000000));
  assert(!lease.receive(command("H", "b1", start + 115000000), start + 125000000));
  assert(!lease.held(start + 125000000));  // delayed hold cannot undo release
  assert(lease.receive(command("H", "b2", start + 130000000), start + 130000000));
  assert(lease.claim(start + 130000000));
}
