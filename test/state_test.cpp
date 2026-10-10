#include <xgc2/chassis_hold/state.hpp>
#include <cassert>
#include <thread>
#include <stdexcept>
using namespace xgc2::chassis_hold;
struct Sink {
  HoldState &state;
  unsigned writes = 0;
  static bool zero(void *context, std::size_t index) noexcept {
    auto &sink = *static_cast<Sink *>(context);
    assert(sink.state.held(index));
    ++sink.writes;
    return true;
  }
};
int main() {
  for (auto count : {std::size_t{0}, max_robots+1}) {
    bool rejected = false;
    try { HoldState invalid(count); } catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
  HoldState state(max_robots);
  Sink sink{state};
  Command command;
  command.mutate = true;
  command.count = max_robots;
  for (std::size_t i = 0; i < max_robots; ++i) command.changes[i] = {static_cast<std::uint8_t>(i), i%2 == 0};
  assert(state.apply(command, Sink::zero, &sink).outcome == Outcome::Applied);
  for (std::size_t i = 0; i < max_robots; ++i) assert(state.held(i) == (i%2 == 0));
  assert(sink.writes == max_robots && state.held(max_robots));
  // Repeated commands still clear cached output; no self-transition is needed.
  command.expected_revision = 1;
  assert(state.apply(command, Sink::zero, &sink).outcome == Outcome::Applied);
  assert(sink.writes == 2*max_robots);
  command.count = 1;
  command.expected_revision = 2;
  command.changes[0] = {0, false};
  assert(state.apply(command, Sink::zero, &sink).outcome == Outcome::Applied);
  assert(!state.held(0) && state.held(2) && !state.held(1));
  // Duplicate/out-of-range entries reject the whole batch before output.
  const auto writes = sink.writes;
  command.count = 2;
  command.expected_revision = 3;
  command.changes[1] = command.changes[0];
  assert(state.apply(command, Sink::zero, &sink).outcome == Outcome::Invalid);
  command.changes[1].robot = max_robots;
  assert(state.apply(command, Sink::zero, &sink).outcome == Outcome::Invalid);
  assert(sink.writes == writes && state.snapshot().revision == 3);
  command.count = 1;
  // The state-machine library enforces the native owner. Fail closed on misuse.
  std::thread other([&] { assert(state.apply(command, Sink::zero, &sink).outcome == Outcome::Failed); });
  other.join();
  assert(!state.healthy() && sink.writes == writes);
  for (std::size_t i = 0; i < max_robots; ++i) assert(state.held(i));
}
