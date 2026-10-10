#include <xgc2/chassis_hold/state.hpp>
#include <state_machine/state_machine.hpp>
#include <limits>
#include <stdexcept>
#include <thread>

namespace xgc2::chassis_hold {
namespace sm = state_machine;
struct HoldState::Impl {
  struct Gate final : sm::State {
    bool &view;
    const bool closed;
    Gate(bool &value, bool held) : view(value), closed(held) {}
    std::string name() const override { return closed ? "Held" : "Released"; }
    sm::Status onEnter(sm::StateContext &) override { view = closed; return {}; }
  };
  const std::size_t robots;
  std::uint64_t revision = 0;
  std::array<bool, max_robots> gates{}, desired{};
  std::unique_ptr<sm::StateMachine> machine;
  std::thread::id owner;
  bool started = false, faulted = false;

  explicit Impl(std::size_t count) : robots(count) {
    if (!count || count > max_robots)
      throw std::invalid_argument("chassis hold requires 1..16 robots");
    sm::RuntimeOptions limits;
    limits.event_log_capacity = 64;
    limits.fault_log_capacity = 4;
    limits.max_pending_events = 1; // condition transitions need no event inbox
    auto builder = sm::StateMachine::builder("chassis.hold", limits);
    for (std::size_t i = 0; i < count; ++i) {
      const auto released = static_cast<sm::StateId>(2*i+1), held = released+1;
      builder.region(static_cast<sm::RegionId>(i+1)).order(static_cast<int>(i))
          .initial(released)
          .state(released).impl(std::make_unique<Gate>(gates[i], false)).endState()
          .state(held).impl(std::make_unique<Gate>(gates[i], true)).endState().endRegion()
          .transition().from(released).to(held).when([this, i](const sm::GuardContext &) { return desired[i]; })
          .transition().from(held).to(released).when([this, i](const sm::GuardContext &) { return !desired[i]; });
    }
    auto built = builder.build();
    if (!built.status.ok()) throw std::runtime_error(built.status.message);
    machine = std::move(built.value);
  }
  bool transition() {
    // Start on the native owner, not on the constructor/framework IO thread.
    if (!started) {
      owner = std::this_thread::get_id();
      if (!machine->start().ok()) return false;
      started = true;
    }
    if (owner != std::this_thread::get_id()) return false;
    auto update = machine->update({1, max_robots, false});
    if (!update.status.ok() || update.value.faults_recorded) return false;
    for (std::size_t i = 0; i < robots; ++i)
      if (gates[i] != desired[i]) return false;
    return true;
  }
};
HoldState::HoldState(std::size_t robots) : impl_(std::make_unique<Impl>(robots)) {}
HoldState::~HoldState() = default;
bool HoldState::healthy() const noexcept { return !impl_->faulted; }
bool HoldState::held(std::size_t robot) const noexcept {
  return impl_->faulted || robot >= impl_->robots || impl_->gates[robot];
}
Result HoldState::snapshot(Outcome outcome) const noexcept {
  Result result;
  result.outcome = outcome;
  result.revision = impl_->revision;
  for (std::size_t i = 0; i < impl_->robots; ++i) result.held[i] = held(i);
  return result;
}
Result HoldState::apply(const Command &command, ZeroFn zero, void *context) noexcept {
  if (!command.mutate) return snapshot(impl_->faulted ? Outcome::Failed : Outcome::Snapshot);
  if (!command.count || command.count > impl_->robots || !zero) return snapshot(Outcome::Invalid);
  std::array<bool, max_robots> seen{};
  for (std::size_t i = 0; i < command.count; ++i) {
    const auto index = command.changes[i].robot;
    if (index >= impl_->robots || seen[index]) return snapshot(Outcome::Invalid);
    seen[index] = true;
  }
  if (command.expected_revision != impl_->revision || impl_->revision == std::numeric_limits<std::uint64_t>::max())
    return snapshot(Outcome::Conflict);
  if (impl_->faulted) return snapshot(Outcome::Failed);
  Result result;
  bool success = true;
  try {
    for (std::size_t i = 0; i < command.count; ++i) impl_->desired[command.changes[i].robot] = true;
    if (!impl_->transition()) throw std::runtime_error("hold gate transition failed");
    // All affected gates are closed before the first hardware write. Always
    // attempt every zero, even if one fails. Never reopen a partial batch.
    for (std::size_t i = 0; i < command.count; ++i) {
      const auto index = command.changes[i].robot;
      result.zero_applied[index] = zero(context, index);
      success = result.zero_applied[index] && success;
    }
    if (success) {
      for (std::size_t i = 0; i < command.count; ++i)
        impl_->desired[command.changes[i].robot] = command.changes[i].held;
      if (!impl_->transition()) throw std::runtime_error("release gate transition failed");
    }
  } catch (...) {
    // Runtime/allocator failure can interrupt a transition: fail closed across
    // the entire roster, including any already-entered Released states.
    impl_->faulted = true;
    success = false;
  }
  ++impl_->revision;
  const auto view = snapshot(success ? Outcome::Applied : Outcome::Failed);
  result.outcome = view.outcome;
  result.revision = view.revision;
  result.held = view.held;
  return result;
}
} // namespace xgc2::chassis_hold
