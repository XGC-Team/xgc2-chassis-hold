// The state machine of one robot.
//
//   Released ---------- Engage ----------> Held
//   Held ------- Release [instance, revision, deadline, no fault] -----> Released
//
//   Held contains          Gated --ZeroWritten--> ZeroWritten --MotionStopped--> Stopped
//                          ZeroWritten, Stopped --ZeroFailed--> Gated      Stopped --MotionResumed--> ZeroWritten
//                          Gated, ZeroWritten, Stopped --Fault--> Faulted --ClearFault--> Gated
//                          ZeroWritten, Stopped, Faulted --Rebound--> Gated
//
// The machine runs on the tick thread (update). Domain::drain holds the entry mutex around start() and every update(),
// so the callbacks below change the published state of the entry and the gate mirror without taking it again, and a
// reader never sees half a transition. Outcomes of engage and release are decided here and handed to whoever waits
// for them through the ticket in the event.
#include "domain_impl.hpp"

#include <stdexcept>
#include <utility>

namespace xgc2::chassis_hold {
namespace {

using detail::Entry;

std::int64_t number(const sm::Event &event, const char *key, std::int64_t fallback = 0) {
  const auto found = event.payload.find(key);
  if (found == event.payload.end())
    return fallback;
  const auto *value = std::get_if<std::int64_t>(&found->second);
  return value ? *value : fallback;
}

std::string text(const sm::Event &event, const char *key) {
  const auto found = event.payload.find(key);
  if (found == event.payload.end())
    return std::string();
  const auto *value = std::get_if<std::string>(&found->second);
  return value ? *value : std::string();
}

// Zero results and motion reports are about one device and one hold: a late one describes an earlier hold or the
// device that was removed.
bool current(const Entry &entry, const sm::Event &event) {
  return static_cast<std::uint64_t>(number(event, "binding")) == entry.binding &&
         static_cast<std::uint64_t>(number(event, "revision")) == entry.state.revision;
}

void forget_progress(Entry &entry) {
  entry.state.zero_written_at.reset();
  entry.state.stopped_at.reset();
  entry.state.write_error.clear();
  entry.rest_since.reset();
  entry.last_sample.reset();
}

// Moves an engage from "on its way" to the machine's hold. Both halves of the gate change in one step.
void apply_gate(Entry &entry, bool consume_engage, bool hold) {
  std::uint32_t gate = entry.gate.load(std::memory_order_acquire);
  for (;;) {
    std::uint32_t next = gate;
    if (consume_engage && (next & ~kBrokenBit) >= kPendingUnit)
      next -= kPendingUnit;
    next = hold ? (next | kHeldBit) : (next & ~kHeldBit);
    if (entry.gate.compare_exchange_weak(gate, next, std::memory_order_acq_rel, std::memory_order_acquire))
      return;
  }
}

class StageState : public sm::State {
public:
  StageState(Entry &entry, const char *name) : entry_(entry), name_(name) {}
  std::string name() const override { return name_; }

protected:
  void enter(Stage stage) {
    entry_.state.stage = stage;
  }
  Entry &entry_;
  const char *name_;
};

class ReleasedState final : public StageState {
public:
  explicit ReleasedState(Entry &entry) : StageState(entry, "Released") {}
  sm::ActionResult onEnter(sm::StateContext &) override {
    enter(Stage::released);
    return {};
  }
  // Nothing consumed a release: there is nothing to release, unless the caller is stale.
  sm::ActionResult onEvent(sm::StateContext &, const sm::Event &event) override {
    if (event.id == ev::Release) {
      const bool stale = text(event, "instance") != entry_.domain.options.instance_id;
      entry_.resolve(number(event, "ticket"), stale ? Outcome::conflict : Outcome::not_held);
    }
    return {};
  }
};

class HeldState final : public StageState {
public:
  explicit HeldState(Entry &entry) : StageState(entry, "Held") {}
  // The guard of the release refused it: late, or the wrong instance, revision or a fault.
  sm::ActionResult onEvent(sm::StateContext &, const sm::Event &event) override {
    if (event.id == ev::Release) {
      const bool late = entry_.domain.now() > number(event, "deadline", no_deadline);
      entry_.resolve(number(event, "ticket"), late ? Outcome::expired : Outcome::conflict);
    }
    return {};
  }
};

class GatedState final : public StageState {
public:
  explicit GatedState(Entry &entry) : StageState(entry, "Gated") {}
  sm::ActionResult onEnter(sm::StateContext &) override {
    enter(Stage::gated);
    return {};
  }
};

class ZeroWrittenState final : public StageState {
public:
  explicit ZeroWrittenState(Entry &entry) : StageState(entry, "ZeroWritten") {}
  sm::ActionResult onEnter(sm::StateContext &) override {
    enter(Stage::zero_written);
    return {};
  }
};

class StoppedState final : public StageState {
public:
  explicit StoppedState(Entry &entry) : StageState(entry, "Stopped") {}
  sm::ActionResult onEnter(sm::StateContext &) override {
    enter(Stage::stopped);
    return {};
  }
};

// A faulted robot is held with its zero not trusted: the stage it reports is gated.
class FaultedState final : public StageState {
public:
  explicit FaultedState(Entry &entry) : StageState(entry, "Faulted") {}
  sm::ActionResult onEnter(sm::StateContext &) override {
    enter(Stage::gated);
    return {};
  }
};

}  // namespace

std::int64_t detail::Entry::add_ticket(const std::shared_ptr<detail::Operation> &operation, detail::Item *item) {
  const std::int64_t ticket = next_ticket++;
  tickets[ticket] = Ticket{operation, item};
  return ticket;
}

void detail::Entry::resolve(std::int64_t ticket, Outcome outcome) {
  const auto found = tickets.find(ticket);
  if (ticket == 0 || found == tickets.end())
    return;
  detail::Item *item = found->second.item;
  std::shared_ptr<detail::Operation> operation = std::move(found->second.operation);
  tickets.erase(found);
  item->resolved.store(static_cast<int>(outcome), std::memory_order_release);
  operation->unresolved.fetch_sub(1, std::memory_order_acq_rel);
}

std::unique_ptr<sm::StateMachine> detail::make_hold_machine(Entry &entry) {
  using Rule = std::function<sm::ActionResult(sm::StateContext &)>;
  using Guard = std::function<bool(const sm::GuardContext &)>;
  Entry *const self = &entry;

  sm::RuntimeOptions options;
  options.allow_prestart_events = true;  // the machine starts on the first tick
  options.event_log_capacity = 8;
  options.fault_log_capacity = 4;
  options.max_pending_events = 4096;
  auto builder = sm::StateMachine::builder("chassis.hold." + entry.id, options);
  builder.region(sm::kDefaultRegion)
      .initial(st::Released)
      .state(st::Released).impl(std::make_unique<ReleasedState>(entry))
      .state(st::Held).impl(std::make_unique<HeldState>(entry)).initial(st::Gated)
        .state(st::Gated).impl(std::make_unique<GatedState>(entry))
        .state(st::ZeroWritten).impl(std::make_unique<ZeroWrittenState>(entry))
        .state(st::Stopped).impl(std::make_unique<StoppedState>(entry))
        .state(st::Faulted).impl(std::make_unique<FaultedState>(entry))
      .endState()
      .endRegion();

  // Engage: not held -> Held (revision + 1); held -> nothing changes. Either way the engage leaves the gate's
  // "on its way" count.
  const Rule engage_applied = [self](sm::StateContext &ctx) {
    const sm::Event &event = *ctx.event();
    self->state.held = true;
    ++self->state.revision;
    self->state.gated_at = number(event, "at");
    forget_progress(*self);
    apply_gate(*self, true, true);
    self->resolve(number(event, "ticket"), Outcome::engaged);
    return sm::Status{};
  };
  const Rule engage_again = [self](sm::StateContext &ctx) {
    const sm::Event &event = *ctx.event();
    apply_gate(*self, true, true);
    self->resolve(number(event, "ticket"), Outcome::already_held);
    return sm::Status{};
  };

  // Release: the fence is taken before admission opens, so that nothing received while held is admitted later.
  const Guard release_allowed = [self](const sm::GuardContext &guard) {
    const sm::Event &event = *guard.event();
    const TimeNs now = self->domain.now();
    return text(event, "instance") == self->domain.options.instance_id &&
           static_cast<std::uint64_t>(number(event, "revision")) == self->state.revision && self->state.fault.empty() &&
           now <= number(event, "deadline", no_deadline);
  };
  const Rule released = [self](sm::StateContext &ctx) {
    const sm::Event &event = *ctx.event();
    self->fence.store(self->domain.now(), std::memory_order_release);
    self->state.held = false;
    ++self->state.revision;
    self->state.gated_at.reset();
    forget_progress(*self);
    apply_gate(*self, false, false);
    self->resolve(number(event, "ticket"), Outcome::released);
    return sm::Status{};
  };

  // Zero results of the tick.
  const Guard is_current = [self](const sm::GuardContext &guard) {
    return current(*self, *guard.event());
  };
  const Rule zero_written = [self](sm::StateContext &) {
    const TimeNs now = self->domain.now();
    self->state.zero_written_at = now;
    self->state.write_error.clear();
    self->rest_since.reset();
    self->last_sample.reset();
    return sm::Status{};
  };
  const Rule write_failed = [self](sm::StateContext &ctx) {
    const sm::Event &event = *ctx.event();
    ++self->state.zero_write_failed;
    self->state.write_error = clean_text(text(event, "error").empty() ? "zero write failed" : text(event, "error"));
    return sm::Status{};
  };
  // A failed write takes the confirmation of the zero back.
  const Rule zero_lost = [self, write_failed](sm::StateContext &ctx) {
    write_failed(ctx);
    self->state.zero_written_at.reset();
    self->state.stopped_at.reset();
    self->rest_since.reset();
    self->last_sample.reset();
    return sm::Status{};
  };

  // Feedback.
  const Rule stopped = [self](sm::StateContext &) {
    const TimeNs now = self->domain.now();
    self->state.stopped_at = now;
    return sm::Status{};
  };
  const Rule resumed = [self](sm::StateContext &) {
    self->state.stopped_at.reset();
    return sm::Status{};
  };

  // Faults.
  const Rule faulted = [self](sm::StateContext &ctx) {
    const sm::Event &event = *ctx.event();
    self->state.fault = clean_text(text(event, "text"));
    self->state.zero_written_at.reset();
    self->state.stopped_at.reset();
    self->rest_since.reset();
    self->last_sample.reset();
    return sm::Status{};
  };
  const Rule fault_cleared = [self](sm::StateContext &) {
    self->state.fault.clear();
    return sm::Status{};
  };

  // The robot was added again: a new device, no zero yet, and the fault belonged to the old one.
  const Rule rebound = [self](sm::StateContext &ctx) {
    const sm::Event &event = *ctx.event();
    self->state.gated_at = number(event, "at");
    self->state.fault.clear();
    forget_progress(*self);
    return sm::Status{};
  };

  const auto internal = [&builder](sm::StateId from, sm::EventId event) -> sm::StateMachine::Builder & {
    return builder.transition().from(from).on(event).type(sm::TransitionType::kInternal);
  };
  const auto transition_to = [&builder](sm::StateId from, sm::StateId to, sm::EventId event) -> sm::StateMachine::Builder & {
    return builder.transition().from(from).to(to).on(event);
  };

  transition_to(st::Released, st::Held, ev::Engage).action(engage_applied);
  internal(st::Held, ev::Engage).action(engage_again);
  transition_to(st::Held, st::Released, ev::Release).priority(10).when(release_allowed).action(released);

  transition_to(st::Gated, st::ZeroWritten, ev::ZeroWritten).when(is_current).action(zero_written);
  internal(st::Gated, ev::ZeroFailed).when(is_current).action(write_failed);
  transition_to(st::ZeroWritten, st::Gated, ev::ZeroFailed).when(is_current).action(zero_lost);
  transition_to(st::Stopped, st::Gated, ev::ZeroFailed).when(is_current).action(zero_lost);
  internal(st::Faulted, ev::ZeroFailed).when(is_current).action(write_failed);

  transition_to(st::ZeroWritten, st::Stopped, ev::MotionStopped).when(is_current).action(stopped);
  transition_to(st::Stopped, st::ZeroWritten, ev::MotionResumed).when(is_current).action(resumed);

  transition_to(st::Gated, st::Faulted, ev::Fault).action(faulted);
  transition_to(st::ZeroWritten, st::Faulted, ev::Fault).action(faulted);
  transition_to(st::Stopped, st::Faulted, ev::Fault).action(faulted);
  internal(st::Faulted, ev::Fault).action(faulted);
  transition_to(st::Faulted, st::Gated, ev::ClearFault).action(fault_cleared);

  internal(st::Gated, ev::Rebound).action(rebound);
  transition_to(st::ZeroWritten, st::Gated, ev::Rebound).action(rebound);
  transition_to(st::Stopped, st::Gated, ev::Rebound).action(rebound);
  transition_to(st::Faulted, st::Gated, ev::Rebound).action(rebound);

  auto built = builder.build();
  if (!built.ok())
    throw std::runtime_error("chassis hold state machine: " + built.status.message);
  return std::move(built.value);
}

}  // namespace xgc2::chassis_hold
