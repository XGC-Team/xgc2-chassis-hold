// The Algebraic Data Type (ADT) state machine of one robot.
//
//   HoldStateVariant = ReleasedState + HeldState(GatedStage + ZeroWrittenStage + StoppedStage + FaultedStage)
//
//   Released ---------- Engage ----------> Held(Gated)
//   Held ------- Release [instance, revision, no fault, not withdrawn] -----> Released
//
//   Held substates:
//     Gated --ZeroWritten--> ZeroWritten --MotionStopped--> Stopped
//     ZeroWritten, Stopped --ZeroFailed--> Gated
//     Stopped --MotionResumed--> ZeroWritten
//     Gated, ZeroWritten, Stopped, Faulted --Fault--> Faulted --ClearFault--> Gated
//     Gated, ZeroWritten, Stopped, Faulted --Rebound--> Gated
//
// The machine runs on the tick thread. Domain::drain holds the entry mutex around start() and every update(),
// so transitions change the published state of the entry and the gate mirror without taking it again, and a
// reader never sees half a transition. Outcomes of engage and release are decided here and handed to whoever waits
// for them through the ticket in the strongly-typed event struct.
#include "domain_impl.hpp"

#include <deque>
#include <stdexcept>
#include <utility>

namespace xgc2::chassis_hold {
namespace {

using detail::Entry;
constexpr std::size_t kMaxPendingEvents = 4096;

// Zero results and motion reports are about one device and one hold: a late one describes an earlier hold or the
// device that was removed.
bool current(const Entry &entry, std::uint64_t binding, std::uint64_t revision) {
  return binding == entry.binding && revision == entry.state.revision;
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

struct GatedStage {
  static constexpr Stage kPublishedStage = Stage::gated;
};

struct ZeroWrittenStage {
  static constexpr Stage kPublishedStage = Stage::zero_written;
};

struct StoppedStage {
  static constexpr Stage kPublishedStage = Stage::stopped;
};

// A faulted robot is held with its zero not trusted: the stage it reports is gated.
struct FaultedStage {
  static constexpr Stage kPublishedStage = Stage::gated;
};

using HeldSubstate = std::variant<
    GatedStage,
    ZeroWrittenStage,
    StoppedStage,
    FaultedStage>;

Stage publishedStageOf(const HeldSubstate &sub) {
  return std::visit(
      [](const auto &s) -> Stage {
        using T = std::decay_t<decltype(s)>;
        return T::kPublishedStage;
      },
      sub);
}

struct ReleasedState {};

struct HeldState {
  HeldSubstate substate{GatedStage{}};
};

using HoldStateVariant = std::variant<ReleasedState, HeldState>;
using HoldTransition = adt::Transition<HoldStateVariant, std::string>;

HoldTransition onReleasedEvent(ReleasedState &, Entry &entry, const ev::Event &event) {
  return std::visit(
      adt::Overloaded{
          [&](const ev::Engage &e) -> HoldTransition {
            entry.state.held = true;
            ++entry.state.revision;
            entry.state.gated_at = e.at;
            forget_progress(entry);
            apply_gate(entry, true, true);
            entry.resolve(e.ticket, Outcome::engaged);
            return HoldTransition::to(HeldState{GatedStage{}});
          },
          [&](const ev::Release &e) -> HoldTransition {
            const bool stale = e.instance != entry.domain.options.instance_id;
            entry.resolve(e.ticket, stale ? Outcome::conflict : Outcome::not_held);
            return HoldTransition::stay();
          },
          [&](const auto &) -> HoldTransition {
            return HoldTransition::stay();
          }},
      event);
}

HoldTransition onHeldEvent(HeldState &held, Entry &entry, const ev::Event &event) {
  return std::visit(
      adt::Overloaded{
          [&](const ev::Engage &e) -> HoldTransition {
            apply_gate(entry, true, true);
            entry.resolve(e.ticket, Outcome::already_held);
            return HoldTransition::stay();
          },
          [&](const ev::Release &e) -> HoldTransition {
            const bool allowed =
                entry.tickets.count(e.ticket) == 1 &&
                e.instance == entry.domain.options.instance_id &&
                e.revision == entry.state.revision &&
                entry.state.fault.empty();
            if (!allowed) {
              entry.resolve(e.ticket, Outcome::conflict);
              return HoldTransition::stay();
            }
            entry.fence.store(entry.domain.now(), std::memory_order_release);
            entry.state.held = false;
            ++entry.state.revision;
            entry.state.gated_at.reset();
            forget_progress(entry);
            apply_gate(entry, false, false);
            entry.resolve(e.ticket, Outcome::released);
            return HoldTransition::to(ReleasedState{});
          },
          [&](const ev::ZeroWritten &e) -> HoldTransition {
            if (!current(entry, e.binding, e.revision))
              return HoldTransition::stay();
            if (std::holds_alternative<GatedStage>(held.substate)) {
              const TimeNs now = entry.domain.now();
              entry.state.zero_written_at = now;
              entry.state.write_error.clear();
              entry.rest_since.reset();
              entry.last_sample.reset();
              return HoldTransition::to(HeldState{ZeroWrittenStage{}});
            }
            return HoldTransition::stay();
          },
          [&](const ev::ZeroFailed &e) -> HoldTransition {
            if (!current(entry, e.binding, e.revision))
              return HoldTransition::stay();
            ++entry.state.zero_write_failed;
            entry.state.write_error =
                clean_text(e.error.empty() ? "zero write failed" : e.error);
            if (std::holds_alternative<ZeroWrittenStage>(held.substate) ||
                std::holds_alternative<StoppedStage>(held.substate)) {
              entry.state.zero_written_at.reset();
              entry.state.stopped_at.reset();
              entry.rest_since.reset();
              entry.last_sample.reset();
              return HoldTransition::to(HeldState{GatedStage{}});
            }
            return HoldTransition::stay();
          },
          [&](const ev::MotionStopped &e) -> HoldTransition {
            if (!current(entry, e.binding, e.revision))
              return HoldTransition::stay();
            if (std::holds_alternative<ZeroWrittenStage>(held.substate)) {
              entry.state.stopped_at = entry.domain.now();
              return HoldTransition::to(HeldState{StoppedStage{}});
            }
            return HoldTransition::stay();
          },
          [&](const ev::MotionResumed &e) -> HoldTransition {
            if (!current(entry, e.binding, e.revision))
              return HoldTransition::stay();
            if (std::holds_alternative<StoppedStage>(held.substate)) {
              entry.state.stopped_at.reset();
              return HoldTransition::to(HeldState{ZeroWrittenStage{}});
            }
            return HoldTransition::stay();
          },
          [&](const ev::Fault &e) -> HoldTransition {
            entry.state.fault = clean_text(e.text);
            entry.state.zero_written_at.reset();
            entry.state.stopped_at.reset();
            entry.rest_since.reset();
            entry.last_sample.reset();
            if (std::holds_alternative<FaultedStage>(held.substate))
              return HoldTransition::stay();
            return HoldTransition::to(HeldState{FaultedStage{}});
          },
          [&](const ev::ClearFault &) -> HoldTransition {
            if (std::holds_alternative<FaultedStage>(held.substate)) {
              entry.state.fault.clear();
              return HoldTransition::to(HeldState{GatedStage{}});
            }
            return HoldTransition::stay();
          },
          [&](const ev::Rebound &e) -> HoldTransition {
            entry.state.gated_at = e.at;
            entry.state.fault.clear();
            forget_progress(entry);
            if (std::holds_alternative<GatedStage>(held.substate))
              return HoldTransition::stay();
            return HoldTransition::to(HeldState{GatedStage{}});
          }},
      event);
}

struct HoldMachineSpec {
  using StateVariant = HoldStateVariant;
  using EventVariant = ev::Event;
  using OutputVariant = std::monostate;
  using Context = Entry;
  using RejectReason = std::string;
  using OutputBuffer = adt::BoundedVector<OutputVariant, 4>;

  static void onEnter(ReleasedState &, Context &entry, double, OutputBuffer &) {
    entry.state.stage = Stage::released;
  }

  static void onEnter(HeldState &held, Context &entry, double, OutputBuffer &) {
    entry.state.stage = publishedStageOf(held.substate);
  }

  template <typename ConcreteEvent>
  static HoldTransition onEvent(
      ReleasedState &released, Context &entry, const ConcreteEvent &event, double, OutputBuffer &) {
    return onReleasedEvent(released, entry, ev::Event{event});
  }

  template <typename ConcreteEvent>
  static HoldTransition onEvent(
      HeldState &held, Context &entry, const ConcreteEvent &event, double, OutputBuffer &) {
    return onHeldEvent(held, entry, ev::Event{event});
  }

  static adt::TickPolicy tickPolicy(const ReleasedState &, const Context &, double, double) {
    return adt::TickIdle{};
  }

  static adt::TickPolicy tickPolicy(const HeldState &, const Context &, double, double) {
    return adt::TickPeriodic{0.02};
  }
};

}  // namespace

namespace detail {

class HoldMachine {
 public:
  HoldMachine() : adt_machine_(HoldStateVariant{ReleasedState{}}) {}

  bool postEvent(ev::Event event) {
    std::lock_guard<std::mutex> lock(inbox_mutex_);
    if (inbox_.size() >= kMaxPendingEvents)
      return false;
    inbox_.push_back(std::move(event));
    return true;
  }

  void start(Entry &entry) {
    checkOwnerThread();
    adt_machine_.start(entry, 0.0);
  }

  HoldDrainRound update(Entry &entry, std::size_t max_events) {
    checkOwnerThread();
    HoldDrainRound round;
    while (round.events_taken < max_events) {
      std::optional<ev::Event> next_event;
      {
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        if (inbox_.empty()) {
          round.events_remaining = 0;
          break;
        }
        next_event = std::move(inbox_.front());
        inbox_.pop_front();
        round.events_remaining = inbox_.size();
      }
      ++round.events_taken;
      (void)adt_machine_.dispatch(entry, *next_event, 0.0);
    }
    (void)adt_machine_.step(entry, 0.0);
    return round;
  }

 private:
  void checkOwnerThread() {
    const auto caller = std::this_thread::get_id();
    if (owner_thread_ == std::thread::id()) {
      owner_thread_ = caller;
    } else if (owner_thread_ != caller) {
      throw std::logic_error("Domain::tick must always run on the same thread");
    }
  }

  std::mutex inbox_mutex_;
  std::deque<ev::Event> inbox_;
  std::thread::id owner_thread_;
  adt::AdtMachine<HoldMachineSpec, 4, 8, 8> adt_machine_;
};

Entry::Entry(DomainImpl &domain_in, std::string id_in)
    : domain(domain_in),
      id(std::move(id_in)),
      machine(std::make_unique<HoldMachine>()) {
  state.robot_id = id;
}

Entry::~Entry() = default;

std::int64_t Entry::add_ticket(const std::shared_ptr<Operation> &operation, Item *item) {
  const std::int64_t ticket = next_ticket++;
  tickets[ticket] = Ticket{operation, item};
  return ticket;
}

void Entry::resolve(std::int64_t ticket, Outcome outcome) {
  const auto found = tickets.find(ticket);
  if (ticket == 0 || found == tickets.end())
    return;
  Item *item = found->second.item;
  std::shared_ptr<Operation> operation = std::move(found->second.operation);
  tickets.erase(found);
  item->resolved.store(static_cast<int>(outcome), std::memory_order_release);
  operation->unresolved.fetch_sub(1, std::memory_order_acq_rel);
}

std::unique_ptr<HoldMachine> make_hold_machine(Entry &) {
  return std::make_unique<HoldMachine>();
}

bool post_hold_event(HoldMachine &machine, ev::Event event) {
  return machine.postEvent(std::move(event));
}

void start_hold_machine(HoldMachine &machine, Entry &entry) {
  machine.start(entry);
}

HoldDrainRound update_hold_machine(HoldMachine &machine, Entry &entry, std::size_t max_events) {
  return machine.update(entry, max_events);
}

}  // namespace detail
}  // namespace xgc2::chassis_hold
