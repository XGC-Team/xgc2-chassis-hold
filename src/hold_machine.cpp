// Typed decisions are prepared by the output owner without domain locks. The
// domain publishes their effects atomically with ticket withdrawal and admission;
// only then does the owner install the value transition, outside those locks.
#include "domain_impl.hpp"

#include <utility>

namespace xgc2::chassis_hold::detail {
namespace {
template <class... F> struct Visit : F... { using F::operator()...; };
template <class... F> Visit(F...) -> Visit<F...>;

FeedbackProgress *progress(const Held &held) noexcept {
  return std::visit(Visit{
    [](const Gated &) -> FeedbackProgress * { return nullptr; },
    [](const ZeroWritten &s) { return s.progress.get(); },
    [](const Stopped &s) { return s.progress.get(); },
    [](const Faulted &) -> FeedbackProgress * { return nullptr; }
  }, held.child.state());
}

HoldResult receipt(std::int64_t ticket, Outcome outcome, bool engage = false) {
  HoldResult result;
  result.ticket = ticket;
  result.outcome = outcome;
  result.consume_engage = engage;
  return result;
}
}  // namespace

HoldResult HoldSpec::handle(Released &, const ev::Event &event, HoldContext &ctx) {
  return std::visit(Visit{
    [&](const ev::Engage &e) {
      ctx.at = e.at;
      auto result = receipt(e.ticket, Outcome::engaged, true);
      result.transition.emplace<adt::Enter<Held>>(adt::Enter{Held{e.at}});
      return result;
    },
    [](const ev::Release &e) { return receipt(e.ticket, e.instance_ok ? Outcome::not_held : Outcome::conflict); },
    [](const ev::ZeroWritten &) { return HoldResult{}; },
    [](const ev::ZeroFailed &) { return HoldResult{}; },
    [](const ev::MotionStopped &) { return HoldResult{}; },
    [](const ev::MotionResumed &) { return HoldResult{}; },
    [](const ev::Fault &) { return HoldResult{}; },
    [](const ev::ClearFault &) { return HoldResult{}; },
    [](const ev::Rebound &) { return HoldResult{}; }
  }, event);
}

HoldResult HoldSpec::handle(Held &held, const ev::Event &event, HoldContext &ctx) {
  return std::visit(Visit{
    [](const ev::Engage &e) { return receipt(e.ticket, Outcome::already_held, true); },
    [&](const ev::Release &e) {
      // Ticket withdrawal is the remaining part of this original condition; the
      // domain checks it in the publication critical section, before any effect.
      if (!e.instance_ok || e.revision != ctx.revision || !held.fault.empty())
        return receipt(e.ticket, Outcome::conflict);
      auto result = receipt(e.ticket, Outcome::released);
      ctx.at = ctx.domain.now();
      result.transition = adt::Enter{Released{}};
      return result;
    },
    [&](const ev::ZeroWritten &) {
      HoldResult result;
      if (std::holds_alternative<Gated>(held.child.state())) {
        ctx.at = ctx.domain.now();
        result.child = ConfirmZero{std::make_unique<FeedbackProgress>(ctx.at)};
        result.write_error = std::string();
      }
      return result;
    },
    [&](const ev::ZeroFailed &e) {
      HoldResult result;
      result.write_failed = true;
      result.write_error = e.error;
      if (std::holds_alternative<ZeroWritten>(held.child.state()) ||
          std::holds_alternative<Stopped>(held.child.state())) {
        ctx.at = ctx.domain.now();
        result.child = adt::Enter{Gated{}};
      }
      return result;
    },
    [&](const ev::MotionStopped &) {
      HoldResult result;
      if (std::holds_alternative<ZeroWritten>(held.child.state())) {
        ctx.at = ctx.domain.now();
        result.child = MarkStopped{ctx.at};
      }
      return result;
    },
    [&](const ev::MotionResumed &) {
      HoldResult result;
      if (std::holds_alternative<Stopped>(held.child.state())) {
        ctx.at = ctx.domain.now();
        result.child = MarkResumed{};
      }
      return result;
    },
    [&](const ev::Fault &e) {
      HoldResult result;
      result.fault = e.text;
      if (!std::holds_alternative<Faulted>(held.child.state())) {
        ctx.at = ctx.domain.now();
        result.child = adt::Enter{Faulted{}};
      }
      return result;
    },
    [&](const ev::ClearFault &) {
      HoldResult result;
      if (std::holds_alternative<Faulted>(held.child.state())) {
        ctx.at = ctx.domain.now();
        result.fault = std::string();
        result.child = adt::Enter{Gated{}};
      }
      return result;
    },
    [&](const ev::Rebound &e) {
      ctx.at = e.at;
      HoldResult result;
      result.transition.emplace<adt::Enter<Held>>(adt::Enter{Held{e.at}});
      return result;
    }
  }, event);
}

void HoldSpec::handle(Released &, const Install &) {}
void HoldSpec::handle(Held &held, const Install &install) {
  auto &result = install.result;
  if (result.fault) held.fault = std::move(*result.fault);
  if (result.write_error) held.write_error = std::move(*result.write_error);
  std::visit(Visit{
    [](adt::Stay) {},
    [&](adt::Enter<Gated> change) { held.child.apply(std::move(change), install.at); },
    [&](ConfirmZero &change) { held.child.apply(adt::Enter{ZeroWritten{std::move(change.progress)}}, install.at); },
    [&](MarkStopped change) {
      held.child.apply(adt::Enter{Stopped{held.child.dispatch(StageSpec::TakeProgress{}), change.at}}, install.at);
    },
    [&](MarkResumed) {
      held.child.apply(adt::Enter{ZeroWritten{held.child.dispatch(StageSpec::TakeProgress{})}}, install.at);
    },
    [&](adt::Enter<Faulted> change) { held.child.apply(std::move(change), install.at); }
  }, result.child);
}

RobotState project(const HoldMachine &machine, const std::string &id, std::uint64_t revision,
                   std::uint64_t zero_write_failed) {
  RobotState state;
  state.robot_id = id;
  state.revision = revision;
  state.zero_write_failed = zero_write_failed;
  if (const auto *held = std::get_if<Held>(&machine.state())) {
    state.held = true;
    state.gated_at = held->gated_at;
    state.fault = held->fault;
    state.write_error = held->write_error;
    std::visit(Visit{
      [&](const Gated &) { state.stage = Stage::gated; },
      [&](const Faulted &) { state.stage = Stage::gated; },
      [&](const ZeroWritten &s) { state.stage = Stage::zero_written; state.zero_written_at = s.progress->zero_written_at; },
      [&](const Stopped &s) {
        state.stage = Stage::stopped;
        state.zero_written_at = s.progress->zero_written_at;
        state.stopped_at = s.stopped_at;
      }
    }, held->child.state());
  }
  return state;
}

FeedbackProgress *feedback_progress(const HoldMachine &machine) noexcept {
  const auto *held = std::get_if<Held>(&machine.state());
  return held ? progress(*held) : nullptr;
}

RobotState project_result(const HoldMachine &machine, const HoldResult &result, const std::string &id,
                          std::uint64_t revision, std::uint64_t zero_write_failed) {
  auto state = project(machine, id, revision, zero_write_failed + result.write_failed);
  if (const auto *entered = std::get_if<adt::Enter<Held>>(&result.transition)) {
    state.held = true;
    state.stage = Stage::gated;
    state.gated_at = entered->value.gated_at;
    state.zero_written_at.reset(); state.stopped_at.reset();
    state.fault.clear(); state.write_error.clear();
  } else if (std::holds_alternative<adt::Enter<Released>>(result.transition)) {
    state.held = false;
    state.stage = Stage::released;
    state.gated_at.reset(); state.zero_written_at.reset(); state.stopped_at.reset();
    state.fault.clear(); state.write_error.clear();
  }
  if (result.outcome == Outcome::engaged || result.outcome == Outcome::released) ++state.revision;
  if (result.fault) state.fault = *result.fault;
  if (result.write_error) state.write_error = *result.write_error;
  std::visit(Visit{
    [](adt::Stay) {},
    [&](adt::Enter<Gated>) { state.stage = Stage::gated; state.zero_written_at.reset(); state.stopped_at.reset(); },
    [&](adt::Enter<Faulted>) { state.stage = Stage::gated; state.zero_written_at.reset(); state.stopped_at.reset(); },
    [&](const ConfirmZero &change) { state.stage = Stage::zero_written; state.zero_written_at = change.progress->zero_written_at; },
    [&](MarkStopped change) { state.stage = Stage::stopped; state.stopped_at = change.at; },
    [&](MarkResumed) { state.stage = Stage::zero_written; state.stopped_at.reset(); }
  }, result.child);
  return state;
}

FeedbackProgress *result_progress(const HoldMachine &machine, const HoldResult &result) noexcept {
  if (!std::holds_alternative<adt::Stay>(result.transition) ||
      std::holds_alternative<adt::Enter<Gated>>(result.child) ||
      std::holds_alternative<adt::Enter<Faulted>>(result.child)) return nullptr;
  if (const auto *zero = std::get_if<ConfirmZero>(&result.child)) return zero->progress.get();
  return feedback_progress(machine);
}

void apply_gate(Entry &entry, bool consume_engage, bool hold) {
  std::uint32_t gate = entry.gate.load(std::memory_order_acquire);
  for (;;) {
    std::uint32_t next = gate;
    if (consume_engage && (next & ~kBrokenBit) >= kPendingUnit) next -= kPendingUnit;
    next = hold ? (next | kHeldBit) : (next & ~kHeldBit);
    if (entry.gate.compare_exchange_weak(gate, next, std::memory_order_acq_rel, std::memory_order_acquire)) return;
  }
}

std::int64_t Entry::add_ticket(const std::shared_ptr<Operation> &operation, Item *item) {
  const std::int64_t ticket = next_ticket++;
  tickets[ticket] = Ticket{operation, item};
  return ticket;
}
void Entry::resolve(std::int64_t ticket, Outcome outcome) {
  const auto found = tickets.find(ticket);
  if (ticket == 0 || found == tickets.end()) return;
  Item *item = found->second.item;
  std::shared_ptr<Operation> operation = std::move(found->second.operation);
  tickets.erase(found);
  item->resolved.store(static_cast<int>(outcome), std::memory_order_release);
  operation->unresolved.fetch_sub(1, std::memory_order_acq_rel);
}
}  // namespace xgc2::chassis_hold::detail
