#pragma once
// Owner-only HOLD values. The confirmed-zero input accumulator has exactly the
// lifetime of the child that owns it; publication borrows it under Entry::mutex.
#include <state_machine/adt_machine.hpp>
#include <xgc2/chassis_hold/domain.hpp>

#include <memory>
#include <variant>

namespace xgc2::chassis_hold {
namespace adt = state_machine::adt;
namespace ev {
struct Engage { std::int64_t ticket; TimeNs at; };
struct Release { std::int64_t ticket; bool instance_ok; std::uint64_t revision; };
struct ZeroWritten { std::uint64_t binding, revision; };
struct ZeroFailed { std::uint64_t binding, revision; std::string error; };
struct MotionStopped { std::uint64_t binding, revision; };
struct MotionResumed { std::uint64_t binding, revision; };
struct Fault { std::string text; };
struct ClearFault {};
struct Rebound { TimeNs at; };
using Event = std::variant<Engage, Release, ZeroWritten, ZeroFailed, MotionStopped, MotionResumed,
                           Fault, ClearFault, Rebound>;
}  // namespace ev

namespace detail {
struct DomainImpl;
struct FeedbackProgress {
  explicit FeedbackProgress(TimeNs at) : zero_written_at(at) {}
  const TimeNs zero_written_at;
  // Only observe() touches these fields, under the publication mutex.
  std::optional<TimeNs> rest_since;
  std::optional<TimeNs> last_sample;
};
struct Gated {};
struct ZeroWritten { std::unique_ptr<FeedbackProgress> progress; };
struct Stopped { std::unique_ptr<FeedbackProgress> progress; TimeNs stopped_at; };
struct Faulted {};
struct StageSpec {
  using State = std::variant<Gated, ZeroWritten, Stopped, Faulted>;
  using Time = TimeNs;
  struct TakeProgress {};
  static std::unique_ptr<FeedbackProgress> handle(Gated &, const TakeProgress &) { return {}; }
  static std::unique_ptr<FeedbackProgress> handle(ZeroWritten &s, const TakeProgress &) { return std::move(s.progress); }
  static std::unique_ptr<FeedbackProgress> handle(Stopped &s, const TakeProgress &) { return std::move(s.progress); }
  static std::unique_ptr<FeedbackProgress> handle(Faulted &, const TakeProgress &) { return {}; }
};
struct Released {};
struct Held {
  explicit Held(TimeNs at) : gated_at(at), child(Gated{}, at) {}
  TimeNs gated_at;
  std::string fault;
  std::string write_error;
  adt::AdtMachine<StageSpec> child;
};
// Child outcomes separate new evidence from a transfer of the existing evidence.
struct ConfirmZero { std::unique_ptr<FeedbackProgress> progress; };
struct MarkStopped { TimeNs at; };
struct MarkResumed {};
using ChildChange = std::variant<adt::Stay, adt::Enter<Gated>, ConfirmZero, MarkStopped,
                                 MarkResumed, adt::Enter<Faulted>>;
struct HoldResult {
  adt::Transition<Released, Held> transition{adt::Stay{}};
  std::int64_t ticket = 0;
  std::optional<Outcome> outcome;
  bool consume_engage = false;
  ChildChange child{adt::Stay{}};
  std::optional<std::string> fault;
  std::optional<std::string> write_error;
  bool write_failed = false;
};
struct HoldContext {
  DomainImpl &domain;
  std::uint64_t revision;
  TimeNs at = 0;
};
struct HoldSpec {
  using State = std::variant<Released, Held>;
  using Time = TimeNs;
  static HoldResult handle(Released &, const ev::Event &, HoldContext &);
  static HoldResult handle(Held &, const ev::Event &, HoldContext &);
  // The owner installs a selected outcome only after its publication commits.
  struct Install { HoldResult &result; TimeNs at; };
  static void handle(Released &, const Install &);
  static void handle(Held &, const Install &);
};
using HoldMachine = adt::AdtMachine<HoldSpec>;

// Projection only: no reader can reach the mutable machine through these values.
RobotState project(const HoldMachine &, const std::string &id, std::uint64_t revision,
                   std::uint64_t zero_write_failed);
FeedbackProgress *feedback_progress(const HoldMachine &) noexcept;
RobotState project_result(const HoldMachine &, const HoldResult &, const std::string &id,
                          std::uint64_t revision, std::uint64_t zero_write_failed);
FeedbackProgress *result_progress(const HoldMachine &, const HoldResult &) noexcept;
}  // namespace detail
}  // namespace xgc2::chassis_hold
