#pragma once
// Chassis HOLD domain (contracts/chassis-hold.md). Plain C++17: no transport, no ROS, no JSON.
//
// HOLD stops one robot's chassis from executing motion commands until it is explicitly released. The state of every
// robot is an xgc2-state-machine: Released, or Held with the stages Gated -> ZeroWritten -> Stopped, or Faulted. The
// host feeds the domain commands (admit), runs the native tick on the thread that owns device output, and may report
// velocity feedback (observe). Operations that arrive on other threads (a transport's IO thread) are events of the
// machines: the tick applies them, in arrival order, and the machines run on the tick thread only. The one thing
// that does not wait for the tick is the gate: engage closes command admission before it returns.
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace xgc2::chassis_hold {

namespace detail {
struct Operation;
struct DomainImpl;
}

// Nanoseconds on the domain clock (DomainOptions::clock). Command receipt times, feedback stamps and every
// timestamp the domain reports are on this clock.
using TimeNs = std::int64_t;

enum class Stage { released, gated, zero_written, stopped };
// expired: a release that reached the machine after its deadline and was not applied; the service answers it with
// an error status, it is never a per-robot outcome on the wire.
enum class Outcome { engaged, already_held, released, conflict, not_held, unknown_robot, expired };

const char *to_string(Stage) noexcept;
const char *to_string(Outcome) noexcept;

// Robot ids match [A-Za-z0-9._:-]{1,128}. Instance ids are 32 lowercase hex digits (128 bits); the all-zero
// value is only a format-valid id (a domain refuses it, transports use it for "no instance").
bool valid_robot_id(const std::string &) noexcept;
bool valid_instance_id(const std::string &) noexcept;

struct DomainOptions {
  // Robots added to the roster for the first time start held (revision 1, stage gated). No persistence: a
  // restarted host is a new instance.
  bool boot_held = false;
  // observe(): a held robot is stopped after |v| <= linear_eps and |w| <= angular_eps continuously for
  // stopped_dwell.
  double linear_eps = 0.02;   // m/s
  double angular_eps = 0.05;  // rad/s
  std::chrono::milliseconds stopped_dwell{300};
  // 32 lowercase hex digits, not all zero. Empty: a random 128-bit id. A host whose transport already has an
  // instance id (the xrpc server of a world host) passes it here, so that the transport fence and the domain
  // agree; UdpEndpoint does the reverse and gives its server the id of the domain.
  std::string instance_id;
  // Empty: std::chrono::steady_clock. Hosts that stamp command receipt with another clock (a ROS driver using
  // MessageEvent::getReceiptTime() passes ros::Time::now()) must give the domain that same clock, otherwise the
  // release fence compares unrelated times. Called from every thread that uses the domain, sometimes with a lock
  // held: keep it cheap and non-blocking.
  std::function<TimeNs()> clock;
  // Upper bound on distinct robot ids ever added (removed ids keep their state).
  std::size_t max_robots = 1024;
};

struct RobotState {
  std::string robot_id;
  bool held = false;
  // Per robot, monotonic within one host instance; increments on every held/released transition.
  std::uint64_t revision = 0;
  Stage stage = Stage::released;
  // Stages reached since the last engage (domain clock); empty when not reached or not held.
  std::optional<TimeNs> gated_at;
  std::optional<TimeNs> zero_written_at;
  std::optional<TimeNs> stopped_at;
  // Unrecoverable device fault reported by the host (set_fault); empty when none. Printable ASCII, bounded.
  std::string fault;
  // Error of the most recent zero write when that write failed; empty otherwise.
  std::string write_error;
  std::uint64_t zero_write_failed = 0;  // failed zero writes since the robot was first added
  bool feedback = false;                // the host reports velocity feedback (observe) for this robot
};

struct RobotResult {
  std::string robot_id;
  Outcome outcome = Outcome::unknown_robot;
  std::optional<RobotState> state;  // state after the operation; empty for unknown_robot
};

struct Snapshot {
  std::string instance;
  TimeNs start = 0;                  // domain clock when the domain was created
  std::vector<RobotState> robots;    // requested order, or sorted by id for "all"
  std::vector<std::string> unknown;  // requested ids that are not in the roster
};

struct RobotInfo {
  std::string robot_id;
  bool feedback = false;
};
struct Description {
  std::string instance;
  std::vector<RobotInfo> robots;  // sorted by id
};

struct ReleaseChange {
  std::string robot_id;
  std::uint64_t expected_revision = 0;
};

// Device output of the host, called by Domain::tick on the thread that owns it.
class ZeroSink {
public:
  virtual ~ZeroSink() = default;
  // Writes one zero motion command for the robot. Returns false and a short reason in `error` when the command
  // did not reach the device. A thrown std::exception counts as a failed write.
  virtual bool write_zero(const std::string &robot_id, std::string &error) = 0;
};

struct TickStats {
  std::size_t written = 0;  // zero commands written
  std::size_t failed = 0;   // zero writes that failed
};

// An engage or a release that the next tick applies. Copies share the operation. It must not outlive its domain.
class Pending {
public:
  Pending() = default;
  // True once the tick has applied the operation for every robot (unknown robots count as applied at once).
  bool done() const;
  // The result per robot, in the order of the request. Once done() these are the outcomes the machines decided;
  // before that an engage reports what the gate decided when it arrived (engaged if it was the one that closed
  // it, already_held otherwise) and a release reports conflict, "not applied". The state of every robot is the one
  // that State would report now.
  std::vector<RobotResult> results() const;

  // Made by the domain.
  explicit Pending(std::shared_ptr<detail::Operation> operation) : operation_(std::move(operation)) {}

private:
  std::shared_ptr<detail::Operation> operation_;
};

class Domain {
public:
  using Listener = std::function<void()>;

  explicit Domain(DomainOptions options = {});
  ~Domain();
  Domain(const Domain &) = delete;
  Domain &operator=(const Domain &) = delete;

  const std::string &instance_id() const noexcept;
  // The domain clock. Hosts that stamp command receipt themselves (simulators: enqueue time) use it.
  TimeNs now() const;

  // Roster. add() returns false if the id is already in the roster; it throws std::invalid_argument for an
  // invalid id and std::length_error beyond DomainOptions::max_robots. remove() returns false if the id is not
  // in the roster. A removed id keeps its HOLD state: adding it again (a re-created model) inherits it, with
  // stage gated until the next tick writes zero to the new device. Unknown or removed ids are not addressable
  // by engage, release, state or tick, and admit() refuses them.
  bool add(const std::string &robot_id, bool feedback = false);
  bool remove(const std::string &robot_id);

  // Unconditional and idempotent. Not held -> held, revision + 1, stage gated; held -> unchanged. Command
  // admission is closed before this returns, and State reports the robot as held and gated at once (with the
  // revision the tick will assign); the machine itself follows with the next tick.
  Pending engage(const std::vector<std::string> &robot_ids);
  Pending engage_all();

  // Compare and set, decided by the machine of each robot when the tick applies it. expected_instance must equal
  // instance_id(), otherwise every change is a conflict. Per robot: not held -> not_held; a host fault or
  // expected_revision != revision -> conflict; otherwise released: revision + 1, stage released, release fence =
  // the time the tick applied it, which is also when admission opens again. A release that the tick has not
  // applied by deadline_ns (domain clock) is not applied at all (expired); no deadline: it waits for the tick.
  Pending release(const std::string &expected_instance, const std::vector<ReleaseChange> &changes,
                  std::optional<TimeNs> deadline_ns = std::nullopt);

  // Read-only snapshot; no ids = every robot in the roster.
  Snapshot state(const std::vector<std::string> &robot_ids = {}) const;
  Description describe() const;

  // Host command path, for every motion command the host receives. False if the robot is unknown, held (an
  // engage that the tick has not applied yet counts), or the command was received before the last release
  // (receipt_time_ns < release fence): it must not be replayed. receipt_time_ns is the time the command was
  // received, not the time it is dispatched. Wait free apart from the roster lookup.
  bool admit(const std::string &robot_id, TimeNs receipt_time_ns) const;

  // Host native tick, every control cycle on the thread that owns device output (after the command callbacks
  // of the cycle). It runs the state machines (their update() belongs to this thread: always call tick from the
  // same one, std::logic_error otherwise), writes zero for every held robot, every tick, not only once, and
  // applies the results. The first successful write after an engage advances the stage to zero_written; a failed
  // write puts it back to (or keeps it at) gated. The sink runs without domain locks, so a slow device never
  // delays engage, admit or state.
  TickStats tick(ZeroSink &sink);

  // Optional velocity feedback of the host (m/s, rad/s) taken at stamp_ns. Only samples of a held robot that
  // reports feedback, at or after its zero write and in stamp order, count. Returns whether the sample counted.
  // Samples further apart than stopped_dwell break the continuity of the at-rest run. A moving sample puts
  // stopped back to zero_written. Hosts without trusted feedback never report stopped.
  bool observe(const std::string &robot_id, double linear_speed, double angular_speed, TimeNs stamp_ns);

  // Unrecoverable device fault: the robot is engaged if it is not held, stays held (stage gated, the zero is not
  // trusted) and cannot be released while the fault is set, and the text is reported in state. An empty message
  // clears the fault. False for an id that is not in the roster. The host decides whether to exit (drivers do,
  // so systemd restarts them).
  bool set_fault(const std::string &robot_id, const std::string &message);

  // Called, without domain locks, from the thread that applied events, changed the roster or a fault (tick,
  // remove). Service uses it to complete pending replies. An invocation may still be running on another thread
  // when remove_listener returns: a listener must only touch state that outlives its registration.
  std::uint64_t add_listener(Listener listener);
  void remove_listener(std::uint64_t token);

private:
  std::unique_ptr<detail::DomainImpl> impl_;
};

}  // namespace xgc2::chassis_hold
