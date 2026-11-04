#pragma once
// Internals of Domain: the roster entries with their state machines (hold_machine.cpp) and the operations.
#include "text.hpp"

#include <xgc2/chassis_hold/domain.hpp>

#include "hold_state.hpp"

#include <deque>

#include <atomic>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <unordered_map>

namespace xgc2::chassis_hold {
constexpr TimeNs no_fence = std::numeric_limits<TimeNs>::min();

// The gate mirror of a robot: what command admission reads without taking a lock. Bit 0 says the machine holds the
// robot; each engage that arrived and that the machine has not applied yet counts 2; the top bit says the machine
// failed and the robot stays held. Admission is open only when the whole word is 0, so an engage closes it on
// arrival, and a release that the tick applies cannot open it while an engage is on its way.
constexpr std::uint32_t kHeldBit = 1;
constexpr std::uint32_t kPendingUnit = 2;
constexpr std::uint32_t kBrokenBit = 0x80000000u;

namespace detail {

struct DomainImpl;
struct Entry;

// One robot of one engage or release call.
struct Item {
  Entry *entry = nullptr;  // null: the robot is not in the roster
  std::string robot_id;
  std::int64_t ticket = 0;  // of a release: what Pending::withdraw takes back
  Outcome provisional = Outcome::unknown_robot;
  std::atomic<int> resolved{-1};  // the Outcome the machine decided, or -1
};

struct Operation {
  DomainImpl *domain = nullptr;
  std::vector<std::unique_ptr<Item>> items;
  std::atomic<std::size_t> unresolved{0};
};

struct Entry {
  Entry(DomainImpl &domain_in, std::string id_in) : domain(domain_in), id(std::move(id_in)) { published.robot_id = id; }
  Entry(const Entry &) = delete;
  Entry &operator=(const Entry &) = delete;

  // A call that waits for the machine: the ticket in the event leads back to it. Entry mutex held.
  std::int64_t add_ticket(const std::shared_ptr<Operation> &operation, Item *item);
  // The machine decided; wakes whoever waits on the operation. Entry mutex held; ticket 0 is nobody.
  void resolve(std::int64_t ticket, Outcome outcome);

  DomainImpl &domain;
  const std::string id;
  // Held while an operation that changes the hold takes its place in the gate and in the machine's inbox, so that
  // both see the same order: the engage that closes the gate is the one the machine applies first, and a fault
  // reported by the host cannot be separated from its engage by a release. Callbacks of the machine never take it.
  std::mutex post_order;
  std::atomic<std::uint32_t> gate{0};
  std::atomic<TimeNs> fence{no_fence};  // host-clock time of the last release
  std::atomic<bool> pending{false};       // the inbox may need an update
  // Only tick_thread accesses these values. Ingress never locks the reducer.
  HoldMachine machine{Released{}, 0};
  std::uint64_t revision = 0;
  std::uint64_t zero_write_failed = 0;

  // Short ingress/publication boundary. No reducer or host callback runs under it.
  mutable std::mutex mutex;
  RobotState published;  // output projection, never an input to the reducer
  FeedbackProgress *progress = nullptr;  // borrowed only while mutex is held
  std::deque<ev::Event> inbox;  // replaces the old engine inbox, same 4096 capacity
  std::optional<TimeNs> pending_rebind_at;  // immediate new-device fact, consumed by owner
  bool active = false;
  std::uint64_t binding = 0;          // increments on every (re)add: a zero write for an earlier device is stale
  TimeNs pending_since = 0;           // arrival of the oldest engage that the machine has not applied
  std::string broken;                 // the machine failed: the robot stays held
  std::int64_t next_ticket = 1;
  struct Ticket {
    std::shared_ptr<Operation> operation;
    Item *item = nullptr;
  };
  std::unordered_map<std::int64_t, Ticket> tickets;
};

struct DomainImpl {
  explicit DomainImpl(DomainOptions given);

  TimeNs now() const { return options.clock(); }
  // The entry of a robot in the roster, or null. The roster lock is held.
  Entry *find_active(const std::string &id) const;
  // The state as State reports it: what the machine applied, with an engage that arrived and has not been applied
  // yet shown as held and gated.
  RobotState view(Entry &entry) const;
  // An engage of one robot: closes the gate, posts the event. The roster lock is held, the entry mutex is not.
  // Without an item nobody waits for the outcome (the boot hold, a fault).
  void engage_entry(Entry &entry, const std::shared_ptr<Operation> &operation, Item *item);
  // The same with entry.post_order already held by the caller.
  void engage_ordered(Entry &entry, const std::shared_ptr<Operation> &operation, Item *item, TimeNs at);
  // Posts an event to the machine. No lock of the entry may be held. False if the machine refused it.
  bool post(Entry &entry, ev::Event event);
  // The robot stays held for good: the machine could not follow.
  void fail_closed(Entry &entry, const std::string &why);
  // Runs the machine until its inbox is empty (bounded); true if it took any event. Tick thread.
  bool drain(Entry &entry);
  void notify();
  // Tells the host that the tick has work (DomainOptions::wake). No lock of the domain may be held.
  void wake() noexcept;
  // The roster change behind Domain::add; needs_tick says whether the tick has an event to apply afterwards.
  bool add_robot(const std::string &robot_id, bool feedback, bool &needs_tick);

  DomainOptions options;
  TimeNs start = 0;
  std::uint64_t dwell_ns = 0;
  // Writers: add and remove. Readers: everything else.
  mutable std::shared_mutex roster_mutex;
  std::map<std::string, std::unique_ptr<Entry>> robots;  // never erased: removed ids keep their state

  // Only pending or held entries belong here. Never take an entry/roster lock while holding work_mutex;
  // callers that need both take the existing roster/entry locks first. No lock crosses a host callback.
  std::mutex work_mutex;
  std::map<std::string, Entry *> work;
  std::thread::id tick_thread;
  // Scratch storage belongs to the one tick thread; entries are never erased from robots.
  std::vector<Entry *> tick_entries;
  struct Target {
    Target(Entry *entry_in, std::uint64_t binding_in, std::uint64_t revision_in, Stage stage_in, bool faulted_in)
        : entry(entry_in), binding(binding_in), revision(revision_in), stage(stage_in), faulted(faulted_in) {}
    Entry *entry;
    std::uint64_t binding;
    std::uint64_t revision;
    Stage stage;
    bool faulted;
    bool ok = false;
    bool reported = false;
    std::string error;
  };
  std::vector<Target> tick_targets;

  std::mutex listener_mutex;
  std::vector<std::pair<std::uint64_t, std::shared_ptr<const Domain::Listener>>> listeners;
  std::uint64_t next_token = 1;
};

// Commit the existing atomic admission mirror; called at publication.
void apply_gate(Entry &entry, bool consume_engage, bool hold);

}  // namespace detail
}  // namespace xgc2::chassis_hold
