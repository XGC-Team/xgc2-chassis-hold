#include <xgc2/chassis_hold/domain.hpp>

#include "domain_impl.hpp"

#include <cmath>
#include <cstdio>
#include <exception>
#include <random>
#include <stdexcept>
#include <utility>

namespace xgc2::chassis_hold {
namespace {

std::string random_instance_id() {
  std::random_device device;
  std::string id;
  do {
    id.clear();
    for (int word = 0; word < 4; ++word) {
      char hex[9];
      std::snprintf(hex, sizeof hex, "%08x", static_cast<unsigned>(device()));
      id += hex;
    }
  } while (id == std::string(32, '0'));
  return id;
}

// later - earlier for later >= earlier, without signed overflow for stamps far apart.
std::uint64_t elapsed_ns(TimeNs later, TimeNs earlier) {
  return static_cast<std::uint64_t>(later) - static_cast<std::uint64_t>(earlier);
}

TimeNs steady_now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::int64_t as_payload(std::uint64_t value) { return static_cast<std::int64_t>(value); }

}  // namespace

std::string clean_text(const std::string &text) {
  std::string out;
  for (const char ch : text) {
    if (out.size() == max_text_bytes)
      break;
    const unsigned char c = static_cast<unsigned char>(ch);
    out.push_back(c >= 0x20 && c < 0x7f ? ch : '?');
  }
  return out;
}

const char *to_string(Stage stage) noexcept {
  switch (stage) {
    case Stage::released: return "released";
    case Stage::gated: return "gated";
    case Stage::zero_written: return "zero_written";
    case Stage::stopped: return "stopped";
  }
  return "unknown";
}

const char *to_string(Outcome outcome) noexcept {
  switch (outcome) {
    case Outcome::engaged: return "engaged";
    case Outcome::already_held: return "already_held";
    case Outcome::released: return "released";
    case Outcome::conflict: return "conflict";
    case Outcome::not_held: return "not_held";
    case Outcome::unknown_robot: return "unknown_robot";
    case Outcome::expired: return "expired";
  }
  return "unknown";
}

bool valid_robot_id(const std::string &id) noexcept {
  if (id.empty() || id.size() > 128)
    return false;
  for (const char c : id)
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '.' || c == '_' ||
          c == ':' || c == '-'))
      return false;
  return true;
}

bool valid_instance_id(const std::string &id) noexcept {
  if (id.size() != 32)
    return false;
  for (const char c : id)
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')))
      return false;
  return true;
}

namespace detail {

DomainImpl::DomainImpl(DomainOptions given) : options(std::move(given)) {
  if (!options.clock)
    options.clock = steady_now;
  if (options.instance_id.empty())
    options.instance_id = random_instance_id();
  else if (!valid_instance_id(options.instance_id) || options.instance_id == std::string(32, '0'))
    throw std::invalid_argument("instance_id must be 32 lowercase hex digits and not zero");
  if (!(options.linear_eps >= 0) || !(options.angular_eps >= 0) || options.stopped_dwell.count() < 0 ||
      options.stopped_dwell > std::chrono::hours(1))
    throw std::invalid_argument("stopped thresholds must be between 0 and 1 hour");
  dwell_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(options.stopped_dwell).count());
  start = options.clock();
}

Entry *DomainImpl::find_active(const std::string &id) const {
  const auto found = robots.find(id);
  if (found == robots.end())
    return nullptr;
  std::lock_guard<std::mutex> lock(found->second->mutex);
  return found->second->active ? found->second.get() : nullptr;
}

RobotState DomainImpl::view(Entry &entry) const {
  std::lock_guard<std::mutex> lock(entry.mutex);
  RobotState state = entry.state;
  const std::uint32_t gate = entry.gate.load(std::memory_order_acquire);
  if (!entry.broken.empty()) {
    state.held = true;
    state.stage = Stage::gated;
    if (state.fault.empty())
      state.fault = "hold state machine failed: " + entry.broken;
  } else if (!state.held && (gate & ~kBrokenBit) >= kPendingUnit) {
    // An engage has closed the gate and the tick has not applied it: the robot is held, with the revision the
    // machine will give it.
    state.held = true;
    ++state.revision;
    state.stage = Stage::gated;
    state.gated_at = entry.pending_since;
  }
  return state;
}

bool DomainImpl::post(Entry &entry, sm::EventId id, sm::EventPayload payload) {
  sm::Event event(id);
  event.payload = std::move(payload);
  return entry.machine->postEvent(std::move(event)).ok();
}

void DomainImpl::fail_closed(Entry &entry, const std::string &why) {
  entry.gate.fetch_or(kBrokenBit, std::memory_order_acq_rel);
  std::lock_guard<std::mutex> lock(entry.mutex);
  if (entry.broken.empty())
    entry.broken = clean_text(why);
}

void DomainImpl::engage_entry(Entry &entry, const std::shared_ptr<Operation> &operation, Item *item) {
  std::lock_guard<std::mutex> order(entry.post_order);
  engage_ordered(entry, operation, item);
}

void DomainImpl::engage_ordered(Entry &entry, const std::shared_ptr<Operation> &operation, Item *item) {
  const TimeNs at = now();
  std::int64_t ticket = 0;
  {
    std::lock_guard<std::mutex> lock(entry.mutex);
    // The gate closes here, before anything else happens: from now on admit() refuses.
    const std::uint32_t previous = entry.gate.fetch_add(kPendingUnit, std::memory_order_acq_rel);
    const bool closed_it = previous == 0;
    if (closed_it)
      entry.pending_since = at;
    if (item) {
      item->provisional = closed_it ? Outcome::engaged : Outcome::already_held;
      operation->unresolved.fetch_add(1, std::memory_order_acq_rel);
      ticket = entry.add_ticket(operation, item);
    }
  }
  if (!post(entry, ev::Engage, {{"ticket", ticket}, {"at", static_cast<std::int64_t>(at)}})) {
    // The machine cannot take the event: the gate stays closed for good and the caller gets what the gate decided.
    fail_closed(entry, "the event queue of the machine is full");
    std::lock_guard<std::mutex> lock(entry.mutex);
    entry.resolve(ticket, item ? item->provisional : Outcome::engaged);
  }
}

bool DomainImpl::drain(Entry &entry) {
  const auto wrong_thread = [](const sm::Status &status) {
    if (status.code == sm::ErrorCode::kWrongOwnerThread)
      throw std::logic_error("Domain::tick must always run on the same thread");
  };
  // The entry mutex is held around the machine's calls: its callbacks change the published state of the robot, and
  // a reader (state, admit, a transport) must never see half a transition. Nothing in a callback waits for a thread.
  if (!entry.started) {
    sm::Status status;
    {
      std::lock_guard<std::mutex> lock(entry.mutex);
      status = entry.machine->start();
    }
    wrong_thread(status);
    if (!status.ok()) {
      fail_closed(entry, status.message);
      return false;
    }
    entry.started = true;
  }
  bool progressed = false;
  for (int round = 0; round < 64; ++round) {
    sm::Result<sm::UpdateResult> result;
    {
      std::lock_guard<std::mutex> lock(entry.mutex);
      result = entry.machine->update(sm::UpdateOptions{1, 16, false});
    }
    wrong_thread(result.status);
    if (!result.ok()) {
      fail_closed(entry, result.status.message);
      return progressed;
    }
    if (result.value.faults_recorded > 0)
      fail_closed(entry, "a callback of the machine failed");
    if (result.value.events_taken > 0)
      progressed = true;
    if (result.value.events_remaining == 0)
      break;
  }
  return progressed;
}

void DomainImpl::notify() {
  std::vector<std::shared_ptr<const Domain::Listener>> called;
  {
    std::lock_guard<std::mutex> lock(listener_mutex);
    for (const auto &listener : listeners)
      called.push_back(listener.second);
  }
  for (const auto &listener : called)
    if (*listener)
      (*listener)();
}

void DomainImpl::wake() noexcept {
  if (!options.wake)
    return;
  try {
    options.wake();
  } catch (...) {
    // The operation that called this has taken effect; the host's wake-up is not its business.
  }
}

}  // namespace detail

bool Pending::done() const { return !operation_ || operation_->unresolved.load(std::memory_order_acquire) == 0; }

std::vector<RobotResult> Pending::results() const {
  std::vector<RobotResult> results;
  if (!operation_)
    return results;
  for (const auto &item : operation_->items) {
    RobotResult result;
    result.robot_id = item->robot_id;
    const int resolved = item->resolved.load(std::memory_order_acquire);
    result.outcome = resolved >= 0 ? static_cast<Outcome>(resolved) : item->provisional;
    if (item->entry)
      result.state = operation_->domain->view(*item->entry);
    results.push_back(std::move(result));
  }
  return results;
}

Domain::Domain(DomainOptions options) : impl_(std::make_unique<detail::DomainImpl>(std::move(options))) {}
Domain::~Domain() = default;

const std::string &Domain::instance_id() const noexcept { return impl_->options.instance_id; }
TimeNs Domain::now() const { return impl_->now(); }

bool detail::DomainImpl::add_robot(const std::string &robot_id, bool feedback, bool &needs_tick) {
  std::unique_lock<std::shared_mutex> roster(roster_mutex);
  const auto found = robots.find(robot_id);
  if (found == robots.end()) {
    if (robots.size() >= options.max_robots)
      throw std::length_error("chassis hold roster is full");
    auto entry = std::make_unique<detail::Entry>(*this, robot_id);
    entry->machine = detail::make_hold_machine(*entry);
    entry->active = true;
    entry->binding = 1;
    entry->state.feedback = feedback;
    detail::Entry *added = entry.get();
    robots.emplace(robot_id, std::move(entry));
    // The boot hold is an engage that nobody waits for: the first tick applies it (revision 1, stage gated).
    if (options.boot_held) {
      engage_entry(*added, nullptr, nullptr);
      needs_tick = true;
    }
    return true;
  }
  detail::Entry &entry = *found->second;
  const TimeNs at = now();
  std::uint64_t binding = 0;
  bool rebind = false;
  {
    std::lock_guard<std::mutex> lock(entry.mutex);
    if (entry.active)
      return false;
    entry.active = true;
    binding = ++entry.binding;
    entry.state.feedback = feedback;
    // Only a hold has anything to learn about the new device: one the machine applied, or an engage on its way.
    rebind = entry.state.held || (entry.gate.load(std::memory_order_acquire) & ~kBrokenBit) >= kPendingUnit;
    if (entry.state.held) {
      // A held robot comes back with a new device: no zero yet, and the fault belonged to the old one. The machine
      // follows with the rebound event; State must not show the old device's progress in the meantime.
      entry.state.stage = Stage::gated;
      entry.state.gated_at = at;
      entry.state.fault.clear();
      entry.state.zero_written_at.reset();
      entry.state.stopped_at.reset();
      entry.state.write_error.clear();
      entry.rest_since.reset();
      entry.last_sample.reset();
    }
  }
  if (rebind) {
    post(entry, ev::Rebound, {{"binding", as_payload(binding)}, {"at", static_cast<std::int64_t>(at)}});
    needs_tick = true;
  }
  return true;
}

bool Domain::add(const std::string &robot_id, bool feedback) {
  if (!valid_robot_id(robot_id))
    throw std::invalid_argument("invalid robot id \"" + clean_text(robot_id) + "\"");
  bool needs_tick = false;
  const bool added = impl_->add_robot(robot_id, feedback, needs_tick);
  if (needs_tick)
    impl_->wake();
  return added;
}

bool Domain::remove(const std::string &robot_id) {
  {
    std::unique_lock<std::shared_mutex> roster(impl_->roster_mutex);
    const auto found = impl_->robots.find(robot_id);
    if (found == impl_->robots.end())
      return false;
    std::lock_guard<std::mutex> lock(found->second->mutex);
    if (!found->second->active)
      return false;
    found->second->active = false;
  }
  impl_->notify();  // pending replies stop waiting for a robot that is gone
  return true;
}

Pending Domain::engage(const std::vector<std::string> &robot_ids) {
  auto operation = std::make_shared<detail::Operation>();
  operation->domain = impl_.get();
  bool needs_tick = false;
  {
    std::shared_lock<std::shared_mutex> roster(impl_->roster_mutex);
    for (const auto &id : robot_ids) {
      auto item = std::make_unique<detail::Item>();
      item->robot_id = id;
      detail::Item *raw = item.get();
      operation->items.push_back(std::move(item));
      if (detail::Entry *entry = impl_->find_active(id)) {
        raw->entry = entry;
        impl_->engage_entry(*entry, operation, raw);
        needs_tick = true;
      } else {
        raw->resolved.store(static_cast<int>(Outcome::unknown_robot));
      }
    }
  }
  if (needs_tick)
    impl_->wake();
  return Pending(std::move(operation));
}

Pending Domain::engage_all() {
  std::vector<std::string> ids;
  {
    std::shared_lock<std::shared_mutex> roster(impl_->roster_mutex);
    for (const auto &item : impl_->robots)
      if (impl_->find_active(item.first))
        ids.push_back(item.first);
  }
  return engage(ids);
}

Pending Domain::release(const std::string &expected_instance, const std::vector<ReleaseChange> &changes,
                        std::optional<TimeNs> deadline_ns) {
  auto operation = std::make_shared<detail::Operation>();
  operation->domain = impl_.get();
  bool needs_tick = false;
  {
    std::shared_lock<std::shared_mutex> roster(impl_->roster_mutex);
    for (const auto &change : changes) {
      auto item = std::make_unique<detail::Item>();
      item->robot_id = change.robot_id;
      detail::Item *raw = item.get();
      operation->items.push_back(std::move(item));
      detail::Entry *entry = impl_->find_active(change.robot_id);
      if (!entry) {
        raw->resolved.store(static_cast<int>(Outcome::unknown_robot));
        continue;
      }
      raw->entry = entry;
      raw->provisional = Outcome::conflict;  // not applied
      std::int64_t ticket = 0;
      {
        std::lock_guard<std::mutex> lock(entry->mutex);
        operation->unresolved.fetch_add(1, std::memory_order_acq_rel);
        ticket = entry->add_ticket(operation, raw);
      }
      sm::EventPayload payload;
      payload["ticket"] = ticket;
      payload["instance"] = expected_instance;
      payload["revision"] = as_payload(change.expected_revision);
      payload["deadline"] = deadline_ns ? static_cast<std::int64_t>(*deadline_ns) : no_deadline;
      bool posted = false;
      {
        std::lock_guard<std::mutex> order(entry->post_order);
        posted = impl_->post(*entry, ev::Release, std::move(payload));
      }
      if (posted) {
        needs_tick = true;
      } else {
        impl_->fail_closed(*entry, "the event queue of the machine is full");
        std::lock_guard<std::mutex> lock(entry->mutex);
        entry->resolve(ticket, Outcome::conflict);
      }
    }
  }
  if (needs_tick)
    impl_->wake();
  return Pending(std::move(operation));
}

Snapshot Domain::state(const std::vector<std::string> &robot_ids) const {
  Snapshot snapshot;
  snapshot.instance = impl_->options.instance_id;
  snapshot.start = impl_->start;
  std::shared_lock<std::shared_mutex> roster(impl_->roster_mutex);
  if (robot_ids.empty()) {
    for (const auto &item : impl_->robots)
      if (impl_->find_active(item.first))
        snapshot.robots.push_back(impl_->view(*item.second));
    return snapshot;
  }
  for (const auto &id : robot_ids) {
    if (detail::Entry *entry = impl_->find_active(id))
      snapshot.robots.push_back(impl_->view(*entry));
    else
      snapshot.unknown.push_back(id);
  }
  return snapshot;
}

Description Domain::describe() const {
  Description description;
  description.instance = impl_->options.instance_id;
  std::shared_lock<std::shared_mutex> roster(impl_->roster_mutex);
  for (const auto &item : impl_->robots) {
    detail::Entry &entry = *item.second;
    std::lock_guard<std::mutex> lock(entry.mutex);
    if (entry.active)
      description.robots.push_back({item.first, entry.state.feedback});
  }
  return description;
}

bool Domain::admit(const std::string &robot_id, TimeNs receipt_time_ns) const {
  std::shared_lock<std::shared_mutex> roster(impl_->roster_mutex);
  const detail::Entry *entry = impl_->find_active(robot_id);
  return entry && entry->gate.load(std::memory_order_acquire) == 0 &&
         !(receipt_time_ns < entry->fence.load(std::memory_order_acquire));
}

TickStats Domain::tick(ZeroSink &sink) {
  std::vector<detail::Entry *> entries;
  {
    std::shared_lock<std::shared_mutex> roster(impl_->roster_mutex);
    for (const auto &item : impl_->robots)
      if (impl_->find_active(item.first))
        entries.push_back(item.second.get());
  }
  // The machines apply what arrived since the last tick.
  bool progressed = false;
  for (detail::Entry *entry : entries)
    progressed = impl_->drain(*entry) || progressed;

  struct Target {
    Target(detail::Entry *entry_in, std::uint64_t binding_in, std::uint64_t revision_in, Stage stage_in, bool faulted_in)
        : entry(entry_in), binding(binding_in), revision(revision_in), stage(stage_in), faulted(faulted_in) {}
    detail::Entry *entry;
    std::uint64_t binding;
    std::uint64_t revision;
    Stage stage;
    bool faulted = false;
    bool ok = false;
    std::string error;
  };
  std::vector<Target> targets;
  for (detail::Entry *entry : entries) {
    std::lock_guard<std::mutex> lock(entry->mutex);
    if (entry->active && (entry->state.held || !entry->broken.empty()))
      targets.emplace_back(entry, entry->binding, entry->state.revision, entry->state.stage, !entry->state.fault.empty());
  }

  TickStats stats;
  // No lock while the device is written: a slow or stuck device must not delay engage, admit or state.
  for (auto &target : targets) {
    try {
      target.ok = sink.write_zero(target.entry->id, target.error);
    } catch (const std::exception &e) {
      target.ok = false;
      target.error = e.what();
    } catch (...) {
      target.ok = false;
      target.error = "unknown exception";
    }
    if (target.ok)
      ++stats.written;
    else
      ++stats.failed;
  }

  // What the writes did is an event of the machine like any other. A confirmation that changes nothing (the zero
  // was already written) is not posted.
  std::vector<detail::Entry *> reported;
  for (const auto &target : targets) {
    if (target.entry->gate.load(std::memory_order_acquire) & kBrokenBit)
      continue;  // the machine failed: the robot stays held and nothing follows
    sm::EventPayload payload;
    payload["binding"] = as_payload(target.binding);
    payload["revision"] = as_payload(target.revision);
    sm::EventId id = 0;
    if (!target.ok) {
      id = ev::ZeroFailed;
      payload["error"] = target.error.empty() ? std::string("zero write failed") : target.error;
    } else if (target.stage == Stage::gated && !target.faulted) {
      id = ev::ZeroWritten;
    }
    if (id != 0 && impl_->post(*target.entry, id, std::move(payload)))
      reported.push_back(target.entry);
  }
  for (detail::Entry *entry : reported)
    progressed = impl_->drain(*entry) || progressed;
  if (progressed)
    impl_->notify();
  return stats;
}

bool Domain::observe(const std::string &robot_id, double linear_speed, double angular_speed, TimeNs stamp_ns) {
  std::shared_lock<std::shared_mutex> roster(impl_->roster_mutex);
  detail::Entry *entry = impl_->find_active(robot_id);
  if (!entry)
    return false;
  sm::EventId report = 0;
  std::uint64_t binding = 0;
  std::uint64_t revision = 0;
  {
    std::lock_guard<std::mutex> lock(entry->mutex);
    RobotState &state = entry->state;
    if (!state.feedback || !state.held || !entry->broken.empty())
      return false;
    // Evidence needs the zero to be asserted, a sample taken after it was written, and stamp order.
    if (state.stage != Stage::zero_written && state.stage != Stage::stopped)
      return false;
    if (stamp_ns < *state.zero_written_at || (entry->last_sample && stamp_ns < *entry->last_sample))
      return false;
    // NaN compares false: a sample that is not a number is not at rest.
    const bool at_rest = std::fabs(linear_speed) <= impl_->options.linear_eps &&
                         std::fabs(angular_speed) <= impl_->options.angular_eps;
    if (entry->last_sample && elapsed_ns(stamp_ns, *entry->last_sample) > impl_->dwell_ns)
      entry->rest_since.reset();
    entry->last_sample = stamp_ns;
    if (!at_rest) {
      entry->rest_since.reset();
      if (state.stage == Stage::stopped)
        report = ev::MotionResumed;
    } else {
      if (!entry->rest_since)
        entry->rest_since = stamp_ns;
      if (state.stage == Stage::zero_written && elapsed_ns(stamp_ns, *entry->rest_since) >= impl_->dwell_ns)
        report = ev::MotionStopped;
    }
    binding = entry->binding;
    revision = state.revision;
  }
  bool posted = false;
  if (report != 0)
    posted = impl_->post(*entry, report, {{"binding", as_payload(binding)}, {"revision", as_payload(revision)}});
  roster.unlock();
  if (posted)
    impl_->wake();
  return true;
}

bool Domain::set_fault(const std::string &robot_id, const std::string &message) {
  {
    std::shared_lock<std::shared_mutex> roster(impl_->roster_mutex);
    detail::Entry *entry = impl_->find_active(robot_id);
    if (!entry)
      return false;
    const std::string fault = clean_text(message);
    if (fault.empty()) {
      impl_->post(*entry, ev::ClearFault, {});
    } else {
      // A fault holds the robot: the engage and the fault are one step in the queue, so that no release can come
      // between them and leave the robot released with a fault.
      std::lock_guard<std::mutex> order(entry->post_order);
      impl_->engage_ordered(*entry, nullptr, nullptr);
      impl_->post(*entry, ev::Fault, {{"text", fault}});
    }
  }
  impl_->wake();
  return true;
}

std::uint64_t Domain::add_listener(Listener listener) {
  std::lock_guard<std::mutex> lock(impl_->listener_mutex);
  const std::uint64_t token = impl_->next_token++;
  impl_->listeners.emplace_back(token, std::make_shared<const Listener>(std::move(listener)));
  return token;
}

void Domain::remove_listener(std::uint64_t token) {
  std::lock_guard<std::mutex> lock(impl_->listener_mutex);
  for (auto it = impl_->listeners.begin(); it != impl_->listeners.end(); ++it) {
    if (it->first == token) {
      impl_->listeners.erase(it);
      return;
    }
  }
}

}  // namespace xgc2::chassis_hold
