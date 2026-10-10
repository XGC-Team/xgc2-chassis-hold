#include <xgc2/chassis_hold/domain.hpp>

#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <map>
#include <mutex>
#include <random>
#include <shared_mutex>
#include <stdexcept>
#include <utility>

namespace xgc2::chassis_hold {
namespace {

constexpr TimeNs no_fence = std::numeric_limits<TimeNs>::min();
constexpr std::size_t max_text_bytes = 160;

// Host text (device errors, fault descriptions) ends up in JSON replies for remote callers: keep it printable
// ASCII and bounded.
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

}  // namespace

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

struct Domain::Impl {
  struct Entry {
    RobotState state;
    bool active = false;
    // Increments on every (re)add: a zero write that started for an earlier binding must not advance the stage
    // of the new one.
    std::uint64_t binding = 0;
    TimeNs fence = no_fence;  // host-clock time of the last release
    std::optional<TimeNs> rest_since;   // stamp of the first at-rest sample of the current run
    std::optional<TimeNs> last_sample;  // stamp of the last counted sample
  };

  explicit Impl(DomainOptions given) : options(std::move(given)) {
    if (!options.clock)
      options.clock = steady_now;
    if (options.instance_id.empty())
      options.instance_id = random_instance_id();
    else if (!valid_instance_id(options.instance_id))
      throw std::invalid_argument("instance_id must be 32 lowercase hex digits");
    if (!(options.linear_eps >= 0) || !(options.angular_eps >= 0) || options.stopped_dwell.count() < 0 ||
        options.stopped_dwell > std::chrono::hours(1))
      throw std::invalid_argument("stopped thresholds must be between 0 and 1 hour");
    dwell_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(options.stopped_dwell).count());
    start = options.clock();
  }

  Entry *find_active(const std::string &id) {
    const auto found = robots.find(id);
    return found != robots.end() && found->second.active ? &found->second : nullptr;
  }
  const Entry *find_active(const std::string &id) const {
    const auto found = robots.find(id);
    return found != robots.end() && found->second.active ? &found->second : nullptr;
  }

  // The state transitions. All callers hold the unique lock.
  static void clear_progress(Entry &entry) {
    entry.state.gated_at.reset();
    entry.state.zero_written_at.reset();
    entry.state.stopped_at.reset();
    entry.state.write_error.clear();
    entry.rest_since.reset();
    entry.last_sample.reset();
  }
  static void gate(Entry &entry, TimeNs at) {
    clear_progress(entry);
    entry.state.held = true;
    ++entry.state.revision;
    entry.state.stage = Stage::gated;
    entry.state.gated_at = at;
  }
  static void ungate(Entry &entry, TimeNs at) {
    clear_progress(entry);
    entry.state.held = false;
    ++entry.state.revision;
    entry.state.stage = Stage::released;
    entry.fence = at;
  }
  // A held robot whose zero is no longer confirmed: gated again, gated_at keeps the engage time.
  static void lose_zero(Entry &entry) {
    entry.state.stage = Stage::gated;
    entry.state.zero_written_at.reset();
    entry.state.stopped_at.reset();
    entry.rest_since.reset();
    entry.last_sample.reset();
  }

  RobotResult engage_entry(Entry &entry, TimeNs at) {
    RobotResult result;
    result.robot_id = entry.state.robot_id;
    if (entry.state.held) {
      result.outcome = Outcome::already_held;
    } else {
      gate(entry, at);
      result.outcome = Outcome::engaged;
    }
    result.state = entry.state;
    return result;
  }

  void notify() {
    std::vector<std::shared_ptr<const Listener>> called;
    {
      std::lock_guard<std::mutex> lock(listener_mutex);
      for (const auto &listener : listeners)
        called.push_back(listener.second);
    }
    for (const auto &listener : called)
      if (*listener)
        (*listener)();
  }

  DomainOptions options;
  TimeNs start = 0;
  std::uint64_t dwell_ns = 0;
  // Writers: engage, release, add, remove, observe, set_fault and the record step of tick. Readers: admit, state,
  // describe and the gather step of tick. Critical sections never call into the host except for the clock.
  mutable std::shared_mutex mutex;
  std::map<std::string, Entry> robots;  // never erased: removed ids keep their state

  std::mutex listener_mutex;
  std::vector<std::pair<std::uint64_t, std::shared_ptr<const Listener>>> listeners;
  std::uint64_t next_token = 1;
};

Domain::Domain(DomainOptions options) : impl_(std::make_unique<Impl>(std::move(options))) {}
Domain::~Domain() = default;

const std::string &Domain::instance_id() const noexcept { return impl_->options.instance_id; }
TimeNs Domain::now() const { return impl_->options.clock(); }

bool Domain::add(const std::string &robot_id, bool feedback) {
  if (!valid_robot_id(robot_id))
    throw std::invalid_argument("invalid robot id");
  std::unique_lock<std::shared_mutex> lock(impl_->mutex);
  const TimeNs at = impl_->options.clock();
  auto found = impl_->robots.find(robot_id);
  if (found == impl_->robots.end()) {
    if (impl_->robots.size() >= impl_->options.max_robots)
      throw std::length_error("chassis hold roster is full");
    Impl::Entry &entry = impl_->robots[robot_id];
    entry.state.robot_id = robot_id;
    entry.state.feedback = feedback;
    entry.active = true;
    ++entry.binding;
    if (impl_->options.boot_held)
      Impl::gate(entry, at);
    return true;
  }
  Impl::Entry &entry = found->second;
  if (entry.active)
    return false;
  entry.active = true;
  ++entry.binding;
  entry.state.feedback = feedback;
  entry.state.fault.clear();  // the fault belonged to the device that was removed
  if (entry.state.held) {
    // Inherits HOLD; the new device has no zero yet.
    Impl::clear_progress(entry);
    entry.state.stage = Stage::gated;
    entry.state.gated_at = at;
  }
  return true;
}

bool Domain::remove(const std::string &robot_id) {
  {
    std::unique_lock<std::shared_mutex> lock(impl_->mutex);
    Impl::Entry *entry = impl_->find_active(robot_id);
    if (!entry)
      return false;
    entry->active = false;
  }
  impl_->notify();  // pending engage replies stop waiting for a robot that is gone
  return true;
}

std::vector<RobotResult> Domain::engage(const std::vector<std::string> &robot_ids) {
  std::vector<RobotResult> results;
  results.reserve(robot_ids.size());
  std::unique_lock<std::shared_mutex> lock(impl_->mutex);
  const TimeNs at = impl_->options.clock();
  for (const auto &id : robot_ids) {
    if (Impl::Entry *entry = impl_->find_active(id)) {
      results.push_back(impl_->engage_entry(*entry, at));
    } else {
      RobotResult unknown;
      unknown.robot_id = id;
      results.push_back(std::move(unknown));
    }
  }
  return results;
}

std::vector<RobotResult> Domain::engage_all() {
  std::vector<RobotResult> results;
  std::unique_lock<std::shared_mutex> lock(impl_->mutex);
  const TimeNs at = impl_->options.clock();
  for (auto &item : impl_->robots)
    if (item.second.active)
      results.push_back(impl_->engage_entry(item.second, at));
  return results;
}

std::vector<RobotResult> Domain::release(const std::string &expected_instance,
                                         const std::vector<ReleaseChange> &changes) {
  std::vector<RobotResult> results;
  results.reserve(changes.size());
  bool released_any = false;
  {
    std::unique_lock<std::shared_mutex> lock(impl_->mutex);
    const bool current_instance = expected_instance == impl_->options.instance_id;
    const TimeNs at = impl_->options.clock();
    for (const auto &change : changes) {
      RobotResult result;
      result.robot_id = change.robot_id;
      if (Impl::Entry *entry = impl_->find_active(change.robot_id)) {
        const RobotState &state = entry->state;
        if (!current_instance) {
          result.outcome = Outcome::conflict;
        } else if (!state.held) {
          result.outcome = Outcome::not_held;
        } else if (!state.fault.empty() || change.expected_revision != state.revision) {
          result.outcome = Outcome::conflict;
        } else {
          Impl::ungate(*entry, at);
          result.outcome = Outcome::released;
          released_any = true;
        }
        result.state = entry->state;
      }
      results.push_back(std::move(result));
    }
  }
  if (released_any)
    impl_->notify();
  return results;
}

Snapshot Domain::state(const std::vector<std::string> &robot_ids) const {
  Snapshot snapshot;
  snapshot.instance = impl_->options.instance_id;
  snapshot.start = impl_->start;
  std::shared_lock<std::shared_mutex> lock(impl_->mutex);
  if (robot_ids.empty()) {
    for (const auto &item : impl_->robots)
      if (item.second.active)
        snapshot.robots.push_back(item.second.state);
    return snapshot;
  }
  for (const auto &id : robot_ids) {
    if (const Impl::Entry *entry = impl_->find_active(id))
      snapshot.robots.push_back(entry->state);
    else
      snapshot.unknown.push_back(id);
  }
  return snapshot;
}

Description Domain::describe() const {
  Description description;
  description.instance = impl_->options.instance_id;
  std::shared_lock<std::shared_mutex> lock(impl_->mutex);
  for (const auto &item : impl_->robots)
    if (item.second.active)
      description.robots.push_back({item.first, item.second.state.feedback});
  return description;
}

bool Domain::admit(const std::string &robot_id, TimeNs receipt_time_ns) const {
  std::shared_lock<std::shared_mutex> lock(impl_->mutex);
  const Impl::Entry *entry = impl_->find_active(robot_id);
  return entry && !entry->state.held && !(receipt_time_ns < entry->fence);
}

TickStats Domain::tick(ZeroSink &sink) {
  struct Target {
    Target(std::string id_in, std::uint64_t revision_in, std::uint64_t binding_in)
        : id(std::move(id_in)), revision(revision_in), binding(binding_in) {}
    std::string id;
    std::uint64_t revision;
    std::uint64_t binding;
    bool ok = false;
    std::string error;
  };
  std::vector<Target> targets;
  {
    std::shared_lock<std::shared_mutex> lock(impl_->mutex);
    for (const auto &item : impl_->robots)
      if (item.second.active && item.second.state.held)
        targets.emplace_back(item.first, item.second.state.revision, item.second.binding);
  }
  TickStats stats;
  if (targets.empty())
    return stats;
  // No domain lock while the device is written: a slow or stuck device must not delay engage or admit.
  for (auto &target : targets) {
    try {
      target.ok = sink.write_zero(target.id, target.error);
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
  bool changed = false;
  {
    std::unique_lock<std::shared_mutex> lock(impl_->mutex);
    const TimeNs at = impl_->options.clock();
    for (const auto &target : targets) {
      Impl::Entry &entry = impl_->robots.at(target.id);
      RobotState &state = entry.state;
      // Released, re-engaged, removed or re-added while the device was written: the result is not about the
      // current hold.
      if (!entry.active || !state.held || state.revision != target.revision || entry.binding != target.binding)
        continue;
      if (target.ok) {
        state.write_error.clear();
        if (state.stage == Stage::gated) {
          state.stage = Stage::zero_written;
          state.zero_written_at = at;
          changed = true;
        }
      } else {
        ++state.zero_write_failed;
        state.write_error = clean_text(target.error.empty() ? "zero write failed" : target.error);
        if (state.stage != Stage::gated) {
          Impl::lose_zero(entry);
          changed = true;
        }
      }
    }
  }
  if (changed)
    impl_->notify();
  return stats;
}

bool Domain::observe(const std::string &robot_id, double linear_speed, double angular_speed, TimeNs stamp_ns) {
  bool counted = false;
  bool changed = false;
  {
    std::unique_lock<std::shared_mutex> lock(impl_->mutex);
    Impl::Entry *entry = impl_->find_active(robot_id);
    if (!entry || !entry->state.feedback || !entry->state.held)
      return false;
    RobotState &state = entry->state;
    // Evidence needs the zero to be asserted, a sample taken after it was written, and stamp order.
    if (state.stage != Stage::zero_written && state.stage != Stage::stopped)
      return false;
    if (stamp_ns < *state.zero_written_at || (entry->last_sample && stamp_ns < *entry->last_sample))
      return false;
    counted = true;
    // NaN compares false: a sample that is not a number is not at rest.
    const bool at_rest = std::fabs(linear_speed) <= impl_->options.linear_eps &&
                         std::fabs(angular_speed) <= impl_->options.angular_eps;
    if (entry->last_sample && elapsed_ns(stamp_ns, *entry->last_sample) > impl_->dwell_ns)
      entry->rest_since.reset();
    entry->last_sample = stamp_ns;
    if (!at_rest) {
      entry->rest_since.reset();
      if (state.stage == Stage::stopped) {
        state.stage = Stage::zero_written;
        state.stopped_at.reset();
        changed = true;
      }
    } else {
      if (!entry->rest_since)
        entry->rest_since = stamp_ns;
      if (state.stage == Stage::zero_written && elapsed_ns(stamp_ns, *entry->rest_since) >= impl_->dwell_ns) {
        state.stage = Stage::stopped;
        state.stopped_at = impl_->options.clock();
        changed = true;
      }
    }
  }
  if (changed)
    impl_->notify();
  return counted;
}

bool Domain::set_fault(const std::string &robot_id, const std::string &message) {
  bool changed = false;
  {
    std::unique_lock<std::shared_mutex> lock(impl_->mutex);
    Impl::Entry *entry = impl_->find_active(robot_id);
    if (!entry)
      return false;
    const std::string text = clean_text(message);
    if (!text.empty() && !entry->state.held)
      Impl::gate(*entry, impl_->options.clock());
    changed = entry->state.fault != text;
    entry->state.fault = text;
  }
  if (changed)
    impl_->notify();
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
