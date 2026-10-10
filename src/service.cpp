#include <xgc2/chassis_hold/service.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <condition_variable>
#include <future>
#include <initializer_list>
#include <list>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

// Only the API subset shared by nlohmann-json 2.1.1 (Bionic) and 3.x is used: parse, find, iterators, is_*,
// get<T>, operator[], push_back, dump; every parse failure is caught as std::exception.
namespace xgc2::chassis_hold {
namespace {

using json = nlohmann::json;
using Clock = Service::Clock;

constexpr std::size_t max_pending_replies = 256;
constexpr int max_json_depth = 4;  // object -> array -> object is the deepest request

const char *const capabilities[] = {"engage", "release", "state"};

struct RequestError {
  Status status;
  std::string message;
};

[[noreturn]] void invalid(const std::string &message) { throw RequestError{Status::invalid_argument, message}; }

// Request text that is echoed in a message: printable ASCII, short.
std::string printable(const std::string &text) {
  std::string out;
  for (const char ch : text) {
    if (out.size() == 32)
      break;
    const unsigned char c = static_cast<unsigned char>(ch);
    out.push_back(c >= 0x20 && c < 0x7f ? ch : '?');
  }
  return out;
}

// The JSON parser is recursive: bound the nesting before it sees the text.
bool nesting_ok(const std::string &text) {
  int depth = 0;
  bool in_string = false;
  bool escaped = false;
  for (const char c : text) {
    if (in_string) {
      if (escaped)
        escaped = false;
      else if (c == '\\')
        escaped = true;
      else if (c == '"')
        in_string = false;
    } else if (c == '"') {
      in_string = true;
    } else if (c == '{' || c == '[') {
      if (++depth > max_json_depth)
        return false;
    } else if (c == '}' || c == ']') {
      if (depth > 0)
        --depth;
    }
  }
  return true;
}

json parse_object(const std::string &body, const ServiceOptions &options) {
  if (body.size() > options.max_body_bytes)
    invalid("request body exceeds " + std::to_string(options.max_body_bytes) + " bytes");
  if (body.find_first_not_of(" \t\r\n") == std::string::npos)
    return json::object();
  if (!nesting_ok(body))
    invalid("request nesting is too deep");
  json parsed;
  try {
    parsed = json::parse(body);
  } catch (const std::exception &) {
    invalid("request body is not valid JSON");
  }
  if (!parsed.is_object())
    invalid("request body must be a JSON object");
  return parsed;
}

void check_fields(const json &object, std::initializer_list<const char *> allowed) {
  for (auto it = object.begin(); it != object.end(); ++it) {
    const std::string key = it.key();
    bool known = false;
    for (const char *name : allowed)
      known = known || key == name;
    if (!known)
      invalid("unknown field \"" + printable(key) + "\"");
  }
}

std::string robot_id_of(const json &value, const char *field) {
  if (!value.is_string())
    invalid(std::string(field) + " must be a string");
  const std::string id = value.get<std::string>();
  if (!valid_robot_id(id))
    invalid(std::string(field) + " is not a valid robot id: \"" + printable(id) + "\"");
  return id;
}

void check_unique(const std::vector<std::string> &ids, const char *field) {
  std::vector<std::string> sorted(ids);
  std::sort(sorted.begin(), sorted.end());
  if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end())
    invalid(std::string(field) + " must not repeat a robot id");
}

std::vector<std::string> parse_ids(const json &array, const char *field, const ServiceOptions &options) {
  if (!array.is_array())
    invalid(std::string(field) + " must be an array");
  if (array.size() > options.max_ids)
    invalid(std::string(field) + " has more than " + std::to_string(options.max_ids) + " entries");
  std::vector<std::string> ids;
  for (const auto &item : array)
    ids.push_back(robot_id_of(item, field));
  check_unique(ids, field);
  return ids;
}

struct EngageRequest {
  bool all = false;
  std::vector<std::string> ids;
};

EngageRequest parse_engage(const json &body, const ServiceOptions &options) {
  check_fields(body, {"robot_ids", "all"});
  const auto ids = body.find("robot_ids");
  const auto all = body.find("all");
  const bool has_ids = ids != body.end();
  const bool has_all = all != body.end();
  if (has_ids == has_all)
    invalid("exactly one of robot_ids and all is required");
  EngageRequest request;
  if (has_all) {
    if (!all->is_boolean() || !all->get<bool>())
      invalid("all must be true");
    request.all = true;
  } else {
    request.ids = parse_ids(*ids, "robot_ids", options);
    if (request.ids.empty())
      invalid("robot_ids must not be empty");
  }
  return request;
}

std::vector<std::string> parse_state(const json &body, const ServiceOptions &options) {
  check_fields(body, {"robot_ids"});
  const auto ids = body.find("robot_ids");
  return ids == body.end() ? std::vector<std::string>() : parse_ids(*ids, "robot_ids", options);
}

struct ReleaseRequest {
  std::string expected_instance;
  std::vector<ReleaseChange> changes;
};

ReleaseRequest parse_release(const json &body, const ServiceOptions &options) {
  check_fields(body, {"expected_instance", "changes"});
  const auto instance = body.find("expected_instance");
  const auto changes = body.find("changes");
  if (instance == body.end() || changes == body.end())
    invalid("expected_instance and changes are required");
  ReleaseRequest request;
  if (!instance->is_string() || !valid_instance_id(instance->get<std::string>()))
    invalid("expected_instance must be 32 lowercase hex digits");
  request.expected_instance = instance->get<std::string>();
  if (!changes->is_array())
    invalid("changes must be an array");
  if (changes->empty())
    invalid("changes must not be empty");
  if (changes->size() > options.max_ids)
    invalid("changes has more than " + std::to_string(options.max_ids) + " entries");
  std::vector<std::string> ids;
  for (const auto &item : *changes) {
    if (!item.is_object())
      invalid("each change must be an object");
    check_fields(item, {"robot_id", "expected_revision"});
    const auto id = item.find("robot_id");
    const auto revision = item.find("expected_revision");
    if (id == item.end() || revision == item.end())
      invalid("each change needs robot_id and expected_revision");
    if (!revision->is_number_unsigned())
      invalid("expected_revision must be an unsigned integer");
    ReleaseChange change;
    change.robot_id = robot_id_of(*id, "robot_id");
    change.expected_revision = revision->get<std::uint64_t>();
    ids.push_back(change.robot_id);
    request.changes.push_back(std::move(change));
  }
  check_unique(ids, "changes");
  return request;
}

json milliseconds_since(const std::optional<TimeNs> &time, TimeNs start) {
  if (!time)
    return json(nullptr);
  if (*time <= start)
    return json(static_cast<std::int64_t>(0));
  const std::uint64_t elapsed = static_cast<std::uint64_t>(*time) - static_cast<std::uint64_t>(start);
  return json(static_cast<std::int64_t>(elapsed / 1000000));
}

json state_json(const RobotState &state, TimeNs start) {
  json j;
  j["robot_id"] = state.robot_id;
  j["held"] = state.held;
  j["revision"] = state.revision;
  j["stage"] = to_string(state.stage);
  j["gated_at_ms"] = milliseconds_since(state.gated_at, start);
  j["zero_written_at_ms"] = milliseconds_since(state.zero_written_at, start);
  j["stopped_at_ms"] = milliseconds_since(state.stopped_at, start);
  if (!state.fault.empty())
    j["fault"] = state.fault;
  else if (!state.write_error.empty())
    j["fault"] = "zero write failed: " + state.write_error;
  else
    j["fault"] = nullptr;
  return j;
}

json result_json(const RobotResult &result) {
  json j;
  j["robot_id"] = result.robot_id;
  j["outcome"] = to_string(result.outcome);
  if (result.state) {
    j["held"] = result.state->held;
    j["revision"] = result.state->revision;
    j["stage"] = to_string(result.state->stage);
  }
  return j;
}

ServiceReply ok(const json &body) { return {Status::ok, body.dump()}; }

ServiceReply error_reply(Status status, const std::string &message) {
  json j;
  j["code"] = to_string(status);
  j["message"] = message;
  return {status, j.dump()};
}

ServiceReply mutation_reply(const std::string &instance, const std::vector<RobotResult> &results) {
  json j;
  j["instance"] = instance;
  j["robots"] = json::array();
  for (const auto &result : results)
    j["robots"].push_back(result_json(result));
  return ok(j);
}

}  // namespace

const char *to_string(Status status) noexcept {
  switch (status) {
    case Status::ok: return "ok";
    case Status::invalid_argument: return "invalid_argument";
    case Status::not_found: return "not_found";
    case Status::internal: return "internal";
  }
  return "internal";
}

struct Service::Impl {
  // An engage whose robots have not all been zeroed by the tick yet.
  struct Waiter {
    std::vector<RobotResult> results;
    std::vector<std::string> ids;  // robots in results that exist
    Clock::time_point until;
    Callback callback;
  };
  // Shared with the domain listener, which may run after the service is gone.
  struct Shared {
    std::mutex mutex;
    std::condition_variable changed;
    bool wake = false;
    bool stop = false;
    std::list<Waiter> waiters;
  };

  Impl(Domain &domain_in, ServiceOptions options_in)
      : domain(domain_in), options(std::move(options_in)), shared(std::make_shared<Shared>()) {
    if (options.reply_wait.count() < 0 || options.reply_wait > std::chrono::hours(1) || options.max_ids == 0 ||
        options.max_body_bytes < 2)
      throw std::invalid_argument("invalid chassis hold service options");
    auto wake_target = shared;
    token = domain.add_listener([wake_target] {
      {
        std::lock_guard<std::mutex> lock(wake_target->mutex);
        wake_target->wake = true;
      }
      wake_target->changed.notify_one();
    });
    try {
      worker = std::thread([this] { run(); });
    } catch (...) {
      domain.remove_listener(token);
      throw;
    }
  }

  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(shared->mutex);
      shared->stop = true;
    }
    shared->changed.notify_all();
    worker.join();
    domain.remove_listener(token);
    std::list<Waiter> rest;
    {
      std::lock_guard<std::mutex> lock(shared->mutex);
      rest.swap(shared->waiters);
    }
    for (auto &waiter : rest)
      deliver(waiter.callback, finish_engage(waiter.results));
  }

  static void deliver(Callback &callback, ServiceReply reply) {
    try {
      callback(std::move(reply));
    } catch (...) {
      // A callback must not take the service thread down; its transport owns the failure.
    }
  }

  // True once every robot of the waiter is past the gated stage, released, or gone.
  bool zero_settled(const Waiter &waiter) const {
    const Snapshot snapshot = domain.state(waiter.ids);
    for (const auto &state : snapshot.robots)
      if (state.held && state.stage == Stage::gated)
        return false;
    return true;
  }

  // The reply reports the state at the time it is sent, with the outcome each robot got from the engage.
  ServiceReply finish_engage(std::vector<RobotResult> &results) const {
    std::vector<std::string> ids;
    for (const auto &result : results)
      if (result.state)
        ids.push_back(result.robot_id);
    if (!ids.empty()) {
      const Snapshot snapshot = domain.state(ids);
      for (auto &result : results) {
        if (!result.state)
          continue;
        for (const auto &state : snapshot.robots)
          if (state.robot_id == result.robot_id)
            result.state = state;
      }
    }
    return mutation_reply(domain.instance_id(), results);
  }

  // The service thread: completes waiters when the tick has written zero (the domain listener wakes it) or
  // their wait ends.
  void run() {
    std::unique_lock<std::mutex> lock(shared->mutex);
    while (!shared->stop) {
      shared->wake = false;
      std::vector<std::pair<Callback, ServiceReply>> done;
      auto next = Clock::time_point::max();
      const auto now = Clock::now();
      for (auto it = shared->waiters.begin(); it != shared->waiters.end();) {
        if (it->until <= now || zero_settled(*it)) {
          done.emplace_back(std::move(it->callback), finish_engage(it->results));
          it = shared->waiters.erase(it);
        } else {
          next = std::min(next, it->until);
          ++it;
        }
      }
      if (!done.empty()) {
        lock.unlock();
        for (auto &item : done)
          deliver(item.first, std::move(item.second));
        lock.lock();
        continue;
      }
      if (next == Clock::time_point::max())
        shared->changed.wait(lock, [this] { return shared->stop || shared->wake; });
      else
        shared->changed.wait_until(lock, next, [this] { return shared->stop || shared->wake; });
    }
  }

  void engage(const EngageRequest &request, Clock::time_point deadline, Callback &callback) {
    std::vector<RobotResult> results = request.all ? domain.engage_all() : domain.engage(request.ids);
    bool waiting = false;
    std::vector<std::string> ids;
    for (const auto &result : results) {
      if (!result.state)
        continue;
      ids.push_back(result.robot_id);
      waiting = waiting || (result.state->held && result.state->stage == Stage::gated);
    }
    const auto until = std::min(Clock::now() + options.reply_wait, deadline);
    if (waiting && until > Clock::now()) {
      std::unique_lock<std::mutex> lock(shared->mutex);
      if (shared->waiters.size() < max_pending_replies) {
        shared->waiters.push_back({std::move(results), std::move(ids), until, std::move(callback)});
        // The tick may have written zero between the engage and here: have the thread look at once.
        shared->wake = true;
        lock.unlock();
        shared->changed.notify_one();
        return;
      }
    }
    deliver(callback, finish_engage(results));
  }

  void dispatch(const std::string &method, const std::string &body, Clock::time_point deadline,
                Callback &callback) {
    try {
      if (method == "Describe") {
        const json request = parse_object(body, options);
        check_fields(request, {});
        const Description description = domain.describe();
        json j;
        j["instance"] = description.instance;
        j["robots"] = json::array();
        j["feedback"] = json::object();
        for (const auto &robot : description.robots) {
          j["robots"].push_back(robot.robot_id);
          j["feedback"][robot.robot_id] = robot.feedback;
        }
        j["capabilities"] = json::array();
        for (const char *capability : capabilities)
          j["capabilities"].push_back(capability);
        return deliver(callback, ok(j));
      }
      if (method == "State") {
        const std::vector<std::string> ids = parse_state(parse_object(body, options), options);
        if (ids.empty() && domain.describe().robots.size() > options.max_ids)
          invalid("the roster has more than " + std::to_string(options.max_ids) + " robots: list robot_ids");
        const Snapshot snapshot = domain.state(ids);
        if (!snapshot.unknown.empty()) {
          json j;
          j["code"] = "not_found";
          j["message"] = "unknown robot_ids";
          j["details"]["robot_ids"] = snapshot.unknown;
          return deliver(callback, ServiceReply{Status::not_found, j.dump()});
        }
        json j;
        j["instance"] = snapshot.instance;
        j["robots"] = json::array();
        for (const auto &state : snapshot.robots)
          j["robots"].push_back(state_json(state, snapshot.start));
        return deliver(callback, ok(j));
      }
      if (method == "Engage") {
        const EngageRequest request = parse_engage(parse_object(body, options), options);
        if (request.all && domain.describe().robots.size() > options.max_ids)
          invalid("the roster has more than " + std::to_string(options.max_ids) + " robots: list robot_ids");
        return engage(request, deadline, callback);
      }
      if (method == "Release") {
        const ReleaseRequest request = parse_release(parse_object(body, options), options);
        const std::vector<RobotResult> results = domain.release(request.expected_instance, request.changes);
        return deliver(callback, mutation_reply(domain.instance_id(), results));
      }
      return deliver(callback, error_reply(Status::not_found, "unknown method \"" + printable(method) + "\""));
    } catch (const RequestError &e) {
      deliver(callback, error_reply(e.status, e.message));
    } catch (const std::exception &) {
      deliver(callback, error_reply(Status::internal, "internal error"));
    }
  }

  Domain &domain;
  ServiceOptions options;
  std::shared_ptr<Shared> shared;
  std::uint64_t token = 0;
  std::thread worker;
};

Service::Service(Domain &domain, ServiceOptions options) : impl_(std::make_unique<Impl>(domain, std::move(options))) {}
Service::~Service() = default;

const std::string &Service::instance_id() const noexcept { return impl_->domain.instance_id(); }

void Service::call_async(const std::string &method, const std::string &body, Clock::time_point deadline,
                         Callback callback) {
  impl_->dispatch(method, body, deadline, callback);
}

ServiceReply Service::call(const std::string &method, const std::string &body, Clock::time_point deadline) {
  std::promise<ServiceReply> promise;
  std::future<ServiceReply> reply = promise.get_future();
  call_async(method, body, deadline, [&promise](ServiceReply result) { promise.set_value(std::move(result)); });
  return reply.get();
}

}  // namespace xgc2::chassis_hold
