#include <xgc2/chassis_hold/rpc_handler.hpp>
#include <xgc2/xrpc/json_http.hpp>
#include <algorithm>
#include <atomic>
#include <stdexcept>

namespace xgc2::chassis_hold {
struct HoldRpcHandler::Impl {
  explicit Impl(RpcOptions options)
      : options_(std::move(options)), state_(options_.robot_ids.size()), policy_(options_.host_policy),
        json_http_([this](xrpc::JsonHttpRequest request, xrpc::JsonHttpReply reply) {
          handle_json(std::move(request), std::move(reply));
        }, {4096, 8}, policy_ ? xrpc::http_limits(*policy_).response_bytes : 8192) {
    if (!policy_) throw std::invalid_argument("hold handler requires host policy");
    validate_options();
  }
  void shutdown() noexcept {
    stopping_.store(true, std::memory_order_release);
    for (auto &slot : slots_) slot.reply = {};
  }
  // Native thread only. The zero sink applies to its sole command output and
  // reports success only after the native write/command-state update returns.
  // No JSON, socket access or waiting. Cancellation is one atomic SDK query;
  // the published reply is immutable until this owner reports Phase::Done.
  std::size_t control_tick(HoldState::ZeroFn zero, void *context,
                           std::size_t budget = max_robots) noexcept {
    std::size_t processed = 0;
    auto tail = tail_.load(std::memory_order_relaxed);
    while (processed < budget && tail != head_.load(std::memory_order_acquire)) {
      auto &slot = slots_[queue_[tail % queue_capacity]];
      slot.phase.store(Phase::Executing, std::memory_order_release);
      if (slot.reply.cancelled())
        slot.result = state_.snapshot(Outcome::Cancelled);
      else if (xrpc::Clock::now() >= slot.deadline)
        slot.result = state_.snapshot(Outcome::Expired);
      else
        slot.result = state_.apply(slot.command, zero, context);
      slot.phase.store(Phase::Done, std::memory_order_release);
      ++tail;
      tail_.store(tail, std::memory_order_release);
      ++processed;
    }
    return processed;
  }
  bool held(std::size_t robot = 0) const noexcept { return state_.held(robot); }
  bool healthy() const noexcept { return state_.healthy(); }
  std::size_t pending() const noexcept {
    return pending_.load(std::memory_order_acquire);
  }
  const std::string &instance_id() const noexcept { return options_.instance_id; }

private:
  enum class Phase { Free, Queued, Executing, Done };
  struct Slot {
    std::atomic<Phase> phase{Phase::Free};
    Command command;
    xrpc::Clock::time_point deadline;
    Result result;
    xrpc::JsonHttpReply reply; // IO assigns/completes/releases; native reads cancellation.
  };
  static bool valid_id(const std::string &value) {
    if (value.empty() || value.size() > 128)
      return false;
    for (const unsigned char c : value)
      if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.'))
        return false;
    return true;
  }
  void validate_options() {
    if (options_.socket_path.empty() || options_.socket_path.front() != '/' ||
        options_.instance_id.empty() || !valid_id(options_.target_id))
      throw std::invalid_argument("explicit chassis endpoint/instance/target required");
    for (std::size_t i = 0; i < options_.robot_ids.size(); ++i) {
      if (!valid_id(options_.robot_ids[i]))
        throw std::invalid_argument("invalid chassis robot id");
      for (std::size_t j = 0; j < i; ++j)
        if (options_.robot_ids[i] == options_.robot_ids[j])
          throw std::invalid_argument("duplicate chassis robot id");
    }
  }
  xrpc::Json effective_policy() const {
    xrpc::Json value;
    value["revision"] = std::uint64_t(policy_->revision());
    value["fields"] = xrpc::Json::array();
    for (const auto &field : policy_->fields()) {
      xrpc::Json record;
      record["name"] = std::string(field.name);
      record["source"] = std::string(field.source);
      record["source_detail"] = field.source_detail;
      record["dynamic"] = false; // This handler exposes no live policy mutation.
      if (const auto *integer = std::get_if<std::int64_t>(&field.value))
        record["value"] = std::int64_t(*integer);
      else record["value"] = std::get<std::string>(field.value);
      if (field.ceiling) record["ceiling"] = std::int64_t(*field.ceiling);
      value["fields"].push_back(std::move(record));
    }
    return value;
  }
  xrpc::Json descriptor() const {
    xrpc::Json value;
    value["target_id"] = options_.target_id;
    value["service"] = "chassis.hold";
    value["api_version"] = "1";
    value["instance_id"] = options_.instance_id;
    value["profile"] = "http.v1";
    value["endpoint"]["kind"] = "unix";
    value["endpoint"]["address"] = options_.socket_path;
    value["max_batch"] = static_cast<unsigned>(max_robots);
    value["queue_capacity"] = static_cast<unsigned>(queue_capacity);
    value["completion"] = "command-gated";
    value["physical_stop_supported"] = false;
    value["robots"] = xrpc::Json::array();
    for (const auto &id : options_.robot_ids)
      value["robots"].push_back(id);
    return value;
  }
  bool parse(const xrpc::Json &value, Command &command) const {
    if (!value.is_object() || value.size() != 2 || !value.contains("expected_revision") ||
        !value.contains("changes") || !value["expected_revision"].is_number_unsigned() ||
        !value["changes"].is_array()) return false;
    const auto &changes = value["changes"];
    if (changes.empty() || changes.size() > max_robots)
      return false;
    command.mutate = true;
    command.expected_revision = value["expected_revision"].get<std::uint64_t>();
    command.count = static_cast<std::uint8_t>(changes.size());
    std::array<bool, max_robots> seen{};
    for (std::size_t n = 0; n < changes.size(); ++n) {
      const auto &change = changes[n];
      if (!change.is_object() || change.size() != 2 || !change.contains("robot_id") || !change.contains("held") ||
          !change["robot_id"].is_string() || !change["held"].is_boolean())
        return false;
      const auto &id = change["robot_id"].get_ref<const std::string&>();
      std::size_t index = 0;
      while (index < options_.robot_ids.size() && options_.robot_ids[index] != id)
        ++index;
      if (index == options_.robot_ids.size() || seen[index])
        return false;
      seen[index] = true;
      command.changes[n] = {static_cast<std::uint8_t>(index), change["held"].get<bool>()};
    }
    return true;
  }
public:
  void handle(xrpc::HttpRequest request, xrpc::HttpReply reply) {
    json_http_(std::move(request), std::move(reply));
  }
  // Host IO owner only; its existing HttpServer performs
  // instance fencing, admission and native request deadlines before dispatch.
  void handle_json(xrpc::JsonHttpRequest request, xrpc::JsonHttpReply reply) {
    try {
      if (stopping_.load(std::memory_order_acquire)) {
        reply.error(503, "unavailable", "chassis hold stopping");
        return;
      }
      if (request.target == "/v1/chassis/descriptor" && request.method == "GET" &&
          !request.body) {
        reply.complete(descriptor());
        return;
      }
      if (request.target == "/v1/chassis/runtime-policy" && request.method == "GET" &&
          !request.body) {
        reply.complete(effective_policy());
        return;
      }
      if (request.target != "/v1/chassis/hold") {
        reply.error(404, "not_found", "unknown chassis method");
        return;
      }
      Command command;
      if (request.method == "POST") {
        if (!request.body || !parse(*request.body, command)) {
          reply.error(400, "invalid_argument", "invalid hold batch");
          return;
        }
      } else if (request.method != "GET" || request.body) {
        reply.error(400, "invalid_argument", "GET or POST required");
        return;
      }
      const auto head = head_.load(std::memory_order_relaxed);
      const auto tail = tail_.load(std::memory_order_acquire);
      std::size_t index = 0;
      while (index < queue_capacity &&
             slots_[index].phase.load(std::memory_order_acquire) != Phase::Free)
        ++index;
      if (index == queue_capacity || head - tail >= queue_capacity) {
        reply.error(429, "resource_exhausted", "hold queue full");
        return;
      }
      auto &slot = slots_[index];
      slot.command = command;
      slot.deadline = std::min(request.deadline, xrpc::Clock::now()+std::chrono::milliseconds(250));
      slot.reply = std::move(reply);
      slot.phase.store(Phase::Queued, std::memory_order_release);
      queue_[head % queue_capacity] = static_cast<std::uint8_t>(index);
      pending_.fetch_add(1, std::memory_order_release);
      head_.store(head + 1, std::memory_order_release);
    } catch (const std::bad_alloc &) {
      reply.error(503, "resource_exhausted", "chassis request allocation failed");
    } catch (const xrpc::Json::exception &) {
      reply.error(400, "invalid_argument", "invalid chassis request");
    }
  }
  void finish() {
    for (auto &slot : slots_) {
      const auto phase = slot.phase.load(std::memory_order_acquire);
      if (phase != Phase::Done)
        continue;
      const auto &result = slot.result;
      int status = 200;
      const char *stage = "observed", *code = "";
      switch (result.outcome) {
      case Outcome::Applied: stage = "applied"; break;
      case Outcome::Conflict: stage = "rejected"; status = 409; code = "conflict"; break;
      case Outcome::Invalid: stage = "rejected"; status = 400; code = "invalid_argument"; break;
      case Outcome::Expired: stage = "not-applied"; status = 504; code = "deadline_exceeded"; break;
      case Outcome::Cancelled: stage = "not-applied"; status = 499; code = "cancelled"; break;
      case Outcome::Failed: stage = "failed"; status = 503; code = "native_write_failed"; break;
      case Outcome::Snapshot: break;
      }
      xrpc::Json body;
      body["stage"] = stage;
      body["revision"] = std::uint64_t(result.revision);
      body["completion"] = "command-gated";
      body["physical_stop_confirmed"] = false;
      if (*code)
        body["error"]["code"] = code;
      body["robots"] = xrpc::Json::array();
      for (std::size_t i = 0; i < options_.robot_ids.size(); ++i) {
        xrpc::Json robot;
        robot["robot_id"] = options_.robot_ids[i];
        robot["held"] = result.held[i];
        body["robots"].push_back(std::move(robot));
      }
      body["changes"] = xrpc::Json::array();
      for (std::size_t i = 0; i < slot.command.count; ++i) {
        const auto change = slot.command.changes[i];
        xrpc::Json record;
        record["robot_id"] = options_.robot_ids[change.robot];
        record["requested_held"] = change.held;
        record["zero_applied"] = result.zero_applied[change.robot];
        body["changes"].push_back(std::move(record));
      }
      slot.reply.complete(body, status);
      slot.reply = {};
      slot.phase.store(Phase::Free, std::memory_order_release);
      pending_.fetch_sub(1, std::memory_order_release);
    }
  }
private:
  RpcOptions options_;
  HoldState state_;
  const std::shared_ptr<const xrpc::RuntimePolicy> policy_;
  xrpc::JsonHttpHandler json_http_;
  std::array<Slot, queue_capacity> slots_{};
  std::array<std::uint8_t, queue_capacity> queue_{};
  std::atomic<std::uint64_t> head_{0}, tail_{0};
  std::atomic<std::size_t> pending_{0};
  std::atomic<bool> stopping_{false};
};

HoldRpcHandler::HoldRpcHandler(RpcOptions options) : impl_(std::make_unique<Impl>(std::move(options))) {}
HoldRpcHandler::~HoldRpcHandler() = default;
void HoldRpcHandler::handle(xrpc::HttpRequest request, xrpc::HttpReply reply) { impl_->handle(std::move(request), std::move(reply)); }
void HoldRpcHandler::finish() { impl_->finish(); }
void HoldRpcHandler::shutdown() noexcept { impl_->shutdown(); }
std::size_t HoldRpcHandler::control_tick(HoldState::ZeroFn zero, void *context, std::size_t budget) noexcept { return impl_->control_tick(zero, context, budget); }
bool HoldRpcHandler::held(std::size_t robot) const noexcept { return impl_->held(robot); }
bool HoldRpcHandler::healthy() const noexcept { return impl_->healthy(); }
std::size_t HoldRpcHandler::pending() const noexcept { return impl_->pending(); }
const std::string &HoldRpcHandler::instance_id() const noexcept { return impl_->instance_id(); }
} // namespace xgc2::chassis_hold
