#include <xgc2/chassis_hold/server.hpp>
#include <xgc2/chassis_hold/rpc_handler.hpp>
#include <atomic>
#include <condition_variable>
#include <future>
#include <mutex>
#include <thread>

namespace xgc2::chassis_hold {
std::vector<std::pair<std::string, std::string>> environment_snapshot(char *const *environment) {
  std::vector<std::pair<std::string, std::string>> snapshot;
  if (environment) for (auto entry = environment; *entry; ++entry) {
    const std::string value(*entry);
    if (value.rfind("XGC2_XRPC_", 0) != 0) continue;
    const auto equals = value.find('=');
    snapshot.emplace_back(value.substr(0, equals), equals == std::string::npos ? "" : value.substr(equals+1));
  }
  return snapshot;
}
namespace {
xrpc::RuntimePolicy resolve_policy(const ServerOptions &options) {
  xrpc::RuntimePolicyOptions policy;
  policy.environment = options.environment;
  policy.default_source = "chassis.hold.v1";
  policy.defaults = {{"HOST_MAX_CONNECTIONS", "32"}, {"HOST_MAX_IN_FLIGHT", "32"},
    {"MAX_HEADER_BYTES", "4096"}, {"MAX_REQUEST_BYTES", "4096"},
    {"MAX_RESPONSE_BYTES", "8192"}, {"CALL_TIMEOUT_MS", "250"},
    {"HEADER_TIMEOUT_MS", "250"}, {"IDLE_TIMEOUT_MS", "5000"}, {"SHUTDOWN_TIMEOUT_MS", "1000"}};
  policy.ceilings = {{"HOST_MAX_CONNECTIONS", 32}, {"HOST_MAX_IN_FLIGHT", 32},
    {"MAX_HEADER_BYTES", 4096}, {"MAX_REQUEST_BYTES", 4096},
    {"MAX_RESPONSE_BYTES", 8192}, {"CALL_TIMEOUT_MS", 250}};
  return xrpc::resolve_runtime_policy(policy);
}
RpcOptions rpc_options(const ServerOptions &options) {
  RpcOptions rpc;
  static_cast<Endpoint &>(rpc) = options;
  if (rpc.instance_id.empty()) rpc.instance_id = xrpc::new_instance_id();
  rpc.host_policy = std::make_shared<xrpc::RuntimePolicy>(resolve_policy(options));
  return rpc;
}
} // namespace
struct HoldServer::Impl {
  const RpcOptions options;
  HoldRpcHandler handler;
  std::atomic<bool> stopping{false}, running{false};
  std::atomic<xrpc::HttpServer *> host{nullptr};
  std::mutex shutdown_mutex;
  std::condition_variable shutdown_changed;
  std::thread io;
  explicit Impl(ServerOptions input) : options(rpc_options(input)), handler(options) {
    std::promise<void> ready;
    auto started = ready.get_future();
    io = std::thread([this, ready = std::move(ready)]() mutable {
      bool announced = false;
      std::unique_ptr<xrpc::HttpServer> server;
      try {
        xrpc::UnixOptions endpoint;
        endpoint.path = options.socket_path;
        xrpc::HttpIdentity identity;
        identity.instance_id = options.instance_id;
        identity.discovery_targets = {"/v1/chassis/descriptor"};
        server = std::make_unique<xrpc::HttpServer>(endpoint,
          [this](xrpc::HttpRequest request, xrpc::HttpReply reply) { handler.handle(std::move(request), std::move(reply)); },
          xrpc::http_limits(*options.host_policy), std::move(identity));
        server->set_wakeup_handler([this] { handler.finish(); });
        host.store(server.get(), std::memory_order_release);
        running.store(true, std::memory_order_release);
        ready.set_value();
        announced = true;
        server->run();
      } catch (...) {
        if (!announced) ready.set_exception(std::current_exception());
        // On unexpected IO failure keep native work/replies alive until the
        // process owner observes unhealthy(), stops native work and destroys us.
      }
      running.store(false, std::memory_order_release);
      // Keep the host alive after an IO failure, until the process owner has
      // observed unhealthy(), stopped native work and begun destruction.
      // Native completion may still wake it; no host pointer is reclaimed early.
      std::unique_lock<std::mutex> lock(shutdown_mutex);
      if (announced) shutdown_changed.wait(lock, [this] { return stopping.load(std::memory_order_acquire); });
      host.store(nullptr, std::memory_order_release);
      handler.shutdown();
      server.reset(); // Host destruction stays on its IO owner.
    });
    try { started.get(); } catch (...) { io.join(); throw; }
  }
  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(shutdown_mutex);
      stopping.store(true, std::memory_order_release);
      if (auto *server = host.load(std::memory_order_acquire)) server->request_stop();
    }
    shutdown_changed.notify_one();
    if (io.joinable()) io.join();
  }
  std::size_t control_tick(HoldState::ZeroFn zero, void *context, std::size_t budget) noexcept {
    const auto processed = handler.control_tick(zero, context, budget);
    if (processed) if (auto *server = host.load(std::memory_order_acquire)) server->wake();
    return processed;
  }
};
HoldServer::HoldServer(ServerOptions options) : impl_(std::make_unique<Impl>(std::move(options))) {}
HoldServer::~HoldServer() = default;
std::size_t HoldServer::control_tick(HoldState::ZeroFn zero, void *context, std::size_t budget) noexcept { return impl_->control_tick(zero, context, budget); }
bool HoldServer::held(std::size_t robot) const noexcept { return impl_->handler.held(robot); }
bool HoldServer::healthy() const noexcept { return impl_->running.load(std::memory_order_acquire) && impl_->handler.healthy(); }
std::size_t HoldServer::pending() const noexcept { return impl_->handler.pending(); }
const std::string &HoldServer::instance_id() const noexcept { return impl_->handler.instance_id(); }
} // namespace xgc2::chassis_hold
