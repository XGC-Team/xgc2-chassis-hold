#include <xgc2/chassis_hold/server.hpp>
#include <xgc2/chassis_hold/rpc_handler.hpp>
#include <json/json.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <iostream>
#include <new>
#include <stdexcept>
#include <thread>

using namespace xgc2;
using namespace std::chrono;

thread_local bool reject_native_allocations = false;
[[gnu::noinline]] void *operator new(std::size_t size) {
  if (reject_native_allocations) throw std::bad_alloc();
  if (auto *pointer = std::malloc(size ? size : 1)) return pointer;
  throw std::bad_alloc();
}
[[gnu::noinline]] void *operator new(std::size_t size, const std::nothrow_t &) noexcept {
  try { return ::operator new(size); } catch (...) { return nullptr; }
}
[[gnu::noinline]] void operator delete(void *pointer, const std::nothrow_t &) noexcept { std::free(pointer); }
[[gnu::noinline]] void operator delete(void *pointer) noexcept { std::free(pointer); }
[[gnu::noinline]] void operator delete(void *pointer, std::size_t) noexcept { std::free(pointer); }

namespace {
void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
template<class Predicate> void await(Predicate condition) {
  const auto until = xrpc::Clock::now() + seconds(2);
  while (!condition() && xrpc::Clock::now() < until)
    std::this_thread::sleep_for(milliseconds(1));
  require(condition(), "fixture condition timed out");
}
struct Directory {
  std::string path;
  Directory() {
    char pattern[] = "/tmp/chassis-hold-native-XXXXXX";
    const auto *created = ::mkdtemp(pattern);
    require(created != nullptr, "private fixture directory failed");
    path = created;
  }
  ~Directory() { std::filesystem::remove_all(path); }
};
struct Sink {
  std::thread::id owner = std::this_thread::get_id();
  chassis_hold::HoldState *state = nullptr;
  int writes = 0;
  int fail_robot = -1;
  bool wrong_thread = false, unmasked = false;
  static bool zero(void *context, std::size_t index) noexcept {
    auto &self = *static_cast<Sink *>(context);
    self.wrong_thread |= std::this_thread::get_id() != self.owner;
    self.unmasked |= self.state && (!self.state->held(0) || !self.state->held(1));
    ++self.writes;
    return static_cast<int>(index) != self.fail_robot;
  }
};
Json::Value json(const xrpc::HttpResponse &response) {
  Json::CharReaderBuilder reader;
  Json::Value value;
  std::string errors;
  const std::unique_ptr<Json::CharReader> parser(reader.newCharReader());
  require(parser->parse(response.body.data(), response.body.data()+response.body.size(),
                        &value, &errors), "invalid response JSON");
  return value;
}
xrpc::HttpRequest request(const char *method, const char *target, std::string body = {}) {
  xrpc::HttpRequest value;
  value.method = method;
  value.target = target;
  value.body = std::move(body);
  if (!value.body.empty()) value.headers.emplace_back("Content-Type", "application/json");
  return value;
}
std::string mutation(std::uint64_t revision, bool held) {
  return "{\"expected_revision\":" + std::to_string(revision) +
    ",\"changes\":[{\"robot_id\":\"scout\",\"held\":" + (held ? "true" : "false") + "]}";
}
// Keep mutation JSON explicit so the fixture doesn't use the provider parser
// or serializer to produce valid requests for itself.
std::string batch(std::uint64_t revision, bool held) {
  return "{\"expected_revision\":" + std::to_string(revision) +
    ",\"changes\":[{\"robot_id\":\"scout\",\"held\":" + (held ? "true" : "false") + "}]}";
}
void state_contract() {
  chassis_hold::HoldState state(2);
  Sink sink;
  sink.state = &state;
  chassis_hold::Command command;
  command.mutate = true;
  command.count = 2;
  command.changes[0] = {0, true};
  command.changes[1] = {1, true};
  auto result = state.apply(command, Sink::zero, &sink);
  require(result.outcome == chassis_hold::Outcome::Applied && result.revision == 1,
          "native batch did not apply");
  require(sink.writes == 2 && !sink.unmasked && !sink.wrong_thread,
          "batch wasn't fully masked before native writes");
  command.expected_revision = 0;
  require(state.apply(command, Sink::zero, &sink).outcome == chassis_hold::Outcome::Conflict &&
          sink.writes == 2, "stale CAS reached native output");
  command.expected_revision = 1;
  command.changes[0].held = command.changes[1].held = false;
  sink.fail_robot = 1;
  result = state.apply(command, Sink::zero, &sink);
  require(result.outcome == chassis_hold::Outcome::Failed && result.revision == 2 &&
          result.zero_applied[0] && !result.zero_applied[1] && state.held(0) && state.held(1),
          "partial native failure opened a gate or claimed applied");
  command.expected_revision = 2;
  sink.fail_robot = -1;
  result = state.apply(command, Sink::zero, &sink);
  require(result.outcome == chassis_hold::Outcome::Applied && result.revision == 3 &&
          !state.held(0) && !state.held(1), "release didn't zero before opening");
}
void runtime_failure_contract() {
  chassis_hold::HoldState state(2);
  Sink sink;
  chassis_hold::Command command;
  command.mutate = true;
  command.count = 1;
  command.changes[0] = {0, false};
  reject_native_allocations = true;
  const auto result = state.apply(command, Sink::zero, &sink);
  reject_native_allocations = false;
  require(result.outcome == chassis_hold::Outcome::Failed && !state.healthy() &&
          state.held(0) && state.held(1), "runtime allocation failure opened a gate or claimed applied");
}
void transport_contract() {
  Directory directory;
  chassis_hold::ServerOptions options;
  options.socket_path = directory.path + "/hold.sock";
  options.target_id = "fixture";
  options.instance_id = xrpc::new_instance_id();
  options.robot_ids = {"scout"};
  options.environment = {{"XGC2_XRPC_IDLE_TIMEOUT_MS", "1000"}};
  for (const auto &environment : std::vector<std::pair<std::string,std::string>>{
      {"XGC2_XRPC_HOST_MAX_IN_FLIGHT", "33"}, {"XGC2_XRPC_UNKNOWN", "1"},
      {"XGC2_XRPC_LOG_LEVEL", "debug"}}) {
    auto invalid = options;
    invalid.environment = {environment};
    bool rejected = false;
    try { chassis_hold::HoldServer forbidden(invalid); }
    catch (const xrpc::RuntimePolicyError &) { rejected = true; }
    require(rejected && !std::filesystem::exists(options.socket_path),
            "unsupported/unknown/over-ceiling policy opened an endpoint");
  }
  chassis_hold::HoldServer provider(options);
  xrpc::HttpClient discovery(options.socket_path);
  auto description = json(discovery.call(request("GET", "/v1/chassis/descriptor"),
                                         xrpc::Clock::now()+seconds(1)));
  require(description["instance_id"].asString() == options.instance_id &&
          !description["physical_stop_supported"].asBool(), "wrong ServiceRef/completion capability");
  xrpc::HttpClient client(options.socket_path, {}, options.instance_id);
  auto policy = json(client.call(request("GET", "/v1/chassis/runtime-policy"),
                                  xrpc::Clock::now()+seconds(1)));
  bool policy_applied = false;
  for (const auto &field : policy["fields"])
    if (field["name"] == "IDLE_TIMEOUT_MS")
      policy_applied = field["value"].asInt() == 1000 && field["source"] == "environment";
  require(policy_applied, "effective startup policy omitted environment source/value");
  bool conflict_owner = false;
  try { chassis_hold::HoldServer duplicate(options); }
  catch (const std::exception &) { conflict_owner = true; }
  require(conflict_owner && std::filesystem::exists(options.socket_path),
          "duplicate host took or removed a live endpoint");
  Sink sink;
  auto call = [&](const char *method, std::string body = {}, milliseconds budget = milliseconds(200)) {
    return std::async(std::launch::async, [&, method, body=std::move(body), budget] {
      return client.call(request(method, "/v1/chassis/hold", body), xrpc::Clock::now()+budget);
    });
  };
  auto first = call("POST", batch(0, true));
  await([&] { return provider.pending() == 1; });
  require(sink.writes == 0, "RPC thread wrote native output");
  provider.control_tick(Sink::zero, &sink);
  reject_native_allocations = true;
  require(provider.held() && provider.control_tick(Sink::zero, &sink) == 0, "idle gate allocated or processed work");
  reject_native_allocations = false;
  auto receipt = json(first.get());
  require(receipt["stage"] == "applied" && receipt["revision"].asUInt64() == 1 &&
          receipt["changes"][0]["zero_applied"].asBool() && provider.held() &&
          sink.writes == 1 && !sink.wrong_thread && !receipt["physical_stop_confirmed"].asBool(),
          "receipt preceded the real native write");

  for (const auto &body : std::vector<std::string>{
      "{}", mutation(1, false),
      "{\"expected_revision\":1,\"expected_revision\":1,\"changes\":[]}",
      "{\"expected_revision\":1.0,\"changes\":[{\"robot_id\":\"scout\",\"held\":false}]}",
      "{\"expected_revision\":1,\"changes\":[{\"robot_id\":\"scout\",\"held\":\"false\"}]}",
      "{\"expected_revision\":1,\"changes\":[{\"robot_id\":\"unknown\",\"held\":false}]}",
      "{\"expected_revision\":1,\"changes\":[{\"robot_id\":\"\",\"held\":false}]}",
      "{\"expected_revision\":1,\"changes\":[{\"robot_id\":\"scout\",\"held\":false},{\"robot_id\":\"scout\",\"held\":true}]}",
      batch(1,false) + "garbage", batch(1,false) + " /* comment */",
      "{\"expected_revision\":1,\"expected_revision\":2,\"changes\":[{\"robot_id\":\"scout\",\"held\":false}]}"}) {
    require(client.call(request("POST", "/v1/chassis/hold", body),
                        xrpc::Clock::now()+seconds(1)).status == 400,
            "malformed/ambiguous domain payload accepted");
  }
  require(sink.writes == 1 && provider.pending() == 0 && provider.held(),
          "invalid payload changed native state");
  xrpc::HttpClient stale_instance(options.socket_path, {}, "old-instance");
  bool fenced = false;
  try {
    fenced = stale_instance.call(request("POST", "/v1/chassis/hold", batch(1,false)),
                                  xrpc::Clock::now()+seconds(1)).status == 409;
  } catch (const xrpc::HttpCallError &) {
    fenced = true; // The client additionally rejects the response's new identity.
  }
  require(fenced && provider.pending() == 0 && sink.writes == 1,
          "stale process instance reached native admission");
  auto conflict = call("POST", batch(0, false));
  await([&] { return provider.pending() == 1; });
  provider.control_tick(Sink::zero, &sink);
  require(conflict.get().status == 409 && sink.writes == 1 && provider.held(),
          "stale revision reached native output");

  auto expired = call("POST", batch(1, false), milliseconds(30));
  await([&] { return provider.pending() == 1; });
  std::this_thread::sleep_for(milliseconds(60));
  provider.control_tick(Sink::zero, &sink);
  try { require(expired.get().status == 504, "deadline yielded a success receipt"); }
  catch (const xrpc::HttpCallError &error) {
    require(error.delivery == xrpc::Delivery::OutcomeUnknown, "wrong expired-call delivery state");
  }
  await([&] { return provider.pending() == 0; });
  require(sink.writes == 1 && provider.held(), "expired queue entry released chassis");

  auto failed = call("POST", batch(1, false));
  await([&] { return provider.pending() == 1; });
  sink.fail_robot = 0;
  provider.control_tick(Sink::zero, &sink);
  auto failure = failed.get();
  receipt = json(failure);
  require(failure.status == 503 && receipt["stage"] == "failed" &&
          receipt["revision"].asUInt64() == 2 && provider.held(), "native write failure claimed applied");
  sink.fail_robot = -1;
  auto released = call("POST", batch(2, false));
  await([&] { return provider.pending() == 1; });
  provider.control_tick(Sink::zero, &sink);
  require(released.get().status == 200 && !provider.held() && sink.writes == 3,
          "release failed or skipped native zero");

  auto observation = call("GET");
  await([&] { return provider.pending() == 1; });
  provider.control_tick(Sink::zero, &sink);
  receipt = json(observation.get());
  require(receipt["stage"] == "observed" && receipt["revision"].asUInt64() == 3 &&
          !receipt["robots"][0]["held"].asBool() && sink.writes == 3,
          "GET didn't observe the native revision");
  await([&] { return provider.pending() == 0; });

  client.close();
  discovery.close();
  stale_instance.close();
  std::this_thread::sleep_for(milliseconds(15));
  std::vector<std::unique_ptr<xrpc::HttpClient>> peers;
  std::vector<std::future<xrpc::HttpResponse>> queued;
  for (std::size_t i = 0; i < chassis_hold::queue_capacity; ++i) {
    peers.push_back(std::make_unique<xrpc::HttpClient>(options.socket_path,
                                                    xrpc::HttpLimits{}, options.instance_id));
    auto *peer = peers.back().get();
    queued.push_back(std::async(std::launch::async, [peer] {
      return peer->call(request("POST", "/v1/chassis/hold", batch(3, true)),
                        xrpc::Clock::now()+milliseconds(240));
    }));
  }
  await([&] { return provider.pending() == chassis_hold::queue_capacity; });
  xrpc::HttpClient overflow(options.socket_path, {}, options.instance_id);
  bool overloaded = false;
  try {
    overloaded = overflow.call(request("POST", "/v1/chassis/hold", batch(3, false)),
                                xrpc::Clock::now()+milliseconds(40)).status == 429;
  } catch (const xrpc::HttpCallError &) { overloaded = true; }
  require(overloaded && provider.pending() == chassis_hold::queue_capacity && sink.writes == 3,
          "overload exceeded retained slot bound or reached native output");
  provider.control_tick(Sink::zero, &sink);
  provider.control_tick(Sink::zero, &sink);
  int applied = 0;
  for (auto &queued_call : queued) {
    try {
      const auto response = queued_call.get();
      applied += response.status == 200;
      require(response.status == 200 || response.status == 409 || response.status == 504,
              "unexpected queued mutation result");
    } catch (const xrpc::HttpCallError &) { /* Explicit unknown delivery; no replay. */ }
  }
  await([&] { return provider.pending() == 0; });
  require(applied == 1 && sink.writes == 4 && provider.held(),
          "same-revision queued batch wrote twice or replayed under overload");

  peers.clear();
  overflow.close();
  std::this_thread::sleep_for(milliseconds(15));
  std::stop_source cancellation;
  xrpc::HttpClient cancelled_client(options.socket_path, {}, options.instance_id);
  auto cancelled = std::async(std::launch::async, [&] {
    return cancelled_client.call(request("POST", "/v1/chassis/hold", batch(4,false)),
      xrpc::Clock::now()+milliseconds(200), cancellation.get_token());
  });
  await([&] { return provider.pending() == 1; });
  cancellation.request_stop();
  bool was_cancelled = false;
  try { (void)cancelled.get(); }
  catch (const xrpc::HttpCallError &error) { was_cancelled = error.code == "cancelled"; }
  require(was_cancelled && provider.pending() == 1,
          "cancelled caller freed native work before its termination");
  std::this_thread::sleep_for(milliseconds(15));
  provider.control_tick(Sink::zero, &sink);
  await([&] { return provider.pending() == 0; });
  require(sink.writes == 4 && provider.held(), "cancelled queued mutation released chassis");
}

void shutdown_contract() {
  Directory directory;
  chassis_hold::ServerOptions options;
  options.socket_path = directory.path + "/shutdown.sock";
  options.target_id = "fixture";
  options.instance_id = xrpc::new_instance_id();
  options.robot_ids = {"scout"};
  auto provider = std::make_unique<chassis_hold::HoldServer>(options);
  xrpc::HttpClient client(options.socket_path, {}, options.instance_id);
  auto queued = std::async(std::launch::async, [&] {
    return client.call(request("POST", "/v1/chassis/hold", batch(0, true)),
                        xrpc::Clock::now()+milliseconds(200));
  });
  await([&] { return provider->pending() == 1; });
  // The native owner is quiescent; destruction aborts queued work and joins
  // its transport owner before the old endpoint can be reused.
  provider.reset();
  bool unknown = false;
  try { (void)queued.get(); }
  catch (const xrpc::HttpCallError &error) {
    unknown = error.delivery == xrpc::Delivery::OutcomeUnknown;
  }
  require(unknown && !std::filesystem::exists(options.socket_path),
          "shutdown claimed applied or retained a quiescent endpoint");
  client.close();
  options.instance_id = xrpc::new_instance_id();
  provider = std::make_unique<chassis_hold::HoldServer>(options);

  struct ExecutingSink {
    std::atomic<bool> entered{false}, release{false};
    std::atomic<int> writes{0};
    static bool zero(void *context, std::size_t) noexcept {
      auto &self = *static_cast<ExecutingSink *>(context);
      self.entered.store(true, std::memory_order_release);
      // Fixture only: hold a native write open to test caller departure.
      while (!self.release.load(std::memory_order_acquire)) std::this_thread::yield();
      self.writes.fetch_add(1, std::memory_order_release);
      return true;
    }
  } sink;
  std::stop_source stop;
  xrpc::HttpClient executing(options.socket_path, {}, options.instance_id);
  auto call = std::async(std::launch::async, [&] {
    return executing.call(request("POST", "/v1/chassis/hold", batch(0, true)),
        xrpc::Clock::now()+milliseconds(200), stop.get_token());
  });
  await([&] { return provider->pending() == 1; });
  std::thread native([&] { provider->control_tick(ExecutingSink::zero, &sink); });
  try {
    await([&] { return sink.entered.load(std::memory_order_acquire); });
    stop.request_stop();
    unknown = false;
    try { (void)call.get(); }
    catch (const xrpc::HttpCallError &error) {
      unknown = error.delivery == xrpc::Delivery::OutcomeUnknown;
    }
    bool replacement_rejected = false;
    try { chassis_hold::HoldServer replacement(options); }
    catch (const std::exception &) { replacement_rejected = true; }
    require(unknown && replacement_rejected && provider->pending() == 1 &&
            sink.writes.load(std::memory_order_acquire) == 0,
            "caller cancellation freed executing work or replaced its source owner");
  } catch (...) {
    sink.release.store(true, std::memory_order_release);
    native.join();
    throw;
  }
  sink.release.store(true, std::memory_order_release);
  native.join();
  await([&] { return provider->pending() == 0; });
  require(sink.writes.load(std::memory_order_acquire) == 1 && provider->held(),
          "executing cancellation interrupted or replayed the native zero");
  provider.reset();
  require(!std::filesystem::exists(options.socket_path), "executing work leaked its owner lease");
}

void shared_host_contract() {
  Directory directory;
  chassis_hold::RpcOptions options;
  options.socket_path = directory.path + "/world.sock";
  options.target_id = "world";
  options.instance_id = xrpc::new_instance_id();
  options.robot_ids = {"scout"};
  options.host_policy = std::make_shared<xrpc::RuntimePolicy>(
      xrpc::resolve_runtime_policy(xrpc::RuntimePolicyOptions{}));
  std::promise<void> ready;
  auto started = ready.get_future();
  std::atomic<bool> stopping{false};
  chassis_hold::HoldRpcHandler *native = nullptr;
  std::thread framework([&] {
    bool announced = false;
    try {
      auto routes = std::make_unique<chassis_hold::HoldRpcHandler>(options);
      require(!std::filesystem::exists(options.socket_path), "embedded provider opened a second listener");
      xrpc::UnixOptions endpoint;
      endpoint.path = options.socket_path;
      xrpc::HttpIdentity identity;
      identity.instance_id = options.instance_id;
      identity.discovery_targets = {"/v1/chassis/descriptor"};
      xrpc::HttpServer world(endpoint, [&](xrpc::HttpRequest call, xrpc::HttpReply reply) {
        if (call.target == "/v1/world/ping") {
          xrpc::HttpResponse response;
          response.body = "world";
          reply.complete(std::move(response));
        } else routes->handle(std::move(call), std::move(reply));
      }, xrpc::http_limits(*options.host_policy), identity);
      native = routes.get();
      ready.set_value();
      announced = true;
      while (!stopping.load(std::memory_order_acquire)) {
        world.poll(milliseconds(5));
        routes->finish();
      }
      // Native loop has stopped; all framework-owned reply references end
      // before the framework releases its shared transport lease.
      routes.reset();
      world.request_stop();
    } catch (...) {
      if (!announced) ready.set_exception(std::current_exception());
      else stopping.store(true, std::memory_order_release);
    }
  });
  try {
    started.get();
    xrpc::HttpClient client(options.socket_path, {}, options.instance_id);
    require(client.call(request("GET", "/v1/world/ping"), xrpc::Clock::now()+seconds(1)).body == "world",
            "framework route lost when adding hold");
    auto result = std::async(std::launch::async, [&] {
      return client.call(request("POST", "/v1/chassis/hold", batch(0, true)),
                         xrpc::Clock::now()+milliseconds(200));
    });
    await([&] { return native->pending() == 1; });
    Sink sink;
    require(sink.writes == 0, "embedded framework IO invoked hardware");
    native->control_tick(Sink::zero, &sink);
    require(result.get().status == 200 && sink.writes == 1 && !sink.wrong_thread && native->held(),
            "embedded hold did not complete through sole native output");
    await([&] { return native->pending() == 0; });
  } catch (...) {
    stopping.store(true, std::memory_order_release);
    framework.join();
    throw;
  }
  stopping.store(true, std::memory_order_release);
  framework.join();
  require(!std::filesystem::exists(options.socket_path), "framework endpoint leaked on shutdown");
}
} // namespace
int main() {
  try {
    std::cerr << "CHECK state_contract\n";
    state_contract();
    std::cerr << "CHECK transport_contract\n";
    runtime_failure_contract();
    transport_contract();
    std::cerr << "CHECK shutdown_contract\n";
    shutdown_contract();
    std::cerr << "CHECK shared_host_contract\n";
    shared_host_contract();
    std::cout << "PASS native hold CAS, batch, instance, deadline, output and failure contract\n";
  } catch (const std::exception &error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
