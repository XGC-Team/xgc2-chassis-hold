// The http.v1 adapter on a real xrpc http host over a Unix socket, called by the xrpc http client: the way a world
// host (Gazebo, XSIM) serves the capability next to its own routes.
#include "check.hpp"
#include "fixtures.hpp"

#include <xgc2/chassis_hold/chassis_hold.hpp>
#include <xgc2/chassis_hold/http_adapter.hpp>
#include <xgc2/xrpc/http.hpp>

#include <nlohmann/json.hpp>

#include <strings.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

using namespace xgc2::chassis_hold;
namespace xrpc = xgc2::xrpc;
using json = nlohmann::json;
using fixtures::Sink;
using fixtures::TempDir;
using fixtures::TickThread;
using fixtures::wait_until;
using std::chrono::milliseconds;
using std::chrono::seconds;

namespace {

std::string header(const xrpc::HttpResponse &response, const std::string &name) {
  for (const auto &entry : response.headers)
    if (strcasecmp(entry.first.c_str(), name.c_str()) == 0)
      return entry.second;
  return std::string();
}

json parse(const std::string &text) {
  try {
    return json::parse(text);
  } catch (const std::exception &e) {
    test::fail(__FILE__, __LINE__, std::string("not JSON: ") + e.what() + ": " + text);
    return json();
  }
}

bool is_envelope(const xrpc::HttpResponse &response, const std::string &code) {
  const json body = parse(response.body);
  return body.is_object() && body.size() == 1 && body.count("error") == 1 && body["error"].size() == 2 &&
         body["error"]["code"] == code && body["error"]["message"].is_string() &&
         !body["error"]["message"].get<std::string>().empty() &&
         header(response, "Content-Type").find("application/json") == 0;
}

struct RigOptions {
  bool tick = true;
  std::vector<std::string> robots = {"gz-1", "gz-2"};
  ServiceOptions service;
};

// A world host: a hold, an xrpc http server that gives every request to the adapter first and has a route of its
// own, and a client pinned to the instance of the host.
struct Rig {
  explicit Rig(RigOptions rig = {}) {
    options.robots = rig.robots;
    options.service = rig.service;
    options.domain.instance_id = xrpc::new_instance_id();  // the identity of the http host, shared with the domain
    hold = std::make_unique<ChassisHold>(options, sink);
    adapter = std::make_unique<HttpAdapter>(hold->service());
    socket = dir.path() + "/hold.sock";
    xrpc::UnixOptions endpoint;
    endpoint.path = socket;
    server = std::make_unique<xrpc::HttpServer>(
        endpoint, [this](xrpc::HttpRequest request, xrpc::HttpReply reply) { dispatch(std::move(request), std::move(reply)); },
        xrpc::HttpLimits{}, xrpc::HttpIdentity{hold->instance_id(), {"/v1/describe"}});
    loop = std::thread([this] { server->run(); });
    if (rig.tick)
      start_ticker();
  }
  ~Rig() {
    ticker.reset();
    server->request_stop();
    loop.join();
  }

  void start_ticker() { ticker = std::make_unique<TickThread>([this] { hold->tick(); }); }

  // The host's own routes.
  void dispatch(xrpc::HttpRequest request, xrpc::HttpReply reply) {
    if (adapter->handle(request, reply))
      return;
    ++left_to_the_host;
    if (request.method == "GET" && request.target == "/v1/describe") {
      xrpc::HttpResponse response;
      response.headers.emplace_back("Content-Type", "application/json");
      response.body = json{{"facts", {{"capabilities", json::array({parse(adapter->capability_json())})}}}}.dump();
      reply.complete(std::move(response));
      return;
    }
    reply.complete(xrpc::http_error(404, "not_found", "unknown route"));
  }

  xrpc::HttpResponse call(const std::string &verb, const std::string &target, const std::string &body = "{}",
                          milliseconds timeout = seconds(5), const std::string &pin = std::string()) {
    xrpc::HttpClient client(socket, xrpc::HttpLimits{}, pin.empty() ? hold->instance_id() : pin);
    return call_with(client, verb, target, body, timeout);
  }
  static xrpc::HttpResponse call_with(xrpc::HttpClient &client, const std::string &verb, const std::string &target,
                                      const std::string &body, milliseconds timeout) {
    xrpc::HttpRequest request;
    request.method = verb;
    request.target = target;
    request.body = body;
    request.headers.emplace_back("Content-Type", "application/json");
    return client.call(std::move(request), xrpc::Clock::now() + timeout);
  }
  xrpc::HttpResponse post(const std::string &method, const std::string &body = "{}",
                          milliseconds timeout = seconds(5)) {
    return call("POST", HttpAdapter::route_prefix() + method, body, timeout);
  }
  std::string release_body(const std::string &robot, std::uint64_t revision) const {
    return json{{"expected_instance", hold->instance_id()},
                {"changes", json::array({json{{"robot_id", robot}, {"expected_revision", revision}}})}}
        .dump();
  }

  // Destruction order: the tick thread, the server, the adapter, the hold, the sink, the directory.
  TempDir dir;
  Sink sink;
  HoldOptions options;
  std::unique_ptr<ChassisHold> hold;
  std::unique_ptr<HttpAdapter> adapter;
  std::string socket;
  std::unique_ptr<xrpc::HttpServer> server;
  std::thread loop;
  std::unique_ptr<TickThread> ticker;
  std::atomic<int> left_to_the_host{0};
};

}  // namespace

TEST(a_hold_cycle_over_http_with_the_instance_of_the_host) {
  Rig rig;
  const std::string instance = rig.hold->instance_id();

  const xrpc::HttpResponse describe = rig.post("Describe");
  REQUIRE(describe.status == 200);
  CHECK_EQ(header(describe, "Content-Type"), "application/json");
  CHECK_EQ(header(describe, "X-Xrpc-Instance-ID"), instance);  // the host's header, the domain's id
  const json described = parse(describe.body);
  CHECK_EQ(described["instance"].get<std::string>(), instance);
  REQUIRE(described["robots"].size() == 2);
  CHECK_EQ(described["robots"][0].get<std::string>(), "gz-1");

  const xrpc::HttpResponse engage = rig.post("Engage", "{\"robot_ids\":[\"gz-1\"]}");
  REQUIRE(engage.status == 200);
  const json engaged = parse(engage.body);
  CHECK_EQ(engaged["robots"][0]["outcome"].get<std::string>(), "engaged");
  CHECK_EQ(engaged["robots"][0]["stage"].get<std::string>(), "zero_written");  // the tick ran meanwhile
  CHECK_EQ(engaged["robots"][0]["revision"].get<std::uint64_t>(), 1);
  CHECK(!rig.hold->admit("gz-1", rig.hold->now()));
  CHECK(rig.hold->admit("gz-2", rig.hold->now()));

  const json state = parse(rig.post("State", "{\"robot_ids\":[\"gz-1\"]}").body);
  CHECK(state["robots"][0]["held"].get<bool>());

  const xrpc::HttpResponse release = rig.post("Release", rig.release_body("gz-1", 1));
  REQUIRE(release.status == 200);
  CHECK_EQ(parse(release.body)["robots"][0]["outcome"].get<std::string>(), "released");
  CHECK(rig.hold->admit("gz-1", rig.hold->now()));
  const json replay = parse(rig.post("Release", rig.release_body("gz-1", 1)).body);
  CHECK_EQ(replay["robots"][0]["outcome"].get<std::string>(), "not_held");

  // A release that names another instance conflicts for every robot instead of failing.
  rig.hold->domain().engage({"gz-1"});
  const json stale = parse(rig.post("Release", json{{"expected_instance", std::string(32, 'a')},
                                                    {"changes", json::array({json{{"robot_id", "gz-1"}, {"expected_revision", 3}}})}}
                                                       .dump()).body);
  CHECK_EQ(stale["robots"][0]["outcome"].get<std::string>(), "conflict");
  CHECK(rig.hold->domain().state({"gz-1"}).robots[0].held);
  CHECK_EQ(rig.left_to_the_host.load(), 0);
}

TEST(errors_use_the_xrpc_envelope_and_the_http_statuses_of_the_vocabulary) {
  RigOptions options;
  options.service.release_wait = milliseconds(100);
  Rig rig(options);
  const std::string prefix = HttpAdapter::route_prefix();

  const xrpc::HttpResponse get = rig.call("GET", prefix + "State");
  CHECK_EQ(get.status, 405);
  CHECK_EQ(header(get, "Allow"), "POST");
  CHECK(is_envelope(get, "invalid_argument"));
  CHECK_EQ(rig.call("PUT", prefix + "Engage", "{\"all\":true}").status, 405);
  CHECK(!rig.hold->domain().state({"gz-1"}).robots[0].held);

  const xrpc::HttpResponse unknown = rig.post("Nope");
  CHECK_EQ(unknown.status, 404);
  CHECK(is_envelope(unknown, "not_found"));
  CHECK_EQ(rig.call("POST", prefix, "{}").status, 404);  // the capability has no method with an empty name
  CHECK_EQ(rig.call("POST", prefix + "State/extra", "{}").status, 404);

  const xrpc::HttpResponse invalid = rig.post("Engage", "{}");
  CHECK_EQ(invalid.status, 400);
  CHECK(is_envelope(invalid, "invalid_argument"));
  CHECK_EQ(rig.post("Engage", "{\"robot_ids\":[").status, 400);
  CHECK_EQ(rig.post("Describe", "{\"extra\":1}").status, 400);
  const xrpc::HttpResponse query = rig.call("POST", prefix + "State?robot=gz-1", "{}");
  CHECK_EQ(query.status, 400);
  CHECK(is_envelope(query, "invalid_argument"));

  const xrpc::HttpResponse missing = rig.post("State", "{\"robot_ids\":[\"gz-1\",\"nobody\"]}");
  CHECK_EQ(missing.status, 404);
  CHECK(is_envelope(missing, "not_found"));
  CHECK(parse(missing.body)["error"]["message"].get<std::string>().find("nobody") != std::string::npos);

  // Nothing ticks the host's native loop once the ticker is gone for good: the release cannot be applied.
  rig.ticker.reset();
  CHECK_EQ(rig.post("Engage", "{\"robot_ids\":[\"gz-1\"]}").status, 200);
  const xrpc::HttpResponse late = rig.post("Release", rig.release_body("gz-1", 1));
  CHECK_EQ(late.status, 504);
  CHECK(is_envelope(late, "deadline_exceeded"));
  CHECK(rig.hold->domain().state({"gz-1"}).robots[0].held);
  CHECK_EQ(rig.left_to_the_host.load(), 0);
}

TEST(requests_outside_the_capability_stay_with_the_host) {
  Rig rig;
  const xrpc::HttpResponse own = rig.call("GET", "/v1/describe");
  CHECK_EQ(own.status, 200);
  CHECK_EQ(rig.left_to_the_host.load(), 1);
  for (const char *target : {"/v1/call/other.service/Engage", "/v1/call/xgc2.chassis.holdx/Engage",
                             "/v1/call/xgc2.chassis.hold", "/v1/call", "/", "/xgc2.chassis.hold/Engage"}) {
    const xrpc::HttpResponse response = rig.call("POST", target, "{}");
    CHECK_EQ(response.status, 404);  // the host's route table answered
  }
  CHECK_EQ(rig.left_to_the_host.load(), 7);
  CHECK(!rig.hold->domain().state({"gz-1"}).robots[0].held);
}

TEST(the_describe_facts_list_the_capability_and_the_roster_now) {
  Rig rig;
  CHECK_EQ(std::string(HttpAdapter::route_prefix()), std::string("/v1/call/") + service_name + "/");
  CHECK_EQ(rig.adapter->capability_json(), "{\"entities\":[\"gz-1\",\"gz-2\"],\"name\":\"xgc2.chassis.hold\"}");

  const json facts = parse(rig.call("GET", "/v1/describe").body)["facts"];
  REQUIRE(facts["capabilities"].size() == 1);
  CHECK_EQ(facts["capabilities"][0]["name"].get<std::string>(), "xgc2.chassis.hold");
  REQUIRE(facts["capabilities"][0]["entities"].size() == 2);

  // A model that is spawned or deleted changes the entities at once.
  CHECK(rig.hold->add("gz-3"));
  CHECK(rig.hold->remove("gz-1"));
  const json later = parse(rig.call("GET", "/v1/describe").body)["facts"]["capabilities"][0]["entities"];
  REQUIRE(later.size() == 2);
  CHECK_EQ(later[0].get<std::string>(), "gz-2");
  CHECK_EQ(later[1].get<std::string>(), "gz-3");
  CHECK_EQ(parse(rig.post("Describe").body)["robots"].size(), 2);

  RigOptions empty;
  empty.robots = {};
  Rig bare(empty);
  CHECK_EQ(bare.adapter->capability_json(), "{\"entities\":[],\"name\":\"xgc2.chassis.hold\"}");
}

TEST(a_request_that_waits_for_the_tick_does_not_block_the_io_thread_of_the_host) {
  RigOptions options;
  options.tick = false;
  options.service.reply_wait = milliseconds(600);
  Rig rig(options);

  std::atomic<int> status{0};
  std::string stage;
  std::thread engager([&] {
    const xrpc::HttpResponse response = rig.post("Engage", "{\"robot_ids\":[\"gz-1\"]}", seconds(5));
    status = response.status;
    stage = parse(response.body)["robots"][0]["stage"].get<std::string>();
  });
  CHECK(wait_until([&] { return rig.hold->domain().state({"gz-1"}).robots[0].held; }));
  CHECK(!rig.hold->admit("gz-1", rig.hold->now()));  // the gate does not wait for the reply

  // The host's loop is free while the engage waits: other requests are answered at once.
  const auto started = xrpc::Clock::now();
  CHECK_EQ(rig.post("State").status, 200);
  CHECK_EQ(rig.call("GET", "/v1/describe").status, 200);
  CHECK(xrpc::Clock::now() - started < milliseconds(400));
  CHECK_EQ(status.load(), 0);  // still waiting

  rig.start_ticker();  // the tick comes before the wait ends: the reply reports the zero
  engager.join();
  CHECK_EQ(status.load(), 200);
  CHECK_EQ(stage, "zero_written");
}

TEST(a_caller_pinned_to_another_instance_is_refused_by_the_host_and_nothing_runs) {
  Rig rig;
  const std::string other = xrpc::new_instance_id();
  bool refused = false;
  try {
    const xrpc::HttpResponse response =
        rig.call("POST", HttpAdapter::route_prefix() + std::string("Engage"), "{\"all\":true}", seconds(5), other);
    refused = response.status == 409;
  } catch (const xrpc::HttpCallError &) {
    refused = true;  // the client reports a response of another instance as an unknown outcome
  }
  CHECK(refused);
  for (const auto &robot : rig.hold->domain().state().robots)
    CHECK(!robot.held);
}

TEST(clients_race_for_one_transition) {
  Rig rig;
  constexpr int callers = 6;
  for (int round = 0; round < 5; ++round) {
    const std::uint64_t revision = rig.hold->domain().state({"gz-1"}).robots[0].revision;
    std::atomic<int> engaged{0}, already{0}, released{0}, other{0};
    std::vector<std::thread> threads;
    for (int c = 0; c < callers; ++c)
      threads.emplace_back([&] {
        const xrpc::HttpResponse response = rig.post("Engage", "{\"robot_ids\":[\"gz-1\"]}");
        const std::string outcome =
            response.status == 200 ? parse(response.body)["robots"][0]["outcome"].get<std::string>() : "";
        (outcome == "engaged" ? engaged : outcome == "already_held" ? already : other)++;
      });
    for (auto &thread : threads)
      thread.join();
    CHECK_EQ(engaged.load(), 1);
    CHECK_EQ(already.load(), callers - 1);
    CHECK_EQ(other.load(), 0);

    threads.clear();
    for (int c = 0; c < callers; ++c)
      threads.emplace_back([&] {
        const xrpc::HttpResponse response = rig.post("Release", rig.release_body("gz-1", revision + 1));
        const std::string outcome =
            response.status == 200 ? parse(response.body)["robots"][0]["outcome"].get<std::string>() : "";
        (outcome == "released" ? released : other)++;
      });
    for (auto &thread : threads)
      thread.join();
    CHECK_EQ(released.load(), 1);
    CHECK_EQ(other.load(), callers - 1);  // not_held for the replays
  }
}

TEST(closing_the_host_with_a_reply_pending_is_harmless) {
  RigOptions options;
  options.tick = false;
  options.service.reply_wait = seconds(5);
  auto rig = std::make_unique<Rig>(options);
  std::atomic<bool> failed{false};
  const std::string socket = rig->socket;
  const std::string instance = rig->hold->instance_id();
  std::thread caller([&] {
    try {
      xrpc::HttpClient client(socket, xrpc::HttpLimits{}, instance);
      xrpc::HttpRequest request;
      request.method = "POST";
      request.target = std::string(HttpAdapter::route_prefix()) + "Engage";
      request.body = "{\"all\":true}";
      client.call(std::move(request), xrpc::Clock::now() + milliseconds(900));
    } catch (const xrpc::HttpCallError &) {
      failed = true;  // the host went away with the reply pending
    }
  });
  CHECK(wait_until([&] { return rig->hold->domain().state({"gz-1"}).robots[0].held; }));
  rig.reset();  // the server closes first, then the adapter and the service complete the waiting reply
  caller.join();
  CHECK(failed.load());
}

TEST_MAIN()
