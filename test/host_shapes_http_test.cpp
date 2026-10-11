// The two world-host shapes of the README, "Instantiating chassis-hold", with stand-ins for their worlds: a Gazebo
// world plugin that ticks in the physics step, and an XSIM world thread that sleeps between steps and is woken by the
// library. Each gives its XRPC http.v1 host an HttpAdapter and is called like a world host over its Unix socket.
#include "check.hpp"
#include "fixtures.hpp"
#include "http_fixtures.hpp"

#include <xgc2/chassis_hold/chassis_hold.hpp>
#include <xgc2/chassis_hold/http_adapter.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

using namespace xgc2::chassis_hold;
using namespace http_fixtures;
using fixtures::wait_until;

namespace {

// ---- the XRPC server of a world host: its hooks and its describe route ---------------------------------------------

class WorldHost {
public:
  using RequestHook = std::function<bool(const xrpc::HttpRequest &, const xrpc::HttpReply &)>;

  explicit WorldHost(const std::string &instance_id)
      : host_(instance_id, [this](xrpc::HttpRequest request, xrpc::HttpReply reply) { http_request(std::move(request), std::move(reply)); }) {}

  // The world host's own registration points for a capability (set up before the first request).
  void add_request_hook(RequestHook hook) {
    std::lock_guard<std::mutex> lock(mutex_);
    hooks_.push_back(std::move(hook));
  }
  void add_describe_capability(std::function<std::string()> entry) {
    std::lock_guard<std::mutex> lock(mutex_);
    capabilities_.push_back(std::move(entry));
  }

  xrpc::HttpResponse call(const std::string &verb, const std::string &target, const std::string &body = "{}",
                          milliseconds timeout = seconds(5)) const {
    return host_.call(verb, target, body, timeout);
  }
  xrpc::HttpResponse post(const std::string &method, const json &body = json::object()) const {
    return call("POST", std::string("/v1/call/") + service_name + "/" + method, body.dump());
  }
  json ok(const std::string &method, const json &body = json::object()) const {
    const xrpc::HttpResponse response = post(method, body);
    if (response.status != 200) {
      test::fail(__FILE__, __LINE__, method + " answered " + std::to_string(response.status) + ": " + response.body);
      return json::object();
    }
    return parse(response.body);
  }
  json described_capabilities() const {
    const xrpc::HttpResponse response = call("GET", "/v1/describe");
    return parse(response.body)["facts"]["capabilities"];
  }
  RobotState state(const std::string &robot) const {
    const json s = ok("State", json{{"robot_ids", json::array({robot})}})["robots"][0];
    RobotState state;
    state.held = s["held"].get<bool>();
    state.revision = s["revision"].get<std::uint64_t>();
    state.stage = s["stage"] == "released"       ? Stage::released
                  : s["stage"] == "gated"        ? Stage::gated
                  : s["stage"] == "zero_written" ? Stage::zero_written
                                                 : Stage::stopped;
    return state;
  }
  json release(const std::string &instance, const std::string &robot, std::uint64_t revision) const {
    return ok("Release", json{{"expected_instance", instance},
                              {"changes", json::array({json{{"robot_id", robot}, {"expected_revision", revision}}})}});
  }
  const std::string &instance() const { return host_.instance(); }

private:
  void http_request(xrpc::HttpRequest request, xrpc::HttpReply reply) {
    std::vector<RequestHook> hooks;
    std::vector<std::function<std::string()>> capabilities;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      hooks = hooks_;
      capabilities = capabilities_;
    }
    for (const auto &hook : hooks)
      if (hook(request, reply))
        return;
    if (request.method == "GET" && request.target == "/v1/describe") {
      json list = json::array();
      for (const auto &entry : capabilities)
        list.push_back(parse(entry()));
      xrpc::HttpResponse response;
      response.headers.emplace_back("Content-Type", "application/json");
      response.body = json{{"facts", {{"capabilities", list}}}}.dump();
      reply.complete(std::move(response));
      return;
    }
    reply.complete(xrpc::http_error(404, "not_found", "unknown route"));
  }

  std::mutex mutex_;
  std::vector<RequestHook> hooks_;
  std::vector<std::function<std::string()>> capabilities_;
  HttpHost host_;
};

struct Twist {
  double linear = 0, angular = 0;
};

// What a chassis plugin of a simulator keeps per model: the command that the physics step applies, and the velocity
// the model has as a result (a perfect chassis follows its command at once).
class Chassis {
public:
  void add(const std::string &model) {
    std::lock_guard<std::mutex> lock(mutex_);
    targets_[model] = Twist();
  }
  void remove(const std::string &model) {
    std::lock_guard<std::mutex> lock(mutex_);
    targets_.erase(model);
  }
  bool set_target(const std::string &model, double linear, double angular) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = targets_.find(model);
    if (found == targets_.end())
      return false;
    found->second = {linear, angular};
    if (linear != 0.0)
      driven_[model].push_back(linear);
    return true;
  }
  // Every non-zero speed that was ever applied to the model.
  std::vector<double> driven(const std::string &model) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = driven_.find(model);
    return found == driven_.end() ? std::vector<double>() : found->second;
  }
  bool ever_driven(const std::string &model, double linear) const {
    for (const double speed : driven(model))
      if (speed == linear)
        return true;
    return false;
  }
  bool set_target(const std::string &model, const Twist &twist) { return set_target(model, twist.linear, twist.angular); }
  Twist target(const std::string &model) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = targets_.find(model);
    return found == targets_.end() ? Twist() : found->second;
  }

private:
  mutable std::mutex mutex_;
  std::map<std::string, Twist> targets_;
  std::map<std::string, std::vector<double>> driven_;
};

// A command with the stamp of the moment it entered the host's queue.
struct Queued {
  std::string model;
  Twist twist;
  TimeNs enqueued_ns = 0;
};
template <typename T>
class Inbox {
public:
  void push(T item) {
    std::lock_guard<std::mutex> lock(mutex_);
    items_.push_back(std::move(item));
  }
  std::vector<T> take() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<T> taken(items_.begin(), items_.end());
    items_.clear();
    return taken;
  }

private:
  std::mutex mutex_;
  std::deque<T> items_;
};

// ---- Gazebo world plugin --------------------------------------------------------------------------------------------

// Zero output: the chassis plugin of the model keeps the command that its physics step applies.
struct ModelZero : ZeroSink {
  explicit ModelZero(Chassis &c) : chassis(c) {}
  bool write_zero(const std::string &model, std::string &) override { return chassis.set_target(model, 0.0, 0.0); }
  Chassis &chassis;
};
// Feedback: ground-truth velocity of the model, read in the physics thread and stored.
struct ModelVelocity : FeedbackSource {
  ModelVelocity(Chassis &c, std::function<TimeNs()> n) : chassis(c), now(std::move(n)) {}
  bool latest(const std::string &model, Feedback &sample) override {
    const Twist twist = chassis.target(model);
    sample = {twist.linear, twist.angular, now()};
    return true;
  }
  Chassis &chassis;
  std::function<TimeNs()> now;
};

// The world with its physics thread: WorldUpdateBegin callbacks run in every iteration, unless the world is paused.
class GazeboWorld {
public:
  GazeboWorld() : thread_([this] {
    while (running_) {
      if (!paused_) {
        std::function<void()> callback;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          callback = update_begin_;
        }
        if (callback)
          callback();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }) {}
  ~GazeboWorld() {
    running_ = false;
    thread_.join();
  }
  void connect_world_update_begin(std::function<void()> callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    update_begin_ = std::move(callback);
  }
  void disconnect() { connect_world_update_begin(nullptr); }
  std::atomic<bool> paused_{false};

private:
  std::mutex mutex_;
  std::function<void()> update_begin_;
  std::atomic<bool> running_{true};
  std::thread thread_;
};

class HoldWorldPlugin {
public:
  HoldWorldPlugin(GazeboWorld &world, WorldHost &host, Chassis &chassis)
      : world_(world), chassis_(chassis), zero_(chassis), velocity_(chassis, [this] { return hold_->now(); }) {
    // Load(): the identity of the world host's HttpServer is the identity of the domain.
    HoldOptions options;
    options.domain.instance_id = host.instance();
    options.domain.stopped_dwell = std::chrono::milliseconds(30);
    options.service.release_wait = std::chrono::milliseconds(150);
    options.feedback = &velocity_;
    hold_ = std::make_unique<ChassisHold>(options, zero_);
    adapter_ = std::make_unique<HttpAdapter>(hold_->service());
    host.add_request_hook([this](const xrpc::HttpRequest &request, const xrpc::HttpReply &reply) {
      return adapter_->handle(request, reply);  // POST /v1/call/xgc2.chassis.hold/<Method>; false: not ours
    });
    host.add_describe_capability([this] { return adapter_->capability_json(); });
    world_.connect_world_update_begin([this] {
      if (!held_back_commands_)
        apply_queued_commands();
      hold_->tick();
    });
  }
  ~HoldWorldPlugin() { world_.disconnect(); }

  // Models appear and disappear; a model that is created again inherits its HOLD.
  void on_model_created(const std::string &model) {
    chassis_.add(model);
    hold_->add(model);
  }
  void on_model_deleted(const std::string &model) {
    hold_->remove(model);
    chassis_.remove(model);
  }

  // The model's command callback (transport thread): stamp at enqueue, apply in the physics step.
  void on_cmd_vel(const std::string &model, const Twist &twist) { queue_.push({model, twist, hold_->now()}); }
  void apply_queued_commands() {  // physics thread, before tick()
    for (const auto &c : queue_.take())
      if (hold_->admit(c.model, c.enqueued_ns))
        chassis_.set_target(c.model, c.twist);
  }

  GazeboWorld &world_;
  Chassis &chassis_;
  ModelZero zero_;
  ModelVelocity velocity_;
  std::unique_ptr<ChassisHold> hold_;
  std::unique_ptr<HttpAdapter> adapter_;
  Inbox<Queued> queue_;
  std::atomic<bool> held_back_commands_{false};  // a slow command consumer
};

// ---- XSIM world thread ----------------------------------------------------------------------------------------------

class XsimWorld {
public:
  struct Command {
    std::string entity;
    Twist twist;
    TimeNs enqueued_ns = 0;
  };

  XsimWorld(WorldHost &host, Chassis &chassis, std::chrono::milliseconds step_period, const std::vector<std::string> &entities)
      : chassis_(chassis), zero_(chassis), velocity_(chassis, [this] { return hold->now(); }), step_period_(step_period) {
    HoldOptions options;
    options.domain.instance_id = host.instance();  // the HttpServer identity
    options.domain.wake = [this] {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        wanted_ = true;
      }
      wake_.notify_one();
    };
    options.domain.stopped_dwell = std::chrono::milliseconds(30);
    options.service.release_wait = std::chrono::milliseconds(600);
    options.feedback = &velocity_;
    for (const auto &entity : entities) {
      chassis_.add(entity);
      options.robots.push_back(entity);
    }
    hold = std::make_unique<ChassisHold>(options, zero_);
    adapter = std::make_unique<HttpAdapter>(hold->service());
    host.add_request_hook([this](const xrpc::HttpRequest &request, const xrpc::HttpReply &reply) {
      return adapter->handle(request, reply);  // the hold capability
    });
    host.add_describe_capability([this] { return adapter->capability_json(); });
    thread_ = std::thread([this] { run(); });
  }
  ~XsimWorld() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    wake_.notify_one();
    thread_.join();
  }

  // Input side (I/O or ROS threads): stamp when the command enters the queue.
  void on_command(const std::string &entity, const Twist &twist) { inbox_.push({entity, twist, hold->now()}); }
  void set_paused(bool paused) {
    paused_ = paused;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      wanted_ = true;
    }
    wake_.notify_one();
  }
  int steps() const { return steps_; }

  // The world thread:
  void run() {
    for (;;) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait_for(lock, step_period_, [&] { return wanted_ || stopping_; });
        wanted_ = false;
        if (stopping_)
          break;
      }
      if (!paused_) {
        for (const auto &c : inbox_.take())
          if (hold->admit(c.entity, c.enqueued_ns))
            chassis_.set_target(c.entity, c.twist);
        ++steps_;
      }
      hold->tick();  // every iteration, also while the world is paused
    }
  }

  Chassis &chassis_;
  ModelZero zero_;
  ModelVelocity velocity_;
  std::unique_ptr<ChassisHold> hold;
  std::unique_ptr<HttpAdapter> adapter;

private:
  std::chrono::milliseconds step_period_;
  std::mutex mutex_;
  std::condition_variable wake_;
  bool wanted_ = false;
  bool stopping_ = false;
  std::atomic<bool> paused_{false};
  std::atomic<int> steps_{0};
  Inbox<Command> inbox_;
  std::thread thread_;
};

}  // namespace

TEST(a_gazebo_style_plugin_follows_its_models_and_gates_commands_by_receipt_time) {
  GazeboWorld world;
  Chassis chassis;
  WorldHost host(xrpc::new_instance_id());
  HoldWorldPlugin plugin(world, host, chassis);
  plugin.on_model_created("gz-1");
  plugin.on_model_created("gz-2");

  // The describe facts list the capability for the models that exist.
  const json capabilities = host.described_capabilities();
  REQUIRE(capabilities.size() == 1);
  CHECK_EQ(capabilities[0]["name"].get<std::string>(), "xgc2.chassis.hold");
  CHECK_EQ(capabilities[0]["entities"].size(), 2);
  CHECK_EQ(host.ok("Describe")["instance"].get<std::string>(), host.instance());  // one identity for the host and the domain

  // A model is spawned while the world runs.
  plugin.on_model_created("gz-3");
  CHECK_EQ(host.described_capabilities()[0]["entities"].size(), 3);
  plugin.on_cmd_vel("gz-3", {0.7, 0.0});
  CHECK(wait_until([&] { return chassis.target("gz-3").linear == 0.7; }));

  // Engage over http.v1: the physics thread ticks, and the reply says the zero is written.
  const json engaged = host.ok("Engage", json{{"robot_ids", json::array({"gz-1"})}});
  CHECK_EQ(engaged["robots"][0]["outcome"].get<std::string>(), "engaged");
  CHECK_EQ(engaged["robots"][0]["stage"].get<std::string>(), "zero_written");
  plugin.on_cmd_vel("gz-1", {0.5, 0.0});
  plugin.on_cmd_vel("gz-2", {0.5, 0.0});
  CHECK(wait_until([&] { return chassis.target("gz-2").linear == 0.5; }));
  CHECK(!chassis.ever_driven("gz-1", 0.5));  // held: the command was dropped, the zero stands
  CHECK(wait_until([&] { return host.state("gz-1").stage == Stage::stopped; }));  // ground truth shows rest

  // A command received while the robot was held is not applied after the release, even when the world applies its
  // queue late; one received after the release is.
  plugin.held_back_commands_ = true;  // a slow command consumer: the queue is not drained for a while
  plugin.on_cmd_vel("gz-1", {0.9, 0.0});
  const json released = host.release(host.instance(), "gz-1", host.state("gz-1").revision);
  CHECK_EQ(released["robots"][0]["outcome"].get<std::string>(), "released");
  plugin.on_cmd_vel("gz-1", {0.6, 0.0});
  plugin.held_back_commands_ = false;
  CHECK(wait_until([&] { return chassis.target("gz-1").linear == 0.6; }));
  CHECK(!chassis.ever_driven("gz-1", 0.9));
  CHECK(plugin.hold_->admit("gz-1", plugin.hold_->now()));

  // A model that is deleted and created again inherits its HOLD.
  host.ok("Engage", json{{"robot_ids", json::array({"gz-2"})}});
  plugin.on_model_deleted("gz-2");
  CHECK_EQ(host.described_capabilities()[0]["entities"].size(), 2);
  CHECK_EQ(host.post("State", json{{"robot_ids", json::array({"gz-2"})}}).status, 404);
  plugin.on_model_created("gz-2");
  CHECK(host.state("gz-2").held);
  CHECK(wait_until([&] { return host.state("gz-2").stage == Stage::stopped; }));
}

TEST(a_gazebo_style_plugin_closes_admission_in_a_paused_world_and_applies_the_rest_when_it_runs_again) {
  GazeboWorld world;
  Chassis chassis;
  WorldHost host(xrpc::new_instance_id());
  HoldWorldPlugin plugin(world, host, chassis);
  plugin.on_model_created("gz-1");
  plugin.on_model_created("gz-2");

  const json first = host.ok("Engage", json{{"robot_ids", json::array({"gz-1"})}});
  CHECK_EQ(first["robots"][0]["stage"].get<std::string>(), "zero_written");
  const std::uint64_t revision = host.state("gz-1").revision;
  world.paused_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));  // the last iteration is over

  // Paused: an engage closes admission at once though the machine has to wait for an iteration, and the reply says
  // the zero is not written yet.
  const json second = host.ok("Engage", json{{"robot_ids", json::array({"gz-2"})}});
  CHECK_EQ(second["robots"][0]["outcome"].get<std::string>(), "engaged");
  CHECK_EQ(second["robots"][0]["stage"].get<std::string>(), "gated");
  CHECK(!plugin.hold_->admit("gz-2", plugin.hold_->now()));
  // A release needs the tick, and says so when it cannot get one.
  const json late = host.release(host.instance(), "gz-1", revision);
  CHECK_EQ(late["robots"][0]["outcome"].get<std::string>(), "expired");
  CHECK(late["robots"][0]["held"].get<bool>());

  world.paused_ = false;  // the world runs again: gz-2 gets its zero, and the withdrawn release does not come back
  CHECK(wait_until([&] { return host.state("gz-2").stage == Stage::zero_written || host.state("gz-2").stage == Stage::stopped; }));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  CHECK(host.state("gz-1").held);
  const json released = host.release(host.instance(), "gz-1", host.state("gz-1").revision);
  CHECK_EQ(released["robots"][0]["outcome"].get<std::string>(), "released");
}

TEST(an_xsim_style_world_thread_is_woken_by_the_library_and_releases_a_paused_world) {
  Chassis chassis;
  WorldHost host(xrpc::new_instance_id());
  // The thread sleeps two seconds between steps: only the library's wake-up makes the requests quick.
  XsimWorld world(host, chassis, std::chrono::milliseconds(2000), {"xsim-1", "xsim-2"});
  CHECK_EQ(host.described_capabilities()[0]["entities"].size(), 2);

  const auto started = std::chrono::steady_clock::now();
  const json engaged = host.ok("Engage", json{{"robot_ids", json::array({"xsim-1"})}});
  CHECK_EQ(engaged["robots"][0]["stage"].get<std::string>(), "zero_written");
  CHECK(std::chrono::steady_clock::now() - started < std::chrono::milliseconds(1000));
  world.on_command("xsim-1", {0.4, 0.0});
  world.on_command("xsim-2", {0.4, 0.0});
  world.set_paused(false);  // wakes the thread to apply the inbox
  CHECK(wait_until([&] { return chassis.target("xsim-2").linear == 0.4; }));
  CHECK(!chassis.ever_driven("xsim-1", 0.4));  // held

  // Paused world: commands wait in the inbox, and HOLD still works because the thread ticks while paused.
  world.set_paused(true);
  world.on_command("xsim-1", {0.8, 0.0});  // received while the robot is held and the world is paused
  const json released = host.release(host.instance(), "xsim-1", host.state("xsim-1").revision);
  CHECK_EQ(released["robots"][0]["outcome"].get<std::string>(), "released");
  CHECK(world.hold->admit("xsim-1", world.hold->now()));
  world.on_command("xsim-1", {0.3, 0.0});  // received after the release
  world.set_paused(false);
  CHECK(wait_until([&] { return chassis.target("xsim-1").linear == 0.3; }));
  CHECK(!chassis.ever_driven("xsim-1", 0.8));  // the inbox had both; only the one received after the release counts
}

TEST_MAIN()
