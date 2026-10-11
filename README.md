# XGC2 Chassis HOLD

HOLD stops one robot's chassis from executing motion commands until it is explicitly released. This repository
is the shared C++17 library behind it: a state machine per robot on `xgc2-state-machine`, exposed as one XRPC
service, `xgc2.chassis.hold`, with a few lines of host code. The Wheeltec and Scout (AgileX) chassis drivers
and the Gazebo and XSIM world hosts are four shapes of the same library. The behavior and the wire surface are
defined in [chassis-hold](contracts/chassis-hold.md).

| Component | CMake target | Contents | Needs |
| --- | --- | --- | --- |
| `core` | `XgcChassisHold::core` | `ChassisHold` (the facade), `Domain` (state machines, admission, tick, feedback), `Service` (the JSON methods `Describe`, `State`, `Engage`, `Release`) | `xgc2-state-machine` (>= 0.1.3); nlohmann-json at build time only |
| `udp` | `XgcChassisHold::udp` | `UdpEndpoint`, the udp.v1 server a chassis driver exposes; the `xgc2-hold` command line tool | `core`; the XRPC `udp` component (`XgcXrpc::udp`, OpenSSL libcrypto) |
| `http` | `XgcChassisHold::http` | `HttpAdapter`, `POST /v1/call/xgc2.chassis.hold/<Method>` on a world host's http.v1 server | `core`; the XRPC `http` component (`XgcXrpc::http`, Boost >= 1.70: not on Ubuntu 18.04) |

```cmake
find_package(XgcChassisHold 0.2.0 CONFIG REQUIRED COMPONENTS core udp)   # a driver; add or use http for a world host
target_link_libraries(my_driver PRIVATE XgcChassisHold::udp)
```

## Instantiating chassis-hold

A host provides six things, and the library does the rest:

| The host provides | In the library | Used by |
| --- | --- | --- |
| the **roster**: the robot ids it serves | `HoldOptions::robots`, later `ChassisHold::add` / `remove` | all; Gazebo and XSIM add and remove models while they run |
| a **clock** for the receipt stamps | `HoldOptions::domain.clock` (default `steady_clock`), `ChassisHold::now()` | all |
| the **zero output**: write one zero command to the device | `ZeroSink::write_zero` | all |
| **feedback**: measured velocity, if it is trusted | `FeedbackSource::latest`, or `Domain::observe` | all that measure it |
| the **tick**: one call per control cycle, on the thread that owns device output | `ChassisHold::tick()` | all |
| a **wake-up** if its loop sleeps | `HoldOptions::domain.wake` | Gazebo, XSIM |

and for every motion command it is about to execute, one call: `hold.admit(robot, receipt_time)`, with the time the
command was received. One more object exposes the service: `UdpEndpoint(hold.service(), ...)` for a driver,
`HttpAdapter(hold.service())` for a world host.

### Admission uses the time a command was received

Stamp each command when it arrives, with the clock the domain uses, and give `admit` that stamp when you execute
the command, however late that is. A command that arrived while the robot was held, or before it was released, is
refused after the release: you do not have to drain your callback queue before releasing, or flush your transport.

* ROS drivers: `event.getReceiptTime().toNSec()` of the `ros::MessageEvent`, with the domain clock set to
  `ros::Time::now().toNSec()`.
* Simulators: `hold.now()` taken when the message enters the host's input queue, stored with the command, and
  passed to `admit` when the world thread applies it. Keep the default clock: a simulation clock stands still
  while the world is paused and cannot order a command before a release applied in the same instant.

An engage closes admission before it returns, without waiting for the tick. A release opens it when the tick has
applied it, and the release fence is that instant. The domain remembers at most 1024 distinct robot ids over its life
(`DomainOptions::max_robots`, per process); removed ids keep their state and stay counted, and `add` throws
`std::length_error` beyond the limit.

### Wheeltec serial driver (udp.v1)

A ROS1 node with a control loop that owns the serial port. `test/host_shapes_udp_test.cpp` runs this code against
stand-ins for ROS and the serial link.

```cpp
#include <xgc2/chassis_hold/chassis_hold.hpp>
#include <xgc2/chassis_hold/udp_endpoint.hpp>
using namespace xgc2::chassis_hold;

// Zero output: the serial frame of a stop command. False and a reason when the port refuses it.
struct WheeltecZero : ZeroSink {
  explicit WheeltecZero(SerialLink &l) : link(l) {}
  bool write_zero(const std::string &, std::string &error) override { return link.send_velocity(0.0, 0.0, error); }
  SerialLink &link;
};
// Feedback: the speeds the STM32 reports, stored by the serial reader. Return the stored sample; do not wait.
struct WheeltecSpeeds : FeedbackSource {
  explicit WheeltecSpeeds(SerialLink &l) : link(l) {}
  bool latest(const std::string &, Feedback &sample) override {
    const Telemetry t = link.last_telemetry();
    sample = {t.linear_x, t.angular_z, t.stamp_ns};
    return t.valid;
  }
  SerialLink &link;
};

// At start:
WheeltecZero zero(link);
WheeltecSpeeds speeds(link);
HoldOptions options;
options.robots = {"wheeltec-1"};
options.domain.clock = [] { return ros::Time::now().toNSec(); };   // the clock of the receipt times below
options.domain.boot_held = true;                                  // optional: held after every restart
options.feedback = &speeds;
ChassisHold hold(options, zero);
UdpEndpoint endpoint(hold.service(), {"0.0.0.0", 19530, "/etc/xgc2/chassis-hold.key"});   // throws without a usable key

// cmd_vel callback:
void on_cmd_vel(const ros::MessageEvent<geometry_msgs::Twist const> &event) {
  if (!hold.admit("wheeltec-1", event.getReceiptTime().toNSec()))
    return;   // held, or received before the last release
  link.send_velocity(event.getMessage()->linear.x, event.getMessage()->angular.z);
}

// Control loop, on the thread that writes to the serial port:
while (ros::ok()) {
  link.read_telemetry();
  ros::spinOnce();   // on_cmd_vel
  hold.tick();       // applies engage and release, writes zero while held, passes the speeds on
  rate.sleep();
}
```

The loop runs every cycle, so it needs no `wake`. A driver that requires HOLD (`hold_required`) lets the exception
of `UdpEndpoint` end the process, and systemd restarts it.

### Scout (AgileX) CAN SDK driver (udp.v1)

The same shape over the CAN SDK; the measured velocity is polled from the SDK's state, and the endpoint is
optional when the driver is configured with `hold_required: false`. Tested in `test/host_shapes_udp_test.cpp`.

```cpp
struct ScoutZero : ZeroSink {
  explicit ScoutZero(ScoutBase &b) : base(b) {}
  bool write_zero(const std::string &, std::string &error) override {
    try { base.SetMotionCommand(0.0, 0.0); return true; }   // ugv_sdk: linear m/s, angular rad/s
    catch (const std::exception &e) { error = e.what(); return false; }
  }
  ScoutBase &base;
};
struct ScoutSpeeds : FeedbackSource {
  ScoutSpeeds(ScoutBase &b, std::function<TimeNs()> c) : base(b), clock(std::move(c)) {}
  bool latest(const std::string &, Feedback &sample) override {
    const ScoutState state = base.GetScoutState();   // the latest CAN frames; field names follow the SDK version
    sample = {state.linear_velocity, state.angular_velocity, clock()};
    return true;
  }
  ScoutBase &base;
  std::function<TimeNs()> clock;   // the domain clock
};

std::function<TimeNs()> clock = [] { return ros::Time::now().toNSec(); };
ScoutZero zero(base);
ScoutSpeeds speeds(base, clock);
HoldOptions options;
options.robots = {"scout-1"};
options.domain.clock = clock;
options.feedback = &speeds;
ChassisHold hold(options, zero);

std::unique_ptr<UdpEndpoint> endpoint;
try {
  endpoint = std::make_unique<UdpEndpoint>(hold.service(), udp_options);   // from the node's parameters
} catch (const std::exception &e) {
  if (hold_required) throw;   // exit; systemd restarts the driver
  ROS_ERROR("HOLD is unavailable: %s", e.what());
}

// cmd_vel callback:
void on_cmd_vel(const ros::MessageEvent<geometry_msgs::Twist const> &event) {
  if (hold.admit("scout-1", event.getReceiptTime().toNSec()))
    base.SetMotionCommand(event.getMessage()->linear.x, event.getMessage()->angular.z);
}

// Control loop at 50 Hz: publish the state, then
ros::spinOnce();
hold.tick();
```

### Gazebo world plugin (http.v1)

The plugin is the host of the library inside gzserver. The world host already runs an XRPC `HttpServer`; the plugin
hands it the adapter through the registration points of the world host (`add_request_hook` and
`add_describe_capability` stand for whatever the host offers). Models come and go while the world runs. Tested in
`test/host_shapes_http_test.cpp`.

```cpp
#include <xgc2/chassis_hold/chassis_hold.hpp>
#include <xgc2/chassis_hold/http_adapter.hpp>
using namespace xgc2::chassis_hold;

// Zero output: the chassis plugin of the model keeps the command that its physics step applies.
struct ModelZero : ZeroSink {
  explicit ModelZero(Chassis &c) : chassis(c) {}
  bool write_zero(const std::string &model, std::string &) override { return chassis.set_target(model, 0.0, 0.0); }
  Chassis &chassis;
};
// Feedback: ground-truth velocity of the model, read in the physics thread and stored.
struct ModelVelocity : FeedbackSource {
  explicit ModelVelocity(GroundTruth &t) : truth(t) {}
  bool latest(const std::string &model, Feedback &sample) override { return truth.get(model, sample); }
  GroundTruth &truth;
};

// Load(): the identity of the world host's HttpServer is the identity of the domain.
HoldOptions options;
options.domain.instance_id = host.instance_id();
options.feedback = &velocity;
hold = std::make_unique<ChassisHold>(options, zero);
adapter = std::make_unique<HttpAdapter>(hold->service());
host.add_request_hook([this](const xgc2::xrpc::HttpRequest &request, const xgc2::xrpc::HttpReply &reply) {
  return adapter->handle(request, reply);   // POST /v1/call/xgc2.chassis.hold/<Method>; false: not ours
});
host.add_describe_capability([this] { return adapter->capability_json(); });   // {"name":"xgc2.chassis.hold","entities":[...]}
update = gazebo::event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo &) {
  apply_queued_commands();
  hold->tick();
});

// Models appear and disappear; a model that is created again inherits its HOLD.
void on_model_created(const std::string &model) { hold->add(model); }
void on_model_deleted(const std::string &model) { hold->remove(model); }

// The model's command callback (transport thread): stamp at enqueue, apply in the physics step.
void on_cmd_vel(const std::string &model, const Twist &twist) { queue.push({model, twist, hold->now()}); }
void apply_queued_commands() {   // physics thread, before tick()
  for (const auto &c : queue.take())
    if (hold->admit(c.model, c.enqueued_ns)) chassis.set_target(c.model, c.twist);
}
```

`tick()` here runs in every physics iteration. While the world is paused there are no iterations: an engage still
closes admission at once, but a release waits for the next iteration, and the service reports it `expired` after
`release_wait` (2 s). A plugin that must release a paused world ticks from a thread of its own, with a sink that only
stores the zero command for the physics step.

### XSIM world thread (http.v1)

The XSIM process owns its XRPC server and a world thread that steps the simulation. The thread sleeps between
steps, so it asks the library to wake it; it ticks in every iteration, also while the world is paused, so an engage
and a release both work then. Tested in `test/host_shapes_http_test.cpp`.

```cpp
HoldOptions options;
options.domain.instance_id = server.instance_id();   // the HttpServer identity
options.domain.wake = [this] { { std::lock_guard<std::mutex> lock(mutex); wanted = true; } wake.notify_one(); };
options.feedback = &entity_velocity;
for (const auto &entity : world.robot_entities()) options.robots.push_back(entity.id);
hold = std::make_unique<ChassisHold>(options, entity_zero);
adapter = std::make_unique<HttpAdapter>(hold->service());

// The server's request handler, on the I/O thread:
void http_request(xgc2::xrpc::HttpRequest request, xgc2::xrpc::HttpReply reply) {
  if (adapter->handle(request, reply)) return;   // the hold capability
  /* the management routes of xsim */
}
// Describe facts: capabilities = [..., adapter->capability_json()]

// Input side (I/O or ROS threads): stamp when the command enters the queue.
void on_command(const std::string &entity, const Command &command) { inbox.push({entity, command, hold->now()}); }

// The world thread:
for (;;) {
  {
    std::unique_lock<std::mutex> lock(mutex);
    wake.wait_for(lock, step_period, [&] { return wanted || stopping; });
    wanted = false;
    if (stopping) break;
  }
  if (!paused) {
    for (const auto &c : inbox.take())   // commands wait in the inbox while the world is paused
      if (hold->admit(c.entity, c.enqueued_ns)) world.apply(c.entity, c.command);
    world.step();
  }
  hold->tick();   // every iteration, also while the world is paused
}
```

Spawning and removing entities calls `hold->add(id)` and `hold->remove(id)` from the thread that changes the world.

### Notes for every host

* **One tick thread.** `tick()` always runs on the same thread; a second thread gets `std::logic_error`. Other
  threads (I/O, ROS callbacks) only call `admit`, `engage`, `release`, `state`, `observe` and `set_fault`.
* **Shutdown.** Destroy the transport (`UdpEndpoint`, or stop handing requests to `HttpAdapter`), then the
  `ChassisHold`. The sink and the feedback source must outlive it.
* **Faults.** A host that cannot talk to its device reports it with `hold.domain().set_fault(id, text)`: the robot
  is held, cannot be released while the fault lasts, and shows the text in `State`. Drivers usually exit afterwards.
* **Instance identity.** A world host passes the instance id of its `HttpIdentity` as
  `HoldOptions::domain.instance_id`; `UdpEndpoint` does the reverse and gives its server the id of the domain.

## Using the pieces directly

`ChassisHold` is `Domain` plus `Service` plus the tick and feedback glue; each is usable on its own.

```cpp
Domain domain(options);                       // DomainOptions: boot_held, clock, wake, instance_id, thresholds, max_robots
domain.add("ugv-1", /*feedback=*/true);       // a dynamic roster
bool ok = domain.admit("ugv-1", receipt_ns);  // wait free; false when held, unknown or received before the last release
Pending pending = domain.engage({"ugv-1"});   // closes admission now; the tick applies the machine's part
pending.done(); pending.results();            // outcomes and states; pending.withdraw() takes back unapplied releases
domain.tick(sink);                            // the one tick thread: applies events, writes zero while held
domain.observe("ugv-1", linear, angular, stamp_ns);   // optional trusted velocity feedback
ServiceReply reply = Service(domain).call("Engage", R"({"robot_ids":["ugv-1"]})");   // the JSON surface
```

`Service` runs one small thread of its own that completes pending replies; it never calls back on the tick thread.

`xgc2-hold --endpoint 192.168.1.20:19530 --key-file /etc/xgc2/chassis-hold.key engage --robot ugv-1` is the
command line client (section 3.3 of the contract).

## Build and test

C++17, CMake 3.10 or newer, nlohmann-json 2.1.1 or newer (Ubuntu 18.04 `nlohmann-json-dev`, later
`nlohmann-json3-dev`), the installed `xgc2-state-machine` (`libxgc2-state-machine-dev`) and, per component, the
installed XRPC `udp` and `http` components. Supported: Ubuntu 18.04, 20.04, 22.04 and 24.04 with g++ 7.5 or
newer, amd64 and arm64. `-DXGC2_CHASSIS_HOLD_BUILD_UDP=OFF` and `-DXGC2_CHASSIS_HOLD_BUILD_HTTP=OFF` leave a
component out; Ubuntu 18.04 builds without `http`. The commands avoid `cmake -S/-B`, `--parallel` and
`ctest --test-dir`, which Ubuntu 18.04's CMake 3.10 lacks.

```sh
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DXGC2_CHASSIS_HOLD_WERROR=ON -DCMAKE_PREFIX_PATH="<state machine prefix>;<xrpc prefix>"
cmake --build . -- -j3
ctest --output-on-failure
```

The tests are plain programs (`test/check.hpp`): the state machines and the domain rules (`domain_test`), the JSON
service including its closed schemas and limits (`service_test`), the facade (`facade_test`), threads against one
domain and one service (`concurrency_test`), the udp.v1 endpoint on loopback with a fake tick thread and the XRPC
client (`udp_endpoint_test`), the `xgc2-hold` tool (`hold_cli_test`), the http.v1 adapter on a real XRPC host over
a Unix socket (`http_adapter_test`) and the four hosts above with stand-ins for their devices (`host_shapes_*_test`).
`.xgc2/scripts/run-sanitizers.sh` runs all of them under ThreadSanitizer and under AddressSanitizer plus
UndefinedBehaviorSanitizer (`-DCMAKE_CXX_FLAGS=-fsanitize=...`). The programs in `test/` also build as a separate
project against an installed package (`mkdir b && cd b && cmake ../test`); CI does this after installing the
packages it built.

## Packages

`libxgc2-chassis-hold1` (runtime: the libraries and `/usr/bin/xgc2-hold`) and `libxgc2-chassis-hold-dev` (headers,
CMake package `XgcChassisHold`, the contract) for bionic, focal, jammy and noble, built by
`.xgc2/scripts/build_debs.sh`; the `http` component is part of the focal, jammy and noble packages. The runtime
package depends on `libxgc2-state-machine-dev` (which ships the state machine library) and on the XRPC runtime
packages of the components it contains. The product metadata is `.xgc2/product.yml`.
