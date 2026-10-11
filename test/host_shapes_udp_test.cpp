// The two driver shapes of the README, "Instantiating chassis-hold", with stand-ins for their devices: a Wheeltec-style
// ROS node over a serial link and a Scout-style node over a CAN SDK. Each owns a ChassisHold, exposes it with a
// UdpEndpoint and is called over udp.v1 like a robot on the LAN.
#include "check.hpp"
#include "fixtures.hpp"
#include "udp_fixtures.hpp"

#include <xgc2/chassis_hold/chassis_hold.hpp>
#include <xgc2/chassis_hold/udp_endpoint.hpp>

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace udp_fixtures;
using std::chrono::microseconds;

// The conditions below ask the robot over udp.v1, which limits the requests of one source: poll gently.
template <typename Condition>
bool eventually(Condition condition) {
  return fixtures::wait_until(condition, seconds(10), microseconds(40000));
}
// A robot at rest passes from zero_written to stopped once the dwell is over.
bool zeroed(Stage stage) { return stage == Stage::zero_written || stage == Stage::stopped; }
using std::chrono::system_clock;

namespace {

// ---- stand-ins for ROS1 -----------------------------------------------------------------------------------------

namespace ros {
struct Time {
  TimeNs ns = 0;
  TimeNs toNSec() const { return ns; }
  static Time now() { return Time{std::chrono::duration_cast<std::chrono::nanoseconds>(system_clock::now().time_since_epoch()).count()}; }
};
}  // namespace ros

namespace geometry_msgs {
struct Vector3 {
  double x = 0, y = 0, z = 0;
};
struct Twist {
  Vector3 linear, angular;
};
}  // namespace geometry_msgs

namespace ros {
template <typename M>
class MessageEvent {
public:
  MessageEvent(M message, Time receipt) : message_(std::make_shared<M>(std::move(message))), receipt_(receipt) {}
  Time getReceiptTime() const { return receipt_; }
  std::shared_ptr<M> getMessage() const { return message_; }

private:
  std::shared_ptr<M> message_;
  Time receipt_;
};

// The callback queue of a node: a message waits here, with its receipt time, until spinOnce() runs the callback.
class Subscriber {
public:
  using Callback = std::function<void(const MessageEvent<geometry_msgs::Twist const> &)>;
  explicit Subscriber(Callback callback) : callback_(std::move(callback)) {}
  // delay: the message sits in the queue this long before the node gets to it
  void publish(geometry_msgs::Twist twist, std::chrono::milliseconds delay = std::chrono::milliseconds(0)) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back({MessageEvent<geometry_msgs::Twist const>(twist, Time::now()), std::chrono::steady_clock::now() + delay});
  }
  void spinOnce() {
    std::vector<MessageEvent<geometry_msgs::Twist const>> due;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      while (!queue_.empty() && queue_.front().second <= std::chrono::steady_clock::now()) {
        due.push_back(queue_.front().first);
        queue_.pop_front();
      }
    }
    for (const auto &event : due)
      callback_(event);
  }

private:
  Callback callback_;
  std::mutex mutex_;
  std::deque<std::pair<MessageEvent<geometry_msgs::Twist const>, std::chrono::steady_clock::time_point>> queue_;
};
}  // namespace ros

struct Frame {
  double linear = 0, angular = 0;
};

// The driver thread of a node: calls the cycle until it is stopped.
class ControlLoop {
public:
  explicit ControlLoop(std::function<void()> cycle) : thread_([this, cycle] {
    while (running_) {
      cycle();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }) {}
  ~ControlLoop() { stop(); }
  void stop() {
    running_ = false;
    if (thread_.joinable())
      thread_.join();
  }

private:
  std::atomic<bool> running_{true};
  std::thread thread_;
};

// The udp.v1 client of the person at the robot.
struct Caller {
  explicit Caller(const UdpEndpoint &endpoint) : target("127.0.0.1:" + std::to_string(endpoint.port())), client(Rig::make_keys()) {}
  udp::Response call(const std::string &method, const json &body) const {
    return client.call(target, key_id, std::string(service_name) + "/" + method, body.dump(),
                       std::chrono::steady_clock::now() + seconds(3));
  }
  json ok(const std::string &method, const json &body = json::object()) const {
    const udp::Response response = call(method, body);
    if (!answered(response)) {
      test::fail(__FILE__, __LINE__, method + " was not answered");
      return json::object();
    }
    return parse(response.body);
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
    if (!s["fault"].is_null())
      state.fault = s["fault"].get<std::string>();
    return state;
  }
  std::string target;
  udp::Client client;
};

// ---- Wheeltec serial driver -------------------------------------------------------------------------------------

struct Telemetry {
  double linear_x = 0, angular_z = 0;
  TimeNs stamp_ns = 0;
  bool valid = false;
};

// The serial port of the chassis and the STM32 behind it, which follows every command at once.
class SerialLink {
public:
  bool send_velocity(double linear, double angular, std::string &error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (broken_) {
      error = "port closed";
      return false;
    }
    frames_.push_back({linear, angular});
    return true;
  }
  void read_telemetry() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!frames_.empty())
      telemetry_ = {frames_.back().linear, frames_.back().angular, ros::Time::now().toNSec(), true};
  }
  Telemetry last_telemetry() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return telemetry_;
  }
  std::vector<Frame> frames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_;
  }
  void set_broken(bool broken) {
    std::lock_guard<std::mutex> lock(mutex_);
    broken_ = broken;
  }

private:
  mutable std::mutex mutex_;
  std::vector<Frame> frames_;
  Telemetry telemetry_;
  bool broken_ = false;
};

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

class WheeltecDriver {
public:
  WheeltecDriver(const std::string &key_file, bool boot_held)
      : zero(link), speeds(link), hold(options(boot_held), zero),
        endpoint(hold.service(), UdpEndpointOptions{"127.0.0.1", 0, key_file}),
        cmd_vel([this](const ros::MessageEvent<geometry_msgs::Twist const> &event) { on_cmd_vel(event); }),
        loop([this] { cycle(); }) {}

  HoldOptions options(bool boot_held) {
    HoldOptions o;
    o.robots = {"wheeltec-1"};
    o.domain.clock = [] { return ros::Time::now().toNSec(); };  // the clock of the receipt times below
    o.domain.boot_held = boot_held;
    o.feedback = &speeds;
    return o;
  }

  // cmd_vel callback:
  void on_cmd_vel(const ros::MessageEvent<geometry_msgs::Twist const> &event) {
    if (!hold.admit("wheeltec-1", event.getReceiptTime().toNSec()))
      return;  // held, or received before the last release
    std::string ignored;
    link.send_velocity(event.getMessage()->linear.x, event.getMessage()->angular.z, ignored);
  }

  // One cycle of the control loop, on the thread that writes to the serial port:
  void cycle() {
    link.read_telemetry();
    cmd_vel.spinOnce();  // on_cmd_vel
    hold.tick();         // applies engage and release, writes zero while held, passes the speeds on
  }

  bool drove(double linear) const {
    for (const auto &frame : link.frames())
      if (frame.linear == linear)
        return true;
    return false;
  }

  SerialLink link;
  WheeltecZero zero;
  WheeltecSpeeds speeds;
  ChassisHold hold;
  UdpEndpoint endpoint;
  ros::Subscriber cmd_vel;
  ControlLoop loop;  // last: it stops first
};

geometry_msgs::Twist drive(double linear) {
  geometry_msgs::Twist twist;
  twist.linear.x = linear;
  return twist;
}

// ---- Scout CAN SDK driver ---------------------------------------------------------------------------------------

struct ScoutState {
  double linear_velocity = 0, angular_velocity = 0;
};

// The CAN SDK: commands go to the base, the state is what its CAN frames last said.
class ScoutBase {
public:
  void SetMotionCommand(double linear, double angular) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (can_down_)
      throw std::runtime_error("CAN bus off");
    commands_.push_back({linear, angular});
    state_ = {linear, angular};
  }
  ScoutState GetScoutState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }
  std::vector<Frame> commands() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return commands_;
  }
  void set_can_down(bool down) {
    std::lock_guard<std::mutex> lock(mutex_);
    can_down_ = down;
  }

private:
  mutable std::mutex mutex_;
  std::vector<Frame> commands_;
  ScoutState state_;
  bool can_down_ = false;
};

struct ScoutZero : ZeroSink {
  explicit ScoutZero(ScoutBase &b) : base(b) {}
  bool write_zero(const std::string &, std::string &error) override {
    try {
      base.SetMotionCommand(0.0, 0.0);  // ugv_sdk: linear m/s, angular rad/s
      return true;
    } catch (const std::exception &e) {
      error = e.what();
      return false;
    }
  }
  ScoutBase &base;
};

struct ScoutSpeeds : FeedbackSource {
  ScoutSpeeds(ScoutBase &b, std::function<TimeNs()> c) : base(b), clock(std::move(c)) {}
  bool latest(const std::string &, Feedback &sample) override {
    const ScoutState state = base.GetScoutState();  // the latest CAN frames
    sample = {state.linear_velocity, state.angular_velocity, clock()};
    return true;
  }
  ScoutBase &base;
  std::function<TimeNs()> clock;  // the domain clock
};

class ScoutDriver {
public:
  ScoutDriver(const UdpEndpointOptions &udp_options, bool hold_required)
      : zero(base), speeds(base, clock), hold(options(), zero),
        cmd_vel([this](const ros::MessageEvent<geometry_msgs::Twist const> &event) { on_cmd_vel(event); }) {
    try {
      endpoint = std::make_unique<UdpEndpoint>(hold.service(), udp_options);
    } catch (const std::exception &e) {
      if (hold_required)
        throw;  // exit; systemd restarts the driver
      udp_error = e.what();
    }
    loop = std::make_unique<ControlLoop>([this] { cycle(); });
  }
  ~ScoutDriver() { loop.reset(); }

  HoldOptions options() {
    HoldOptions o;
    o.robots = {"scout-1"};
    o.domain.clock = clock;
    o.feedback = &speeds;
    return o;
  }

  void on_cmd_vel(const ros::MessageEvent<geometry_msgs::Twist const> &event) {
    if (hold.admit("scout-1", event.getReceiptTime().toNSec())) {
      try {
        base.SetMotionCommand(event.getMessage()->linear.x, event.getMessage()->angular.z);
      } catch (const std::exception &) {
      }
    }
  }
  void cycle() {
    cmd_vel.spinOnce();
    hold.tick();
  }

  std::function<TimeNs()> clock = [] { return ros::Time::now().toNSec(); };
  ScoutBase base;
  ScoutZero zero;
  ScoutSpeeds speeds;
  ChassisHold hold;
  ros::Subscriber cmd_vel;
  std::unique_ptr<UdpEndpoint> endpoint;
  std::string udp_error;
  std::unique_ptr<ControlLoop> loop;
};

}  // namespace

TEST(a_wheeltec_style_driver_is_gated_zeroed_and_released_over_udp) {
  TempDir dir;
  WheeltecDriver driver(dir.write("hold.key", key_line(key_id, key_bytes)), /*boot_held=*/false);
  const Caller caller(driver.endpoint);

  const json described = caller.ok("Describe");
  CHECK_EQ(described["robots"][0].get<std::string>(), "wheeltec-1");
  CHECK(described["feedback"]["wheeltec-1"].get<bool>());  // the FeedbackSource makes it a robot with feedback
  CHECK_EQ(described["instance"].get<std::string>(), driver.hold.instance_id());

  // Released: the command reaches the serial port.
  driver.cmd_vel.publish(drive(0.3));
  CHECK(fixtures::wait_until([&] { return driver.drove(0.3); }));

  // Engage over the LAN: admission is closed, the loop writes zero, and the reply says so.
  const json engaged = caller.ok("Engage", json{{"robot_ids", json::array({"wheeltec-1"})}});
  CHECK_EQ(engaged["robots"][0]["outcome"].get<std::string>(), "engaged");
  CHECK_EQ(engaged["robots"][0]["stage"].get<std::string>(), "zero_written");
  const std::size_t frames = driver.link.frames().size();
  driver.cmd_vel.publish(drive(0.4));
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  CHECK(!driver.drove(0.4));  // a command of a held robot is dropped
  CHECK(driver.link.frames().size() > frames);  // the zero is written every cycle

  // The speeds come back as zero: with the dwell passed, the robot is stopped.
  CHECK(eventually([&] { return caller.state("wheeltec-1").stage == Stage::stopped; }));

  // A command that was received before the release and is dispatched after it is not executed; one received
  // after the release is.
  driver.cmd_vel.publish(drive(0.5), std::chrono::milliseconds(150));  // waits in the callback queue
  const RobotState held = caller.state("wheeltec-1");
  const json released = caller.ok(
      "Release", json{{"expected_instance", driver.hold.instance_id()},
                      {"changes", json::array({json{{"robot_id", "wheeltec-1"}, {"expected_revision", held.revision}}})}});
  CHECK_EQ(released["robots"][0]["outcome"].get<std::string>(), "released");
  CHECK(driver.hold.admit("wheeltec-1", driver.hold.now()));
  driver.cmd_vel.publish(drive(0.6));
  CHECK(fixtures::wait_until([&] { return driver.drove(0.6); }));
  std::this_thread::sleep_for(std::chrono::milliseconds(250));  // the old command has been dispatched by now
  CHECK(!driver.drove(0.5));
  CHECK(!driver.drove(0.4));
  CHECK(driver.drove(0.3));
}

TEST(a_wheeltec_style_driver_that_starts_held_stays_held_until_released_and_reports_a_dead_port) {
  TempDir dir;
  WheeltecDriver driver(dir.write("hold.key", key_line(key_id, key_bytes)), /*boot_held=*/true);
  const Caller caller(driver.endpoint);

  // Held from the first instant, before any tick: the command is dropped, then the tick writes zero.
  driver.cmd_vel.publish(drive(0.3));
  CHECK(eventually([&] { return zeroed(caller.state("wheeltec-1").stage); }));
  CHECK(caller.state("wheeltec-1").held);
  CHECK_EQ(caller.state("wheeltec-1").revision, 1);
  CHECK(!driver.drove(0.3));

  // The port dies: the zero cannot be written, and the robot says so instead of claiming it is zeroed.
  driver.link.set_broken(true);
  CHECK(eventually([&] { return caller.state("wheeltec-1").stage == Stage::gated; }));
  const json state = caller.ok("State", json{{"robot_ids", json::array({"wheeltec-1"})}});
  CHECK_EQ(state["robots"][0]["fault"].get<std::string>(), "zero write failed: port closed");
  driver.link.set_broken(false);
  CHECK(eventually([&] { return zeroed(caller.state("wheeltec-1").stage); }));
}

TEST(a_scout_style_driver_polls_the_can_state_and_reports_a_dead_bus) {
  TempDir dir;
  ScoutDriver driver(UdpEndpointOptions{"127.0.0.1", 0, dir.write("hold.key", key_line(key_id, key_bytes))},
                     /*hold_required=*/true);
  REQUIRE(driver.endpoint != nullptr);
  const Caller caller(*driver.endpoint);

  driver.cmd_vel.publish(drive(0.8));
  CHECK(fixtures::wait_until([&] {
    for (const auto &c : driver.base.commands())
      if (c.linear == 0.8)
        return true;
    return false;
  }));

  const json engaged = caller.ok("Engage", json{{"all", true}});
  CHECK_EQ(engaged["robots"][0]["outcome"].get<std::string>(), "engaged");
  CHECK_EQ(engaged["robots"][0]["stage"].get<std::string>(), "zero_written");
  CHECK(eventually([&] { return caller.state("scout-1").stage == Stage::stopped; }));  // polled CAN state: at rest

  driver.base.set_can_down(true);
  CHECK(eventually([&] { return caller.state("scout-1").stage == Stage::gated; }));
  CHECK_EQ(caller.state("scout-1").fault, "zero write failed: CAN bus off");
  driver.base.set_can_down(false);
  CHECK(eventually([&] { return zeroed(caller.state("scout-1").stage); }));
  CHECK(caller.state("scout-1").fault.empty());

  const json released = caller.ok(
      "Release", json{{"expected_instance", driver.hold.instance_id()},
                      {"changes", json::array({json{{"robot_id", "scout-1"}, {"expected_revision", caller.state("scout-1").revision}}})}});
  CHECK_EQ(released["robots"][0]["outcome"].get<std::string>(), "released");
}

TEST(a_scout_style_driver_without_a_usable_key_exits_or_runs_without_the_endpoint) {
  TempDir dir;
  const UdpEndpointOptions missing{"127.0.0.1", 0, dir.path() + "/no-such.key"};
  bool refused = false;
  try {
    ScoutDriver required(missing, /*hold_required=*/true);
  } catch (const std::runtime_error &e) {
    refused = std::string(e.what()).find("no-such.key") != std::string::npos;
  }
  CHECK(refused);  // hold_required: the exception ends the process and systemd restarts the driver

  ScoutDriver optional(missing, /*hold_required=*/false);
  CHECK(optional.endpoint == nullptr);
  CHECK(optional.udp_error.find("no-such.key") != std::string::npos);
  // HOLD still gates locally: the host's own safety logic can use it.
  optional.hold.domain().engage({"scout-1"});
  CHECK(!optional.hold.admit("scout-1", optional.hold.now()));
  CHECK(fixtures::wait_until([&] { return optional.hold.domain().state({"scout-1"}).robots[0].stage != Stage::gated; }));
}

TEST_MAIN()
