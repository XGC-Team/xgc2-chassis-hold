// ChassisHold: a host's roster, clock, zero sink and feedback behind one object.
#include "check.hpp"
#include "fixtures.hpp"

#include <xgc2/chassis_hold/chassis_hold.hpp>

#include <nlohmann/json.hpp>

#include <atomic>
#include <map>
#include <mutex>
#include <stdexcept>

using namespace xgc2::chassis_hold;
using json = nlohmann::json;
using fixtures::FakeClock;
using fixtures::Sink;
using fixtures::TickThread;
using fixtures::ms;
using std::chrono::milliseconds;
using std::chrono::seconds;

namespace {

json body_of(const ServiceReply &reply) { return json::parse(reply.body); }

// The velocities a host measured, by robot.
struct Odometry : FeedbackSource {
  bool latest(const std::string &robot_id, Feedback &sample) override {
    std::lock_guard<std::mutex> lock(mutex);
    if (throws)
      throw std::runtime_error("the odometry is gone");
    const auto found = samples.find(robot_id);
    if (found == samples.end())
      return false;
    sample = found->second;
    ++asked[robot_id];
    return true;
  }
  void set(const std::string &robot_id, double linear, double angular, TimeNs stamp) {
    std::lock_guard<std::mutex> lock(mutex);
    samples[robot_id] = Feedback{linear, angular, stamp};
  }
  int times_asked(const std::string &robot_id) {
    std::lock_guard<std::mutex> lock(mutex);
    return asked[robot_id];
  }
  std::mutex mutex;
  std::map<std::string, Feedback> samples;
  std::map<std::string, int> asked;
  bool throws = false;
};

}  // namespace

TEST(a_host_instantiates_the_capability_from_its_roster_clock_and_sink) {
  FakeClock clock;
  Sink sink;
  HoldOptions options;
  options.domain = clock.options();
  options.robots = {"scout-1", "scout-2"};
  ChassisHold hold(options, sink);

  CHECK_EQ(hold.instance_id(), hold.domain().instance_id());
  CHECK_EQ(hold.service().instance_id(), hold.instance_id());
  CHECK_EQ(hold.now(), clock.ns.load());
  const json described = body_of(hold.service().call("Describe", "{}"));
  REQUIRE(described["robots"].size() == 2);
  CHECK_EQ(described["robots"][0].get<std::string>(), "scout-1");
  CHECK(!described["feedback"]["scout-1"].get<bool>());

  CHECK(hold.admit("scout-1", hold.now()));
  hold.domain().engage({"scout-1"});
  CHECK(!hold.admit("scout-1", hold.now()));  // closed at once
  CHECK(hold.admit("scout-2", hold.now()));
  const TickStats stats = hold.tick();
  CHECK_EQ(stats.written, 1);
  CHECK_EQ(sink.writes("scout-1"), 1);
  CHECK_EQ(sink.writes("scout-2"), 0);
  CHECK_EQ(hold.domain().state({"scout-1"}).robots[0].stage, Stage::zero_written);
}

TEST(the_options_reach_the_domain_and_the_service) {
  FakeClock clock;
  Sink sink;
  std::atomic<int> woken{0};
  HoldOptions options;
  options.domain = clock.options();
  options.domain.boot_held = true;
  options.domain.instance_id = "0123456789abcdef0123456789abcdef";
  options.domain.wake = [&woken] { ++woken; };
  options.service.max_ids = 1;
  options.robots = {"a"};
  ChassisHold hold(options, sink);

  CHECK_EQ(hold.instance_id(), "0123456789abcdef0123456789abcdef");
  CHECK(hold.domain().state({"a"}).robots[0].held);  // boot_held
  CHECK(!hold.admit("a", hold.now()));
  CHECK_EQ(woken.load(), 1);  // the boot hold needs the tick
  CHECK(hold.add("b"));
  CHECK(hold.domain().state({"b"}).robots[0].held);
  CHECK_EQ(hold.service().call("State", "{}").status, Status::invalid_argument);  // max_ids = 1 < roster of 2
  CHECK(hold.remove("b"));
  CHECK(!hold.remove("b"));
  CHECK_EQ(hold.service().call("State", "{}").status, Status::ok);
  hold.tick();
  CHECK_EQ(sink.writes("a"), 1);
}

TEST(a_bad_roster_is_refused_with_the_id_in_the_message) {
  FakeClock clock;
  Sink sink;
  const auto message_of = [&](std::vector<std::string> robots) {
    HoldOptions options;
    options.domain = clock.options();
    options.robots = std::move(robots);
    try {
      ChassisHold hold(options, sink);
    } catch (const std::exception &e) {
      return std::string(e.what());
    }
    return std::string("no exception");
  };
  CHECK(message_of({"a", "bad id"}).find("bad id") != std::string::npos);
  CHECK(message_of({"a", "b", "a"}).find("\"a\"") == std::string::npos);  // the repeated id is named, plainly
  CHECK(message_of({"a", "b", "a"}).find("a is in the roster twice") != std::string::npos);
  CHECK_EQ(message_of({}), "no exception");  // an empty roster is fine: robots may be added later

  HoldOptions small;
  small.domain = clock.options();
  small.domain.max_robots = 2;
  small.robots = {"a", "b", "c"};
  bool refused = false;
  try {
    ChassisHold hold(small, sink);
  } catch (const std::length_error &) {
    refused = true;
  }
  CHECK(refused);
}

TEST(feedback_reaches_the_domain_on_every_tick) {
  FakeClock clock;
  Sink sink;
  Odometry odometry;
  HoldOptions options;
  options.domain = clock.options();  // stopped after 300 ms at rest
  options.robots = {"a", "b"};
  options.feedback = &odometry;
  ChassisHold hold(options, sink);

  const json described = body_of(hold.service().call("Describe", "{}"));
  CHECK(described["feedback"]["a"].get<bool>());
  CHECK(described["feedback"]["b"].get<bool>());

  hold.tick();
  CHECK_EQ(odometry.times_asked("a"), 0);  // nothing is held: nobody asks
  hold.domain().engage({"a"});
  odometry.set("a", 0.0, 0.0, clock.ns);
  hold.tick();  // zero written, then the sample taken at the same instant is evidence
  CHECK_EQ(odometry.times_asked("a"), 1);
  CHECK_EQ(odometry.times_asked("b"), 0);
  CHECK_EQ(hold.domain().state({"a"}).robots[0].stage, Stage::zero_written);

  clock.advance(100);
  odometry.set("a", 0.0, 0.0, clock.ns);
  hold.tick();
  CHECK_EQ(hold.domain().state({"a"}).robots[0].stage, Stage::zero_written);  // 100 ms of rest are not enough

  clock.advance(250);
  odometry.set("a", 0.0, 0.0, clock.ns);
  hold.tick();  // the sample completes the dwell and posts the report ...
  hold.tick();  // ... which this tick applies
  const RobotState a = hold.domain().state({"a"}).robots[0];
  CHECK_EQ(a.stage, Stage::stopped);
  CHECK(a.feedback);

  clock.advance(10);
  odometry.set("a", 0.4, 0.0, clock.ns);
  hold.tick();
  hold.tick();
  CHECK_EQ(hold.domain().state({"a"}).robots[0].stage, Stage::zero_written);
}

TEST(a_host_without_a_feedback_source_never_reports_stopped) {
  FakeClock clock;
  Sink sink;
  HoldOptions options;
  options.domain = clock.options();
  options.robots = {"a"};
  ChassisHold hold(options, sink);
  hold.domain().engage({"a"});
  hold.tick();
  clock.advance(1000);
  CHECK(!hold.domain().observe("a", 0.0, 0.0, clock.ns));  // not a robot with feedback
  hold.tick();
  CHECK_EQ(hold.domain().state({"a"}).robots[0].stage, Stage::zero_written);
}

TEST(robots_added_later_follow_the_feedback_setting_and_a_failing_source_is_a_missing_sample) {
  FakeClock clock;
  Sink sink;
  Odometry odometry;
  HoldOptions options;
  options.domain = clock.options();
  options.feedback = &odometry;
  ChassisHold hold(options, sink);
  CHECK(hold.add("late"));
  CHECK(!hold.add("late"));
  CHECK(hold.domain().state({"late"}).robots[0].feedback);

  hold.domain().engage({"late"});
  odometry.throws = true;
  hold.tick();  // writes zero and survives the source
  CHECK_EQ(sink.writes("late"), 1);
  CHECK_EQ(hold.domain().state({"late"}).robots[0].stage, Stage::zero_written);
  odometry.throws = false;
  hold.tick();
  CHECK_EQ(sink.writes("late"), 2);
}

TEST(requests_reach_a_ticking_host_through_the_service) {
  Sink sink;
  HoldOptions options;
  options.robots = {"a"};
  options.service.reply_wait = seconds(5);
  options.service.release_wait = seconds(5);
  ChassisHold hold(options, sink);
  TickThread ticker([&hold] { hold.tick(); });

  const json engaged = body_of(hold.service().call("Engage", "{\"robot_ids\":[\"a\"]}"));
  CHECK_EQ(engaged["robots"][0]["outcome"].get<std::string>(), "engaged");
  CHECK_EQ(engaged["robots"][0]["stage"].get<std::string>(), "zero_written");
  CHECK(!hold.admit("a", hold.now()));
  const std::string release = "{\"expected_instance\":\"" + hold.instance_id() +
                              "\",\"changes\":[{\"robot_id\":\"a\",\"expected_revision\":1}]}";
  const json released = body_of(hold.service().call("Release", release));
  CHECK_EQ(released["robots"][0]["outcome"].get<std::string>(), "released");
  CHECK(hold.admit("a", hold.now()));
}

TEST_MAIN()
