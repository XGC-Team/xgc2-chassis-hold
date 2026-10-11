#include "check.hpp"
#include "fixtures.hpp"

#include <xgc2/chassis_hold/service.hpp>

#include <nlohmann/json.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

using namespace xgc2::chassis_hold;
using json = nlohmann::json;

namespace {

using fixtures::FakeClock;
using fixtures::Sink;
using fixtures::TickThread;
using fixtures::wait_until;
using Clock = Service::Clock;
using std::chrono::milliseconds;
using std::chrono::seconds;

ServiceOptions quick_wait() {
  ServiceOptions options;
  options.reply_wait = milliseconds(30);
  options.release_wait = milliseconds(200);
  return options;
}

json body_of(const ServiceReply &reply) {
  json parsed;
  try {
    parsed = json::parse(reply.body);
  } catch (const std::exception &e) {
    test::fail(__FILE__, __LINE__, std::string("reply is not JSON: ") + e.what() + ": " + reply.body);
  }
  return parsed;
}

std::string release_body(const std::string &instance, const std::string &changes) {
  return "{\"expected_instance\":\"" + instance + "\",\"changes\":" + changes + "}";
}

bool only_keys(const json &object, const std::vector<std::string> &keys) {
  if (!object.is_object() || object.size() != keys.size())
    return false;
  for (const auto &key : keys)
    if (object.find(key) == object.end())
      return false;
  return true;
}

// Replies are text for remote callers: printable ASCII only.
bool printable_ascii(const std::string &text) {
  for (const unsigned char c : text)
    if (c < 0x20 || c >= 0x7f)
      return false;
  return true;
}

}  // namespace

// Informational: the log of a build says which JSON library it ran against (2.1.1 on Bionic).
TEST(json_library_in_use) {
#ifdef NLOHMANN_JSON_VERSION_MAJOR
  std::printf("nlohmann-json %d.%d.%d\n", NLOHMANN_JSON_VERSION_MAJOR, NLOHMANN_JSON_VERSION_MINOR,
              NLOHMANN_JSON_VERSION_PATCH);
#else
  std::printf("nlohmann-json 2.x (no version macro)\n");
#endif
}

TEST(describe_lists_roster_feedback_and_capabilities) {
  FakeClock clock;
  Domain domain(clock.options());
  domain.add("b");
  domain.add("a", true);
  Service service(domain);

  const ServiceReply reply = service.call("Describe", "{}");
  CHECK_EQ(reply.status, Status::ok);
  json j = body_of(reply);
  CHECK(only_keys(j, {"instance", "robots", "feedback", "capabilities"}));
  CHECK_EQ(j["instance"].get<std::string>(), domain.instance_id());
  CHECK_EQ(service.instance_id(), domain.instance_id());
  REQUIRE(j["robots"].size() == 2);
  CHECK_EQ(j["robots"][0].get<std::string>(), "a");
  CHECK_EQ(j["robots"][1].get<std::string>(), "b");
  CHECK(j["feedback"]["a"].get<bool>());
  CHECK(!j["feedback"]["b"].get<bool>());
  REQUIRE(j["capabilities"].size() == 3);
  CHECK_EQ(j["capabilities"][0].get<std::string>(), "engage");
  CHECK_EQ(j["capabilities"][1].get<std::string>(), "release");
  CHECK_EQ(j["capabilities"][2].get<std::string>(), "state");

  CHECK_EQ(service.call("Describe", "").status, Status::ok);  // an empty body is {}
  CHECK_EQ(service.call("Describe", " \n").status, Status::ok);
  CHECK_EQ(service.call("Describe", "{\"x\":1}").status, Status::invalid_argument);
}

TEST(state_reports_every_field) {
  FakeClock clock;
  Domain domain(clock.options());
  domain.add("a");
  domain.add("b");
  Service service(domain);
  Sink sink;
  clock.advance(100);
  domain.engage({"a"});
  clock.advance(20);
  domain.tick(sink);

  json j = body_of(service.call("State", ""));
  CHECK_EQ(j["instance"].get<std::string>(), domain.instance_id());
  REQUIRE(j["robots"].size() == 2);
  json a = j["robots"][0];
  CHECK(only_keys(a, {"robot_id", "held", "revision", "stage", "gated_at_ms", "zero_written_at_ms",
                      "stopped_at_ms", "fault"}));
  CHECK_EQ(a["robot_id"].get<std::string>(), "a");
  CHECK(a["held"].get<bool>());
  CHECK_EQ(a["revision"].get<std::uint64_t>(), 1);
  CHECK_EQ(a["stage"].get<std::string>(), "zero_written");
  CHECK_EQ(a["gated_at_ms"].get<std::int64_t>(), 100);  // since the domain was created
  CHECK_EQ(a["zero_written_at_ms"].get<std::int64_t>(), 120);
  CHECK(a["stopped_at_ms"].is_null());
  CHECK(a["fault"].is_null());
  json b = j["robots"][1];
  CHECK(!b["held"].get<bool>());
  CHECK_EQ(b["revision"].get<std::uint64_t>(), 0);
  CHECK_EQ(b["stage"].get<std::string>(), "released");
  CHECK(b["gated_at_ms"].is_null());
  CHECK(b["zero_written_at_ms"].is_null());

  j = body_of(service.call("State", "{\"robot_ids\":[\"b\"]}"));
  REQUIRE(j["robots"].size() == 1);
  CHECK_EQ(j["robots"][0]["robot_id"].get<std::string>(), "b");
  CHECK_EQ(body_of(service.call("State", "{\"robot_ids\":[]}"))["robots"].size(), 2);

  // A zero write that failed is reported as a fault; a host fault takes precedence.
  sink.fail("a", true);
  clock.advance(10);
  domain.tick(sink);
  a = body_of(service.call("State", "{\"robot_ids\":[\"a\"]}"))["robots"][0];
  CHECK_EQ(a["stage"].get<std::string>(), "gated");
  CHECK_EQ(a["fault"].get<std::string>(), "zero write failed: device unplugged");
  CHECK(a["zero_written_at_ms"].is_null());
  domain.set_fault("a", "serial port lost");
  domain.tick(sink);
  a = body_of(service.call("State", "{\"robot_ids\":[\"a\"]}"))["robots"][0];
  CHECK_EQ(a["fault"].get<std::string>(), "serial port lost");

  // Unknown robots are not found; the reply names them.
  const ServiceReply unknown = service.call("State", "{\"robot_ids\":[\"a\",\"zz\",\"yy\"]}");
  CHECK_EQ(unknown.status, Status::not_found);
  j = body_of(unknown);
  CHECK_EQ(j["code"].get<std::string>(), "not_found");
  REQUIRE(j["details"]["robot_ids"].size() == 2);
  CHECK_EQ(j["details"]["robot_ids"][0].get<std::string>(), "zz");
}

TEST(engage_reports_the_outcome_of_each_robot) {
  Domain domain;
  domain.add("a");
  domain.add("b");
  Service service(domain, quick_wait());

  ServiceReply reply = service.call("Engage", "{\"robot_ids\":[\"a\",\"zz\"]}");
  CHECK_EQ(reply.status, Status::ok);
  json j = body_of(reply);
  CHECK_EQ(j["instance"].get<std::string>(), domain.instance_id());
  REQUIRE(j["robots"].size() == 2);
  json a = j["robots"][0];
  CHECK(only_keys(a, {"robot_id", "outcome", "held", "revision", "stage"}));
  CHECK_EQ(a["outcome"].get<std::string>(), "engaged");
  CHECK(a["held"].get<bool>());
  CHECK_EQ(a["revision"].get<std::uint64_t>(), 1);
  CHECK_EQ(a["stage"].get<std::string>(), "gated");  // nobody ticks: the reply reports the gate
  json zz = j["robots"][1];
  CHECK(only_keys(zz, {"robot_id", "outcome"}));
  CHECK_EQ(zz["outcome"].get<std::string>(), "unknown_robot");

  j = body_of(service.call("Engage", "{\"robot_ids\":[\"a\"]}"));
  CHECK_EQ(j["robots"][0]["outcome"].get<std::string>(), "already_held");
  CHECK_EQ(j["robots"][0]["revision"].get<std::uint64_t>(), 1);

  j = body_of(service.call("Engage", "{\"all\":true}"));
  REQUIRE(j["robots"].size() == 2);
  CHECK_EQ(j["robots"][0]["outcome"].get<std::string>(), "already_held");
  CHECK_EQ(j["robots"][1]["outcome"].get<std::string>(), "engaged");
}

TEST(engage_reply_waits_for_the_tick_and_comes_from_the_service_thread) {
  Domain domain;
  domain.add("a");
  ServiceOptions options;
  options.reply_wait = seconds(10);
  Service service(domain, options);

  std::promise<ServiceReply> promise;
  std::future<ServiceReply> future = promise.get_future();
  std::thread::id callback_thread;
  service.call_async("Engage", "{\"robot_ids\":[\"a\"]}", Clock::now() + seconds(20), [&](ServiceReply reply) {
    callback_thread = std::this_thread::get_id();
    promise.set_value(std::move(reply));
  });
  // Gated, but nobody has written zero: no reply yet.
  CHECK(future.wait_for(milliseconds(80)) == std::future_status::timeout);

  Sink sink;
  const auto ticked = Clock::now();
  domain.tick(sink);  // this thread is the tick thread
  REQUIRE(future.wait_for(seconds(5)) == std::future_status::ready);
  CHECK(Clock::now() - ticked < seconds(2));
  CHECK(callback_thread != std::this_thread::get_id());
  const json j = body_of(future.get());
  CHECK_EQ(j["robots"][0]["outcome"].get<std::string>(), "engaged");
  CHECK_EQ(j["robots"][0]["stage"].get<std::string>(), "zero_written");
}

TEST(engage_wait_is_bounded_by_reply_wait_and_by_the_deadline) {
  Domain domain;
  domain.add("a");
  domain.add("b");
  domain.add("c");

  {  // reply_wait
    ServiceOptions options;
    options.reply_wait = milliseconds(60);
    Service service(domain, options);
    const auto started = Clock::now();
    const ServiceReply reply = service.call("Engage", "{\"robot_ids\":[\"a\"]}", started + seconds(30));
    const auto elapsed = Clock::now() - started;
    CHECK(elapsed >= milliseconds(55));
    CHECK(elapsed < seconds(3));
    CHECK_EQ(body_of(reply)["robots"][0]["stage"].get<std::string>(), "gated");
  }
  {  // the call deadline is shorter than reply_wait
    ServiceOptions options;
    options.reply_wait = seconds(30);
    Service service(domain, options);
    const auto started = Clock::now();
    const ServiceReply reply = service.call("Engage", "{\"robot_ids\":[\"b\"]}", started + milliseconds(50));
    const auto elapsed = Clock::now() - started;
    CHECK(elapsed < seconds(3));
    CHECK_EQ(reply.status, Status::ok);
    CHECK_EQ(body_of(reply)["robots"][0]["stage"].get<std::string>(), "gated");
  }
  {  // no wait at all: the callback runs before call_async returns, also for a deadline that has passed
    ServiceOptions options;
    options.reply_wait = milliseconds(0);
    Service service(domain, options);
    bool called = false;
    service.call_async("Engage", "{\"robot_ids\":[\"c\"]}", Clock::time_point::max(), [&](ServiceReply reply) {
      called = true;
      CHECK_EQ(reply.status, Status::ok);
    });
    CHECK(called);
    Service waiting(domain, quick_wait());
    called = false;
    waiting.call_async("Engage", "{\"all\":true}", Clock::now() - seconds(1), [&](ServiceReply) { called = true; });
    CHECK(called);
  }
}

TEST(engage_replies_at_once_when_zero_is_already_written_and_for_errors) {
  Domain domain;
  domain.add("a");
  Service service(domain, quick_wait());
  Sink sink;
  domain.engage({"a"});
  domain.tick(sink);

  bool called = false;
  service.call_async("Engage", "{\"robot_ids\":[\"a\"]}", Clock::now() + seconds(5), [&](ServiceReply reply) {
    called = true;
    CHECK_EQ(body_of(reply)["robots"][0]["outcome"].get<std::string>(), "already_held");
    CHECK_EQ(body_of(reply)["robots"][0]["stage"].get<std::string>(), "zero_written");
  });
  CHECK(called);

  called = false;
  service.call_async("Engage", "{}", Clock::now() + seconds(5), [&](ServiceReply reply) {
    called = true;
    CHECK_EQ(reply.status, Status::invalid_argument);
  });
  CHECK(called);
}

TEST(a_pending_engage_ends_when_the_robot_is_released_or_removed) {
  Domain domain;
  domain.add("a");
  domain.add("b");
  ServiceOptions options;
  options.reply_wait = seconds(30);
  Service service(domain, options);
  Sink sink;

  std::promise<ServiceReply> released_promise;
  std::promise<ServiceReply> removed_promise;
  std::future<ServiceReply> released = released_promise.get_future();
  std::future<ServiceReply> removed = removed_promise.get_future();
  service.call_async("Engage", "{\"robot_ids\":[\"a\"]}", Clock::now() + seconds(60),
                     [&](ServiceReply reply) { released_promise.set_value(std::move(reply)); });
  service.call_async("Engage", "{\"robot_ids\":[\"b\"]}", Clock::now() + seconds(60),
                     [&](ServiceReply reply) { removed_promise.set_value(std::move(reply)); });
  CHECK(released.wait_for(milliseconds(50)) == std::future_status::timeout);
  CHECK(removed.wait_for(milliseconds(50)) == std::future_status::timeout);

  domain.remove("b");  // nothing is left to wait for
  REQUIRE(removed.wait_for(seconds(5)) == std::future_status::ready);
  CHECK_EQ(body_of(removed.get())["robots"][0]["outcome"].get<std::string>(), "engaged");
  CHECK(released.wait_for(milliseconds(50)) == std::future_status::timeout);

  domain.release(domain.instance_id(), {{"a", 1}});  // queued behind the engage
  domain.tick(sink);                                 // applies both: the robot is released when the reply is sent
  REQUIRE(released.wait_for(seconds(5)) == std::future_status::ready);
  json a = body_of(released.get())["robots"][0];
  CHECK_EQ(a["outcome"].get<std::string>(), "engaged");  // what the engage did
  CHECK(!a["held"].get<bool>());                          // what is true when the reply is sent
  CHECK_EQ(a["stage"].get<std::string>(), "released");
}

TEST(destroying_the_service_completes_pending_replies) {
  Domain domain;
  domain.add("a");
  std::promise<ServiceReply> promise;
  std::future<ServiceReply> future = promise.get_future();
  {
    ServiceOptions options;
    options.reply_wait = seconds(30);
    Service service(domain, options);
    service.call_async("Engage", "{\"robot_ids\":[\"a\"]}", Clock::now() + seconds(60),
                       [&](ServiceReply reply) { promise.set_value(std::move(reply)); });
    CHECK(future.wait_for(milliseconds(30)) == std::future_status::timeout);
  }
  REQUIRE(future.wait_for(milliseconds(0)) == std::future_status::ready);
  CHECK_EQ(body_of(future.get())["robots"][0]["stage"].get<std::string>(), "gated");
  // The domain outlives the service and must not call into it anymore.
  Sink sink;
  domain.tick(sink);
  domain.release(domain.instance_id(), {{"a", 1}});
}

TEST(a_throwing_callback_does_not_stop_the_service) {
  Domain domain;
  domain.add("a");
  domain.add("b");
  ServiceOptions options;
  options.reply_wait = seconds(10);
  Service service(domain, options);

  service.call_async("Engage", "{\"robot_ids\":[\"a\"]}", Clock::now() + seconds(20),
                     [](ServiceReply) { throw std::runtime_error("transport failed"); });
  std::promise<ServiceReply> promise;
  std::future<ServiceReply> future = promise.get_future();
  service.call_async("Engage", "{\"robot_ids\":[\"b\"]}", Clock::now() + seconds(20),
                     [&](ServiceReply reply) { promise.set_value(std::move(reply)); });
  Sink sink;
  domain.tick(sink);
  REQUIRE(future.wait_for(seconds(5)) == std::future_status::ready);
  CHECK_EQ(body_of(future.get())["robots"][0]["stage"].get<std::string>(), "zero_written");
  CHECK_EQ(service.call("Describe", "{}").status, Status::ok);
}

TEST(release_applies_the_compare_and_set) {
  Domain domain;
  domain.add("a");
  domain.add("b");
  Service service(domain, quick_wait());
  Sink sink;
  TickThread ticker(domain, sink);
  const std::string instance = domain.instance_id();
  CHECK_EQ(service.call("Engage", "{\"robot_ids\":[\"a\",\"b\"]}").status, Status::ok);

  const std::string changes =
      "[{\"robot_id\":\"a\",\"expected_revision\":1},{\"robot_id\":\"b\",\"expected_revision\":7},"
      "{\"robot_id\":\"zz\",\"expected_revision\":0}]";
  const ServiceReply reply = service.call("Release", release_body(instance, changes));
  CHECK_EQ(reply.status, Status::ok);
  const json j = body_of(reply);
  CHECK_EQ(j["instance"].get<std::string>(), instance);
  REQUIRE(j["robots"].size() == 3);
  json a = j["robots"][0];
  CHECK(only_keys(a, {"robot_id", "outcome", "held", "revision", "stage"}));
  CHECK_EQ(a["outcome"].get<std::string>(), "released");
  CHECK(!a["held"].get<bool>());
  CHECK_EQ(a["revision"].get<std::uint64_t>(), 2);
  CHECK_EQ(a["stage"].get<std::string>(), "released");
  json b = j["robots"][1];
  CHECK_EQ(b["outcome"].get<std::string>(), "conflict");
  CHECK(b["held"].get<bool>());
  CHECK_EQ(b["revision"].get<std::uint64_t>(), 1);
  CHECK_EQ(j["robots"][2]["outcome"].get<std::string>(), "unknown_robot");

  // Replayed: the robot is not held anymore.
  const json replay = body_of(service.call("Release", release_body(instance, changes)));
  CHECK_EQ(replay["robots"][0]["outcome"].get<std::string>(), "not_held");

  // A caller that talked to another incarnation of the host gets a conflict for every robot, not an error.
  const std::string other(32, 'a');
  const ServiceReply stale = service.call(
      "Release", release_body(other, "[{\"robot_id\":\"b\",\"expected_revision\":1}]"));
  CHECK_EQ(stale.status, Status::ok);
  const json s = body_of(stale);
  CHECK_EQ(s["instance"].get<std::string>(), instance);
  CHECK_EQ(s["robots"][0]["outcome"].get<std::string>(), "conflict");
  CHECK(domain.state({"b"}).robots[0].held);
}

TEST(a_release_waits_for_the_tick_and_is_not_applied_after_its_call_gave_up) {
  Domain domain;
  domain.add("a");
  domain.add("b");
  ServiceOptions options;
  options.reply_wait = milliseconds(0);
  options.release_wait = milliseconds(60);
  Service service(domain, options);
  Sink sink;
  const std::string instance = domain.instance_id();
  const std::string release = release_body(
      instance, "[{\"robot_id\":\"a\",\"expected_revision\":1},{\"robot_id\":\"zz\",\"expected_revision\":0}]");
  domain.engage({"a", "b"});
  domain.tick(sink);  // this thread is the tick thread, and it stops ticking for now

  // Nobody ticks: the change is withdrawn at the end of the wait and the robot stays held. The reply says so per
  // robot; it is not an error.
  const auto started = Clock::now();
  const ServiceReply gave_up = service.call("Release", release);
  CHECK(Clock::now() - started >= milliseconds(50));
  CHECK_EQ(gave_up.status, Status::ok);
  const json first = body_of(gave_up);
  REQUIRE(first["robots"].size() == 2);
  CHECK_EQ(first["robots"][0]["outcome"].get<std::string>(), "expired");
  CHECK(first["robots"][0]["held"].get<bool>());
  CHECK_EQ(first["robots"][0]["revision"].get<std::uint64_t>(), 1);
  CHECK_EQ(first["robots"][1]["outcome"].get<std::string>(), "unknown_robot");
  CHECK(domain.state({"a"}).robots[0].held);
  CHECK(!domain.admit("a", domain.now()));

  // The tick comes back: the change that gave up is not applied late.
  domain.tick(sink);
  CHECK(domain.state({"a"}).robots[0].held);
  CHECK(!domain.admit("a", domain.now()));

  // A release the tick gets to goes through; the reply comes from the service thread, not the tick thread.
  std::promise<ServiceReply> promise;
  std::future<ServiceReply> future = promise.get_future();
  std::thread::id callback_thread;
  options.release_wait = seconds(10);
  Service patient(domain, options);
  patient.call_async("Release", release, Clock::now() + seconds(20), [&](ServiceReply reply) {
    callback_thread = std::this_thread::get_id();
    promise.set_value(std::move(reply));
  });
  CHECK(future.wait_for(milliseconds(50)) == std::future_status::timeout);
  domain.tick(sink);
  REQUIRE(future.wait_for(seconds(5)) == std::future_status::ready);
  CHECK(callback_thread != std::this_thread::get_id());
  const ServiceReply done = future.get();
  CHECK_EQ(done.status, Status::ok);
  CHECK_EQ(body_of(done)["robots"][0]["outcome"].get<std::string>(), "released");

  // A request whose deadline is over by the time it arrives is executed if it is an engage and withdrawn if it
  // is a release.
  const ServiceReply late_release = patient.call("Release", release_body(instance, "[{\"robot_id\":\"b\",\"expected_revision\":1}]"),
                                                 Clock::now() - seconds(1));
  CHECK_EQ(body_of(late_release)["robots"][0]["outcome"].get<std::string>(), "expired");
  CHECK(domain.state({"b"}).robots[0].held);
  domain.tick(sink);
  CHECK(domain.state({"b"}).robots[0].held);
}

// A control loop that sleeps until it is woken: DomainOptions::wake is all the host has to connect.
TEST(a_sleeping_host_loop_is_woken_by_the_requests) {
  std::mutex mutex;
  std::condition_variable woken;
  bool work = false;
  bool stop = false;
  DomainOptions domain_options;
  domain_options.wake = [&] {
    {
      std::lock_guard<std::mutex> lock(mutex);
      work = true;
    }
    woken.notify_one();
  };
  Domain domain(domain_options);
  domain.add("a");
  ServiceOptions options;
  options.reply_wait = seconds(10);
  options.release_wait = seconds(10);
  Service service(domain, options);
  Sink sink;
  std::thread host([&] {
    std::unique_lock<std::mutex> lock(mutex);
    while (!stop) {
      woken.wait_for(lock, seconds(60), [&] { return work || stop; });
      work = false;
      lock.unlock();
      domain.tick(sink);  // this thread is the tick thread
      lock.lock();
    }
  });

  const auto started = Clock::now();
  const ServiceReply engage = service.call("Engage", "{\"robot_ids\":[\"a\"]}");
  CHECK_EQ(engage.status, Status::ok);
  CHECK_EQ(body_of(engage)["robots"][0]["stage"].get<std::string>(), "zero_written");
  const ServiceReply release = service.call(
      "Release", release_body(domain.instance_id(), "[{\"robot_id\":\"a\",\"expected_revision\":1}]"));
  CHECK_EQ(release.status, Status::ok);
  CHECK_EQ(body_of(release)["robots"][0]["outcome"].get<std::string>(), "released");
  CHECK(Clock::now() - started < seconds(5));  // nobody polled: the requests woke the loop
  CHECK(domain.admit("a", domain.now()));

  {
    std::lock_guard<std::mutex> lock(mutex);
    stop = true;
  }
  woken.notify_one();
  host.join();
}

TEST(requests_are_closed_schemas) {
  Domain domain;
  domain.add("a");
  domain.add("b");
  ServiceOptions options = quick_wait();
  options.max_ids = 3;
  options.max_body_bytes = 600;
  Service service(domain, options);
  Sink sink;
  TickThread ticker(domain, sink);  // a valid release waits for it
  const std::string instance = domain.instance_id();
  const std::string change = "{\"robot_id\":\"a\",\"expected_revision\":1}";

  struct Case {
    const char *method;
    std::string body;
  };
  const std::vector<Case> invalid = {
      // not a JSON object
      {"Describe", "{"},
      {"Describe", "[]"},
      {"Describe", "\"text\""},
      {"Describe", "null"},
      {"Describe", "123"},
      {"Describe", "{} trailing"},
      {"Describe", "{\"x\":1}"},
      {"State", "{\"all\":true}"},
      {"State", "{\"robot_ids\":\"a\"}"},
      {"State", "{\"robot_ids\":null}"},
      {"State", "{\"robot_ids\":[1]}"},
      {"State", "{\"robot_ids\":[\"a\",\"a\"]}"},
      {"State", "{\"robot_ids\":[\"bad id\"]}"},
      {"State", "{\"robot_ids\":[\"\"]}"},
      {"State", "{\"robot_ids\":[\"a\",\"b\",\"c\",\"d\"]}"},  // more than max_ids
      // Engage
      {"Engage", "{}"},
      {"Engage", "{\"robot_ids\":[]}"},
      {"Engage", "{\"all\":false}"},
      {"Engage", "{\"all\":\"yes\"}"},
      {"Engage", "{\"all\":1}"},
      {"Engage", "{\"all\":true,\"robot_ids\":[\"a\"]}"},
      {"Engage", "{\"robot_ids\":[\"a\"],\"extra\":1}"},
      {"Engage", "{\"robot_ids\":[\"a\",\"a\"]}"},
      {"Engage", "{\"robot_ids\":[[\"a\"]]}"},
      {"Engage", "{\"robot_ids\":[\"a\\u0000b\"]}"},
      {"Engage", "{\"robot_ids\":[\"\\u00e9\"]}"},
      {"Engage", "{\"robot_ids\":[\"" + std::string(129, 'x') + "\"]}"},
      {"Engage", "{\"robot_ids\":[\"a\",\"b\",\"c\",\"d\"]}"},
      {"Engage", "{\"robot_ids\":{\"a\":1}}"},
      // Release
      {"Release", "{}"},
      {"Release", "{\"changes\":[" + change + "]}"},
      {"Release", "{\"expected_instance\":\"" + instance + "\"}"},
      {"Release", release_body("short", "[" + change + "]")},
      {"Release", release_body(std::string(32, 'A'), "[" + change + "]")},
      {"Release", release_body(std::string(32, 'g'), "[" + change + "]")},
      {"Release", "{\"expected_instance\":null,\"changes\":[" + change + "]}"},
      {"Release", "{\"expected_instance\":1,\"changes\":[" + change + "]}"},
      {"Release", release_body(instance, "[]")},
      {"Release", release_body(instance, "{}")},
      {"Release", release_body(instance, "\"a\"")},
      {"Release", release_body(instance, "[\"a\"]")},
      {"Release", release_body(instance, "[{\"robot_id\":\"a\"}]")},
      {"Release", release_body(instance, "[{\"expected_revision\":1}]")},
      {"Release", release_body(instance, "[{\"robot_id\":\"a\",\"expected_revision\":-1}]")},
      {"Release", release_body(instance, "[{\"robot_id\":\"a\",\"expected_revision\":1.0}]")},
      {"Release", release_body(instance, "[{\"robot_id\":\"a\",\"expected_revision\":1e0}]")},
      {"Release", release_body(instance, "[{\"robot_id\":\"a\",\"expected_revision\":\"1\"}]")},
      {"Release", release_body(instance, "[{\"robot_id\":\"a\",\"expected_revision\":true}]")},
      {"Release", release_body(instance, "[{\"robot_id\":\"a\",\"expected_revision\":18446744073709551616}]")},
      {"Release", release_body(instance, "[{\"robot_id\":\"a\",\"expected_revision\":1,\"held\":false}]")},
      {"Release", release_body(instance, "[{\"robot_id\":7,\"expected_revision\":1}]")},
      {"Release", release_body(instance, "[" + change + "," + change + "]")},
      {"Release", release_body(instance, "[" + change + "," + change + "," + change + "," + change + "]")},
      {"Release", "{\"expected_instance\":\"" + instance + "\",\"changes\":[" + change + "],\"all\":true}"},
      // size and nesting
      {"State", "{\"robot_ids\":[" + std::string(700, ' ') + "]}"},
      {"State", std::string(60, '[') + std::string(60, ']')},
      {"State", "{\"robot_ids\":[[[[[[[[\"a\"]]]]]]]]}"},
  };
  for (const auto &item : invalid) {
    const ServiceReply reply = service.call(item.method, item.body);
    if (reply.status != Status::invalid_argument)
      test::fail(__FILE__, __LINE__, std::string(item.method) + " accepted: " + item.body.substr(0, 120));
    const json j = body_of(reply);
    CHECK(only_keys(j, {"code", "message"}));
    CHECK_EQ(j["code"].get<std::string>(), "invalid_argument");
    CHECK(!j["message"].get<std::string>().empty());
    CHECK(printable_ascii(reply.body));
  }
  // A rejected request changes nothing.
  CHECK(!domain.state({"a"}).robots[0].held);
  CHECK_EQ(domain.state({"a"}).robots[0].revision, 0);

  // Limits are inclusive; the largest revision is a valid (and conflicting) request.
  CHECK_EQ(service.call("State", "{\"robot_ids\":[\"a\",\"b\"]}").status, Status::ok);
  CHECK_EQ(service.call("Release", release_body(instance, "[{\"robot_id\":\"a\",\"expected_revision\":"
                                                          "18446744073709551615}]"))
               .status,
           Status::ok);
  CHECK_EQ(service.call("State", "{\"robot_ids\":[\"ugv-1.a_b:c\"]}").status, Status::not_found);
}

TEST(unknown_methods_are_not_found) {
  Domain domain;
  Service service(domain);
  for (const char *method : {"", "describe", "Describe ", "Hold", "xgc2.chassis.hold/Describe"}) {
    const ServiceReply reply = service.call(method, "{}");
    CHECK_EQ(reply.status, Status::not_found);
    CHECK_EQ(body_of(reply)["code"].get<std::string>(), "not_found");
  }
  const ServiceReply odd = service.call(std::string("bad\xff\x01method"), "{}");
  CHECK_EQ(odd.status, Status::not_found);
  CHECK(printable_ascii(odd.body));
}

TEST(hostile_text_never_reaches_a_reply_unescaped) {
  Domain domain;
  domain.add("a");
  Service service(domain);
  // Unknown field names and ids are echoed in messages only after they were made printable.
  for (const std::string &body : {std::string("{\"\xff\x01\xc3\":1}"), std::string("{\"robot_ids\":[\"\xff\xfe\"]}"),
                                 std::string("{\"robot_ids\":[\"a\"],\"\\u00e9\\ud83d\\ude00\":1}")}) {
    for (const char *method : {"State", "Engage", "Describe"}) {
      const ServiceReply reply = service.call(method, body);
      CHECK(reply.status == Status::invalid_argument);
      CHECK(printable_ascii(reply.body));
      CHECK(!reply.body.empty() && reply.body.front() == '{');
    }
  }
  CHECK(!domain.state({"a"}).robots[0].held);
}

TEST(all_forms_are_refused_for_a_roster_larger_than_max_ids) {
  Domain domain;
  for (const char *id : {"a", "b", "c"})
    domain.add(id);
  ServiceOptions options = quick_wait();
  options.max_ids = 2;
  Service service(domain, options);

  // The reply for every robot would exceed the limit: refused before anything is engaged.
  const ServiceReply engage = service.call("Engage", "{\"all\":true}");
  CHECK_EQ(engage.status, Status::invalid_argument);
  CHECK_EQ(service.call("State", "{}").status, Status::invalid_argument);
  CHECK_EQ(service.call("State", "").status, Status::invalid_argument);
  for (const auto &robot : domain.state().robots)
    CHECK(!robot.held);

  CHECK_EQ(service.call("State", "{\"robot_ids\":[\"a\",\"b\"]}").status, Status::ok);
  CHECK_EQ(service.call("Engage", "{\"robot_ids\":[\"c\"]}").status, Status::ok);
  CHECK_EQ(service.call("Describe", "{}").status, Status::ok);  // not limited: names only
}

TEST(replies_for_a_single_robot_fit_one_datagram) {
  FakeClock clock;
  Domain domain(clock.options());
  const std::string id(128, 'r');
  domain.add(id, true);
  Service service(domain, quick_wait());
  Sink sink;
  sink.fail(id, true);
  TickThread ticker(domain, sink);
  domain.engage({id});
  domain.set_fault(id, std::string(500, 'f'));
  REQUIRE(wait_until([&] { return !domain.state({id}).robots[0].fault.empty(); }));

  const std::string ids = "[\"" + id + "\"]";
  const ServiceReply describe = service.call("Describe", "{}");
  const ServiceReply state = service.call("State", "{\"robot_ids\":" + ids + "}");
  const ServiceReply engage = service.call("Engage", "{\"robot_ids\":" + ids + "}");
  const ServiceReply release = service.call("Release", release_body(domain.instance_id(), "[{\"robot_id\":\"" + id +
                                                                                           "\",\"expected_revision\":1}]"));
  // udp.v1: 1200 bytes minus 52 header, 128 method and 32 tag bytes leave 988 for the body.
  for (const ServiceReply *reply : {&describe, &state, &engage, &release}) {
    CHECK_EQ(reply->status, Status::ok);
    CHECK(reply->body.size() < 900);
  }
  CHECK_EQ(body_of(release)["robots"][0]["outcome"].get<std::string>(), "conflict");  // a fault holds the robot
}

TEST(body_size_limit_is_inclusive_and_options_are_validated) {
  Domain domain;
  domain.add("a");
  ServiceOptions options;
  options.max_body_bytes = 40;
  Service service(domain, options);
  const std::string exact = "{\"robot_ids\":[\"a\"]" + std::string(21, ' ') + "}";
  CHECK_EQ(exact.size(), 40);
  CHECK_EQ(service.call("State", exact).status, Status::ok);
  CHECK_EQ(service.call("State", exact + " ").status, Status::invalid_argument);

  const auto throws = [&domain](ServiceOptions bad) {
    try {
      Service service(domain, bad);
    } catch (const std::invalid_argument &) {
      return true;
    }
    return false;
  };
  ServiceOptions negative;
  negative.reply_wait = milliseconds(-1);
  CHECK(throws(negative));
  ServiceOptions endless;
  endless.reply_wait = std::chrono::hours(2);
  CHECK(throws(endless));
  ServiceOptions no_release_wait;
  no_release_wait.release_wait = milliseconds(0);  // every release would be withdrawn at once
  CHECK(throws(no_release_wait));
  ServiceOptions endless_release;
  endless_release.release_wait = std::chrono::hours(2);
  CHECK(throws(endless_release));
  ServiceOptions no_ids;
  no_ids.max_ids = 0;
  CHECK(throws(no_ids));
  ServiceOptions no_body;
  no_body.max_body_bytes = 1;
  CHECK(throws(no_body));
}

// Mutated requests are answered or refused, never crash or leak text: meant for the sanitizer builds. The
// generator is fixed, so a failure reproduces.
TEST(mutated_requests_are_answered_safely) {
  FakeClock clock;
  Domain domain(clock.options());
  for (const char *id : {"a", "b", "ugv-1"})
    domain.add(id, true);
  ServiceOptions options;
  options.reply_wait = milliseconds(0);
  options.max_ids = 4;
  Service service(domain, options);
  Sink sink;
  TickThread ticker(domain, sink);  // the valid releases wait for it
  const std::string instance = domain.instance_id();
  const std::vector<std::pair<std::string, std::string>> seeds = {
      {"Describe", "{}"},
      {"State", "{}"},
      {"State", "{\"robot_ids\":[\"a\",\"ugv-1\"]}"},
      {"Engage", "{\"robot_ids\":[\"a\",\"zz\"]}"},
      {"Engage", "{\"all\":true}"},
      {"Release", release_body(instance, "[{\"robot_id\":\"a\",\"expected_revision\":1},"
                                         "{\"robot_id\":\"b\",\"expected_revision\":18446744073709551615}]")},
  };
  std::uint64_t random = 0x9e3779b97f4a7c15ULL;
  const auto next = [&random] {
    random ^= random << 13;
    random ^= random >> 7;
    random ^= random << 17;
    return random;
  };
  const std::string alphabet = "{}[]\":,-+.0123456789eE \\tnulfaser_\x01\x7f\x80\xc3\xff";
  int answered = 0;
  for (int i = 0; i < 20000; ++i) {
    const auto &seed = seeds[next() % seeds.size()];
    std::string body = seed.second;
    for (int edits = 1 + static_cast<int>(next() % 4); edits > 0 && !body.empty(); --edits) {
      const std::size_t at = next() % body.size();
      switch (next() % 5) {
        case 0: body[at] = alphabet[next() % alphabet.size()]; break;
        case 1: body.insert(at, 1, alphabet[next() % alphabet.size()]); break;
        case 2: body.erase(at, 1 + next() % 8); break;
        case 3: body.insert(at, body.substr(at, 1 + next() % 16)); break;
        default: body.resize(at); break;
      }
    }
    const ServiceReply reply = service.call(seed.first, body);
    json parsed;
    try {
      parsed = json::parse(reply.body);
    } catch (const std::exception &) {
      test::fail(__FILE__, __LINE__, "reply is not JSON for: " + body);
      return;
    }
    if (!printable_ascii(reply.body) && reply.status != Status::ok)
      test::fail(__FILE__, __LINE__, "error text is not printable ASCII for: " + body);
    if (reply.status == Status::ok) {
      ++answered;
      CHECK(parsed.is_object());
    } else {
      CHECK(only_keys(parsed, {"code", "message"}) || only_keys(parsed, {"code", "message", "details"}));
    }
    if (i % 500 == 0) {  // keep the robots from staying held forever so Engage keeps doing work
      domain.release(instance, {{"a", domain.state({"a"}).robots[0].revision}});
      domain.release(instance, {{"b", domain.state({"b"}).robots[0].revision}});
    }
  }
  CHECK(answered > 500);  // the mutations leave a good share of requests valid
}

TEST_MAIN()
