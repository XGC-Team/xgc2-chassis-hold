// Threads against one domain and one service. Meant to run under -fsanitize=thread and
// -fsanitize=address,undefined as well as plain.
#include "check.hpp"
#include "fixtures.hpp"

#include <xgc2/chassis_hold/service.hpp>

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <future>
#include <thread>
#include <vector>

using namespace xgc2::chassis_hold;
using json = nlohmann::json;
using fixtures::Sink;
using fixtures::TickThread;
using Clock = Service::Clock;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::seconds;

namespace {

using fixtures::wait_until;

// Threads of a test cannot REQUIRE: report and return an empty state instead.
RobotState get(const Domain &domain, const std::string &id) {
  const Snapshot snapshot = domain.state({id});
  if (snapshot.robots.size() != 1) {
    test::fail(__FILE__, __LINE__, "unknown robot " + id);
    return RobotState();
  }
  return snapshot.robots[0];
}

constexpr int robots = 3;

}  // namespace

// engage/release from one thread while another thread ticks, commands are admitted, state is read and the
// roster churns. Invariants are checked from the outside only.
TEST(engage_and_release_while_ticks_run) {
  DomainOptions options;
  options.stopped_dwell = milliseconds(2);  // the robot with feedback reaches stopped within a hold
  Domain domain(options);
  const std::string ids[robots] = {"r0", "r1", "r2"};
  for (const auto &id : ids)
    domain.add(id, id == "r2");
  Sink sink;
  std::atomic<bool> stop{false};
  // Per robot: (transition counter << 2) | phase, phase 0 released, 1 held (engage has returned), 2 releasing.
  // Only the main thread writes; a reader that sees one value before and after a call knows no transition
  // happened during it.
  std::atomic<std::uint64_t> gate[robots];
  for (auto &g : gate)
    g = 0;
  const auto set_phase = [&](int i, std::uint64_t phase) { gate[i] = (((gate[i] >> 2) + 1) << 2) | phase; };

  TickThread ticker(domain, sink, [&] {
    // Feedback of the robot that has any: at rest.
    domain.observe("r2", 0.0, 0.0, domain.now());
  });

  // Commands arrive on their own thread and are admitted with their receipt time.
  std::atomic<std::uint64_t> admitted{0};
  std::thread commands([&] {
    while (!stop) {
      for (int i = 0; i < robots; ++i) {
        const std::uint64_t before = gate[i];
        const bool admit = domain.admit(ids[i], domain.now());
        const std::uint64_t after = gate[i];
        if (before == after && (before & 3) == 1 && admit)
          test::fail(__FILE__, __LINE__, "a command was admitted while " + ids[i] + " was held");
        admitted += admit;
      }
    }
  });

  // Readers: a snapshot is consistent with the state machine rules at every point.
  std::thread reader([&] {
    std::uint64_t last_revision[robots] = {0, 0, 0};
    while (!stop) {
      const Snapshot snapshot = domain.state();
      for (const auto &state : snapshot.robots) {
        if (state.robot_id.size() != 2 || state.robot_id[0] != 'r')
          continue;  // the roster-churn robot
        const int i = state.robot_id[1] - '0';
        if (state.revision < last_revision[i])
          test::fail(__FILE__, __LINE__, "revision went backwards for " + state.robot_id);
        last_revision[i] = state.revision;
        if ((state.revision % 2 == 1) != state.held)
          test::fail(__FILE__, __LINE__, "held does not match the revision parity for " + state.robot_id);
        if (state.held == (state.stage == Stage::released))
          test::fail(__FILE__, __LINE__, "held does not match the stage for " + state.robot_id);
        const bool zero = state.stage == Stage::zero_written || state.stage == Stage::stopped;
        if (zero != state.zero_written_at.has_value())
          test::fail(__FILE__, __LINE__, "zero_written_at does not match the stage for " + state.robot_id);
        if ((state.stage == Stage::stopped) != state.stopped_at.has_value())
          test::fail(__FILE__, __LINE__, "stopped_at does not match the stage for " + state.robot_id);
        if (state.stage == Stage::stopped && !state.feedback)
          test::fail(__FILE__, __LINE__, "stopped without feedback for " + state.robot_id);
        if (state.held != state.gated_at.has_value())
          test::fail(__FILE__, __LINE__, "gated_at does not match held for " + state.robot_id);
      }
      domain.describe();
    }
  });

  // A model that is deleted and created again keeps its HOLD. Its releases are not waited for: the next round
  // engages it again, in the queue behind the release, and the gate must stay closed through all of it.
  std::thread churn([&] {
    domain.add("model");
    for (int i = 0; i < 150; ++i) {
      domain.engage({"model"});
      domain.remove("model");
      if (domain.admit("model", domain.now()))
        test::fail(__FILE__, __LINE__, "a removed robot admitted a command");
      domain.add("model");
      const RobotState state = get(domain, "model");
      if (!state.held || domain.admit("model", domain.now()))
        test::fail(__FILE__, __LINE__, "a re-created model did not inherit HOLD");
      if (state.stage != Stage::gated && state.stage != Stage::zero_written)
        test::fail(__FILE__, __LINE__, "a held model is neither gated nor zeroed");
      domain.release(domain.instance_id(), {{"model", state.revision}});
    }
  });

  for (int round = 0; round < 150; ++round) {
    for (int i = 0; i < robots; ++i) {
      const Pending engaged = domain.engage({ids[i]});
      CHECK_EQ(engaged.results()[0].outcome, Outcome::engaged);
      set_phase(i, 1);
      // The gate is effective for this thread right away.
      CHECK(!domain.admit(ids[i], domain.now()));
      // The tick writes zero without being asked again.
      const int before = sink.writes(ids[i]);
      CHECK(wait_until([&] { return get(domain, ids[i]).stage != Stage::gated; }, seconds(10)));
      CHECK(wait_until([&] { return sink.writes(ids[i]) >= before + 3; }, seconds(10)));  // re-asserted

      set_phase(i, 2);
      const TimeNs before_release = domain.now();
      const Pending released = domain.release(domain.instance_id(), {{ids[i], get(domain, ids[i]).revision}});
      CHECK(wait_until([&] { return released.done(); }, seconds(10)));  // the tick applies it
      CHECK_EQ(released.results()[0].outcome, Outcome::released);
      set_phase(i, 0);
      // Commands received before the release call are dropped, later ones are admitted.
      CHECK(!domain.admit(ids[i], before_release));
      CHECK(domain.admit(ids[i], domain.now()));
    }
  }
  churn.join();
  stop = true;
  commands.join();
  reader.join();
  ticker.stop();
  for (const auto &id : ids)
    CHECK(!get(domain, id).held);
  CHECK(admitted.load() > 0);
}

// Many threads engage the same robot and release it with the same revision: exactly one of each succeeds.
TEST(racing_callers_get_exactly_one_transition) {
  Domain domain;
  domain.add("a");
  Service service(domain);
  Sink sink;
  TickThread ticker(domain, sink);
  constexpr int callers = 8;

  for (int round = 0; round < 40; ++round) {
    std::atomic<int> engaged{0}, already{0}, released{0}, other{0};
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;
    const std::uint64_t revision = get(domain, "a").revision;
    for (int c = 0; c < callers; ++c)
      threads.emplace_back([&] {
        while (!go)
          std::this_thread::yield();
        const json engage = json::parse(service.call("Engage", "{\"robot_ids\":[\"a\"]}").body);
        const std::string outcome = engage["robots"][0]["outcome"].get<std::string>();
        if (outcome == "engaged")
          ++engaged;
        else if (outcome == "already_held")
          ++already;
        else
          ++other;
      });
    go = true;
    for (auto &t : threads)
      t.join();
    CHECK_EQ(engaged.load(), 1);
    CHECK_EQ(already.load(), callers - 1);
    CHECK_EQ(other.load(), 0);
    CHECK_EQ(get(domain, "a").revision, revision + 1);

    threads.clear();
    go = false;
    const std::string body = "{\"expected_instance\":\"" + domain.instance_id() +
                             "\",\"changes\":[{\"robot_id\":\"a\",\"expected_revision\":" +
                             std::to_string(revision + 1) + "}]}";
    for (int c = 0; c < callers; ++c)
      threads.emplace_back([&] {
        while (!go)
          std::this_thread::yield();
        const json release = json::parse(service.call("Release", body).body);
        const std::string outcome = release["robots"][0]["outcome"].get<std::string>();
        if (outcome == "released")
          ++released;
        else if (outcome == "not_held" || outcome == "conflict")
          ++other;
      });
    go = true;
    for (auto &t : threads)
      t.join();
    CHECK_EQ(released.load(), 1);
    CHECK_EQ(other.load(), callers - 1);
    CHECK_EQ(get(domain, "a").revision, revision + 2);
  }
}

// Async engages from several client threads while the tick thread runs: every callback runs exactly once, and
// with a ticking host the reply reports the zeroed stage.
TEST(async_engage_replies_under_concurrent_ticks) {
  Domain domain;
  constexpr int clients = 4;
  const std::string ids[clients] = {"c0", "c1", "c2", "c3"};
  for (const auto &id : ids)
    domain.add(id);
  ServiceOptions options;
  options.reply_wait = seconds(10);
  Service service(domain, options);
  Sink sink;
  TickThread ticker(domain, sink);
  std::atomic<int> callbacks{0};
  std::atomic<int> zeroed{0};

  std::vector<std::thread> threads;
  for (int c = 0; c < clients; ++c)
    threads.emplace_back([&, c] {
      for (int round = 0; round < 60; ++round) {
        std::promise<ServiceReply> promise;
        std::future<ServiceReply> future = promise.get_future();
        service.call_async("Engage", "{\"robot_ids\":[\"" + ids[c] + "\"]}", Clock::now() + seconds(20),
                           [&](ServiceReply reply) {
                             ++callbacks;
                             promise.set_value(std::move(reply));
                           });
        if (future.wait_for(seconds(15)) != std::future_status::ready) {
          test::fail(__FILE__, __LINE__, "engage reply did not arrive");
          return;
        }
        const json reply = json::parse(future.get().body);
        const json &robot = reply["robots"][0];
        if (robot["stage"].get<std::string>() == "zero_written")
          ++zeroed;
        const std::string body = "{\"expected_instance\":\"" + domain.instance_id() +
                                 "\",\"changes\":[{\"robot_id\":\"" + ids[c] +
                                 "\",\"expected_revision\":" + std::to_string(robot["revision"].get<std::uint64_t>()) +
                                 "}]}";
        const json released = json::parse(service.call("Release", body).body);
        if (released["robots"][0]["outcome"].get<std::string>() != "released")
          test::fail(__FILE__, __LINE__, "release did not apply");
      }
    });
  for (auto &t : threads)
    t.join();
  CHECK_EQ(callbacks.load(), clients * 60);
  CHECK_EQ(zeroed.load(), clients * 60);
}

// A fault reported by the host engages the robot and sets the fault in one step of the machine's queue. A release
// that races it, at any point, can neither undo the hold nor swallow the fault.
TEST(a_release_cannot_come_between_a_fault_and_its_engage) {
  Domain domain;
  domain.add("a");
  Sink sink;
  TickThread ticker(domain, sink);
  std::atomic<bool> stop{false};
  std::thread releaser([&] {
    std::deque<Pending> outstanding;
    while (!stop) {
      outstanding.push_back(domain.release(domain.instance_id(), {{"a", get(domain, "a").revision}}));
      if (outstanding.size() >= 32) {  // the machine's inbox is bounded: keep the backlog small
        wait_until([&] { return outstanding.front().done(); }, seconds(10));
        outstanding.pop_front();
      }
    }
    for (const auto &pending : outstanding)
      wait_until([&] { return pending.done(); }, seconds(10));
  });
  std::atomic<bool> invariant{true};
  std::thread reader([&] {
    while (!stop) {
      const RobotState state = get(domain, "a");
      if (!state.fault.empty() && !state.held)
        invariant = false;
    }
  });

  for (int round = 0; round < 200; ++round) {
    CHECK(domain.set_fault("a", "bus off"));
    CHECK(wait_until([&] { return get(domain, "a").fault == "bus off"; }, seconds(10)));
    CHECK(get(domain, "a").held);
    CHECK(!domain.admit("a", domain.now()));
    CHECK(domain.set_fault("a", ""));
    CHECK(wait_until([&] { return get(domain, "a").fault.empty(); }, seconds(10)));
  }
  stop = true;
  releaser.join();
  reader.join();
  CHECK(invariant.load());
  CHECK(get(domain, "a").fault.empty());
}

// A device write that does not return must not hold up the gate, the other operations or the replies. What it does
// hold up is the machine: a release needs the tick, so it fails instead of waiting for a device that is stuck.
TEST(a_stuck_device_write_does_not_delay_the_gate) {
  Domain domain;
  domain.add("a");
  domain.add("b");
  ServiceOptions options;
  options.reply_wait = milliseconds(100);
  options.release_wait = milliseconds(150);
  Service service(domain, options);
  Sink sink;
  std::atomic<bool> unblock{false};
  std::atomic<bool> inside{false};
  sink.during = [&] {
    inside = true;
    while (!unblock)
      std::this_thread::sleep_for(milliseconds(1));
  };
  domain.engage({"b"});
  TickThread ticker(domain, sink);
  REQUIRE(wait_until([&] { return inside.load(); }, seconds(10)));  // the tick thread is stuck in the sink

  const auto started = Clock::now();
  const ServiceReply engage = service.call("Engage", "{\"robot_ids\":[\"a\"]}");
  const auto elapsed = Clock::now() - started;
  CHECK(!domain.admit("a", domain.now()));
  CHECK(elapsed < seconds(5));
  CHECK_EQ(json::parse(engage.body)["robots"][0]["stage"].get<std::string>(), "gated");  // honest: no zero yet
  CHECK_EQ(json::parse(engage.body)["robots"][0]["revision"].get<std::uint64_t>(), 1);
  CHECK_EQ(service.call("State", "").status, Status::ok);
  CHECK(domain.admit("zz", domain.now()) == false);

  // The release cannot be applied while the tick is stuck: the call fails and nothing is released, now or later.
  const std::string release = "{\"expected_instance\":\"" + domain.instance_id() +
                              "\",\"changes\":[{\"robot_id\":\"a\",\"expected_revision\":1}]}";
  const ServiceReply failed = service.call("Release", release);
  CHECK_EQ(failed.status, Status::deadline_exceeded);
  CHECK(!domain.admit("a", domain.now()));
  CHECK(get(domain, "a").held);

  unblock = true;
  CHECK(wait_until([&] { return get(domain, "a").stage == Stage::zero_written; }, seconds(10)));
  CHECK(wait_until([&] { return get(domain, "b").stage == Stage::zero_written; }, seconds(10)));
  CHECK(get(domain, "a").held);  // the release that gave up was not applied when the tick came back
  CHECK(!domain.admit("a", domain.now()));

  // With the tick running again the same release goes through.
  const ServiceReply done = service.call("Release", release);
  CHECK_EQ(done.status, Status::ok);
  CHECK_EQ(json::parse(done.body)["robots"][0]["outcome"].get<std::string>(), "released");
  CHECK(domain.admit("a", domain.now()));
}

TEST_MAIN()
