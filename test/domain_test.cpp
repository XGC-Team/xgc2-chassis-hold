#include "check.hpp"
#include "fixtures.hpp"

#include <xgc2/chassis_hold/domain.hpp>

#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <thread>

using namespace xgc2::chassis_hold;

namespace {

using fixtures::FakeClock;
using fixtures::Sink;
using fixtures::ms;

RobotState get(const Domain &domain, const std::string &id) {
  const Snapshot snapshot = domain.state({id});
  REQUIRE(snapshot.robots.size() == 1);
  return snapshot.robots[0];
}

std::string bad_instance() { return std::string(32, '0'); }

// The host's tick applies what was asked; every test thread that ticks a domain keeps to its one thread.
std::vector<RobotResult> engaged(Domain &domain, Sink &sink, const std::vector<std::string> &ids) {
  const Pending pending = domain.engage(ids);
  domain.tick(sink);
  REQUIRE(pending.done());
  return pending.results();
}

std::vector<RobotResult> released(Domain &domain, Sink &sink, const std::string &instance,
                                  const std::vector<ReleaseChange> &changes) {
  const Pending pending = domain.release(instance, changes);
  domain.tick(sink);
  REQUIRE(pending.done());
  return pending.results();
}

}  // namespace

TEST(engage_closes_the_gate_at_once_and_the_tick_applies_it) {
  FakeClock clock;
  Domain domain(clock.options());
  Sink sink;
  domain.add("r1");
  domain.add("r2");

  const Pending first = domain.engage({"r1"});
  // The gate is closed when engage returns, for the selected robot only; the machine has not run yet.
  CHECK(!first.done());
  CHECK(!domain.admit("r1", clock.ns));
  CHECK(domain.admit("r2", clock.ns));
  const std::vector<RobotResult> provisional = first.results();
  REQUIRE(provisional.size() == 1);
  CHECK_EQ(provisional[0].outcome, Outcome::engaged);  // what the gate decided
  REQUIRE(provisional[0].state.has_value());
  CHECK(provisional[0].state->held);
  CHECK_EQ(provisional[0].state->revision, 1);  // the revision the tick will give it
  CHECK_EQ(provisional[0].state->stage, Stage::gated);
  CHECK_EQ(*provisional[0].state->gated_at, clock.ns.load());
  // State shows the same at once.
  CHECK(get(domain, "r1").held);
  CHECK_EQ(get(domain, "r1").stage, Stage::gated);
  CHECK(!get(domain, "r2").held);

  const TimeNs engaged_at = clock.ns;
  clock.advance(5);
  domain.tick(sink);
  CHECK(first.done());
  const std::vector<RobotResult> applied = first.results();
  CHECK_EQ(applied[0].outcome, Outcome::engaged);
  CHECK_EQ(applied[0].state->revision, 1);
  CHECK_EQ(applied[0].state->stage, Stage::zero_written);  // the same tick wrote the zero
  CHECK_EQ(*applied[0].state->gated_at, engaged_at);
  CHECK(!domain.admit("r1", clock.ns));

  clock.advance(5);
  const Pending again = domain.engage({"r1"});
  CHECK_EQ(again.results()[0].outcome, Outcome::already_held);
  domain.tick(sink);
  CHECK_EQ(again.results()[0].outcome, Outcome::already_held);
  CHECK_EQ(again.results()[0].state->revision, 1);
  CHECK_EQ(*again.results()[0].state->gated_at, engaged_at);

  const std::vector<RobotResult> mixed = engaged(domain, sink, {"r2", "nobody", "r1"});
  REQUIRE(mixed.size() == 3);
  CHECK_EQ(mixed[0].outcome, Outcome::engaged);
  CHECK_EQ(mixed[1].outcome, Outcome::unknown_robot);
  CHECK(!mixed[1].state.has_value());
  CHECK_EQ(mixed[1].robot_id, "nobody");
  CHECK_EQ(mixed[2].outcome, Outcome::already_held);

  const Pending all = domain.engage_all();
  domain.tick(sink);
  REQUIRE(all.results().size() == 2);
  CHECK_EQ(all.results()[0].outcome, Outcome::already_held);
  CHECK_EQ(all.results()[1].outcome, Outcome::already_held);
}

TEST(revision_counts_held_released_transitions) {
  FakeClock clock;
  Domain domain(clock.options());
  Sink sink;
  domain.add("r");
  CHECK_EQ(get(domain, "r").revision, 0);
  CHECK(!get(domain, "r").held);
  CHECK_EQ(get(domain, "r").stage, Stage::released);

  engaged(domain, sink, {"r"});
  CHECK_EQ(get(domain, "r").revision, 1);
  engaged(domain, sink, {"r"});
  CHECK_EQ(get(domain, "r").revision, 1);

  const std::vector<RobotResult> result = released(domain, sink, domain.instance_id(), {{"r", 1}});
  CHECK_EQ(result[0].outcome, Outcome::released);
  CHECK_EQ(result[0].state->revision, 2);
  CHECK(!result[0].state->held);
  CHECK_EQ(result[0].state->stage, Stage::released);
  CHECK(!result[0].state->gated_at.has_value());

  CHECK_EQ(released(domain, sink, domain.instance_id(), {{"r", 2}})[0].outcome, Outcome::not_held);
  CHECK_EQ(get(domain, "r").revision, 2);
  engaged(domain, sink, {"r"});
  CHECK_EQ(get(domain, "r").revision, 3);
}

TEST(release_is_a_compare_and_set_on_instance_and_revision) {
  FakeClock clock;
  Domain domain(clock.options());
  Sink sink;
  for (const char *id : {"a", "b", "c"})
    domain.add(id);
  engaged(domain, sink, {"a", "b"});
  const std::string instance = domain.instance_id();

  // A stale caller (other instance): every change is a conflict and nothing is released.
  const auto stale = released(domain, sink, bad_instance(), {{"a", 1}, {"b", 1}, {"c", 0}});
  for (const auto &result : stale) {
    CHECK_EQ(result.outcome, Outcome::conflict);
    CHECK(result.state.has_value());
  }
  CHECK(get(domain, "a").held);
  CHECK(get(domain, "b").held);

  // Per robot: wrong revision conflicts (state returned), not held says so, unknown is unknown, the rest applies.
  const auto mixed = released(domain, sink, instance, {{"a", 0}, {"b", 1}, {"c", 0}, {"zzz", 1}});
  REQUIRE(mixed.size() == 4);
  CHECK_EQ(mixed[0].outcome, Outcome::conflict);
  CHECK(mixed[0].state->held);
  CHECK_EQ(mixed[0].state->revision, 1);
  CHECK_EQ(mixed[1].outcome, Outcome::released);
  CHECK_EQ(mixed[2].outcome, Outcome::not_held);
  CHECK_EQ(mixed[3].outcome, Outcome::unknown_robot);
  CHECK(!mixed[3].state.has_value());
  CHECK(get(domain, "a").held);
  CHECK(!get(domain, "b").held);

  // A replayed release never applies twice: not held while released, a conflict once the robot is held again.
  CHECK_EQ(released(domain, sink, instance, {{"b", 1}})[0].outcome, Outcome::not_held);
  engaged(domain, sink, {"b"});
  const auto replay = released(domain, sink, instance, {{"b", 1}});
  CHECK_EQ(replay[0].outcome, Outcome::conflict);
  CHECK_EQ(replay[0].state->revision, 3);
  CHECK(get(domain, "b").held);
  CHECK_EQ(released(domain, sink, instance, {{"b", 3}})[0].outcome, Outcome::released);
}

TEST(release_fence_drops_commands_received_before_release) {
  FakeClock clock;
  Domain domain(clock.options());
  Sink sink;
  domain.add("r");
  // Never released: no fence.
  CHECK(domain.admit("r", 0));
  CHECK(domain.admit("r", clock.ns));

  engaged(domain, sink, {"r"});
  clock.advance(10);
  const TimeNs received_while_held = clock.ns - 3 * ms;
  const Pending release = domain.release(domain.instance_id(), {{"r", 1}});
  CHECK(!domain.admit("r", clock.ns));  // admission opens when the tick applies the release, not before
  clock.advance(7);
  domain.tick(sink);
  REQUIRE(release.done());
  const TimeNs fence = clock.ns;  // the time the tick applied it

  CHECK(!domain.admit("r", received_while_held));
  CHECK(!domain.admit("r", fence - 1));
  CHECK(domain.admit("r", fence));
  CHECK(domain.admit("r", fence + 1));

  // The next release moves the fence forward.
  clock.advance(50);
  engaged(domain, sink, {"r"});
  clock.advance(50);
  released(domain, sink, domain.instance_id(), {{"r", 3}});
  CHECK(!domain.admit("r", fence + 1));
  CHECK(!domain.admit("r", clock.ns - 1));
  CHECK(domain.admit("r", clock.ns));
}

TEST(a_release_queued_before_an_engage_cannot_open_the_gate) {
  FakeClock clock;
  Domain domain(clock.options());
  Sink sink;
  domain.add("r");
  engaged(domain, sink, {"r"});
  const std::string instance = domain.instance_id();

  // The release is waiting for the tick when the engage arrives: admission stays closed throughout, and the
  // machine applies them in the order they came, so the robot ends up held again.
  const Pending release = domain.release(instance, {{"r", 1}});
  const Pending engage = domain.engage({"r"});
  CHECK(!domain.admit("r", clock.ns));
  CHECK_EQ(engage.results()[0].outcome, Outcome::already_held);  // it was held when the engage arrived
  domain.tick(sink);
  REQUIRE(release.done());
  REQUIRE(engage.done());
  CHECK_EQ(release.results()[0].outcome, Outcome::released);
  CHECK_EQ(engage.results()[0].outcome, Outcome::engaged);  // after the release it was a new hold
  CHECK(!domain.admit("r", clock.ns));
  CHECK(get(domain, "r").held);
  CHECK_EQ(get(domain, "r").revision, 3);
}

TEST(a_release_that_the_tick_does_not_reach_in_time_is_not_applied) {
  FakeClock clock;
  Domain domain(clock.options());
  Sink sink;
  domain.add("r");
  engaged(domain, sink, {"r"});

  const Pending release = domain.release(domain.instance_id(), {{"r", 1}}, clock.ns + 5 * ms);
  clock.advance(10);  // the tick is late
  domain.tick(sink);
  REQUIRE(release.done());
  CHECK_EQ(release.results()[0].outcome, Outcome::expired);
  CHECK(get(domain, "r").held);
  CHECK_EQ(get(domain, "r").revision, 1);
  CHECK(!domain.admit("r", clock.ns));

  // On time it applies.
  const Pending on_time = domain.release(domain.instance_id(), {{"r", 1}}, clock.ns + 5 * ms);
  domain.tick(sink);
  CHECK_EQ(on_time.results()[0].outcome, Outcome::released);
}

TEST(tick_reasserts_zero_every_tick_while_held) {
  FakeClock clock;
  Domain domain(clock.options());
  domain.add("a");
  domain.add("b");
  domain.engage({"a"});
  Sink sink;

  TimeNs first_zero = 0;
  for (int i = 1; i <= 5; ++i) {
    clock.advance(10);
    const TickStats stats = domain.tick(sink);
    CHECK_EQ(stats.written, 1);
    CHECK_EQ(stats.failed, 0);
    CHECK_EQ(sink.writes("a"), i);
    const RobotState state = get(domain, "a");
    CHECK_EQ(state.stage, Stage::zero_written);
    if (i == 1)
      first_zero = *state.zero_written_at;
    CHECK_EQ(*state.zero_written_at, first_zero);  // the first write advanced the stage, later ones re-assert
  }
  CHECK_EQ(sink.writes("b"), 0);

  released(domain, sink, domain.instance_id(), {{"a", 1}});  // the tick applies the release before it writes
  const int after_release = sink.writes("a");
  const TickStats idle = domain.tick(sink);
  CHECK_EQ(idle.written, 0);
  CHECK_EQ(sink.writes("a"), after_release);

  clock.advance(10);
  engaged(domain, sink, {"a"});
  CHECK_EQ(get(domain, "a").stage, Stage::zero_written);
  CHECK(*get(domain, "a").zero_written_at > first_zero);
}

TEST(failed_zero_write_keeps_the_stage_gated) {
  FakeClock clock;
  Domain domain(clock.options());
  domain.add("a");
  domain.engage({"a"});
  const TimeNs gated_at = *get(domain, "a").gated_at;
  Sink sink;
  sink.fail("a", true);

  for (int i = 1; i <= 2; ++i) {
    clock.advance(10);
    const TickStats stats = domain.tick(sink);
    CHECK_EQ(stats.failed, 1);
    CHECK_EQ(stats.written, 0);
    const RobotState state = get(domain, "a");
    CHECK_EQ(state.stage, Stage::gated);
    CHECK(!state.zero_written_at.has_value());
    CHECK_EQ(state.zero_write_failed, i);
    CHECK_EQ(state.write_error, "device unplugged");
    CHECK(state.held);
  }

  sink.clear_failures();
  clock.advance(10);
  domain.tick(sink);
  RobotState state = get(domain, "a");
  CHECK_EQ(state.stage, Stage::zero_written);
  CHECK(state.write_error.empty());
  CHECK_EQ(state.zero_write_failed, 2);

  // A failure after the zero was confirmed takes the confirmation back; gated_at keeps the engage time.
  sink.fail("a", true);
  clock.advance(10);
  domain.tick(sink);
  state = get(domain, "a");
  CHECK_EQ(state.stage, Stage::gated);
  CHECK(!state.zero_written_at.has_value());
  CHECK_EQ(*state.gated_at, gated_at);
  CHECK_EQ(state.zero_write_failed, 3);

  // An exception from the sink is a failed write too.
  sink.clear_failures();
  sink.throws = true;
  const TickStats thrown = domain.tick(sink);
  CHECK_EQ(thrown.failed, 1);
  CHECK_EQ(get(domain, "a").write_error, "sink exploded");
  CHECK_EQ(get(domain, "a").stage, Stage::gated);
}

TEST(stopped_needs_the_dwell_at_rest) {
  FakeClock clock;
  Domain domain(clock.options());  // 0.02 m/s, 0.05 rad/s, 300 ms
  domain.add("r", true);
  Sink sink;
  engaged(domain, sink, {"r"});
  const TimeNs zero = *get(domain, "r").zero_written_at;

  CHECK(!domain.observe("r", 0, 0, zero - 1));  // taken before the zero was written: no evidence
  CHECK(domain.observe("r", 0.0, 0.0, zero));
  CHECK(domain.observe("r", 0.02, 0.05, zero + 100 * ms));  // the thresholds are inclusive
  CHECK(domain.observe("r", 0.01, -0.04, zero + 299 * ms));
  domain.tick(sink);
  CHECK_EQ(get(domain, "r").stage, Stage::zero_written);

  clock.advance(400);
  CHECK(domain.observe("r", 0.0, 0.0, zero + 300 * ms));
  domain.tick(sink);  // applies the report
  RobotState state = get(domain, "r");
  CHECK_EQ(state.stage, Stage::stopped);
  CHECK_EQ(*state.stopped_at, clock.ns.load());

  CHECK(!domain.observe("r", 0.0, 0.0, zero + 10 * ms));  // out of order

  // Motion takes the stopped stage back.
  CHECK(domain.observe("r", 0.5, 0.0, zero + 320 * ms));
  domain.tick(sink);
  state = get(domain, "r");
  CHECK_EQ(state.stage, Stage::zero_written);
  CHECK(!state.stopped_at.has_value());
  // A new run of rest needs the whole dwell again.
  CHECK(domain.observe("r", 0.0, 0.0, zero + 330 * ms));
  CHECK(domain.observe("r", 0.0, 0.0, zero + 629 * ms));
  domain.tick(sink);
  CHECK_EQ(get(domain, "r").stage, Stage::zero_written);
  CHECK(domain.observe("r", 0.0, 0.0, zero + 630 * ms));
  domain.tick(sink);
  CHECK_EQ(get(domain, "r").stage, Stage::stopped);
  // A rotation alone is motion; so is a sample that is not a number.
  CHECK(domain.observe("r", 0.0, 0.2, zero + 640 * ms));
  domain.tick(sink);
  CHECK_EQ(get(domain, "r").stage, Stage::zero_written);
  CHECK(domain.observe("r", 0.0, 0.0, zero + 650 * ms));
  CHECK(domain.observe("r", std::nan(""), 0.0, zero + 700 * ms));
  CHECK(domain.observe("r", 0.0, 0.0, zero + 900 * ms));
  domain.tick(sink);
  CHECK_EQ(get(domain, "r").stage, Stage::zero_written);
  CHECK(domain.observe("r", 0.0, 0.0, zero + 1200 * ms));
  domain.tick(sink);
  CHECK_EQ(get(domain, "r").stage, Stage::stopped);

  // Losing the zero (failed write) invalidates it until the next successful write.
  sink.fail("r", true);
  domain.tick(sink);
  CHECK_EQ(get(domain, "r").stage, Stage::gated);
  CHECK(!domain.observe("r", 0.0, 0.0, zero + 2000 * ms));
}

TEST(feedback_gaps_break_the_continuity_of_rest) {
  FakeClock clock;
  Domain domain(clock.options());
  domain.add("r", true);
  Sink sink;
  engaged(domain, sink, {"r"});
  const TimeNs zero = *get(domain, "r").zero_written_at;

  CHECK(domain.observe("r", 0, 0, zero));
  CHECK(domain.observe("r", 0, 0, zero + 200 * ms));
  // Nothing was heard for 400 ms: the rest before the gap does not count.
  CHECK(domain.observe("r", 0, 0, zero + 600 * ms));
  CHECK(domain.observe("r", 0, 0, zero + 800 * ms));
  domain.tick(sink);
  CHECK_EQ(get(domain, "r").stage, Stage::zero_written);
  CHECK(domain.observe("r", 0, 0, zero + 900 * ms));
  domain.tick(sink);
  CHECK_EQ(get(domain, "r").stage, Stage::stopped);
}

TEST(extreme_stamps_and_times_do_not_overflow) {
  FakeClock clock;
  clock.ns = std::numeric_limits<TimeNs>::min() + 5 * ms;
  Domain domain(clock.options());
  domain.add("r", true);
  Sink sink;
  engaged(domain, sink, {"r"});
  // Samples a whole int64 range apart: the differences must not overflow.
  CHECK(domain.observe("r", 0, 0, std::numeric_limits<TimeNs>::min() + 5 * ms));
  CHECK(domain.observe("r", 0, 0, std::numeric_limits<TimeNs>::max()));
  domain.tick(sink);
  CHECK_EQ(get(domain, "r").stage, Stage::zero_written);  // the gap broke the continuity of the rest
  CHECK(!domain.observe("r", 0, 0, std::numeric_limits<TimeNs>::min()));
  CHECK(domain.observe("r", 0, 0, std::numeric_limits<TimeNs>::max()));
  clock.ns = std::numeric_limits<TimeNs>::max();
  released(domain, sink, domain.instance_id(), {{"r", 1}});
  CHECK(domain.admit("r", std::numeric_limits<TimeNs>::max()));
  CHECK(!domain.admit("r", std::numeric_limits<TimeNs>::max() - 1));
}

TEST(hosts_without_feedback_never_report_stopped) {
  FakeClock clock;
  Domain domain(clock.options());
  domain.add("blind");  // no feedback
  domain.add("sighted", true);
  Sink sink;
  domain.engage_all();
  domain.tick(sink);
  const TimeNs zero = *get(domain, "blind").zero_written_at;

  for (int i = 0; i < 20; ++i)
    CHECK(!domain.observe("blind", 0, 0, zero + i * 100 * ms));
  domain.tick(sink);
  CHECK_EQ(get(domain, "blind").stage, Stage::zero_written);

  // Unknown robots and robots that are not held ignore feedback.
  CHECK(!domain.observe("nobody", 0, 0, zero));
  released(domain, sink, domain.instance_id(), {{"sighted", 1}});
  CHECK(!domain.observe("sighted", 0, 0, zero + 1000 * ms));
  CHECK_EQ(get(domain, "sighted").stage, Stage::released);

  const Description description = domain.describe();
  REQUIRE(description.robots.size() == 2);
  CHECK_EQ(description.robots[0].robot_id, "blind");
  CHECK(!description.robots[0].feedback);
  CHECK(description.robots[1].feedback);
}

TEST(boot_held_starts_every_new_robot_held) {
  FakeClock clock;
  DomainOptions options = clock.options();
  options.boot_held = true;
  Domain domain(options);
  domain.add("a");
  domain.add("b", true);

  // Held from the moment they are in the roster, before any tick has run.
  for (const char *id : {"a", "b"}) {
    const RobotState state = get(domain, id);
    CHECK(state.held);
    CHECK_EQ(state.revision, 1);
    CHECK_EQ(state.stage, Stage::gated);
    CHECK_EQ(*state.gated_at, clock.ns.load());
    CHECK(!domain.admit(id, clock.ns));
  }
  Sink sink;
  domain.tick(sink);
  CHECK_EQ(get(domain, "a").stage, Stage::zero_written);
  CHECK_EQ(get(domain, "a").revision, 1);
  CHECK_EQ(sink.writes("a"), 1);
  CHECK_EQ(sink.writes("b"), 1);

  // Released through the normal CAS; boot_held does not hold a robot again when it is re-added.
  CHECK_EQ(released(domain, sink, domain.instance_id(), {{"a", 1}})[0].outcome, Outcome::released);
  CHECK(domain.remove("a"));
  CHECK(domain.add("a"));
  domain.tick(sink);
  CHECK(!get(domain, "a").held);
  CHECK_EQ(get(domain, "a").revision, 2);
}

TEST(roster_removal_keeps_state_and_readding_inherits_hold) {
  FakeClock clock;
  Domain domain(clock.options());
  Sink sink;
  CHECK(domain.add("a", true));
  CHECK(!domain.add("a"));  // already in the roster
  engaged(domain, sink, {"a"});
  CHECK_EQ(get(domain, "a").stage, Stage::zero_written);

  CHECK(domain.remove("a"));
  CHECK(!domain.remove("a"));
  // A removed robot is not addressable and its device is not written.
  const Snapshot snapshot = domain.state({"a"});
  CHECK(snapshot.robots.empty());
  REQUIRE(snapshot.unknown.size() == 1);
  CHECK(domain.state().robots.empty());
  CHECK(domain.describe().robots.empty());
  const Pending engage = domain.engage({"a"});
  CHECK(engage.done());
  CHECK_EQ(engage.results()[0].outcome, Outcome::unknown_robot);
  const Pending release = domain.release(domain.instance_id(), {{"a", 1}});
  CHECK(release.done());
  CHECK_EQ(release.results()[0].outcome, Outcome::unknown_robot);
  CHECK(!domain.admit("a", clock.ns));
  const int writes = sink.writes("a");
  CHECK_EQ(domain.tick(sink).written, 0);
  CHECK_EQ(sink.writes("a"), writes);

  // Re-created model: starts held, same revision, the new device has no zero yet.
  clock.advance(500);
  CHECK(domain.add("a", false));
  const RobotState state = get(domain, "a");
  CHECK(state.held);
  CHECK_EQ(state.revision, 1);
  CHECK_EQ(state.stage, Stage::gated);
  CHECK_EQ(*state.gated_at, clock.ns.load());
  CHECK(!state.zero_written_at.has_value());
  CHECK(!state.feedback);
  CHECK(!domain.admit("a", clock.ns));
  domain.tick(sink);
  CHECK_EQ(get(domain, "a").stage, Stage::zero_written);

  // A released robot keeps its release fence across removal.
  CHECK_EQ(released(domain, sink, domain.instance_id(), {{"a", 1}})[0].outcome, Outcome::released);
  const TimeNs fence = clock.ns;
  clock.advance(100);
  domain.remove("a");
  domain.add("a");
  domain.tick(sink);
  CHECK(!get(domain, "a").held);
  CHECK(!domain.admit("a", fence - 1));
  CHECK(domain.admit("a", fence));
}

TEST(roster_is_validated_and_bounded) {
  FakeClock clock;
  DomainOptions options = clock.options();
  options.max_robots = 2;
  Domain domain(options);
  bool threw = false;
  for (const std::string &bad : {std::string(), std::string("has space"), std::string("slash/id"),
                                 std::string(129, 'x'), std::string("ünï")}) {
    threw = false;
    try {
      domain.add(bad);
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    CHECK(threw);
  }
  CHECK(valid_robot_id("ugv-1.a_b:c"));
  CHECK(valid_robot_id(std::string(128, 'x')));
  CHECK(domain.add("a"));
  CHECK(domain.add("b"));
  threw = false;
  try {
    domain.add("c");
  } catch (const std::length_error &) {
    threw = true;
  }
  CHECK(threw);
  // Re-adding a removed id does not need a new slot.
  CHECK(domain.remove("a"));
  CHECK(domain.add("a"));
}

TEST(host_fault_holds_the_robot_and_blocks_release) {
  FakeClock clock;
  Domain domain(clock.options());
  Sink sink;
  domain.add("a");
  domain.add("b");
  const std::string instance = domain.instance_id();

  CHECK(!domain.set_fault("nobody", "x"));
  CHECK(domain.set_fault("a", "serial port lost"));
  CHECK(!domain.admit("a", clock.ns));  // a fault holds a robot that was released, at once
  domain.tick(sink);
  RobotState state = get(domain, "a");
  CHECK(state.held);
  CHECK_EQ(state.revision, 1);
  CHECK_EQ(state.fault, "serial port lost");
  CHECK_EQ(state.stage, Stage::gated);  // the zero of a faulted robot is not trusted
  domain.tick(sink);
  CHECK_EQ(get(domain, "a").stage, Stage::gated);
  CHECK(sink.writes("a") > 0);  // but the device is still written

  const auto refused = released(domain, sink, instance, {{"a", 1}});
  CHECK_EQ(refused[0].outcome, Outcome::conflict);
  CHECK(refused[0].state->held);
  CHECK_EQ(refused[0].state->fault, "serial port lost");

  CHECK(domain.set_fault("a", ""));
  domain.tick(sink);
  CHECK(get(domain, "a").fault.empty());
  CHECK_EQ(get(domain, "a").stage, Stage::zero_written);  // the tick wrote a fresh zero
  CHECK_EQ(released(domain, sink, instance, {{"a", 1}})[0].outcome, Outcome::released);

  // A fault on a robot that is already held does not change its revision. Text is printable ASCII, bounded.
  engaged(domain, sink, {"b"});
  domain.set_fault("b", std::string("ab\xff\n") + std::string(500, 'z'));
  domain.tick(sink);
  state = get(domain, "b");
  CHECK_EQ(state.revision, 1);
  CHECK_EQ(state.fault.size(), 160);
  CHECK_EQ(state.fault.substr(0, 6), "ab??zz");

  // The fault belongs to the device: a re-created model starts without it, still held.
  domain.remove("b");
  domain.add("b");
  CHECK(get(domain, "b").held);
  CHECK(get(domain, "b").fault.empty());
  domain.tick(sink);
  CHECK(get(domain, "b").fault.empty());
}

TEST(a_fault_queued_behind_a_release_still_holds_the_robot) {
  FakeClock clock;
  Domain domain(clock.options());
  Sink sink;
  domain.add("a");
  engaged(domain, sink, {"a"});

  const Pending release = domain.release(domain.instance_id(), {{"a", 1}});
  CHECK(domain.set_fault("a", "encoder lost"));
  CHECK(!domain.admit("a", clock.ns));
  domain.tick(sink);
  CHECK_EQ(release.results()[0].outcome, Outcome::released);  // it was first in line
  const RobotState state = get(domain, "a");
  CHECK(state.held);  // and the fault held the robot again
  CHECK_EQ(state.fault, "encoder lost");
  CHECK_EQ(state.revision, 3);
  CHECK(!domain.admit("a", clock.ns));
}

TEST(tick_results_of_a_finished_hold_are_discarded) {
  FakeClock clock;
  Domain domain(clock.options());
  domain.add("a");
  const std::string instance = domain.instance_id();
  Sink sink;

  // Released while the device is being written: the robot must not end up zero_written.
  domain.engage({"a"});
  sink.during = [&] { domain.release(instance, {{"a", 1}}); };
  domain.tick(sink);
  RobotState state = get(domain, "a");
  CHECK(!state.held);
  CHECK_EQ(state.stage, Stage::released);
  CHECK(!state.zero_written_at.has_value());

  // Removed and re-added while the old device is being written: the new device still has no zero.
  sink.during = nullptr;
  engaged(domain, sink, {"a"});
  sink.during = [&] {
    domain.remove("a");
    domain.add("a");
  };
  domain.tick(sink);
  state = get(domain, "a");
  CHECK(state.held);
  CHECK_EQ(state.stage, Stage::gated);
  sink.during = nullptr;
  domain.tick(sink);
  CHECK_EQ(get(domain, "a").stage, Stage::zero_written);
}

TEST(listeners_hear_about_progress) {
  FakeClock clock;
  Domain domain(clock.options());
  domain.add("a", true);
  std::atomic<int> calls{0};
  const std::uint64_t token = domain.add_listener([&] { ++calls; });
  Sink sink;

  domain.engage({"a"});
  CHECK_EQ(calls.load(), 0);  // nothing was applied yet
  domain.tick(sink);
  CHECK_EQ(calls.load(), 1);  // the engage and the zero were
  domain.tick(sink);
  CHECK_EQ(calls.load(), 1);  // a re-assert is not progress
  const TimeNs rest = clock.ns;
  domain.observe("a", 0, 0, rest);
  domain.observe("a", 0, 0, rest + 200 * ms);
  domain.observe("a", 0, 0, rest + 400 * ms);
  CHECK_EQ(calls.load(), 1);  // reports wait for the tick as well
  domain.tick(sink);
  CHECK_EQ(calls.load(), 2);  // stopped
  domain.set_fault("a", "broken");
  domain.tick(sink);
  CHECK_EQ(calls.load(), 3);
  domain.set_fault("a", "");
  domain.tick(sink);
  CHECK_EQ(calls.load(), 4);
  domain.release(domain.instance_id(), {{"a", 1}});
  domain.tick(sink);
  CHECK_EQ(calls.load(), 5);
  domain.remove("a");
  CHECK_EQ(calls.load(), 6);

  domain.remove_listener(token);
  domain.add("a");
  domain.engage({"a"});
  domain.tick(sink);
  domain.remove("a");
  CHECK_EQ(calls.load(), 6);
}

TEST(the_tick_belongs_to_one_thread) {
  Domain domain;
  domain.add("a");
  Sink sink;
  domain.tick(sink);  // this thread owns the machines from now on
  bool refused = false;
  std::thread other([&] {
    try {
      domain.tick(sink);
    } catch (const std::logic_error &) {
      refused = true;
    }
  });
  other.join();
  CHECK(refused);
  domain.tick(sink);  // the owner still can
}

TEST(a_flood_without_a_tick_fails_closed) {
  FakeClock clock;
  Domain domain(clock.options());
  domain.add("a");
  // Nobody ticks. Every engage is an event of the machine; when its queue is full the machine cannot follow any
  // more, and the robot stays held for good rather than risk letting a release through.
  for (int i = 0; i < 6000; ++i)
    domain.engage({"a"});
  CHECK(!domain.admit("a", clock.ns));
  const RobotState state = get(domain, "a");
  CHECK(state.held);
  CHECK(state.fault.find("hold state machine failed") != std::string::npos);

  Sink sink;
  domain.tick(sink);
  const Pending release = domain.release(domain.instance_id(), {{"a", 1}});
  domain.tick(sink);
  CHECK(release.done() ? release.results()[0].outcome != Outcome::released : true);
  CHECK(get(domain, "a").held);
  CHECK(!domain.admit("a", clock.ns));
}

TEST(options_and_identity) {
  const std::string chosen = "0123456789abcdef0123456789abcdef";
  DomainOptions options;
  options.instance_id = chosen;
  Domain given(options);
  CHECK_EQ(given.instance_id(), chosen);

  Domain first;
  Domain second;
  CHECK(valid_instance_id(first.instance_id()));
  CHECK(first.instance_id() != second.instance_id());
  CHECK(!valid_instance_id(""));
  CHECK(!valid_instance_id(std::string(32, 'A')));
  CHECK(!valid_instance_id(std::string(31, 'a')));

  const TimeNs a = first.now();
  const TimeNs b = first.now();
  CHECK(b >= a);  // the default clock is steady

  const auto throws = [](DomainOptions bad) {
    try {
      Domain domain(bad);
    } catch (const std::invalid_argument &) {
      return true;
    }
    return false;
  };
  DomainOptions bad_instance_id;
  bad_instance_id.instance_id = "not-hex";
  CHECK(throws(bad_instance_id));
  DomainOptions zero_instance_id;
  zero_instance_id.instance_id = std::string(32, '0');
  CHECK(throws(zero_instance_id));
  DomainOptions negative;
  negative.linear_eps = -1;
  CHECK(throws(negative));
  DomainOptions nan_eps;
  nan_eps.angular_eps = std::nan("");
  CHECK(throws(nan_eps));
  DomainOptions negative_dwell;
  negative_dwell.stopped_dwell = std::chrono::milliseconds(-1);
  CHECK(throws(negative_dwell));
  DomainOptions endless_dwell;
  endless_dwell.stopped_dwell = std::chrono::hours(2);
  CHECK(throws(endless_dwell));
}

TEST(state_reports_requested_order_and_unknown_ids) {
  FakeClock clock;
  Domain domain(clock.options());
  domain.add("b");
  domain.add("a");
  const Snapshot all = domain.state();
  REQUIRE(all.robots.size() == 2);
  CHECK_EQ(all.robots[0].robot_id, "a");  // sorted
  CHECK_EQ(all.start, clock.ns.load());
  CHECK_EQ(all.instance, domain.instance_id());

  const Snapshot some = domain.state({"b", "x", "a", "y"});
  REQUIRE(some.robots.size() == 2);
  CHECK_EQ(some.robots[0].robot_id, "b");
  CHECK_EQ(some.robots[1].robot_id, "a");
  REQUIRE(some.unknown.size() == 2);
  CHECK_EQ(some.unknown[0], "x");
  CHECK_EQ(some.unknown[1], "y");
}

TEST_MAIN()
