// Tick work must stay proportional to held robots, with no steady-state heap allocation.
#include "check.hpp"
#include "fixtures.hpp"

#include <xgc2/chassis_hold/chassis_hold.hpp>

#include <atomic>
#include <cstdlib>
#include <new>
#include <thread>

// The replacement operators below pair new with malloc and delete with free on purpose; GCC 13 inlines them in
// sanitizer builds and reports the pair as a mismatch.
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ >= 11
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif

namespace allocation {
std::atomic<bool> counting{false};
std::atomic<std::size_t> calls{0};

void record() {
  if (counting.load(std::memory_order_relaxed))
    calls.fetch_add(1, std::memory_order_relaxed);
}
}  // namespace allocation

void *operator new(std::size_t size) {
  allocation::record();
  if (void *pointer = std::malloc(size ? size : 1))
    return pointer;
  throw std::bad_alloc();
}
void *operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void *pointer) noexcept { std::free(pointer); }
void operator delete[](void *pointer) noexcept { std::free(pointer); }
void operator delete(void *pointer, std::size_t) noexcept { std::free(pointer); }
void operator delete[](void *pointer, std::size_t) noexcept { std::free(pointer); }

// Every form must pair with malloc and free, or a sanitizer that supplies its own nothrow variants (the state
// machine library allocates with them) reports new/free mismatches.
void *operator new(std::size_t size, const std::nothrow_t &) noexcept {
  allocation::record();
  return std::malloc(size ? size : 1);
}
void *operator new[](std::size_t size, const std::nothrow_t &) noexcept { return ::operator new(size, std::nothrow); }
void operator delete(void *pointer, const std::nothrow_t &) noexcept { std::free(pointer); }
void operator delete[](void *pointer, const std::nothrow_t &) noexcept { std::free(pointer); }

void *operator new(std::size_t size, std::align_val_t alignment) {
  allocation::record();
  void *pointer = nullptr;
  if (posix_memalign(&pointer, static_cast<std::size_t>(alignment), size ? size : 1) == 0)
    return pointer;
  throw std::bad_alloc();
}
void *operator new[](std::size_t size, std::align_val_t alignment) { return ::operator new(size, alignment); }
void *operator new(std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept {
  allocation::record();
  void *pointer = nullptr;
  return posix_memalign(&pointer, static_cast<std::size_t>(alignment), size ? size : 1) == 0 ? pointer : nullptr;
}
void *operator new[](std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept {
  return ::operator new(size, alignment, std::nothrow);
}
void operator delete(void *pointer, std::align_val_t, const std::nothrow_t &) noexcept { std::free(pointer); }
void operator delete[](void *pointer, std::align_val_t, const std::nothrow_t &) noexcept { std::free(pointer); }
void operator delete(void *pointer, std::align_val_t) noexcept { std::free(pointer); }
void operator delete[](void *pointer, std::align_val_t) noexcept { std::free(pointer); }
void operator delete(void *pointer, std::size_t, std::align_val_t) noexcept { std::free(pointer); }
void operator delete[](void *pointer, std::size_t, std::align_val_t) noexcept { std::free(pointer); }

using namespace xgc2::chassis_hold;
using fixtures::FakeClock;
using fixtures::Sink;
using fixtures::ms;

namespace {

struct Output : ZeroSink {
  std::size_t writes = 0;
  bool write_zero(const std::string &, std::string &) override {
    ++writes;
    return true;
  }
};

struct Samples : FeedbackSource {
  Feedback sample;
  bool available = true;
  std::size_t asked = 0;
  bool latest(const std::string &, Feedback &out) override {
    ++asked;
    out = sample;
    return available;
  }
};

RobotState get(const Domain &domain, const std::string &id) {
  const Snapshot snapshot = domain.state({id});
  REQUIRE(snapshot.robots.size() == 1);
  return snapshot.robots[0];
}

void steady_ticks(int robots, bool facade) {
  FakeClock clock;
  Output output;
  Samples samples;
  HoldOptions options;
  options.domain = clock.options();
  options.feedback = &samples;
  // Long ids expose accidental string copies as well as vector/payload allocation.
  for (int i = 0; i < robots; ++i)
    options.robots.push_back(std::string(40, 'r') + std::to_string(i));
  ChassisHold hold(options, output);
  const std::string id = options.robots.front();
  const auto tick = [&] { return facade ? hold.tick() : hold.domain().tick(output); };
  const auto measure = [&](bool held) {
    for (int i = 0; i < 100; ++i)
      tick();
    const std::size_t writes = output.writes;
    const std::size_t asked = samples.asked;
    std::size_t written = 0, failed = 0;
    allocation::calls = 0;
    allocation::counting = true;
    for (int i = 0; i < 10000; ++i) {
      const TickStats stats = tick();
      written += stats.written;
      failed += stats.failed;
    }
    allocation::counting = false;
    CHECK_EQ(allocation::calls.load(), 0);
    CHECK_EQ(written, held ? 10000 : 0);
    CHECK_EQ(failed, 0);
    CHECK_EQ(output.writes - writes, held ? 10000 : 0);
    CHECK_EQ(samples.asked - asked, held && facade ? 10000 : 0);
  };

  measure(false);
  hold.domain().engage({id});
  samples.sample.stamp = clock.ns;
  hold.tick();
  for (int i = 0; i < 3; ++i) {
    clock.advance(100);
    samples.sample.stamp = clock.ns;
    hold.tick();
  }
  hold.tick();  // applies the feedback report before the allocation window
  REQUIRE(get(hold.domain(), id).stage == Stage::stopped);
  measure(true);
  CHECK_EQ(get(hold.domain(), id).stage, Stage::stopped);

  const Pending release = hold.domain().release(hold.instance_id(), {{id, 1}});
  tick();
  REQUIRE(release.done());
  CHECK_EQ(release.results()[0].outcome, Outcome::released);
  measure(false);  // removing the last held robot restores the idle path
}

}  // namespace

TEST(the_global_allocation_counter_observes_scalar_array_and_aligned_new) {
  allocation::calls = 0;
  allocation::counting = true;
  void *scalar = ::operator new(1);
  void *array = ::operator new[](2);
  void *aligned = ::operator new(64, std::align_val_t(64));
  allocation::counting = false;
  ::operator delete(scalar);
  ::operator delete[](array);
  ::operator delete(aligned, std::align_val_t(64));
  CHECK_EQ(allocation::calls.load(), 3);
}

TEST(domain_ticks_allocate_nothing_with_twenty_or_one_hundred_robots) {
  for (const int robots : {20, 100})
    steady_ticks(robots, false);
}

TEST(facade_ticks_allocate_nothing_with_twenty_or_one_hundred_robots) {
  for (const int robots : {20, 100})
    steady_ticks(robots, true);
}

TEST(an_idle_roster_applies_releases_and_fault_events_on_the_next_tick) {
  FakeClock clock;
  Domain domain(clock.options());
  Sink sink;
  for (int i = 0; i < 100; ++i)
    domain.add("robot_" + std::to_string(i));
  domain.tick(sink);
  const Pending idle_release = domain.release(domain.instance_id(), {{"robot_0", 0}});
  CHECK(!idle_release.done());
  CHECK_EQ(domain.tick(sink).written, 0);
  REQUIRE(idle_release.done());
  CHECK_EQ(idle_release.results()[0].outcome, Outcome::not_held);

  CHECK(domain.set_fault("robot_0", "encoder lost"));
  CHECK(!domain.admit("robot_0", clock.ns));
  CHECK_EQ(domain.tick(sink).written, 1);
  CHECK_EQ(get(domain, "robot_0").fault, "encoder lost");
  CHECK_EQ(get(domain, "robot_0").stage, Stage::gated);
  CHECK(domain.set_fault("robot_0", ""));
  CHECK_EQ(domain.tick(sink).written, 1);
  CHECK(get(domain, "robot_0").fault.empty());
  CHECK_EQ(get(domain, "robot_0").stage, Stage::zero_written);

  const Pending release = domain.release(domain.instance_id(), {{"robot_0", 1}});
  clock.advance(10);
  CHECK(!release.done());
  CHECK_EQ(domain.tick(sink).written, 0);
  REQUIRE(release.done());
  CHECK_EQ(release.results()[0].outcome, Outcome::released);
  CHECK_EQ(get(domain, "robot_0").revision, 2);
  CHECK(!domain.admit("robot_0", clock.ns - 1));
  CHECK(domain.admit("robot_0", clock.ns));
  CHECK(domain.set_fault("robot_0", ""));  // a no-op event also has to be drained
  CHECK_EQ(domain.tick(sink).written, 0);
}

TEST(late_facade_feedback_starts_its_own_dwell_and_gaps_reset_continuity) {
  FakeClock clock;
  Output output;
  Samples samples;
  samples.available = false;
  HoldOptions options;
  options.domain = clock.options();
  options.robots = {"a", "idle"};
  options.feedback = &samples;
  ChassisHold hold(options, output);
  hold.tick();
  CHECK_EQ(samples.asked, 0);
  hold.domain().engage({"a"});
  hold.tick();
  const TimeNs zero = *get(hold.domain(), "a").zero_written_at;
  clock.advance(1000);
  hold.tick();
  CHECK_EQ(get(hold.domain(), "a").stage, Stage::zero_written);
  samples.available = true;
  samples.sample.stamp = zero - 1;
  hold.tick();  // evidence older than the zero cannot count
  samples.sample.stamp = clock.ns;
  hold.tick();  // the first late sample starts the dwell
  clock.advance(100);
  samples.sample.stamp = clock.ns;
  hold.tick();
  CHECK_EQ(get(hold.domain(), "a").stage, Stage::zero_written);
  clock.advance(401);
  samples.sample.stamp = clock.ns;
  hold.tick();  // the gap invalidates the earlier rest
  clock.advance(299);
  samples.sample.stamp = clock.ns;
  hold.tick();
  hold.tick();
  CHECK_EQ(get(hold.domain(), "a").stage, Stage::zero_written);
  clock.advance(1);
  samples.sample.stamp = clock.ns;
  hold.tick();
  CHECK_EQ(get(hold.domain(), "a").stage, Stage::zero_written);  // report waits for the tick
  hold.tick();
  CHECK_EQ(get(hold.domain(), "a").stage, Stage::stopped);
  samples.sample.linear = 0.5;
  samples.sample.stamp = ++clock.ns;
  hold.tick();
  hold.tick();
  CHECK_EQ(get(hold.domain(), "a").stage, Stage::zero_written);
}

TEST(idle_roster_changes_preserve_pending_work_across_removal) {
  FakeClock clock;
  Domain domain(clock.options());
  Sink sink;
  domain.tick(sink);
  CHECK(domain.add("a"));
  CHECK(domain.add("b"));
  CHECK_EQ(domain.tick(sink).written, 0);
  CHECK(domain.remove("b"));
  CHECK(domain.add("c"));
  CHECK_EQ(domain.tick(sink).written, 0);
  REQUIRE(domain.state().robots.size() == 2);
  CHECK_EQ(domain.state().robots[1].robot_id, "c");
  CHECK(!domain.admit("b", clock.ns));

  const Pending release = domain.release(domain.instance_id(), {{"a", 0}});
  CHECK(domain.remove("a"));
  domain.tick(sink);
  CHECK(!release.done());
  CHECK(domain.add("a"));
  domain.tick(sink);
  REQUIRE(release.done());
  CHECK_EQ(release.results()[0].outcome, Outcome::not_held);

  const Pending engage = domain.engage({"a"});
  CHECK(domain.remove("a"));
  CHECK_EQ(domain.tick(sink).written, 0);
  CHECK(!engage.done());
  CHECK(domain.add("a"));
  CHECK_EQ(domain.tick(sink).written, 1);
  CHECK(engage.done());
  CHECK_EQ(get(domain, "a").revision, 1);
}

TEST(an_idle_tick_still_belongs_to_one_thread_before_any_robot_is_added) {
  Domain domain;
  Sink sink;
  domain.tick(sink);
  bool refused = false;
  std::thread other([&] {
    try {
      domain.tick(sink);
    } catch (const std::logic_error &error) {
      refused = std::string(error.what()) == "Domain::tick must always run on the same thread";
    }
  });
  other.join();
  CHECK(refused);
  domain.add("a");
  CHECK_EQ(domain.tick(sink).written, 0);
}

TEST_MAIN()
