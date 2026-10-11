#pragma once
// Test doubles shared by the test programs.
#include <xgc2/chassis_hold/domain.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

namespace fixtures {

using xgc2::chassis_hold::DomainOptions;
using xgc2::chassis_hold::TimeNs;

constexpr TimeNs ms = 1000000;

// Polls until the condition holds or the timeout ends; false on timeout.
inline bool wait_until(const std::function<bool()> &condition,
                       std::chrono::milliseconds timeout = std::chrono::seconds(10)) {
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (!condition()) {
    if (std::chrono::steady_clock::now() > end)
      return false;
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  return true;
}

// A domain clock the test advances by hand.
struct FakeClock {
  std::atomic<TimeNs> ns{1000 * ms};
  void advance(int milliseconds) { ns += milliseconds * ms; }
  DomainOptions options() {
    DomainOptions options;
    options.clock = [this] { return ns.load(); };
    return options;
  }
};

// Device output that records the zero writes it receives; thread safe.
struct Sink : xgc2::chassis_hold::ZeroSink {
  int writes(const std::string &robot_id) {
    std::lock_guard<std::mutex> lock(mutex);
    return counts[robot_id];
  }
  void fail(const std::string &robot_id, bool failing) {
    std::lock_guard<std::mutex> lock(mutex);
    if (failing)
      failing_ids.insert(robot_id);
    else
      failing_ids.erase(robot_id);
  }
  void clear_failures() {
    std::lock_guard<std::mutex> lock(mutex);
    failing_ids.clear();
  }

  std::atomic<bool> throws{false};
  std::function<void()> during;  // runs inside every write, without any lock of the sink or the domain

  bool write_zero(const std::string &robot_id, std::string &error) override {
    bool failing;
    {
      std::lock_guard<std::mutex> lock(mutex);
      ++counts[robot_id];
      failing = failing_ids.count(robot_id) != 0;
    }
    if (during)
      during();
    if (throws)
      throw std::runtime_error("sink exploded");
    if (failing) {
      error = "device unplugged";
      return false;
    }
    return true;
  }

private:
  std::mutex mutex;
  std::map<std::string, int> counts;
  std::set<std::string> failing_ids;
};

// The host's native thread: ticks the domain until stopped.
class TickThread {
public:
  TickThread(xgc2::chassis_hold::Domain &domain, Sink &sink, std::function<void()> extra = nullptr)
      : thread_([this, &domain, &sink, extra] {
          while (running_) {
            domain.tick(sink);
            if (extra)
              extra();
            std::this_thread::sleep_for(std::chrono::microseconds(150));
          }
        }) {}
  ~TickThread() { stop(); }
  void stop() {
    running_ = false;
    if (thread_.joinable())
      thread_.join();
  }

private:
  std::atomic<bool> running_{true};
  std::thread thread_;
};

}  // namespace fixtures
