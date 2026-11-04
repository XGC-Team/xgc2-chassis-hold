// Deliberately stalled injected clock / device: separates ingress from owner work.
#include <xgc2/chassis_hold/domain.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace xgc2::chassis_hold;
using Clock = std::chrono::steady_clock;
namespace {
struct Sink : ZeroSink {
  std::function<void()> during;
  bool write_zero(const std::string &, std::string &) override {
    if (during) during();
    return true;
  }
};
long long sample(bool stall_clock) {
  std::promise<void> entered, resume, first_tick, next_tick;
  auto resumed = resume.get_future().share();
  auto next = next_tick.get_future();
  std::atomic<bool> armed{false};
  std::thread::id owner;
  DomainOptions options;
  options.instance_id = "0123456789abcdef0123456789abcdef";
  options.clock = [&] {
    if (stall_clock && armed.load() && std::this_thread::get_id() == owner) {
      armed = false;
      entered.set_value();
      resumed.wait();
    }
    return TimeNs{1000000000};
  };
  Domain domain(options);
  domain.add("r");
  Sink sink;
  domain.engage({"r"});
  std::thread output([&] {
    owner = std::this_thread::get_id();
    domain.tick(sink);
    first_tick.set_value();
    next.wait();
    domain.tick(sink);
  });
  first_tick.get_future().wait();
  if (stall_clock) domain.release(domain.instance_id(), {{"r", 1}});
  else sink.during = [&] { entered.set_value(); resumed.wait(); };
  armed = true;
  next_tick.set_value();
  entered.get_future().wait();
  std::promise<void> started;
  long long elapsed = 0;
  std::thread ingress([&] {
    started.set_value();
    const auto begin = Clock::now();
    domain.state();
    domain.admit("r", 1000000000);
    domain.engage({"r"});
    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - begin).count();
  });
  started.get_future().wait();
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  resume.set_value();
  ingress.join();
  output.join();
  return elapsed;
}
}  // namespace
int main() {
  for (bool clock : {false, true}) {
    std::vector<long long> values;
    for (int i = 0; i < 20; ++i) values.push_back(sample(clock));
    std::sort(values.begin(), values.end());
    std::cout << (clock ? "stalled_owner_clock" : "stalled_device")
              << " n=20 stall_ms=25 ingress_ns_p50=" << values[10]
              << " p95=" << values[19] << " max=" << values.back() << '\n';
  }
}
