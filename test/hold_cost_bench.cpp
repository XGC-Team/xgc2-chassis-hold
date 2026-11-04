// Opt-in matched-load public API measurements; no production instrumentation.
#include <xgc2/chassis_hold/domain.hpp>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <sys/resource.h>
#include <vector>

using namespace xgc2::chassis_hold;
using Clock = std::chrono::steady_clock;
namespace {
struct Sink : ZeroSink {
  std::size_t writes = 0;
  bool write_zero(const std::string &, std::string &) override { ++writes; return true; }
};
long long cpu_ns() {
  timespec value{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value);
  return value.tv_sec * 1000000000LL + value.tv_nsec;
}
void run(const std::string &mode, int count) {
  TimeNs now = 1000000000;
  unsigned wakes = 0;
  DomainOptions options;
  options.instance_id = "0123456789abcdef0123456789abcdef";
  options.clock = [&] { return now; };
  options.wake = [&] { ++wakes; };
  Domain domain(options);
  Sink sink;
  for (int i = 0; i < 100; ++i) domain.add("r" + std::to_string(i), true);
  if (mode != "idle" && mode != "cycle") domain.engage({"r0"});
  domain.tick(sink);
  std::vector<long long> elapsed;
  elapsed.reserve(count);
  const auto initial_writes = sink.writes;
  const auto initial_wakes = wakes;
  const auto cpu_start = cpu_ns();
  rusage before{}, after{};
  getrusage(RUSAGE_SELF, &before);
  const auto start = Clock::now();
  for (int i = 0; i < count; ++i) {
    now += 1000000;
    const auto t = Clock::now();
    if (mode == "cycle") {
      domain.engage({"r0"});
      domain.tick(sink);
      domain.release(domain.instance_id(), {{"r0", static_cast<std::uint64_t>(2 * i + 1)}});
    } else if (mode == "feedback") {
      domain.observe("r0", (i % 400 == 0) ? 0.5 : 0, 0, now);
    }
    domain.tick(sink);
    elapsed.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t).count());
  }
  const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count();
  getrusage(RUSAGE_SELF, &after);
  const auto cpu = cpu_ns() - cpu_start;
  std::sort(elapsed.begin(), elapsed.end());
  std::cout << mode << " n=" << count << " cpu_ns/op=" << cpu / count << " wall_ns/op=" << wall / count
            << " p50=" << elapsed[count / 2] << " p95=" << elapsed[count * 95 / 100]
            << " max=" << elapsed.back() << " writes=" << sink.writes - initial_writes
            << " wakes=" << wakes - initial_wakes << " voluntary_cs=" << after.ru_nvcsw - before.ru_nvcsw
            << " involuntary_cs=" << after.ru_nivcsw - before.ru_nivcsw << '\n';
}
}  // namespace
int main() {
  for (int repetition = 0; repetition < 5; ++repetition)
    for (const char *mode : {"idle", "held", "feedback", "cycle"}) run(mode, 100000);
}
