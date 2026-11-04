// Recorded against 10c10a9 before the ADT cutover. Uses only the installed public API.
#include <xgc2/chassis_hold/domain.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace xgc2::chassis_hold;

namespace {
struct Sink : ZeroSink {
  bool fail = false;
  std::size_t writes = 0;
  bool write_zero(const std::string &, std::string &error) override {
    ++writes;
    if (fail) error = "disconnected";
    return !fail;
  }
};

std::string trace() {
  TimeNs now = 1000000000;
  unsigned wakes = 0, notices = 0;
  DomainOptions options;
  options.instance_id = "0123456789abcdef0123456789abcdef";
  options.clock = [&] { return now; };
  options.wake = [&] { ++wakes; };
  Domain domain(options);
  Sink sink;
  domain.add("a", true);
  domain.add("b", false);
  domain.add_listener([&] { ++notices; });
  std::ostringstream out;
  const auto stamp = [&](const std::optional<TimeNs> &value) {
    if (value) out << *value;
    else out << '-';
  };
  const auto state = [&] {
    for (const auto &s : domain.state().robots) {
      out << ' ' << s.robot_id << ':' << s.held << ',' << s.revision << ',' << to_string(s.stage) << ',';
      stamp(s.gated_at); out << ','; stamp(s.zero_written_at); out << ','; stamp(s.stopped_at);
      out << ',' << s.fault << ',' << s.write_error << ',' << s.zero_write_failed << ',' << s.feedback
          << ',' << domain.admit(s.robot_id, now - 1) << domain.admit(s.robot_id, now);
    }
    out << " w=" << wakes << " n=" << notices << " z=" << sink.writes << '\n';
  };
  const auto result = [&](const Pending &p) {
    out << " done=" << p.done();
    for (const auto &r : p.results()) out << ' ' << r.robot_id << '=' << to_string(r.outcome);
  };
  Pending pending;
  std::uint32_t seed = 0x59f13aU;
  for (unsigned i = 0; i < 2000; ++i) {
    seed = seed * 1664525U + 1013904223U;
    now += static_cast<TimeNs>((seed >> 16) % 401) * 1000000;
    const std::string id = (seed & 1) ? "a" : "b";
    const unsigned action = (seed >> 8) % 12;
    out << i << ' ' << now << ' ' << action << ' ' << id;
    switch (action) {
      case 0: case 1: pending = domain.engage({id, "missing"}); result(pending); break;
      case 2: case 3: {
        const auto snapshot = domain.state({id});
        const auto revision = snapshot.robots.empty() ? 0 : snapshot.robots[0].revision;
        pending = domain.release(action == 2 ? domain.instance_id() : std::string(32, 'f'), {{id, revision}});
        result(pending);
        break;
      }
      case 4: out << " withdrawn=" << pending.withdraw(); result(pending); break;
      case 5: sink.fail = !sink.fail; break;
      case 6: out << " fault=" << domain.set_fault(id, "bus fault"); break;
      case 7: out << " clear=" << domain.set_fault(id, ""); break;
      case 8: out << " removed=" << domain.remove(id); break;
      case 9: out << " added=" << domain.add(id, id == "a"); break;
      case 10: case 11:
        out << " observed=" << domain.observe(id, action == 10 ? 0 : 0.5, 0, now);
        break;
    }
    state();
    if (i % 3 != 0) {
      const auto stats = domain.tick(sink);
      out << "tick " << stats.written << ',' << stats.failed;
      result(pending);
      state();
    }
  }
  return out.str();
}
}  // namespace

int main(int argc, char **argv) {
  const auto actual = trace();
  if (argc == 1) { std::cout << actual; return 0; }
  std::ifstream input(argv[1]);
  const std::string expected{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  if (actual == expected) return 0;
  std::cerr << "HOLD trace differs from original baseline\n";
  return 1;
}
