#pragma once
// Test doubles shared by the test programs.
#include <xgc2/chassis_hold/domain.hpp>

#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fixtures {

using xgc2::chassis_hold::DomainOptions;
using xgc2::chassis_hold::TimeNs;

constexpr TimeNs ms = 1000000;

// Polls until the condition holds or the timeout ends; false on timeout. A condition that calls a rate limited
// service passes a longer interval.
inline bool wait_until(const std::function<bool()> &condition,
                       std::chrono::milliseconds timeout = std::chrono::seconds(10),
                       std::chrono::microseconds interval = std::chrono::microseconds(100)) {
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (!condition()) {
    if (std::chrono::steady_clock::now() > end)
      return false;
    std::this_thread::sleep_for(interval);
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

// A private scratch directory below $TMPDIR (ctest sets it to the build directory), for key files and sockets. The
// destructor removes whatever the test left in it.
class TempDir {
public:
  TempDir() {
    const char *base = std::getenv("TMPDIR");
    std::string pattern = std::string(base && *base ? base : "/tmp") + "/xgc2-hold-test-XXXXXX";
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    if (!mkdtemp(buffer.data()))
      throw std::runtime_error("mkdtemp failed");
    path_ = buffer.data();
  }
  ~TempDir() {
    if (DIR *directory = opendir(path_.c_str())) {
      while (const dirent *entry = readdir(directory)) {
        const std::string name = entry->d_name;
        if (name != "." && name != "..")
          unlink((path_ + "/" + name).c_str());
      }
      closedir(directory);
    }
    rmdir(path_.c_str());
  }
  TempDir(const TempDir &) = delete;
  TempDir &operator=(const TempDir &) = delete;

  std::string write(const std::string &name, const std::string &content) {
    const std::string file = path_ + "/" + name;
    const int fd = open(file.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd < 0)
      throw std::runtime_error("cannot create " + file);
    std::size_t done = 0;
    while (done < content.size()) {
      const ssize_t n = ::write(fd, content.data() + done, content.size() - done);
      if (n <= 0)
        break;
      done += static_cast<std::size_t>(n);
    }
    close(fd);
    return file;
  }
  const std::string &path() const { return path_; }

private:
  std::string path_;
};

// The host's native thread: runs its tick until stopped.
class TickThread {
public:
  explicit TickThread(std::function<void()> tick, std::function<void()> extra = nullptr)
      : thread_([this, tick, extra] {
          while (running_) {
            tick();
            if (extra)
              extra();
            std::this_thread::sleep_for(std::chrono::microseconds(150));
          }
        }) {}
  TickThread(xgc2::chassis_hold::Domain &domain, Sink &sink, std::function<void()> extra = nullptr)
      : TickThread([&domain, &sink] { domain.tick(sink); }, std::move(extra)) {}
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
