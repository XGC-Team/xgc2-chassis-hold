#pragma once
// A chassis HOLD host on loopback for the udp.v1 tests: one robot "ugv-1" with feedback, the service, the endpoint
// and a client with the key; plus a scratch directory for key files.
#include "check.hpp"
#include "fixtures.hpp"

#include <xgc2/chassis_hold/udp_endpoint.hpp>
#include <xgc2/xrpc/udp.hpp>

#include <nlohmann/json.hpp>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace udp_fixtures {

using namespace xgc2::chassis_hold;
namespace udp = xgc2::xrpc::udp;
using json = nlohmann::json;
using fixtures::Sink;
using fixtures::TickThread;
using std::chrono::milliseconds;
using std::chrono::seconds;

constexpr std::uint32_t key_id = 7;
const std::string key_bytes(32, 'k');

inline std::string base64(const std::string &bytes) {
  static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  for (std::size_t i = 0; i < bytes.size(); i += 3) {
    const std::size_t left = bytes.size() - i;
    const unsigned a = static_cast<unsigned char>(bytes[i]);
    const unsigned b = left > 1 ? static_cast<unsigned char>(bytes[i + 1]) : 0;
    const unsigned c = left > 2 ? static_cast<unsigned char>(bytes[i + 2]) : 0;
    out += alphabet[a >> 2];
    out += alphabet[((a & 3) << 4) | (b >> 4)];
    out += left > 1 ? alphabet[((b & 15) << 2) | (c >> 6)] : '=';
    out += left > 2 ? alphabet[c & 63] : '=';
  }
  return out;
}

inline std::string key_line(std::uint32_t id, const std::string &key) { return std::to_string(id) + " " + base64(key) + "\n"; }

// A scratch directory below $TMPDIR (ctest sets it to the build directory) for key files.
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
    for (const auto &file : files_)
      unlink(file.c_str());
    rmdir(path_.c_str());
  }
  TempDir(const TempDir &) = delete;
  TempDir &operator=(const TempDir &) = delete;

  std::string write(const std::string &name, const std::string &content) {
    const std::string file = path_ + "/" + name;
    const int fd = open(file.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd < 0)
      throw std::runtime_error("cannot create " + file);
    files_.push_back(file);
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
  std::vector<std::string> files_;
};

struct RigOptions {
  std::uint16_t port = 0;
  bool tick = true;  // run the fake native tick thread
  DomainOptions domain;
  ServiceOptions service;
};

// A host: one robot "ugv-1" with feedback, the service, the endpoint on 127.0.0.1 and a client with the key.
struct Rig {
  explicit Rig(RigOptions options = {})
      : domain(options.domain), service(domain, options.service), client(make_keys()) {
    domain.add("ugv-1", true);
    key_file = dir.write("hold.key", key_line(key_id, key_bytes));
    if (options.tick)
      start_ticker();
    UdpEndpointOptions endpoint_options;
    endpoint_options.bind_address = "127.0.0.1";
    endpoint_options.port = options.port;
    endpoint_options.key_file = key_file;
    endpoint = std::make_unique<UdpEndpoint>(service, endpoint_options);
  }

  static udp::KeyRing make_keys() {
    udp::KeyRing keys;
    keys.add(key_id, key_bytes);
    return keys;
  }
  void start_ticker() { ticker = std::make_unique<TickThread>(domain, sink); }
  std::string endpoint_text() const { return "127.0.0.1:" + std::to_string(endpoint->port()); }

  udp::Response call(const std::string &method, const json &body, milliseconds timeout = seconds(2),
                     const std::optional<std::string> &pin = std::nullopt) const {
    std::optional<udp::InstanceId> expected;
    if (pin)
      expected = udp::instance_from_hex(*pin);
    return client.call(endpoint_text(), key_id, std::string(service_name) + "/" + method, body.dump(),
                       std::chrono::steady_clock::now() + timeout, expected);
  }
  std::string release_body(const std::string &instance, std::uint64_t revision) const {
    return json{{"expected_instance", instance},
                {"changes", json::array({json{{"robot_id", "ugv-1"}, {"expected_revision", revision}}})}}
        .dump();
  }

  // Destruction order: the endpoint, the tick thread, the sink, the service, the domain, the files.
  TempDir dir;
  Domain domain;
  Service service;
  Sink sink;
  std::unique_ptr<TickThread> ticker;
  std::unique_ptr<UdpEndpoint> endpoint;
  udp::Client client;
  std::string key_file;
};

inline bool answered(const udp::Response &response, udp::Status status = udp::Status::Ok) {
  return response.delivery == udp::Delivery::ResponseReceived && response.status == status;
}

inline json parse(const std::string &text) {
  try {
    return json::parse(text);
  } catch (const std::exception &e) {
    test::fail(__FILE__, __LINE__, std::string("not JSON: ") + e.what() + ": " + text);
    return json();
  }
}

inline std::string first_outcome(const json &reply) { return reply["robots"][0]["outcome"].get<std::string>(); }

}  // namespace udp_fixtures
