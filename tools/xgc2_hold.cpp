// xgc2-hold: command line client of the chassis HOLD udp.v1 endpoint (contracts/chassis-hold.md).
//
//   xgc2-hold [options] describe
//   xgc2-hold [options] state [--robot ID]...
//   xgc2-hold [options] engage (--robot ID)... | --all
//   xgc2-hold [options] release (--robot ID[=REVISION])... [--instance HEX]
//
// Prints one JSON object per run: {"disposition", "status", "instance", "attempts", "reply" or "message"}. The
// exit code follows the disposition of the call: 0 a reply was received and the request did what it asked, 1 a
// reply was received and it is an error or reports a robot that was not engaged or released, 2 the outcome is
// unknown (a request was sent, no valid reply arrived), 3 nothing was sent, 64 a usage error, 78 a key file
// or address that cannot be used.
#include <xgc2/chassis_hold/service.hpp>
#include <xgc2/xrpc/udp.hpp>

#include <nlohmann/json.hpp>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace udp = xgc2::xrpc::udp;
using json = nlohmann::json;
using xgc2::chassis_hold::valid_instance_id;
using xgc2::chassis_hold::valid_robot_id;

constexpr int exit_usage = 64;
constexpr int exit_config = 78;

const char *const usage_text =
    "usage: xgc2-hold [options] <command>\n"
    "\n"
    "commands:\n"
    "  describe                          instance, roster and capabilities of the host\n"
    "  state   [--robot ID]...           HOLD state of the named robots, of all when none is named\n"
    "  engage  --robot ID [--robot ID]... | --all\n"
    "  release --robot ID[=REVISION]... [--instance HEX]\n"
    "                                    a robot without =REVISION is released at the revision a State call\n"
    "                                    reports; a missing --instance is taken from that call as well\n"
    "\n"
    "options:\n"
    "  --endpoint HOST:PORT   address of the driver, an IP address or a name (default 127.0.0.1:19530)\n"
    "  --key-file PATH        key file (default /etc/xgc2/chassis-hold.key)\n"
    "  --key-id N             key id; default: the key of a file that holds exactly one\n"
    "  --instance HEX         pin the call to this instance of the host (release: also the fenced instance)\n"
    "  --timeout-ms N         deadline of each call, 1..60000 (default 2000)\n"
    "  -h, --help             this text\n";

struct UsageError : std::runtime_error {
  using std::runtime_error::runtime_error;
};
struct ConfigError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct Robot {
  std::string id;
  std::optional<std::uint64_t> revision;
};

struct Arguments {
  std::string command;
  std::string endpoint = "127.0.0.1:19530";
  std::string key_file = "/etc/xgc2/chassis-hold.key";
  std::optional<std::uint32_t> key_id;
  std::optional<std::string> instance;
  std::chrono::milliseconds timeout{2000};
  std::vector<Robot> robots;
  bool all = false;
};

std::uint64_t parse_unsigned(const std::string &text, std::uint64_t max, const std::string &what) {
  if (text.empty() || text.size() > 20 || text.find_first_not_of("0123456789") != std::string::npos)
    throw UsageError(what + " must be a non-negative integer");
  std::uint64_t value = 0;
  for (const char c : text) {
    const std::uint64_t digit = static_cast<std::uint64_t>(c - '0');
    if (value > (UINT64_MAX - digit) / 10)
      throw UsageError(what + " is too large");
    value = value * 10 + digit;
  }
  if (value > max)
    throw UsageError(what + " is too large");
  return value;
}

Arguments parse_arguments(int argc, char **argv) {
  Arguments args;
  std::vector<std::string> words(argv + 1, argv + argc);
  for (std::size_t i = 0; i < words.size(); ++i) {
    std::string word = words[i];
    if (word == "-h" || word == "--help") {
      std::fputs(usage_text, stdout);
      std::exit(0);
    }
    if (word.compare(0, 2, "--") != 0) {
      if (!args.command.empty())
        throw UsageError("more than one command: " + args.command + " and " + word);
      args.command = word;
      continue;
    }
    std::string value;
    bool has_value = false;
    const std::size_t equals = word.find('=');
    if (equals != std::string::npos) {
      value = word.substr(equals + 1);
      word.resize(equals);
      has_value = true;
    }
    if (word == "--all") {
      if (has_value)
        throw UsageError("--all takes no value");
      args.all = true;
      continue;
    }
    if (!has_value) {
      if (i + 1 >= words.size())
        throw UsageError(word + " needs a value");
      value = words[++i];
    }
    if (word == "--endpoint") {
      args.endpoint = value;
    } else if (word == "--key-file") {
      args.key_file = value;
    } else if (word == "--key-id") {
      args.key_id = static_cast<std::uint32_t>(parse_unsigned(value, UINT32_MAX, "--key-id"));
    } else if (word == "--instance") {
      if (!valid_instance_id(value))
        throw UsageError("--instance must be 32 lowercase hex digits");
      args.instance = value;
    } else if (word == "--timeout-ms") {
      args.timeout = std::chrono::milliseconds(static_cast<std::int64_t>(parse_unsigned(value, 60000, "--timeout-ms")));
      if (args.timeout.count() < 1)
        throw UsageError("--timeout-ms must be at least 1");
    } else if (word == "--robot") {
      Robot robot;
      const std::size_t split = value.find('=');
      robot.id = value.substr(0, split);
      if (split != std::string::npos)
        robot.revision = parse_unsigned(value.substr(split + 1), UINT64_MAX, "the revision of " + robot.id);
      if (!valid_robot_id(robot.id))
        throw UsageError("not a valid robot id: " + robot.id);
      args.robots.push_back(robot);
    } else {
      throw UsageError("unknown option " + word);
    }
  }

  if (args.command.empty())
    throw UsageError("no command");
  std::set<std::string> seen;
  for (const auto &robot : args.robots)
    if (!seen.insert(robot.id).second)
      throw UsageError("robot named twice: " + robot.id);
  const bool revisions = std::any_of(args.robots.begin(), args.robots.end(),
                                     [](const Robot &robot) { return robot.revision.has_value(); });
  if (args.command == "describe") {
    if (!args.robots.empty() || args.all)
      throw UsageError("describe takes no robots");
  } else if (args.command == "state") {
    if (args.all)
      throw UsageError("state takes all robots when none is named; --all is for engage");
  } else if (args.command == "engage") {
    if (args.all == !args.robots.empty())
      throw UsageError("engage needs --robot ID... or --all");
  } else if (args.command == "release") {
    if (args.all || args.robots.empty())
      throw UsageError("release needs --robot ID[=REVISION]...");
  } else {
    throw UsageError("unknown command " + args.command);
  }
  if (revisions && args.command != "release")
    throw UsageError("only release takes a revision (--robot ID=REVISION)");
  return args;
}

// Peeks at the key ids of a key file: the default key id is the one of a file with exactly one key.
std::optional<std::uint32_t> only_key_id(const std::string &path) {
  std::ifstream file(path);
  std::vector<std::uint32_t> ids;
  for (std::string line; std::getline(file, line);) {
    line = line.substr(0, line.find('#'));
    std::istringstream words(line);
    std::string id;
    if (words >> id)
      ids.push_back(static_cast<std::uint32_t>(parse_unsigned(id, UINT32_MAX, "a key id in " + path)));
  }
  if (ids.size() == 1)
    return ids.front();
  return std::nullopt;
}

// The udp client sends to numeric addresses; names are resolved here, once, before the call.
std::string numeric_endpoint(const std::string &endpoint) {
  const std::size_t colon = endpoint.rfind(':');
  if (colon == std::string::npos || colon + 1 == endpoint.size())
    throw ConfigError("endpoint must be HOST:PORT: " + endpoint);
  std::string host = endpoint.substr(0, colon);
  const std::string port = endpoint.substr(colon + 1);
  if (host.size() >= 2 && host.front() == '[' && host.back() == ']')
    host = host.substr(1, host.size() - 2);
  if (host.empty() || parse_unsigned(port, 65535, "the port") == 0)
    throw ConfigError("endpoint must be HOST:PORT: " + endpoint);
  in6_addr v6;
  in_addr v4;
  if (inet_pton(AF_INET, host.c_str(), &v4) == 1)
    return host + ":" + port;
  if (inet_pton(AF_INET6, host.c_str(), &v6) == 1)
    return "[" + host + "]:" + port;
  addrinfo hints;
  std::memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  addrinfo *found = nullptr;
  if (getaddrinfo(host.c_str(), nullptr, &hints, &found) != 0 || !found)
    throw ConfigError("cannot resolve " + host);
  // IPv4 first: the endpoint binds 0.0.0.0 unless the driver was told otherwise.
  std::string resolved;
  for (const int family : {AF_INET, AF_INET6}) {
    for (const addrinfo *item = found; item && resolved.empty(); item = item->ai_next) {
      char text[INET6_ADDRSTRLEN] = {0};
      if (item->ai_family != family)
        continue;
      if (family == AF_INET) {
        inet_ntop(AF_INET, &reinterpret_cast<const sockaddr_in *>(item->ai_addr)->sin_addr, text, sizeof text);
        resolved = std::string(text) + ":" + port;
      } else {
        inet_ntop(AF_INET6, &reinterpret_cast<const sockaddr_in6 *>(item->ai_addr)->sin6_addr, text, sizeof text);
        resolved = "[" + std::string(text) + "]:" + port;
      }
    }
  }
  freeaddrinfo(found);
  if (resolved.empty())
    throw ConfigError("no usable address for " + host);
  return resolved;
}

struct Outcome {
  json envelope;
  json reply;  // the parsed reply body, null when there is none
  int exit_code = 0;
  std::optional<std::string> instance;
};

// One call. The envelope carries what the caller can know: whether a reply arrived, its status and instance, and
// the reply itself.
Outcome call(const udp::Client &client, const Arguments &args, std::uint32_t key_id, const std::string &endpoint,
             const std::string &method, const json &body, const std::optional<std::string> &pin) {
  const auto deadline = std::chrono::steady_clock::now() + args.timeout;
  std::optional<udp::InstanceId> expected;
  if (pin)
    expected = udp::instance_from_hex(*pin);
  const udp::Response response = client.call(endpoint, key_id, std::string(xgc2::chassis_hold::service_name) + "/" + method,
                                             body.dump(), deadline, expected);
  Outcome outcome;
  json &envelope = outcome.envelope;
  envelope["disposition"] = udp::delivery_name(response.delivery);
  envelope["status"] = std::string(udp::status_name(response.status));
  envelope["attempts"] = response.attempts;
  if (response.instance != udp::InstanceId{}) {
    outcome.instance = udp::to_hex(response.instance);
    envelope["instance"] = *outcome.instance;
  }
  if (!response.message.empty())
    envelope["message"] = response.message;
  if (!response.body.empty()) {
    try {
      outcome.reply = json::parse(response.body);
      envelope["reply"] = outcome.reply;
      (void)envelope.dump();  // strings that are not valid UTF-8 surface here
    } catch (const std::exception &) {
      outcome.reply = json();
      envelope["reply"] = json();
      envelope["message"] = "the reply is not valid JSON text";
    }
  }

  if (response.delivery == udp::Delivery::NotSent)
    outcome.exit_code = 3;
  else if (response.delivery == udp::Delivery::OutcomeUnknown)
    outcome.exit_code = 2;
  else
    outcome.exit_code = response.status == udp::Status::Ok ? 0 : 1;
  return outcome;
}

// The fields a release reads from a State reply; the reply is authentic but not trusted to have the shape.
std::optional<std::string> reported_instance(const json &reply) {
  if (!reply.is_object())
    return std::nullopt;
  const auto found = reply.find("instance");
  if (found == reply.end() || !found->is_string())
    return std::nullopt;
  return found->get<std::string>();
}

std::optional<std::uint64_t> reported_revision(const json &reply, const std::string &robot_id) {
  if (!reply.is_object() || reply.find("robots") == reply.end() || !reply["robots"].is_array())
    return std::nullopt;
  for (const auto &robot : reply["robots"]) {
    if (!robot.is_object() || robot.find("robot_id") == robot.end() || robot.find("revision") == robot.end())
      continue;
    if (robot["robot_id"].is_string() && robot["robot_id"].get<std::string>() == robot_id &&
        robot["revision"].is_number_unsigned())
      return robot["revision"].get<std::uint64_t>();
  }
  return std::nullopt;
}

// A reply with per-robot outcomes is a success only if every robot got the outcome the command asks for.
bool outcomes_are(const json &reply, const std::vector<std::string> &wanted) {
  if (!reply.is_object() || reply.find("robots") == reply.end() || !reply["robots"].is_array())
    return false;
  for (const auto &robot : reply["robots"]) {
    if (!robot.is_object() || robot.find("outcome") == robot.end() || !robot["outcome"].is_string())
      return false;
    const std::string outcome = robot["outcome"].get<std::string>();
    if (std::find(wanted.begin(), wanted.end(), outcome) == wanted.end())
      return false;
  }
  return true;
}

int run(const Arguments &args) {
  udp::KeyRing keys;
  try {
    keys = udp::KeyRing::load_file(args.key_file);
  } catch (const std::exception &e) {
    throw ConfigError(std::string("key file: ") + e.what());
  }
  if (keys.empty())
    throw ConfigError("key file " + args.key_file + " holds no key");
  std::optional<std::uint32_t> key_id = args.key_id ? args.key_id : only_key_id(args.key_file);
  if (!key_id)
    throw UsageError("the key file holds several keys: --key-id is required");
  if (!keys.find(*key_id))
    throw ConfigError("the key file has no key " + std::to_string(*key_id));
  const std::string endpoint = numeric_endpoint(args.endpoint);
  const udp::Client client(std::move(keys));

  json ids = json::array();
  for (const auto &robot : args.robots)
    ids.push_back(robot.id);

  Outcome outcome;
  if (args.command == "describe") {
    outcome = call(client, args, *key_id, endpoint, "Describe", json::object(), args.instance);
  } else if (args.command == "state") {
    json body = json::object();
    if (!args.robots.empty())
      body["robot_ids"] = ids;
    outcome = call(client, args, *key_id, endpoint, "State", body, args.instance);
  } else if (args.command == "engage") {
    json body = json::object();
    if (args.all)
      body["all"] = true;
    else
      body["robot_ids"] = ids;
    outcome = call(client, args, *key_id, endpoint, "Engage", body, args.instance);
    if (outcome.exit_code == 0 && !outcomes_are(outcome.reply, {"engaged", "already_held"}))
      outcome.exit_code = 1;
  } else {
    // Release is fenced on the instance and on each robot's revision. What the operator did not give is read with one
    // State call and used as it was reported, the way a person would copy it from the screen.
    std::optional<std::string> instance = args.instance;
    std::vector<Robot> robots = args.robots;
    const bool lookup = !instance || std::any_of(robots.begin(), robots.end(),
                                                 [](const Robot &robot) { return !robot.revision; });
    if (lookup) {
      json body = json::object();
      body["robot_ids"] = ids;
      Outcome state = call(client, args, *key_id, endpoint, "State", body, args.instance);
      if (state.exit_code != 0) {
        std::printf("%s\n", state.envelope.dump().c_str());
        return state.exit_code;
      }
      if (!instance)
        instance = reported_instance(state.reply);
      for (auto &robot : robots)
        if (!robot.revision)
          robot.revision = reported_revision(state.reply, robot.id);
    }
    if (!instance || !valid_instance_id(*instance))
      throw ConfigError("the host did not report its instance");
    json changes = json::array();
    for (const auto &robot : robots) {
      if (!robot.revision)
        throw ConfigError("the host did not report the revision of " + robot.id);
      json change = json::object();
      change["robot_id"] = robot.id;
      change["expected_revision"] = *robot.revision;
      changes.push_back(change);
    }
    json body = json::object();
    body["expected_instance"] = *instance;
    body["changes"] = changes;
    outcome = call(client, args, *key_id, endpoint, "Release", body, instance);
    if (outcome.exit_code == 0 && !outcomes_are(outcome.reply, {"released"}))
      outcome.exit_code = 1;
  }
  std::printf("%s\n", outcome.envelope.dump().c_str());
  return outcome.exit_code;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    return run(parse_arguments(argc, argv));
  } catch (const UsageError &e) {
    std::fprintf(stderr, "xgc2-hold: %s\n\n%s", e.what(), usage_text);
    return exit_usage;
  } catch (const ConfigError &e) {
    std::fprintf(stderr, "xgc2-hold: %s\n", e.what());
    return exit_config;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "xgc2-hold: %s\n", e.what());
    return 1;
  }
}
