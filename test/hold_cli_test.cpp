// xgc2-hold against a UdpEndpoint on loopback (the path of the tool is the first argument of the program): the
// commands, their JSON output and the exit codes.
#include "check.hpp"
#include "fixtures.hpp"
#include "udp_fixtures.hpp"

#include <poll.h>
#include <spawn.h>
#include <sys/wait.h>

extern char **environ;

using namespace udp_fixtures;

namespace {

struct CliResult {
  int exit_code = -1;
  std::string out;
  std::string err;
};

std::string cli_path() {
  REQUIRE(!test::arguments().empty());
  return test::arguments()[0];
}

// Runs xgc2-hold with the arguments; stdout and stderr are captured.
CliResult run_cli(const std::vector<std::string> &arguments) {
  const std::string program = cli_path();
  std::vector<std::string> words{program};
  words.insert(words.end(), arguments.begin(), arguments.end());
  std::vector<char *> argv;
  for (auto &word : words)
    argv.push_back(&word[0]);
  argv.push_back(nullptr);

  int out_pipe[2], err_pipe[2];
  REQUIRE(pipe(out_pipe) == 0 && pipe(err_pipe) == 0);
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, out_pipe[1], 1);
  posix_spawn_file_actions_adddup2(&actions, err_pipe[1], 2);
  for (const int fd : {out_pipe[0], out_pipe[1], err_pipe[0], err_pipe[1]})
    posix_spawn_file_actions_addclose(&actions, fd);
  pid_t pid = 0;
  const int spawned = posix_spawn(&pid, program.c_str(), &actions, nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  close(out_pipe[1]);
  close(err_pipe[1]);
  REQUIRE(spawned == 0);

  CliResult result;
  pollfd fds[2] = {{out_pipe[0], POLLIN, 0}, {err_pipe[0], POLLIN, 0}};
  std::string *targets[2] = {&result.out, &result.err};
  int open_fds = 2;
  while (open_fds > 0) {
    if (poll(fds, 2, -1) < 0)
      break;
    for (int i = 0; i < 2; ++i) {
      if (fds[i].fd < 0 || !(fds[i].revents & (POLLIN | POLLHUP | POLLERR)))
        continue;
      char buffer[4096];
      const ssize_t n = read(fds[i].fd, buffer, sizeof buffer);
      if (n > 0) {
        targets[i]->append(buffer, static_cast<std::size_t>(n));
      } else {
        close(fds[i].fd);
        fds[i].fd = -1;
        --open_fds;
      }
    }
  }
  int status = 0;
  waitpid(pid, &status, 0);
  result.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
  return result;
}

// xgc2-hold against the rig, with the rig's endpoint and key file unless the arguments say otherwise.
CliResult cli(const Rig &rig, std::vector<std::string> arguments) {
  std::vector<std::string> all{"--endpoint", rig.endpoint_text(), "--key-file", rig.key_file};
  all.insert(all.end(), arguments.begin(), arguments.end());
  return run_cli(all);
}

}  // namespace

TEST(the_cli_walks_through_a_hold_cycle) {
  Rig rig;
  const std::string instance = rig.domain.instance_id();

  CliResult run = cli(rig, {"describe"});
  CHECK_EQ(run.exit_code, 0);
  json out = parse(run.out);
  CHECK_EQ(out["disposition"].get<std::string>(), "response_received");
  CHECK_EQ(out["status"].get<std::string>(), "ok");
  CHECK_EQ(out["instance"].get<std::string>(), instance);
  CHECK_EQ(out["reply"]["robots"][0].get<std::string>(), "ugv-1");

  run = cli(rig, {"engage", "--robot", "ugv-1"});
  CHECK_EQ(run.exit_code, 0);
  out = parse(run.out);
  CHECK_EQ(out["reply"]["robots"][0]["outcome"].get<std::string>(), "engaged");
  CHECK_EQ(out["reply"]["robots"][0]["stage"].get<std::string>(), "zero_written");
  CHECK(rig.domain.state({"ugv-1"}).robots[0].held);

  run = cli(rig, {"state"});
  CHECK_EQ(run.exit_code, 0);
  CHECK(parse(run.out)["reply"]["robots"][0]["held"].get<bool>());

  // Release without a revision reads State first and fences on what it saw.
  run = cli(rig, {"release", "--robot", "ugv-1"});
  CHECK_EQ(run.exit_code, 0);
  CHECK_EQ(parse(run.out)["reply"]["robots"][0]["outcome"].get<std::string>(), "released");
  CHECK(!rig.domain.state({"ugv-1"}).robots[0].held);

  // Nothing left to release: a reply, but not the outcome the command asks for.
  run = cli(rig, {"release", "--robot", "ugv-1"});
  CHECK_EQ(run.exit_code, 1);
  out = parse(run.out);
  CHECK_EQ(out["disposition"].get<std::string>(), "response_received");
  CHECK_EQ(out["status"].get<std::string>(), "ok");
  CHECK_EQ(out["reply"]["robots"][0]["outcome"].get<std::string>(), "not_held");

  // Explicit fence: the right revision releases, a wrong one conflicts and leaves the robot held.
  CHECK_EQ(cli(rig, {"engage", "--all"}).exit_code, 0);
  const std::uint64_t revision = rig.domain.state({"ugv-1"}).robots[0].revision;
  run = cli(rig, {"release", "--instance", instance, "--robot", "ugv-1=" + std::to_string(revision + 5)});
  CHECK_EQ(run.exit_code, 1);
  CHECK_EQ(parse(run.out)["reply"]["robots"][0]["outcome"].get<std::string>(), "conflict");
  CHECK(rig.domain.state({"ugv-1"}).robots[0].held);
  run = cli(rig, {"release", "--instance", instance, "--robot", "ugv-1=" + std::to_string(revision)});
  CHECK_EQ(run.exit_code, 0);
  CHECK(!rig.domain.state({"ugv-1"}).robots[0].held);
}

TEST(the_cli_turns_the_disposition_into_the_exit_code) {
  Rig rig;
  // A reply that says no.
  CliResult run = cli(rig, {"engage", "--robot", "nobody"});
  CHECK_EQ(run.exit_code, 1);
  CHECK_EQ(parse(run.out)["reply"]["robots"][0]["outcome"].get<std::string>(), "unknown_robot");
  run = cli(rig, {"state", "--robot", "nobody"});
  CHECK_EQ(run.exit_code, 1);
  {
    const json out = parse(run.out);
    CHECK_EQ(out["status"].get<std::string>(), "not_found");
    CHECK_EQ(out["reply"]["code"].get<std::string>(), "not_found");
  }

  // No valid reply: the outcome is unknown. A wrong key and a pin on another instance are both silent or refused.
  TempDir dir;
  const std::string other_key = dir.write("other.key", key_line(key_id, std::string(32, 'x')));
  run = cli(rig, {"--key-file", other_key, "--timeout-ms", "300", "engage", "--all"});
  CHECK_EQ(run.exit_code, 2);
  CHECK_EQ(parse(run.out)["disposition"].get<std::string>(), "outcome_unknown");
  CHECK(!rig.domain.state({"ugv-1"}).robots[0].held);
  run = cli(rig, {"--instance", std::string(32, 'a'), "describe"});
  CHECK_EQ(run.exit_code, 2);
  {
    const json out = parse(run.out);
    CHECK_EQ(out["disposition"].get<std::string>(), "outcome_unknown");
    CHECK_EQ(out["status"].get<std::string>(), "conflict");
    CHECK_EQ(out["instance"].get<std::string>(), rig.domain.instance_id());  // who answered
  }

  // Nothing sent: nine long ids do not fit one datagram.
  std::vector<std::string> many{"engage"};
  for (int i = 0; i < 9; ++i) {
    many.push_back("--robot");
    many.push_back(std::string(127, 'r') + std::to_string(i));
  }
  run = cli(rig, many);
  CHECK_EQ(run.exit_code, 3);
  CHECK_EQ(parse(run.out)["disposition"].get<std::string>(), "not_sent");
}

TEST(the_cli_reports_usage_and_configuration_errors) {
  Rig rig;
  const std::vector<std::vector<std::string>> usage = {
      {},
      {"bogus"},
      {"describe", "state"},
      {"engage"},
      {"engage", "--all", "--robot", "ugv-1"},
      {"engage", "--robot", "bad id"},
      {"engage", "--robot", "ugv-1=3"},
      {"state", "--all"},
      {"describe", "--robot", "ugv-1"},
      {"release"},
      {"release", "--all"},
      {"release", "--robot", "ugv-1=x"},
      {"release", "--robot", "ugv-1", "--robot", "ugv-1"},
      {"release", "--instance", "short", "--robot", "ugv-1"},
      {"state", "--timeout-ms", "0"},
      {"state", "--timeout-ms", "60001"},
      {"state", "--no-such-option", "1"},
      {"state", "--robot"},
  };
  for (const auto &arguments : usage) {
    const CliResult run = cli(rig, arguments);
    if (run.exit_code != 64)
      test::fail(__FILE__, __LINE__, "usage error expected for: " + (arguments.empty() ? std::string("(none)") : arguments[0]) +
                                         " exit " + std::to_string(run.exit_code));
    CHECK(run.out.empty());
    CHECK(run.err.find("usage:") != std::string::npos);
  }

  TempDir dir;
  const std::string empty_key = dir.write("empty.key", "# nothing\n");
  const std::string two_keys = dir.write("two.key", key_line(1, key_bytes) + key_line(key_id, key_bytes));
  CliResult run = run_cli({"--key-file", dir.path() + "/missing.key", "describe"});
  CHECK_EQ(run.exit_code, 78);
  run = run_cli({"--endpoint", rig.endpoint_text(), "--key-file", empty_key, "describe"});
  CHECK_EQ(run.exit_code, 78);  // a key file without a key
  run = run_cli({"--endpoint", rig.endpoint_text(), "--key-file", two_keys, "describe"});
  CHECK_EQ(run.exit_code, 64);  // several keys need --key-id
  run = run_cli({"--endpoint", rig.endpoint_text(), "--key-file", two_keys, "--key-id", "5", "describe"});
  CHECK_EQ(run.exit_code, 78);  // a key id the file does not have
  run = run_cli({"--endpoint", "127.0.0.1", "--key-file", rig.key_file, "describe"});
  CHECK_EQ(run.exit_code, 78);
  run = run_cli({"--endpoint", "127.0.0.1:0", "--key-file", rig.key_file, "describe"});
  CHECK_EQ(run.exit_code, 78);
}

TEST(the_cli_picks_the_key_and_resolves_names) {
  Rig rig;
  TempDir dir;
  const std::string two_keys = dir.write("two.key", key_line(1, std::string(32, 'a')) + key_line(key_id, key_bytes));
  // Several keys: --key-id chooses; a name resolves (IPv4 first); the sole key of rig.key_file is the default.
  CliResult run = run_cli({"--endpoint", "localhost:" + std::to_string(rig.endpoint->port()), "--key-file", two_keys,
                           "--key-id", std::to_string(key_id), "describe"});
  CHECK_EQ(run.exit_code, 0);
  run = run_cli({"--endpoint=" + rig.endpoint_text(), "--key-file=" + rig.key_file, "describe"});
  CHECK_EQ(run.exit_code, 0);
  CHECK_EQ(parse(run.out)["instance"].get<std::string>(), rig.domain.instance_id());
}

TEST(the_cli_prints_its_usage_on_request) {
  const CliResult run = run_cli({"--help"});
  CHECK_EQ(run.exit_code, 0);
  CHECK(run.out.find("usage: xgc2-hold") != std::string::npos);
  CHECK(run.err.empty());
}

TEST_MAIN()
