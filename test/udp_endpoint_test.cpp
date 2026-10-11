// The udp.v1 endpoint end to end: UdpEndpoint on loopback over a real domain with a fake native tick thread, called
// by the xrpc udp client.
#include "check.hpp"
#include "fixtures.hpp"
#include "udp_fixtures.hpp"

#include <system_error>
#include <thread>

using namespace udp_fixtures;
using fixtures::wait_until;

namespace {

template <typename Exception>
bool throws_with(const std::function<void()> &action, const std::string &fragment) {
  try {
    action();
  } catch (const Exception &e) {
    const bool found = std::string(e.what()).find(fragment) != std::string::npos;
    if (!found)
      test::fail(__FILE__, __LINE__, "message lacks \"" + fragment + "\": " + e.what());
    return found;
  } catch (const std::exception &e) {
    test::fail(__FILE__, __LINE__, std::string("another exception type: ") + e.what());
    return false;
  }
  test::fail(__FILE__, __LINE__, "nothing was thrown, expected \"" + fragment + "\"");
  return false;
}

}  // namespace

TEST(a_hold_cycle_over_udp_and_the_instance_is_shared) {
  Rig rig;
  const udp::Response describe = rig.call("Describe", json::object());
  REQUIRE(answered(describe));
  const json described = parse(describe.body);
  // One identity: the id in every reply header, the one in the bodies, the domain's and the endpoint's.
  const std::string instance = rig.domain.instance_id();
  CHECK_EQ(udp::to_hex(describe.instance), instance);
  CHECK_EQ(described["instance"].get<std::string>(), instance);
  CHECK_EQ(rig.endpoint->instance_id(), instance);
  CHECK_EQ(described["robots"][0].get<std::string>(), "ugv-1");
  CHECK(described["feedback"]["ugv-1"].get<bool>());

  // With the tick running the engage reply says the zero is written.
  const udp::Response engage = rig.call("Engage", json{{"robot_ids", json::array({"ugv-1"})}}, seconds(2), instance);
  REQUIRE(answered(engage));
  const json engaged = parse(engage.body);
  CHECK_EQ(first_outcome(engaged), "engaged");
  CHECK_EQ(engaged["robots"][0]["stage"].get<std::string>(), "zero_written");
  CHECK_EQ(engaged["robots"][0]["revision"].get<std::uint64_t>(), 1);
  CHECK(!rig.domain.admit("ugv-1", rig.domain.now()));
  CHECK(rig.sink.writes("ugv-1") > 0);

  const udp::Response state = rig.call("State", json::object());
  REQUIRE(answered(state));
  CHECK(parse(state.body)["robots"][0]["held"].get<bool>());

  // Release fenced on the shared instance and the revision just read.
  const udp::Response release = rig.call("Release", parse(rig.release_body(instance, 1)), seconds(2), instance);
  REQUIRE(answered(release));
  CHECK_EQ(first_outcome(parse(release.body)), "released");
  CHECK(rig.domain.admit("ugv-1", rig.domain.now()));
  // A replay does not apply twice.
  const udp::Response replay = rig.call("Release", parse(rig.release_body(instance, 1)));
  CHECK_EQ(first_outcome(parse(replay.body)), "not_held");
}

TEST(an_explicit_domain_instance_is_the_transport_instance) {
  RigOptions options;
  options.domain.instance_id = "0123456789abcdef0123456789abcdef";
  Rig rig(options);
  CHECK_EQ(rig.endpoint->instance_id(), options.domain.instance_id);
  const udp::Response describe = rig.call("Describe", json::object());
  REQUIRE(answered(describe));
  CHECK_EQ(udp::to_hex(describe.instance), options.domain.instance_id);
  CHECK_EQ(parse(describe.body)["instance"].get<std::string>(), options.domain.instance_id);
}

TEST(a_restarted_host_is_a_new_instance_and_holds_nothing) {
  std::string old_instance;
  std::uint16_t port = 0;
  {
    Rig rig;
    old_instance = rig.domain.instance_id();
    port = rig.endpoint->port();
    REQUIRE(answered(rig.call("Engage", json{{"all", true}})));
    CHECK(rig.domain.state({"ugv-1"}).robots[0].held);
  }
  RigOptions options;
  options.port = port;
  Rig restarted(options);
  CHECK(restarted.domain.instance_id() != old_instance);
  CHECK(!restarted.domain.state({"ugv-1"}).robots[0].held);  // no persistence

  // A caller that pinned the old instance learns that it is gone, and its request does not run.
  const udp::Response pinned = restarted.call("Engage", json{{"all", true}}, seconds(2), old_instance);
  CHECK(pinned.delivery == udp::Delivery::OutcomeUnknown);
  CHECK(pinned.status == udp::Status::Conflict);
  CHECK_EQ(udp::to_hex(pinned.instance), restarted.domain.instance_id());
  CHECK(!restarted.domain.state({"ugv-1"}).robots[0].held);

  // A release that carries the old instance in its body conflicts for every robot instead of failing.
  restarted.domain.engage({"ugv-1"});
  const udp::Response stale = restarted.call("Release", parse(restarted.release_body(old_instance, 1)));
  REQUIRE(answered(stale));
  CHECK_EQ(first_outcome(parse(stale.body)), "conflict");
  CHECK(restarted.domain.state({"ugv-1"}).robots[0].held);
}

TEST(requests_with_a_wrong_key_are_not_answered_and_not_executed) {
  Rig rig;
  udp::KeyRing wrong_key;
  wrong_key.add(key_id, std::string(32, 'x'));
  wrong_key.add(99, std::string(32, 'y'));
  const udp::Client intruder(wrong_key);
  const std::string method = std::string(service_name) + "/Engage";
  for (const std::uint32_t id : {key_id, std::uint32_t(99)}) {  // a bad tag, then an unknown key id
    const udp::Response response = intruder.call(rig.endpoint_text(), id, method, "{\"all\":true}",
                                                 std::chrono::steady_clock::now() + milliseconds(300));
    CHECK(response.delivery == udp::Delivery::OutcomeUnknown);
  }
  CHECK(!rig.domain.state({"ugv-1"}).robots[0].held);
  CHECK(answered(rig.call("Describe", json::object())));
}

TEST(engage_without_a_tick_reports_gated_and_the_zero_follows) {
  RigOptions options;
  options.tick = false;
  Rig rig(options);
  const auto started = std::chrono::steady_clock::now();
  const udp::Response engage = rig.call("Engage", json{{"robot_ids", json::array({"ugv-1"})}});
  const auto elapsed = std::chrono::steady_clock::now() - started;
  REQUIRE(answered(engage));
  // The reply waited for reply_wait (60 ms), then told the truth.
  CHECK(elapsed >= milliseconds(50));
  CHECK_EQ(parse(engage.body)["robots"][0]["stage"].get<std::string>(), "gated");
  CHECK(!rig.domain.admit("ugv-1", rig.domain.now()));  // the gate does not wait for the tick

  rig.start_ticker();
  std::string stage;
  for (int i = 0; i < 100 && stage != "zero_written"; ++i) {
    stage = parse(rig.call("State", json::object()).body)["robots"][0]["stage"].get<std::string>();
    if (stage != "zero_written")
      std::this_thread::sleep_for(milliseconds(5));
  }
  CHECK_EQ(stage, "zero_written");
}

TEST(a_release_the_tick_cannot_apply_fails_with_deadline_exceeded_and_is_not_applied_later) {
  RigOptions options;
  options.tick = false;
  options.service.release_wait = milliseconds(150);
  Rig rig(options);
  const std::string instance = rig.domain.instance_id();
  REQUIRE(answered(rig.call("Engage", json{{"all", true}})));  // the gate closes without a tick

  // Nobody ticks: the release is not applied, and the caller is told so instead of waiting for a device.
  const auto started = std::chrono::steady_clock::now();
  const udp::Response release = rig.call("Release", parse(rig.release_body(instance, 1)), seconds(2), instance);
  CHECK(std::chrono::steady_clock::now() - started >= milliseconds(120));
  CHECK(answered(release, udp::Status::DeadlineExceeded));
  CHECK_EQ(parse(release.body)["code"].get<std::string>(), "deadline_exceeded");
  CHECK(rig.domain.state({"ugv-1"}).robots[0].held);

  // The tick starts; the release that gave up does not come back to life, and a new one goes through.
  rig.start_ticker();
  CHECK(wait_until([&] { return rig.domain.state({"ugv-1"}).robots[0].stage == Stage::zero_written; }));
  CHECK(rig.domain.state({"ugv-1"}).robots[0].held);
  CHECK(!rig.domain.admit("ugv-1", rig.domain.now()));
  const udp::Response again = rig.call("Release", parse(rig.release_body(instance, 1)), seconds(2), instance);
  REQUIRE(answered(again));
  CHECK_EQ(first_outcome(parse(again.body)), "released");
  CHECK(rig.domain.admit("ugv-1", rig.domain.now()));
}

TEST(a_reply_that_does_not_fit_a_datagram_is_refused_but_the_engage_stands) {
  Rig rig;
  for (int i = 0; i < 8; ++i)
    rig.domain.add(std::string(127, 'r') + std::to_string(i));
  // Nine robots with long ids: the reply is larger than one datagram, the transport replaces it by an error.
  const udp::Response engage = rig.call("Engage", json{{"all", true}});
  CHECK(answered(engage, udp::Status::ResourceExhausted));
  CHECK_EQ(parse(engage.body)["code"].get<std::string>(), "resource_exhausted");
  for (const auto &robot : rig.domain.state().robots)
    CHECK(robot.held);  // the request had its effect; the caller reads State, with fewer robots at a time

  const udp::Response state = rig.call("State", json{{"robot_ids", json::array({"ugv-1"})}});
  REQUIRE(answered(state));
  CHECK(parse(state.body)["robots"][0]["held"].get<bool>());
}

TEST(bad_requests_get_error_statuses) {
  Rig rig;
  const std::string unknown_method = std::string(service_name) + "/Nope";
  const udp::Response nope = rig.client.call(rig.endpoint_text(), key_id, unknown_method, "{}",
                                             std::chrono::steady_clock::now() + seconds(2));
  CHECK(answered(nope, udp::Status::NotFound));
  const udp::Response other_service = rig.client.call(rig.endpoint_text(), key_id, "xgc2.chassis.other/State", "{}",
                                                      std::chrono::steady_clock::now() + seconds(2));
  CHECK(answered(other_service, udp::Status::NotFound));

  const udp::Response invalid = rig.call("Engage", json::object());
  CHECK(answered(invalid, udp::Status::InvalidArgument));
  CHECK_EQ(parse(invalid.body)["code"].get<std::string>(), "invalid_argument");
  const udp::Response unknown_robot = rig.call("State", json{{"robot_ids", json::array({"nobody"})}});
  CHECK(answered(unknown_robot, udp::Status::NotFound));
  const udp::Response not_json = rig.client.call(rig.endpoint_text(), key_id, std::string(service_name) + "/State",
                                                 "{\"robot_ids\":[", std::chrono::steady_clock::now() + seconds(2));
  CHECK(answered(not_json, udp::Status::InvalidArgument));
  CHECK(!rig.domain.state({"ugv-1"}).robots[0].held);
}

TEST(many_clients_hold_and_release_their_own_robots) {
  Rig rig;
  constexpr int clients = 4;
  constexpr int rounds = 8;  // 64 requests: below the transport's burst allowance for one source
  for (int c = 0; c < clients; ++c)
    rig.domain.add("c" + std::to_string(c));
  std::vector<std::thread> threads;
  for (int c = 0; c < clients; ++c)
    threads.emplace_back([&rig, c] {
      const std::string id = "c" + std::to_string(c);
      for (int round = 0; round < rounds; ++round) {
        const udp::Response engage = rig.call("Engage", json{{"robot_ids", json::array({id})}}, seconds(5));
        if (!answered(engage)) {
          test::fail(__FILE__, __LINE__, "engage of " + id + " was not answered");
          return;
        }
        const json reply = parse(engage.body);
        if (reply["robots"][0]["stage"].get<std::string>() != "zero_written")
          test::fail(__FILE__, __LINE__, "engage of " + id + " did not report the zero");
        const json release = json{{"expected_instance", rig.domain.instance_id()},
                                  {"changes", json::array({json{{"robot_id", id},
                                                                {"expected_revision",
                                                                 reply["robots"][0]["revision"].get<std::uint64_t>()}}})}};
        const udp::Response released = rig.call("Release", release, seconds(5));
        if (!answered(released) || first_outcome(parse(released.body)) != "released")
          test::fail(__FILE__, __LINE__, "release of " + id + " did not apply");
      }
    });
  for (auto &thread : threads)
    thread.join();
  for (int c = 0; c < clients; ++c)
    CHECK(!rig.domain.state({"c" + std::to_string(c)}).robots[0].held);
}

TEST(closing_the_endpoint_with_a_reply_pending_is_harmless) {
  RigOptions options;
  options.tick = false;
  options.service.reply_wait = seconds(5);
  Rig rig(options);
  udp::Response response;
  // The caller must not touch the endpoint object, which the test resets: it gets the address up front.
  const std::string target = rig.endpoint_text();
  std::thread caller([&] {
    response = rig.client.call(target, key_id, std::string(service_name) + "/Engage", "{\"all\":true}",
                               std::chrono::steady_clock::now() + milliseconds(700));
  });
  for (int i = 0; i < 400 && !rig.domain.state({"ugv-1"}).robots[0].held; ++i)
    std::this_thread::sleep_for(milliseconds(5));
  CHECK(rig.domain.state({"ugv-1"}).robots[0].held);  // the engage ran; its reply waits for a tick
  rig.endpoint.reset();                                  // the socket closes with the reply pending
  caller.join();
  CHECK(response.delivery == udp::Delivery::OutcomeUnknown);
  // The service completes the waiting reply when it goes away; the closed server just refuses it.
}

TEST(the_endpoint_refuses_to_start_without_a_usable_key_or_socket) {
  TempDir dir;
  Domain domain;
  Service service(domain);
  const auto endpoint_with = [&](const std::string &file, const std::string &address = "127.0.0.1",
                                 std::uint16_t port = 0) {
    UdpEndpointOptions options;
    options.bind_address = address;
    options.port = port;
    options.key_file = file;
    return std::make_unique<UdpEndpoint>(service, options);
  };

  const std::string missing = dir.path() + "/missing.key";
  CHECK((throws_with<std::runtime_error>([&] { endpoint_with(missing); }, missing)));
  const std::string comments = dir.write("comments.key", "# no key yet\n\n");
  CHECK((throws_with<std::runtime_error>([&] { endpoint_with(comments); }, "holds no key")));
  const std::string malformed = dir.write("malformed.key", "1 not-base64!\n");
  CHECK((throws_with<std::runtime_error>([&] { endpoint_with(malformed); }, malformed)));
  const std::string short_key = dir.write("short.key", key_line(1, std::string(16, 'k')));
  CHECK((throws_with<std::runtime_error>([&] { endpoint_with(short_key); }, short_key)));
  const std::string twice = dir.write("twice.key", key_line(1, key_bytes) + key_line(1, key_bytes));
  CHECK((throws_with<std::runtime_error>([&] { endpoint_with(twice); }, twice)));

  const std::string good = dir.write("good.key", key_line(1, key_bytes));
  CHECK((throws_with<std::invalid_argument>([&] { endpoint_with(good, "not-an-address"); }, "not-an-address")));
  std::unique_ptr<UdpEndpoint> first = endpoint_with(good);
  CHECK((throws_with<std::system_error>([&] { endpoint_with(good, "127.0.0.1", first->port()); },
                                        "chassis hold udp endpoint 127.0.0.1")));
}

TEST_MAIN()
