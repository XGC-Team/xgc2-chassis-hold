#pragma once
// The whole chassis HOLD capability of a host behind one object: the robots' state machines (Domain), the JSON
// service that transports expose (Service), the native tick with the host's zero output, and the host's velocity
// feedback. A host provides its roster, a clock, a ZeroSink and, if it measures velocity, a FeedbackSource; it calls
// tick() in its control loop and admit() for every command. One more object exposes the capability: UdpEndpoint for
// drivers on a LAN (udp.v1), HttpAdapter for world hosts that already serve http.v1. See the README, "Instantiating
// chassis-hold".
#include <xgc2/chassis_hold/domain.hpp>
#include <xgc2/chassis_hold/service.hpp>

#include <string>
#include <vector>

namespace xgc2::chassis_hold {

// One measurement of a robot's velocity.
struct Feedback {
  double linear = 0.0;   // m/s
  double angular = 0.0;  // rad/s
  TimeNs stamp = 0;      // when it was measured, on the domain clock
};

// The host's measured velocities. tick() asks for the latest measurement of every held robot that reports feedback
// and gives it to Domain::observe, which decides when the robot counts as stopped. A host that has no such
// measurement on its tick thread calls Domain::observe itself from wherever the measurement arrives.
class FeedbackSource {
public:
  virtual ~FeedbackSource() = default;
  // The most recent measurement of the robot; false while there is none yet. Called on the tick thread, once per
  // tick for each held robot: return the stored sample, do not wait for a new one.
  virtual bool latest(const std::string &robot_id, Feedback &sample) = 0;
};

struct HoldOptions {
  // boot_held, the clock, wake, the thresholds for stopped, an instance id shared with the transport, max_robots.
  DomainOptions domain;
  // Reply waits and request limits of the JSON service.
  ServiceOptions service;
  // The roster at start. add() and remove() change it while the host runs (a simulator that spawns and deletes
  // models). Ids match [A-Za-z0-9._:-]{1,128}; at most domain.max_robots distinct ids over the life of the object.
  std::vector<std::string> robots;
  // The host's velocity measurements. Set: every robot of the roster reports feedback, so it can reach `stopped`.
  // Null: no robot does, and `stopped` is never reported (HOLD still gates and writes zero).
  FeedbackSource *feedback = nullptr;
};

class ChassisHold {
public:
  // Creates the domain, starts the service thread and adds the roster. Throws std::invalid_argument for options the
  // domain refuses, an invalid or repeated robot id, and std::length_error for a roster beyond max_robots. The sink
  // and the feedback source must outlive the object. Destroy the transport (UdpEndpoint, HttpAdapter users) first.
  ChassisHold(HoldOptions options, ZeroSink &sink);
  ~ChassisHold();
  ChassisHold(const ChassisHold &) = delete;
  ChassisHold &operator=(const ChassisHold &) = delete;

  // What a transport serves.
  Service &service() noexcept { return service_; }
  // The state machines, for everything the facade does not wrap: observe, set_fault, engage, release, state.
  Domain &domain() noexcept { return domain_; }
  const std::string &instance_id() const noexcept { return domain_.instance_id(); }

  // The domain clock: stamp every command with it when the command arrives (or use the receipt stamp of your
  // middleware, if the clock in DomainOptions is built on it).
  TimeNs now() const { return domain_.now(); }
  // Roster changes; both return false when there is nothing to change.
  bool add(const std::string &robot_id);
  bool remove(const std::string &robot_id);

  // For every motion command the host is about to execute; receipt_time_ns is the stamp of its arrival. See
  // Domain::admit.
  bool admit(const std::string &robot_id, TimeNs receipt_time_ns) const { return domain_.admit(robot_id, receipt_time_ns); }

  // The native tick, once per control cycle on the thread that owns device output, after the cycle's commands were
  // written: applies what other threads asked, writes zero for every held robot, applies the results and passes
  // the latest feedback on. Always the same thread (Domain::tick).
  TickStats tick();

private:
  ZeroSink &sink_;
  FeedbackSource *feedback_;
  Domain domain_;
  Service service_;
};

}  // namespace xgc2::chassis_hold
