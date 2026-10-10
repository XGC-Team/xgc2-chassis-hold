#pragma once
#include "state.hpp"
#include "endpoint.hpp"
#include <utility>

namespace xgc2::chassis_hold {
struct ServerOptions : Endpoint {
  // An empty instance_id generates a fresh XRPC incarnation at startup.
  std::vector<std::pair<std::string, std::string>> environment;
};
std::vector<std::pair<std::string, std::string>> environment_snapshot(char *const *);
// Standalone process host: composes one handler with one XRPC listener/IO
// thread. Construct before, destroy after the native command loop has stopped.
class HoldServer {
public:
  explicit HoldServer(ServerOptions);
  ~HoldServer();
  HoldServer(const HoldServer &) = delete;
  HoldServer &operator=(const HoldServer &) = delete;
  std::size_t control_tick(HoldState::ZeroFn, void *, std::size_t budget = max_robots) noexcept;
  bool held(std::size_t robot = 0) const noexcept;
  bool healthy() const noexcept;
  std::size_t pending() const noexcept;
  const std::string &instance_id() const noexcept;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace xgc2::chassis_hold
