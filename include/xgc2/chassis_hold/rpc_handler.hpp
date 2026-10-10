#pragma once
#include "state.hpp"
#include "endpoint.hpp"
#include <xgc2/xrpc/http.hpp>
#include <xgc2/xrpc/runtime_policy.hpp>
#include <memory>
#include <string>
#include <vector>

namespace xgc2::chassis_hold {
struct RpcOptions : Endpoint {
  // The host's actually enforced policy, resolved once by its composition root.
  std::shared_ptr<const xrpc::RuntimePolicy> host_policy;
};
// No listener or thread. handle/finish/destruction belong to the host IO owner;
// control_tick/held/healthy belong to one native owner. The native owner must
// be quiescent before destruction; replies retain the host endpoint lease.
class HoldRpcHandler {
public:
  explicit HoldRpcHandler(RpcOptions);
  ~HoldRpcHandler();
  HoldRpcHandler(const HoldRpcHandler &) = delete;
  HoldRpcHandler &operator=(const HoldRpcHandler &) = delete;
  void handle(xrpc::HttpRequest, xrpc::HttpReply);
  void finish();
  // IO owner only, after native quiescence; releases retained request leases.
  void shutdown() noexcept;
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
