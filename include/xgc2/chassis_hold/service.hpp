#pragma once
// Transport-neutral JSON surface of the chassis HOLD domain (contracts/chassis-hold.md, section 2). The udp.v1
// endpoint and the http.v1 world hosts translate their requests to Service::call and its reply back to their
// wire format; the domain semantics live in Domain.
#include <xgc2/chassis_hold/domain.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace xgc2::chassis_hold {

inline constexpr const char *service_name = "xgc2.chassis.hold";

// Reply status. The values are the udp.v1 status codes (xrpc.md 5.3); http.v1 hosts map the names with their
// usual table. Errors carry the body {"code":"<name>","message":"..."}.
enum class Status : std::uint32_t { ok = 0, invalid_argument = 1, not_found = 2, internal = 8 };
const char *to_string(Status) noexcept;

struct ServiceOptions {
  // How long Engage waits for the native tick to write zero before it replies (never longer than the call
  // deadline). 0 replies at once with the stage the gate has reached.
  std::chrono::milliseconds reply_wait{60};
  // Robot ids per request; "all" and an empty State selection are refused for a larger roster. Keeps replies
  // small enough for one udp.v1 datagram on single-robot hosts and bounded everywhere else.
  std::size_t max_ids = 64;
  std::size_t max_body_bytes = 4096;
};

struct ServiceReply {
  Status status = Status::ok;
  std::string body;  // UTF-8 JSON
};

// The domain must outlive the service. All methods are thread safe.
class Service {
public:
  using Clock = std::chrono::steady_clock;
  using Callback = std::function<void(ServiceReply)>;

  explicit Service(Domain &domain, ServiceOptions options = {});
  // Completes pending engage replies with the state reached so far.
  ~Service();
  Service(const Service &) = delete;
  Service &operator=(const Service &) = delete;

  // The instance id of the domain: what transports put in their server identity.
  const std::string &instance_id() const noexcept;

  // method is one of Describe, State, Engage, Release (not found otherwise); body is the request JSON (an empty
  // body is {}). Engage blocks the calling thread for at most reply_wait; do not call it from the tick thread.
  ServiceReply call(const std::string &method, const std::string &body,
                    Clock::time_point deadline = Clock::time_point::max());

  // The same without blocking: callback runs exactly once, inline for every reply that needs no waiting, or
  // otherwise from the service's own thread when the tick has written zero or the wait has ended. It never
  // runs on the tick thread, so a slow transport cannot delay the control loop. The deadline bounds the engage
  // wait only; a request that arrives after its deadline is still executed (engage is safe, release is fenced).
  void call_async(const std::string &method, const std::string &body, Clock::time_point deadline,
                  Callback callback);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xgc2::chassis_hold
