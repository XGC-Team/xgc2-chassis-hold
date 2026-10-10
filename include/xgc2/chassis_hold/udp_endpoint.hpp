#pragma once
// The udp.v1 binding of the chassis HOLD service (contracts/chassis-hold.md, section 3.1): what a chassis driver
// links to be reachable from the LAN. No xrpc type appears in this header.
#include <xgc2/chassis_hold/service.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace xgc2::chassis_hold {

struct UdpEndpointOptions {
  // Numeric IPv4 or IPv6 address of the management interface callers use; "::" also accepts IPv4 senders. The
  // reply leaves from the address the kernel routes it from, so on a host with several addresses bind the one
  // that callers talk to.
  std::string bind_address = "0.0.0.0";
  // 0 picks a free port (tests); UdpEndpoint::port() is the port in use.
  std::uint16_t port = 19530;
  // One `<key_id> <base64 of 32 bytes>` per line, `#` starts a comment. Installed by the robot setup, owned by the
  // driver's user, mode 0600.
  std::string key_file = "/etc/xgc2/chassis-hold.key";
};

// Serves Describe, State, Engage and Release of a Service as the udp.v1 methods xgc2.chassis.hold/<Method>:
// one UDP socket and one I/O thread of the udp.v1 server, every datagram authenticated with the HMAC key of its
// key_id. Replies to Engage are completed from the service's thread, so the I/O thread never waits for the native
// tick. The service, and with it the domain, must outlive the endpoint; destroy the endpoint first.
//
// The udp.v1 server instance is the instance id of the service's domain: the id in every reply header, the one a
// caller pins, and the `instance` in the bodies are the same value, and both change when the host restarts.
class UdpEndpoint {
public:
  // Loads the key file, binds the socket and starts serving. Throws std::runtime_error when the key file cannot
  // be read or holds no key, std::invalid_argument for a malformed key file or an address that is not numeric,
  // and std::system_error when the socket cannot be bound; every message names the file or the address. A driver
  // that requires HOLD treats a throw as fatal and exits (systemd restarts it).
  explicit UdpEndpoint(Service &service, UdpEndpointOptions options = {});
  // Stops admitting requests, gives replies that are still pending a short time to leave, closes the socket.
  ~UdpEndpoint();
  UdpEndpoint(const UdpEndpoint &) = delete;
  UdpEndpoint &operator=(const UdpEndpoint &) = delete;

  std::uint16_t port() const noexcept;
  const std::string &instance_id() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xgc2::chassis_hold
