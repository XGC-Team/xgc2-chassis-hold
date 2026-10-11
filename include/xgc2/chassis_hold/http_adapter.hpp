#pragma once
// The http.v1 binding of the chassis HOLD service (contracts/chassis-hold.md, section 3.2): what a world host that
// already runs an xgc2::xrpc::HttpServer (Gazebo, XSIM) links to serve the capability xgc2.chassis.hold next to its own
// routes. The adapter owns no socket, thread or routing table; the host's request handler offers it every request.
#include <xgc2/chassis_hold/service.hpp>

#include <xgc2/xrpc/http.hpp>

#include <string>

namespace xgc2::chassis_hold {

class HttpAdapter {
public:
  // The service, and with it the domain, must outlive the adapter.
  explicit HttpAdapter(Service &service);

  // Every method of the capability is served at POST <route_prefix()><Method>, Method one of Describe, State, Engage,
  // Release: "/v1/call/xgc2.chassis.hold/".
  static const char *route_prefix() noexcept;

  // Call it first in the host's request handler, with the request and the reply it was given:
  //
  //   if (hold_http.handle(request, reply)) return;   // ... then the host's own routes
  //
  // False, with request and reply untouched, when the target is not under route_prefix(): the host serves it.
  // True when the adapter took the request, and the reply is completed exactly once: at once for Describe, State and
  // errors, and from the service's thread, never the host's I/O thread, for an Engage or Release that waits for the
  // native tick. The request deadline bounds the wait. Bodies and statuses are those of Service; errors use the
  // XRPC envelope {"error":{"code","message"}} with 400 invalid_argument, 404 not_found, 405 for a method other than
  // POST (code invalid_argument, header Allow: POST) and 500 internal. The instance header
  // is the host's: give the domain the instance id of the host's HttpIdentity (DomainOptions::instance_id), so
  // that the header, the pin of a caller and the instance in the bodies are one value.
  bool handle(const xgc2::xrpc::HttpRequest &request, const xgc2::xrpc::HttpReply &reply) const;

  // The entry of this capability for the facts of the host's describe reply (xrpc.md, 4b), with the robots in the
  // roster now: {"name":"xgc2.chassis.hold","entities":["robot-a","robot-b"]}.
  std::string capability_json() const;

private:
  Service &service_;
};

}  // namespace xgc2::chassis_hold
