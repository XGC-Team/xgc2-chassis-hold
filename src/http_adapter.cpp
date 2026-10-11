#include <xgc2/chassis_hold/http_adapter.hpp>

#include <nlohmann/json.hpp>

#include <string>
#include <utility>

namespace xgc2::chassis_hold {
namespace {

using nlohmann::json;
namespace xrpc = xgc2::xrpc;

// "/v1/call/" + service_name + "/" (the test of the adapter checks it).
const std::string prefix = "/v1/call/xgc2.chassis.hold/";

int http_status(Status status) {
  switch (status) {
    case Status::ok: return 200;
    case Status::invalid_argument: return 400;
    case Status::not_found: return 404;
    case Status::internal: return 500;
  }
  return 500;
}

xrpc::HttpResponse response_of(const ServiceReply &reply) {
  if (reply.status != Status::ok)
    return xrpc::http_error(http_status(reply.status), to_string(reply.status), reply.message);
  xrpc::HttpResponse response;
  response.status = 200;
  response.headers.emplace_back("Content-Type", "application/json");
  response.body = reply.body;
  return response;
}

}  // namespace

HttpAdapter::HttpAdapter(Service &service) : service_(service) {}

const char *HttpAdapter::route_prefix() noexcept { return prefix.c_str(); }

bool HttpAdapter::handle(const xrpc::HttpRequest &request, const xrpc::HttpReply &reply) const {
  if (request.target.compare(0, prefix.size(), prefix) != 0)
    return false;
  if (request.method != "POST") {
    xrpc::HttpResponse refused = xrpc::http_error(405, "invalid_argument", "POST is required");
    refused.headers.emplace_back("Allow", "POST");
    reply.complete(std::move(refused));
    return true;
  }
  const std::string method = request.target.substr(prefix.size());
  if (method.find('?') != std::string::npos) {
    reply.complete(xrpc::http_error(400, "invalid_argument", "a query string is not accepted"));
    return true;
  }
  // The copy keeps the host's admission of the call until the reply has been sent.
  const xrpc::HttpReply pending = reply;
  service_.call_async(method, request.body, request.deadline,
                      [pending](ServiceReply result) { pending.complete(response_of(result)); });
  return true;
}

std::string HttpAdapter::capability_json() const {
  json entry;
  entry["name"] = service_name;
  entry["entities"] = service_.entities();
  return entry.dump();
}

}  // namespace xgc2::chassis_hold
