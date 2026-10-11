#pragma once
// An xrpc http.v1 host over a Unix socket for the http tests: the server runs on its own thread, a client pinned to
// its instance calls it.
#include "check.hpp"
#include "fixtures.hpp"

#include <xgc2/xrpc/http.hpp>
#include <xgc2/xrpc/unix.hpp>

#include <nlohmann/json.hpp>

#include <strings.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>

namespace http_fixtures {

namespace xrpc = xgc2::xrpc;
using json = nlohmann::json;
using std::chrono::milliseconds;
using std::chrono::seconds;

inline std::string header(const xrpc::HttpResponse &response, const std::string &name) {
  for (const auto &entry : response.headers)
    if (strcasecmp(entry.first.c_str(), name.c_str()) == 0)
      return entry.second;
  return std::string();
}

inline json parse(const std::string &text) {
  try {
    return json::parse(text);
  } catch (const std::exception &e) {
    test::fail(__FILE__, __LINE__, std::string("not JSON: ") + e.what() + ": " + text);
    return json();
  }
}

class HttpHost {
public:
  using Handler = std::function<void(xrpc::HttpRequest, xrpc::HttpReply)>;

  // The identity of the host is the instance id the caller passes: a world host shares it with its domain.
  HttpHost(const std::string &instance_id, Handler handler) : instance_(instance_id), handler_(std::move(handler)) {
    socket_ = dir_.path() + "/host.sock";
    xrpc::UnixOptions endpoint;
    endpoint.path = socket_;
    server_ = std::make_unique<xrpc::HttpServer>(
        endpoint, [this](xrpc::HttpRequest request, xrpc::HttpReply reply) { handler_(std::move(request), std::move(reply)); },
        xrpc::HttpLimits{}, xrpc::HttpIdentity{instance_, {"/v1/describe"}});
    loop_ = std::thread([this] { server_->run(); });
  }
  ~HttpHost() {
    server_->request_stop();
    loop_.join();
  }
  HttpHost(const HttpHost &) = delete;
  HttpHost &operator=(const HttpHost &) = delete;

  const std::string &socket() const { return socket_; }
  const std::string &instance() const { return instance_; }
  xrpc::HttpServer &server() { return *server_; }

  // One request on a client of its own, pinned to `pin` (the host's instance when empty).
  xrpc::HttpResponse call(const std::string &verb, const std::string &target, const std::string &body = "{}",
                          milliseconds timeout = seconds(5), const std::string &pin = std::string()) const {
    xrpc::HttpClient client(socket_, xrpc::HttpLimits{}, pin.empty() ? instance_ : pin);
    xrpc::HttpRequest request;
    request.method = verb;
    request.target = target;
    request.body = body;
    request.headers.emplace_back("Content-Type", "application/json");
    return client.call(std::move(request), xrpc::Clock::now() + timeout);
  }

private:
  fixtures::TempDir dir_;  // destroyed after the server: the socket and its lock file are removed with it
  std::string instance_;
  Handler handler_;
  std::string socket_;
  std::unique_ptr<xrpc::HttpServer> server_;
  std::thread loop_;
};

}  // namespace http_fixtures
