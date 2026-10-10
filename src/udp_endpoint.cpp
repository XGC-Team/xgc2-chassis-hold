#include <xgc2/chassis_hold/udp_endpoint.hpp>

#include <xgc2/xrpc/udp.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace xgc2::chassis_hold {
namespace {

namespace udp = xgc2::xrpc::udp;

constexpr std::chrono::milliseconds drain_budget{250};
const char *const methods[] = {"Describe", "State", "Engage", "Release"};

udp::KeyRing load_keys(const std::string &path) {
  udp::KeyRing keys;
  try {
    keys = udp::KeyRing::load_file(path);
  } catch (const std::exception &e) {
    throw std::runtime_error("chassis hold key file " + path + ": " + e.what());
  }
  if (keys.empty())
    throw std::runtime_error("chassis hold key file " + path + " holds no key");
  return keys;
}

// The udp.v1 handler runs on the server's I/O thread: it hands the request to the service, whose callback completes
// the reply later when the engage has to wait for the tick. Reply is move-only and the callback is copied into a
// std::function, hence the shared pointer.
udp::Server::Handler handler_of(Service &service, const std::string &method) {
  return [&service, method](udp::Request request, udp::Reply reply) {
    auto pending = std::make_shared<udp::Reply>(std::move(reply));
    service.call_async(method, request.body, request.deadline, [pending](ServiceReply result) {
      pending->complete(static_cast<udp::Status>(result.status), result.body);
    });
  };
}

}  // namespace

struct UdpEndpoint::Impl {
  Impl(Service &service, const UdpEndpointOptions &options) {
    udp::ServerOptions server_options;
    server_options.bind_address = options.bind_address;
    server_options.port = options.port;
    // The domain validated its id as 32 lowercase hex digits, so this cannot fail.
    server_options.instance = udp::instance_from_hex(service.instance_id());
    udp::KeyRing keys = load_keys(options.key_file);
    try {
      server = std::make_unique<udp::Server>(server_options, std::move(keys));
    } catch (const std::system_error &e) {
      throw std::system_error(e.code(), "chassis hold udp endpoint " + options.bind_address + ":" +
                                            std::to_string(options.port) + ": " + e.what());
    } catch (const std::invalid_argument &e) {
      throw std::invalid_argument("chassis hold udp endpoint " + options.bind_address + ": " + e.what());
    }
    for (const char *method : methods)
      server->add_method(std::string(service_name) + "/" + method, handler_of(service, method));
    instance = server->instance_hex();
    server->start();
  }
  ~Impl() { server->shutdown(drain_budget); }

  std::unique_ptr<udp::Server> server;
  std::string instance;
};

UdpEndpoint::UdpEndpoint(Service &service, UdpEndpointOptions options)
    : impl_(std::make_unique<Impl>(service, options)) {}
UdpEndpoint::~UdpEndpoint() = default;

std::uint16_t UdpEndpoint::port() const noexcept { return impl_->server->port(); }
const std::string &UdpEndpoint::instance_id() const noexcept { return impl_->instance; }

}  // namespace xgc2::chassis_hold
