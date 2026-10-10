#include <xgc2/chassis_hold/server.hpp>
#include <cassert>
#include <cstdlib>
#include <filesystem>
#if __cplusplus >= 202002L
#error State and standalone Server consumers must compile without C++20
#endif
int main() {
  char directory[] = "/tmp/hold-cxx17-XXXXXX";
  assert(::mkdtemp(directory));
  xgc2::chassis_hold::ServerOptions options;
  options.socket_path = std::string(directory)+"/hold.sock";
  options.target_id = "cxx17";
  options.robot_ids = {"robot"};
  {
    xgc2::chassis_hold::HoldServer server(options);
    assert(server.healthy() && !server.instance_id().empty());
    assert(!server.held() && server.control_tick(nullptr, nullptr) == 0);
    assert(std::filesystem::exists(options.socket_path));
  }
  assert(!std::filesystem::exists(options.socket_path));
  std::filesystem::remove_all(directory);
}
