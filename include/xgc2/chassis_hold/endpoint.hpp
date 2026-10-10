#pragma once
#include <string>
#include <vector>
namespace xgc2::chassis_hold {
struct Endpoint {
  std::string socket_path;
  std::string instance_id;
  std::string target_id;
  std::vector<std::string> robot_ids;
};
} // namespace xgc2::chassis_hold
