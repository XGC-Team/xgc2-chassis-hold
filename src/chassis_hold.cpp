#include <xgc2/chassis_hold/chassis_hold.hpp>

#include "text.hpp"

#include <exception>
#include <stdexcept>
#include <utility>

namespace xgc2::chassis_hold {

ChassisHold::ChassisHold(HoldOptions options, ZeroSink &sink)
    : sink_(sink), feedback_(options.feedback), domain_(std::move(options.domain)), service_(domain_, options.service) {
  for (const auto &id : options.robots)
    if (!domain_.add(id, feedback_ != nullptr))
      throw std::invalid_argument("robot id " + clean_text(id) + " is in the roster twice");
}

ChassisHold::~ChassisHold() = default;

bool ChassisHold::add(const std::string &robot_id) { return domain_.add(robot_id, feedback_ != nullptr); }

bool ChassisHold::remove(const std::string &robot_id) { return domain_.remove(robot_id); }

TickStats ChassisHold::tick() {
  const TickStats stats = domain_.tick(sink_);
  if (feedback_ == nullptr)
    return stats;
  for (const RobotState &robot : domain_.state().robots) {
    if (!robot.held || !robot.feedback)
      continue;
    Feedback sample;
    try {
      if (!feedback_->latest(robot.robot_id, sample))
        continue;
    } catch (const std::exception &) {
      continue;  // like a failed write: the host's measurement is not available this cycle
    }
    domain_.observe(robot.robot_id, sample.linear, sample.angular, sample.stamp);
  }
  return stats;
}

}  // namespace xgc2::chassis_hold
