#pragma once
#include <cstddef>
#include <string>

namespace xgc2::chassis_hold {

constexpr std::size_t max_text_bytes = 160;

// Reduces host text (device errors, fault descriptions), which ends up in JSON replies for remote callers, to
// printable ASCII of bounded length.
std::string clean_text(const std::string &text);

}  // namespace xgc2::chassis_hold
