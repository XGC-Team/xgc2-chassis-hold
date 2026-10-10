#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace xgc2::chassis_hold {
inline constexpr std::size_t max_robots = 16;
inline constexpr std::size_t queue_capacity = 32;
struct Change { std::uint8_t robot = 0; bool held = false; };
struct Command {
  bool mutate = false;
  std::uint64_t expected_revision = 0;
  std::uint8_t count = 0;
  std::array<Change, max_robots> changes{};
};
enum class Outcome { Snapshot, Applied, Conflict, Invalid, Expired, Cancelled, Failed };
struct Result {
  Outcome outcome = Outcome::Snapshot;
  std::uint64_t revision = 0;
  std::array<bool, max_robots> held{};
  std::array<bool, max_robots> zero_applied{};
};

// One native owner. The existing state_machine runtime owns Held/Released
// transitions; the compact gate view is updated only by state entry callbacks.
// No transport or worker. Queries do not allocate; mutations may allocate in
// the state-machine runtime. A runtime failure closes every gate until restart.
class HoldState {
public:
  using ZeroFn = bool (*)(void *, std::size_t) noexcept;
  explicit HoldState(std::size_t robots);
  ~HoldState();
  HoldState(const HoldState &) = delete;
  HoldState &operator=(const HoldState &) = delete;
  bool held(std::size_t robot = 0) const noexcept;
  bool healthy() const noexcept;
  Result snapshot(Outcome outcome = Outcome::Snapshot) const noexcept;
  Result apply(const Command &, ZeroFn, void *) noexcept;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace xgc2::chassis_hold
