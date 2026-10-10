#pragma once
// Minimal test harness: TEST registers a case, CHECK records a failure and continues, REQUIRE stops the case.
// Thread safe, so worker threads of a test may CHECK.
#include <atomic>
#include <cstdio>
#include <exception>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace test {

struct Case {
  const char *name;
  void (*run)();
};
struct Abort {};

inline std::vector<Case> &cases() {
  static std::vector<Case> all;
  return all;
}
inline std::atomic<int> &failures() {
  static std::atomic<int> count{0};
  return count;
}
struct Register {
  Register(const char *name, void (*run)()) { cases().push_back({name, run}); }
};

inline void fail(const char *file, int line, const std::string &what) {
  std::fprintf(stderr, "%s:%d: %s\n", file, line, what.c_str());
  ++failures();
}

namespace detail {
using std::to_string;
// Enums of the library have to_string overloads that ADL finds; numbers use std::to_string.
template <typename T>
auto show(const T &value, int) -> decltype(std::string(to_string(value))) {
  return to_string(value);
}
template <typename T>
std::string show(const T &value, long) {
  std::ostringstream out;
  out << value;
  return out.str();
}
}  // namespace detail

template <typename T>
std::string show(const T &value) {
  return detail::show(value, 0);
}
inline std::string show(bool value) { return value ? "true" : "false"; }
inline std::string show(const std::string &value) { return "\"" + value + "\""; }

// == that is also right for integers of different signedness.
template <typename A, typename B>
bool same(const A &a, const B &b) {
  if constexpr (std::is_integral<A>::value && std::is_integral<B>::value &&
                std::is_signed<A>::value != std::is_signed<B>::value) {
    if constexpr (std::is_signed<A>::value)
      return a >= 0 && static_cast<std::make_unsigned_t<A>>(a) == b;
    else
      return b >= 0 && a == static_cast<std::make_unsigned_t<B>>(b);
  } else {
    return a == b;
  }
}

template <typename A, typename B>
bool equal(const A &a, const B &b, const char *file, int line, const char *left, const char *right) {
  if (same(a, b))
    return true;
  fail(file, line, std::string("CHECK_EQ(") + left + ", " + right + ") failed: " + show(a) + " vs " + show(b));
  return false;
}

inline int run_all() {
  for (const auto &item : cases()) {
    const int before = failures();
    try {
      item.run();
    } catch (const Abort &) {
    } catch (const std::exception &e) {
      fail(item.name, 0, std::string("uncaught exception: ") + e.what());
    }
    std::printf("%s %s\n", failures() == before ? "ok  " : "FAIL", item.name);
  }
  std::printf("%zu cases, %d failures\n", cases().size(), failures().load());
  return failures() == 0 ? 0 : 1;
}

}  // namespace test

#define TEST(name)                                       \
  static void name();                                    \
  static const test::Register register_##name(#name, name); \
  static void name()

#define CHECK(condition)                                                          \
  do {                                                                            \
    if (!(condition)) test::fail(__FILE__, __LINE__, "CHECK(" #condition ") failed"); \
  } while (0)
#define CHECK_EQ(a, b) test::equal((a), (b), __FILE__, __LINE__, #a, #b)
#define REQUIRE(condition)                                                          \
  do {                                                                              \
    if (!(condition)) {                                                             \
      test::fail(__FILE__, __LINE__, "REQUIRE(" #condition ") failed");              \
      throw test::Abort{};                                                          \
    }                                                                               \
  } while (0)

#define TEST_MAIN() \
  int main() { return test::run_all(); }
