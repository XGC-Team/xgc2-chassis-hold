# XGC2 Chassis HOLD

HOLD stops one robot's chassis from executing motion commands until it is explicitly released. This repository
is the shared C++17 implementation used by the Wheeltec and AgileX chassis drivers and by the Gazebo and xsim
world hosts. The behavior and the wire surface are defined in [chassis-hold-v2](contracts/chassis-hold-v2.md).

| Component | CMake target | Contents | Needs |
| --- | --- | --- | --- |
| `core` | `XgcChassisHold::core` | `Domain` (HOLD state, admission, tick, feedback) and `Service` (the JSON methods `Describe`, `State`, `Engage`, `Release`) | C++17 standard library; nlohmann-json at build time only |

The domain has no transport, no ROS and no state machine. A host owns the transport: drivers expose `Service`
over `udp.v1`, the world hosts over their existing `http.v1` server.

```cmake
find_package(XgcChassisHold 1.0.0 CONFIG REQUIRED COMPONENTS core)
target_link_libraries(my_host PRIVATE XgcChassisHold::core)
```

## Using the domain

```cpp
#include <xgc2/chassis_hold/service.hpp>
using namespace xgc2::chassis_hold;

DomainOptions options;                     // boot_held, linear_eps, angular_eps, stopped_dwell, instance_id, clock
options.clock = [] { return ros::Time::now().toNSec(); };   // the clock of the receipt times passed to admit()
Domain domain(options);
domain.add("ugv-1", /*feedback=*/true);    // a dynamic roster: add() and remove() at any time

struct DeviceZero : ZeroSink {             // the host's device output
  bool write_zero(const std::string &robot_id, std::string &error) override;   // false + reason on failure
} sink;

// Command callback: drop commands of a held robot, and commands received before the last release.
if (!domain.admit("ugv-1", event.getReceiptTime().toNSec())) return;

// Control loop, on the thread that owns device output, after the command callbacks:
domain.tick(sink);                                          // zero for every held robot, every tick
domain.observe("ugv-1", linear_speed, angular_speed, stamp_ns);   // optional trusted velocity feedback

// Remote surface; transports call this, or call_async to reply later without blocking their IO thread:
Service service(domain);                   // reply_wait, max_ids, max_body_bytes
ServiceReply reply = service.call("Engage", R"({"robot_ids":["ugv-1"]})");
service.call_async("Release", body, deadline, [](ServiceReply reply) { /* send it */ });
```

`Service` runs one small thread of its own that completes pending `Engage` replies; it never calls back on the
tick thread. Simulators stamp command enqueue time with `domain.now()`. The host rules (clock, tick ordering,
shutdown order) are in section 4 of the contract.

## Build and test

C++17, CMake 3.10 or newer, nlohmann-json 2.1.1 or newer (Ubuntu 18.04 `nlohmann-json-dev`, later
`nlohmann-json3-dev`). Supported: Ubuntu 18.04, 20.04, 22.04 and 24.04 with g++ 7.5 or newer, amd64 and arm64.
The commands avoid `cmake -S/-B`, `--parallel` and `ctest --test-dir`, which Ubuntu 18.04's CMake 3.10 lacks.

```sh
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DXGC2_CHASSIS_HOLD_WERROR=ON
cmake --build . -- -j3
ctest --output-on-failure
```

The tests are plain programs (`test/check.hpp`): the domain rules (`domain_test`), the JSON service including
its closed schemas and limits (`service_test`) and threads against one domain and one service
(`concurrency_test`). `.xgc2/scripts/run-sanitizers.sh` runs all of them under ThreadSanitizer and under
AddressSanitizer plus UndefinedBehaviorSanitizer (`-DCMAKE_CXX_FLAGS=-fsanitize=...`). The programs in `test/`
also build as a separate project against an installed package (`mkdir b && cd b && cmake ../test`); CI does this
after installing the packages it built.

## Packages

`libxgc2-chassis-hold1` (runtime) and `libxgc2-chassis-hold-dev` (headers, CMake package `XgcChassisHold`,
contracts) for bionic, focal, jammy and noble, built by `.xgc2/scripts/build_debs.sh`. The product metadata is
`.xgc2/product.yml`.
