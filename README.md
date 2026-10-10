# XGC2 Chassis HOLD

HOLD stops one robot's chassis from executing motion commands until it is explicitly released. This repository
is the shared C++17 implementation used by the Wheeltec and AgileX chassis drivers and by the Gazebo and xsim
world hosts. The behavior and the wire surface are defined in [chassis-hold](contracts/chassis-hold.md).

| Component | CMake target | Contents | Needs |
| --- | --- | --- | --- |
| `core` | `XgcChassisHold::core` | `Domain` (HOLD state, admission, tick, feedback) and `Service` (the JSON methods `Describe`, `State`, `Engage`, `Release`) | C++17 standard library; nlohmann-json at build time only |
| `udp` | `XgcChassisHold::udp` | `UdpEndpoint`, the udp.v1 server a chassis driver exposes; the `xgc2-hold` command line tool | `core`; the XRPC `udp` component (`XgcXrpc::udp`, OpenSSL libcrypto) |

The domain has no transport, no ROS and no state machine. Drivers expose `Service` through `UdpEndpoint`; the world
hosts call it from their existing `http.v1` server (`POST /v1/call/xgc2.chassis.hold/<Method>`).

```cmake
find_package(XgcChassisHold 0.2.0 CONFIG REQUIRED COMPONENTS core udp)
target_link_libraries(my_driver PRIVATE XgcChassisHold::udp)       # XgcChassisHold::core alone for a world host
```

## Using the domain

```cpp
#include <xgc2/chassis_hold/udp_endpoint.hpp>      // includes service.hpp and domain.hpp
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

// Remote surface. Service is transport neutral; a world host calls it from its http.v1 handler:
Service service(domain);                   // reply_wait, max_ids, max_body_bytes
ServiceReply reply = service.call("Engage", R"({"robot_ids":["ugv-1"]})");
service.call_async("Release", body, deadline, [](ServiceReply reply) { /* send it */ });

// A chassis driver serves it over udp.v1; the endpoint's instance is the domain's instance:
UdpEndpointOptions endpoint_options;       // bind_address 0.0.0.0, port 19530, key_file /etc/xgc2/chassis-hold.key
UdpEndpoint endpoint(service, endpoint_options);   // throws without a usable key file or socket
```

`Service` runs one small thread of its own that completes pending `Engage` replies; it never calls back on the
tick thread. Simulators stamp command enqueue time with `domain.now()`. A host whose transport already owns an
instance id passes it as `DomainOptions::instance_id`. The host rules (clock, tick ordering, shutdown order) are
in section 4 of the contract.

`xgc2-hold --endpoint 192.168.1.20:19530 --key-file /etc/xgc2/chassis-hold.key engage --robot ugv-1` is the
command line client (section 3.3 of the contract).

## Build and test

C++17, CMake 3.10 or newer, nlohmann-json 2.1.1 or newer (Ubuntu 18.04 `nlohmann-json-dev`, later
`nlohmann-json3-dev`), and for the `udp` component the installed XRPC `udp` component. Supported: Ubuntu 18.04,
20.04, 22.04 and 24.04 with g++ 7.5 or newer, amd64 and arm64. `-DXGC2_CHASSIS_HOLD_BUILD_UDP=OFF` builds `core`
alone. The commands avoid `cmake -S/-B`, `--parallel` and `ctest --test-dir`, which Ubuntu 18.04's CMake 3.10 lacks.

```sh
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DXGC2_CHASSIS_HOLD_WERROR=ON -DCMAKE_PREFIX_PATH=<xrpc prefix>
cmake --build . -- -j3
ctest --output-on-failure
```

The tests are plain programs (`test/check.hpp`): the domain rules (`domain_test`), the JSON service including
its closed schemas and limits (`service_test`), threads against one domain and one service (`concurrency_test`),
the udp.v1 endpoint on loopback with a fake tick thread and the XRPC client (`udp_endpoint_test`) and the
`xgc2-hold` tool (`hold_cli_test`). `.xgc2/scripts/run-sanitizers.sh` runs all of them under ThreadSanitizer and
under AddressSanitizer plus UndefinedBehaviorSanitizer (`-DCMAKE_CXX_FLAGS=-fsanitize=...`). The programs in
`test/` also build as a separate project against an installed package (`mkdir b && cd b && cmake ../test`); CI
does this after installing the packages it built.

## Packages

`libxgc2-chassis-hold1` (runtime: both libraries and `/usr/bin/xgc2-hold`) and `libxgc2-chassis-hold-dev`
(headers, CMake package `XgcChassisHold`, the contract) for bionic, focal, jammy and noble, built by
`.xgc2/scripts/build_debs.sh`. The runtime package depends on the XRPC udp runtime package. The product metadata is
`.xgc2/product.yml`.
