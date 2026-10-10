# XGC2 Chassis HOLD

Command gating for a fixed roster of 1–16 chassis. HOLD and release both clear
native command output; release never replays the cached command. Completion
confirms command gating and zero output, not measured physical rest.

| Component | Responsibility | Dependencies |
| --- | --- | --- |
| `HoldState` / `XgcChassisHold::State` | Held/Released region per robot, revision and batch semantics | xgc2-state-machine |
| `HoldRpcHandler` / `XgcChassisHold::Rpc` | HOLD schema, bounded admission, native completion | State, XRPC JSON-HTTP |
| `HoldServer` / `XgcChassisHold::Server` | Own listener and IO thread | Rpc |

Physical drivers use `HoldServer`. Frameworks such as Gazebo compose
`HoldRpcHandler` with their existing XRPC server and resolved host policy.
There is no transport mode flag, per-robot server, or per-robot worker.

```cmake
find_package(XgcChassisHold 0.1.0 CONFIG REQUIRED COMPONENTS Server)
target_link_libraries(my_driver PRIVATE XgcChassisHold::Server)
```

```cpp
#include <xgc2/chassis_hold/server.hpp>
xgc2::chassis_hold::ServerOptions options;
options.socket_path = "/tmp/my-chassis.sock";
options.target_id = "my-chassis";
options.robot_ids = {"robot-1"};
xgc2::chassis_hold::HoldServer hold(std::move(options));
// Native owner: hold.control_tick(zero_output, context), then gate commands
// with hold.held(). Stop that owner before destroying the server.
```

The state-machine runtime starts on the first valid native mutation. Entry
callbacks maintain a compact, allocation-free gate view; state transitions may
allocate in the runtime's bounded diagnostics. Runtime failures close every
gate until restart. All gates and native writes must share one native owner
or the host's synchronization. RPC replies remain retained until native work
ends, including after caller cancellation.
The standalone IO owner sleeps in XRPC's event loop; native completion uses
its coalesced, allocation-free wakeup. There is no periodic completion polling.

Build with CMake, a C++20 compiler, the installed XRPC and state-machine SDKs, and its JSON dependency. JsonCpp >=1.7.4 is used only by the independent test peer. The State and Server public APIs require only C++17 and have no XRPC or JSON headers.
The Rpc integration API uses the C++20 XRPC SDK.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

The protocol is defined in [chassis-hold-v1](contracts/chassis-hold-v1.md).
