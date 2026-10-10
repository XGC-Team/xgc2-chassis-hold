# Chassis HOLD v1

The native output owner implements one fixed roster of 1–16 robot IDs and a
32-slot queue. The SDK owns HTTP parsing, limits, binding and endpoint leases.
There is no per-robot listener or management worker. The composed `HoldRpcHandler`
shares the app's actual ServiceRef instance, listener, IO owner and
resolved RuntimePolicy. It does not resolve another environment snapshot.

GET `/v1/chassis/descriptor` is the declared discovery route and returns the
complete `chassis.hold`, API `1`, `http.v1` ServiceRef, roster and limits.
GET `/v1/chassis/runtime-policy` reports the actually enforced host policy.
GET `/v1/chassis/hold` runs a bounded observation on the native control owner.

POST `/v1/chassis/hold` takes exactly:

```json
{"expected_revision":0,"changes":[{"robot_id":"robot-1","held":true}]}
```

Revision is an unsigned JSON integer; changes contain 1–16 unique known IDs
and booleans. Unknown fields, duplicates and integer-shaped floats are invalid.
The native owner validates the whole batch and CAS before writes, masks every
selected robot, and applies zero to every native command output. It reports
`stage:applied` only after all zero sinks return successfully. Failed native
writes leave the entire selected batch held and advance the visible revision;
the result states each `zero_applied` outcome. No rollback is claimed.

Release also clears cached commands and applies zero before opening the gate;
it never replays a command accepted before or during HOLD. All command
acceptance and native output writes must share the embedding app's gate with
the control executor. The pure `HoldState` class is exclusively owned, not internally
thread safe. A Gazebo RT producer uses a try gate and skips contention.

Completion is `command-gated`, with `physical_stop_confirmed:false`: native
output zero is confirmed; inertia, contact and measured physical rest are
separate facts. A missing output owner is a native write failure, not success.
Receipts report revision, every robot's actual held state, and per-change zero
completion. There is no durable state or cross-instance recovery promise.

The domain body limit is 4096 bytes and queued admission lasts at most 250 ms,
even when the host app permits larger general requests. A caller leaving
does not undo an executing write. The host retains admitted work until native
completion and releases reply leases after quiescence. Do not automatically
retry mutations after a lost response; inspect the bound native revision/state.

`HoldState` uses the existing xgc2-state-machine runtime, with one two-state
region per chassis. There is no alternative hand-written transition engine.
The native gate view is allocation-free; a mutation may allocate in bounded
runtime diagnostics. Runtime failure closes every gate until process restart.
