# Chassis HOLD v2

HOLD stops one robot's chassis from executing motion commands until it is explicitly released. It is not a
world pause, not an entity disable and not a reset: other robots, the simulation clock and sensors keep running.

This is the normative contract of `xgc2-chassis-hold` 1.x: the domain library, the transport-neutral JSON
service and the bindings the hosts expose. It replaces chassis-hold-v1 completely (no v1 routes, sockets,
revision-gated engage or state-machine regions remain).

| Host | Device | Transport | Domain runs in |
| --- | --- | --- | --- |
| Wheeltec, AgileX driver | serial / CAN, bound to the robot | `udp.v1` server inside the driver | the driver process |
| Gazebo world | simulated models | routes on the world host's `http.v1` Unix server | gzserver + world plugin |
| xsim world | simulated entities | routes on xsim's management `http.v1` server | the xsim process |

HOLD needs no agent link, session or workflow. A physical robot is reachable as long as its driver runs and the
LAN delivers datagrams.

## 1. Domain

The domain keeps the HOLD state of a roster of robots (keyed by robot id) for one host instance. It has no
transport, no ROS and no JSON; every operation is thread safe.

### 1.1 State

Per host instance: `instance` (32 lowercase hex digits, random 128 bits at start, a new value on every restart;
a host that already owns an instance id for its transport passes the same one).

Per robot:

| Field | Meaning |
| --- | --- |
| `held` | commands are gated |
| `revision` | unsigned 64-bit, starts at 0, +1 on every held/released transition, monotonic within one instance |
| `stage` | `released`, `gated` (held, zero not confirmed), `zero_written` (held, the latest zero write succeeded), `stopped` (held, zero written, feedback shows rest) |
| `gated_at`, `zero_written_at`, `stopped_at` | host-clock times at which the stage was reached during the current hold; empty when not reached or not held |
| `fault` | host-reported unrecoverable device fault, empty when none |

A robot's state survives removal from the roster: the same id added again (a re-created model) inherits it.

### 1.2 Operations

- **engage**: unconditional and idempotent. A robot that is not held becomes held (`revision + 1`, stage
  `gated`); a held robot is unchanged (same revision, same timestamps). The gate is effective for command
  admission before the call returns. Unknown robots are reported as `unknown_robot`.
- **release** with `expected_instance` and per robot `expected_revision` (compare and set):
  - `expected_instance` is not the current instance: every change is a `conflict` (a stale caller).
  - robot unknown: `unknown_robot`; robot not held: `not_held`; robot has a host fault or
    `expected_revision != revision`: `conflict`; otherwise `released` (`revision + 1`, stage `released`,
    release fence = host-clock time of the release).
  - Partial success is allowed and reported per robot. A replayed release cannot apply twice: the robot is not
    held, or its revision has moved on.
- **state**: read-only snapshot of the requested robots (all when none are named).
- **describe**: instance, roster, which robots report feedback, capabilities.

### 1.3 Command admission

For every motion command it receives, the host asks `admit(robot, receipt_time)`. The answer is false when the
robot is unknown or held, or when the command was received before the robot's last release
(`receipt_time < release fence`): a command queued before the release must not be replayed after it. The host
passes the time the command was *received* (ROS `MessageEvent::getReceiptTime()` on drivers, the enqueue time
in simulators), not the time it is dispatched.

### 1.4 Native tick

The host calls `tick(sink)` every control cycle, after the cycle's command callbacks, on the thread that owns
device output. For every held robot in the roster the domain asks the sink to write a zero motion command, every
tick, not only once, so a command that slipped through between `admit` and the device write is overwritten within
one cycle. The first successful write after an engage moves the stage to `zero_written`. A failed write (the sink
returns false or throws) keeps or puts the stage back to `gated` and is reported as the robot's `fault` text
("zero write failed: ...") until a write succeeds again. The sink is called without any domain lock: a slow or
stuck device never delays engage, release, admit or state.

### 1.5 Feedback and `stopped`

A host with trusted velocity feedback declares it when it adds the robot and reports samples with
`observe(robot, linear, angular, stamp)`. While the robot is held and its zero is written, samples with
`|v| <= linear_eps` and `|w| <= angular_eps` (defaults 0.02 m/s, 0.05 rad/s) continuously for `stopped_dwell`
(default 300 ms) move the stage to `stopped`; a sample above the thresholds (or not a number) moves it back to
`zero_written`. Samples older than the zero write or out of order are ignored; samples further apart than the
dwell break the continuity of the rest. Hosts without trusted feedback never report `stopped`.

### 1.6 Boot, roster and faults

- Boot state is released, or held when the host is configured with `boot_held`: every robot added for the first
  time starts held (revision 1, stage `gated`). Nothing is persisted: a restarted host is a new instance, which
  callers detect through the instance id.
- The roster is dynamic. Adding an id that was removed before restores its state; if it was held, the stage is
  `gated` until the tick has written zero to the new device. A removed or unknown robot is not addressable and
  its commands are refused.
- A host that meets an unrecoverable device fault reports it (`set_fault`): the robot becomes held if it was not,
  cannot be released while the fault is set, and the text is part of its state. The host decides whether to exit;
  drivers exit so that systemd restarts them.

### 1.7 Clock

All times are on the domain clock, a monotonic clock unless the host injects another one. A host that stamps
command receipt with another clock (a ROS driver using `getReceiptTime()` gives the domain `ros::Time::now()`)
must give the domain that clock, otherwise the release fence compares unrelated times. The clock is called with
the domain lock held: it must be cheap and must not block. Times in replies are milliseconds since the domain
was created, for ordering only.

## 2. Service

`Service::call(method, json_body) -> {status, json_body}` is the whole remote surface. Both bindings use it; it
knows nothing about transports, keys or sockets. Bodies are UTF-8 JSON objects (an empty body is `{}`).
Requests are closed schemas: unknown or repeated fields, wrong types, duplicate or invalid robot ids and
numbers that are not unsigned integers are `invalid_argument`. A rejected request changes nothing.

| Method | Request | Reply |
| --- | --- | --- |
| `Describe` | `{}` | `{"instance":"<hex>","robots":["id",...],"feedback":{"id":true},"capabilities":["engage","release","state"]}` |
| `State` | `{"robot_ids":["id",...]}`, or `{}` for all | `{"instance":"..","robots":[{"robot_id","held","revision","stage","gated_at_ms","zero_written_at_ms","stopped_at_ms","fault"}]}` |
| `Engage` | `{"robot_ids":["id",...]}` or `{"all":true}` | `{"instance":"..","robots":[{"robot_id","outcome","held","revision","stage"}]}`, outcome `engaged`, `already_held` or `unknown_robot` |
| `Release` | `{"expected_instance":"<hex>","changes":[{"robot_id","expected_revision"}]}` | the same shape, outcome `released`, `conflict`, `not_held` or `unknown_robot` |

- `robot_id`: 1 to 128 characters of `[A-Za-z0-9._:-]`. `expected_revision`: an unsigned 64-bit integer written
  without fraction or exponent. `expected_instance`: 32 lowercase hex digits.
- In `State`, `*_at_ms` and `fault` are `null` when not applicable. `fault` is the host fault text, or
  `zero write failed: <reason>` while the latest zero write failed.
- A reply entry for `unknown_robot` carries only `robot_id` and `outcome`. `held`, `revision` and `stage` in an
  `Engage` reply are the values when the reply is sent.
- `State` with an unknown robot id is `not_found`; the error details name the unknown ids.
- A stale `expected_instance` is not an error status: the reply is `ok`, carries the current `instance`, and
  every change is a `conflict`.

**Engage reply wait.** The engage itself takes effect at once. The reply is held back for up to `reply_wait`
(default 60 ms, never longer than the call deadline) until the native tick has written zero for every selected
robot, so it usually reports `zero_written`; otherwise it reports `gated` and the caller reads `State` later.
The wait does not depend on the tick thread: a stuck device cannot keep the reply from going out. Services
with asynchronous transports use `call_async`, whose callback runs exactly once, inline for replies that need no
wait and otherwise on the service's own thread, never on the tick thread.

**Limits.** Request body at most 4096 bytes and nesting depth 4; at most 64 robot ids per request
(`max_ids`); `{"all":true}` and an empty `State` selection are refused (`invalid_argument`) when the roster has
more robots than `max_ids`, before anything is changed. A single robot's replies stay below 900 bytes, so they
fit one `udp.v1` datagram.

**Status.** `ok`, `invalid_argument`, `not_found` (unknown method or robot), `internal`; the numeric values are
the `udp.v1` status codes. Errors carry `{"code":"<status>","message":"...","details":{...}}`. Request text echoed
in a message is reduced to printable ASCII.

Example:

```
Engage  {"robot_ids":["ugv-1"]}
     -> {"instance":"3f1c...","robots":[{"robot_id":"ugv-1","outcome":"engaged","held":true,"revision":7,"stage":"zero_written"}]}
Release {"expected_instance":"3f1c...","changes":[{"robot_id":"ugv-1","expected_revision":7}]}
     -> {"instance":"3f1c...","robots":[{"robot_id":"ugv-1","outcome":"released","held":false,"revision":8,"stage":"released"}]}
```

## 3. Bindings

### 3.1 `udp.v1` (physical drivers)

The driver serves the four methods as `xgc2.chassis.hold.v2/Describe`, `/State`, `/Engage` and `/Release` on a
`udp.v1` server bound to the management interface, default port 19530. The server authenticates every datagram
with the robot's HMAC key; the key file defaults to `/etc/xgc2/chassis-hold.key` (one line, `<key_id> <base64
32-byte key>`, mode 0600, installed by the robot setup). A driver refuses to start the server without a key and
exits (`hold_required` defaults to true; with `hold_required=false` the driver runs and reports HOLD as
unavailable in its diagnostics). The transport provides authentication, the deadline, at-most-once execution
inside its reply-cache window and rate limiting; replay beyond the window is made harmless by the revision
compare and set. `Release` is fenced on the `instance` of the HOLD bodies (section 1.1).

### 3.2 `http.v1` (world hosts)

One endpoint per world, on the host's existing `http.v1` Unix server: `GET /v2/chassis/describe`,
`GET|POST /v2/chassis/state`, `POST /v2/chassis/engage`, `POST /v2/chassis/release`; `GET` routes call the
method with `{}`. Status mapping: `ok` 200, `invalid_argument` 400, `not_found` 404, `internal` 500. Simulated
robots register their entity id when their model binds and are removed when it unbinds; the HOLD state of an
unbound id is kept, so a re-created model starts held. HOLD applies while the world is paused.

## 4. Host rules

1. Give the domain the clock of the receipt times you will pass to `admit`; use the domain clock to stamp
   enqueue time in simulators.
2. Call `admit` for every motion command before it reaches the device, with the receipt time.
3. Run `tick` once per control cycle, after the command callbacks, on the thread that owns device output; the
   sink writes a real zero command and reports failure honestly.
4. Declare feedback only when the samples are trusted, and report them with the time they were taken.
5. Stop ticking and destroy the service before the domain.

## 5. What HOLD does not promise

- HOLD is command gating plus zero output. `zero_written` says the device was written, not that the robot has
  stopped; inertia, contact and slope are separate facts, and `stopped` exists only with trusted feedback.
- No persistence and no cross-instance recovery: after a restart the state is the boot state and the instance
  changes.
- No transport-level guarantee beyond what the binding provides; callers do not retry mutations blindly after a
  lost reply, they read `State` and act on the revision.
