# Chassis HOLD

HOLD stops one robot's chassis from executing motion commands until it is explicitly released. It is not a
world pause, not an entity disable and not a reset: other robots, the simulation clock and sensors keep running.

This is the normative contract of `xgc2-chassis-hold`: the domain library (one state machine per robot), the
transport-neutral JSON service, the facade a host instantiates, the exposures of the service on `udp.v1` and
`http.v1`, and the `xgc2-hold` command line tool. HOLD is domain semantics only: a caller reaches it as the
generic XRPC capability `xgc2.chassis.hold` and needs nothing but the JSON bodies of section 2.

| Host | Device | Exposure | Domain runs in |
| --- | --- | --- | --- |
| Wheeltec driver | serial, bound to the robot | `udp.v1` server inside the driver (`UdpEndpoint`) | the driver process |
| Scout (AgileX) driver | CAN SDK, bound to the robot | `udp.v1` server inside the driver (`UdpEndpoint`) | the driver process |
| Gazebo world | simulated models | `http.v1` on the world host's XRPC server (`HttpAdapter`) | gzserver + world plugin |
| XSIM world | simulated entities | `http.v1` on the XRPC server of the xsim process (`HttpAdapter`) | the xsim process |

The four hosts are four shapes of one library: each provides its roster, a clock, a zero output and, where it
measures velocity, a feedback source, calls `tick` once per control cycle, and exposes the same service.
HOLD needs no agent link, session or workflow. A physical robot is reachable as long as its driver runs and the
LAN delivers datagrams.

## 1. Domain

The domain keeps the HOLD state of a roster of robots (keyed by robot id) for one host instance. It has no
transport, no ROS and no JSON. The state of every robot is a state machine on `xgc2-state-machine`.

### 1.1 State

Per host instance: `instance` (32 lowercase hex digits, not all zero; random 128 bits at start, a new value on
every restart). A host whose transport already has an instance id gives it to the domain, and the udp.v1
endpoint gives the id of the domain to its server, so that transport and domain have one identity.

Per robot:

| Field | Meaning |
| --- | --- |
| `held` | commands are gated |
| `revision` | unsigned 64-bit, starts at 0, +1 on every held/released transition, monotonic within one instance |
| `stage` | `released`, `gated` (held, zero not confirmed), `zero_written` (held, the latest zero write succeeded), `stopped` (held, zero written, feedback shows rest) |
| `gated_at`, `zero_written_at`, `stopped_at` | host-clock times at which the stage was reached during the current hold; empty when not reached or not held |
| `fault` | host-reported unrecoverable device fault, empty when none |

A robot's state survives removal from the roster: the same id added again (a re-created model) inherits it.

### 1.2 The state machine

```
Released --engage--> Held(Gated)
Held --release [expected instance, expected revision, no fault]--> Released

Held:  Gated --zero_written--> ZeroWritten --motion_stopped--> Stopped
       ZeroWritten, Stopped --zero_failed--> Gated          Stopped --motion_resumed--> ZeroWritten
       Gated, ZeroWritten, Stopped --fault--> Faulted --clear_fault--> Gated
       ZeroWritten, Stopped, Faulted --rebound--> Gated     (a held robot was added again, for a new device)
```

Each robot has its own machine. Its events are `engage`, `release`, `zero_written`, `zero_failed`,
`motion_stopped`, `motion_resumed`, `fault`, `clear_fault` and `rebound`; every other input of the domain is turned
into one of them. `Faulted` is a held state whose zero is not trusted: it reports the stage `gated` and, while it
lasts, a release is refused.

Threads. The machines run only inside `tick` (`xgc2-state-machine` allows `update` on one thread, which here is the
host's native tick thread; `tick` throws `std::logic_error` when called from a second thread). Every other thread
posts events: a transport's I/O thread calls `engage`, `release`, `set_fault` and `observe`, which queue events and
return. The tick applies them in the order they were queued, one robot at a time, and publishes the result
atomically: a reader never sees half a transition. A machine takes at most 4096 queued events; a robot whose queue
overflows, or whose machine fails, stays held for good with `fault` text `hold state machine failed: ...`. The
host decides to restart.

### 1.3 Operations

- **engage**: unconditional and idempotent. A robot that is not held becomes held (`revision + 1`, stage
  `gated`); a held robot is unchanged (same revision, same timestamps). Command admission is closed before the
  call returns, without waiting for the tick; `state` and the reply show the robot as held and `gated`, with the
  revision the tick will assign, until the machine has applied the engage. Unknown robots are reported as
  `unknown_robot`. Two engages that race get one `engaged` and `already_held` for the rest.
- **release** with `expected_instance` and per robot `expected_revision` (compare and set), decided by the
  machine of each robot when the tick applies it, in queue order:
  - `expected_instance` is not the current instance: every change is a `conflict` (a stale caller).
  - robot unknown: `unknown_robot`; robot not held: `not_held`; robot has a host fault or
    `expected_revision != revision`: `conflict`; otherwise `released` (`revision + 1`, stage `released`,
    release fence = host-clock time at which the tick applied it, which is also when admission opens again).
  - Partial success is allowed and reported per robot. A replayed release cannot apply twice: the robot is not
    held, or its revision has moved on. An engage that was queued behind a release holds the robot again, and
    admission stays closed throughout.
- **withdraw**: a release is a request to the tick. A caller that cannot wait any longer withdraws the changes the
  tick has not applied: each change is applied or withdrawn, never both, and a withdrawn change is not applied
  when the tick gets to it. Withdrawn changes are reported as `expired`.
- **state**: read-only snapshot of the requested robots (all when none are named).
- **describe**: instance, roster, which robots report feedback, capabilities.

### 1.4 Command admission

For every motion command it is about to execute, the host asks `admit(robot, receipt_time)`. The answer is false
when the robot is unknown or held (an engage counts from the moment it arrives, not from the moment the tick
applies it), or when the command was received before the robot's last release
(`receipt_time < release fence`): a command queued before the release must not be replayed after it.

The host passes the time the command was *received*, on the domain clock, not the time it is dispatched: the
receipt stamp of its middleware (ROS `MessageEvent::getReceiptTime()` on drivers), or the time at which the message
entered its input queue (`domain.now()` taken at enqueue, in simulators). A command that waited in a queue while
the robot was held, or that arrived while the release was being applied, is refused although it is executed after
the release, so a host does not drain its queues before it releases. `admit` is wait free apart from the lookup of
the robot.

### 1.5 Native tick

The host calls `tick(sink)` every control cycle, after the cycle's command callbacks, on the thread that owns
device output, and always on the same thread. A tick
1. applies the events that arrived since the last one (engage, release, fault, feedback reports);
2. asks the sink to write a zero motion command for every held robot in the roster, every tick, not only once, so
   a command that slipped through between `admit` and the device write is overwritten within one cycle;
3. turns the results into events and applies them: the first successful write after an engage moves the stage to
   `zero_written`; a failed write (the sink returns false or throws) keeps or puts the stage back to `gated` and is
   reported as the robot's `fault` text (`zero write failed: ...`) until a write succeeds again.

The sink is called without any lock of the domain: a slow or stuck device never delays engage, admit or state. It
does delay the tick, and with it the application of releases; a release that the tick does not reach is withdrawn
by the service (section 2) and never applied late.

A host whose loop sleeps gives the domain a `wake` callback (`DomainOptions::wake`). It is called, once per call
and without any lock of the domain, whenever something needs the tick: an engage, a release, a fault or its
clearing, a held robot that is added again or added held, feedback that moves a stage. The host notifies its loop
from it; it needs no other wake-up code. A world that is paused still needs ticks to apply a release: a host that
must release while paused ticks from a thread of its own.

### 1.6 Feedback and `stopped`

A host with trusted velocity feedback declares it when it adds the robot and reports samples with
`observe(robot, linear, angular, stamp)`, or gives the facade a `FeedbackSource` that it polls once per tick for
every held robot. While the robot is held and its zero is written, samples with `|v| <= linear_eps` and
`|w| <= angular_eps` (defaults 0.02 m/s, 0.05 rad/s) continuously for `stopped_dwell` (default 300 ms) move the
stage to `stopped`; a sample above the thresholds (or not a number) moves it back to `zero_written`. Samples older
than the zero write or out of order are ignored; samples further apart than the dwell break the continuity of the
rest. Hosts without trusted feedback never report `stopped`.

### 1.7 Boot, roster and faults

- Boot state is released, or held when the host is configured with `boot_held`: every robot added for the first
  time starts held (revision 1, stage `gated`), from the moment it is in the roster. Nothing is persisted: a
  restarted host is a new instance, which callers detect through the instance id.
- The roster is dynamic. Adding an id that was removed before restores its state; if it was held, the stage is
  `gated` until the tick has written zero to the new device. A removed or unknown robot is not addressable and
  its commands are refused. A domain keeps the state of at most 1024 distinct robot ids over its life
  (`max_robots`, one domain per process): removed ids stay counted, and `add` throws `std::length_error` beyond it.
- A host that meets an unrecoverable device fault reports it (`set_fault`): the robot becomes held if it was not,
  in the same step of the queue, so that no release can come between the engage and the fault; it cannot be
  released while the fault is set, and the text is part of its state. The host decides whether to exit; drivers
  exit so that systemd restarts them.

### 1.8 Clock

All times are on the domain clock, `steady_clock` unless the host injects another one. A host that stamps command
receipt with another clock (a ROS driver using `getReceiptTime()` gives the domain `ros::Time::now()`) must give
the domain that clock, otherwise the release fence compares unrelated times. Simulators keep the default clock and
stamp enqueue time with `now()`: a simulation clock that stands still while the world is paused cannot order a
command before a release applied in the same instant. The clock is called from every thread that uses the domain,
sometimes with a lock held: it must be cheap and must not block. Times in replies are milliseconds since the
domain was created, for ordering only.

## 2. Service

`Service::call(method, json_body) -> {status, json_body}` is the whole remote surface. Every exposure uses it; it
knows nothing about transports, keys or sockets. The method names are the bare `Describe`, `State`, `Engage` and
`Release`; a caller addresses them as `xgc2.chassis.hold/<Method>`. Bodies are UTF-8 JSON objects (an empty body
is `{}`). Requests are closed schemas: unknown or repeated fields, wrong types, duplicate or invalid robot ids and
numbers that are not unsigned integers are `invalid_argument`. A rejected request changes nothing.

| Method | Request | Reply |
| --- | --- | --- |
| `Describe` | `{}` | `{"instance":"<hex>","robots":["id",...],"feedback":{"id":true},"capabilities":["engage","release","state"]}` |
| `State` | `{"robot_ids":["id",...]}`, or `{}` for all | `{"instance":"..","robots":[{"robot_id","held","revision","stage","gated_at_ms","zero_written_at_ms","stopped_at_ms","fault"}]}` |
| `Engage` | `{"robot_ids":["id",...]}` or `{"all":true}` | `{"instance":"..","robots":[{"robot_id","outcome","held","revision","stage"}]}`, outcome `engaged`, `already_held` or `unknown_robot` |
| `Release` | `{"expected_instance":"<hex>","changes":[{"robot_id","expected_revision"}]}` | the same shape, outcome `released`, `conflict`, `not_held`, `unknown_robot` or `expired` |

- `robot_id`: 1 to 128 characters of `[A-Za-z0-9._:-]`. `expected_revision`: an unsigned 64-bit integer written
  without fraction or exponent. `expected_instance`: 32 lowercase hex digits.
- In `State`, `*_at_ms` and `fault` are `null` when not applicable. `fault` is the host fault text, or
  `zero write failed: <reason>` while the latest zero write failed.
- A reply entry for `unknown_robot` carries only `robot_id` and `outcome`. `held`, `revision` and `stage` in an
  `Engage` or `Release` reply are the values when the reply is sent. The `outcome` of an `Engage` is the decision
  of the machine if the tick applied the engage before the reply went out, and otherwise what the gate decided when
  the engage arrived: `engaged` for the call that closed the gate, `already_held` for a robot that was held.
- `State` with an unknown robot id is `not_found`; the error details name the unknown ids.
- A stale `expected_instance` is not an error status: the reply is `ok`, carries the current `instance`, and
  every change is a `conflict`.

**Engage reply wait.** The engage itself takes effect at once. The reply is held back for up to `reply_wait`
(default 60 ms, never longer than the call deadline) until the native tick has written zero for every selected
robot, so it usually reports `zero_written`; otherwise it reports `gated` and the caller reads `State` later. A
stuck device cannot keep the reply from going out.

**Release wait.** A release needs the tick. The reply is held back until the tick has applied every change, for at
most `release_wait` (default 2000 ms, never longer than the call deadline). What the tick has not applied by then is
withdrawn and reported as `expired`, with the robot's state as it is: still held. A withdrawn change is never
applied afterwards, so the reply stays true. The caller may release again (with the revision it reads from
`State`). With a tick that does not run, a release always comes back `expired`.

Services with asynchronous transports use `call_async`, whose callback runs exactly once, inline for replies that
need no wait and otherwise on the service's own thread, never on the tick thread.

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

## 3. Exposures

### 3.1 `udp.v1` (physical drivers)

`UdpEndpoint` serves the four methods as `xgc2.chassis.hold/Describe`, `/State`, `/Engage` and `/Release` on a
`udp.v1` server bound to the management interface, default port 19530. Every datagram is authenticated with the
HMAC key of its key id; the key file defaults to `/etc/xgc2/chassis-hold.key` (one `<key_id> <base64 of 32
bytes>` per line, `#` comments, mode 0600, installed by the robot setup). The endpoint refuses to start, by
throwing, when the key file is missing, malformed or holds no key, and when the socket cannot be bound; a driver
that requires HOLD (`hold_required`, default true) exits then, with `hold_required=false` it runs and reports
HOLD as unavailable in its diagnostics.

The server instance of the transport is the instance of the domain: the id in every reply header, the id a
caller pins and the `instance` in the bodies are one value. `Release` is fenced on it in the body and may be
pinned in the header as well. The transport provides authentication, the deadline, at-most-once execution inside
its reply cache (120 s, 1024 replies) and a rate limit per source address (50 requests per second, burst 100);
a replay beyond the cache is made harmless by the revision compare and set. A reply larger than one datagram
(1200 bytes) is replaced by `resource_exhausted`: the request had its effect, and the caller reads `State` for
fewer robots at a time. The replies for one robot always fit.

### 3.2 `http.v1` (world hosts)

A world host serves the methods on its existing `http.v1` XRPC server by the generic method addressing
(xrpc.md, 4b): `POST /v1/call/xgc2.chassis.hold/Describe`, `/State`, `/Engage` and `/Release` with the JSON body of
section 2; the reply body is the JSON result. `HttpAdapter` is the part of the host that does it: the host's
request handler offers it every request first (`handle(request, reply)`), it takes the ones under
`/v1/call/xgc2.chassis.hold/` and the host serves the rest. Replies are completed from the service's thread, so
the host's I/O thread never waits for the tick.

Status mapping: `ok` 200, `invalid_argument` 400, `not_found` 404, `internal` 500, and 405 with `Allow: POST` for
another verb; an error is the XRPC envelope around the error object of section 2: `{"error":{"code","message"
[,"details"]}}`. The instance fence is the host's: it requires the instance header and answers `conflict` (409) for
another instance before the adapter sees the request. For that to mean the same instance as the bodies, the
domain is created with the instance id of the host's `HttpIdentity` (`DomainOptions::instance_id`).

The host lists the capability in its describe facts as `{"capabilities":[{"name":"xgc2.chassis.hold",
"entities":["<id>",...]}]}`; `HttpAdapter::capability_json()` gives the entry with the robots in the roster now,
so that a caller can find the host that serves a given robot. Simulated robots are added to the roster when their
model binds and removed when it unbinds; the HOLD state of an unbound id is kept, so a re-created model starts
held.

### 3.3 `xgc2-hold`

```
xgc2-hold [--endpoint HOST:PORT] [--key-file PATH] [--key-id N] [--instance HEX] [--timeout-ms N] <command>
  describe
  state   [--robot ID]...
  engage  --robot ID... | --all
  release --robot ID[=REVISION]...
```

A client of the udp.v1 endpoint for tests and for the person at the robot. The endpoint defaults to
`127.0.0.1:19530` (a name is resolved once, IPv4 first), the key file to `/etc/xgc2/chassis-hold.key`, the key id
to the key of a file that holds exactly one, the timeout to 2000 ms. `--instance` pins the call to an instance
of the host and, for `release`, is the fenced instance. A `release` that names a robot without `=REVISION` (or no
`--instance`) first reads `State` and uses what it reported; with both given it makes the one call.

Each run prints one JSON object: `disposition` (`response_received`, `outcome_unknown`, `not_sent`), `status`,
`instance` of the host that answered, `attempts`, and the `reply` or a `message`. The exit code follows the
disposition: 0 a reply was received and the request did what it asked (every robot engaged, or released), 1 a
reply was received and it is an error status or reports a robot that was not engaged or released (a release that
came back `expired` is one), 2 the outcome is unknown (a request was sent and no valid reply arrived, including a
pin on another instance), 3 nothing was sent, 64 a usage error, 78 a key file or an address that cannot be used.

## 4. The facade and the host rules

`ChassisHold` is the domain and the service behind one object. A host fills `HoldOptions` with its roster, the
domain options (`boot_held`, the clock, `wake`, `instance_id`, the thresholds of `stopped`), the service options and
an optional `FeedbackSource`, passes its `ZeroSink`, and gets `tick()`, `admit()`, `add()`, `remove()`, `now()` and
`service()`; `domain()` is there for everything else. One more object exposes `service()`: `UdpEndpoint` for a
driver, `HttpAdapter` for a world host. The README shows the four hosts.

1. Give the domain the clock of the receipt times you will pass to `admit`; in simulators use the domain clock and
   stamp enqueue time with `now()`.
2. Call `admit` for every motion command before it reaches the device, with the time it was received.
3. Run `tick` once per control cycle, after the command callbacks, on the one thread that owns device output; the
   sink writes a real zero command and reports failure honestly. Connect `wake` to your loop if it sleeps.
4. Declare feedback only when the samples are trusted, and report them with the time they were taken.
5. Stop ticking, destroy the transport (`UdpEndpoint`, or stop serving requests to `HttpAdapter`), then the
   facade. Destroying the service completes the replies that are still waiting: engages with the state reached so
   far, releases as `expired`.

## 5. What HOLD does not promise

- HOLD is command gating plus zero output. `zero_written` says the device was written, not that the robot has
  stopped; inertia, contact and slope are separate facts, and `stopped` exists only with trusted feedback.
- No persistence and no cross-instance recovery: after a restart the state is the boot state and the instance
  changes.
- No transport-level guarantee beyond what the exposure provides; callers do not retry mutations blindly after a
  lost reply, they read `State` and act on the revision.
- A release is applied by the tick: with a stuck or stopped tick the robot stays held, which is the safe direction.
