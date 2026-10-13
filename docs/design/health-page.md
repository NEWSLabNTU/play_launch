# Health page (Tier 1 observability)

**Status: W0–W7 done (2026-10-13); roadmap: phase 87.** W7: `health/startup.rs` records
each member's spawn, load start and load (timestamped as the actor emits
them) and, from the interceptor, each node's first endpoint and first
publish; `GET /api/startup`, `startup_timeline.json` at startup complete,
a Gantt on the Health page with the last-ready member highlighted, and a
`never_ready` warning at 60 s. W6: `health/graph.rs`
reads the graph every 10 s on `spawn_blocking` and reports nodes the run
expects and the graph lacks, nodes on the domain the run did not start,
names two nodes share, and subscriptions whose topic is a typo of a
published one; `/api/graph` runs off the async workers and the page fetches
it only while something shows it. W5:
`interception/latency.rs` matches each stamped take to its publish by
stamp and keeps a 64-bucket log histogram per subscription (run, and the
last 10–20 s); Entities shows p99/p50 and Health a `delivery_latency`
warning. W4: `health/logs.rs`
reads each member's `out`/`err` growth every 2 s (≤1 MiB per file per tick,
counted beyond), classifies rcutils levels, collapses repeats into
signatures, and reports floods, error bursts and errors new after the first
minute; `list_members` reads only the last 64 KiB of stderr. W3: every exit carries
its signal, uptime and an OOM verdict from the member's cgroup; the
coordinator keeps the last 20 per member; Health reports crash loops (≥3
exits in 5 min), OOM kills, members that crashed and stayed down (not ones a
user stopped), and failures; node cards show the last exit. W0 found that the
diagnostics subscriber was *not* the ~30% (see "Prerequisite" below for what
it was); W1's per-subsystem accounting is in (`util/self_cost.rs`, logged and
shown on the page); W2's book, API (`/api/health/issues`, its `/stream`) and
page are in, fed by the Entities flags and self cost. The Tier 1 items of
`docs/design/big-system-observability.md`: a Health page listing what is
wrong now, fed by crash-loop tracking, a startup timeline, delivery latency,
log statistics, a graph-versus-model diff, and play_launch's own cost. Tier 2
and 3 there (TF, time, parameter drift, resource trends, host checks, chain
latency, discovery load, network) are optional and not designed here.

## What exists, and the gaps

From a read of the code (2026-10-13):

| Need | Today | Gap |
|---|---|---|
| Exit code and signal per exit | `save_status` writes the code to `node/<n>/status`, overwritten each exit; `StateEvent::Exited{exit_code}`; `MemberState::Stopped` drops the code | **no signal, no timestamp, no history**; `/api/nodes` has none of it |
| Crash streak | `respawn_count` resets after 30 s up (`regular_node_actor.rs:99,324`); `restart_count` reaches the UI only for composables | no "N exits in 5 min" view |
| OOM kill | `cgroup.rs::report_memory_events` reads `oom_kill`, **only at shutdown** | not live, not per exit |
| Spawn / load / lifecycle times | private `Instant`s, `#[serde(skip)]`, or log lines | **nothing queryable** |
| Startup complete | `HealthSummary::startup_complete()` polled every 100 ms | logged only |
| Log reading | log viewer polls a file every 500 ms; `list_members` reads the **whole** stderr file per member per `/api/nodes` call (`coordinator/mod.rs:51`) | no rates, no levels server-side, cost grows with the file |
| `/rosout` | not subscribed (and should not be: per-message subscription) | the interceptor already counts `/rosout` publishes per node |
| Graph | `build_graph_snapshot` = 1 + 2×topics rcl queries, synchronous on a tokio worker, re-fetched after every state event (500 ms debounce) | foreign nodes need only `get_node_names`, which nothing calls |
| Model in the web server | `Arc<SystemModel>` exists in `up::play`, not passed on | the web server cannot say what *should* run |
| Self cost | one tokio runtime, every worker named `play_launch-worker`; rclrs executor on an unnamed thread | no per-subsystem attribution |

## Architecture

One new module, `health/`, owning a `HealthBook`. Sources feed it facts;
it keeps the current issues and a bounded history; the web server reads a
1 Hz snapshot of it over a `watch` channel, the way the Entities page does.

```
 StateEvents (broadcaster) ─┐   member spawned / exited (code, signal, uptime) / loaded / lifecycle
 Interception snapshot ─────┤   silent, stale, QoS, deficit, delivery latency, node identities
 LogWatcher task ───────────┼─► HealthBook ──► watch<Arc<HealthSnapshot>> ──► /api/health, SSE
 GraphWatcher task (10 s) ──┤      │                                         /api/startup
 SelfCost sampler (1 s) ────┘      └─► play_log/<ts>/health.jsonl (open/close transitions)
                                      play_log/<ts>/startup_timeline.json (at startup complete)
```

### The issue model

```rust
struct Issue {
    key: String,            // stable: "<kind>:<subject>", e.g. "crash_loop:/sensing/lidar/driver"
    kind: IssueKind,        // CrashLoop, OomKill, NeverReady, Silent, Stale, QosIncompatible,
                            // TakeDeficit, DeliveryLatency, LogFlood, ErrorBurst, MissingNode,
                            // ForeignNode, NearMissTopic, SelfCost, DiskLow
    severity: Severity,     // error | warning | info
    subject: Subject,       // Member(id) | Node(fqn) | Topic(name) | Endpoint(id) | Host
    title: String,          // one line: "crashed 4 times in 5 min (SIGSEGV)"
    detail: String,         // the evidence
    opened_at: SystemTime,
    last_seen: SystemTime,
    closed_at: Option<SystemTime>,
    link: Link,             // { view: "nodes" | "entities" | "logs" | ..., target }
}
```

Each source reports the **conditions true now** on its own cadence; the book
diffs against open issues — a new condition opens an issue, a condition
absent for one full cadence closes it (so a flapping condition does not open
and close every tick: closing needs two consecutive absent reports). Closed
issues stay in a ring of 500 so "what went wrong ten minutes ago" survives.
Every open and close is appended to `health.jsonl`, a few lines a minute, so a
bundle explains a run after the fact.

Severity is fixed per kind, not tuned per run:

| Error | Warning | Info |
|---|---|---|
| CrashLoop, OomKill, NeverReady, QosIncompatible, MissingNode, DiskLow | Stale, TakeDeficit, DeliveryLatency, LogFlood, ErrorBurst, ForeignNode, NearMissTopic, SelfCost | Silent (most silent endpoints are fine: 862 on Autoware) |

## The items

### B. Crash loops and exits

**Capture.** At `child.wait()` (`regular_node_actor.rs:318`, and the container
path `process_lifecycle.rs:270,473`): `ExitStatus::code()` **and**
`ExitStatus::signal()`, `SystemTime::now()`, and uptime from `spawned_at`.
`StateEvent::Exited` grows `{signal, at, uptime_ms}` (serde-defaulted, so old
consumers are unaffected). `save_status` writes `signal=SIGSEGV` too.

**OOM, per exit.** When the member has a cgroup (phase 66), read its
`memory.events` `oom_kill` at exit and compare with the value read at spawn: a
difference is an OOM kill, stated as fact. Without a cgroup a `SIGKILL` that
play_launch did not send is reported as "killed by SIGKILL (not by
play_launch; the OOM killer is the usual sender)", with the last RSS sample
from the monitor. Never guess beyond that.

**History.** The coordinator keeps the last 20 exits per member
(`ExitRecord {at, code, signal, uptime_ms, oom}`); `/api/nodes` gains
`last_exit` and the node panel lists the history.

**Rules.** CrashLoop: ≥3 exits within 5 min, independent of the 30 s streak
reset (a node that crashes every 40 s never trips the streak and is the one
people miss). OomKill: any. Composables: the container's exits plus
`ComposableRespawning`.

### C. Startup timeline

**Milestones per member**, all on `CLOCK_MONOTONIC` relative to the run's
start (the interceptor's clock, so the two join without conversion):

| Milestone | Source |
|---|---|
| spawned | actor, at successful spawn (new: record it in the event) |
| first endpoint | interceptor `PublisherInit`/`SubscriptionInit`, first per process (have) |
| first publish | interceptor `Publish`, first per process (have) |
| loaded (composable) | `ComposableState::Loaded` transition (new: timestamp it) |
| lifecycle active | `lifecycle_autostart` outcome (new: an event instead of a log line) |
| ready | the member's own "ready" by the existing startup-complete definition |

Composables in an `observable` container share a ring with their container;
the interceptor's `EndpointOwner` node FQN, mapped through the coordinator's
`node_fqn_map`, attributes their endpoints and first publish.

**Output.** `GET /api/startup` and `startup_timeline.json` written at startup
complete. **UI**: a Gantt chart (SVG, no library), one row per member sorted
by spawn, segments coloured by phase, the startup-complete line, and the
member that finished last highlighted: the one that set the startup time.
**Rule.** NeverReady: a member not ready 60 s after spawn, or whatever
`startup` config says (error).

As built (W7):

- **Times are taken where events are emitted**, through a hook in
  `events::emit`. The first version took them where the forwarder receives
  them, and on Autoware nodes "spawned" ~250 ms after their first endpoint:
  the receiver drains only after the spawn burst. Even at emission a fast
  node can create an endpoint a few ms before its `Started` (post-fork setup
  sits between); the data keeps both and the chart starts at the earlier.
- **Ready** is per kind: a ROS node at its first endpoint when intercepting,
  else at spawn; an `<executable>` and a container at spawn; a composable
  at its load; a lifecycle node its launch asks to activate (Jazzy
  `autostart`), when it is ACTIVE.
- **Lifecycle** (added 2026-10-14): each lifecycle node's first inactive and
  active, from two sources, the earlier winning. The interceptor reads the
  node's own `lifecycle_msgs/TransitionEvent` on publish (start and goal
  state ids by introspection; event kind 22), so a transition counts
  whoever asked for it — nav2's lifecycle manager, a script, `ros2
  lifecycle set`. play_launch's autostart driver reports what it brought
  about, so a run without interception has the times too. A node to be
  activated that is not, 60 s on, is `never_ready` ("still inactive").
  The Gantt draws configure and activate as their own segments.
- **Interceptor marks join by node FQN** through the FQN map, else by the
  process: the ring is keyed by model key and the member id carries a kind
  prefix (`node:/system/aggregator_node-1`), which the first Autoware run
  missed for all eleven unnamed nodes. `/rosout` and `/parameter_events`
  publishes are not a node's first publish.
- **NeverReady is a warning**, at 60 s, and skips a member with recent exits
  (a crash loop is reported as one; rviz2 without a display was both).
- The chart is HTML, not SVG: one grid row per member, segments positioned
  in percent.

Autoware (planning simulator, intercepting): complete at 6–11 s depending
on host load, 33 of 34 nodes ready by first endpoint (rviz2 crash-loops),
all 70 composables; last ready a `topic_state_monitor` or `glog_component`.
The page also showed `/parameter_events` subscribers raising
`delivery_latency` through startup (W5): `ParameterEvent` has a top-level
stamp and the burst queues behind constructors. Event topics are now
excluded from both the latency and the take-deficit rule. And it measured
W6's fetch change: a tab still running the old JS kept play_launch's graph
cost at 3.2% of a core (one 34 ms, 500 KB snapshot per state event, and
rviz2's crash loop makes one every few seconds); reloaded, 0.33%.

### D. Delivery latency per subscription

For a stamped message the interceptor reports the publish (monotonic time,
stamp) and every take of it (monotonic time, same stamp). In the consumer,
per topic, keep the last 256 `stamp → publish time` pairs; on a take with a
stamp, look it up: `take − publish` is how long the message waited for that
subscriber — transport plus executor queueing. Record it in a per-endpoint
log-scale histogram (32 buckets, 10 µs to 10 s) over the live window and the
run.

- **Equality, not comparison, of stamps**: it works under `use_sim_time`,
  since the stamp is only a key.
- **Several publishers with one stamp** (relays republish with the source's
  stamp): take the latest publish not after the take.
- **Coverage is reported**: matched takes / takes. Unstamped types, and
  publishers outside play_launch, give no sample, and the column says
  "no stamped traffic" rather than 0.

Entities gains `lat p50 / p99` and a DeliveryLatency warning when p99 exceeds
max(100 ms, 2 × the topic's period) for 10 s. Cost: one hash insert per
stamped publish, one lookup per stamped take; memory 256 × stamped topics.
~0.1–0.2 µs per event on top of the current ~0.13.

As built (W5), four departures from the sketch above:

- **512 stamps per topic, 64 buckets.** 256 stamps is a quarter second of a
  1 kHz IMU; 32 buckets over six decades are 54% wide, too coarse to tell
  60 ms from 90 ms. 64 buckets are ~25% wide, and a percentile reports its
  bucket's upper edge capped at the largest sample, so it errs high by at
  most one bucket.
- **Takes read before their publish.** The consumer drains each child's ring
  in turn, so the subscriber's take can be read first. Such a take waits in
  a pending queue (≤4096) retried after every drain pass and dropped,
  counting only against coverage, after 1 s.
- **Several publishes of one stamp** keep the first and the latest, and a
  take matches the latest not after it — enough for a relay, not for three
  publishers sharing stamps.
- **"For 10 s"** is the window, not a debounce: the live histogram covers
  the current 10 s epoch and the one before, and the rule needs ≥20 matched
  takes in it.
- **Judged on p95, not p99** (W7): with a few dozen samples the p99 is the
  largest one, and in the fixture a single message delivered 114 ms late
  after discovery flagged the subscriber that keeps up. Event topics
  (`/parameter_events`, `/rosout`) are not judged at all. The Health book's two-absent-reports rule does the rest.

The fixture (`latency_probe.launch.xml`: 20 Hz `std_msgs/Header`, a
subscriber that returns at once and one that sleeps 150 ms with depth 10)
measured p99 1 ms and 467 ms — the slow one takes the oldest of ten queued
messages, 10 × 47 ms. Per-event cost was not measured separately.

### E. Log rate and error bursts

**A LogWatcher task** replaces repeated whole-file reads. Every 2 s, for each
member's `out` and `err`: `fstat`; read the bytes appended since the last
offset, **at most 1 MiB per file per tick**; beyond that, count bytes only
and mark the file as flooding (reading a 40 GB log to count it would be the
problem it reports). Per line:

- **level**: the rclcpp/rclpy prefix `[ERROR] [stamp] [logger]:` (the regex
  `LogTab.js` already uses, moved server-side); other lines are "plain".
- **signature**: the message with numbers, hex, quoted strings and paths
  replaced by `#`, hashed. Per member keep the top 50 signatures by count
  (count, first/last seen, one example), evicting the least recent.

Kept per member: lines/s and bytes/s (10 s window), errors and warnings per
minute, signatures. `/rosout` rates per node come from the Entities snapshot
(no subscription). `list_members`' last-5-lines read moves onto this
watcher's tail buffer, which fixes its read-the-whole-file cost as a side
effect.

**Rules.** LogFlood: >100 lines/s or >1 MiB/min for 30 s. ErrorBurst: >10
ERROR lines/min, or any ERROR signature never seen before this run after
startup complete (a new failure mode). **UI**: a Logs section on the Health
page ranking members by log rate, and per member its top signatures with
counts — today's Autoware run would show `QoSProfile.conversion: History
policy is unknown` collapsed into one row with its count.

### F. Graph versus model

**Pass the model on**: `Arc<SystemModel>` into `WebState` and the HealthBook.

- **MissingNode** — a node the model expects that has no identity 30 s after
  its member is up. Identity comes from the interceptor (every node with
  rosout creates a `/rosout` publisher, so every node announces itself
  through `EndpointOwner`), joined to the model through the coordinator's
  `node_fqn_map` and, for nodes the launch file did not name, the real names
  in `node_identity` (#0017). No graph query.
- **ForeignNode** — `get_node_names_and_namespaces` every 10 s (one
  discovery-cache read, O(nodes), not O(topics)); any node not started by
  this run and not play_launch's own is foreign. Most often a domain-id
  collision or a stray `ros2 run`. The host is not reported by rmw; say so.
  This is the only graph call, and it runs on `spawn_blocking`.
- **NearMissTopic** — for each subscription whose topic has no publisher, and
  each publisher whose topic has no subscriber, find live topics that differ
  by one path segment, or whose last segment is within edit distance 2, or
  that share a last segment with a different namespace. Report the top 3
  candidates: "subscribes `/sensing/lidar/top/pointcloud`; a publisher exists
  on `/sensing/lidar/top/points`". Computed when the set of topics changes,
  not per tick (O(dangling × topics) ≈ 400 × 800 on Autoware).

Also: **move `build_graph_snapshot` onto `spawn_blocking`** and stop
re-fetching it after every state event (fetch on Graph-page open and every
10 s while open). It is synchronous rcl work on an async worker today.

As built (W6), and where it departs from the above:

- **No model in `WebState`.** Everything the three rules need is already in
  the member handle: the FQN map (named nodes and containers at start,
  composables on load) and, per member, whether its name is a ROS name.
  That last bit is new (`MemberMetadata::ros_name_known`): `node_name` falls
  back to the executable-derived key (#0017) for an unnamed node, and the
  first Autoware run reported eleven such nodes missing.
- **MissingNode uses the graph read**, not interceptor identities, so it
  works without interception. "Up" restarts with each new pid: a member
  crash-looping between ticks (rviz2 without a display) is not "up for 88 s".
- **ForeignNode without interception** cannot name a node the launch left
  unnamed, nor an `<executable>`'s. Such members excuse as many unclaimed
  graph nodes in their namespace; beyond that the issue is the namespace's
  ("/ (1 unaccounted)") and says to run with interception, which names every
  node a process of ours creates (`EntitySnapshot.nodes`). Hidden nodes
  (`_`-prefixed segments), rclcpp's `transform_listener_impl_*` and
  play_launch's own are never foreign.
- **DuplicateNode** was added: one FQN listed twice. Cheap from the same read,
  and two launches on one domain produce exactly this.
- **NearMissTopic is narrower.** The first Autoware run reported 53, every one
  an optional input: a segment differing entirely, or a shared leaf, matches
  `processing_time_ms` and `pointcloud` everywhere. A candidate is now a
  same-depth topic one segment off by a typo (edit distance ≤ 2 and ≤ a third
  of the word, and neither extending the other: `/initialpose3d` is a sibling
  of `/initialpose`), or the ONLY published topic with that leaf and one
  nobody subscribes to. The subscription must have dangled for two reads.
  Autoware: 0, with or without interception; the fixture's `pointz` typo: 1.
- **Graph page fetching**: once on opening and every 10 s while the Graph or
  Nodes page or a Topics tab is shown, instead of 500 ms after every state
  event. The read cost 0.3–0.4% of a core on Autoware.

### G. play_launch's own cost

**Process**: CPU (utime+stime delta), RSS, thread count, from
`/proc/self/stat` once a second. **Per subsystem**: a `cost::section("name")`
guard that reads `CLOCK_THREAD_CPUTIME_ID` on entry and exit (~85 ns each)
and adds the delta to a per-name counter. Wrapped around the synchronous
work of each subsystem: diagnostics per message, the interception drain per
tick, the monitor tick, snapshot building, graph queries, the log watcher
tick, SSE serialisation. Async waiting is not CPU and is not counted, which
is the point. The rclrs executor thread gets a name (`play-launch-rclrs`) and
is read from `/proc/self/task`.

Also reported: ring drops, `events.jsonl` size, SSE client count. **Rule**:
SelfCost warning when play_launch exceeds 10% of a core for 60 s, naming the
top subsystem.

## Prerequisite: the diagnostics subscriber

**Outcome (2026-10-13).** The diagnostics path costs 0.2–0.4% of a core; the
~30% A/B was an intermittent busy Fast DDS shared-memory listener thread in
play_launch's own participant (gdb: `SharedMemChannelResource::
perform_listen_operation`), seen in about half the day's runs and then in none
of ten, diagnostics on or off. `/dev/shm` held 285 stale `fastrtps_port*`
segments; that is the suspect, unproven. Parked. What the investigation did
fix in the diagnostics path: per-message `debug!` lines (~270 MB/h of run
log), a CSV flush per status, clones before filtering, and the subscription
moved to its own executor. The original analysis follows.

Measured at ~30% of a core on Autoware (~250 msg/s, so ~1 ms per message).
The per-message path clones every status and key/value before filtering
(`diagnostic_monitor.rs:91-98`), formats a `debug!` line per message,
allocates a key `String` per status, and **flushes the CSV once per status**
(`storage.rs:84`). Fix: filter and debounce on borrowed data, clone only what
is stored; batch CSV rows and flush once a second; gate the debug format on
`tracing::enabled!`. Verify with G before and after. This comes first: it
costs more than everything below together.

## Web UI

- **Health** becomes the first tab and the landing view, with an issue count
  badge (red if any error). Sections: Issues (open, then recently closed),
  Startup timeline, Logs, play_launch itself.
- Issue rows link to where the evidence lives: a node in Nodes, an endpoint
  in Entities, a member's log, the timeline.
- The Nodes card shows the last exit (`SIGSEGV after 12 s`) and a crash-loop
  badge; Entities gains the latency columns and near-miss hints.

API: `GET /api/health` (open + recent issues, counts), SSE
`/api/health/stream` (on change, at most 1 Hz), `GET /api/startup`,
`GET /api/self`.

## Cost on an AGX Orin

| Item | Work | Estimate (Orin) |
|---|---|---|
| B exits | per exit only | 0 |
| C timeline | once at startup | 0 |
| D latency | per stamped publish/take, ~0.1–0.2 µs | <0.2% at 2.2k events/s |
| E logs | bytes the nodes write, capped 1 MiB/file/2 s | ~0 normally; bounded under a flood |
| F graph | one node-name query / 10 s; near-miss on topic-set change | <0.1% |
| G self cost | two clock reads per section, one `/proc` read/s | <0.1% |
| HealthBook | diff + snapshot at 1 Hz | <0.1% |

Together well under 1% of an Orin core, against the ~30% the diagnostics fix
recovers.

## Plan

| Step | Content | Test that fails without it |
|---|---|---|
| W0 | Diagnostics subscriber fix, measured | CPU per diagnostics message under a bound at 250 msg/s (bench) |
| W1 | G self cost + `cost::section` | a subsystem burning CPU on purpose shows as the top entry |
| W2 | HealthBook, issue model, `/api/health`, Health page shell; feed it the Entities flags | an incompatible pair opens one issue and closes when fixed |
| W3 | B exits: signal, history, OOM delta, CrashLoop rule | a node that segfaults three times shows SIGSEGV and a CrashLoop |
| W4 | E LogWatcher (and `list_members` off whole-file reads) | a node logging 1 kHz raises LogFlood; repeated errors collapse to one signature |
| W5 | D delivery latency in Entities | a subscriber with a 50 ms sleep in its callback shows p99 ≥ 50 ms |
| W6 | F model in web state, MissingNode, ForeignNode, NearMissTopic; graph snapshot off the async worker | a stray node on the domain is reported; a remap typo yields a near-miss hint |
| W7 | C startup timeline | a member with a slow constructor is the highlighted last bar |

Order follows value per effort and dependency: W0 and W1 first so each later
step is measured as it lands; W2 before the items so each one lands as issues
on a page that exists.
