# Live entity monitor

**Status: W1–W3 implemented 2026-10-13** (live snapshot API, QoS check,
Entities page; W4 external endpoints and W5 counter mode not built). Verified
on the full Autoware planning simulator — see "Measured on Autoware" below. A web page, in the spirit of
`htop`, that lists every live publisher and subscription with its message rate
and byte rate, and flags QoS mismatches — cheap enough to leave on during a
drive on an AGX Orin.

## What the page answers

1. **Who is loud.** Endpoints sorted by byte rate, then message rate. The
   topic that saturates a link or a core is the first row.
2. **Who is silent.** Endpoints that were created and never carried a
   message, or carried messages and stopped (`last message N s ago`). A
   sensor whose driver hung reads exactly like this.
3. **Who cannot talk.** A publisher and a subscription on one topic whose QoS
   makes them incompatible, so DDS never matches them, plus the softer case
   where they match but the subscriber is too shallow for the rate.

## What exists today

| Piece | State | Where |
|---|---|---|
| Per-message events from every child (typed, serialized, loaned) | done | `play_launch_interception` |
| Per-endpoint aggregation keyed by (ring, endpoint), owner node, sampled size | done, **at shutdown only** | `interception/endpoint_stats.rs` |
| Declared QoS per endpoint | partial: `QosDeclared*` events carry reliability, durability, history, depth and liveliness kind, keyed by the **rmw** handle; no deadline or lease, and no link to the rcl endpoint the stats use | `plugins/qos_negotiation.rs` |
| DDS-reported incompatibility | done, when the node's executor polls QoS events (rclcpp installs a default incompatible-QoS callback, so C++ nodes usually do) | `plugins/dds_events.rs`, `RuleEngine` |
| QoS of every endpoint on the graph, including ones play_launch did not start | done, on demand per HTTP request | `ros/graph_builder.rs` (`GET /api/graph`) |
| Live channel from interception to the web server | **none**: the interception task's state is task-local; `WebState` has no field for it | `web/mod.rs:38` |
| Web pages | Preact + htm + signals; nav in `Header.js`, router in `app.js`; SSE for live data; no virtualised table | `web/assets/js` |

So the counting, attribution and sizing are done; what is missing is a
**window** (rates now, not averages over the run), a **live hand-off** to the
web server, **QoS on the endpoint**, and the **page**.

## Design

### 1. Windowed counters in the consumer

`EndpointTracker` keeps run totals. Add, per endpoint, a ring of ten 1-second
buckets `{messages, bytes_sampled, size_samples}`; the bucket index is
`monotonic_ns / 1e9 mod 10`, cleared when the clock enters it. From that:

- `rate_1s`, `rate_10s` — messages in the last bucket / last ten.
- `bytes_per_s` — rate × mean sampled size over the window (exact on the
  serialized paths, as today).
- `last_message_age_s` — from the existing `last_message_ns`.

Cost: one extra array increment per event, already inside the per-event
`HashMap` lookup. Memory: ~10 × 24 B per endpoint, ~250 KB for Autoware's
982 endpoints.

### 2. Live hand-off: a 1 Hz snapshot on a `watch` channel

The interception task already wakes every 10 ms. Once a second it builds a
`Vec<LiveEndpoint>` (the rows below) and publishes it as
`tokio::sync::watch::Sender<Arc<EntitySnapshot>>`. `WebState` gains
`entities: Option<watch::Receiver<Arc<EntitySnapshot>>>`.

Why `watch` and not a shared `RwLock` over the tracker: the hot loop never
waits on the web server, a slow browser never sees a half-updated table, and
a snapshot nobody reads costs one `Arc` swap. Building it is O(endpoints) —
about 1000 rows once a second.

- `GET /api/entities` — the current snapshot (JSON).
- `GET /api/entities/stream` — SSE, one snapshot per second, sent **only
  while a client is connected**. Rows are sorted and, by default, trimmed
  server-side to the top 200 by byte rate plus every flagged row (silent,
  mismatched, stale), so a 1000-endpoint system ships ~30 KB/s, not ~150.
  The client asks for `?all=1` when the user clears the filter.

### 3. QoS on the endpoint, and the mismatch check

**Source.** In `rcl_publisher_init` / `rcl_subscription_init`, after the
original succeeds, call `rcl_publisher_get_actual_qos` /
`rcl_subscription_get_actual_qos` and emit one `EndpointQos` event keyed by the
same rcl id the stats use (`lib.rs::rcl_id`). "Actual" matters: the declared
profile may say `SYSTEM_DEFAULT`, the actual one says what the middleware
chose. The 56-byte event fits it: `_pad` = reliability, durability, history;
`stamp_sec` = depth; `stamp_nanosec` = liveliness kind; `monotonic_ns` =
deadline (ns); `cpu_ns` = lease duration (ns). Once per endpoint, cold path.
This replaces the rmw-keyed `QosDeclared*` events for this purpose and closes
their gap (no durations, no link to the endpoint).

**Check**, per topic, every publisher × every subscription, using DDS's
request-vs-offered rules (the same matrix `runtime_enforcement::check_qos_match`
applies against contracts — factor it out and share it):

| Policy | Incompatible when |
|---|---|
| reliability | pub `BEST_EFFORT`, sub `RELIABLE` |
| durability | pub `VOLATILE`, sub `TRANSIENT_LOCAL` |
| deadline | pub period > sub period |
| liveliness | pub `AUTOMATIC`, sub `MANUAL_BY_TOPIC` |
| lease duration | pub lease > sub lease |

These never match: DDS delivers nothing. Shown red, on both rows.

**Advisories** (they match, but something is likely wrong), shown amber:

- **Take deficit**: the sum of publish rates on a topic exceeds a reliable
  subscriber's take rate by more than 10% over 10 s. Messages are being
  dropped or queued — usually `KEEP_LAST` depth too small for the rate, or a
  callback too slow. This is the most useful one in practice and needs no QoS
  data at all, only the counters from §1.
- **Depth 1 on a topic faster than the subscriber's callback** (same signal,
  with the depth named as the likely cause).
- **Late-joiner loss**: a `TRANSIENT_LOCAL` subscriber on a topic whose
  publishers are all `VOLATILE` (a map or a static TF that a late node never
  receives). Formally a mismatch, listed separately because it is the
  commonest one.

**Confirmation.** The DDS `*_QOS_INCOMPATIBLE` events the `.so` already
forwards are the middleware's own verdict: when one arrives for an endpoint,
the row says "DDS confirmed". The computed check stands without it (a node
whose executor never polls QoS events never produces one).

**Endpoints play_launch did not start** (RViz, `ros2 bag`, another host) are
invisible to the interceptor. Their QoS comes from the graph API that
`graph_builder.rs` already queries — but only for topics that have a
launch-side endpoint, only while the page is open, and at most every 10 s,
because that query walks every topic (§Cost).

### 4. The page

A fourth nav tab, **Entities** (`Header.js` button, `app.js` branch,
`store.js` signal), fed by `/api/entities/stream`.

```
 Entities   pubs 412 (37 silent)   subs 570 (61 silent)   QoS: 2 incompatible, 5 advisories   ring drops 0
 ┌─────┬──────────────────────────────┬─────────────────────────────┬────────┬─────────┬────────┬──────┬──────────┐
 │ dir │ topic                        │ node                        │ msg/s  │ bytes/s │ size   │ last │ QoS      │
 ├─────┼──────────────────────────────┼─────────────────────────────┼────────┼─────────┼────────┼──────┼──────────┤
 │ pub │ /sensing/lidar/concat/points │ /sensing/.../concatenate    │  10.0  │ 38 MB/s │ 3.8 MB │ 0.1s │ R V K5   │
 │ sub │ /sensing/lidar/concat/points │ /perception/.../centerpoint │   7.9  │ 30 MB/s │ 3.8 MB │ 0.1s │ ⚠ deficit│
 │ sub │ /map/vector_map              │ /planning/.../behavior      │   0.0  │   —     │   —    │never │ ✖ dur    │
 │ ... │                              │                             │        │         │        │      │          │
```

- Columns sortable; default sort bytes/s, then msg/s. `size` is the mean
  serialized message size; `R/B`, `V/T`, `K<n>/A` are reliability,
  durability, history+depth.
- Filters: text (topic or node), direction, "hide silent", "only flagged".
  Toggle **group by topic** to show each topic as a row with its publishers
  and subscriptions nested, the publish total beside each take rate (where a
  deficit is visible at a glance).
- Click a row: the existing right panel opens on that node (`TopicsTab`
  already lists its topics), with a QoS sub-table and the mismatch reason.
- No virtualised table exists; with server-side trimming to ~200 rows a
  plain Preact table re-rendered at 1 Hz is fine. Add windowing only if
  `?all=1` on a 1000-endpoint system is measurably slow.

## Cost on an AGX Orin

### Measured (this x86 host, 2026-10-13)

Synthetic rclpy publisher + subscriber, `play_launch launch`, 15 s windows
(`tmp/bench/` scripts, not committed):

| Config | Events/s (pub + take) | play_launch CPU | Disk |
|---|---|---|---|
| interception off | — | 0.3% of a core | — |
| interception on, stats, `events: false` | 145 k | **1.9%** | — |
| interception on, stats, `events: true` (the default) | 143 k | **6.3%** | **~15 MB/s** |
| on, stats, 10 k events/s; node CPU vs off | 10 k | 0.6% | — |

So the consumer costs ~0.13 µs per event for aggregation, and ~0.43 µs per
event once `events.jsonl` formats and writes a JSON line for it. On the node
side the documented hook cost is ~150 ns per publish or take (phase 58 W2);
at 10 k events/s it was below the noise of the test's own Python nodes.

### Scaled to the Orin

The Orin's Cortex-A78AE cores are roughly 2–3× slower per thread than this
host for this kind of work; the table assumes 3×. **Autoware's real message
rate has never been measured here** (phase 77 counted 982 endpoints and 63
carrying traffic in the planning simulator, with no sensors); a vehicle with
sensors is plausibly 10–50 k events/s. Measure it first (step W0 below); until
then:

| Load (events/s) | Hooks, summed over all nodes | Consumer, stats only | Consumer + `events.jsonl` | `events.jsonl` disk |
|---|---|---|---|---|
| 10 k | ~0.5% of a core | ~0.4% | ~1.3% | 1.1 MB/s (4 GB/h) |
| 30 k | ~1.4% | ~1.2% | ~3.9% | 3.3 MB/s (12 GB/h) |
| 100 k | ~4.5% | ~3.9% | ~13% | 11 MB/s (40 GB/h) |

The live view itself adds, on top of the stats-only column:

| Item | Cost |
|---|---|
| Window buckets (§1) | one increment per event: noise |
| 1 Hz snapshot, ~1000 rows | ~1 ms/s: < 0.1% of a core |
| SSE, page open, trimmed | ~30 KB/s; serialisation ~1 ms/s |
| QoS: `EndpointQos` + per-topic pairing | once per endpoint at init; pairing O(pubs × subs) per topic, recomputed only when an endpoint appears |
| Size sampling (already shipped) | one `rmw_serialize` per busy endpoint per second; dominated by large messages: ~10 point clouds × 4 MB at ~5 GB/s ≈ 8 ms/s ≈ 0.8% of a core, plus a 4 MB allocation each |
| Graph API for external endpoints | each `get_*_info_by_topic` walks the discovery cache; for ~500 topics, tens of ms per sweep. Every 10 s, page open only: < 0.5% |

**What actually burns, in order:**

1. **`events.jsonl`** — 3× the consumer CPU of everything else, and the
   disk. **Decided 2026-10-13: off by default**, switchable at runtime
   (`--record-events`, the web UI's "Record events" toggle,
   `POST /api/interception/events`). The live monitor never needs it.
2. **One ring event per message.** Both the hook and the consumer scale
   with messages, not endpoints. At 100 k events/s that is ~8% of a core
   across both sides for numbers the page shows once a second.
3. **The producer mutex.** `StatsPlugin` takes `Mutex<Producer>` for every
   event; a multi-threaded executor publishing from several threads
   contends on it. Not measured yet; it is the likeliest source of a tail
   latency a node would notice.
4. Size sampling, only for systems with several multi-megabyte topics.

### The lever if that is too much: counter mode

When nothing needs per-message events — no `events.jsonl`, no trace, no
frontier, no contract rules that read stamps — the `.so` can count instead of
report: per endpoint, `AtomicU64` message and byte counters in the registry
record (already per endpoint, already looked up on every message), and a
1 Hz `EndpointTick { id, messages, bytes, size_samples }` per active endpoint,
emitted by whichever hook first crosses the second boundary.

- Ring traffic and consumer work drop from O(messages) to O(endpoints): at
  100 k events/s across 1000 endpoints, 100× less.
- The hook becomes a lookup and two relaxed atomic adds — no mutex, no
  timestamp syscall per message (`cpu_ns` and `monotonic_ns` are only needed
  for per-message events).
- Cost: no per-message timing, so `measure`, the trace, `max_age`/latency
  rules and the frontier are unavailable in this mode. That is the trade, and
  why it is a mode and not the default.

Do not build it speculatively: the stats-only numbers above are already
small at plausible rates. Build it if W0 shows a vehicle above ~50 k events/s
or if the mutex shows up in a node's latency.

## Measured on Autoware (2026-10-13)

Full planning simulator with the sample map, `--interception on`, x86 host.

- **788 publishers (475 silent), 630 subscriptions (387 silent)**, ~2,200
  events/s, ~357 KiB/s published. Busiest by bytes:
  `/system/emergency/hazard_status` (11 KiB at 10 Hz, ~110 KiB/s, no
  subscriber in the launch); busiest by rate: `/diagnostics`, 247 msg/s from
  33 publishers.
- **0 incompatible pairs, 10 take-deficit advisories.** All are real
  mechanically; most are Autoware's polling pattern — a depth-1 subscriber
  that takes the latest value at 10 Hz from a 40 Hz publisher
  (`/vehicle/status/control_mode`) — and the rest are subscribers that take
  nothing because planning has no route. The advisory text says which reading
  applies.
- **play_launch's own CPU, 30 s each arm** (no browser attached):

  | Arm | CPU |
  |---|---|
  | everything off (`--disable-monitoring --disable-diagnostics`, no interception) | 1.1% |
  | + interception and the Entities snapshot | **2.1%** |
  | + resource monitoring only | 7.7% |
  | + diagnostics only | **31.5%** |

  The monitor costs ~1% of a core at Autoware's real rate. What burns is
  play_launch's own `/diagnostics` subscriber: ~250 msg/s for ~30% of a
  core, about 1 ms per message, on by default. That is the item to fix
  before an Orin deployment, not anything here.

Two defects the run found, both fixed with tests: a 10-slot ring holds only
nine complete seconds (rates read 9.0 for 10.0), and "stale" fired on every
startup burst (`/parameter_events`); stale now needs a stream of at least five
messages over five seconds and skips `/parameter_events` and `/rosout`.

## Plan

| Step | Content | Size |
|---|---|---|
| W0 | Measure: one bag-replay Autoware run on the Orin with `stats: true, events: false`; record events/s, play_launch CPU, ring drops. Decides whether counter mode is needed | half a day |
| W1 | Window buckets + 1 Hz `watch` snapshot + `/api/entities` and `/stream` | 1–2 days |
| W2 | `EndpointQos` from `get_actual_qos`; shared compatibility matrix with `runtime_enforcement`; take-deficit advisory; DDS-event confirmation | 2 days |
| W3 | Entities page: table, sort, filters, group-by-topic, row → right panel | 2–3 days |
| W4 | External endpoints from the graph API, page-open only, 10 s | 1 day |
| W5 (conditional) | Counter mode, if W0 says so | 2–3 days |

Each step ships with a test that fails without it: W1 a rate that changes
mid-run and must show in the 1 s window and not the run average; W2 a
deliberately BEST_EFFORT/RELIABLE pair (the `qos_mismatch.launch.xml` fixture
already exists) and a depth-1 subscriber behind a fast publisher.

## Open questions

- Is a terminal view (`play_launch top`, reading the same SSE) worth having
  for headless vehicles? The snapshot API makes it cheap; the page is the
  ask.
- Intra-process traffic (rclcpp intra-process comms) bypasses rcl and is
  invisible here, as in `endpoint_stats.json`. Composable containers in
  `observable` mode with intra-process enabled will show such topics as
  silent. Label it rather than hide it.
