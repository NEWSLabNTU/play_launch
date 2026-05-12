# Phase 36: Runtime Enforcement of Manifest Contracts

**Status**: 36.1 + 36.2 + 36.3 + 36.4 + 36.5 + 36.6 + 36.6.1 + 36.7 done
**Priority**: High (closes static→runtime contract loop)
**Dependencies**:
- Phase 29 (rcl interception, complete) — provides LD_PRELOAD harness, SPSC ring, plugin trait
- Phase 35 (manifest format redesign, complete) — provides contracts to enforce
- Phase 30 (launch scoping, complete) — provides expected-graph data via `record.json`

---

## Overview

The static checker (Phase 35) validates contracts at authoring time and reports zero errors / zero warnings on `autoware_launch planning_simulator.launch.xml`. Phase 36 closes the loop: enforce the same contracts at runtime against live ROS 2 traffic.

The architecture extends the existing Phase 29 LD_PRELOAD harness with a second interception layer (RMW) and adds a rule-evaluation engine that consumes events from both layers.

### Why two interception layers

Different rules need different visibility:

| Rule | Needed signal | Layer |
|------|---------------|-------|
| `max_age_ms` on subscriber | `header.stamp` at take | rcl |
| `min_rate_hz` / `max_rate_hz` | Per-publish / per-take counts | rcl (cheaper) or rmw |
| `qos-match` | Actual DDS QoS negotiation outcome | rmw |
| `drop_rate` (DDS-level lost samples) | DDS lost-sample event | rmw |
| `dangling-entity` post-discovery | Endpoint creation events | rcl or rmw |
| `consistency` (declared vs runtime type) | Type support at endpoint init | rcl |
| `max_latency_ms` E2E chain | Frontier propagation | rcl |
| `graph deviation` | Publisher/subscriber creation outside expected set | rcl (warn) / rcl (block) |

The rcl layer is type-aware (sees message content, type_support pointer). The rmw layer is below — sees actual DDS calls and DDS event callbacks (QoS-incompatible, deadline-missed, liveliness-lost, sample-lost).

### Architecture

```
+--------------------------+   +--------------------------+
|  rcl interception layer  |   |  rmw interception layer  |
|  (Phase 29 — done)        |   |  (NEW — Phase 36.1)      |
|                          |   |                          |
|  Hooks:                  |   |  Hooks:                  |
|    rcl_publisher_init    |   |    rmw_create_publisher  |
|    rcl_subscription_init |   |    rmw_create_subscription
|    rcl_publish           |   |    rmw_publish           |
|    rcl_take              |   |    rmw_take_with_info    |
|                          |   |    DDS QoS events        |
|  Plugins:                |   |  Plugins:                |
|    Frontier              |   |    QosNegotiation        |
|    Stats                 |   |    DeadlineMonitor       |
|    Graph (36.5)          |   |    DropMonitor           |
+------+-------------------+   +------+-------------------+
       |                              |
       +------------+-----------------+
                    | SPSC ring (shared per child)
                    v
+----------------------------------------------------+
|  play_launch RuleEngine  (NEW — Phase 36.3)         |
|                                                     |
|  Loads merged ManifestIndex from check phase        |
|  Subscribes to SPSC events                          |
|                                                     |
|  Per-rule async tasks (one per rule type):          |
|    rate-hierarchy-runtime                           |
|    qos-match-runtime                                |
|    max-age-runtime                                  |
|    drop-budget-runtime                              |
|    max-latency-runtime                              |
|    graph-deviation-runtime                          |
|                                                     |
|  Outputs:                                           |
|    play_log/<ts>/runtime_violations.jsonl           |
|    SSE stream to web UI                             |
|    optional non-zero exit (--enforce-rules=strict)  |
+-----------------------------------------------------+
```

---

## Work items

### 36.1 RMW symbol hooks (in progress)

Extend the existing `play_launch_interception` cdylib with hooks on the rmw layer. No new package — same .so, same `LD_PRELOAD`, same SPSC ring.

**Approach**: dlsym from `librmw_implementation.so` and wrap. CARET uses a separate shim package; we extend the LD_PRELOAD because the .so already lives in every child process.

**FFI additions** in `src/vendor/rcl_interception_sys/`:

- `opaque.rs` — `rmw_publisher_t`, `rmw_subscription_t`, `rmw_node_t`, `rmw_qos_profile_t` (opaque)
- `fn_types.rs` — `FnRmwCreatePublisher`, `FnRmwCreateSubscription`, `FnRmwPublish`, `FnRmwTakeWithInfo`

**Hooks** in `src/play_launch_interception/src/lib.rs`:

- Resolve rmw symbols alongside rcl symbols (lazy, same `try_resolve_originals` pattern, extended to two libraries — `librcl.so` and `librmw_implementation.so`).
- Wrap each call: dispatch event → call original → return original's return code.
- Inert mode preserved: if no plugins activated, hooks pass through with zero extra work.

**New event variants** in `event.rs`:
- `RmwPublisherCreated { node_handle, pub_handle, topic_hash, qos_profile_hash }`
- `RmwSubscriptionCreated { node_handle, sub_handle, topic_hash, qos_profile_hash }`
- `RmwPublishCalled { pub_handle, monotonic_ns }`
- `RmwTakeReturned { sub_handle, monotonic_ns, taken: bool }`

QoS profile hash is fnv1a of the reliability + durability + history + depth bytes — enough to distinguish profiles without the full struct.

**Plugin trait extension** in `plugin.rs`:
```rust
pub(crate) trait InterceptionPlugin: Send + Sync {
    fn on_publisher_init(&self, ...);                      // existing
    fn on_subscription_init(&self, ...);                   // existing
    fn on_publish(&self, ...);                             // existing
    fn on_take(&self, ...);                                // existing
    fn on_rmw_publisher_created(&self, _: RmwPubInfo) {}   // new, default noop
    fn on_rmw_subscription_created(&self, _: RmwSubInfo) {}// new, default noop
    fn on_rmw_publish(&self, _: usize, _: u64) {}          // new
    fn on_rmw_take(&self, _: usize, _: u64, _: bool) {}    // new
}
```

Existing FrontierPlugin and StatsPlugin pick up the new defaults (no-op) and stay untouched. New plugins implement the rmw methods.

### 36.2 QoS negotiation visibility — producer side done

**Done**:
- Layout-aware `RmwQosProfile` struct added to `rcl_interception_sys::qos` matching the C `rmw_qos_profile_s` from `rmw/types.h`. Compile-time `offset_of!` assertions verify struct layout on Humble + Jazzy (Linux x86_64 / aarch64).
- Trait method `on_rmw_publisher_created` / `on_rmw_subscription_created` extended to pass `Option<&RmwQosProfile>` alongside the existing `qos_hash`. Existing plugins (FrontierPlugin, StatsPlugin) compile unchanged with the default no-op.
- New `QosNegotiationPlugin` compiled into the .so. On every rmw create call it reads the parsed profile and emits a `QosDeclaredPub` / `QosDeclaredSub` event via the shared SPSC ring.
- Two new `EventKind` variants (4, 5). The 40-byte `InterceptionEvent` budget is reused via field overloading: `_pad[0..3]` = (reliability, durability, history) as u8, `stamp_sec` = liveliness (i32), `stamp_nanosec` = depth (u32), `handle` = rmw_handle, `topic_hash` = topic FQN hash. `InterceptionEvent::qos_declared()` constructor + 2 unit tests confirm the layout roundtrip.
- Consumer `EventKind` enum in `play_launch::interception` extended with the two new variants. `process_event` ignores them (they flow through to the RuleEngine in 36.3).

**Pending in 36.2**:
- ~~DDS QoS event callbacks.~~ Done — see 36.8 below.

### 36.8 DDS event callbacks — done

The DDS layer surfaces async events (incompatible QoS, deadline missed, liveliness lost/changed, message lost) via the `rmw_event_t` API. We piggyback on the polling that rclcpp / rclpy already does — when a user registers a QoS event callback (or rclpy installs its default incompatible-QoS callback), the executor wakes up and calls `rmw_take_event`. We hook three rmw symbols:

- `rmw_publisher_event_init(event, publisher, event_type)` — bind event handle to (entity, event_type)
- `rmw_subscription_event_init(event, subscription, event_type)` — same for subscriptions
- `rmw_take_event(event, event_info, taken)` — after the original returns OK with `taken=true`, decode the status struct based on the bound event_type and push to the SPSC ring

No polling thread, no participation in DDS waitsets. Pure passive observation. If the application's executor never polls a particular event, we never see it — operators who care about catching DDS events must register an event callback (rclpy installs default callbacks for `*_INCOMPATIBLE_QOS`, so the most common cases work out of the box).

**FFI** in `src/vendor/rcl_interception_sys/`:
- `opaque.rs::rmw_event_t` — layout-aware view exposing the `event_type` field
- `events.rs` (new) — status struct layouts (`RmwQosIncompatibleEventStatus`, `RmwDeadlineMissedStatus`, `RmwLivelinessChangedStatus`, `RmwLivelinessLostStatus`, `RmwMessageLostStatus`) + `event_type` discriminant constants + `qos_policy_kind` bitmask constants
- `fn_types.rs` — `FnRmwPublisherEventInit`, `FnRmwSubscriptionEventInit`, `FnRmwTakeEvent`

**Hooks** in `src/play_launch_interception/src/lib.rs`:
- `rmw_publisher_event_init` / `rmw_subscription_event_init` register `(rmw_event_t*, side, entity_ptr, event_type)` in the new event registry
- `rmw_create_publisher` / `rmw_create_subscription` now also register the rmw entity ptr → topic_hash mapping (separate from rcl entity registry, since rcl and rmw handles are distinct)
- `rmw_take_event` looks up the binding, resolves topic_hash from the rmw entity registry, and dispatches `on_rmw_dds_event(entity_ptr, topic_hash, event_type, status_ptr)`

**Plugin** in `src/play_launch_interception/src/plugins/dds_events.rs`:
- `DdsEventsPlugin` decodes the status struct per event_type and emits one of 7 new event variants:
  - `OfferedQosIncompatible = 6`, `RequestedQosIncompatible = 7` — payload: total_count, total_count_change, last_policy_kind
  - `OfferedDeadlineMissed = 8`, `RequestedDeadlineMissed = 9` — payload: total_count, total_count_change
  - `LivelinessLost = 10` — same as deadline-missed
  - `LivelinessChanged = 11` — payload: alive_count, not_alive_count, alive_count_change (i8), not_alive_count_change (i8)
  - `MessageLost = 12` — payload: total_count (saturating), total_count_change (saturating)
- Plugin activated alongside StatsPlugin and QosNegotiationPlugin whenever a shared memory producer exists.

**Consumer** in `src/play_launch/src/runtime_enforcement/mod.rs`:
- `observe()` extended with 7 new match arms calling new `emit_dds_*` methods.
- Skips dedup (uses `emit_repeatable`) since each event carries a real delta — we want one violation line per occurrence.
- Maps:
  - `OfferedQosIncompatible` / `RequestedQosIncompatible` → `qos-match-runtime` (Error)
  - `OfferedDeadlineMissed` / `RequestedDeadlineMissed` → `deadline-runtime` (Warning)
  - `LivelinessLost` / `LivelinessChanged` (when not_alive_change ≠ 0) → `liveliness-runtime` (Warning/Error)
  - `MessageLost` → `drop-rate-runtime` (Warning)

**Smoke test**: `qos_match_runtime_fires_on_dds_incompatibility` in `tests/tests/runtime_enforcement.rs`. Two rclpy nodes — one publisher with `BEST_EFFORT` reliability, one subscription with `RELIABLE`. DDS discovery flags incompatibility; rclpy's default incompatible-qos callback drives `rmw_take_event` polling; the runtime path emits `qos-match-runtime` with a `DDS reported incompatible QoS` message. Test asserts the violation lands in `runtime_violations.jsonl`.

### 36.3 RuleEngine in play_launch — done

Module: `src/play_launch/src/runtime_enforcement/mod.rs`.

**Implemented**:
- `RuleEngine` struct holds `Arc<ManifestIndex>` (cloned from the static-check pass) plus per-topic runtime aggregation state.
- Runs **inline in the existing interception listener task** rather than as a separate tokio task — every event goes through `process_event` first (frontier/stats aggregation), then through `rule_engine.observe(&event)` if `--enforce-rules` is not `Off`.
- Emits `RuntimeViolation { rule_id, severity, fqn, message, timestamp_ns }` to `play_log/<ts>/runtime_violations.jsonl` (BufWriter, flushed per emit).
- Duplicates suppressed: identical `(rule_id, fqn)` pairs fire once.
- `strict_violated` flag set on first violation in `EnforceMode::Strict` for owner-driven shutdown.

**Rules implemented**:

| Rule | Inputs | Status |
|------|--------|--------|
| `qos-match-runtime` | `QosDeclaredPub` + `QosDeclaredSub` events; DDS offered ≥ requested matrix on `reliability` and `durability` | ✓ |
| `rate-hierarchy-runtime` | `Publish` event counts windowed (every 1024 publishes); compared against `EndpointProps.min_rate_hz` for each publisher endpoint | ✓ |
| `max-age-runtime` | `SystemTime::now() - header.stamp` at take; compared against `EndpointProps.max_age_ms` on the consuming subscriber. Approximation: wall-clock is sampled at consumer-side observation, not at the in-kernel take moment — overstates age by listener poll lag (≤10ms). Acceptable for budgets ≥100ms. | ✓ |
| `drop-rate-runtime` | Per-sub take_count vs pub_count windowed (every 256 takes per subscriber); worst-case ratio compared against `topic.drop.max_count` | ✓ |
| `consistency-runtime` | Runtime msg-type identity from `rosidl_typesupport_introspection_c` (read in the `.so`'s `rcl_*_init` hook; hashed via fnv1a) carried via the overloaded `stamp_sec` + `stamp_nanosec` slots of init events; compared against fnv1a of declared `topic.msg_type`. TODO/placeholder types skipped. | ✓ |
| `max-latency-runtime` | Per-stamp publish monotonic_ns tracked per topic (bounded to last 512 stamps); on a publish to a scope path's output topic, look up the same stamp on the input topic and compute the diff against `max_latency_ms` | ✓ |
| `graph-deviation-runtime` | Lives in `GraphPlugin v2` (36.5) | ✓ |

**Tests**: 8 unit + integration tests covering each rule:

- `reliability_matrix`, `durability_matrix` — DDS compat matrix
- `qos_match_runtime_fires_on_incompatible_pair` — end-to-end via synthetic events
- `consistency_runtime_fires_on_type_mismatch` — wrong type hash in init event
- `max_age_runtime_fires_when_stamp_too_old` — 1-second-old stamp vs 10ms budget
- `graph_deviation_fires_on_unknown_topic`
- `lifecycle_gating_default_unknown_blocks_check`
- `strict_mode_trips_atomic_flag`

Total: 120 play_launch tests pass.

### Smoke tests (real ROS 2 launch)

`tests/tests/runtime_enforcement.rs` spawns the cargo-built `play_launch` against `tests/fixtures/simple_test/launch/pure_nodes.launch.xml` (two `demo_nodes_cpp` nodes under namespace `/pure_test`) with:
- Interception enabled via inline config YAML
- `--manifest-dir <tempdir>` with `_/pure_nodes.yaml`
- `--enforce-rules=<mode>` per test

Four tests, all passing:
- `graph_deviation_runtime_fires_for_undeclared_topic` — empty manifest topics block; verifies `graph-deviation-runtime` warnings are emitted to `play_log/<ts>/runtime_violations.jsonl`.
- `runtime_violations_jsonl_written_on_real_launch` — manifest with declared topic; verifies the jsonl file exists with at least one entry tagged with a `-runtime` rule.
- `enforce_off_skips_rule_engine` — `--enforce-rules=off`; verifies no jsonl file is produced.
- `block_unauthorized_endpoints_writes_allowlist_file` — `--block-unauthorized-endpoints`; verifies `play_log/<ts>/expected_graph.txt` is written with the declared FQNs.

### Topic-name expansion in the `.so` (done)

`rcl_publisher_init` / `rcl_subscription_init` hooks receive the topic
name as the caller passed it — for `demo_nodes_cpp::talker`, that's
bare `chatter`. To make the consumer-side `topic_hash_to_fqn` map
match manifest-declared absolute FQNs, the `.so` expands the topic
before hashing.

Implementation:

- `rcl_node_get_name` and `rcl_node_get_namespace` resolved via dlsym
  alongside the other rcl symbols (optional — `Originals.node_get_name`
  / `node_get_namespace` are `Option<FnRclNodeGetX>` so non-ROS
  processes degrade gracefully).
- `expand_topic_name(originals, node, raw_topic)` in `lib.rs` applies
  the simplified ROS 2 topic-name resolution rules:
  - Absolute (`/...`) → unchanged.
  - `~` prefix → `<node_ns>/<node_name>/<rest>`.
  - Relative → `<node_ns>/<topic>`.
  Falls back to the raw topic string if the node accessors are
  unavailable or fail (defensive — preserves Phase 29's inert-mode
  guarantees).
- Both rcl init hooks now call `expand_topic_name(...)` and use the
  expanded form for `register_publisher`/`register_subscription` and
  the topic_hash passed to plugins.

Advanced rcl substitutions (`{var}`, `{ns}`, etc.) are not supported —
they're rare in practice and would need parsing the full
`rcutils_string_map_t` substitution table. Document as future work
if a real workload hits them.

Smoke test verifies the fix: `consistency_runtime_fires_when_type_mismatch_real_launch`
declares `/pure_test/chatter` in the manifest and expects
`consistency-runtime` to fire on the exact FQN.

### 36.4 CLI: enforce flag — done

`play_launch launch ... --enforce-rules=<mode>`:

| Mode | Behavior |
|------|----------|
| `off` | Skip runtime checks. |
| `warn` (default) | Log violations to `runtime_violations.jsonl`, never exit early. |
| `strict` | First violation → SIGTERM all children → exit non-zero. For CI. |
| `record-only` | Skip rule evaluation; just collect data. (RuleEngine constructed but `observe()` short-circuits.) |

**Implementation**: `RuleEngine` exposes `strict_violated_handle() ->
Arc<AtomicBool>`. In `commands/replay.rs`, when `--enforce-rules=strict`,
a watcher tokio task polls the flag every 100ms and sends `true` on
`shutdown_tx` when the flag flips. The existing shutdown pipeline
SIGTERMs all children and produces a non-zero exit. Unit test
`strict_mode_trips_atomic_flag` confirms the flag flips on first
violation.

### 36.5 GraphPlugin v2 (warn-only) — done

Implemented as a **consumer-side rule** instead of a separate `.so`
plugin, because the existing `PublisherInit` and `SubscriptionInit`
events from `StatsPlugin` already carry topic_hash + handle and are
all that's needed. The RuleEngine maintains `topic_hash_to_fqn` built
from the merged `ManifestIndex` (both `topics:` and
`external_topics:`); any `PublisherInit` / `SubscriptionInit` event
with a hash absent from that map fires `graph-deviation-runtime` as a
warning.

Trade-off: the warning carries the hash as the FQN placeholder since
the .so today doesn't ship the topic string in the SPSC event budget
(40 bytes per event, fully consumed). Operators can grep their
manifests for which expected FQN was meant; alternatively a future
event variant can ship the topic string out-of-band for unknown
topics.

Blocking enforcement (36.7) still lives in the .so via
`rcl_publisher_init` returning `RCL_RET_ERROR`. The consumer-side
check is the warn-only half and was simpler to land first.

Unit test `graph_deviation_fires_on_unknown_topic` confirms an
unknown-hash `PublisherInit` triggers the rule.

### 36.6 Lifecycle-aware gating (issue #49) — engine done; subscriber pending

**Engine side done**:

- `LifecycleState { Unknown, Unconfigured, Inactive, Active, Finalized }` enum mirroring `lifecycle_msgs/msg/State`.
- `RuleEngine` carries `lifecycle_nodes: HashMap<String, bool>` built from `index.manifests` at startup — every node with `lifecycle: true` is enrolled.
- `RuleEngine.lifecycle_state: HashMap<String, LifecycleState>` records observed states.
- `is_node_enforceable(node_fqn)` — non-lifecycle nodes always true; lifecycle nodes only when `Active`. Default `Unknown` blocks checks conservatively until first transition observed (avoids false positives during boot).
- `set_lifecycle_state(node_fqn, state)` — public API for the transition-event subscriber to update state.
- `rate-hierarchy-runtime` gated on `is_node_enforceable(publisher_node_fqn)`. Other state-driven rules (`max-age-runtime`, `drop-rate-runtime`, `max-latency-runtime`) gain the same gate as they land.

Unit test `lifecycle_gating_default_unknown_blocks_check` confirms:
- Manifest `lifecycle: true` enrolls the node.
- Default `Unknown` state suppresses rate-hierarchy violations.
- `set_lifecycle_state(.., Active)` re-enables the check.
- `Inactive` re-suppresses.
- Non-lifecycle nodes are always enforceable regardless of map state.

### 36.6.1 — Transition-event subscriber wiring (done)

- `lifecycle_msgs = "*"` added to `play_launch/Cargo.toml`; patched in `.cargo/config.toml` to `/opt/ros/humble/share/lifecycle_msgs/rust` until added to colcon build.
- `RuleEngine::lifecycle_node_fqns()` returns every lifecycle node from the manifest tree.
- `commands/replay.rs`: walks the lifecycle node list and creates a subscription to `<fqn>/transition_event` on the shared rclrs node for each. Callback maps `goal_state.id ∈ {1,2,3,4}` to `LifecycleState::{Unconfigured,Inactive,Active,Finalized}` and forwards via mpsc.
- `run_interception_task` signature extended with `Option<mpsc::UnboundedReceiver<(String, LifecycleState)>>`. Each tick drains the channel and applies updates to `RuleEngine::set_lifecycle_state(...)` before processing the next event batch.
- Subscription handles stored as `Vec<Box<dyn Any + Send + Sync>>` so the call site doesn't drag generic parameters around.

### 36.7 v3 blocking enforcement — done

`rcl_publisher_init` and `rcl_subscription_init` hooks now refuse to
create endpoints whose topic FQN isn't on the allowlist, returning
`RCL_RET_TOPIC_INVALID` (1004).

**Activation** (two env vars, both required):
- `PLAY_LAUNCH_INTERCEPTION_GRAPH_FILE` — path to newline-separated
  topic FQN file (`#` comments and blank lines skipped).
- `PLAY_LAUNCH_INTERCEPTION_BLOCK_GRAPH=1` — explicit opt-in to
  blocking mode. With the env var unset the file is ignored entirely.

When inactive, the `is_allowed()` function returns `true` unconditionally and the hooks pass through.

**CLI integration**:
- `--block-unauthorized-endpoints` flag on `CommonOptions`. Requires `--manifest-dir`.
- When set, `commands/replay.rs` writes
  `play_log/<ts>/expected_graph.txt` listing every `topics:` and
  `external_topics:` FQN from the merged `ManifestIndex`, then injects
  the env vars into every child via `setup_child_interception(...)`.

**`src/play_launch_interception/src/allowlist.rs`**:
- `OnceLock<Option<HashSet<u64>>>` populated lazily on first hook
  invocation.
- Hash entries via fnv1a so the hot path is a single hash + set lookup.
- 1 unit test verifies the file format parser (comments, blank lines,
  whitespace trimming).

**Risk + mitigation**:
- Nodes that don't handle `rcl_publisher_init` failure may crash.
- Default is off — operators opt in via `--block-unauthorized-endpoints`
  after running in warn-only mode (Phase 36.5 `graph-deviation-runtime`)
  and confirming no surprise publishers slip through.

---

## Order and dependencies

```
36.1 RMW hooks (cdylib + FFI)  ──┐
                                  ├──→ 36.3 RuleEngine ──→ 36.4 CLI flag
36.2 QoS negotiation plugin   ───┘                       ──→ 36.5 GraphPlugin v2
                                                          ──→ 36.6 Lifecycle gating
                                                          ──→ 36.7 v3 blocking (opt)
```

36.1 → 36.2 → 36.3 is the critical path. 36.5–36.7 are independent extensions.

---

## Risks and unknowns

1. **RMW symbol coverage varies by implementation.** `rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp`, `rmw_connextdds` expose the same core API but with optional extensions. Detect via `rmw_get_implementation_identifier()` and skip vendor-specific hooks gracefully.

2. **dlsym ordering.** `librcl.so` and `librmw_implementation.so` both loaded. `dlsym(RTLD_NEXT)` finds the next-defined symbol in load order. The LD_PRELOAD .so is loaded first, so hooks should be found first by everything. Verified for rcl; verify for rmw.

3. **Both interception layers can fire for one publish.** `rcl_publish` → `rmw_publish`. To avoid double-counting in StatsPlugin: hooks tagged with origin (rcl vs rmw) and plugins pick one. Default: rcl for content-aware plugins, rmw for transport-aware plugins.

4. **Performance.** Phase 29 hot path: 40–50ns/publish. Adding rmw layer + QoS plugin should stay under 100ns/publish. Critical for safety-critical pipelines.

5. **Lifecycle transition listening.** Subscribing to `<node>/transition_event` topics from RuleEngine is the cleanest, but requires a full rclrs Node. Already have one (used for graph polling). Reuse.

6. **--enforce-rules=strict and crash semantics.** Killing a node mid-publish can leave a topic without consumers / producers in an unrecoverable state. `strict` only makes sense for CI / pre-production. Document.

---

## Out of scope for Phase 36

- B1 (RMW shim package via `RMW_IMPLEMENTATION` env var). Cleaner but adds packaging. Revisit if B2 hits portability limits.
- Distributed multi-machine enforcement. RuleEngine runs in play_launch; remote nodes would need a second consumer. Defer until multi-machine deployments are a documented use case.
- Recording / replay of runtime violations. Existing `play_log/<ts>/` capture is sufficient.

---

## Tests

Per work item:

- 36.1: smoke test under planning_simulator confirms rmw hooks fire; SPSC events delivered; no performance regression on existing tests.
- 36.2: fixture with `best_effort` pub + `reliable` sub; assert `qos-match-runtime` violation logged.
- 36.3: fixture with declared `min_rate_hz: 30` and producer at 10 Hz; assert `rate-hierarchy-runtime` violation logged within 2 windows.
- 36.4: `--enforce-rules=strict` exits with non-zero on first violation.
- 36.5: fixture with unauthorized topic; assert `graph-deviation` warning.
- 36.6: lifecycle node in Inactive state; assert checks suppressed.

Integration target: re-run autoware-contract end-to-end as a launchable system. Initial expectation: zero runtime violations on a clean planning_simulator run. Long-running soak surfaces real bugs that static checking can't catch (drop spikes, age violations under load).

---

## Comparison with prior art

- **CARET (Tier4)** — same LD_PRELOAD pattern, hooks rcl and rmw, focuses on tracing/latency analysis. We share the technique but our scope is contract enforcement, not analysis. CARET outputs CTF; we output structured violations matched to manifest rule IDs.
- **`rmw_stats_shim`** — RMW shim package collecting stats. B1-style. We chose B2 (LD_PRELOAD on rmw) to avoid the second package.
- **DDS Security plugins** — vendor-specific, focus on access control. Not portable across RMWs. We stay at rcl/rmw layer for portability.
