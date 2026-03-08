# Data Transmission Quality Semantics in Launch Systems

**Status**: Research / Exploration
**Date**: 2026-03-06

---

## Motivation

ROS 2 launch files specify *what* runs and *how* it connects, but not *how well* data should flow. Three critical quality dimensions are absent from launch-time specification:

1. **Bounded data loss** — "drop at most 1 in 10 lidar frames" has no representation
2. **End-to-end latency budgets** — "sensor-to-actuation within 100ms" cannot be expressed
3. **Coordinated synchronization** — "if lidar is late, drop the corresponding camera frame" requires ad-hoc application code

This document surveys the state of the art and identifies what's missing from current launch file specifications.

---

## 1. Data Loss Specification

### Current state: ROS 2 QoS

ROS 2 QoS offers binary reliability — `RELIABLE` (retransmit until delivered) or `BEST_EFFORT` (fire and forget). There is no middle ground:

| Policy                     | Behavior                                   | Gap                                      |
|----------------------------|--------------------------------------------|------------------------------------------|
| `reliability: RELIABLE`    | Retransmit, blocks publisher if queue full | No bounded loss, unbounded latency       |
| `reliability: BEST_EFFORT` | Drop silently                              | No loss budget, no feedback              |
| `history: KEEP_LAST(N)`    | Ring buffer of depth N                     | Overflow is silent, not budgeted         |
| `lifespan: Duration`       | Expire stale samples                       | Binary fresh/stale, no degradation curve |
| `deadline: Duration`       | Trigger callback on missed deadline        | Detection only, no corrective action     |

**What's missing**: A policy like `max_loss_ratio: 0.1` (tolerate 10% loss) or `max_consecutive_drops: 3` does not exist. Applications that need bounded loss must implement their own monitoring and backpressure.

### Related work

- **AUTOSAR Adaptive Platform** (AP R22-11): `E2EProtection` profiles specify `maxNoNewOrRepeatedData` — a count of consecutive missing/repeated messages before declaring communication loss. This is the closest industry standard to bounded-loss specification.
- **Data Distribution Service (DDS)**: The `TRANSPORT_PRIORITY` QoS policy enables differentiated treatment but has no loss budget. The OMG DDS-XRCE (Extremely Resource Constrained Environments) profile adds `best_effort_with_history` but still no loss ratio.
- **TSN (IEEE 802.1Qci)**: Per-stream filtering and policing can enforce frame loss ratios at the network layer, but this is below the middleware abstraction.

### Proposed vocabulary

```yaml
# Hypothetical launch-level data quality annotation
- topic:
    name: /lidar/points
    quality:
      max_loss_ratio: 0.05        # 5% loss budget over sliding window
      max_consecutive_drops: 2     # Alert after 3 consecutive misses
      window_size: 100             # Evaluate over last 100 messages
```

---

## 2. End-to-End Latency Budgets

### Current state

ROS 2 has no concept of end-to-end latency across a chain of nodes. The `deadline` QoS policy is per-topic and per-subscription — it detects missed deadlines on a single hop, not across a pipeline.

Autoware's planning-to-control path (`/planning/scenario_planning/trajectory` → `/control/command/control_cmd`) has an implicit real-time requirement (~100ms) that exists only in human knowledge and design documents.

### Cause-effect chains (AUTOSAR TIMEX)

The AUTOSAR Timing Extensions (TIMEX) specification defines:

- **Event Chain**: An ordered sequence of stimulus → response events across ECUs
- **Maximum Reaction Time (MRT)**: Worst-case time from stimulus to response (includes all buffering, scheduling, network delays)
- **Maximum Data Age (MDA)**: Maximum time a data item may be used after generation

These map directly to ROS 2 node chains:

```
sensor_driver → preprocessor → detector → tracker → planner → controller
|<----------------------- MRT ≤ 100ms ------------------------------>|
                                         |<-- MDA ≤ 50ms ---------->|
```

### Theoretical foundations

| Framework                                                  | Key concept                                                 | Applicability                                                                    |
|------------------------------------------------------------|-------------------------------------------------------------|----------------------------------------------------------------------------------|
| **Network Calculus** (Le Boudec & Thiran)                  | Arrival curves, service curves → deterministic delay bounds | Formal worst-case analysis of node chains; requires known processing time bounds |
| **Logical Execution Time (LET)** (Henzinger et al., 2001)  | Read at period start, write at deadline                     | Deterministic timing, composable, used in automotive (AUTOSAR CP)                |
| **Age of Information (AoI)** (Kaul et al., 2012)           | Time since generation of latest received update             | Captures freshness, not just latency; optimal scheduling results exist           |
| **Synchronous Dataflow (SDF)** (Lee & Messerschmitt, 1987) | Static token rates → compile-time schedule                  | Multi-rate pipelines (10Hz lidar + 30Hz camera) analyzable offline               |
| **Cyclo-Static Dataflow (CSDF)**                           | Cyclic token rate patterns                                  | More expressive than SDF for bursty patterns                                     |

### Lingua Franca / Reactor Model

**Lingua Franca** (Lohstroh et al., UC Berkeley) is a polyglot coordination language built on the reactor model:

- Reactions have declared inputs/outputs with **logical time** semantics
- Deadlines are first-class: `deadline(50 msec) {= /* handler */ =}`
- Connections can specify `after(10 msec)` logical delays
- The runtime (Xronos) enforces deterministic execution order

Benchmarked against ROS 2 (`ros-lingua_franca` bridge): **2-3x lower latency, 10x less jitter** for typical pub-sub chains. The key insight is that Lingua Franca separates *logical time* (for correctness) from *physical time* (for real-time), while ROS 2 conflates them.

### Contract-based timing

Derler, Lee, and Sangiovanni-Vincentelli (2013) propose **assume-guarantee contracts** for timing:

- **Assume**: "Input arrives within [0, 10ms] of period start"
- **Guarantee**: "Output produced within [0, 5ms] of input"
- **Composition**: Chain contracts → end-to-end bound

This enables modular verification: each node's timing contract is verified independently, and the system-level property follows from composition.

**Plan B** (Zimmer et al.): Runtime monitoring of timing contracts with automatic degradation (e.g., switch to simpler algorithm if deadline is at risk).

### Proposed vocabulary

```yaml
# Hypothetical launch-level latency chain annotation
- latency_chain:
    name: sensor_to_actuation
    stimulus: /sensing/lidar/points          # First topic
    response: /control/command/control_cmd   # Last topic
    max_reaction_time: 100ms                 # End-to-end budget
    max_data_age: 50ms                       # At response point
    nodes:                                   # Optional: per-node budgets
      - name: pointcloud_preprocessor
        budget: 15ms
      - name: lidar_centerpoint
        budget: 30ms
      - name: multi_object_tracker
        budget: 20ms
      - name: behavior_path_planner
        budget: 25ms
      - name: vehicle_cmd_gate
        budget: 10ms
```

---

## 3. Coordinated Synchronization / Frame Dropping

### The problem

Multi-sensor fusion requires synchronized data from parallel pipelines:

```
lidar_driver ──→ lidar_preprocess ──→ lidar_detect ──┐
                                                       ├──→ fusion ──→ tracker
camera_driver ──→ camera_preprocess ──→ camera_detect ─┘
```

If `lidar_detect` drops a frame (overload, scheduling delay), the fusion node receives mismatched data. Worse, `camera_detect` may process a frame that will never be fused — wasting compute. Current ROS 2 has no mechanism to propagate "this frame is abandoned" across pipelines.

### Existing approaches

**Autoware's collector pattern**: Timeout-based collectors wait for all inputs up to a deadline, then proceed with whatever arrived. The `multi_object_tracker` uses `message_filters::ApproximateTimeSynchronizer` or custom collectors. Incomplete data sets are either discarded or processed with reduced confidence. This is entirely application-level — the launch system has no visibility into it.

**ROS 2 `message_filters`**: `TimeSynchronizer` (exact) and `ApproximateTimeSynchronizer` (within tolerance) match messages by header timestamp. Limitations:
- No backpressure to upstream nodes
- No cross-pipeline frame cancellation
- No launch-time configuration (hardcoded in node source)

### Theoretical foundations

| Framework                                       | Key concept                                                 | Applicability                                                                                          |
|-------------------------------------------------|-------------------------------------------------------------|--------------------------------------------------------------------------------------------------------|
| **Timely Dataflow** (Naiad, Murray et al. 2013) | Progress tracking via pointstamps and capabilities          | Formal model for "all inputs for epoch T have arrived" — directly models frame-level synchronization   |
| **Flink/Spark Watermarks**                      | `Watermark(t)` = "no events with timestamp < t will arrive" | Bounded-out-of-orderness; minimum-across-inputs policy at merge points                                 |
| **Lustre/SCADE Clock Calculus**                 | `when`/`current` operators, multi-rate clocks               | "lidar at 10Hz, camera at 30Hz, fuse at 10Hz" — statically typed rates with formal clock relationships |
| **Scenario-Aware Dataflow (SADF)**              | Mode-dependent token rates                                  | Models degraded modes (e.g., lidar-only when camera drops)                                             |
| **ReactiveX**                                   | `withLatestFrom`, `combineLatest`, `zip`                    | Operator vocabulary for describing merge semantics                                                     |

### Timely Dataflow (most relevant)

Timely Dataflow introduces **frontiers** — a set of timestamps that a dataflow operator might still produce. When an operator advances its frontier past time `t`, all downstream operators know that no more data for time `t` will arrive.

Applied to ROS 2 sensor fusion:
- Each pipeline stage would announce "I will produce no more data for lidar frame 42"
- The fusion node's frontier advances when *both* inputs advance past frame 42
- If lidar drops frame 42, its frontier advances past 42 → fusion knows to skip → camera pipeline can be notified to skip

This is the formal foundation for coordinated frame dropping. The key primitive is **frontier advancement notification**, which ROS 2 lacks entirely.

### DejaVu: Impact of frame misalignment

DejaVu (Chen et al., 2025) demonstrated that even a single-frame LiDAR delay collapses MVXNet detection accuracy:

- **mAP drops from 84.1% to 9.7%** (88.5% degradation) with 1-frame LiDAR delay
- NDS (nuScenes Detection Score) drops from 86.2% to 14.7%
- The effect is asymmetric: camera delay has much less impact than LiDAR delay

This quantifies why coordinated frame dropping matters — misaligned fusion is worse than skipping a frame entirely.

### Proposed vocabulary

```yaml
# Hypothetical launch-level synchronization annotation
- sync_group:
    name: lidar_camera_fusion
    policy: coordinated_drop          # Drop aligned frames across pipelines
    timestamp_source: header           # Use message header timestamps
    tolerance: 50ms                    # Temporal alignment window

    inputs:
      - path: [lidar_driver, lidar_preprocess, lidar_detect]
        rate: 10Hz
        priority: primary              # If this drops, drop all

      - path: [camera_driver, camera_preprocess, camera_detect]
        rate: 30Hz
        downsample_to: 10Hz           # Match lidar rate at fusion point
        priority: secondary            # Follows primary's frame decisions

    merge:
      node: multi_sensor_fusion
      strategy: wait_all               # Wait for all inputs (vs. wait_any)
      timeout: 80ms                    # Proceed without stragglers after timeout
      on_incomplete: skip              # skip | degrade | use_latest
```

---

## 4. Gap Analysis: What Launch Files Cannot Express

| Capability                  | ROS 2 today                    | What's needed                      | Theoretical basis           |
|-----------------------------|--------------------------------|------------------------------------|-----------------------------|
| Bounded data loss           | Binary RELIABLE/BEST_EFFORT    | Loss ratio, consecutive drop limit | AUTOSAR E2E profiles        |
| Per-topic freshness         | `lifespan` QoS (binary)        | Graduated staleness, AoI tracking  | Age of Information theory   |
| End-to-end latency          | Per-hop `deadline` only        | Chain-level MRT/MDA budgets        | Network Calculus, TIMEX     |
| Per-node timing budget      | None                           | Assume-guarantee contracts         | Contract-based design       |
| Multi-rate specification    | None                           | Clock relationships (10Hz/30Hz)    | Lustre clock calculus, SDF  |
| Coordinated frame dropping  | None                           | Frontier-based progress tracking   | Timely Dataflow             |
| Cross-pipeline cancellation | None                           | "Drop frame N on all paths"        | Dataflow frontiers          |
| Degraded-mode switching     | None                           | Mode-dependent pipeline config     | SADF, Plan B monitoring     |
| Synchronization policy      | `message_filters` (code-level) | Launch-level merge semantics       | Flink watermarks, ReactiveX |

### Key insight

These are not independent features — they form a coherent **data quality contract** layer:

1. **Loss budgets** define acceptable degradation
2. **Latency chains** define timing requirements
3. **Sync groups** define coordination policies
4. **Degraded modes** define fallback behavior

A launch system that supports all four can reason about the system holistically — e.g., "if the lidar pipeline exceeds its 30ms budget, switch to lidar-only mode (SADF), which relaxes the fusion sync group and changes the end-to-end MRT from 100ms to 80ms."

---

## 5. Enforcement Mechanisms: What a Launch System Can Actually Do

The specification is declarative — it describes quality requirements. But ROS 2 data flow happens inside nodes via DDS, outside the launch system's control. This section catalogs every practical enforcement mechanism available.

### 5.1 Control surfaces available to a launch orchestrator

| Surface | Scope | When | What it controls |
|---------|-------|------|-----------------|
| **DDS vendor XML** (FastDDS/CycloneDDS) | Per-process, per-topic | Startup | QoS policies at DDS level |
| **Environment variables** (`FASTRTPS_DEFAULT_PROFILES_FILE`, `CYCLONEDDS_URI`) | Per-process | Startup | DDS config file selection |
| **ROS parameters** (`qos_overrides.*`) | Per-node, per-topic | Startup | QoS policies (node must opt in) |
| **Topic remapping** (`<remap>`) | Per-node | Startup | Topic routing |
| **Proxy node injection** (`topic_tools`) | Per-topic | Runtime | Rate limiting, delay, drop patterns |
| **Container `on_load_node`** | Per-composable-node | Load time | Parameter and remap injection |
| **`ros2_tracing`** (LTTng) | System-wide | Runtime | Observation only (latency, throughput) |
| **`software_watchdogs`** | Per-topic | Runtime | Deadline/liveliness violation detection |

### 5.2 DDS QoS injection (no node changes required)

A launch orchestrator can generate DDS XML config files and inject them per-node via environment variables scoped with `<group>` or `<env>`:

```xml
<!-- Launch file scopes DDS config to a specific node -->
<group>
  <env name="FASTRTPS_DEFAULT_PROFILES_FILE" value="$(dirname)/qos/lidar_node.xml"/>
  <env name="RMW_FASTRTPS_USE_QOS_FROM_XML" value="1"/>
  <node pkg="sensing" exec="lidar_driver" name="lidar_driver"/>
</group>
```

**Critical limitation**: Only policies set to `SYSTEM_DEFAULT` in node code are overridable. If a node hardcodes `reliability: reliable`, the XML cannot change it. This works well for nodes that follow REP 2003 conventions (sensor nodes use `SensorDataQoS` which defaults appropriately).

**Composable node limitation**: All composable nodes in a container share the same process environment. Per-node QoS differentiation requires either separate containers or the parameter-based `qos_overrides` mechanism.

### 5.3 Parameter-based QoS overrides (node must opt in)

Nodes using `rclcpp::QosOverridingOptions` expose QoS as read-only parameters:

```yaml
# Injected via <param from="..."/> in launch file
/lidar_driver:
  ros__parameters:
    qos_overrides:
      /lidar/points:
        publisher:
          reliability: best_effort
          history: keep_last
          depth: 5
          deadline: 100000000  # nanoseconds
```

This is the most granular mechanism — works per-node, per-topic, per-entity (publisher vs subscriber), and works for composable nodes. However, most existing Autoware nodes do **not** opt in.

### 5.4 Proxy node injection via topic_tools + remapping

The `topic_tools` package provides type-agnostic proxy nodes that operate on serialized messages (zero deserialization overhead):

| Proxy | Effect | Use case |
|-------|--------|----------|
| `throttle` | Rate-limit to N Hz | Enforce rate budgets |
| `drop` | Drop X of every Y messages | Simulate/enforce loss ratios |
| `delay` | Add fixed latency | Test latency tolerance |
| `relay` | Forward topic A → B | Transparent interception point |
| `mux` | N:1 topic switching | Degraded-mode input selection |

A launch orchestrator can **transparently insert proxies** by:
1. Remapping the original publisher: `/lidar/points` → `/lidar/points_raw`
2. Spawning a throttle node: subscribes `/lidar/points_raw`, publishes `/lidar/points`

Since `play_launch` already controls the launch graph and generates `record.json`, it could inject these modifications before replay. The subscriber sees the expected topic name unchanged.

### 5.5 Container-level interception

`play_launch`'s `ObservableComponentManager` already intercepts `LoadNode` service calls. This could be extended to:
- Inject `qos_overrides` parameters into `request->parameters` before node creation
- Add remap rules to `request->remap_rules` to route topics through proxy nodes
- Log all requested topic names for graph analysis

This is the only mechanism that can modify composable node configuration at load time without changing node source code.

### 5.6 What CANNOT be enforced from outside

| Requirement | Why it's unreachable |
|-------------|---------------------|
| **Coordinated frame dropping** | Requires frontier propagation between nodes — no DDS primitive exists. Nodes must participate. |
| **Cross-pipeline cancellation** | Requires a shared "frame epoch" concept that DDS topics don't carry. |
| **message_filters tolerance** | `ApproximateTimeSynchronizer` `slop`/`queue_size` are constructor args, not parameters. |
| **Content-filtered topics** | Requires programmatic setup via `SubscriptionOptions` — no external config. |
| **Dynamic QoS changes** | `qos_overrides` are read-only at startup. No runtime QoS modification. |

### 5.7 The specification-enforcement gap

For a launch-level quality specification to be useful, it must map to enforceable primitives:

| Spec element | Enforceable? | Mechanism |
|-------------|-------------|-----------|
| Topic QoS (reliability, history, deadline, lifespan) | Partially | DDS XML + `qos_overrides` (if node cooperates) |
| Per-topic rate limit | Yes | `topic_tools/throttle` proxy injection |
| Loss budget monitoring | Yes (observe) | Generic subscription + counter |
| End-to-end latency monitoring | Yes (observe) | `ros2_tracing` + header timestamp analysis |
| Per-node timing budget | No (enforce) | Requires node-internal instrumentation |
| Sync group coordination | No | Requires new middleware primitive |
| Degraded-mode switching | Partially | Node lifecycle transitions + `mux` topic switching |

The practical implication: **a launch-level spec can fully specify intent, but enforcement is a spectrum from "fully automatic" (QoS injection) to "monitoring only" (latency chains) to "requires new infrastructure" (coordinated frame dropping).**

---

## 6. Evidence from Autoware/AutoSDV

Concrete patterns found in the AutoSDV codebase that validate the need for launch-level quality specification:

### 6.1 Synchronization parameters already externalized

Image projection fusion (`fusion_common.param.yaml`) exposes per-camera offsets, timeouts, and match thresholds:

```yaml
input_offset_ms: [61.67, 111.67, 45.0, 28.33, 78.33, 95.0]
timeout_ms: 70.0
match_threshold_ms: 50.0
image_buffer_size: 15
```

Grid map fusion (`synchronized_grid_map_fusion_node.param.yaml`):

```yaml
match_threshold_sec: 0.01   # 10ms
timeout_sec: 0.1            # 100ms
input_offset_sec: [0.0, 0.0]
```

These demonstrate that Autoware developers *want* externalized timing configuration — they just lack a unified vocabulary.

### 6.2 Synchronization parameters still hardcoded

| Component | Hardcoded value | What should be configurable |
|-----------|----------------|-----------------------------|
| Seyond multi-lidar fusion | Queue size = 20 | Sync buffer depth |
| Seyond multi-lidar fusion | `ApproximateTime` only | Sync policy selection |
| Ground filter | `message_filters` default tolerance | Sync tolerance (slop) |
| All perception nodes | `SensorDataQoS` | Reliability, history, deadline |
| Point cloud accumulator | Cache size = 10 | Temporal buffer depth |

### 6.3 Completely absent

- **No `deadline` or `lifespan` QoS usage** anywhere in AutoSDV — data staleness is detected only by timeout
- **No explicit data age validation** — freshness is implicit through synchronizer timeouts
- **No cross-pipeline coordination** — if lidar drops a frame, camera pipeline is unaware
- **No end-to-end latency specification** — the 100ms sensor-to-actuation budget exists only in design docs

---

## 7. Specification Design: Layered Approach

Given the enforcement gap, a practical specification should have three layers:

### Layer 1: QoS profiles (fully enforceable)

Maps directly to DDS QoS and can be injected via XML or `qos_overrides`:

```yaml
quality_profiles:
  sensor_data:
    reliability: best_effort
    history: keep_last
    depth: 5
    deadline: 100ms
    lifespan: 500ms

  control_cmd:
    reliability: reliable
    history: keep_last
    depth: 1
    deadline: 10ms

topic_assignments:
  /sensing/lidar/points: sensor_data
  /sensing/camera/*/image: sensor_data
  /control/command/*: control_cmd
```

**Enforcement**: Generate per-node FastDDS XML files and inject via `FASTRTPS_DEFAULT_PROFILES_FILE` in launch groups.

### Layer 2: Data flow policies (enforceable via proxy injection)

```yaml
flow_policies:
  - topic: /sensing/camera/*/image
    max_rate: 10Hz              # Throttle from 30Hz to match lidar rate

  - topic: /perception/objects
    max_rate: 10Hz
    drop_policy: newest_first   # Under overload, drop newest (keep pipeline fresh)
```

**Enforcement**: Generate `topic_tools/throttle` nodes and remap rules, inject into `record.json`.

### Layer 3: Quality contracts (monitoring + alerting only)

```yaml
contracts:
  sensor_to_actuation:
    type: latency_chain
    stimulus: /sensing/lidar/points
    response: /control/command/control_cmd
    max_reaction_time: 100ms
    severity: critical           # Triggers alert, not enforcement

  lidar_camera_sync:
    type: sync_group
    inputs:
      - /perception/lidar/objects
      - /perception/camera/objects
    tolerance: 50ms
    on_violation: log            # Future: could trigger degraded mode

  lidar_availability:
    type: loss_budget
    topic: /sensing/lidar/points
    max_loss_ratio: 0.05
    window: 100
    on_violation: warn
```

**Enforcement**: Runtime monitoring via generic subscriptions and `ros2_tracing`. Violations produce diagnostics on `/diagnostics` or structured log events. No automatic correction — the spec documents intent and enables automated verification.

### Why three layers?

1. **Layer 1** is immediately useful — can be implemented today with zero node changes
2. **Layer 2** requires proxy management in the launch orchestrator but no node changes
3. **Layer 3** requires a monitoring framework but establishes the vocabulary for future enforcement (e.g., when nodes adopt frontier-based coordination)

The spec file itself would live alongside launch files (e.g., `quality.yaml`) and be referenced from the launch configuration. `play_launch` would read it, generate DDS configs and proxy nodes for Layers 1-2, and spawn a monitoring node for Layer 3.

---

## 8. References

1. Le Boudec, J.-Y., & Thiran, P. (2001). *Network Calculus: A Theory of Deterministic Queuing Systems for the Internet*. Springer.
2. Henzinger, T. A., Horowitz, B., & Kirsch, C. M. (2001). Giotto: A time-triggered language for embedded programming. *EMSOFT*.
3. Kaul, S., Yates, R., & Gruteser, M. (2012). Real-time status: How often should one update? *IEEE INFOCOM*.
4. Lee, E. A., & Messerschmitt, D. G. (1987). Synchronous data flow. *Proceedings of the IEEE*, 75(9).
5. Murray, D. G., et al. (2013). Naiad: A timely dataflow system. *SOSP*.
6. Lohstroh, M., et al. (2021). Toward a lingua franca for deterministic concurrent systems. *ACM TECS*.
7. Derler, P., Lee, E. A., & Sangiovanni-Vincentelli, A. (2013). Modeling cyber-physical systems. *Proceedings of the IEEE*.
8. Chen, Q., et al. (2025). DejaVu: Idempotent and consistent multi-sensor 3D object detection via temporal alignment. *CVPR*.
9. AUTOSAR (2022). *Specification of Timing Extensions (TIMEX)*. R22-11.
10. Casini, D., et al. (2019). Response-time analysis of ROS 2 processing chains. *ECRTS*.
11. Blass, T., et al. (2021). A ROS 2 response time analysis exploiting starvation freedom and execution time variation. *RTAS*.
