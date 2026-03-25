# Launch Manifest

**Declaring and Auditing the ROS 2 Communication Graph**

play_launch — Phase 31

---

## Problem

- Launch files declare **nodes**, not **topics**
- Topic creation happens in source code — invisible until runtime
- Autoware: ~110 nodes create ~500 topics
- A code change adds/removes a topic → goes unnoticed
- No way to answer: "what SHOULD the graph look like?"

---

## Manifest in 30 Seconds

```yaml
version: 1

nodes:
  talker:
    pub: [chatter]            # endpoint = pre-remap name
  listener:
    sub: [chatter]

topics:
  chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]     # who publishes
    sub: [listener/chatter]   # who subscribes

exports:
  output: [talker/chatter]
```

One manifest per launch file. Topics wire endpoints. Names are relative.

---

## Manifest Structure

```
┌─────────────────────────────────────────────┐
│  nodes:       endpoint declarations         │
│  topics:      wiring + type + QoS           │
│  includes:    child scopes (namespace+path) │
│  imports:     subscriber dependencies       │
│  exports:     publisher outputs             │
│  io:          scope-level timing contract   │
└─────────────────────────────────────────────┘
```

- `nodes:` — what the launch file runs
- `topics:` — how they connect (the graph)
- `includes:` — child launch files as subscopes
- `imports:`/`exports:` — the scope's boundary
- `io:` — timing contract

---

## Wiring: Topics Connect Endpoints

```yaml
nodes:
  cropbox_filter:
    pub: [output]             # pre-remap topic name
    sub: [input]

topics:
  cropped:                    # real topic name → <ns>/cropped
    type: PointCloud2
    pub: [cropbox_filter/output]
    sub: [ground_filter/input]
    contract:
      rate_hz: 10
```

- Endpoint names = node's internal names (before `<remap>`)
- Topic gives the real name and wires pub ↔ sub
- `contract:` = channel-level quality bound

---

## Composition: Includes and Scope Hierarchy

```yaml
includes:
  lidar:                      # name = ROS namespace
    manifest: tier4_perception_launch/lidar_perception.launch.yaml
  camera:
    manifest: tier4_perception_launch/camera_perception.launch.yaml

topics:
  lidar_objects:
    pub: [lidar/output]       # child's export
    sub: [fusion_node/lidar_in]
  camera_objects:
    pub: [camera/output]       # child's export
    sub: [fusion_node/camera_in]

imports:
  input: [lidar/input, camera/input]   # aggregate children's imports
exports:
  output: [tracker/tracked]
```

Parent wires children explicitly via topics. No implicit accumulation.

---

## I/O Patterns in Autoware

| Pattern      | Trigger                      | Example                                     |
|--------------|------------------------------|---------------------------------------------|
| Pipe (1→1)   | `on_arrival`                 | cropbox_filter, centerpoint, ground_filter  |
| Fusion (N→1) | `all_ready` + `timestamp`    | concatenate_pointclouds, radar_fusion       |
| Source (0→N) | `periodic`                   | lidar_driver (10 Hz), camera_driver (30 Hz) |
| Timer-driven | `periodic` + `state` inputs  | multi_object_tracker (10 Hz)                |
| Feedback     | `on_arrival` + `state` cycle | NDT ↔ EKF localization                      |

All built from the same primitives — no enumerated pattern types.

---

## Node I/O Contract

```yaml
nodes:
  centerpoint:
    sub: [pointcloud]
    pub: [objects]
    io: { latency_ms: 30 }           # simple pipe

  fusion_node:
    sub: [lidar_objects, camera_objects]
    pub: [fused]
    io:
      trigger: all_ready              # wait for all inputs
      correlation: timestamp          # match by header.stamp
      tolerance_ms: 50
      latency_ms: 20

  ndt_scan_matcher:
    sub: [points_raw, ekf_pose]
    pub: [ndt_pose]
    io:
      state: [ekf_pose]              # read-latest, not trigger
      required: [map]                 # must receive at least once
      latency_ms: 50
```

`io:` only adds annotations — no redundancy with `sub:`/`pub:`.

---

## Contract Primitives

**Input roles** — per-endpoint annotations on `sub:` list

| Property    | Default   | Meaning                                            |
|-------------|-----------|----------------------------------------------------|
| `role`      | `trigger` | `trigger` fires computation; `state` = read-latest |
| `consume`   | 1         | Messages per firing (SDF rate)                     |
| `min_count` | 1/0       | Readiness precondition                             |

**Trigger** — what causes output

| Value         | Meaning                             |
|---------------|-------------------------------------|
| `on_arrival`  | Any trigger-input arrival (default) |
| `all_ready`   | All trigger-inputs must arrive      |
| `periodic: N` | Timer-driven at N Hz                |

**Timing** — quantitative bounds

| Field          | Meaning               |
|----------------|-----------------------|
| `latency_ms`   | Max trigger → output  |
| `freshness_ms` | Max age of input data |

---

## Topic Channel Contract

```yaml
topics:
  detected_objects:
    type: DetectedObjects
    pub: [centerpoint/objects]
    sub: [tracker/input]
    contract:
      rate_hz: 10
      deadline_ms: 150
      max_drop_ratio: 0.05
      window: 100
```

Separate from node `io:`:
- **Node `io:`** = computation (how fast input → output)
- **Topic `contract:`** = channel (what the link promises)

---

## Full Pipeline Example

```
planning_simulator
├── sensing               source, 10 Hz
│   ├── lidar_driver      periodic: 10
│   └── camera_driver     periodic: 30
├── perception            latency_ms: 85
│   ├── lidar             latency_ms: 50
│   │   ├── cropbox       5 ms
│   │   ├── ground        15 ms
│   │   └── centerpoint   30 ms
│   ├── camera            latency_ms: 30
│   │   ├── rectifier     5 ms
│   │   └── yolo          25 ms
│   ├── fusion_node       all_ready, timestamp, 20 ms
│   └── tracker           periodic: 10
├── planning              latency_ms: 100
└── control               latency_ms: 10, deadline: 100 ms
```

Critical path: max(50, 30) + 20 + 100 = **170 ms**

---

## Working with Launch Files

```
<include pkg="tier4_perception_launch" file="lidar.launch.xml">
  ←→  includes:
         lidar:
           manifest: tier4_perception_launch/lidar.launch.yaml
```

- Manifest loaded alongside `<include>` — parser resolves namespace
- Endpoint names match `<remap from="...">` in launch XML (warn if missing)
- Same manifest reused across different namespace contexts
- Partial coverage: missing manifest = no auditing for that scope
- Full ROS 2 compatibility — manifests are sidecar files, not replacements

---

## Contract Violation Detection

**RCL Interception (Phase 29)**

```
┌──────────────┐    LD_PRELOAD    ┌──────────────┐
│  ROS 2 Node  │ ──────────────→ │  Interception │
│  rcl_publish  │                 │    .so        │
│  rcl_take     │                 │  hooks pub/   │
└──────────────┘                  │  take calls   │
                                  └──────┬───────┘
                                         │ zero-copy
                                         │ SPSC ring
                                  ┌──────▼───────┐
                                  │  play_launch  │
                                  │  consumer     │
                                  │  task         │
                                  └──────────────┘
```

Captures per message:
- Monotonic timestamp (when pub/take happened)
- `header.stamp` (data timestamp)
- Topic name, node name

---

## What the Monitor Checks

| Contract field            | Check                        | Data source     |
|---------------------------|------------------------------|-----------------|
| `io.latency_ms`           | pub_time - take_time ≤ bound | Interception    |
| `io.freshness_ms`         | pub_time - stamp ≤ bound     | Frontier plugin |
| `contract.rate_hz`        | interval ≈ 1/rate            | Stats plugin    |
| `contract.deadline_ms`    | interval ≤ deadline          | Stats plugin    |
| `contract.max_drop_ratio` | drops/total ≤ ratio          | Stats plugin    |

**4-way diagnosis:**

| Assumption | Guarantee | →                           |
|:----------:|:---------:|:----------------------------|
|     OK     |    OK     | Nominal                     |
|     OK     |   Fail    | **Node bug**                |
|    Fail    |    OK     | Node robust beyond contract |
|    Fail    |   Fail    | Upstream problem            |

---

## Scope Budget Verification

**Critical path = longest path in DAG** (O(V+E))

```
scope.io.latency_ms ≥ longest_path(imports → exports)
```

Composition rules:
- Series: sum
- Parallel: max
- Periodic: period + jitter

```python
# petgraph: topo_sort + DP
for node in topo_order:
    for edge in node.outgoing:
        dist[edge.target] = max(dist[edge.target],
                                dist[node] + node.latency)
# dist[export] = critical path
```

Checked statically at parse time — before any node starts.

---

## Contract Theory Overview

**Assume-Guarantee**: C = (A, G)
- A = input constraints (rate, availability)
- G = output promises (latency, rate) — valid only when A holds

**Refinement**: C' refines C ⟺ weaker A' + stronger G'
- Faster node = valid upgrade
- More tolerant node = valid upgrade

**QoS as base layer**: reliability ≥, durability ≥, deadline ≤

**Empirical derivation**: capture → observe → derive contracts
```
guarantee = max(observed) × margin
```

See `docs/design/contract-theory.md` for full formalization.

---

## Verification Tooling Path

| Tier | Tool                 | What                                             | When       |
|------|----------------------|--------------------------------------------------|------------|
| 1    | `petgraph` + Rust    | Critical path, scope budgets, QoS, types, wiring | Phase 31   |
| 2    | Hand-rolled monitors | Runtime rate, deadline, latency, drops           | With audit |
| 3    | `z3` / `good_lp`     | Constraint satisfiability, budget optimization   | If needed  |

Tier 1 covers ~95% of checks with zero new dependencies.

---

## Workflow

```
1. CAPTURE
   play_launch launch <pkg> <file> --save-manifest-dir manifests/
   Run → stabilize → snapshot → write per-launch-file manifests

2. REFINE
   Edit manifests: add io: contracts, tighten bounds

3. AUDIT
   play_launch launch <pkg> <file> --manifest-dir manifests/
   Parse-time: scope budgets, QoS, wiring
   Runtime: latency, rate, drops via interception
```

Gradual adoption: start with topology (nodes + topics), add timing later.

---

## Status and Next Steps

**Done:**
- Manifest format design (v1)
- I/O contract primitives (trigger, correlation, timing, drops)
- RCL interception infrastructure (Phase 29)
- Scope table in record.json (Phase 30)
- Contract theory formalization

**Next:**
- Manifest parser (Rust crate)
- Parse-time consistency checks (petgraph)
- Capture mode (graph → per-scope manifests)
- Runtime audit (interception → monitor → warnings)
- Web UI manifest diff view
