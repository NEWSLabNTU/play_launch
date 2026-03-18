# Launch Manifest & RCL Interception

## play_launch — Communication Graph Specification and Enforcement

---

## 1. The Problem

ROS 2 launch files declare **nodes** but not **topics**.

```xml
<!-- autoware planning_simulator.launch.xml -->
<include pkg="tier4_sensing_launch" file="sensing.launch.xml"/>
<include pkg="tier4_perception_launch" file="perception.launch.xml"/>
<include pkg="tier4_planning_launch" file="planning.launch.xml"/>
```

What's missing:

- What topics do these nodes create?
- What QoS do they use? Are publisher/subscriber QoS compatible?
- Did a code change add or remove a topic between releases?

Autoware: **~110 nodes, ~500 topics** — invisible until runtime.

---

## 2. The Solution: Launch Manifest

A **manifest file** per launch file describes the expected communication
graph. The parser loads manifests alongside launch files, and the executor
audits the runtime graph against them.

```
                --manifest-dir ./manifests/
                       │
  Launch files ──→ Parser ──→ record.json
                     │            ├── nodes, params, ...   (existing)
                     │            └── expected_graph        (new, from manifests)
                     │
               Manifest files
               manifests/<package>/<launch>.yaml
```

**Two-phase audit:**

| Phase      | When               | What                                |
|------------|--------------------|-------------------------------------|
| Parse-time | Before nodes start | QoS compatibility, type consistency |
| Runtime    | During execution   | Actual graph vs expected graph      |

---

## 3. Manifest Format

Each manifest describes one launch file. Names are **relative** — the
parser applies the namespace from the include context.

```yaml
# manifests/tier4_perception_launch/lidar_perception.launch.yaml

topics:
  cropped: sensor_msgs/msg/PointCloud2           # shorthand: type, default QoS
  no_ground: sensor_msgs/msg/PointCloud2
  objects:                                        # full form: type + QoS
    type: autoware_perception_msgs/msg/DetectedObjects
    qos: { reliability: best_effort, depth: 1 }

nodes:
  cropbox_filter:
    pub: [cropped]
    sub: [/sensing/pointcloud]                   # absolute = cross-namespace
  ground_filter:
    sub: [cropped]
    pub: [no_ground]
  centerpoint:
    sub: [no_ground]
    pub: [objects]

interface:
  pub: [objects]                                  # exposed to parent

requirements:
  latency_ms: 50
```

**Key concepts:**

| Concept | What it is |
|---------|------------|
| **Topic** | Communication channel — carries message type + QoS |
| **Node** | Execution entity — references topics by name (pub/sub/srv/cli) |
| **Component** | `<group>` within a manifest — optional namespace grouping |
| **Interface** | Topics that cross the launch file boundary — internal by default |
| **Service/Action** | Same pattern as topics — declared with type, referenced by nodes |

Connection is by **topic name matching** — same as ROS 2. No edges needed.

---

## 4. Autoware Example

```
manifests/
├── autoware_launch/
│   └── planning_simulator.launch.yaml     ← root (cross-component topics)
├── tier4_sensing_launch/
│   └── sensing.launch.yaml                ← 3 nodes
├── tier4_perception_launch/
│   ├── perception.launch.yaml             ← fusion + tracker
│   ├── lidar_perception.launch.yaml       ← 3 nodes
│   └── camera_perception.launch.yaml      ← 2 nodes
└── tier4_planning_launch/
    └── planning.launch.yaml               ← 2 nodes
```

```yaml
# manifests/autoware_launch/planning_simulator.launch.yaml
topics:
  /sensing/pointcloud:
    type: sensor_msgs/msg/PointCloud2
    qos: { reliability: best_effort, depth: 1 }
  /perception/tracked_objects:
    type: autoware_perception_msgs/msg/TrackedObjects
    qos: { reliability: reliable, depth: 1 }
  /planning/trajectory:
    type: autoware_planning_msgs/msg/Trajectory
    qos: { reliability: reliable, durability: transient_local, depth: 1 }

interface:
  pub: [/planning/trajectory]
```

Cross-component topics with QoS live in the root manifest. Internal topics
live in each component's manifest. ~50 cross-component + ~450 internal,
split across ~10 files.

---

## 5. RCL Interception

`libplay_launch_interception.so` is injected via LD_PRELOAD into every
managed node. It hooks RCL functions to observe communication transparently.

```
  Node process (unmodified code)
  ┌────────────────────────────────────────────┐
  │  rcl_publisher_init(pub, node, topic, ..)  │
  │       │                                    │
  │       v                                    │
  │  ┌──────────────────────────────────────┐  │
  │  │ libplay_launch_interception.so       │  │
  │  │                                      │  │
  │  │  InterceptionPlugin trait            │  │
  │  │    ├── FrontierPlugin (timestamps)   │  │
  │  │    ├── StatsPlugin (counts, rates)   │  │
  │  │    └── GraphPlugin (future)          │  │
  │  └──────────┬───────────────────────────┘  │
  │             │ SPSC ring buffer (zero-copy)  │
  └─────────────┼──────────────────────────────┘
                v
  play_launch ──→ audit, log, web UI
```

| Hook | When | Purpose |
|------|------|---------|
| `rcl_publisher_init` | Endpoint created | Detect new publisher |
| `rcl_subscription_init` | Endpoint created | Detect new subscriber |
| `rcl_publish` | Every message | Timestamp tracking, stats |
| `rcl_take` | Every message | Receive-side stats |

Future hooks: `rcl_service_init`, `rcl_client_init`,
`rcl_action_server_init`, `rcl_action_client_init`.

---

## 6. Manifest Enforcement via RCL Interception

**Evolution from monitoring to enforcement:**

```
v1: Polling                    v2: Event-driven              v3: Blocking
─────────────────              ──────────────────             ─────────────────
play_launch polls              rcl_publisher_init hook        Hook checks expected
GraphSnapshot every Ns    →    sends event via SPSC      →   graph in-process
                               play_launch diffs instantly    Returns RCL_RET_ERROR
Warn on deviations             Warn on deviations            to reject unauthorized
                                                             publishers
```

**v1 (Phase 30):** Executor periodically builds `GraphSnapshot`, diffs
against expected graph from `record.json`. Warn-only.

**v2:** `GraphPlugin` reports endpoint creation events through the SPSC
ring buffer. play_launch consumer diffs immediately — detection before the
first message is published.

**v3:** The interception .so loads the expected graph via shared memory.
On `rcl_publisher_init`, it checks the topic against the manifest. If
unauthorized, the hook returns an error — the publisher is never created.

---

## 7. Quality Contracts

Contracts are specified at **per-node** and **per-chain** granularity, using
AUTOSAR TIMEX semantics (MRT, MDA) and CARET measurement definitions.

**Per-node timing contract** (assume-guarantee):
```yaml
nodes:
  centerpoint:
    requirements:
      node_latency_ms: 30         # t_publish - t_subscribe (p99)
      max_deadline_misses: 3
```

**End-to-end latency chain** (stimulus → response):
```yaml
chains:
  sensor_to_actuation:
    stimulus: /sensing/pointcloud
    response: /control/command
    max_reaction_time_ms: 100     # MRT: cause → first effect
    max_data_age_ms: 50           # MDA: data freshness at output
    node_budgets:                 # optional per-node breakdown
      - { node: /perception/centerpoint, budget_ms: 30 }
      - { node: /planning/motion_planner, budget_ms: 35 }
```

**Reliability contract** (per-topic):
```yaml
reliability:
  /sensing/pointcloud:
    max_loss_ratio: 0.05          # 5% over sliding window
    max_consecutive_drops: 2
  /planning/trajectory:
    deadline_ms: 100              # max inter-message interval
    max_deadline_misses: 0        # zero tolerance
```

**Fork-join**: MRT at merge = `max(branch MRTs) + merge node budget`.

**Measurement → enforcement path:**

| Metric | Measured by | Monitoring | Enforcement |
|--------|------------|------------|-------------|
| Node latency | Interception: `t_pub - t_sub` | Warn on budget miss | Degrade mode (future) |
| Chain MRT | Frontier: sensor vs control | Warn on threshold | — |
| Loss ratio | Stats: `1 - take/pub` | Warn on threshold | Rate-limit upstream (future) |
| QoS compat | Parse-time: manifest QoS | Error before start | Block at `rcl_*_init` (v3) |
| Sync | Frontier: temporal gaps | Warn | Coordinated drop (future) |

All monitoring uses existing Phase 29 interception (FrontierPlugin +
StatsPlugin). No new hooks needed for measurement.

---

## 8. Summary

| Layer                 | What                                                    | How                                                     |
|-----------------------|---------------------------------------------------------|---------------------------------------------------------|
| **Manifest**          | Expected graph: topics, types, QoS, interfaces          | YAML files per launch file, loaded by parser            |
| **record.json**       | Expected graph embedded alongside execution plan        | Optional field, backward compatible                     |
| **RCL interception**  | Runtime observation: every endpoint init, every message | LD_PRELOAD, zero-copy SPSC ring buffer                  |
| **Audit**             | Diff expected vs actual                                 | Parse-time (static) + runtime (dynamic)                 |
| **Enforcement**       | Reject unauthorized endpoints                           | Future: in-process check at hook time                   |
| **System properties** | Latency, drop rate, sync, reliability                   | Monitoring via interception → enforcement via contracts |

**The manifest fills the gap between launch files (which declare nodes) and
runtime (which creates topics).** RCL interception provides the observation
layer. Together, they enable a complete specify → monitor → enforce pipeline
for ROS 2 communication graphs.
