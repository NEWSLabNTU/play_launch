---
marp: true
theme: default
paginate: true
---

# Launch Manifest
## Communication Contracts for ROS 2 Launch Files

---

# The Problem

ROS 2 launch files declare **what to run** — but not what communicates.

- Topics are created in source code — invisible until runtime
- No reference specification for the expected communication graph
- A code change can silently break timing or remove a topic

**Example:** Autoware planning_simulator — 110 nodes, 500+ topics.
Who checks that the perception pipeline meets its 100ms latency budget?

---

# What is a Launch Manifest?

A sidecar YAML file that describes what one launch file **contributes
to the communication graph**.

| Launch file says | Manifest adds |
|-----------------|--------------|
| Which nodes to run | Named endpoints (pub/sub/srv/cli) |
| How to remap names | First-class topic wiring with type + QoS |
| Which files to include | Child scopes with their own manifests |
| *(nothing)* | Timing contracts: latency, age, drops |

One manifest per launch file. The tree of manifests mirrors the tree
of launch file includes.

---

# From Launch Files to Manifests

![w:900](img/manifest-mapping.png)

---

# Nodes and Endpoints

Nodes declare named **endpoints** — logical ports for communication.

```yaml
nodes:
  controller:
    pub:
      cmd:
        min_rate_hz: 30           # I produce at least 30 Hz
    sub:
      trajectory:
        min_rate_hz: 10           # I need at least 10 Hz
      map:
        state: true               # polled, not causal
        required: true            # must receive at least once
```

- `state: true` — read-latest (breaks feedback cycles in the graph)
- `required: true` — node can't operate without initial data

---

# Topics: First-Class Wiring

Topics wire endpoints together with type, QoS, rate, and drop tolerance.

```yaml
topics:
  control_cmd:
    type: autoware_control_msgs/msg/Control
    pub: [controller/cmd]
    sub: [validator/input]
    rate_hz: 30
    max_drop_rate: 0.01           # 1% transport loss allowed
    max_consecutive: 3            # never 3+ drops in a row
    qos:
      reliability: reliable
      durability: transient_local
```

In launch files, this information is split across `<remap>`, source
code, and convention. The manifest makes it **explicit and checkable**.

---

# Rate Hierarchy with Drops

The publisher produces, transport may drop, subscriber demands:

```
pub.min_rate_hz  >=  rate_hz  >=  rate_hz × (1 - max_drop_rate)  >=  sub.min_rate_hz
     30                30          30 × 0.99 = 29.7                      29
```

**supply ≥ channel ≥ effective delivery ≥ demand**

Drop rates compose multiplicatively through a pipeline:

$$\mathcal{R}_{\text{chain}} = \prod_i (1 - d_i)$$

Three topics at 2% each: $0.98^3 = 0.941$ → 5.9% E2E drop rate.

---

# Timing Contracts: Latency and Age

**`max_latency_ms`** — processing time (trigger input → output publish)

```
sensor → [cropbox: 5ms] → [ground_filter: 15ms] → [detector: 30ms]
         ├── 5ms ──┤      ├──── 15ms ────┤         ├── 30ms ──┤
         └──────────── scope: 50ms ─────────────────────────────┘
```

- **Node**: `rcl_take` → `rcl_publish` (processing only)
- **Scope**: first take → last publish (includes internal transport)

**`max_age_ms`** — data freshness from original source

$$\text{age} = \text{now} - \text{header.stamp}$$

Causal paths preserve `header.stamp`. Periodic nodes reset the chain.

---

# Scope Interface and Composition

Each scope declares its boundary ports:

```yaml
sub:                                    # what flows IN
  trajectory: [controller/trajectory]
pub:                                    # what flows OUT
  control_output: [controller/cmd]
srv:                                    # services exposed
  operate: [operator/operate]
```

Parent wires children: `child_name/group_name` in topics and services.

**Opaque scope** (has budget) → parent trusts the declared value
**Transparent scope** (no budget) → parent looks through to children

---

# Verification Rules

**14 static checks** at authoring time, before any code runs:

| Rule | What it catches | Severity |
|------|----------------|----------|
| `rate-hierarchy` | pub rate < topic rate < sub rate | Error |
| `budget-overflow` | Node 20ms inside scope 10ms (part > whole) | Error |
| `scope-budget` | Sum of children > scope budget | Warning |
| `drop-rate` | Scope drop < composed topics; effective rate < sub demand | Error |
| `causal-dag` | Feedback cycle (state: true breaks it) | Error |
| `satisfiability` | Arg combo produces dangling entities (Z3) | Error |
| `dangling-entity` | Topic with 0 publishers after filtering | Warning |

Plus: `endpoint-unique`, `wiring`, `qos-compat`, `rate-chain`,
`drop-consecutive`, `service-wiring`, `service-type`

---

# Partial Decomposition

You don't need contracts on every node. Start top-down:

```
scope S: max_latency_ms: 100
  ├── sub-scope P: max_latency_ms: 50    (opaque — black box)
  │     ├── node A: max_latency_ms: 20
  │     └── node B: max_latency_ms: 25
  ├── sub-scope Q: (no budget)            (transparent — look through)
  │     └── node C: max_latency_ms: 30
  └── node E: (no budget)                 → 20ms residual
```

- P checks: 20 + 25 = 45 ≤ 50 ✓ (5ms transport headroom)
- S checks: P(50) + C(30) = 80 ≤ 100 ✓ (20ms residual for E)

Fill in per-node budgets as you measure them.

---

# Composition Summary

| Topology | Latency | Rate | Drop | Age |
|----------|---------|------|------|-----|
| **Series** | sum | preserved | multiply $\mathcal{R}$ | sum along chain |
| **Parallel** | max + fusion | min | user-declared | max + fusion |
| **Periodic** | +P+J (wait) | 1000/P | resets consecutive | resets stamp chain |

Drop composition:
- **`max_drop_rate`** on topics (transport) and scope paths (E2E)
- **`max_consecutive`** on topics and scope paths — never N+ in a row
- Node paths have **latency only** — drops modeled where they occur

---

# Conditional Configurations (Z3)

Args with `type: bool` or `choices:` enable exhaustive checking:

```yaml
args:
  launch_validator:
    type: bool                    # 2 valid values
  pose_source:
    choices: [ndt, eagleye, gnss] # 3 valid values
```

The checker uses **Z3 SMT solver** to verify: no valid arg combination
produces a structurally broken manifest (0 publishers, 0 servers).

Example: `pose_source: gnss` but no `gnss_node` declared →
*"topic 'pose' has 0 publishers when pose_source=gnss"*

---

# Status

**Manifest crate**: 186 tests, 14 validation rules, Z3 satisfiability
**Autoware contracts**: 68 manifests covering the full planning_simulator tree

| Category | Count |
|----------|-------|
| Leaf manifests (with nodes) | 36 |
| Intermediate manifests (orchestration) | 32 |
| Validation rules | 14 |
| Design docs | launch-manifest.md + contract-theory.md |

Open items: `budget-overflow` implementation, runtime monitoring
integration, per-rule CLI filter (`--rule`)

---

# Links

- **Manifest spec**: `src/ros-launch-manifest/docs/launch-manifest.md`
- **Contract theory**: `src/ros-launch-manifest/docs/contract-theory.md`
- **Autoware contracts**: `~/repos/autoware-contract/`
- **Design issues**: `src/ros-launch-manifest/docs/design-issues.md`
