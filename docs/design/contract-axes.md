# What a contract must carry: the axis survey

**Status: OPEN. A working document, not a decision.**

`contract-primitives.md` decided one thing — *a contract states what the code
does and what it must achieve; anything computable from those is derived.* That
rule says how to tell a fact from a consequence. It does not say whether the set
of facts and requirements we have is **complete**.

This document is the completeness pass. It collects three kinds of finding:

1. **Wrong arithmetic** — a fact is declared, read, and reduced incorrectly.
2. **Write-only fields** — a fact is declared and no rule ever reads it.
3. **Missing axes** — a concept a safe system needs that we cannot express.

Every number below was measured against the tree at `c47ca2a`. The missing-axis
section is a survey of AADL, AUTOSAR TIMEX, ISO 26262, the weakly-hard
literature, CAST-32A, published ROS 2 executor analysis, AMALTHEA and MARTE;
sources are listed at the end.

Nothing here is settled. The purpose is to have the whole surface visible before
the vocabulary refactor freezes a schema.

---

## 1. Wrong arithmetic

### 1.1 Latency is charged per node, not per path

A node with two causal outputs is the shape that exposes it. One input image,
two distinct extractions, different costs:

```
/image → detector ─(boxes, 20ms)→ tracker   (10ms) → /tracks
                  └(masks, 35ms)→ segmenter ( 5ms) → /seg
```

Both outputs are representable, and the graph derives correctly — every
subscriber edge `causal: true`, both scope paths `cross_node: true`, no cycles.
The checker finds both routes and names them.

The reduction is wrong. `GlobalNode::max_latency_ms` takes the maximum over
**all** of a node's paths, regardless of which one produced the topic the route
traverses:

```rust
pub fn max_latency_ms(&self) -> f64 {
    self.paths.values()
        .filter_map(|p| p.max_latency.map(|d| d.as_millis_f64()))
        .fold(0.0_f64, f64::max)   // max over ALL paths in the node
}
```

Measured — vary `to_masks` alone and watch `to_tracks`, a route that never
touches `masks`:

| `to_masks` | `to_tracks` charged | true (20+10) |
|---|---|---|
| 35 ms | **45 ms** | 30 ms |
| 100 ms | **110 ms** | 30 ms |
| 12 ms | silent (= 30 ms) | 30 ms |

One-for-one above the max. At 12 ms it falls silent because `max(20,12) = 20`
and the route total finally equals the declared 30 ms.

Overestimating is the safe direction, so this is not a hazard. It is a
**silent** wrong answer — no diagnostic says a route was charged for an output
it never used — and it makes a per-route budget ungradeable for exactly the
nodes that most need one.

### 1.2 Cost is refused when a node has several paths

```rust
let node_exec_ms = if path_count == 1 {
    budget_us_for(budgets, &record.fqn).map(...)
} else {
    None
};
```

The comment is correct about why: *"A declared budget is per NODE, and
`MapperPath::exec_ms` is per PATH, so the two only line up when the node has
exactly one path… Absent is the honest answer."* `chain-sampling-feasibility`
then reports "feasible ON INCOMPLETE EVIDENCE" rather than lying.

Honest, and still a hole: the multi-output node — the common shape for a
perception front-end — is precisely the one whose cost never reaches the mapper.
The code already names the fix: *"Per-(node, path) cost is the open question a
`costs:` section would answer."*

**1.1 and 1.2 share one root cause: the graph is node-keyed, the facts are
path-keyed.** Carrying path identity onto the edge fixes both — the DP charges
the path that produced the traversed topic, and `measure` can attribute per
(node, path).

### 1.3 Sampling cost is missing from the scope-path rule

Carried over from `contract-primitives.md`. `scope-budget`'s subgraph starts
*at* the input topic, so a timer boundary upstream is outside the traced region
by construction. On the `rt_workspace` fixture that is 10 ms of 25 ms — 40%, and
the part no priority assignment can remove:

```
chain form   warning[chain-budget]: total (25.00ms = 15.00ms event-segment
             + 10.00ms sampling_cost) exceeds the chain budget (20ms)
scope path   clean — 0 budget diagnostics
```

---

## 2. Write-only fields

Verified by reading every non-test read site. These are declared, serialized
into the model, and never reach a verdict:

| field | read sites | what they are |
|---|---|---|
| `jitter` | 3 | `model_builder` copy, merge equality, deprecation lint |
| `lifespan` | 3 | same |
| `max_response` | 3 | same |
| sync window (`tolerance`, `max_interval`, `timeout`) | — | declared; no budget rule consumes them |

Six declared facts that no arithmetic reads, counting §1.3 and §1.2.

This matters for sequencing more than for correctness. Adding axes while six
existing fields are decorative makes the ratio worse, and the ratio is what an
author actually experiences: a field you can write and nothing checks is
indistinguishable from a comment.

---

## 3. Missing axes

Ranked by safety consequence.

### 3.1 Fault detection and reaction time

The central timing concept of ISO 26262, and we have no vocabulary for it:

```
FTTI = FDTI (detect) + FRTI (react) + reach safe state
```

We can say *"this data must be ≤50 ms old at use"* (`max_age`) and *"this topic
must arrive at ≥100 Hz"* (`min_rate_hz`). We cannot say **what happens when it
isn't, how fast that must be noticed, or how long until a safe state.**

That is the whole difference between a performance wish and a safety
requirement, and today every rate requirement we carry is the former.

Temporal freedom-from-interference is defined in exactly these terms — a fault
in one element preventing another from executing within its FTTI. Our
criticality→priority mapping is a *mitigation* for it, with no declared
obligation it can be checked against.

### 3.2 Deadline-miss handling

We already have weakly-hard (m,K): `DropSpec { max_count: {n, w},
max_consecutive }`. But it counts **dropped messages**, not **missed
deadlines** — a different failure with a different consequence.

And there is no handling policy. The literature's three are job-kill (discard at
the deadline), skip-next, and continue-to-completion; the choice changes the
stability analysis, so it cannot be left implicit.

The platform half is already half-built: `deadline_policy` is *"not yet
authorable on a v2 path (so `SCHED_FLAG_DL_OVERRUN` is wired but always off)."*
The contract half was never designed.

### 3.3 Operational modes

A fail-operational stack has normal / degraded / minimal-risk-maneuver modes,
and **timing requirements differ per mode**. Our `if`/`unless` are launch-time
conditions: static, resolved once, gone before runtime.

**This is the finding that must be settled first**, not because it is the most
urgent but because it is the only one that is a rewrite if deferred. Modes index
every other requirement. Leaving room is cheap; retrofitting is not. An explicit
"not now, and here is the shape that keeps it possible" is a valid answer.

### 3.4 Executor and callback-group structure

Our critical path sums declared per-node latencies. Nothing checks that nodes
sharing an executor can all achieve theirs. The published analysis is blunt:
with mutually-exclusive callback groups, the default ROS 2 executor yields
**unbounded** worst-case response time for some chains.

Partial awareness exists in the wrong layer.
`chain_container_colocation_warnings` (`execution/sched_plan.rs`) warns when
chain members share a container, and correctly exempts `isolated`, where
fork+exec makes co-location structurally impossible. But it is a launch-time
warning in `play_launch`, not an analysis input, and it models no blocking.

The sharper problem: **container mode decides whether our analysis is valid at
all.** Under `isolated`, nodes are separate processes and summing holds. Under
`observable`/`stock` they share an executor and it does not. The checker never
knows which it will run under.

Authoring split, if this lands: callback-group membership is a **fact** (a
source-level decision, belongs in the contract); executor thread count and
container mode are **platform**.

### 3.5 Cross-chain synchronization and offset

AUTOSAR TIMEX carries both; we carry neither.

- **Synchronization** — "these two outputs must land within X of each other."
  We have fan-in sync at a single node (`correlation`, `tolerance`,
  `SyncPolicy`). We cannot relate two separate chains: left/right camera, or
  perception and localization arriving at planning.
- **Offset** — the phase between two events. Two 100 Hz timers colliding every
  cycle versus offset by 5 ms is a real scheduling input we cannot express.

TIMEX's full set is the useful checklist: Age, Latency, Synchronization, Offset,
ExecutionOrder, ExecutionTime, EventTriggering. We have solid Age and Latency,
partial ExecutionTime, nothing else.

### 3.6 Jitter as a requirement

AADL treats latency **jitter** as a first-class analysis output, not a
by-product — control stability depends on variation, not on the maximum. Its
listed contributors are our blind spots almost exactly: sampling latency,
sampling jitter, queuing time, dispatch protocol.

We have a `jitter` field (§2). It is write-only.

### 3.7 Measurement provenance

CAST-32A's finding is the one that bears directly on `play_launch measure`:
shared-resource contention inflates WCET by **8–12×**, so a measured number
without its interference conditions is not evidence. Our `budget` records a
value and nothing else — no platform, no concurrent load, no sample count, no
coverage.

We proved this on ourselves already: Phase 60 W8's three arms produced
materially different numbers from identical code.

Belongs in the platform file, not the contract — it describes a measurement, not
the code.

---

## 4. Available in ROS 2, unused by us

ROS 2 supports eight QoS policies. We model six. The two missing are precisely
the **runtime-enforced timing contracts**:

| policy | ours | what it would give |
|---|---|---|
| `deadline` | absent | maximum time between messages, with `on_requested_deadline_missed` |
| `lease_duration` | absent | fault-detection interval (§3.1, partially, for free) |
| `liveliness` | policy kind only, as a bare `String` | — |

`min_rate_hz` on a subscriber **is** deadline QoS. Deriving one from the other
is exactly what `contract-primitives.md`'s rule demands — author the requirement
once, derive the enforcement — and it converts a static wish into runtime
detection with a callback.

Best safety-per-effort ratio on this document.

**Ownership/strength** — the standard fail-operational failover mechanism
(redundant publishers, highest strength wins) — is **not in ROS 2's supported
set**. Redundancy can therefore be a contract-level concept but has no
enforcement path. Worth knowing before designing for it.

---

## 5. Provisional split across the two phases

The planned sequencing is: refactor the contract to primitives and migrate, get
the mapper to minimum correctness, then make analysis and mapper correct and
verify. That order is right. Phase 1 is a schema freeze, so it must **decide**
anything that changes the shape — including deciding to defer, explicitly.

**Phase 1 — vocabulary:**

- modes (§3.3) — indexes everything else
- fault detection / reaction (§3.1) — new requirement kind
- deadline-miss policy (§3.2) — new requirement kind
- callback-group membership (§3.4) — a fact, belongs beside `trigger`/`output`
- synchronization and offset constraints (§3.5) — new requirement kinds
- retiring `chains:`/`segments:` and `topics.rate_hz` (`contract-primitives.md`)

**Phase 2 — analysis and mapper:**

- per-path attribution (§1.1, §1.2) — one fix, two symptoms
- sampling cost in the scope-path rule (§1.3)
- consume the write-only fields (§2)
- executor blocking model (§3.4)
- derive QoS `deadline`/`lease_duration` (§4)
- measurement provenance in the platform file (§3.7)

### The hazard in this order

§3.4 is the one that can break the plan. Today's analysis is sound only because
`isolated` is the default. If phase 1 ships a vocabulary that cannot express
executor structure, phase 2 must add a fact — which is another contract
migration, the exact thing phase 1 existed to avoid.

---

## 6. Open questions

Not yet answered, and each changes what phase 1 writes:

1. **Do modes belong in the contract at all**, or are they a separate artifact
   the contract is indexed by? A mode set is a system property, not a node
   property, and putting it inside per-node manifests may be the wrong home.
2. **Is FTTI a contract requirement or a hazard-level one?**
   `criticality-from-hazards.md` already argues criticality should be derived
   from declared hazards rather than labelled `high|medium|low`. FDTI/FRTI are
   hazard-derived in ISO 26262 too. These two may be one design.
3. **Where does callback-group membership come from?** It is a source-level
   fact, but nothing in a launch file reveals it, and the interception layer
   sees threads rather than groups. If it cannot be observed, an author must
   write it — which is a real burden increase, against the direction
   `contract-primitives.md` set.
4. **Does per-path cost need a `costs:` section**, or can `measure` attribute
   per (node, path) once path identity is on the edge? Prefer the second; not
   yet verified possible.
5. **Is a route-level requirement expressible without reintroducing
   `segments:`?** §3.5's synchronization constraint names two chain endpoints,
   which is uncomfortably close to the authored routes we just decided to
   retire.

---

## Sources

- [AADL flow latency analysis (CMU/SEI)](https://kilthub.cmu.edu/articles/journal_contribution/Flow_Latency_Analysis_with_the_Architecture_Analysis_and_Design_Language_AADL_/6573863/1)
- [AUTOSAR Specification of Timing Extensions, CP R22-11](https://www.autosar.org/fileadmin/standards/R22-11/CP/AUTOSAR_TPS_TimingExtensions.pdf)
- [ISO 26262 freedom from interference — spatial, temporal, logical](https://piembsystech.com/freedom-from-interference-iso-26262/)
- [Weakly Hard Real-Time Model for Control Systems: A Survey](https://www.mdpi.com/1424-8220/23/10/4652)
- [Timing Analysis and Priority-driven Enhancements of ROS 2 Multi-threaded Executors](https://arxiv.org/html/2408.08440v1)
- [Response-Time Analysis of ROS 2 Processing Chains (Casini et al., ECRTS'19)](https://pure.mpg.de/rest/items/item_3215453/component/file_3339293/content)
- [ROS 2 design: QoS deadline, liveliness, lifespan](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html)
- [Understanding CAST-32A and AMC 20-193](https://ldra.com/amc20-193/)
- [A Formally Verified Fail-Operational Safety Concept for Automated Driving](https://arxiv.org/pdf/2011.00892)
- [Eclipse APP4MC / AMALTHEA](https://eclipse.dev/app4mc/help/latest/index.html)
- [OMG MARTE](https://www.omg.org/omgmarte/)
