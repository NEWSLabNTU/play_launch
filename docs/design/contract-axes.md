# What a contract must carry: the axis survey

**Status: RULED. §0 carries the decisions. Two axes are deferred by choice
(§3.3, §3.5); §6 holds what is still open inside the adopted ones.**

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

The purpose is to have the whole surface visible before the vocabulary refactor
freezes a schema. §0 records what has been ruled on since, §5 the phase
arrangement those rulings imply; the rest is the evidence behind them, and the
parts still open.

---

## 0. Rulings

Adoption is deliberately gradual. The survey found more than one phase can
absorb, so each axis carries a status rather than a place in a queue.

| axis | status |
|---|---|
| per-path attribution (§1.1, §1.2) | **adopt** — measured defect |
| sampling cost (§1.3) | **adopt** — measured gap |
| jitter (§3.6, §2) | **adopt as a requirement** — see §3.6 |
| ROS 2 QoS `deadline` / `lease_duration` (§4) | **adopt** — the contract is for ROS 2 |
| deadline-miss handling (§3.2) | **define now, validate by implementing** — see §3.2 |
| fault detection / reaction, FTTI (§3.1) | **eventually — ISO 26262 requires it.** Noted, not dropped |
| operational modes (§3.3) | **defer** — revisit once the above settle |
| cross-chain synchronization and offset (§3.5) | **defer** — with §3.3 |
| executor / callback-group structure (§3.4) | **adopt as path exclusion** — see §3.4 |
| measurement provenance (§3.7) | platform file, not contract |

Two of these carry an obligation beyond a status.

**Deferring modes has a known cost**, recorded here so it is a decision rather
than an oversight: modes index every other requirement, so retrofitting them
later touches every requirement kind added before. The mitigation is cheap and
should be taken now even though the feature is deferred — keep requirements
addressable by a key that a mode dimension can later be added to, and avoid any
schema shape that assumes a requirement has exactly one value. §3.3 records what
that means concretely.

**FTTI is noted, not dropped.** The distinction matters because §3.1 is not a
feature we might want; it is the axis that separates a performance wish from a
safety requirement, and ISO 26262 requires it of anything claiming to be safe.
Deferring it is a sequencing choice, not a judgement that we can do without it.

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

### 3.1 Fault detection and reaction time — EVENTUALLY, NOT DROPPED

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

**Ruling: adopt eventually — ISO 26262 requires it.** Kept on the list rather
than deferred into silence, because this is the axis that decides whether the
contract can claim to describe a safe system at all. Two things make the wait
cheaper:

- §4's `lease_duration` is a fault-detection interval available for free from
  ROS 2, and is being adopted now. It is not FDTI, but it is the same shape and
  the first real detection mechanism the contract will have.
- §3.2's miss detection produces the other half of the evidence.

By the time FTTI is written, detection may already exist and the new vocabulary
is mostly the *reaction* and *safe state* halves.

### 3.2 Deadline-miss handling — DEFINE NOW, VALIDATE BY IMPLEMENTING

We already have weakly-hard (m,K): `DropSpec { max_count: {n, w},
max_consecutive }`. But it counts **dropped messages**, not **missed
deadlines** — a different failure with a different consequence. A message that
arrives late is not a message that never arrived, and a controller tolerates
them differently.

The platform half is already half-built: `deadline_policy` is *"not yet
authorable on a v2 path (so `SCHED_FLAG_DL_OVERRUN` is wired but always off)."*

#### Proposed vocabulary

Two independent things, and conflating them is the mistake to avoid: **how many
misses are tolerable**, and **what to do about one**.

```yaml
paths:
  control:
    trigger: { input: [pose] }
    output: [cmd]
    max_latency: 10ms
    miss:
      tolerate: { n: 2, w: 100 }   # weakly-hard (m,K), same shape as `drop`
      consecutive: 1               # never two in a row
      action: continue             # continue | skip_next | abort
```

`tolerate`/`consecutive` deliberately reuse `DropSpec`'s shape. They are the
same mathematics on a different event, and an author who has learned one should
not have to learn a second spelling. Whether they should literally be the same
type is an implementation question — the events differ, so the fields probably
want distinct names even if the arithmetic is shared.

`action` takes the three from the literature: continue-to-completion, skip-next,
job-kill (abort).

#### What Linux actually gives us, which is the reason to validate by building

The ruling is to define it and see how it goes in implementation. That is the
right order, because the three actions are **not equally realizable** and the
gap is not visible from the vocabulary:

| action | mechanism | status |
|---|---|---|
| `continue` | CBS throttles to the next replenishment | native under `SCHED_DEADLINE`; it is what already happens |
| `skip_next` | needs the application to drop the next release | **requires node cooperation** — no kernel mechanism |
| `abort` | needs the callback to be interruptible mid-execution | **requires node cooperation**; `rclcpp` offers no such hook |

Detection is available on both scheduling paths, by different means:

- Under `SCHED_DEADLINE`, `SCHED_FLAG_DL_OVERRUN` raises `SIGXCPU` on overrun —
  already wired, always off.
- Under `SCHED_FIFO` there is no kernel deadline at all, so a miss can only be
  observed by measuring response time. The interception layer already does
  exactly this (Phase 58 W2 measures response and cost separately, per
  invocation).

So the honest expectation going in: **`continue` is implementable, detection is
implementable on both paths, and the other two actions describe an obligation on
the node rather than something play_launch can enforce.** That is worth writing
down before the implementation rather than discovering it — a vocabulary that
lets an author select `abort` and silently gets `continue` is worse than one
that only offers what it can deliver.

Whether to ship `skip_next`/`abort` as declarations-of-intent (checked,
reported, not enforced) or to omit them until there is a mechanism is the
decision the implementation should settle.

One cross-repo note: nano-ros's `ResolvedTier` **already carries
`deadline_policy: Option<String>`**, so the RTOS side has a home for `action`
that our platform file does not. Align the vocabulary before either side
invents a second spelling — see §5.

### 3.3 Operational modes — DEFER, WITH A MITIGATION

A fail-operational stack has normal / degraded / minimal-risk-maneuver modes,
and **timing requirements differ per mode**. Our `if`/`unless` are launch-time
conditions: static, resolved once, gone before runtime.

**Ruling: defer, revisit once the other features settle.**

This was flagged as the one axis that is a rewrite if deferred, because modes
index every other requirement. Deferring it is a legitimate call — the feature
is large and nothing else is blocked on it — but the cost is real and the
mitigation should be taken now, while the schema is being touched anyway:

- **Do not assume a requirement has exactly one value.** A requirement that is
  a scalar today and a map-keyed-by-mode tomorrow is a breaking change to every
  reader. A requirement that is *already* addressable — reached through a lookup
  that currently has one entry — is not.
- **Do not bake mode-like meaning into `if`/`unless`.** They are launch-time
  conditions, resolved once. Using them to approximate modes would produce
  contracts that look mode-aware and are not, which is harder to migrate than
  contracts with no modes at all.

That is the whole mitigation. It costs nothing now and converts a rewrite into
an addition.

### 3.4 Executor and callback-group structure — ADOPT, AS PATH EXCLUSION

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

#### The objection, and why it does not hold

The natural objection is that "executor" and "callback group" are `rclcpp`
implementation vocabulary, and the contract is shared with nano-ros, which
targets RTOSes with no executor at all.

That is not what nano-ros does. Its scheduling assignment key is **already the
callback group**:

```rust
// nros-orchestration-ir, ResolvedTier
/// `(node_name, callback_group_id)` pairs assigned to this tier, sorted.
pub members: Vec<(String, String)>,
```

A tier lowers to a concrete RTOS task; its members are callback groups, not
nodes. `NodeOverride` exists to reassign *"a node's callback groups to tiers at
deploy time without touching the node package"*, and groups themselves are
declared per node:

```rust
pub struct CallbackGroupDecl {
    pub id: String,      // "ctrl_loop", "telemetry"
    pub r#type: String,  // "MutuallyExclusive" (default) or "Reentrant"
    pub tier: String,
}
```

So the concept is portable and already shared. **What is missing is a shared
source for it.** nano-ros learns a node's groups from its own component
declarations; play_launch learns nothing; the contract carries neither. The two
toolchains therefore schedule the same system from different information, and
nothing makes their answers agree. That is a worse defect than the analysis gap
this section started from.

#### Group membership is derived, not authored

An earlier draft of this document listed "an author must write it" as a burden
increase, and that was wrong. nano-ros **infers** groups from dataflow coupling
and treats explicit declaration as an override:

> every chain becomes one mutually-exclusive group (its stages serialize), and
> every callback outside any chain becomes its own reentrant group (no coupling
> detected → concurrent-safe)

— with `PlanCallbackGroup::inferred` recording which. Coupling there is the
causal structure, which is exactly what `trigger`/`output` already give us.

So a group is a **consequence**, and `contract-primitives.md`'s rule applies
unchanged: the author states a constraint, the tool computes the partition.

#### Three things; one is a contract fact

| | what | where |
|---|---|---|
| 1 | may two paths of a node run concurrently? | **contract** — a property of the source |
| 2 | which nodes share an executor / task? | platform — `--container-mode` on Linux, tier on RTOS |
| 3 | how many threads serve it? | platform — `executor_threads`, tier count |

Only (1) is portable and unobservable from deployment. (2) and (3) are already
platform-file concerns and stay there.

#### Shape

Not a group — an **exclusion relation between paths**. The group is derived from
it, the way a route is derived from `trigger`/`output`:

```yaml
nodes:
  detector:
    paths:
      to_boxes: { trigger: { input: [image] }, output: [boxes] }
      to_masks: { trigger: { input: [image] }, output: [masks] }
      health:   { trigger: { timer: { rate_hz: 1 } }, output: [diag] }
    concurrency:
      exclusive: [[to_boxes, to_masks]]   # share state, serialize
      # `health` unlisted -> may run concurrently
```

A maximal set of mutually-exclusive paths *is* a group. The platform file then
assigns that group to a tier or an executor.

**The default carries more weight than the syntax.** Both realizations already
default to serialize — `rclcpp`'s implicit per-node callback group is
`MutuallyExclusive`, and `CallbackGroupDecl`'s `default_cbg_type` is
`"MutuallyExclusive"`. So `concurrency:` absent means *every path of this node
serializes*: conservative, identical on both targets, and requiring no
inference at all.

That inverts the authoring burden. Nothing is written unless the author claims
**more** concurrency than the safe default — which is exactly the claim a
reviewer should be made to look at.

#### What this buys before any blocking model exists

With (1) in the contract and (2) known from the platform, the checker can decide
whether **summing is valid** and say so when it is not. That needs no
response-time analysis, no blocking arithmetic, and no executor simulation. The
full analysis stays a later option; converting a silent unsound assumption into
a diagnostic is the part that matters now, and it is the same move §1.1 makes
for per-path attribution.

#### Granularity divergence to resolve with it

Our platform file assigns per node; nano-ros assigns per `(node,
callback_group)`. Once groups are derivable from the contract, our platform file
should accept group granularity too — otherwise the shared contract can express
a schedule nano-ros can realize and we cannot.

### 3.5 Cross-chain synchronization and offset — DEFER

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

**Ruling: defer with §3.3.** These constraints name pairs of endpoints across
chains, which is uncomfortably close to the authored routes
`contract-primitives.md` just decided to retire — see §6.5. Settling the derived
route first is the right order: whatever expresses "these two outputs" should be
built on derived routes, not on a revived `segments:`.

### 3.6 Jitter as a requirement — ADOPT

AADL treats latency **jitter** as a first-class analysis output, not a
by-product — control stability depends on variation, not on the maximum. Its
listed contributors are our blind spots almost exactly: sampling latency,
sampling jitter, queuing time, dispatch protocol.

We have a `jitter` field on `EndpointProps`. It is write-only (§2), and its
meaning is ambiguous: on a publisher it reads as a promise, on a subscriber as a
tolerance, and nothing distinguishes them.

**Ruling: make it a requirement**, which means moving it to where budgets live
rather than fixing it in place:

```yaml
paths:
  detect:
    trigger: { input: [image] }
    output: [boxes]
    max_latency: 20ms
    max_jitter: 4ms        # variation across invocations, not a second budget
```

and the same field on a scope path, where it is the end-to-end requirement a
controller actually cares about.

#### The consequence worth deciding before writing it

**A jitter requirement forces a best-case fact.** Latency analysis needs only
upper bounds; jitter is a *spread*, so a route's jitter is

```
route_jitter = route_worst_case − route_best_case
```

and we have no lower bound anywhere in the vocabulary. Adding `max_jitter`
without also admitting a best case gives a requirement nothing can check — a
seventh write-only field, which is the specific failure §2 is about.

Two ways to get the lower bound, and they are not equivalent:

1. **Declare it** — a sibling `min_latency` fact per path. Symmetric with
   `max_latency`, and authorable. Cost: one more number per path, against the
   direction `contract-primitives.md` set.
2. **Measure it** — `play_launch measure` already computes per-invocation
   response and cost distributions and reports p50/p99/max. Recording the
   minimum is a one-line change, and it makes best case a *measured fact* rather
   than an authored guess, which is what Phase 58 W2 argued for cost.

Prefer (2), with (1) available where no measurement exists. Not yet verified
that the measured minimum is stable enough to budget against — that is the first
thing to check when this is implemented.

#### Sampling jitter is a separate term

A timer boundary contributes jitter as well as latency: a message arriving at an
arbitrary point in a period waits anywhere from 0 to T, so the boundary's
contribution to end-to-end jitter is the **whole period**, independent of how
fast the callback is. AADL lists this separately from execution jitter for that
reason.

This lands in the same place as §1.3's sampling cost, and should be implemented
with it — one traversal that knows about boundaries, producing both terms.

### 3.7 Measurement provenance — PLATFORM FILE

CAST-32A's finding is the one that bears directly on `play_launch measure`:
shared-resource contention inflates WCET by **8–12×**, so a measured number
without its interference conditions is not evidence. Our `budget` records a
value and nothing else — no platform, no concurrent load, no sample count, no
coverage.

We proved this on ourselves already: Phase 60 W8's three arms produced
materially different numbers from identical code.

Belongs in the platform file, not the contract — it describes a measurement, not
the code. Scheduled with the mapper work rather than the vocabulary (§5, phase
B2), since nothing in the contract changes.

---

## 4. Available in ROS 2, unused by us — ADOPT

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

**Ruling: adopt.** The contract is built for ROS 2, so a timing contract ROS 2
already enforces should not be something we merely describe. Concretely:

- derive subscriber `deadline` QoS from `min_rate_hz`
- carry `liveliness` as a policy *and* a `lease_duration`, not a bare string
- an explicitly authored QoS value always wins over a derived one, and a
  disagreement between the two is a diagnostic — the same rule `chain-link`
  applies to routes

This also changes what a violated requirement does at runtime: today nothing,
after this a DDS callback. That is the first enforcement the contract has, and
it is why §3.1 gets cheaper to add later.

#### Enforcement is opt-in by the node, and that is measured

`rclcpp` exposes both policies we want as overridable kinds — `QosPolicyKind`
lists `Deadline` and `LivelinessLeaseDuration` — and they are settable from
outside the node as parameters:

```yaml
my_node_name:
  ros__parameters:
    qos_overrides:
      /my/topic/name:
        publisher:
          reliability: reliable
          depth: 100
```

But the gate is on the node's side:

```cpp
/// Default constructor, no overrides allowed.
QosOverridingOptions() = default;
```

A publisher or subscription only accepts overrides if its author passed
`QosOverridingOptions` at construction. Most do not.

So the honest scope: **derive always, apply where accepted, and report where it
cannot be.** A node carrying a rate requirement whose endpoints did not opt into
QoS overriding is a node whose requirement can be checked but never enforced —
and saying so is more useful than either silently emitting parameters that do
nothing or silently declining to emit them.

**Ownership/strength** — the standard fail-operational failover mechanism
(redundant publishers, highest strength wins) — is **not in ROS 2's supported
set**. Redundancy can therefore be a contract-level concept but has no
enforcement path. Worth knowing before designing for it.

---

## 5. Phase arrangement

Three phases, not two. The change from the earlier draft is that **retirement
moved out of phase 1 and behind verification**: nothing that works today is
removed until the thing replacing it has been shown to produce the same answer
on a real system.

That ordering is the repository's own precedent. The Duration migration shipped
every new spelling as an alias with a compat deserializer, and its acceptance
criterion was *"a unit suffix appearing, never a value moving"* — verified by
resolving the same launch under both spellings and diffing the models. Phase 47
only hard-removed `record.json` after a release of deprecation. Same shape here.

### Phase A — additive vocabulary and migration

Nothing is removed and nothing new is consumed. New fields parse and reach the
model; every old spelling keeps working.

- `max_jitter` on paths and scope paths, plus its best-case counterpart (§3.6)
- `miss: { tolerate, consecutive, action }` (§3.2)
- `concurrency.exclusive` between a node's paths (§3.4)
- QoS `deadline` and `lease_duration` as authorable policies (§4)
- the modes mitigation — requirements stay addressable, `if`/`unless` are not
  bent into approximating modes (§3.3)
- migrate the fixtures' own contracts to the new spellings

**Mapper: minimum correctness only.** It must not get worse. The new facts reach
it and are ignored; a model resolved before and after phase A differs in nothing
but the presence of unread fields.

**Acceptance:** every existing contract in the tree resolves to a byte-identical
model, `scripts/compare_models.py` clean. A phase A that changes any derived
value has a defect in it.

### Phase B — analysis and mapper consume, then verify

#### B1 — analysis

- per-path attribution (§1.1, §1.2) — one fix, two symptoms
- sampling cost *and* sampling jitter, per route (§1.3, §3.6)
- derive callback groups from the exclusion relation (§3.4)
- diagnose when summing is invalid for the resolved container mode (§3.4)
- consume the write-only fields (§2)

**Acceptance is by known-in-advance numbers**, because they have been measured:
`points_to_cmd` as a scope path reports **25 ms against a 20 ms budget**,
matching `chain-budget` on the same fixture; the two-output fixture charges
`to_tracks` **30 ms, not 45 ms**, and stops tracking an unrelated sibling path's
latency.

#### B2 — mapper

Each adopted axis has a mapper job, and they are not symmetric — two are
derivations, two are refusals:

| axis | what the mapper must do |
|---|---|
| path exclusion (§3.4) | treat an exclusive group as **one** schedulable entity, not N nodes |
| path exclusion (§3.4) | **refuse** a per-thread reservation for a node whose exclusive paths span threads — CLAUDE.md's F2 already records that a reservation is per-thread, so deriving one per node here is unsound |
| `miss.action` (§3.2) | derive `deadline_policy`; turn on `SCHED_FLAG_DL_OVERRUN`, wired since phase 60 and always off |
| `max_jitter` (§3.6) | a declared jitter bound argues against best-effort placement — promote, or **report why it cannot** |
| per-path cost (§1.2) | reservation runtime from the traversed path's cost, not the node's sum |
| QoS (§4) | emit `qos_overrides.*` parameters where accepted; report endpoints that did not opt in |

**Acceptance:** `check --sched --explain` names the provenance of every changed
decision, and a node whose placement changed says which new fact changed it.

#### B3 — verify on play_launch, on real systems

This is the wave that gates retirement. A checker agreeing with itself is not
evidence; the vocabulary has to survive a running system.

| fixture | what it proves | gate |
|---|---|---|
| `tests/fixtures/rt_workspace` | the vocabulary end-to-end through `resolve` → `check` → `up` | contract parity, both parsers |
| `examples/rt_av_demo` (`just ab`) | the derivation still produces a working schedule | **reproduces the published 217 → 9 missed-frame result**; `just ab3` for the deadline arm |
| Autoware (144 nodes, 17 containers) | no regression at scale, and the `observable` case actually exercises §3.4 | resolves; the summing-validity diagnostic fires where co-location is real |
| golf cart / AutoSDV | the launch that motivated phase 61 and 64 | resolves and launches |

`rt_av_demo` is the load-bearing one. Its numbers are published in
`docs/reports/rt-mixed-criticality/` and were produced by measurement, so a
derivation change that quietly degrades scheduling shows up as frames rather
than as a passing test. Run it jailed in its own `ROS_DOMAIN_ID`.

### Phase C — retire, gated on B3

Only after B3 is green:

- `chains:` / `segments:` — the derived route replaces them
  (`contract-primitives.md`)
- `topics.<t>.rate_hz` — propagates from the timer that starts the chain
- `EndpointProps.jitter` — superseded by the path/scope requirement (§3.6)

Each goes deprecated-with-a-lint first, naming the fact it is derivable from,
then removed a release later.

**Retirement gate — all four, not a majority:**

1. every fixture in the tree migrated to the new spellings
2. old and new spellings resolve to identical models (`compare_models.py`)
3. `rt_av_demo`'s A/B reproduces its published numbers under the new derivation
4. one release shipped with the deprecation lint live

Condition 2 is the one that makes the rest safe: while both spellings produce
the same model, retirement is provably lossless, and if they ever diverge the
divergence is a defect in the new path rather than a reason to keep the old one.

### Cross-repo consequences

Two items reach beyond this repository and should be raised with nano-ros
rather than decided here:

- **Platform-file granularity** (§3.4). Ours assigns per node, nano-ros per
  `(node, callback_group)`. The shared contract will soon express groups; the
  platform files should agree on whether they can name one.
- **`deadline_policy` already exists on the RTOS side.** `ResolvedTier` carries
  `deadline_policy: Option<String>`, so nano-ros has a home for §3.2's `action`
  that our platform file does not. Worth aligning the vocabulary before either
  side invents a second spelling.

## 6. Open questions

Each changes what phase 1 writes. Numbering is stable — §3.5 and §5 refer to
these by number.

1. **Best case: measured or declared?** (§3.6) `max_jitter` is unfalsifiable
   without a lower bound. `measure` can supply one for a one-line change;
   whether the measured minimum is stable enough to budget against is unknown
   and is the first thing to check.
2. **Do `miss:` and `drop:` share a type?** (§3.2) Same arithmetic, different
   events. Sharing the shape helps an author; sharing the type may force one
   event's fields onto the other.
3. ~~Where does callback-group membership come from?~~ **Answered** (§3.4):
   nano-ros infers groups from causal coupling and treats declaration as an
   override, so a group is a consequence, not a fact. What remains is narrower —
   is "same node ⇒ exclusive by default" the right rule for us, or does path
   exclusion within a node ever want a finer derivation? Current answer: no
   inference, the conservative default plus an explicit opt-out.
4. **Does per-path cost need a `costs:` section**, or can `measure` attribute
   per (node, path) once path identity is on the edge? Prefer the second; not
   yet verified possible.
5. **Can a route-level requirement be expressed without reviving `segments:`?**
   (§3.5) A synchronization constraint names two chain endpoints. If derived
   routes can be named stably, this works; if they cannot, the deferred axes
   stay blocked.
6. **Is FTTI a contract requirement or a hazard-level one?** (§3.1)
   `criticality-from-hazards.md` already argues criticality should be derived
   from declared hazards rather than a `high|medium|low` label. FDTI/FRTI are
   hazard-derived in ISO 26262 too. These may be one design, and if so §3.1
   should land with that work rather than on its own.
7. **Do modes belong in the contract at all** (§3.3), or in a separate artifact
   the contract is indexed by? A mode set is a system property, not a node
   property. Deferred, but the answer shapes the mitigation.

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
