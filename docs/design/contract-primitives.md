# Contracts carry facts and requirements, never consequences

**Status:** design, decided. Supersedes the authored-route parts of Vocabulary
v2 (`chains:` / `segments:`). Every number below was measured against the tree
at `9d1af73`, mostly on `tests/fixtures/rt_workspace`.

## The rule

> **A contract states what the code does and what it must achieve. Anything
> computable from those two is derived, never written.**

A trigger is a fact. A budget is a requirement. A route, a total, a downstream
rate, a per-node deadline — consequences, all of them.

The point is not elegance. Every consequence written by hand is a second copy
of something the tool already knows, and two copies can disagree. `chain-link`
exists to catch exactly that disagreement for routes; `topics.rate_hz` has the
same problem and nothing catches it at all.

## The three levels an author writes

Node, topic, publisher/subscriber, service and action are ROS concepts an author
already knows. `scope` is this project's addition, and is where requirements
spanning several nodes are annotated.

| level | carries | kind |
|---|---|---|
| **node** | `paths.<n>.trigger`, `paths.<n>.output`, `pub:`/`sub:`, `msg_type` | fact |
| **node** | `paths.<n>.max_latency`, endpoint `min_rate_hz` / `max_age` | requirement |
| **topic** | `qos`, `drop` | requirement |
| **scope** | `paths.<n>.input` / `output`, `max_latency`, `semantics` | requirement |
| platform file | `budget` (cost) | measured fact |

## What is a fact, and why it cannot be derived

The irreducible core is the **causal structure of the code**:

- `trigger` — what *causes* this callback. A launch file says a node exists; it
  cannot say "this subscription causes that publication". Only the source knows.
- `output` — which topics *this* callback publishes. A node with three
  callbacks publishing three topics is indistinguishable from outside.
- `pub:` / `sub:` — which topics the node touches at all. Observable at runtime
  by the interception layer, but not derivable from the launch file.

Everything else in a contract is either a requirement someone chose, or a
consequence of these plus the launch tree.

## What is derived

| derived | from | status |
|---|---|---|
| the route between two endpoints | `output` × `trigger.input` + topic pub/sub | **implemented** — `check_scope_path_critical_path` |
| critical-path total | that route + node latencies + transport | **implemented**, fork-join aware |
| sampling cost at a boundary | a boundary's `trigger: timer` rate | **missing** — the one gap |
| feasibility, residual | budget − critical path | implemented for what it sums |
| per-node deadlines | decomposition over the route | phase 58 W3 |
| priorities, classes, reservations | the mapper, from the above | implemented |
| `chains:` / `segments:` | the route | to retire |
| `topics.<t>.rate_hz` | propagation from the timer that starts the chain | to retire |

**The derived route already exists.** `check_scope_path_critical_path`
(`manifest_loader.rs`) builds the global dataflow graph, takes the subgraph
between a scope path's input and output topics *within that scope's subtree*,
computes the critical path and reports it by name:

```
warning[scope-budget]: scope path 'points_to_cmd' max_latency_ms (12) is less
than critical path: /perception/filter_component → /control/control_node = 15ms
```

Arbitrary endpoints, across a subtree, correct for fork-join. This design does
not invent derivation; it removes the hand-written duplicate of something
already derived.

## The measured redundancy

In `rt_workspace`'s contract — three nodes, ~95 lines — the number **100
appears nine times**:

| copies | what | kind |
|---|---|---|
| 1 | `trigger: { timer: { rate_hz: 100 } }` | the fact |
| 3 | `topics.*.rate_hz` | derivable: rate propagates along the causal graph |
| 5 | `min_rate_hz` on every endpoint | requirements, all identical, all implied |

Plus five lines of `segments:` restating a route the graph defines.

The irreducible content of that file is three `trigger`/`output` pairs, two node
budgets, one `max_age`, one end-to-end budget — roughly **a third** of what is
written.

## The one thing standing in the way

Retiring `segments:` today would silently weaken checking, and the amount is
measurable. The two rules use different arithmetic:

| form | rule | sums |
|---|---|---|
| chain | `chain-budget` | Σ event-segment latency + Σ **sampling_cost** |
| scope path | `scope-budget` | Σ critical-path node latency + Σ transport |

Same fixture, 20 ms budget:

```
chain form   warning[chain-budget]: total (25.00ms = 15.00ms event-segment
             + 10.00ms sampling_cost) exceeds the chain budget (20ms)
scope path   clean — 0 budget diagnostics
```

That 10 ms is 40% of the real total, and it is the part no priority assignment
can remove — the reason `chain-sampling-feasibility` exists.

**Fixed in phase 68 W1.b.** An earlier version of this paragraph blamed the
subgraph for "starting at the input topic", leaving the boundary outside it.
That was wrong — sources are seeded from the input topic's publishers as well as
its subscribers, so the boundary was always in the subgraph. The real causes
were that a timer path was charged only its declared `exec` (zero here), and
that a source was treated as having nothing upstream, which truncated the route
at the subscriber. See `contract-axes.md` §1.3.

Neither rule is a superset of the other: `scope-budget` counts declared topic
transport, `chain-budget` does not.

## The design

### One formula

For each route between a scope path's endpoints:

```
route_total = Σ node latency + Σ topic transport + Σ sampling cost
scope_total = max over routes
```

`sampling cost` is a boundary's `period + exec`, the formula `chain_checks`
already uses. Both new terms enter *per route* and are maximised, not summed —
with fork-join, two branches may have different boundaries, and summing across
branches is the same error as summing node latencies across branches.

### Where the boundary comes from

A scope path's input topic is published by some path in the subtree. If that
path is `trigger: timer`, it is a boundary and its sampling cost belongs to the
route. The facts needed are already present: the trigger gives the period, and
phase 58 W3.a made the cost reachable at `MapperPath::exec_ms`.

### What the checker emits

The checker stops being only a grader. Having derived the routes and their
totals, it emits them as the **byproduct the mapper consumes** — today the
mapper re-derives chains from authored segments; under this design it reads
what the checker already computed. One derivation, one place, two consumers.

### `semantics` moves

`reaction` vs `age` exists only on `ChainDecl`, and the two diverge exactly at
the junctions this work is about. It moves to `PathDecl` so a scope path can say
which of the two its budget means. This is the only field a scope path cannot
express today.

## Migration, in order

Scheduled as [phase 67](../roadmap/phase-67-contract-primitives.md) and
[phase 68](../roadmap/phase-68-contract-consequences.md); the steps below are
the argument, those docs are the work.


The obvious ordering is wrong. Do not drop `segments` first.

1. **Write the acceptance test.** Express `points_to_cmd` as a scope path; assert
   `scope-budget` reports **25 ms against a 20 ms budget**, matching
   `chain-budget` on the same fixture. Expected value known in advance.
2. **Add the sampling-cost term** to `check_scope_path_critical_path`, per route.
3. **Reconcile transport** — one formula, both terms.
4. **Move `semantics`** to `PathDecl`, keeping the chain field as an alias.
5. **Emit the derived routes** for the mapper; switch the mapper to read them.
6. **Deprecate `segments:`**, then `topics.rate_hz`, with a lint naming the fact
   each is derivable from.
7. **Retire** after one release.

Steps 1–2 make the migration *provably* lossless. Everything after is
subtraction.

## Completeness is a separate question

This document says how to tell a fact from a consequence. It does not say
whether the set of facts and requirements we carry is complete — that survey,
against AADL, AUTOSAR TIMEX, ISO 26262, the weakly-hard literature, CAST-32A and
the published ROS 2 executor analysis, is `contract-axes.md`. It is OPEN where
this one is decided, and it holds the measured arithmetic defects (per-path
attribution) alongside the missing axes (modes, fault detection, executor
structure).

## What this does not claim

- **`min_rate_hz` is not obviously redundant.** Five identical copies look like
  restatement, but a per-endpoint rate *requirement* is a real thing to want
  independently. The claim here is only that the default should follow the
  graph, not that the field should go.
- **`max_transport` placement is unresolved.** It is deployment-dependent —
  `--container-mode isolated` alone changes it — which argues for the platform
  file, while its authoring reads naturally per topic. Phase 58 W3's own note
  says it moves when something first consumes it in budget arithmetic, which is
  step 3 above.
- **No performance claim.** This is about what an author writes and what a
  checker can prove, not about runtime behaviour.
