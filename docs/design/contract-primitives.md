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
| sampling cost at a boundary | a boundary's `trigger: timer` rate | **implemented** — phase 68 W1.b |
| feasibility, residual | budget − critical path | implemented for what it sums |
| per-node deadlines | decomposition over the route | phase 58 W3 |
| priorities, classes, reservations | the mapper, from the above | implemented |
| `chains:` / `segments:` | the route | **retired** — phase 68 W4, manifest v0.1.17 |
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

## The one thing that stood in the way

Retiring `segments:` before phase 68 W1.b would have silently weakened checking,
and the amount was measurable. The two rules used different arithmetic:

| form | rule | sums |
|---|---|---|
| chain | `chain-budget` | Σ event-segment latency + Σ **sampling_cost** |
| scope path | `scope-budget` | Σ critical-path node latency + Σ transport |

On `rt_workspace` that gap was 10 ms of 25 — **40%**, and the part no priority
assignment can remove. W1.b closed it: a timer-triggered path is charged
`period + exec`, `CriticalPath` carries `sampling_cost_ms`, and the
`scope-budget` diagnostic reports the split the way `chain-budget` did. Both
forms of that fixture then agreed at 25.00 ms, the value predicted in advance.

W4 closed the second half. `chain-sampling-feasibility` — sampling cost ALONE
meeting the budget, which is structural infeasibility rather than a schedulable
overrun — became `scope-sampling-feasibility`, so retiring the chain rules
dropped no claim. `chains:`/`segments:` are now a parse error naming the scope
path that replaces them.

**What migration proved, and how.** Both spellings of `rt_workspace` and of
`rt_av_demo` produced identical schedules — but the check that mattered was
PROVENANCE, not the numbers. `rt_workspace` alone yields the same two
priorities under budget ranking with no chain at all (`non-chain
budget_ms=5`), so a numeric comparison passes while the derivation is not being
used. `rt_av_demo`, with three members in a real drain order across two
platform files, is what makes the equivalence mean something.

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
