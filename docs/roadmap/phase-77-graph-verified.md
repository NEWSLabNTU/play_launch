# Phase 77 — the inferred graph, measured

Status: **complete** (2026-09-09). Follows phase 76, whose Limits section
named this as the obvious next step.

## Why

Phase 76 filled the topic graph from the launch file's own remaps and reported
110 topics and 114 edges on Autoware where there had been none. Every one of
those edges is an INFERENCE from the `~/input/`–`~/output/` naming convention.
Nothing had checked a single one against a running system.

That matters in one direction more than the other. A missing edge leaves the
graph sparse, which is the state phase 76 found and fails open. A **wrong
direction invents causality**, and every rule that walks the graph — the
ancestor test R6 wants, `causal-dag-global`, the fault-reaction route, the
critical path — inherits it and reports a verdict computed over a system that
does not exist.

## What it does

### The measurement

`interception/endpoints.tsv`, written by the `rcl_publisher_init` and
`rcl_subscription_init` hooks the interceptor already owns:

```text
<model key>\t<pid>\t<node FQN>\t<pub|sub>\t<topic FQN>
```

Deliberately not read off `events.jsonl`. That file records MESSAGES, and a
subscription on a pipeline whose sensor is absent looks there exactly like a
subscription that does not exist — so grading a graph by traffic grades the
run's coverage instead. An endpoint exists the moment it is created. Measured
on the same Autoware run: **982 endpoints created, 63 of them carrying a
message.** Grading by traffic would have called the other 919 missing.

The model key travels beside the registered FQN for the reason issue #0017
gives: for a node the launch file did not name they are different strings, and
under a shared container process every composable inherits the container's
key — so neither name alone identifies the node, and the join is decided per
row.

### The verdicts

`scripts/verify_graph.py <run-dir> --model <m.yaml>`:

| verdict | meaning |
|---|---|
| confirmed | the model claims it, the run created it |
| **CONTRADICTED** | the model claims the opposite direction. Always a defect |
| missing | the run created it, the model does not claim it. Under-coverage |
| unexercised | the model claims it, the run never created it |

`unexercised` is split by whether the node wired anything at all, because the
two are different evidence: a node that never started says nothing about the
inference, while a node that wired eleven endpoints and not this one says the
inferred name was wrong.

`just verify-graph` in `tests/fixtures/autoware/` runs the whole thing.

## What it found first: the interceptor could not see a remap

Before any graph could be graded, the measurement disagreed with the model on
227 of 269 edges. The model was right.

`expand_topic_name` expanded `~/input/odometry` to
`/control/control_evaluator/input/odometry` and then applied the node's remap
rules — through `rcl_get_global_arguments`, **which rcl does not export**. It
appears once in the whole ROS tree, inside a doc comment in `rcl/remap.h`. The
optional symbol group was all-or-nothing, so it resolved to `None` on every
installation, and the un-remapped expansion was used unchanged.

`ros2 node info` on the running node said `/localization/kinematic_state`.

This is not only a verification problem. **Every consumer keyed by topic name
was affected** — the frontier and stats plugins, the Chrome trace,
`play_launch measure` — on any launch file that remaps, which is most of them.
It survived because no fixture in the tree had a remap;
`tests/fixtures/simple_test/launch/remapped_topic.launch.xml` now does, and
`test_interception_reports_remapped_topic_names` asserts both directions (the
remapped name present AND the un-remapped name absent — asserting only the
first passes on a build that records both). Confirmed to fail without the fix.

The replacement is `rcl_node_resolve_name`, the one call rcl itself makes from
`rcl_publisher_init`: expansion and remapping together, in rcl's own order. It
is tried for an already-absolute name too, since `-r /a:=/b` is a legal rule
that the old leading-slash fast path skipped.

## Measured: the phase 76 inference

Autoware 1.5.0 `planning_simulator.launch.xml`, 100 s, `observable`:

| | |
|---|---|
| model endpoints, all inferred | 269 |
| confirmed | **230** |
| **contradicted** | **0** |
| unexercised, node wired nothing | 28 |
| unexercised, node wired other endpoints | 11 |
| observed endpoints the model does not claim | 752 |

**Zero contradictions.** The `~/input/`–`~/output/` convention did not get a
single direction wrong across 269 edges.

The 11 real misses are two kinds, and both are worth naming:

- **Four are services, not topics.** `~/output/mrm/emergency_stop/operate` is
  a service client, and Autoware spells a client the same way it spells a
  publisher. Nothing in a launch file distinguishes them, so the inference
  cannot; the edge lands in `structure.topics` where it should be
  `structure.services`. The causality it asserts is real — the handler does
  drive the operator — so the ancestor closure errs toward more coupling,
  which is the safe direction for an independence test.
- **Seven endpoints the running nodes never created**, three of them
  publishers of `/planning/scenario_planning/status/stop_reasons`. A remap
  naming a topic the node no longer has is a dangling remap in the launch
  file, which is a finding about Autoware rather than about us.

The verifier was checked against a deliberately corrupted model — five topics
with their sides swapped produced nine contradictions and exit 1 — so
"contradicted 0" is a result and not a rule that cannot fire.

## What the graph then said about Autoware

With the graph verified, the second question was what the rules now report on
a tree that has no contracts at all. The answer was **nothing**, twice over,
and both were structural.

1. **`check` returned at "No manifests found"** before rendering any
   cross-scope diagnostic. Correct while the graph was empty; wrong once a
   graph exists that no manifest produced. Most rules do need a declared
   requirement, but `causal-dag-global` does not — a cycle is a defect whether
   or not anyone wrote a budget.
2. **`build_global_graph` took its VERTEX set from the manifests only.** Step 2
   built edges from `index.topics`, which phase 76 had filled, into a
   `graph.nodes` that was empty: 114 edges between 0 vertices. Every consumer
   that asks `graph.nodes` whether an endpoint exists — `contains_node`, the
   path-graph builder, the cycle DFS — dropped them all. Phase 76 delivered
   edges and no vertices, and reported clean.

`ManifestIndex::derived_nodes` now carries the launch scope of every node the
remap derivation touched, and `build_global_graph` seeds a vertex for each one
no manifest declares. A derived vertex carries STRUCTURE ONLY — no paths, no
endpoint properties, no concurrency — so the rules that need a declared cost
still find nothing and report incomplete evidence, while the rules that need
only reachability finally have something to walk. A manifest declaration
always wins.

With both fixed, Autoware's planning simulator reports **11 causal cycles**
from a launch tree with zero contracts. They are honest ones:

- `vehicle_cmd_gate → simple_planning_simulator → …  → vehicle_cmd_gate`, six
  of the eleven. This is the simulator closing the physical loop, which is
  precisely the case the rule's own message names as legitimate — and it is
  also the reason a real vehicle would not show it.
- `mrm_handler ↔ mrm_comfortable_stop_operator` and the emergency-stop twin,
  via `/operate` and `/status`. Two of the eleven, and both are the service
  misclassification above.
- `vehicle_cmd_gate ↔ external_cmd_converter` via `/control/current_gate_mode`
  and `/external/selected/control_cmd`, which is a genuine feedback pair and
  the candidate for a missing `state: true`.

The diagnostic used to end `Contract files: .` on these, because no node on
the cycle came from a contract; it now says so.

## Limits

- **The service/topic distinction is not recoverable from a launch file.**
  Measured cost: 4 of 269 edges, and 2 of 11 reported cycles. A name-shape
  heuristic (`/operate`, `/status`) was considered and rejected — a rule that
  fires on a correct contract is worse than one that does not fire. What DOES
  distinguish them is exactly this phase's measurement, so the fix, if it is
  worth one, is to feed a recorded run back into the model rather than to
  guess harder.
- **Grading needs a run that actually starts.** 28 of the 39 unexercised
  claims are three nodes that wired nothing; the sample map is absent on this
  machine, so the simulator never came up. A verification on a system that
  runs properly would grade those too.
- **Under-coverage is unmeasured in the other direction.** 752 observed
  endpoints are absent from the model, 608 of them on topics it has never
  heard of. A node that does not remap a topic never names it in a launch
  file, so this is the shape of the gap rather than a defect — but it is the
  gap a safety classification fails open through, and nothing yet reports it
  as a coverage figure a user would look at.
- **`derived_from_remaps` is not in the model.** The verifier recovers
  provenance by asking whether any endpoint on a topic appears in the contract
  endpoint tables, which is exact today. Serializing the flag was considered
  and dropped: a new model field with no consuming reader is the write-only
  field this campaign exists to remove.
