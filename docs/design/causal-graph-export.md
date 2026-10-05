# Declared Causal Graph Export (`ros-launch-resolve check --export-graph`)

**Status:** Implemented (Phase 42.1)
**Consumers:** [autoware-system-model-study.md](autoware-system-model-study.md)
Q1 (cycles), W2 (measured-model join), W4 (report).

`ros-launch-resolve check --export-graph <path>` walks the already-loaded
`ManifestIndex` (the same data the `check` rules consult) and serializes the
**declared** causal graph: ROS nodes, topics, pub/sub wiring, node-level and
scope-level `paths:`, and a cycle catalogue computed *before* `state:` cuts
are applied. This is an export, not a validation step — existing rules and
exit codes are unaffected; `--export-graph` composes with `--contracts`,
`--sched`, `--rule`, etc. as usual.

Implementation: `src/play_launch/src/ros/causal_graph.rs`. Reuses
`manifest_graph::build_global_graph()` (Phase 35's node-to-node dataflow
graph, already carrying an `is_state` flag per edge) for the cycle-detection
substrate — no changes to the `ros-launch-manifest` check crate.

## CLI

```
ros-launch-resolve check <pkg> <launch> --export-graph <path> [--contracts <dir>] [...]
```

Format is picked by extension: `.dot` (case-insensitive) writes a Graphviz
digraph; anything else (including no extension) writes pretty-printed JSON.

## JSON schema (`version: 2`)

Version 2 (phase 85 I6) carries everything the fault-reaction walk reads, so
a figure or a report can be drawn from the export alone: services and their
edges, `external:` marks, detectors (`on_violation`), node-path triggers and
safe states, hazards, functions and modes. One field changed meaning: a node
path's `input` is now its EFFECTIVE trigger's inputs. Version 1 copied the
legacy `input:` list, so every `trigger: { input: [...] }` path exported
`input: []`, indistinguishable from a timer; `trigger` now says which.

Top level:

| key | type | meaning |
|---|---|---|
| `version` | `u32` | schema version (bump on breaking shape changes) |
| `nodes` | `NodeOut[]` | ROS node vertices |
| `topics` | `TopicOut[]` | topic vertices |
| `pub_edges` | `PubEdgeOut[]` | node → topic |
| `sub_edges` | `SubEdgeOut[]` | topic → node, tagged causal/state/required |
| `node_paths` | `NodePathOut[]` | intra-node `paths:` (endpoint → endpoint) |
| `scope_paths` | `ScopePathOut[]` | cross-node `paths:` (topic → topic) |
| `cycles` | `CycleOut[]` | cycle catalogue, computed before `state:` cuts |
| `services` | `ServiceOut[]` | client -> service -> server (v2) |
| `hazards` | `HazardOut[]` | guards, fault classes, interval, reaction (v2) |
| `functions` | `FunctionOut[]` | named guard groups (v2) |
| `modes` | `ModeOut[]` | operational modes and the ladder (v2) |

`NodeOut`: `fqn`, `scope_id`, `pkg` (optional), `criticality` (optional,
`high`\|`medium`\|`low` string as authored -- advisory, not validated here),
`derived_criticality` (optional, v2: `{level, hazard, role}`, what the
hazards derive), `contracted` (v2: declared in a contract).

`TopicOut`: `fqn`, `type` (msg type), `rate_hz` (optional, topic-level
declared rate), `max_transport_ms` (optional), `derived_rate_hz` (optional,
v2), `external` (optional, v2: `pub`\|`sub`\|`both`, the side provided
outside the tree).

`PubEdgeOut`: `node`, `topic`, `endpoint`, `rate_hz` (topic-level — the
primary "declared rate fact"), `min_rate_hz`/`max_rate_hz` (optional,
endpoint-level overrides), `on_demand` (v2, present when `true`: the
publisher promises no rate).

`SubEdgeOut`: `topic`, `node`, `endpoint`, `causal` (bool, `!state`),
`state` (bool, the endpoint's `state: true`), `required` (bool, orthogonal
to `causal`/`state`), `min_rate_hz`, `max_age_ms`, `max_transport_ms`
(all optional; the transport is the subscriber's own, else its topic's),
`on_violation` (optional, v2: `{reaction, on: [fault classes]}`, the edge
of a detector).

`NodePathOut`: `node`, `path_name`, `input`/`output` (the node's own
endpoint names, not topic FQNs; `input` is the effective trigger's inputs),
`trigger` (v2: `{"timer": {"rate_hz", "jitter_ms"?}}`, `{"input": [...]}`,
`"once"`, `"spontaneous"` or `"unclassified"`), `safe_state` (optional, v2:
`{emits, settle_ms?, derived}`), `max_latency_ms`, `tolerance_ms` (both
optional), `scope_id`, `cross_node: false` (always -- node paths are
intra-node by construction).

`ServiceOut` (v2): `fqn`, `type`, `servers`/`clients` (`[{node,
endpoint}]`), `external` (optional: `server`\|`client`\|`both`).

`HazardOut` (v2): `name`, `scope_id`, `severity` (optional), `guards`
(`[{members: [topic FQN], all_of}]`, one per group), `on` (the fault
classes claimed; empty when none is named), `ftti_ms`, `reaction` (a scope
path or a mode), `entry_speed_mps` (all optional but `guards` and `on`).

`FunctionOut` (v2): `name`, `scope_id`, `members`, `all_of`.

`ModeOut` (v2): `name`, `scope_id`, `requires` (functions), `fallback` (the
ladder, in order), `reaction` (the rung's scope path), `window_ms` (a
windowed rung's least duration), `exit` (`{on, to}`) (the last three
optional). A hazard's ladder is its `reaction` mode's `fallback`, each rung's
`reaction` the scope path that reaches it.

`ScopePathOut`: `scope_id`, `path_name`, `input_topics`/`output_topics`
(resolved topic FQNs, potentially spanning many nodes), `max_latency_ms`,
`tolerance_ms`, `correlation` (all optional), `cross_node: true` (always).

`CycleOut`: `members` (`CycleMemberOut[]`, the cycle in traversal order —
each member is `{from, to, topic, state}`, node-to-node with the connecting
topic as a label), `cut_by_state` (subset of `members` where `state: true`
— the edges whose removal breaks this cycle under the actual, per-manifest
`causal-dag` rule), `broken` (`bool`, `true` iff `cut_by_state` is
non-empty).

**Cycle detection method:** a single DFS pass (white/gray/black coloring)
over the node-to-node graph, counting `state:` edges as causal (i.e. the
graph *before* cuts). Each back-edge found yields one simple cycle,
reconstructed from the DFS path and deduplicated by its sorted edge-index
set. This is the standard practical "list the cycles" approach at
study/Autoware scale — it is **not** an exhaustive elementary-cycle
enumeration (e.g. Johnson's algorithm); a dense SCG with many alternate
simple cycles through the same strongly-connected component may see only a
subset surfaced. Good enough for tooling/human inspection; not a
substitute for the `causal-dag` validation rule.

**Important semantic note for W2:** the existing `causal-dag` check rule
(`ros-launch-manifest/check/src/rules/causal_dag.rs`) runs **per manifest
file**, not on the cross-scope merged graph. The Autoware export surfaced a
real cycle that is *not* broken by any `state:` edge —
`/vehicle_cmd_gate → /simple_planning_simulator → /controller_node_exe →
/vehicle_cmd_gate` — spanning three different contract files/scopes. `check`
reports 0 errors on this run precisely because the rule never sees the
merged cross-scope graph. This is a known gap the study should flag (Q1);
the export's `cycles[].broken == false` entries are exactly how W2/W4
should surface it, whether or not a future rule is added to check for it
globally.

## DOT rendering

Plain and deliberately unstyled: nodes as ellipses, topics as boxes, `pub`
edges solid node→topic, `sub` edges topic→node dashed when `state: true`
(blue when `required: true` and not `state`). Verified with
`dot -Tsvg <file>.dot -o <file>.svg` (graphviz installed in this
environment) — renders without warnings.
