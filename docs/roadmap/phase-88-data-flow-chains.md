# Phase 88 - data-flow chains without contracts

Status: **planned** (2026-10-14). Design of record:
`docs/design/data-flow-chains.md`. Replaces item M of
`docs/design/big-system-observability.md`, which assumed a contract.

## Why

"How old is the sensor data behind this command?" is the latency question
every robotics team asks. play_launch answers it today only for a path a
contract declares, and most systems have none. The interceptor already sees
every publish and take with its stamp, time, node and thread.

## Measured before planning

Autoware planning simulator, driven to a goal, 60 s (`scripts/chains/`):

- **Carried edges** (a node took stamp s on A, published s on B) are exact:
  47 topics, 59 edges, the planning pipeline as one 8-topic chain, and a
  gate re-sending a 25.9 s-old gear-command stamp.
- **Sampled edges** (same thread, latest take before the publish) cover 90%
  of publishes but say "last input handled", wrong as a trigger for timer
  nodes with several inputs: labelled sampled, an upper bound.
- Chains break where nodes restamp (235 topics); origin propagation per node
  bridges them.

## Work

| Step | Work | Acceptance |
|---|---|---|
| C1 | Carried edges, per-node latency per edge, `stale_stamp` issue | Autoware, live: the 8-topic planning chain, and the 25.9 s gear stamp reported |
| C2 | Sampled edges: same node and thread, input among the node's subscriptions, ≥ 20 observations | the behavior planner's inputs shown as sampled, never as triggers |
| C3 | Origin propagation: data age per topic, upper bound flagged across sampled hops | age on `/control/command/control_cmd` reported |
| C4 | Chain view: source/sink picker, per-hop p50/p99, end-to-end age; budget line where a contract declares one | fixture: a three-node pipeline with a restamping middle node shows one carried and one sampled hop |

Cost target: ~0.2 µs per event (W5's order), measured on Autoware as W5 was.
Nodes, not containers: edges use the endpoint's owner (`EndpointOwner`).
