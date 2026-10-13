# Phase 88 - data-flow chains without contracts

Status: **in progress** (2026-10-14): C1 done. Design of record:
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
| C1 | Carried edges, per-node latency per edge, `stale_stamp` issue | Autoware, live: the 8-topic planning chain, and the 25.9 s gear stamp reported. **Done**: see below |
| C2 | Sampled edges: same node and thread, input among the node's subscriptions, ≥ 20 observations | the behavior planner's inputs shown as sampled, never as triggers |
| C3 | Origin propagation: data age per topic, upper bound flagged across sampled hops | age on `/control/command/control_cmd` reported |
| C4 | Chain view: source/sink picker, per-hop p50/p99, end-to-end age; budget line where a contract declares one | fixture: a three-node pipeline with a restamping middle node shows one carried and one sampled hop |

Cost target: ~0.2 µs per event (W5's order), measured on Autoware as W5 was.
Nodes, not containers: edges use the endpoint's owner (`EndpointOwner`).

## C1 result

`interception/chains.rs` keeps, per node `(ring, EndpointOwner hash)`, the
latest take of each of its last 1024 stamps; a publish of a stamp the same
node took on another topic adds an observation to that carried edge (a W5
histogram of take-to-publish). A node's takes and publishes share its
process's ring, so no pending queue is needed. Edges seen 20 times join into
chains (longest simple paths, sub-paths of a longer one dropped).
`GET /api/chains`, `interception/chains.json` at shutdown, and
`health/chains.rs`'s `stale_stamp`. Fixture `chain_probe.launch.xml`, test
`test_carried_edges_form_a_chain_and_an_old_stamp_is_reported` (verified by a
negative control).

Autoware, driven to a goal, live: 55 edges; the 8-topic planning chain
`path_with_lane_id → … → trajectory` at 67.5 ms summed p50; the gear stamp
reported (5.2 s this run: the edge ends once the vehicle engages and the gate
switches to `shift_decider`). `stale_stamp` opened three times, all one
shape — an input taken ONCE and its stamp re-sent at rate: the gear command,
the gate's operation mode, and `mrm_emergency_stop_operator` re-sending a
control command's stamp 47.7 s old. That is right for a state stamped when it
changed and wrong for data judged by age, and only the author knows which, so
the issue is **info**, not the warning first planned. Cost: 84 ns per event
in a microbenchmark where every publish matches.
