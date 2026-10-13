# Phase 88 - data-flow chains without contracts

Status: **in progress** (2026-10-14): C1, C2 and C3 done. Design of record:
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
  nodes with several inputs: labelled sampled. (Planned as an upper bound on
  staleness; C2 found it is a lower bound, see below.)
- Chains break where nodes restamp (235 topics); origin propagation per node
  bridges them.

## Work

| Step | Work | Acceptance |
|---|---|---|
| C1 | Carried edges, per-node latency per edge, `stale_stamp` issue | Autoware, live: the 8-topic planning chain, and the 25.9 s gear stamp reported. **Done**: see below |
| C2 | Sampled edges: same node and thread, input among the node's subscriptions, ≥ 20 observations | the behavior planner's inputs shown as sampled, never as triggers. **Done**: see below |
| C3 | Origin propagation: data age per topic, upper bound flagged across sampled hops | age on `/control/command/control_cmd` reported. **Done**: see below |
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

## C2 result

A publish that carries no stamp its node took is attributed to the latest
take the same node made on the same thread, stamped or not, and counted as a
SAMPLED edge `(input, node, output)`. Keying by node as well as thread is
the subscription guard: composables sharing an executor thread do not lend
each other inputs. A sampled edge is reported (`sampled` in `/api/chains`
and `chains.json`) once seen 20 times, with its `share` of the node's
publishes on the output, and never when a carried edge states the same
triple. Chains still join carried edges only.

**The hop's time is from the node's latest take of the input on ANY thread,
and it is a lower bound, not the upper bound the design planned.** The first
Autoware run timed it from the publishing thread's own take and read 4 to
10 s for the behavior planner's status topics: on a multi-threaded executor
the thread that publishes may not have taken the map for seconds while
another thread took it 60 ms ago. The node cannot have used data newer than
its latest take, so `t_publish − latest take` bounds the staleness from
below, and is exact for a node that uses the latest value. Fixture: a
`fuse` timer node with two inputs; unit tests for the thread split and the
cross-thread timing.

Autoware, driven to a goal, live: 54 carried and 623 sampled edges. The
behavior planner's `path_with_lane_id` is attributed to
`occupancy_grid_map/map` 482 times (share 0.73) and to the costmap 172 times
(0.26), as the offline analysis found (479/173), p50 5.1 and 6.4 ms; its
status topics read 60 to 74 ms (4 to 10 s before the change). The
controller's `control_cmd` comes from `kinematic_state` at share 1.00, 1.1
ms; the simulator's `kinematic_state` splits across four commands. Four
sampled edges have a p50 over a second, each a timer node whose input
arrives rarely (`/autoware/state` from the operation mode state, three
topic-state monitors on `/diagnostics`). Cost: 77 ns per event in a
microbenchmark (half of publishes carried); interception self-cost 2.2 to
2.3% of a core in the driven run, against 0.9 to 2.0% for C1.

## C3 result

Each publish gets an ORIGIN: the time of the publish its data began at, the
topic that was on, whether a sampled hop (or an input whose publish was not
seen) lies on the way, and the last 8 topics it passed through. A carried
hop passes on the origin of the message whose stamp it carries; a sampled
hop passes on the origin of the attributed input (C2's), timed from the
node's latest take of it; anything else starts a new origin. Origins are
kept per topic for the last 64 stamps and looked up when the taking node
next publishes, so a take read before its publish still resolves. `t_publish
− origin` is the topic's data age: `ages` in `/api/chains` and
`chains.json`, with `estimated_share` and the most common `source`.

Two departures from the design, both forced by feedback, both with a unit
test that fails without them:
- **The attributed input, not the oldest.** The design took the oldest
  origin among a node's sampled inputs. That is dominated by any state taken
  once (a gear command, a route), and around a loop it never refreshes: the
  simulator's state is older than the command that used it, which is older
  than the state, so the minimum sticks at the first publish.
- **A loop starts a new origin.** An origin that would return to a topic it
  passed through closes a loop; the output is born there instead. In the
  simulator → controller → simulator loop the state is born where it is
  published, and the command's age is the age of the state behind it.

Found on the way: a timer node's thread whose only take since startup was
`/parameter_events` attributed its outputs to it, 12 s of age. Takes on
`/rosout` and `/parameter_events` are no longer attributable inputs (matched
by FNV-1a hash, as the `.so` keys topics), which also tightens C2's sampled
edges.

Autoware, driven to a goal, live: ages on 357 topics.
`/control/command/control_cmd` p50 19.5 ms, p99 442 ms, max 444 ms, source
`/localization/kinematic_state` in 70% of publishes, 97% estimated; the
tail is the simulator's state attributed to `/planning/trajectory` (9% of
its publishes), carrying the planner's age into the loop.
`objects` p50 0.1 ms, measured (carried from tracking).
`path_with_lane_id` p50 181 ms from tracked objects. `/autoware/state` reads
12.5 s, as C2's sampled edge already said: a 10 Hz state re-published from
an input that changes rarely.

What age does NOT say: an input that does not trigger the publish never
enters it. The controller is attributed to `kinematic_state` in every
publish, so the trajectory it follows, and the planning age behind that, is
not in `control_cmd`'s age. Cost: 150 ns per event in a microbenchmark with
every publish stamped; interception self-cost 2.7–2.8% of a core in the
driven run (C2: 2.2–2.3%).
