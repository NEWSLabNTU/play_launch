# Data-flow chains without contracts

**Status: design, 2026-10-14; roadmap: phase 88** (`docs/roadmap/phase-88-data-flow-chains.md`). Measured, not built. Supersedes the
"contracts only" framing of `big-system-observability.md` §M: a contract can
still put a budget on a chain, but finding the chain and measuring it needs
none.

## The question

"How long does sensor data take to become a command?" is the latency question
every robotics team asks, and today play_launch answers it only for a path a
contract declares (`check`, `measure`, the RuleEngine). Most systems have no
contracts. The interceptor already sees every publish and take with its
`header.stamp`, its monotonic time, its node and its thread, so the chains can
be discovered from the running system.

## What a run shows (measured)

Autoware planning simulator, driven to a goal in autonomous mode, 60 s of
`events.jsonl` (386 k publishes, 433 topics; recorded with `--record-events`).
Analysis scripts, each taking an `events.jsonl`: `scripts/chains/
stamp_by_topic.py` (the first, topic-only attempt), `carried_edges.py` (A),
`sampled_edges.py` (B).

**A. Stamp carried across a node (exact).** Node N took a message stamped
`s` on topic A and later published `s` on topic B. Then A →N→ B is a real
edge, and `t_pub − t_take` is N's latency for it, queueing included.

- 285 of 433 topics carry stamps; **47** carry, in most messages, a stamp the
  publishing node took on another topic. 59 node-grounded edges.
- The planning pipeline falls out whole: `path_with_lane_id → path → path →
  trajectory → … → /planning/trajectory`, eight topics, per hop 0.01–38 ms
  median, 590 ms worst (behavior planner).
- It also finds bugs: `vehicle_cmd_gate` republishes `/system/emergency/
  gear_cmd`'s stamp on `/control/command/gear_cmd` with a median 25.9 s
  age — an old message re-sent with its old stamp.
- Grounding in the node matters. Linking any two topics that share a stamp
  (the first attempt) produced sibling and diagnostic edges
  (`objects → /diagnostics → /api/perception/objects`).

**B. Same thread, take then publish (sampled).** A publish by N on thread T,
whose latest take on T was topic A within 200 ms. 90% of publishes have one;
370 of 432 topics are covered. But "the last input this node handled" is not
"the input that caused this output":

- right for a callback-driven node: `trajectory_follower/control_cmd →
  /control/command/control_cmd`, 0.04 ms;
- right as DATA FLOW but not as trigger for a timer node:
  `control_cmd → /localization/kinematic_state` (the simulator) at 12 ms;
- wrong for a timer node with several inputs: the behavior planner's output
  is attributed 479 times to `occupancy_grid_map/map` and 173 times to the
  costmap, whichever it handled last.

**Where chains break.** A node that stamps its output with `now()` starts a
new chain: 235 topics are where stamps are born. Planning → control breaks
there (the controller restamps), so A alone gives segments, not end to end.

## Design

Two kinds of edge, never mixed silently:

| Edge | Evidence | Confidence | Latency it gives |
|---|---|---|---|
| carried | same node took and published the same stamp | exact | that node's take → publish |
| sampled | same node and thread, latest take before the publish, over ≥ 20 observations, and A is among the node's subscriptions | the data flowed, the trigger is unknown | a LOWER bound on that hop's staleness: `t_pub` minus the node's latest take of A on any thread (C2; the thread's own take is no bound on a multi-threaded executor) |

**Data age, end to end.** For each publish, the age of the oldest data behind
it: origin time propagated per node.
- On a take, the node remembers, per input topic, the origin time of the
  message it took (for a stamp seen before: when that stamp was first
  published; else its publish time).
- On a publish, the output's origin is the carried stamp's origin if the
  stamp was carried; otherwise the oldest origin among the node's latest
  takes on its sampled inputs.
- `age = t_pub − origin`, recorded per topic as a distribution (the W5
  histogram). On `/control/command/control_cmd` that is how old the newest
  sensor data behind a command is — the number the question asks for.
- Sampled hops make it approximate, and the page says so: per hop the node's
  latest take is a lower bound on the data it used (C2).

**Chains, on demand.** The page lets a user pick a source topic and a sink
topic; play_launch shows the paths between them over observed edges, per hop
p50/p99 and the end-to-end age distribution at the sink, with each hop marked
carried or sampled. No picking: the default view lists the longest carried
chains and their sinks' ages.

**Contracts, optional.** Where a contract declares a path and a budget, the
chain view shows the budget line and the RuleEngine keeps judging it. Nothing
here requires one.

**Findings worth raising as issues** (contract-free):
- `stale_stamp`: a node publishes a stamp it took more than 10× the topic's
  period ago (the 25.9 s gear command). Built in C1 as info: every case on
  Autoware re-sends an input taken once, right for a state, wrong for data.
- `data_age`: a topic's p99 age above a threshold the user sets per sink (no
  default: there is no universal number).

## Where it runs and what it costs

In the consumer, on events it already receives; nothing new in the `.so`.
- One global `stamp → first publish time` map, bounded (as the W5 matcher's
  per-topic index; 512 stamps per topic).
- Per node: its latest take per (input topic, thread), and per-edge
  histograms. Autoware: 59 carried and ~600 sampled edges.
- Per take: one map write. Per publish: one lookup and a min over the node's
  inputs (≤ 20 on Autoware).
- Estimate ~0.2 µs per event, the W5 order; to be measured as W5 was.

Needs interception and stamped types. A node built from the composables of
one container is one "node" here unless the endpoint's owner (already shipped,
`EndpointOwner`) is used — it should be: the container is the wrong unit.

## Plan

| Step | Work | Acceptance |
|---|---|---|
| C1 | carried edges and per-node latency, `stale_stamp` | Autoware: the 8-topic planning chain and the 25.9 s gear stamp, from a live run |
| C2 | sampled edges, guarded by subscriptions and a minimum count | the behavior planner's inputs reported as sampled, not as triggers. Done |
| C3 | origin propagation, data age per topic | age on `/control/command/control_cmd` reported, upper-bound flagged |
| C4 | chain view: source/sink picker, per-hop table, age distribution | fixture: a three-node pipeline with a restamping middle node shows one carried and one sampled hop |
