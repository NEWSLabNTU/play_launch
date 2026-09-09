# Phase 76 — the topic graph the launch file already states

Status: **complete** (2026-09-08). Prerequisite identified by
`r6-mitigation-barriers-review.md` §F4.

## Why

Measured while reviewing R6: the Autoware model resolved to **119 nodes and
0 topics**. `structure.topics` came only from contracts, and four scopes of
eighty-three have one — while **62 nodes carried 371 remaps** describing the
wiring, which nothing read. Every graph rule in the tree (criticality
propagation, `scope-budget`, the fault-reaction walk, R6's ancestor test)
was therefore working on almost nothing, and would have gone on reporting
clean results computed over an empty graph.

## What it does

A remap is a name mapping and carries no direction: `~/input/scan ->
/sensing/scan` names the topic, not whether the node reads or writes it.
What it does carry is a convention every ROS package in reach follows.
`remap_direction` reads that convention — `input`/`in` subscribe,
`output`/`out` publish, a bare `/diagnostics` publishes — in **one place**,
so there is one line to correct when it is wrong.

Three rules keep the inference honest:

- **A contract always wins.** A topic a contract declares is never touched:
  that is a fact an author wrote; this is an inference from a naming
  convention.
- **Undecidable remaps are counted, not guessed.** A wrong direction invents
  a causal edge, and every downstream rule inherits it; a missing one leaves
  the graph as sparse as it already was. Measured on Autoware, the
  convention decides 296 of 371.
- **Derived topics carry provenance** (`ResolvedTopic::derived_from_remaps`)
  and `dangling-entity` skips them — a derived topic with one side missing
  usually means the other side's remap was undecidable, which is a gap in
  the inference rather than a defect in the system.

`graph-from-remaps` (info) reports the counts every run, so the graph a
verdict was computed over is never implicit.

## Measured

Autoware 1.5.0, `planning_simulator.launch.xml`:

| | before | after |
|---|---|---|
| topics in the model | 0 | **110** |
| topics wired on both sides | 0 | **48** |
| publisher→subscriber edges | 0 | **114** |
| ancestors of `/control/vehicle_cmd_gate` | 0 | **19** |

`rt_av_demo` is unchanged: its contract declares every topic, so nothing is
derived and no verdict moves.

## What the graph immediately proved

The R6 review argued from launch files that neither Autoware emergency
channel is independent of the pipeline it bounds. The derived graph now
says it mechanically, which is the check R6 wanted:

- `autonomous_emergency_braking` — 19 ancestors, **all 19 shared** with
  `vehicle_cmd_gate`'s pipeline.
- `mrm_emergency_stop_operator` — 19 ancestors, all shared, and
  `vehicle_cmd_gate` **is itself an ancestor** of it.

An R6 barrier declared over either would be rejected naming the shared
nodes, from data rather than from reading launch files by hand.

## Limits

- **The convention is an inference, not a fact.** 75 of 371 Autoware remaps
  name a topic and no direction, and their endpoints are absent from the
  graph. Under-coverage means missing edges, which for a safety
  classification fails OPEN — the same hazard `criticality-from-hazards.md`
  §R3 names. The counts are reported for exactly this reason.
- **A node that does not remap a topic is invisible.** Default topic names
  never appear in a launch file at all.
- **Derived edges carry no message type, QoS, or rate**, so only the rules
  that need pure structure (ancestors, reachability, `causal-dag`) gain from
  them. `qos-match` and the rate rules still need a contract.
- **Ground truth existed and was not used yet.** Done in phase 77
  (`docs/roadmap/phase-77-graph-verified.md`): `interception/endpoints.tsv`
  records every endpoint a run CREATES, and `scripts/verify_graph.py` grades
  the model against it. Verdict on this phase: **230 of 269 inferred edges
  confirmed, zero contradicted.** Grading it turned up two defects of its own
  — the interceptor had never applied a remap rule, and this phase delivered
  edges into a graph whose vertex set was still contract-only.
