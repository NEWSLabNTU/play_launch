---
id: 53
title: "`--interception on` is inert on `run`, so a single-node run can never be measured"
status: open
type: enhancement
severity: low
---

# 0053 - the verb for iterating on one node is the one that records nothing

**Repo:** `play_launch` at `a6506261`
**Affects:** `src/play_launch/src/commands/run.rs`; compare `up::play`, which
sets up child interception from `runtime_config.interception`

## Symptom

`run` never reads `runtime_config.interception`, so `--interception on` does
nothing: no `LD_PRELOAD` injection, no ring buffer, no consumer task, and no
`interception/` directory in the bundle. The flag is accepted and ignored.

## Cause

Same root as #0045 — `run` calls `load_runtime_config` and uses only the
scheduling half of the result — but a different consequence, which is why it
is filed separately. #0045 was about a false claim (a gate that could not
fail) and was fixed by REFUSING the flag. This one is about a missing
capability, and refusing would be the wrong answer.

## Impact

`run` is the verb for iterating on a single node, which is exactly when
per-message data is most useful and cheapest to collect. Today a developer
must wrap the node in a launch file to get a bundle `play_launch measure` can
read, `scripts/capture_manifest.py` can walk, or the Chrome trace export can
render.

Unlike contracts, interception needs **no launch file**: it hooks
`rcl_publish`/`rcl_take` in whatever process is spawned, and the artifacts it
writes (`events.jsonl`, `endpoints.tsv`, `node_identity.tsv`) are keyed by
node and topic, not by scope. So the obstacle that made #0045 a refusal —
`ContractChannel`'s two variants are both keyed by launch file — does not
apply here.

## Fix direction

Wire the interception half of `runtime_config` into `run`'s spawn, mirroring
`up::play`: create the ring per child, inject the env vars, spawn the consumer
task, and write the summaries at shutdown. The rule engine stays out of it —
#0045 settled that `run` has no contract to enforce — so this is the
observation half only.

Gate: a `run` bundle that `play_launch measure` can read, asserted end to end.

## Provenance

Found 2026-09-24 while fixing #0045; recorded there as a deliberate non-fix
("wiring that would ADD a capability rather than remove a false claim") and
filed now.
