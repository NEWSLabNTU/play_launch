---
id: 53
title: "`--interception on` is inert on `run`, so a single-node run can never be measured"
status: resolved
resolved_in: fix(#0053) on main, 2026-09-25
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

## Fix

**Wired**, and the flag is now the only thing that turns it on there.

`run_direct` reads the interception half of `runtime_config`: it applies
`--interception on|off` over `interception.enabled`, creates the shared-memory
ring per child and injects the env vars (`setup_child_interception`, so the fd
plumbing is not copied), and spawns `run_interception_task` with the run's own
shutdown receiver so the summaries are written when the node stops. Setup sits
AFTER the `--check` early return — `run --check` creates no ring and injects
nothing — and the two names a bundle is keyed by (`interception_identity_path`,
`interception_node_name`) were lifted out of `up::play` as `pub(super)` rather
than duplicated.

**The decision is made against `EnforceMode::Off`, not the real mode.** On
`launch`/`up` a non-`off` mode IMPLIES interception, because the hooks are the
rule engine's only event source (issue #0031). `run` builds no engine and
refuses every explicit non-`off` mode, so nothing there can imply anything:
passing the real mode would have turned interception on for every plain
`play_launch run` in the name of an engine that is not built. Unset therefore
still means off, and the default `run` bundle is byte-for-byte what it was.

**The rule engine stays out, structurally.** `run_interception_task` is called
with `None` for both the engine and the lifecycle channel, so there is nothing
to reach for. #0045's refusal is untouched and still fires ahead of
`create_log_dir`, including alongside `--interception on` — half of its
reasoning was that `run` observes nothing, and that half is now gone, but the
half it stands on (a contract is located by launch file) is not.

**The gate is a bundle two readers can use**, asserted in
`tests/tests/run_interception.rs` (4 tests): `run --disable-all --interception
on demo_nodes_cpp talker`, stopped with SIGTERM, then `play_launch measure
<bundle> --model <m.yaml>` — which reports `17 message events over 8.0 s` off
the file it just read — and `scripts/capture_manifest.py`, which joins
`endpoints.tsv` + `node_identity.tsv` + `stats_summary.json` onto the model and
emits `/chatter`, `8 published, 0 taken, 1.14 Hz`. A test asserting only that
`interception/` exists would prove the plumbing ran, not that what it wrote is
usable. `--disable-all` is kept (it covers monitoring, diagnostics and the web
UI, never interception) so the suite stays off port 8080. Non-vacuity checked
by reverting the setup block: both gate tests fail on `no
.../interception/events.jsonl`, while the negative control and the #0045
refusal still pass.

**The member name is the node's FQN**, from the same `fqn_for` the scheduling
lookup uses — `run` builds its record by hand, so `model_fqn` is `None` and the
shared helper would otherwise stamp the bare executable, which is not the key
`measure` or `capture_manifest.py` join on (`/talker`, not `talker`).

Still stale, outside this change: `docs/guide/runtime-enforcement.md` §Limits
says `--enforce-rules` on `run` "accepts the flag and does nothing with it"
(#0045 made it a refusal) and describes the same invocation as leaving "a
bundle with no `interception/` directory".

## Provenance

Found 2026-09-24 while fixing #0045; recorded there as a deliberate non-fix
("wiring that would ADD a capability rather than remove a false claim") and
filed now.
