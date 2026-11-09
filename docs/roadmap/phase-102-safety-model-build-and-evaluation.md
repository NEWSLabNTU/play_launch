# Phase 102 - the contract safety model: build order and evaluation

**Status (2026-11-08): M0 done; M1 built, waiting on the rlm v0.3.13 tag.** M0a: F0's verdict is that F4 is needed (phase 100). M0b: the field reader, below. M1: below. Puts phases 100 and
101 (the work) into one build order and adds the evaluation that decides
whether the model is right. Design of record: rlm
`docs/design/safety-functions.md` (issue 58) and
`docs/design/fault-tolerance.md` (issue 59), as decided on 2026-11-07.
Overview page: the "Contract Safety Model" artifact.

## What reading the code changed

Before ordering the work, each proposed check was traced to the code it
would extend. Six findings:

1. **No runtime field reader exists.** The observer reads stamps, owners
   and QoS events; no `when:` predicate is evaluated at run time in
   play_launch today. The `commands` invariant needs the interceptor to
   read declared scalar fields by introspection, as `tf.rs` already does
   for `TFMessage`. This is the largest single piece, and the foundation
   of everything runtime-side. It also gives a runtime `when:` for free.
2. **`host` is not in the model.** `Deploy.host` was removed on
   2026-07-31 (multi-host launches partition at resolve time), so every
   node of one resolved model is on one host and `not_sharing: [host]`
   would always fail. Decided 2026-11-07: `host` is dropped; `process`,
   `container`, `cpus` remain. Independence across hosts is the separation
   of two models, which no single check sees.
3. **`process` depends on the container mode.** Composables in one
   container are one process under `observable` and `stock` and separate
   processes under `isolated`, and the model does not carry the mode.
   `check` must assume the shared case and say so; `launch` and `up`
   re-check with the mode they run.
4. **`reaction-bypasses-arbiter` is implied.** A topic with exactly one
   publisher can only be reached through that publisher, so
   `arbiter-bypass` (one publisher) already covers it. The rule is dropped.
5. **The walk already reaches past `emits`.** Since phase 83 it charges a
   safe state commanded "through a downstream hop" (`ReactionWalk.settle`,
   `manifest_loader.rs`). Tying a commanded topic to the level's reaction
   sink is a lookup, not a new walk.
6. **The runtime facts recovery and the fault classes need exist.**
   `respawn_delay` reaches the spawn path (`node_cmdline.rs`), startup
   times are in `startup_timeline.json` (`health/startup.rs`), every
   endpoint has an owner (`EndpointOwner`, #0080) and every publish a
   stamp. Repetition, resequencing and insertion need per-publisher state
   in the observer, not new events.

## Build order

One rlm release carries every grammar and model change (M1), so nano-ros
bumps once.

| Milestone | Units | Size | Depends on |
|---|---|---|---|
| **M0 foundations** | (a) phase 100 F0: the Safe-ROS-shaped fixture and today's walk verdict on it. (b) The interceptor's field reader: per declared `(topic, field path)`, resolve the offset once by introspection, emit the scalar on each publish; measure its cost. | M | - |
| **M1 grammar and static checks** | rlm, one release: `modes.<m>.commands`, `hazards.<h>.requirement`, `functions.<f>.m`, `independence:`, `external.on_silence`, `on:` + three classes; model fields. Resolver: `arbiter-bypass`, `commands-field-unknown`, `commands-unreached` (the commanded topic is not the level's reaction sink or downstream of it), `independence-violated` (shared container assumed), `interference-shared-process`, `recovery-unfit` (with `--against`), F4 only if F0 asks for it. | L | M0a |
| **M2 runtime checks** | play_launch observer: the `commands` invariant from the reaction's bound until the level is left (`safe-state-violated`), silence judged by the consumer (`final-element-silent`), runtime `arbiter-bypass`, `m` member counting, repetition / resequencing / insertion, the recovery a run took. `launch`/`up` re-check `independence` with the real container mode. | L | M0b, M1 |
| **M3 explain, extensions, guides** | `--explain` sentence per hazard with its `requirement` and `commands`; `mobile.protective_stop`; guides (safety function, fault tolerance, Simplex as a pattern). | M | M1 |
| **M4 nano-ros** | The model fields in a pull request; which runtime checks the image generates is recorded. | M | M1 |
| **M5 evaluation** | E1-E7 below; a report. | M | M2, M3 |

Phases 100 and 101 keep their units (F0-F7, T1-T7); this table is the
order they are built in.

### M0b, the field reader (done 2026-11-07)

The model's `when:` predicates name the fields; `play()` passes them to
each child in `PLAY_LAUNCH_INTERCEPTION_FIELDS`, the interceptor resolves
each dotted path once per publisher by introspection and sends one
`FieldValue` event per field per publish (a string as its FNV-1a hash), and
`interception/fields.json` reports each field's messages, changes,
publishers and last value. Test: `test_declared_fields_are_read_on_publish`
(the talker's `data`, a C++ `std::string`, hashed to the exact text of its
last message). Measured cost: 1.5 ns for the read, ~50 ns per field per
publish with the event and ring push, the same as a service call hook
(47 ns). At E5's scale, 10k declared-field publishes a second is 0.05% of a
core. `modes.<m>.commands` (M1) adds its fields to the same list.

### M1, grammar and static checks (built 2026-11-08)

rlm `acc6987` (CHANGELOG "Unreleased", GRAMMAR_VERSION 0.3.13) carries
every key. The resolver's rules, each with a fixture beside its correct
twin and a test in `manifest_check.rs`:

| Rule | Severity | Fixtures |
|---|---|---|
| `arbiter-bypass` | error | `contract_commands_bypass` / `contract_commands` |
| `commands-unreached` | error | `contract_commands_unreached` |
| `commands-field-unknown` | error | `contract_commands_field` |
| `reaction-waits-on-traffic` (F4) | error | `contract_safe_ros`, `contract_safe_ros_plain` / `contract_safe_ros_fixed` |
| `independence-violated` (process, container) | error | `contract_independence` / `contract_independence_ok` |
| `independence-violated` (cpus), `independence-unchecked` | error / info | both, with and without a platform file |
| `interference-shared-process` | warning | `contract_independence` / `contract_independence_ok` |
| `recovery-credited`, `recovery-unfit`, `recovery-uncredited` | info / warning / info | `contract_recovery` with `runs/fits`, `runs/unfit`, no run |

Every other fixture's verdict was compared before and after: only
`contract_safe_ros` moved, gaining the F4 error. A fault class
`repetition`, `resequencing` or `insertion` counts as detected by a
subscriber only with `on_violation.mechanism: application` (the node
checks each message), and removes value functions as silence does. Not in
M1: `m` member counting, `on_silence` settle and the three classes'
detection are runtime (M2); the requirement in every hazard diagnostic is
F3 (M3).

## Evaluation

The model is right if it can state the systems it is for, catches the
faults it claims to, stays quiet on correct systems, predicts the times it
checks, costs little at run time, and is no heavier to write than the
alternative. Each question has a method and a pass line set now, before
the results.

| # | Question | Method | Pass |
|---|---|---|---|
| E1 | **Expressiveness.** Can the cases be written with the decided primitives alone? | A contract per case: Safe-ROS, the seven use cases (as fixtures, physics in extensions), a Simplex system, the island, Autoware's MRM. Record every place a case needs something the grammar cannot state. | Every case written; each gap listed with a ruling (extension, out of scope, or a new primitive with its argument). |
| E2 | **Detection.** Does each rule fire on the defect it is for? | Seeded defects, one per rule, each beside its correct twin: a second publisher on a commanded topic; a gate that sends one safe message and forwards again; a stop that never releases after `exit:`; a silent safety node; redundant members in one container; a restart slower than the FTTI; a repeated, a reordered and an inserted message; a field name that does not exist. | 100% of seeded defects reported by the intended rule; 0 findings on every correct twin (negative controls). |
| E3 | **False positives on real systems.** | The island's contract, Autoware's overlay contracts and `rt_av_demo`, each with `commands` added on its MRC; `check` and a driven run. Classify every finding true or false by hand. | No false error. Every warning either true or explained in the report. |
| E4 | **Timing accuracy.** Do the derived bounds match runs? | Fault injection (`just fault` in `rt_av_demo`, extended): kill the detector with and without `respawn`; compare the credited restart time with the measured one; compare the invariant window's start with the first safe message seen. | Each measured time within the derived bound; the restart credit decided the same way the run went (credited ⇔ the run kept its mode). |
| E5 | **Runtime cost.** | The field reader and the per-publisher state on Autoware's planning simulator, measured as play_launch's per-subsystem self cost (the method of 2026-10-13). | Under 1% of one core added, and per-event cost reported in ns. |
| E6 | **Author burden.** | Lines added to the island's and Autoware's contracts; the same cases written with the dropped `final_elements:` and `recovery:` forms for comparison. | Fewer lines than the dropped forms, and no fact written twice (the field census reports no new write-only or duplicated field). |
| E7 | **Same answer in nano-ros.** | The island's model through play_launch's checks and through nano-ros's image monitors, on the same fault. | Both report the same hazard, level and verdict. |

The report lives in `docs/reports/safety-model/`, regenerated from the
runs (as `docs/reports/rt-mixed-criticality/` is), with every number
traceable to a run directory.

## Risks

- **The field reader's reach.** Introspection gives offsets for C and C++
  messages; nested sequences and strings need the TF path's care. Scope:
  scalar fields at a dotted path, which every `commands` example needs.
- **Container mode at check time.** Finding 3 means `check` can over-report
  independence on systems run `isolated`; the message must say which mode
  it assumed.
- **Release coordination.** M1 is an rlm release during phase 98/99's
  rename schedule; it goes through the session cutting releases.
- **Evaluation by the author.** E3's true/false classification is
  judgement; the report lists each finding so it can be checked.
