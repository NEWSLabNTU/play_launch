# Phase 102 - the contract safety model: build order and evaluation

**Status (2026-11-09): M0 to M4 done, M5 evaluated** (`docs/reports/safety-model/`). M0a: F0's verdict is that F4 is needed (phase 100). M0b: the field reader. M1: rlm v0.3.13 and the static checks. M2: the runtime checks. Each below. Puts phases 100 and
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

### M1, grammar and static checks (done 2026-11-09, rlm v0.3.13)

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

### M2, runtime checks (done 2026-11-09)

- **Container mode reaches the resolver** (`ContractSources::
  process_per_composable`): `launch` passes the mode it runs, `check` and
  `resolve` take `--container-mode`. Under `isolated` only a shared
  container violates `independence`, and `interference-shared-process` is
  silent. Before this a correct `isolated` deployment was refused.
- **Values decide modes**: a value function is lost while its predicate
  holds on its latest sample, and a redundant group with `m` is lost when
  fewer than `m` members publish.
- **A level holds what it commands**: the level a lost mode falls to is
  active until that mode is back; from its derived bound (the largest
  `LevelRoute.route_ms` charged for it) a message on a commanded topic that
  breaks a predicate is `safe-state-violated`, and silence past ten gaps
  with no declared consumer behaviour is `final-element-silent`. A level no
  hazard's walk reached is `safe-state-unjudged`.
- **Publishers and stamps**: runtime `arbiter-bypass` on a commanded topic;
  `insertion`, `repetition` and `resequencing` on a monitored topic whose
  hazard names them (an omitted `on:` names none of the three, since a
  stamp re-sent at rate is right for a state topic).
- **Recovery**: `hazard-recovered` says whether the guard came back inside
  the FTTI.

Not yet: a message constant name in a predicate is not judged at run time
(the model carries the name, not its value), and a hazard's own `when:`
still detects through its detector's output only. Unit tests cover each
rule; the running-system checks are E4's (`rt_av_demo`'s `just fault`).

### M3, explain, extension, guides (done 2026-11-09)

- `check --explain` states each hazard as one sentence
  (`ros/hazard_sentence.rs`): the requirement it implements, what it
  watches for which classes, its bound, the levels it reaches with the time
  charged for each (levels the fault removes and overrun rows left out),
  and what each level commands. Every finding about a hazard ends
  `[requirement <id>]`. Golden test `a_hazard_reads_as_one_sentence`; the
  field census line for `requirement` is gone.
- `mobile.protective_stop`, an rlm reference extension (unreleased; not
  bundled, so a contract ships a copy): `field >= v * reaction_total +
  v^2 / (2 decel) + margin`, all three worst cases proved. Safe-ROS's 5 cm
  field is refused at 1 m/s (840.33 mm needed), 0.9 m passes
  (`contract_protective_stop`, `_sized`).
- Guides: `docs/guide/safety-function.md` (Safe-ROS to a sized protective
  field, and Simplex as a pattern on `contract_simplex`) and
  `docs/guide/fault-tolerance.md`; `runtime-enforcement.md` lists the M2
  rules.

### M4, nano-ros (PR open 2026-11-09)

NEWSLabNTU/nano-ros#3341 bumps nano-ros to rlm v0.3.13. The image counts
nothing for `repetition`/`resequencing`/`insertion` (listed unobserved),
and writes each hazard's requirement as a comment in its generated tables.
What the image checks, from its RFC-0052:

| Field or rule | On the image |
|---|---|
| hazard detected / reaction / recovered, the active level as a mode | generated, unchanged |
| a level's route overrun | generated |
| `requirement` | carried (a comment in the generated table) |
| `repetition`, `resequencing` | carried, not checked: no stamp kept per publisher |
| `insertion`, runtime `arbiter-bypass` | not applicable: an image's publishers are fixed by its source |
| `m` | carried, not checked: the image does not read `functions` |
| `commands` / `safe-state-violated` | carried, not checked: nothing on target reads message fields |
| `on_silence` / `final-element-silent` | not applicable: the consumer is not on the image |
| recovery inside / past the FTTI | not applicable: an image does not respawn |
| `independence` | carried, not checked: Linux deployment facts |

### M5, evaluation (done 2026-11-09)

`docs/reports/safety-model/` (`generate.py` writes `results.md`). E1
pass (12 cases, 6 gaps ruled); E2 16 of 17 (no rule for a stop that never
releases: `level-not-released`, follow-up); E3 pass (the one new finding
on Autoware was a true one about the first draft); E4 credit and run agree
in both cases, but one restart outlasted its credit (the credit samples a
first spawn's startup); E5 no difference above noise on the idle
simulator, 52.2 ns per field; E6 26 lines against 58 for the dropped forms,
one fact written twice until rlm's type-only fix is pinned; E7 not run.
Three defects found and fixed: `startup_timeline.json` was written before
any first publish was known, so no real run could credit a restart; a
type-only contract topic was dropped (rlm) and not wired (resolver); and
`reaction-unguarded` called a mode a reaction.

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
