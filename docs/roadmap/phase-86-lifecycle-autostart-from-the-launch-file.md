# Phase 86 - a lifecycle node's autostart comes from the launch file

Status: **in progress** (2026-10-10, corrected 2026-10-12). W0-W5 landed
2026-10-12 (see "Done"); the Jazzy-side acceptance is not run. This is play_launch's part of a
cross-repository change. The design of record is rlm's
`docs/model-boundary.md`; the work items, ordering and retirement live in
nano-ros phase 486 (`docs/roadmap/phase-486-launch-model-boundary.md`), where
this phase is W5 and the rlm pin bump is W7.

## Why

The SystemModel already has the right per-node shape: `lifecycle: bool` and
`lifecycle_autostart: Option<Autostart>` (`None | Configure | Active`). Today it
has only two inputs:

- the contract's `lifecycle: true`;
- nano-ros's system-wide `[lifecycle] autostart`, projected onto every
  lifecycle node by THIS repository's resolver:
  `src/ros-launch-resolve/resolve/src/model.rs` loops over
  `structure.nodes` with rlm's `SystemConfigToml::lifecycle_autostart()`
  accessor. rlm's `apply_to_launch` writes only the execution layer and never
  touched the per-node field. (Corrected 2026-10-12: this doc, rlm's
  `model-boundary.md` and nano-ros phase 486 had all named rlm as the
  projector, which put the deletion in the wrong repository.)

The second is nano-ros boot policy, and it is leaving the model (rlm
`model-boundary.md`). The portable source of the per-node fact is the launch
file, and this parser cannot read it:

- `attr_spec.rs` says the `lifecycle_node` element "is not dispatched at all",
  so an XML or YAML `<lifecycle_node>` is not modelled as a lifecycle node.
- Jazzy `launch_ros` `LifecycleNode(autostart=True)` (bool, default `False`)
  requests `TRANSITION_CONFIGURE` then `TRANSITION_ACTIVATE`, verified against
  the `jazzy` branch's `lifecycle_node.py`. Humble has no `autostart`.

## Work

- **W0. Stop projecting the system-wide default.** Delete the
  `[lifecycle] autostart` loop in `resolve/src/model.rs`, so the model's
  per-node `lifecycle_autostart` comes only from the launch file (W1/W2) and
  the contract. Lands with W5's pin bump, not before: until nano-ros reads the
  default from its own overlay (phase 486 W3), a nano-ros model resolved by an
  older nano-ros still expects it here. rlm deletes the accessor afterwards
  (phase 486 W8).
- **W1. XML and YAML frontends.**
  - Dispatch `<lifecycle_node>` like `<node>`, plus the lifecycle marker.
  - Add its attribute spec from the Humble/Jazzy diff (`external/diff_attrs.sh`):
    Jazzy's `autostart`, a bool substitution.
  - Resolved `true` gives `lifecycle_autostart = Active`. Absent or `false`
    gives `None`, because "the launch file did not ask" is not "configure only".
- **W2. Python frontend.** The `LifecycleNode` mock (`pyexec/src/api`) captures
  `autostart`. It crosses the dlopen boundary in `ExecCaptures`, which is an
  ABI bump, for the reason the 2 -> 3 bump gave: a serde-defaulted field would
  be accepted by an old object and answered wrong.
- **W3. Parity.** Add a lifecycle fixture under `just test-parity`, and a
  differential test against Jazzy `launch_ros` in the `attr_spec.rs` style. A
  Humble-only launch file stays byte-identical.
- **W4. `up` honours `Active`.** Drive configure then activate after spawn, as
  stock `launch_ros` does. `Configure` (reachable only from a contract or
  another producer) stops after configure. `None` does nothing. Runtime
  enforcement's `lifecycle_nodes` set (`runtime_enforcement/view.rs`) is
  unchanged.
- **W5. Pin bump.** `just bump-manifest <tag>` to the rlm tag from nano-ros
  phase 486 W6, which stops writing `Execution.features`, `Deploy.extra` and
  `Target::Mcu` (W0 above removes the lifecycle projection on this side). Update the field census baseline; `Execution.features`
  leaves the consumer list. Fast-forward `main`.

## Done (2026-10-12)

- **W1** (`3e6452e`). XML and YAML dispatch `<lifecycle_node>` through the
  `<node>` path; `NodeAction` reads the marker off the element name and,
  for that element only, `autostart`. Coercion is `launch`'s bool coercion
  (`1`/`0`, then the YAML 1.1 spellings); anything else is refused. The
  attribute spec is `<node>`'s plus `autostart`, kept in step by a unit
  test. Captures and records carry `lifecycle` / `lifecycle_autostart`,
  skipped when absent. The model lowering: `lifecycle` is the launch
  marker OR the contract's; resolved `true` is `active`, `false` and absent
  are none. The attribute differential landed here, not in W3, because its
  completeness test refuses a specced attribute nobody probes.
- **W2** (`0577497`). The `LifecycleNode` mock takes `autostart=`, resolves
  it at execute. C ABI 9.
- **W3** (`ea62f69`). `tests/fixtures/lifecycle` with two `test-parity`
  gates and a per-node expected answer (`expect.py`); `--parser python`
  records the marker and `autostart` too.
- **W4** (`b8b1c42`). `execution/lifecycle_autostart.rs`: per node, wait
  for `<fqn>/change_state` (no deadline, as stock), configure, then
  activate if `active`. Only `lifecycle: true` nodes are driven.

Measured on Humble (no Jazzy install was available):

- `just test`: parser unit 507/507, play_launch unit 365/365, layer-2 unit
  260/260, fast integration 189/189 (20 skipped: privilege, corpus,
  `rt_workspace` unbuilt). `just test-ir` 553/553. `just check` passes.
- `just test-parity`: simple_test, container_events, **lifecycle** and
  autoware (119 = 119) pass; `lifecycle_autostart` SKIPs (Humble's
  `launch_ros` has no `autostart`), `rt_workspace` SKIPs (unbuilt).
- The attribute differential against Humble: 3/3. Its baseline confirms
  Humble refuses the element (`Unrecognized entity of the type:
  lifecycle_node`). **The Jazzy side is NOT RUN**: the candidate matrix
  for `<lifecycle_node>`, and the `lifecycle_autostart` parity gate, need a
  Jazzy `launch_ros` this host does not have. Both are wired to run there
  without changes.
- **Humble files are unchanged** except where they use `LifecycleNode`.
  Every tracked `*.launch.{xml,yaml,py}` (248) resolved with the base and
  the new resolver (`--parser rust`), compared byte for byte: 187
  identical, 55 fail on both (packages or arguments absent here), 6
  differ. Of the 6: four differ only in `env` order, and the base differs
  from itself run to run on those same four; one is `$(anon)`. The sixth
  is the intended change: `python/test_lifecycle_nodes.launch.py`'s four
  `LifecycleNode`s gain `lifecycle: true`.
- `play_launch up` on the resolved `lifecycle_autostart.launch.xml`:
  `ros2 lifecycle get` gives `/lc_on` active, `/lc_off` and `/lc_unsaid`
  unconfigured. With `/lc_off` set to `configure` in the model, it is
  inactive.

- **W5** (rlm pin `v0.1.55`). `just bump-manifest v0.1.55`; the three
  manifests and locks move together, the layer-2 lock's unrelated
  `[[patch.unused]]` churn discarded. `git grep` finds no reader of
  `execution.features`, `Deploy.target` or `Deploy.extra` here. Field
  census (`--nano-ros` pointed at a nano-ros checkout): 6 unread fields,
  all already in the baseline, so the baseline is unchanged.
- **W0**. The `[lifecycle] autostart` projection loop in
  `resolve/src/model.rs` is gone. No test asserted the projection; a new
  one does the opposite: `tests/tests/resolve_system_lifecycle.rs`
  resolves `lifecycle_autostart.launch.xml` with a `system.toml` setting
  `autostart = "configure"` and expects the launch file's answers
  (`/lc_on` active, the others none). Against the old resolver it fails
  with `configure` on `/lc_on`. `up`'s warning for an autostart request
  on a non-lifecycle node stays: a contract or another producer can still
  make one.

Measured for W0/W5 (Humble): `just test` parser unit 507/507,
play_launch unit 365/365, fast integration 189/190 (20 skipped as above).
The one failure is `check_dx::a_contract_needing_a_newer_grammar_names_both_releases`:
rlm v0.1.55 left `GRAMMAR_VERSION` at 0.1.54, so the refusal names
`v0.1.54` while `--version` names `v0.1.55`. rlm's own
`the_grammar_version_is_the_release` fails on that tag for the same
reason; the fix is an rlm release, not a change here. Layer-2 unit
260/260, run with `NROS_CARGO_FLAGS=` because `--locked` refuses the
lock without the discarded `[[patch.unused]]` hunks. `just check` passes
except `check-layer2-isolation` under that same `--locked`; the gate run
by hand with `NROS_CARGO_FLAGS=` passes.

Two things decided in the code that the design did not spell out:

- **A launch-file `LifecycleNode` on Humble now models as `lifecycle:
  true`.** "A Humble-only launch file stays byte-identical" holds for files
  without one; a file with one changes by exactly that field, which is the
  defect named under "Why".
- **`up` drives only nodes marked `lifecycle: true`**, and reports any other
  node with a request. Before W0, `[lifecycle] autostart` was projected
  onto EVERY node, lifecycle or not; driving those would have waited
  forever for a `change_state` that never appears.

## Acceptance

- A Jazzy `<lifecycle_node autostart="true">` and a `LifecycleNode(autostart=True)`
  resolve to the same model, both with `lifecycle_autostart: active`, under both
  parsers.
- `play_launch up` brings that node to `active` with no external
  `ros2 lifecycle set`.
- A model resolved from a nano-ros `system.toml` carrying `[lifecycle]`,
  `features` and `[deploy.*]` still resolves. Those keys reach nothing: no
  lifecycle default on any node, no `execution.features`, no `Deploy.extra`.
- Nothing here reads `execution.deploy` (`target`, `Mcu`, `extra`); measured
  2026-10-12 by `git grep`, so the fields rlm stops writing cost this repository
  nothing.

## Migration

- **Linux users: nothing to rewrite.** None of the keys leaving the model had
  a Linux meaning. A `system.toml` passed through the legacy `--sched` bridge
  parses as before.
- **New:** `autostart` on a lifecycle node now works in XML, YAML and Python
  launch files.
