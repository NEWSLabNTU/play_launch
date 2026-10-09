# Phase 86 - a lifecycle node's autostart comes from the launch file

Status: **designed** (2026-10-10). Not started. This is play_launch's part of a
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
  lifecycle node by rlm's `apply_to_launch`.

The second is nano-ros boot policy, and it is leaving the model (rlm
`model-boundary.md`). The portable source of the per-node fact is the launch
file, and this parser cannot read it:

- `attr_spec.rs` says the `lifecycle_node` element "is not dispatched at all",
  so an XML or YAML `<lifecycle_node>` is not modelled as a lifecycle node.
- Jazzy `launch_ros` `LifecycleNode(autostart=True)` (bool, default `False`)
  requests `TRANSITION_CONFIGURE` then `TRANSITION_ACTIVATE`, verified against
  the `jazzy` branch's `lifecycle_node.py`. Humble has no `autostart`.

## Work

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
  phase 486 W6, which stops projecting `[lifecycle]`, `features` and the build
  half of `[deploy.*]`. Update the field census baseline; `Execution.features`
  leaves the consumer list. Fast-forward `main`.

## Acceptance

- A Jazzy `<lifecycle_node autostart="true">` and a `LifecycleNode(autostart=True)`
  resolve to the same model, both with `lifecycle_autostart: active`, under both
  parsers.
- `play_launch up` brings that node to `active` with no external
  `ros2 lifecycle set`.
- A model that rlm resolved from a nano-ros `system.toml` carrying
  `[lifecycle]`, `features` and `[deploy.*]` build fields still resolves. Those
  keys reach nothing.

## Migration

- **Linux users: nothing to rewrite.** None of the keys leaving the model had
  a Linux meaning. A `system.toml` passed through the legacy `--sched` bridge
  parses as before.
- **New:** `autostart` on a lifecycle node now works in XML, YAML and Python
  launch files.
