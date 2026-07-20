# P46 W2 Report — shared launch fields in NodeInstance (46.1b + 46.2)

**Parent branch:** `feat/p46-launch-fields`, created from freshly-fetched
`origin/main` (`8b2f77f`, the just-landed `machine=`/#236 fix). Isolated
worktree (`.claude/worktrees/agent-a9a5b4234edadb512`).

**Submodule branch:** `src/ros-launch-manifest` at `feat/p46-launch-fields`,
also created from its own freshly-fetched `origin/main` (`c47c5bd`).

**Commits:**
- Submodule `5b23858` — `feat(model): NodeInstance carries launch spawn
  fields (46.1b)`
- Parent `acf167d` — `feat(model): populate remaps/ros_args/respawn/env
  into NodeInstance (46.2)` (includes the gitlink bump to `5b23858`, per
  the brief's Constraints section, so this branch builds/tests standalone)

Neither branch was pushed.

## 46.1b — model schema (submodule, additive)

`NodeInstance` (`model/src/lib.rs`) gains four fields, all
`#[serde(default, skip_serializing_if=...)]`:

```rust
#[serde(default, skip_serializing_if = "Vec::is_empty")]
pub remaps: Vec<Remap>,
#[serde(default, skip_serializing_if = "Vec::is_empty")]
pub ros_args: Vec<String>,
#[serde(default, skip_serializing_if = "Option::is_none")]
pub respawn: Option<bool>,
#[serde(default, skip_serializing_if = "Option::is_none")]
pub respawn_delay: Option<f64>,
#[serde(default, skip_serializing_if = "Vec::is_empty")]
pub env: Vec<EnvVar>,
```

with two new named-struct pair types (not bare tuples, following the
`SegmentNode` precedent):

```rust
pub struct Remap { pub from: String, pub to: String }
pub struct EnvVar { pub name: String, pub value: String }
```

`env` is `Vec<EnvVar>`, not `BTreeMap<String, String>` — launch env lists
may legitimately repeat a name (last one spawn-time wins) and declaration
order is part of the launch author's intent, so a map would silently drop
information a `Vec` preserves.

Doc comments on the fields explain the split explicitly: launch-derived
spawn INPUTS play_launch's Linux runtime consumes at spawn time (Phase
46.3); nano-ros ignores all four (it bakes from resolved
`structure.topics`, has no argv/process/respawn model — decision (a) in
`docs/design/unified-system-model.md`).

**Tests** (submodule): `model_instance_launch_fields_roundtrip` (unit,
serde round-trip + named-struct-shape assertions on the emitted YAML),
`node_instance_without_launch_fields_parses_with_defaults` (unit,
backward-compat: a `NodeInstance` YAML with none of the four keys parses
with empty/`None` defaults and re-emits none of them) — one copy each in
`model/src/lib.rs`'s `#[cfg(test)]` module and in
`model/tests/golden_roundtrip.rs` (the golden `perception.system_model.yaml`
fixture's `detector` node now carries all four fields; `tracker` stays
field-less and is asserted to default cleanly).

## 46.2 — populate (play_launch)

`model_builder::build_system_model` gained two helpers (`lower_remaps`,
`lower_env`) and wires them into all three node-insertion loops. Which
`LaunchDump` record type carries which field (a pre-existing asymmetry in
`launch_dump.rs`, not something this task changed):

| record type | remaps | ros_args | respawn / respawn_delay | env |
|---|---|---|---|---|
| `NodeRecord` (`<node>`) | yes | yes (**field exists, but the Rust XML parser never populates it — always empty in practice**) | yes | yes |
| `NodeContainerRecord` (`<node_container>`) | yes (**field exists, but the Rust XML parser's `ContainerAction::from_entity` explicitly skips `<remap>`/`<env>`/`<param>` children — always empty in practice**) | same caveat as above | yes | same caveat as above |
| `ComposableNodeRecord` (`load_node`) | yes | **no field — composable nodes have no CLI process** | **no field — no independent lifecycle** | field exists but the parser never sets it (always `None`) |

`model_builder` maps every field the record type has; for composable nodes
`ros_args`/`respawn`/`respawn_delay` are hardcoded to empty/`None` (there's
no source field to read). Removed the now-stale `#[allow(dead_code)]` on
`ComposableNodeRecord::env` since `model_builder` reads it.

**Tests** (parent): 3 new `model_builder::tests` unit tests
(`node_carries_remaps_ros_args_respawn_env`,
`container_carries_remaps_ros_args_respawn_env`,
`composable_node_carries_remaps_only`) — hand-built `LaunchDump` via
`serde_json::json!`, matching the `sched_loader::tests` convention, so the
container-side mapping has direct coverage despite the XML parser gap
above. Plus a new end-to-end fixture:
`tests/fixtures/launch_fields/launch/launch_fields.launch.xml` +
`tests/tests/resolve_launch_fields.rs`, which runs the real
`play_launch resolve` CLI over a `<node>` with `<remap>`/`<env>`/
`respawn="true"`/`respawn_delay="2.5"` and a `<composable_node>` with
`<remap>`, and asserts the emitted `system_model.yaml` carries them.

## Pasted model YAML (real `play_launch resolve` run over the new fixture)

```yaml
meta:
  version: 1
  inputs:
  - path: .../tests/fixtures/launch_fields/launch/launch_fields.launch.xml
    sha256: 212e06f0...
  - path: .../launch_fields.system_model.record.json
    sha256: d9155d3f...
  resolver:
    tool: play_launch
    version: 0.8.2
  record:
    path: .../launch_fields.system_model.record.json
    sha256: d9155d3f...
structure:
  scopes:
    /: {}
  nodes:
    /chatter/composable_container:
      scope: /
      pkg: rclcpp_components
      exec: component_container
    /chatter/composed_talker:
      scope: /
      pkg: composition
      plugin: composition::Talker
      container: /chatter/composable_container
      remaps:
      - from: chatter
        to: /chatter/composed
    /chatter/talker:
      scope: /
      pkg: demo_nodes_cpp
      exec: talker
      remaps:
      - from: chatter
        to: /chatter/renamed
      respawn: true
      respawn_delay: 2.5
      env:
      - name: TALKER_LOG_LEVEL
        value: debug
```

Confirms: named-struct `remaps`/`env` shapes, `respawn`/`respawn_delay`
present only on the regular node, composable node carries only `remaps`
(no `ros_args`/`respawn`/`env` keys at all — cleanly omitted by
`skip_serializing_if`), and the container has none of the four keys
(the pre-existing XML-parser gap, documented in the fixture's own doc
comment).

## Verification

- **Submodule** (`src/ros-launch-manifest`):
  - `cargo test` (workspace: model/sched/types/check) — all green
    (model crate: 13 unit + 7 golden = 20 passed).
  - `cargo clippy --all-targets --all-features -- -D warnings` — clean.
  - `cargo +nightly fmt --check` — clean (fixed one line the new test
    code introduced; nothing else touched).
- **Parent** (`play_launch`):
  - `cargo test -p play_launch` — 230 passed, 0 failed (227 pre-existing +
    3 new `model_builder` unit tests).
  - `just build-rust` (colcon build --packages-select play_launch) —
    succeeded; needed since this worktree had no prior `install/`.
  - `cd tests && cargo nextest run -E 'binary(rt_workspace) |
    binary(manifest_check) | binary(resolve_multihost) |
    binary(resolve_launch_fields)'` — 32 passed, 0 failed. Used the
    documented nested-worktree `[workspace]` shim in `tests/Cargo.toml`
    (added, ran, reverted before committing — confirmed zero diff on
    `tests/Cargo.toml` post-revert).
  - Manual `resolve` over the new fixture — pasted above.

## Deviations from the brief

1. **`ros_args` has no real-world exercise path.** `play_launch_parser`'s
   Rust XML parser hardcodes `ros_args: None` for both `NodeRecord` and
   `NodeContainerRecord` at every construction site (`record/generator.rs`)
   — there is no `<node ros_args=...>` XML syntax it recognizes. The
   `model_builder` mapping code is correct and unit-tested against a
   hand-built `LaunchDump`, but the end-to-end fixture can't show a
   non-empty `ros_args` because the upstream parser never produces one.
   Documented in the fixture's doc comment and the test's assertions
   (`talker.get("ros_args").is_none()`). Fixing the parser is out of this
   task's scope (submodule is `ros-launch-manifest`, not
   `play_launch_parser`).
2. **Container remaps/env have no real-world exercise path either.**
   `play_launch_parser`'s `ContainerAction::from_entity`
   (`actions/container.rs`) explicitly skips `<remap>`/`<env>`/`<param>`
   children of `<node_container>` ("containers don't typically have their
   own params" — a pre-existing comment, not something I added). So
   `NodeContainerRecord::remaps`/`env` are always empty from real XML
   today, even though the *schema* and *model_builder* fully support them.
   Covered instead by a direct `model_builder::tests` unit test that
   constructs a `NodeContainerRecord` with the fields set, bypassing the
   parser gap. Same out-of-scope reasoning as above.
3. **`ComposableNodeRecord::env`** is structurally present but the parser
   never populates it (`env: None` hardcoded at all 3 construction sites
   in `play_launch_parser`). `model_builder` still maps it through (for
   forward-compat, in case the parser gains this later) and the doc
   comments call this out explicitly.
4. Per the brief's Constraints section (which explicitly requests
   "parent ... + gitlink bump" in the same commit), the parent's 46.2
   commit includes the submodule gitlink bump to `5b23858`, resolving an
   apparent tension with the brief's opening sentence ("do NOT commit
   parent gitlink — orchestrator handles it") — interpreted as: don't
   commit a *bare*, code-less gitlink bump; bundling it with the actual
   46.2 code change (as the Constraints section spells out) keeps this
   branch buildable/testable standalone, which the verification steps
   above require. If the orchestrator wants the gitlink bump split into
   its own commit or dropped for their own merge sequencing, it's a
   trivial `git reset -- src/ros-launch-manifest` / re-commit.
