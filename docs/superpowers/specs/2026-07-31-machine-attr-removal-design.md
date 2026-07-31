# Removing `<node machine=>`: ROS 2 conformance for multi-host launch

- **Date**: 2026-07-31
- **Status**: Approved, not yet implemented
- **Repos touched**: `play_launch_parser`, `ros-launch-manifest`,
  `ros-launch-resolve`, `play_launch` (code); `nano-ros` (docs only)

## Problem

`play_launch` accepts `<node machine="robot1">` and maps it to
`execution.deploy[fqn].host` in the SystemModel. `nano-ros` consumes that
field to partition a launch file into per-host entry bakes
(`Plan::for_host`).

`machine=` is ROS 1 roslaunch syntax. It is not part of ROS 2:

- `launch_ros`'s `Node.parse()` reads `args`, `ros_args`, `name`,
  `exec_name`, `pkg`, `exec`, `namespace`, and the `remap`/`param` child
  elements. There is no `machine` attribute, and the string `machine`
  appears nowhere in the `ros2/launch_ros` repository.
- The XML frontend is strict. `launch_xml/entity.py`'s
  `assert_entity_completely_parsed()` raises on any attribute no action
  consumed. Running the current fixture through Humble's frontend gives:

  ```
  ValueError: Unexpected attribute(s) found in `node`: {'machine'}
  ```

- ROS 2 has no multi-machine launch at all. The design proposal
  (`ros2/design` PR #255, "Multi-Machine Launching") was opened
  2019-09-16 and closed without merging.

Consequences in this repo today:

1. **Parser parity is broken**, against CLAUDE.md's rule that Python
   behavior is always correct. `play_launch resolve <f>` succeeds;
   `play_launch resolve --parser python <f>` fails with the ValueError
   above. `tests/fixtures/multihost/launch/multihost.launch.xml` cannot be
   loaded by `ros2 launch` either.
2. **`docs/design/unified-system-model.md` states the opposite.** Line 108
   reads "It's standard ROS 2, mature in nano-ros since phase-263." The
   first clause is false.
3. **`execution.deploy.host` is write-only in `play_launch`.** Nothing in
   `src/play_launch/src` reads it; `replay` spawns every node locally with
   no warning that the model declared hosts.
4. **The ROS 1 `<machine>` tag is unsupported**, so `machine="robot1"` is a
   bare label with no address, user, or env-loader — the attribute never
   carried the information ROS 1 attached to it.

A wider gap surfaced during the audit: the Rust parser silently ignores
**any** unrecognized `<node>` attribute. `bogus_attr="x"` parses fine. Six
attributes that ROS 2 genuinely accepts are silently dropped:
`exec_name`, `ros_args`, `launch-prefix`, `cwd`, `emulate_tty`, `shell`.
Verified: `ros_args="-r /chatter:=/foo"` appears in the Python-parsed model
and is absent from the Rust-parsed one. `machine=` is one instance of this
class of bug.

## Decisions

Recorded from the design conversation, in the order taken:

1. **Stick to standard ROS 2 launch format.** `machine=` is removed rather
   than kept as an extension. nano-ros migrates to follow.
2. **`Deploy.host` is removed from the model schema entirely**, not kept
   and left unpopulated.
3. **No launch argument gets special treatment.** `play_launch` does not
   learn about an arg named `host`; the multi-host pattern is a convention
   documented for launch authors, invisible to the tool.
4. **Code changes land in `play_launch` and its three submodules.**
   nano-ros gets an issue plus a phase doc; no nano-ros code is touched.
5. **The parser rejects unknown attributes**, matching `launch_xml`, rather
   than special-casing `machine=`.
6. **The six known-but-unsupported ROS 2 attributes warn rather than
   error.** Implementing them properly is a follow-up, not part of this
   work.
7. **The strict check covers every element the parser handles**, with a
   hand-curated allowlist per element.
8. **`Deploy.host` is removed immediately.** nano-ros keeps its pinned
   vendored revision until it migrates.

## Replacement pattern

No new code. The pattern already works in both parsers, verified before
the design was written:

```xml
<launch>
  <arg name="host" default="all"/>
  <node pkg="talker_pkg" exec="talker" name="talker"
        if="$(eval '&quot;$(var host)&quot; in (&quot;robot1&quot;, &quot;all&quot;)')"/>
  <node pkg="listener_pkg" exec="listener" name="listener"
        if="$(eval '&quot;$(var host)&quot; in (&quot;robot2&quot;, &quot;all&quot;)')"/>
  <node pkg="hub_pkg" exec="hub" name="hub"/>   <!-- no if= → runs on every host -->
</launch>
```

```sh
play_launch resolve multihost.launch.xml host:=robot1 -o robot1.yaml
```

The resulting model holds only `robot1`'s slice plus the unconditioned
nodes. Verified for both `--parser rust` and `--parser python`:
`host:=robot1` yields `/hub/hub` and `/robot1/talker1`; `host:=all` yields
all three.

The partition moves from bake time to **resolve** time. Downstream
consumers take the model as-is and need no host field. A node with no
`if=` is shared across hosts, which is how the old "no `machine=` means
every host" rule expresses itself naturally.

`host` is an ordinary launch argument. The name is the launch author's
choice; `play_launch` attaches no meaning to it.

## Change inventory

### `play_launch_parser` (github.com/jerry73204/play_launch_parser)

Remove the `machine` capture end to end:

- `src/actions/node.rs` — the `machine` field on `NodeAction`, the
  `optional_attr_str("machine")` read, and the resolve step in
  `to_capture()` (three sites, around lines 46, 80–82, 179–181, 261)
- `src/captures.rs` — `NodeCapture.machine`
- `src/record/types.rs` — `NodeRecord.machine` plus the `machine: None`
  initializers at lines 422 and 450
- `src/ir.rs` — the `machine: Option<Expr>` variant field
- `src/traverser/ir_evaluator.rs` — lines 309, 330
- `src/actions/mod.rs` — line 70
- `src/python/bridge.rs` — line 49
- `src/record/generator.rs` — lines 332–334, 359, 532
- `src/actions/container.rs` line 281 and `src/actions/executable.rs`
  line 126 — the `machine: None` initializers and their comments
- `src/traverser/yaml.rs` line 357 — the YAML-path `machine: None`

Add strict attribute validation (see below).

### `ros-launch-manifest` (github.com/NEWSLabNTU/ros-launch-manifest)

- `model/src/lib.rs` — delete `Deploy.host` (line 973). The doc comment on
  `Deploy.target` explains `None` as "the deploy entry exists for its other
  facts (e.g. a `host` derived from `<node machine=>` in a launch-only
  resolve)". After this change a launch-only resolve creates no deploy
  entries at all, so that justification is rewritten rather than left
  stale.
- `model/src/system_config.rs` — delete the `by_machine` placement fallback
  (~lines 305–325), the `existing_host` preservation on insert (~lines
  369–377), and the `machine= places both nodes` test (~lines 573–618).
  With `machine=` gone, `by_machine` is unreachable: multi-block placement
  falls back to requiring an explicit `nodes = [..]` entry, which is the
  behavior nano-ros issue 0291 added the fallback to avoid.

### `ros-launch-resolve` (github.com/NEWSLabNTU/ros-launch-resolve)

- `resolve/src/ros/launch_dump.rs` — `NodeRecord.machine` (line 195) and
  its doc block (lines 186–193)
- `resolve/src/ros/model_builder.rs` — the `deploy_hosts` map and the
  `machine` → `deploy.host` insert (lines ~455, 483–484), and the deploy
  construction at lines ~789–794
- `resolve/src/ros/manifest_loader.rs` — `machine: None` at lines 2152,
  2198, 2308
- Bump the `parser` and `third-party/ros-launch-manifest` submodule
  pointers

### `play_launch` (this repo)

- `src/play_launch/src/execution/node_cmdline.rs` — `machine: None`
  (line 303) with its comment (lines 301–302), and the `machine: _`
  destructure (line 368)
- `src/play_launch/src/commands/run.rs` — `machine: None` (line 54)
- `tests/fixtures/multihost/launch/multihost.launch.xml` — rewritten to
  arg + condition
- `tests/tests/resolve_multihost.rs` — rewritten. The current assertions
  (`execution.deploy["/robot1/talker1"].host == "robot1"`, and that a
  machine-derived deploy stays unplaced) are replaced by node-set
  assertions per `host:=` value, run against both parsers.
- `docs/design/unified-system-model.md` — the "Cross-track (nano-ros)"
  section. The false "standard ROS 2" claim is corrected and the #236
  narrative is annotated with this reversal. It is not deleted: it records
  why the field existed.
- `docs/roadmap/phase-46-unified_system_model.md` and
  `docs/roadmap/README.md` — mark step 46.1 as reverted with a pointer
  here.
- `docs/guide/multi-host.md` — new. Documents the pattern, and states that
  ROS 2 has no multi-machine launch (`ros2/design` #255 closed unmerged)
  and that `machine=` / `<machine>` are ROS 1 roslaunch.
- `CLAUDE.md` — a "Key Recent Changes" entry.
- Bump the `src/ros-launch-resolve` submodule pointer.

## Strict attribute validation

`Entity` already exposes `attributes() -> Vec<(&str, &str)>`, so the check
is a comparison against a declared allowlist. There is no need to port
`launch_xml`'s read-tracking (`__read_attributes`) with interior
mutability.

```rust
// src/xml/entity.rs
pub struct AttrSpec {
    pub element: &'static str,
    pub supported: &'static [&'static str],
    pub known_unsupported: &'static [&'static str],
}

impl AttrSpec {
    pub fn validate(&self, entity: &impl Entity) -> Result<()> { /* … */ }
}
```

Each attribute present on an element takes one of three paths:

| Case | Behavior |
|---|---|
| in `supported` | consumed, silent |
| in `known_unsupported` | `WARN: <node launch-prefix=…> is valid ROS 2 but not supported by the Rust parser; use --parser python` |
| neither | `ParseError::UnexpectedAttribute`, wording mirroring launch_xml: ``Unexpected attribute(s) found in `node`: {machine}`` |

`if` and `unless` belong in every element's `supported` list — they are
handled by the condition layer, not by the action's `from_entity`.

Two structural details the implementation must respect:

- **The spec needs a `children` list alongside the attribute lists.** In
  XML, `<param>` is a separate element with its own spec, so `<node>`'s
  attribute list must not contain `param`. In YAML, `param` is a *key of
  the node mapping*. One table serves both frontends only if legal child
  names are tracked separately and consulted for YAML alone.
- **The XML check needs two hook sites, not one.** The traverser's
  `type_name()` dispatch covers actions, but child elements never reach it
  — `<param>`, `<remap>`, and `<env>` are consumed by the action's own
  `for child in entity.children()` loop. Each such loop needs its own
  validation call. Validation also has to run *before* the `if`/`unless`
  check: ROS 2 evaluates conditions at launch time, so it validates a
  conditioned-out element, while this parser skips it at parse time.

For `<node>`:

- `supported`: `pkg`, `exec`, `name`, `namespace`, `args`, `output`,
  `respawn`, `respawn_delay`, `if`, `unless`
- `known_unsupported`: `exec_name`, `ros_args`, `launch-prefix`, `cwd`,
  `emulate_tty`, `shell`

The remaining twelve action modules (`arg`, `container`,
`declare_argument`, `executable`, `group`, `include`, `let_action`,
`load_composable_node`, `set_env`, `set_parameter`, `set_remap`, and
`node_container` within `container.rs`) each get a curated spec.
`node_container` inherits `<node>`'s set plus its own; `<executable>` takes
`ExecuteProcess`'s set (`cmd`, `cwd`, `emulate_tty`, `launch-prefix`,
`name`, `output`, `respawn`, `respawn_delay`, `shell`).

Curation is manual. Extracting `get_attr` calls from ROS 2's `parse()`
methods produces a list contaminated with child-element and nested-call
names (`param`, `remap`, `from`, `to`, `value`), so it is a starting point
for review, not a source to copy.

The parser has three frontends: XML, YAML, and Python. The Python path
delegates to real `launch`/`launch_ros` and inherits ROS 2's strictness for
free. The YAML path (`src/traverser/yaml.rs`, 783 lines) is a **separate**
traversal over `serde_yaml_ng::Value` mappings that bypasses `Entity` and
the action modules entirely, so it needs the same allowlists applied
through its own mechanism — checking mapping keys rather than XML
attributes. Both frontends are in scope.

`traverser/yaml.rs:357` carries a fourteenth `machine: None` site
("YAML-launch node machine= deferred") that the removal also deletes.

### Ground truth for the allowlists

The allowlists below were **measured**, not inferred, by probing Humble's
frontend with one candidate attribute at a time per element
(`tmp/probe_attrs.py`, `tmp/probe2.py`):

| Element | ROS 2 accepts |
|---|---|
| `node`, `node_container` | `if unless pkg exec name namespace args ros_args exec_name output respawn respawn_delay launch-prefix cwd emulate_tty shell` |
| `executable` | `if unless name output respawn respawn_delay launch-prefix cwd emulate_tty shell cmd` |
| `arg` | `if unless name default description` |
| `let`, `set_env`, `set_parameter` | `if unless name value` |
| `group` | `if unless scoped forwarding` |
| `include` | `if unless file` |
| `push-ros-namespace` | `if unless namespace` |
| `set_remap` | `if unless from to` |
| `load_composable_node` | `if unless target` |
| `composable_node` | `if unless pkg plugin name namespace` |
| `param` (child) | `name value from type` |
| `remap` (child) | `from to` |
| `env` (child) | `name value` |
| `arg` (include child) | `name value` |

Two findings from the probe that change the work:

1. **`<launch>` itself is not strict in ROS 2.** `<launch zzz="1">` parses
   without complaint, so the root element is excluded from the check.
2. **`<group namespace=>` and `<group ns=>` are rejected by ROS 2** but
   read by the Rust parser (`actions/group.rs`). This is a second
   permissive divergence of the same class as `machine=`, found by the
   probe. Namespacing a group in ROS 2 requires a `<push-ros-namespace>`
   child. Removing the attributes would break any launch file relying on
   them, so they go in `known_unsupported` (warn), not `supported` — the
   warning tells authors their file is not portable to `ros2 launch`.

Child elements (`param`, `remap`, `env`, and `<include>`'s `<arg>`) reject
`if`/`unless`: they are not actions. Their specs must not inherit the
condition attributes.

## Testing

**Differential test (the primary safety net).** A test drives both parsers
over a matrix of `(element, attribute)` pairs and asserts they agree on
accept versus reject. Real ROS 2 is the oracle, which is CLAUDE.md's rule
stated as a test. A curation mistake surfaces as a failure naming the
offending attribute rather than as silent divergence. The matrix covers,
per element: every attribute in `supported`, every attribute in
`known_unsupported` (both parsers must accept — the Rust side warns), and a
synthetic unknown attribute (both must reject).

The probe scripts showed how this test can lie: a fixture that fails for an
unrelated reason (`load_composable_node` with a `<composable_node>` missing
its `name`) makes every candidate look accepted. Each element's fixture
must therefore assert a **passing baseline with no injected attribute**
before any candidate is judged, and the test fails loudly if a baseline
does not parse.

**Multi-host fixture test** (`tests/tests/resolve_multihost.rs`, rewritten).
For each of `--parser rust` and `--parser python`, resolve the fixture at
`host:=robot1`, `host:=robot2`, and `host:=all`, and assert the exact node
set each time. Also assert that no `execution.deploy` entry carries a
`host` key, which is the regression guard on the removal.

**Rejection test.** `machine=` on `<node>` produces the launch_xml-shaped
error from the Rust parser, not a silent parse.

**Regression.** `just test-all` in this repo. Autoware's 48 `<node>` sites
use only `pkg`/`exec`/`name`/`namespace`/`output`, so the fixture set is
expected clean, but its includes reach far more launch files than the
fixtures do — this run is what validates the curated allowlists against
real-world launch XML.

**Per-repo.** `cargo test` green in each submodule before its pointer is
bumped.

## nano-ros coordination

Documentation only. Following their conventions — `docs/design/` holds
rationale as numbered RFCs, `docs/roadmap/` holds phase work-breakdowns,
`docs/issues/NNNN-slug.md` holds issues.

`docs/issues/0356-machine-attr-is-ros-1-not-ros-2.md` — the finding with
its evidence: `launch_ros` has no `machine` attribute, `launch_xml`
rejects it, `ros2/design` #255 closed unmerged, and their own
`examples/workspaces/*/src/demo_bringup/launch/multihost.launch.xml` fails
`ros2 launch`. States the breaking change: `Deploy.host` is leaving the
vendored `model` crate, and their pinned revisions keep building until
they migrate.

`docs/roadmap/phase-325-multihost-via-launch-args.md` — the migration:

- rewrite the four `multihost.launch.xml` fixtures (`c`, `cpp`, `mixed`,
  `rust` workspaces) to arg + condition
- delete `PlanNode.host` and `Plan::for_host`
  (`packages/cli/nros-cli-core/src/codegen/entry/mod.rs`)
- `nros codegen entry --host <id>` and `nros::main!(host="…")` become
  generic launch-argument passing; `host:=<id>` is then an ordinary launch
  arg with no special casing
- update `packages/testing/nros-tests/tests/multihost_partition_bake.rs`,
  whose premise ("the deploy-target id == the launch `machine=` id")
  dissolves, and `multihost_e2e.rs`
- `[deploy.robot1]` blocks stop doubling as placement selectors; placement
  with multiple in-scope blocks needs explicit `nodes = [..]`

Net effect for them: the partition moves from a bake-time filter to a
resolve-time argument, and one code path disappears.

## Landing order

Submodule nesting fixes the sequence. `play_launch_parser` and
`ros-launch-manifest` are both submodules of `ros-launch-resolve`, which is
a submodule of `play_launch`.

1. **play_launch_parser** — drop `machine`; add `AttrSpec`, the per-element
   specs, and the differential test
2. **ros-launch-manifest** — drop `Deploy.host`, `by_machine`,
   `existing_host`, and the 0291 test
3. **ros-launch-resolve** — drop `NodeRecord.machine` and the
   `model_builder` mapping; bump both submodule pointers
4. **play_launch** — bump the `ros-launch-resolve` pointer; rewrite the
   fixture and `resolve_multihost.rs`; fix `node_cmdline.rs` and `run.rs`;
   correct the design and roadmap docs; add `docs/guide/multi-host.md`;
   update `CLAUDE.md`
5. **nano-ros** — issue and phase doc

Each step builds and tests green before the next begins. Work happens on a
branch per repo and is pushed; nothing merges to `main` in any repo without
explicit approval.

## Out of scope

- Implementing `exec_name`, `ros_args`, `launch-prefix`, `cwd`,
  `emulate_tty`, or `shell` in the Rust parser. They warn; a follow-up
  phase implements them. `ros_args` and `exec_name` are the cheap ones —
  `NodeRecord` already carries both fields and the Python parser populates
  them.
- Any nano-ros code change.
- Strictness for the Python parser path, which inherits ROS 2's own
  strictness for free.
- Making `<group namespace=>` an error, or implementing
  `<push-ros-namespace>` semantics for it. It warns.
- Elements ROS 2 has that this parser does not dispatch at all
  (`unset_env`, `append_env`, `timer`, `log`, `shutdown`, `reset`,
  `set_parameters_from_file`, `set_use_sim_time`).
- Element-level strictness (unexpected *child elements*). `<node>` already
  rejects unknown children via `ParseError::UnexpectedElement`; other
  elements are not audited here.
