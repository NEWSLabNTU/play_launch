# Phase 106: Running and exporting part of a launch tree

**Status:** Done (2026-11-24). The gaps W4 and W5 named are closed, except
starting a member a Reload adds.
**Design:** `docs/design/launch-selection.md` (selecting, running, the web
UI's switches) and `docs/design/launch-extraction.md` (exporting a launch
file). Requested by the amr 175 session (2026-11-21) as `--scope
<pkg>/<file>`.

## Why

A launch tree states one whole system, and ROS offers one way to run part of
it: edit a launch file to add an argument and a condition. Running a
sub-file directly loses the namespaces, arguments and global parameters its
parents set. play_launch resolves the whole tree first, so it can run a
subset with every kept node getting exactly what the full launch gives it,
and write that subset down as a launch file. The use cases (design section
2) are fault localisation by switching parts off, repeated testing of one
component, debugging one node under a tool, replacing a part with a bag or
an own build, bans, layer-by-layer bring-up, re-running part of a recorded
run, and checking a launch-file edit.

## Work items

### W1 - The selection

**Done (2026-11-24).** `ros_launch_resolve::selection` (12 unit tests on a
model with Autoware's shapes); `resolve --explain` with a selection; on
Autoware's planning simulator, `--exclude load_topic_state_monitor.launch.xml`
lists the ten inclusions by their `topic` argument.

`ros_launch_resolve::selection`:
- selector parsing: `scope:`, `node:`, `pkg:`, `exec:`, `plugin:`,
  `container:`, the `/…` and launch-file shorthands, the predicates
  `[<arg>=<value>]`, `[under=<pkg>/<file>]`, `[nth=<n>]` and `[*]`, and
  `#<id>`; canonical rendering;
- evaluation over `structure.nodes` and `structure.scopes`: union of
  `only`s, minus `exclude`s, then the closure (a kept composable keeps its
  container); a file included more than once named without a predicate is
  an error listing the choices; a selector that matches nothing is an
  error; a closure that re-adds an explicitly excluded member is an error;
- reports: cut inputs (kept subscribers of topics whose publishers are all
  excluded), the three named traps (lifecycle manager `node_names`,
  `use_sim_time` without `/clock`, `/tf` without publishers);
- `resolve … --only/--exclude/--no-deps --explain`: the marked tree, what
  each selector matched, the closure, the reports, a `ros2 bag play
  --topics` line; writes no model.

### W2 - A run with a selection

**Done (2026-11-24).** `ActorConfig::start_stopped`, `MemberHandle`'s
Excluded set (shared with the runner, so an Excluded required node ends
nothing), `HealthSummary.excluded`, `MemberSummary.excluded`; the
shortfall, stage gate, `never_ready` and progress lines skip Excluded
members. Tests `tests/tests/launch_selection.rs` on the new fixture
`selection.launch.xml`. Contracts: an Excluded publisher never publishes,
so no silence is judged at startup; a publisher switched off mid-run is
W4's.

- `--only`, `--exclude`, `--no-deps`, `--start none` on `launch` and `up`;
- the Excluded state: registered, shown, never spawned, never respawned,
  not counted by startup completion, `never_ready`, `missing_node` or a
  `startup.order` stage, unable to end the launch through `on_exit`;
- a container starts with only its kept composables;
- contract findings and runtime enforcement on Excluded members reported as
  not in this run; scheduling tasks of Excluded members dropped;
- the selection in `run_info.json`, and in `play_launch.log` as one
  startup `info!` with the counts and the cut inputs.

### W3 - Bounded runs and the prefix

**Done (2026-11-24).** `commands/run_control.rs`; a failed verdict exits 4
(`VERDICT_EXIT_CODE`), apart from a launch that could not run (1). The rule
engine records its first error in any mode for `--fail-on contract`. Tests
`tests/tests/run_control.rs`.

- `--run-for <duration>`, `--fail-on health:<kind>[:<glob>] | contract |
  exit:<sel>` (repeatable), `--fail-fast`; the verdict as the exit code and
  in `run_info.json`;
- `--prefix <sel>=<command>`, replacing the resolved `launch_prefix` for
  this run.

### W4 - The Launch tree's switches

**Done (2026-11-24).** `web/selection.rs` (`GET /api/selection`,
`POST /api/selection/{check,apply,reload,export}`), `js/selection.js`,
switches and the pending bar in `LaunchTreeView.js`. Apply fresh stops
every running member and waits (at most 15 s) before starting the set
again. Reload hands a changed member a new spawn context
(`ControlEvent::ReplaceContext`, `ReplaceComposable`) that the next Apply
restarts it with; a member a Reload adds cannot be started without
restarting the launch, since actors are built once. Runtime enforcement
shares the set of topics whose publishers are off (`AbsentTopics`). Each
step line carries `before`, the resource monitor's totals for what was
running just before it, and a run that took steps ends with an `end` line.
`POST /api/nodes/:id/restart-with {prefix}` and the Info tab's **Restart
with prefix** restart one member under a prefix through the same
`ReplaceContext` path; a composable's goes on its container, refused under
`isolated`. Tests `tests/tests/launch_switches.rs`.

- switches on includes and nodes, pending edits, the pending summary;
- `GET /api/selection`, `POST /api/selection/check`,
  `POST /api/selection/apply` (live and fresh), `POST /api/selection/reload`;
- Restart with… (a prefix) on a node's page;
- Reload with changed marks (`launch` only; `up` has no launch files);
- the step record, `play_log/<ts>/selection.jsonl`;
- Copy command.

### W5 - Export

**Done (2026-11-24).** `ros_launch_resolve::verbs::export`,
`commands/export.rs`. The parser records the global parameters, remaps
and environment in effect at each include (`ScopeEntry.entry_context`),
and the include form writes them in front of the `<include>`: on Autoware's
planning simulator, `tier4_planning_component` and `tier4_map_component`
both stay one `<include>`. (Before, the map component was opened down to
its nodes, which missed the vehicle parameters set elsewhere.) The command
form takes one composable: a script that starts its container when kept
and loads it with `ros2 component load`. Tests
`tests/tests/launch_export.rs`, including stock `ros2 launch --print` on
each export and the composable script run to a loaded node.

- `play_launch export` with the command, self-contained and include-based
  forms; the header; `--again`; top-level arguments passed through;
- verification: the exported file re-resolved must give the selection's
  members, and the include form opens includes until it does;
- Export in the web UI, written into `play_log/<ts>/export/<name>/`, with a
  zip download.

## Not in this phase

`play_launch bisect`, a host partition, `meta.selection` in the model (needs
a `ros-launch-manifest` release), a filter box on the tree, a prefix for one
composable under `isolated`. Design section 11.
