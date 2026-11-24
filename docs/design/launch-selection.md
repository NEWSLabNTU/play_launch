# Selecting part of a launch tree

**Status:** exploring, revised 2026-11-24 from the use cases. Requested by
the amr 175 session (2026-11-21) as `--scope <pkg>/<file>`. Decisions are
listed in section 12. Writing a launch file for a selection is a separate
feature: `launch-extraction.md`.

## 1. The problem

A launch tree states one whole system. People routinely want less than the
whole, or the whole minus something, and ROS gives them one way to get it:
edit a launch file, adding a boolean argument and an `if=`. That only works
where the author foresaw the cut. Running the sub-file directly does not work
either, because it loses what its parents set: namespaces, the arguments the
includes passed, and global parameters.

play_launch is in a position nothing else is. It resolves the whole tree first,
and every node in the SystemModel already carries everything it was given:
- its namespace;
- its launch arguments, already applied;
- its parameters, global ones included;
- the scope it came from.

A subset is therefore a filter over a resolved model, not a re-parse. Nothing
a kept node receives changes when its neighbours are dropped: the user runs
the usual launch command, and parameters, contracts and scheduling apply as
in the full run.

## 2. Use cases

Ordered by how often we expect them. Each names what it needs; section 3
shows the workflow, and sections 5 to 9 the design.

**U1. Locate a fault by switching parts off.** The full launch shows a
symptom: a crash loop, an error burst, a planner that stops, latency. The
user switches off half the tree, watches whether the symptom stays, and
narrows down, component first, then the includes inside it, then single
nodes. Needs:
- a symptom judged the same way at every step: a Health issue;
- live steps (fast) and fresh steps, which restart every kept node. A live
  cut leaves state in the kept nodes (TF buffers, `transient_local` topics,
  lifecycle states, converged filters), and a bug caused by that state
  survives every live cut;
- "fixed" told apart from "starved": switching off a producer silences its
  consumers, so a symptom can disappear for a trivial reason;
- a record of the steps, to reproduce the search or hand it to a vendor;
- later, the same search run unattended.

**U2. Test one component repeatedly.** A sensor driver, run with exactly the
namespace and parameters the full tree gives it. The loop is: edit, rebuild,
restart, look; sometimes edit a launch or parameter file too. Needs:
- a subset started from the usual launch command;
- restarting one node (exists; a rebuilt binary is picked up from `install/`);
- re-reading edited launch and parameter files without restarting the run;
- the same pick tomorrow;
- a run with a verdict: "60 s, and fail if `/sensing/lidar/points` drops
  below 10 Hz", for the bench and for CI.

**U3. Debug one node under a tool, inside the full system.** gdb, valgrind,
perf, heaptrack. Needs: stop one node and start it again under a prefix with
its context unchanged.

**U4. Replace a part.** With the user's own build run from an IDE, with a
bag, or with a simulator. Needs:
- exclude the part;
- for an own build: the node's command line and parameter file
  (`launch-extraction.md`, the command form);
- for a bag: the list of topics the rest of the system needs from the cut
  part, ready for `ros2 bag`.

**U5. Ban a broken node.** The file may be a vendor's, read-only, or shared
with people who need the node. Needs: an exclusion shown where people look,
so a missing node in a bundle is not taken for a crash, and a way to keep it
(an export).

**U6. Bring up layer by layer.** On a new vehicle: sensing, check it, add
localization, check it, then planning, then control. Needs: add to a running
subset (Excluded is a state, 5.4), and a verdict per layer, which contracts
give.

**U7. Split one launch across hosts.** Sensing on the robot, the rest on a
workstation, without `host:=` arguments edited into the launch files
(`docs/guide/multi-host.md`). Needs: one selection per host, and a check
that the selections cover every node exactly once. Later (section 11):
`--only` on one host and the same `--exclude` on the other works from
phase 106 on, without the check.

**U8. Re-run part of a recorded run.** A bundle's model is the system as it
ran. Re-running only the part that misbehaved, from that model, reproduces
its context exactly, even after the launch files changed. Combines with U1
on a bench.

**U9. Check a launch-file edit.** The user edits a file deep in the tree and
wants to know what it now does. There is no "before" file, and a diff verb
was ruled out (2026-11-22). Needs: re-read the tree, keep the pick, and mark
the nodes whose resolution changed.

**U10. Measure what a part costs.** Perception on versus off: CPU, memory,
participants, discovery load. Needs: the resource figures per step, which U1's
step record can carry.

**U11. Reproduce or share a pick.** As a command, or as an exported launch
file (section 10).

## 3. Workflows

Decided 2026-11-24: two interfaces, the CLI and the web UI (no TUI), and
one way to make a pick permanent, an exported launch file
(`launch-extraction.md`). There is no selection file format.

### 3.1 On the command line

1. **Filter in place.** The usual launch command, with `--only` and
   `--exclude`, each repeatable:

   ```
   play_launch launch autoware_launch autoware.launch.xml <args> \
       --only tier4_sensing_component.launch.xml \
       --only tier4_localization_component.launch.xml \
       --exclude /sensing/camera/**
   ```

   The whole tree is resolved first; only the selected members start, the
   rest are registered as Excluded (5.4). `--only`s are a union, the
   `--exclude`s are removed from it, and the containers the kept
   composables need are added (5.2).
2. **Look first, if wanted.** The same flags on `resolve` with `--explain`
   print the marked tree, what each selector matched, the containers added,
   the inputs the kept nodes lack (with a `ros2 bag play --topics` line) and
   the warnings, and start nothing.
3. **Make it permanent: export.** `export` takes exactly what `launch` takes,
   plus the form and a directory:

   ```
   play_launch export autoware_launch autoware.launch.xml <args> \
       --only tier4_sensing_component.launch.xml --form include -o sensing_bench/
   play_launch launch sensing_bench/sensing_bench.launch.xml     # or: ros2 launch …
   ```

   When the original tree changes above what was exported, `play_launch
   export --again sensing_bench/sensing_bench.launch.xml` regenerates it
   from the command recorded in its header.

The bounded run (7.3) and the prefix (7.1) are flags on the same command:

```
play_launch launch … --exclude tier4_perception_component.launch.xml \
    --run-for 90s --fail-on health:crash_loop:/planning/**
play_launch launch … --prefix '/sensing/lidar/top/velodyne_driver=xterm -e gdb -ex run --args'
```

### 3.2 In the web UI

1. **Start with everything off**, if wanted: `play_launch launch … --start
   none`. Nothing starts early; the infrastructure (play_launch's own
   participant, monitoring, interception, Health) is up and the tree is
   shown. `--only`/`--exclude` pre-switch the tree instead.
2. **Pick on the Launch page.** Switch includes (a whole subtree) and single
   nodes on and off. The switches edit a pending set; beside the tree, the
   summary shows what will start and stop, the containers added, the cut
   inputs and the warnings.
3. **Apply.** The pending set is carried out in one batch: containers
   before their composables, `startup.order` stages respected, the memory
   admission gate applied. **Apply live** changes only what changed;
   **Apply fresh** restarts every kept member (6.1). **Discard** drops the
   pending edits.
4. **Keep changing it.** The same switches add and remove parts while the
   run goes. **Reload** re-reads the launch files and marks members whose
   resolution changed (7.2).
5. **Make it permanent: Export.** The Export dialog takes a form and a name.
   The server writes the export into the run's own log directory,
   `play_log/<ts>/export/<name>/`, never to a path sent from the browser,
   and the dialog shows:
   - the path on the host;
   - the command to run it, e.g. `play_launch launch
     play_log/<ts>/export/sensing_bench/sensing_bench.launch.xml`, and the
     `ros2 launch` equivalent;
   - a zip download, for another machine.

   **Copy command** gives the in-place form instead: the launch command with
   `--only`/`--exclude`.

### 3.3 The use cases on these workflows

- **U1, locate a fault:** the web flow with Apply fresh and the step record
  (7.4); on the CLI, one bounded run per step (7.3).
- **U2, test a sensor driver:** `--only` its include; restart the driver
  after a rebuild; Reload after a launch or parameter edit; `--run-for 60s
  --fail-on contract` as a test.
- **U3, debug under a tool:** `--prefix`, or **Restart with…** on the
  node's page (7.1).
- **U4, replace a part:** `--exclude` it; the cut inputs give the bag's
  topics; `export --form command` gives one node's command line for an own
  build.
- **U5, ban a node:** `--exclude` it; to keep the ban, export.
- **U6, layer by layer:** `--only` the first layer, then switch on the next
  and Apply live.
- **U8, from a bundle:** `play_launch up play_log/<ts>/system_model.yaml
  --only …`.
- **U9, check an edit:** Reload's changed marks.
- **U10, cost of a part:** two adjacent steps of the step record.

## 4. What exists today

- **Scopes.** Every node names its scope, and `structure.scopes` holds each
  include's package, file, namespace, arguments and parent. A file included
  more than once gets one scope per invocation, keyed `pkg/file#<id>`.
  - Autoware's planning simulator has 119 nodes in 83 scopes, at most 6
    includes deep.
  - Its `load_topic_state_monitor.launch.xml` is included 10 times.
- **Inspection.** `play_launch context <model> --tree | --node <fqn> |
  --launch <pkg> <file> [--all]` prints the tree and one node's or one
  include's context. It reads; it does not select.
- **Web UI.** The Launch page shows the same tree, and per-node Start, Stop
  and Restart exist, as do per-composable load and unload and a container's
  auto-load toggle.
- **Selecting nodes by glob.** `startup.order` and `cgroups.limits` both
  match node FQNs with `glob` patterns. A third matcher should be the same
  one.
- **The dependency facts the model has:**
  - a composable's container (`container:`);
  - the topic graph derived from remaps: on Autoware, 256 of 269 inferred
    edges were confirmed against a run and none contradicted (phase 77);
  - contracts, where a launch has them.
- **Per-node launch prefix.** `launch_prefix` is resolved per node and
  carried on the model (#0087).
- **Verdict sources.** Health issues (journalled to `health.jsonl`) and
  runtime enforcement's contract findings exist; nothing turns either into
  an exit code, and no run ends by itself after a time.
- **Diffing.** `scripts/compare_models.py` compares two models for parser
  parity. It is a developer script, not a user verb.

### 4.1 Prior art

The prior-art search found nothing in ROS that:
- runs a resolved subset of a launch tree;
- excludes a node from an included file without editing it;
- diffs two resolutions.

What people do instead:
- add a boolean argument with a condition: Autoware's `launch_perception:=false`
  and its finer `launch_*` flags, Nav2's `use_localization` and `slam`;
- gate an `<include>` with `<group if=…>`;
- copy and edit the file.

The nearest tools:
- **ROS 1 `roslaunch --args <node> file | bash`** runs one node with its
  resolved command line, which is U4's own-build case for a single node.
- **rqt_launch (ROS 1), rosmon, better_launch and rosmon2** start and stop
  nodes one by one. rosmon has `--no-start`, which is U6's mechanism.
- **ros2launch_gui** draws the tree and selects nothing.
- **launch-plus** resolves trees to select packages to build, not to launch.
- **`launch-prefix="xterm -e gdb --args"`** is the usual ROS way to debug a
  node, written into the launch file (U3 without editing it).

Outside ROS:
- **systemd** has `isolate` (start a unit and its dependencies, stop the
  rest) and `mask` (a unit cannot be started, without editing it).
- **docker compose** `up <svc>` starts a service and its dependencies;
  `--no-deps` opts out; profiles name subsets.
- **Bazel** target patterns take `-//pkg/x` as an exclusion and have
  `deps()`/`rdeps()` queries.
- **git bisect** and delta debugging (`ddmin`) are the shape of U1's
  unattended search.

## 5. The selection

### 5.1 One selection, over the model

A selection is a list of `only` and `exclude` selectors, evaluated in that
order over `structure.nodes`:
1. Start from everything, or from the union of the `only`s when there are
   any.
2. Remove the excludes.
3. Close over hard dependencies.

The result is the set of kept members, over a model that stays whole: the
excluded members remain in it, registered as Excluded (5.4), so the web UI
can show and start them. `launch` and `up` run it; `export` writes it as a
launch file; `resolve --explain` prints it.

The logic lives in `ros_launch_resolve::selection`, beside the shared verbs;
the CLI flags, the web UI's switches and `export` call the same code.

### 5.2 Dependencies: hard ones closed, soft ones reported

- **Hard (closed by default):** a kept composable keeps its container. Under
  `observable` the container starts and loads only the kept composables. On
  Autoware, 13 of the 70 composables are declared in a different include
  from their container. `--no-deps` opts out.
- **Soft (reported, never closed):** what the topic graph says a kept node
  consumes and no kept node produces, e.g. "`/sensing/lidar/points` is
  subscribed by 3 kept nodes and published only by excluded
  `/sensing/lidar/driver`". Reported, not added, because a subset is often
  fed from elsewhere (a bag, a simulator, another host), and the graph is
  inferred from remaps and errs toward more edges. The same list is:
  - the "cut inputs" of a step (U1's starvation check, 7.4);
  - the topic list for a bag (U4), with types where the run knows them (the
    interceptor's endpoints in a running launch; contracts at resolve time;
    a remap-derived topic has a name only).
- **Known traps with a named warning:**
  - a lifecycle manager whose `node_names` lists an excluded node (Nav2's
    aborts bringup);
  - `use_sim_time: true` on kept nodes with the `/clock` publisher excluded;
  - a kept subscriber of `/tf` with every TF publisher excluded.

### 5.3 Where a selection is written

- **Flags:** `--only <sel>` and `--exclude <sel>`, repeatable, and
  `--no-deps`, on `launch`, `up` and `export`; `--start none` (everything
  Excluded, for picking in the web UI) on `launch` and `up`. `resolve` and
  `context --tree` take the selection flags with `--explain` only: they print
  the marked tree and the reports and write no model, since a partial model
  cannot yet say it is partial (`meta.selection`, section 11).
- **Web UI:** the Launch tree's switches (section 6).
- **Nowhere else.** A pick is kept by exporting it (section 10).

### 5.4 Excluded is a state, not an absence

An excluded member is registered as **Excluded**: shown greyed out on the
Nodes and Launch pages, startable from there. That one mechanism gives U6,
makes a ban visible (U5), and answers "why is `/x` not running?" in a
bundle. An Excluded member:
- is not respawned; `--composable-respawn` and the actor's respawn logic
  treat it like a user stop;
- cannot end the launch through `on_exit=Shutdown()` (#0063);
- is not counted by startup completion, `never_ready` or `missing_node`;
- counts in no `startup.order` stage, and an empty stage passes.

The selection is recorded:
- in `run_info.json` and `play_launch.log`;
- as one `info!` at startup naming how many members were excluded, by which
  selectors, and the cut inputs.

### 5.5 Contracts and scheduling over a subset

Parameters are unaffected by construction. Contracts and the schedule are
resolved over the whole tree, then:
- a contract finding, or a runtime enforcement rule, that names an Excluded
  member is reported as "not in this run", never as a violation or a graph
  deviation;
- execution-layer tasks that place only Excluded nodes are dropped, and a
  reservation's admission arithmetic is redone for the kept set.

## 6. The tree's switches

Decided 2026-11-24: no separate plan mode. The Launch page's tree gets an
on/off switch per include and per node.

### 6.1 Flow

A launch starts all on (no flag), all off (`--start none`) or as a subset
(`--only`/`--exclude`). Then:
1. A click toggles an include (its whole subtree) or a node. Clicks edit a
   **pending** set; nothing starts or stops yet.
2. Beside the tree, the pending change is summarised: members to start and
   stop, containers the closure adds, cut inputs, and the warnings of 5.2.
3. **Apply** carries it out, in one of two modes:
   - **live:** start and stop only what changed (U6);
   - **fresh:** stop every running member and start the new set from
     nothing (U1).
   **Discard** returns the switches to what is running.

Every click is pending because one click on
`tier4_perception_component.launch.xml` stops 38 nodes, and the warnings are
worth reading before the change.

### 6.2 What Apply does

- **switches on:** starts containers before their composables, and loads a
  composable into its container when the container already runs;
- **switches off:** unloads composables before their container stops, stops
  members, and marks each Excluded;
- **closure:** a composable switched on brings its container on, shown as
  added; switching a container off while composables in it stay on asks
  first (the `--no-deps` case), and is never silently undone;
- **fresh:** a stop of everything, then a start in startup order, so
  `startup.order` stages apply again. A stop by Apply is a user stop: it
  opens no `down` or `crash_loop` issue.

### 6.3 The tree

```
[~] autoware_launch planning_simulator.launch.xml          41/119
  [~] autoware_launch autoware.launch.xml
    [x] tier4_map_component.launch.xml                     7/7
    [ ] tier4_perception_component.launch.xml              0/38   pending: on
    [~] tier4_system_component.launch.xml                  9/24
      [x] component_state_monitor.launch.py
        [x] load_topic_state_monitor.launch.xml  vector_map
        [ ] load_topic_state_monitor.launch.xml  initialpose3d   changed
        ...
    [+] /pointcloud_container      (needed by 3 selected composables)
Pending: start 38, stop 0         [Apply live] [Apply fresh] [Discard]
Cut inputs
  /sensing/lidar/concatenated/pointcloud: 4 kept subscribers, publisher excluded
Warnings
  use_sim_time on 41 nodes, /clock publisher excluded
```

- **States:** `[x]` on, `[ ]` off (Excluded), `[~]` partly on, `[+]` on
  because the closure needs it. `pending:` marks an edit not yet applied;
  `changed` marks a member whose resolution changed at the last Reload.
- **Repeated includes** are labelled by what tells them apart: the argument
  that differs (`vector_map`), else the parent. Never `#22`.
- **Detail panel:** the Launch page's existing one, for the include's
  arguments and the node's parameters.

### 6.4 Without a browser

`play_launch resolve … --only … --exclude … --explain` (or `context --tree`
with the same flags) prints the same marked tree, the cut inputs with the
`ros2 bag` line, and the warnings, and exits.

## 7. Supporting features

These came out of the use cases. Each is usable without a selection.

### 7.1 A launch prefix per run (U3)

`--prefix <sel>=<command>`, repeatable, and **Restart with…** on a node's
page. It replaces the node's resolved `launch_prefix` for this run, in front
of argv as `launch_prefix` already goes (after container replacement, #0087).
- A composable has no process of its own under `observable`: the prefix
  goes on its container, and the UI says so. Under `isolated`, prefixing
  one composable needs the container to wrap its `component_node` exec; that
  is later.
- The prefix is recorded in `run_info.json` and the node's `cmdline`.

### 7.2 Reload (U2, U9)

**Reload** on the Launch page (and `POST /api/selection/reload`) re-resolves
the tree from the files as they are now, with the same arguments, and
re-applies the selectors to the new tree:
- a member whose resolution changed (command line, parameters, environment,
  prefix) is marked `changed` and is restarted by the next Apply, live or
  fresh; nothing restarts on Reload alone;
- a new member is switched on if a selector covers it, as a pending edit;
- a member that no longer exists is pending stop.

This settles U9 without a diff verb: the changed marks are the comparison.

### 7.3 A bounded run with a verdict (U1, U2, U6, CI)

- `--run-for <duration>` ends the launch with the usual shutdown after the
  time.
- `--fail-on <condition>`, repeatable:
  - `health:<kind>[:<subject-glob>]`: a Health issue of that kind opened;
  - `contract`: a runtime enforcement finding on a kept member;
  - `exit:<node glob>`: a matching member exited other than by a stop.
- The exit code is 0 when no condition held, and 4 when one did (one code
  for every condition; the condition is named in the output);
  the condition, the time and the subject are printed and written to
  `run_info.json`.
- `--fail-fast` ends the run at the first condition rather than at
  `--run-for` (U1's unattended search).

### 7.4 The step record (U1, U10)

Each Apply and each Reload appends one line to
`play_log/<ts>/selection.jsonl`:
- time, mode (live/fresh/reload), the selection in canonical form, what
  started and stopped;
- the cut inputs of the kept set;
- when the step ends (at the next Apply or the end of the run): the Health
  issues open during it, whether each issue's subject had a cut input, and
  CPU and memory totals of the kept set over the step.

That is the evidence for a fault search, and the on/off cost comparison of
U10 falls out of two adjacent lines.

## 8. Pitfalls this has to survive

- **State left by a live cut.** The reason fresh Apply exists (6.1); the
  step record says which mode each step used.
- **Starvation read as a fix.** Covered by the cut inputs on each step.
- **Selecting a composable but not its siblings.** Under `isolated` that is
  free. Under `observable`, the kept composables share a process with
  nothing else, which changes the timing the user may be debugging.
- **An excluded node that a kept node calls as a service.** The graph knows
  services only as well as remaps name them, so some of these will be
  reported and some will not.
- **Excluding a container while including its composables.** An error, not
  a silent re-add (5.1).
- **A prefix that never execs the node** (a typo, an `xterm` that cannot
  open a display). It surfaces as a start failure in the node's `err`, as any
  spawn failure does.

## 9. Selector syntax

### 9.1 What must be nameable

From the use cases, measured on Autoware's planning simulator:

1. **An include and everything under it:** `tier4_perception_component`.
2. **One node:** `/rviz2`.
3. **A namespace:** everything in `/perception`.
4. **One of several invocations of a file.** There are 10 invocations of
   `load_topic_state_monitor.launch.xml`, which differ by their
   `node_name_suffix` argument, and 4 of `global_params.launch.py`, which
   differ by parent. None of them differs by namespace.
5. **What runs:** every composable of plugin `X`, every node of package `P`.
6. **A set minus a part:** `/perception` except one node.

### 9.2 The stability problem

A scope id (`#22`) is traversal order. Adding one include earlier in the tree
renumbers every later one, so a recorded `--exclude scope:…#22` silently bans
a different invocation after an unrelated edit. An export's header, a copied
command and Reload are exactly the cases that outlive one tree, so ids are
unfit for anything recorded.

The stable distinguishers are the two the model already has: the include's
arguments, and its parent.

### 9.3 Three syntaxes, the same six tasks

**A. Prefixed kinds** (`kind:pattern`, an attribute predicate for repeats):

| Task | Selector |
|---|---|
| 1 | `scope:autoware_launch/tier4_perception_component.launch.xml` |
| 2 | `node:/rviz2` |
| 3 | `node:/perception/**` |
| 4 | `scope:autoware_topic_state_monitor/load_topic_state_monitor.launch.xml[node_name_suffix=vector_map]` |
| 5 | `plugin:autoware::pointcloud_preprocessor::*`, `pkg:autoware_lidar_*` |
| 6 | `--only node:/perception/** --exclude node:/perception/obstacle_segmentation/**` |

**B. Bare, kind inferred by shape:** a leading `/` is a node FQN, a
launch-file suffix is an include, a file name matching several packages'
files is an error listing them. Shortest for tasks 1, 2, 3 and 6; cannot say
task 5.

**C. A small query language** (`file=…,arg.x=…,ns=…`): the most general,
and a language to document, quote and keep that no use case needs.

### 9.4 Decision (2026-11-22)

**A is the language, with B's two shorthands accepted on input:**
- `/…` is short for `node:/…`;
- a launch-file name is short for `scope:…`.

`pkg:`, `exec:`, `plugin:` and `container:` stay prefixed. A launch file is
named by its file name, or `<pkg>/<file>` when the name exists in more than
one package (a bare name that matches several is an error listing them).

**A file included more than once** (2026-11-24) is selected only when the
selector says which inclusions it means; a bare name for it is an error
listing the choices, already disambiguated:

```
error: `load_topic_state_monitor.launch.xml` is included 10 times; say which:
  scope:autoware_topic_state_monitor/load_topic_state_monitor.launch.xml[node_name_suffix=vector_map]
  scope:autoware_topic_state_monitor/load_topic_state_monitor.launch.xml[node_name_suffix=initialpose3d]
  ...
  or [*] for all 10
```

- `[<arg>=<value>]` matches the inclusions passed that argument value;
- `[under=<pkg>/<file>]` matches the inclusions with that file among their
  ancestors;
- `[nth=<n>]` is the n-th inclusion of the file by the same parent, for
  inclusions with the same arguments and parent; accepted with a warning,
  since editing that parent can shift it;
- `[*]` matches every inclusion; it is never the default, because matching
  all of them silently would turn "exclude the `vector_map` monitor" into
  "exclude all ten" with a plausible-looking run.

Predicates combine (`[under=a/b.launch.xml][mode=x]`). `#id` (as `context
--all` prints it) is accepted on the command line, where it is copied and
used at once; it is never written by play_launch (an export's header and
Copy command use the stable form).

- **Output uses the canonical prefixed form:** `--explain`, Copy command,
  an export's header, `run_info.json`, `selection.jsonl`.
- **The UI writes selectors, not ids.** A switch is written as the most specific
  stable selector: the include by name if unique; else with `[arg=…]` on the
  argument that differs; else with `[under=…]`.

Patterns use the `glob` crate of `startup.order` and `cgroups.limits`: in an
FQN, `*` stays inside one segment and `**` crosses segments. A `scope:`
selector covers its whole subtree; `exclude` is applied after `only`; the
closure runs last, and a closure that would bring back something explicitly
excluded is an error.

## 10. Reproducing a selection

- **In place:** the launch command with `--only`/`--exclude` in canonical
  form (**Copy command** in the web UI). The selectors are stable (9.2), so
  the command means the same after the tree is edited, as long as what it
  names exists; when it does not, the launch fails naming the selector.
- **Permanent:** an exported launch file, self-contained or include-based
  (`launch-extraction.md`). There is no selection file format.

## 11. Phasing

Implemented as phase 106 (`docs/roadmap/phase-106-launch-selection.md`):
- **W1, the selection:** selector parsing, evaluation over the model, the
  repeated-include rule, the closure, cut inputs and the three named traps,
  and `resolve --explain`.
- **W2, a run with a selection:** `--only`, `--exclude`, `--no-deps` and
  `--start none` on `launch` and `up`; the Excluded state; containers that
  load only their kept composables; Health, startup, respawn and `on_exit`
  over a subset; contracts and scheduling over a subset (5.5); the
  selection in `run_info.json` and `play_launch.log`.
- **W3, bounded runs and the prefix:** `--run-for`, `--fail-on`,
  `--fail-fast`, `--prefix`.
- **W4, the tree:** switches, pending edits, Apply live and fresh, Discard,
  Restart with…, Reload, the step record, Copy command.
- **W5, export:** `play_launch export` in three forms, its verification,
  `--again`, and Export in the web UI (`launch-extraction.md`).

**Later:**
- `play_launch bisect`: U1 unattended, over fresh bounded runs;
- a host partition (U7);
- topic types in the bag line from a running launch;
- a filter box in the selector language on the tree;
- a prefix for one composable under `isolated`;
- `meta.selection` in the model, which needs a `ros-launch-manifest`
  release; until then the selection is recorded in `run_info.json` and the
  log.

## 12. Decisions and open questions

Settled (2026-11-22 unless dated):
- excluded is a state (5.4);
- dependency closure on by default (5.2);
- no `diff` verb; Reload's changed marks are the comparison (7.2,
  2026-11-24);
- selector syntax: A with B's two shorthands (9.4);
- a file included more than once is selected only with `[arg=…]`,
  `[under=…]`, `[nth=…]` or `[*]`; a bare name for it is an error listing
  the choices (9.4, 2026-11-24);
- the user runs the usual launch command with a filter; the whole tree is
  resolved first, so parameters and contracts apply as in the full run
  (5.1, 5.5; 2026-11-23);
- no separate plan mode: tree switches with pending edits and Apply, plus
  `--start none` (section 6, 2026-11-24);
- two interfaces, CLI and web UI; no TUI (2026-11-24);
- a pick is made permanent by exporting a launch file; no selection file
  and no `selection:` config section (2026-11-24);
- writing a launch file is `play_launch export` (`launch-extraction.md`,
  2026-11-24).

- `--fail-on` exits with one code, 4, for every condition, and names the
  condition (2026-11-24);
- a topic whose publishers are all switched off is "not in this run" from
  the switch on: its silence detects no hazard and holds no level to
  account, so `--fail-on contract` cannot fire on it (2026-11-24).
