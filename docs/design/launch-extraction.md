# Extracting a launch file from a launch tree

**Status:** implemented 2026-11-24 (phase 106 W5). The include form
recreates the global parameters, remaps and environment in effect at an
include point (4.2), and the command form takes a composable node (6.1).
Split from
`launch-selection.md`, where it was the "generated launch file". Selecting
what to run and writing a launch file for it are separate features: this one
writes a file and runs nothing. It is also how a selection is made permanent:
there is no selection file format (`launch-selection.md` section 3).

## 1. The problem

A user picks part of a launch tree, some includes and some single nodes, and
wants a launch file for exactly that part. The file must start the same nodes
with the same names, namespaces, parameters and remaps they get in the whole
tree, and stock `ros2 launch` must be able to run it.

Running a sub-file directly does not do this, for the reason given in
`launch-selection.md` section 1: it loses what its parents set. play_launch
resolves the whole tree first, so it knows that context for every node and
every include, and can write it down.

Use cases (the U numbers are `launch-selection.md`'s):
- **run one node outside play_launch** (U3, U4): the user's own build from
  an IDE, or a node under a debugger the IDE drives, with the command line
  and parameter file the tree gives it;
- **test a subsystem in CI** (U2): an include-based file that CI runs with
  stock `ros2 launch`, or with play_launch and `--fail-on`, and that follows
  the vendor tree as it changes;
- **hand a subsystem over** to someone who runs stock `ros2 launch`;
- **freeze a known-good configuration** on a bench, unaffected by later
  edits to the tree;
- **attach the exact nodes involved to a bug report** (U1's result), as a
  file the vendor can run without the reporter's whole tree.

## 2. Three forms

| | Command | Self-contained | Include-based |
|---|---|---|---|
| What it is | one node's command line and its parameter file | one flat `.launch.xml`, one element per node, plus one parameter file per node | a `.launch.xml` holding `<include>`s of the selected original files, and flat elements only for nodes picked singly |
| Follows later edits to the included files | no | no: a snapshot | yes, inside each included file |
| Needs the original packages installed | the node's package | the nodes' packages; parameter files are written out | yes |
| Readable as the original structure | n/a | no | yes |
| Good for | an own build or a debugger run of one node | a frozen configuration; a bug report | CI; a bench launch kept in step with the vendor tree; handing a subsystem over |

The command form is the self-contained form for one node, written as a
shell command (`ros2 run`'s `--ros-args` with `-r`,
`--params-file`, `__ns`, `__node`) instead of XML, with the environment the
node gets beyond the caller's as `env` assignments. A composable has no
command of its own; for one, the form writes `component_node`'s, as the
`isolated` container would run it.

Both are produced from the same selection, by the same evaluator as
`--only`/`--exclude` (`launch-selection.md` 5.1, 9.4), and the file forms are verified
the same way (section 5).

## 3. The self-contained form

Generated from the subset model, one element per member, with nothing to
evaluate:
- `<node>` with `pkg`, `exec`, `name`, `namespace`, `args`,
  `launch-prefix`, `cwd`, `respawn`, `respawn_delay`, `on_exit` and `<env>`;
- `<node_container>` with its `<composable_node>`s and their `<extra_arg>`s;
- `<executable>` for a raw command;
- `<timer period=…>` around a delayed member.

Parameters go into one YAML file per member next to the launch file, as
`<param from=…>`, never as inline `<param value=…>`. The model's values are
already YAML-typed text (#0067, #0068), so a file carries them as they are,
while an inline XML value would be re-typed by launch's own rules. A member's
original parameter files are written as their content, already resolved.
Remaps become `<remap>`, global remaps included, since the model has them per
node.

It needs only the model, so it can be extracted from a recorded run's
`system_model.yaml` as well as from a launch file.

### 3.1 What XML cannot say

Read from the installed Humble `launch`/`launch_ros` parsers:
- **`sigterm_timeout`/`sigkill_timeout` per node.** XML has no attribute for
  them; a launch argument of that name sets them for every process.
- **`on_exit` other than `shutdown`.** XML takes only `shutdown`. A node that
  shuts the launch down on some exit codes only (#0084) cannot be written.
- **An unset environment variable for one node.** XML has a scope-wide
  `<unset_env>` and no per-node form; it needs a `<group>` around the node,
  which is still to be confirmed to scope the environment on Humble.
- **Lifecycle autostart.** The `<lifecycle_node autostart>` attribute exists
  from Jazzy, not on Humble.

**Decided (2026-11-22):** XML is generated anyway, with the lost field in a
comment on the member and in the list the export prints. How rare these
fields are is not measured yet; the Autoware model at hand predates them, so
counting needs a fresh resolve.

## 4. The include-based form

### 4.1 Which parts become an `<include>`

Walk the scope tree from the root:
- an include whose whole subtree is selected is written as **one
  `<include>`** of its original file;
- an include that is partly selected is **opened**: its selected
  sub-includes are handled by the same rule, and its selected nodes outside
  them are written flat, as in section 3;
- an include with nothing selected is left out.

An include cannot be written as `<include>` with some of its nodes removed:
the original file starts all of them. That is why partly selected includes
are opened. Opening goes only as deep as the selection requires, so picking
`tier4_sensing_component.launch.xml` whole gives one `<include>`, and
excluding one node in it gives that file's other children instead.

A composable's container follows the hard-dependency rule
(`launch-selection.md` 5.2). When the container's include is not wholly
selected, the container is written flat, under the name the composables'
`target_container` already uses.

### 4.2 What an `<include>` has to carry

An included file sees everything in effect where it was included, not only
its arguments, since an include does not scope (CLAUDE.md, "An `<include>`
does not scope launch configurations"). To start the same nodes, the
generated `<include>` has to recreate:

| Context at the include point | Model today | Written as |
|---|---|---|
| the file | `ScopeInfo.package` and `.file` (basename only); the in-memory dump has the full path | `$(find-pkg-share <pkg>)/<path relative to share>` |
| the arguments the include passed | `ScopeInfo.args`, resolved | `<arg name=… value=…/>` |
| the namespace | `ScopeInfo.ns` | `<group><push-ros-namespace namespace=…/>` around it |
| launch configurations set by ancestors and earlier siblings, not passed as arguments | the dump's configuration log, through each scope's `config_span` (phase 94); not in the model | `<let>` before the include, for each name the file reads |
| global parameters (`<set_parameter>`) | per node; per scope in the dump (`entry_context`) | `<set_parameter>` before the include |
| global remaps (`<set_remap>`) | per node; per scope in the dump (`entry_context`) | `<set_remap>` before the include |
| environment (`<set_env>`, `<unset_env>`) | per node (`env`, `env_unset`); per scope in the dump (`entry_context`) | `<set_env>`/`<unset_env>` in the group |

Two consequences:
- the include-based form needs the in-memory dump, so it is extracted from a
  launch file (`launch`, `resolve`), not from a saved model, unless the
  model gains a per-scope context record;
- global parameters, remaps and environment have to be recorded per scope,
  as they are in effect at the include point, rather than reconstructed from
  the nodes inside, which may have changed them themselves. The Rust
  parser records them on each include's scope (`ScopeEntry.entry_context`,
  taken where the file is included); the Python dumper does not, so an
  include it resolved that depends on them is still written one level
  down. On Autoware's planning simulator, `tier4_map_component` and
  `tier4_planning_component` both stay one `<include>`, each preceded by
  the vehicle parameters set before it.

**Top-level arguments are passed through** (decided 2026-11-24). The
generated file declares each argument of the original command as its own
`<arg>`, with the value used as the default, so `map_path:=…` can still be
changed when the exported file is launched. An include argument whose
resolved value is exactly a top-level argument's value and whose source
expression is that `$(var …)` is written as `$(var <name>)`; anything
computed is written as its resolved value, frozen. Without the source
expression (a model without the dump) every value is frozen, and the export
says so.

### 4.3 Effects that cross between siblings

Because includes do not scope, an included file can set a configuration that
a later sibling reads. When the earlier sibling is left out, the later one
reads something else. The `<let>`s of 4.2 cover what each include reads, so
this is handled as long as the configuration log names the reads; section 5
catches what it misses.

## 5. Verification

The parsers are verified this way already: resolving the generated file must
give the same model as the selection does, compared with the parity
comparison of `scripts/compare_models.py`. Stock `ros2 launch` on the file is
the second check, in tests.

For the include-based form the check also runs at export time. An `<include>`
whose nodes come out different from the selection is opened (4.1), recursing
until the result matches, and the export names each include it had to open
and the first difference. The worst case is the self-contained form, which
always matches by construction. So the export never writes a file that starts
something other than what was picked; it only loses some of the following of
later edits.

## 6. Where it is invoked

### 6.1 The CLI

```
play_launch export <pkg> <file> [args] [--only …] [--exclude …] [--no-deps] \
    --form command|self-contained|include -o <dir> [--name <name>]
play_launch export --model <system_model.yaml> [--only …] … --form self-contained -o <dir>
play_launch export --again <dir>/<name>.launch.xml
```

`export` takes what `launch` takes, so turning an in-place filter into a file
means replacing the verb. It writes `<dir>/<name>.launch.xml` (and, for the
self-contained form, `<dir>/params/<member>.yaml`), and prints:
- the command to run it, with play_launch and with `ros2 launch`;
- the fields XML could not say (3.1);
- the includes it had to open (5), with the first difference for each.

The command form prints one node's command line and writes its parameter
file; it takes exactly one plain node or container. For one composable it
writes a script that loads it with `ros2 component load` (name, namespace,
each resolved parameter as `-p`, remaps, extra arguments as `-e`). When the
selection keeps its container (the closure does unless `--no-deps`), the
script starts the container first and retries the load until the container
answers; with `--no-deps`, it loads into a container already running.

### 6.2 The header

Every exported launch file starts with a comment recording how it was made:

```xml
<!-- Exported by play_launch 0.42.0 on 2026-11-24 (form: include).
     Source: play_launch export autoware_launch autoware.launch.xml map_path:=… \
             --only scope:autoware_launch/tier4_sensing_component.launch.xml
     Regenerate: play_launch export --again <this file> -->
```

`--again` reads the source command from it and runs it again with the same
form, name and directory. The header is a convenience: launching the file
never reads it, and `ros2 launch` ignores it.

### 6.3 The web UI

The Launch page's **Export** dialog takes the form and a name. The server
writes into the run's own log directory, `play_log/<ts>/export/<name>/`, and
never to a path sent from the browser, which the origin guard and token are
there to prevent. The dialog shows the path on the host, the commands to run
it, what could not be said and what was opened, and offers a zip download
for another machine. A node's page offers the command form as **Copy
command**.

## 7. Questions to settle

Settled 2026-11-24:
- the verb is `export`;
- top-level arguments are passed through as `<arg>`s (4.2);
- the web UI exports into the run's log directory, plus a zip (6.3);
- `--again` regenerates from the header (6.2).

Open:
1. Generated XML only, or `.launch.py` when a member needs what XML cannot
   say (3.1)? Decided XML for the self-contained form; the include-based form
   follows it unless a case shows otherwise.
2. Per-scope context in the model (4.2): add it, so a saved model can also
   be exported in the include-based form, or keep that form to launch files?
