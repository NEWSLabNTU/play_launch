# P46 W6 Report — docs sweep for the unified SystemModel (46.6, closes Phase 46)

Branch `docs/p46-sweep`, created fresh from `origin/main` (`bc28866`). Not pushed.
Docs + help-strings only — no logic changed (the one code file touched,
`src/play_launch/src/cli/options.rs`, is help-text/doc-comment edits only;
`cargo build`/`cargo test -p play_launch` confirmed unchanged behavior).

## Files touched

- `CLAUDE.md` — project overview, "Launch File Parsing" model-parity note,
  "Launch Tree Scoping" scope-table + web-UI-degrade note, "Architecture"
  execution-flow line, "Key Recent Changes" (new 2026-07-20 Phase 46 entry),
  "Documentation" design-doc list (added `system-model.md` /
  `unified-system-model.md`).
- `docs/guide/rt-scheduling.md` — new "One artifact (Phase 46)" callout near
  the top; §2.2 "Resolution" step 1 rewritten off `record.json`-as-artifact;
  troubleshooting-table fix (added the `dump --format record launch <pkg>
  <file>` command needed to actually produce a `record.json` for the
  `context --tree` tip, since `dump`'s default no longer writes one).
- `docs/guide/cli-interface.md` — substantial rewrite: dump defaults/format
  flag, Replay Only section (`--model` primary, `--input-file` deprecated),
  Common Options (`--model`/`--input-file` split), the "Automatic
  Dump-Resolve-Replay" workflow explanation (dropped the "bound pair"/
  cryptographic-binding language that no longer exists), "three-verb split"
  table, and the "Manual Workflow" examples (model-first + a clearly-marked
  legacy record.json path).
- `README.md` — one-line fix: top-level command reference's
  `play_launch replay [--input-file record.json]` → `[--model
  system_model.yaml]`.
- `docs/design/unified-system-model.md` — Status header flipped
  Approved/pending → SHIPPED, with a pointer to the phase doc + W2–W6
  reports; "Migration"/"Retirement" sections' numbered steps marked ✅ done;
  fixed the `file_data`/`variables` "drop" language to "confirmed nothing to
  drop" (matches what actually happened per the W5 report — the model was
  never built by embedding a `LaunchDump`, so there was nothing there to
  begin with); fixed "The artifact and commands" section's syntax
  (`play_launch replay <model.yaml>` isn't valid CLI syntax — there's no
  positional model arg, only `--model`).
- `docs/roadmap/phase-46-unified_system_model.md` — Status → ✅ Complete;
  every work item (46.0–46.6) marked done with brief evidence pointers;
  46.5's `file_data`/variables bullet corrected to "confirmed, not dropped";
  "Order and dependencies"/"Retirement" recap sections tense-corrected.
- `docs/roadmap/README.md` — Phase 46 table row flipped 📋 Planned → ✅
  46.1–46.6 (2026-07-20); added a "Phase 46" narrative section (the other
  complete phases all have one; 46 didn't).
- `docs/roadmap/phase-43-runtime_consumes_system_model.md` — added a
  "Superseded (2026-07-20)" note pointing at Phase 46/the design doc, so a
  reader landing directly on the Phase 43 doc doesn't take its two-artifact
  model at face value.
- `src/play_launch/src/cli/options.rs` — help-text only:
  - Top-level `after_help`: `dump --output autoware.json` → `dump --output
    autoware.yaml` (SystemModel default), `replay --input-file record.json`
    → `replay --model autoware.yaml`.
  - `Replay` subcommand doc comment + `after_help`: now leads with `--model`
    examples, keeps one clearly-labeled deprecated `--input-file` example.
  - `Resolve` subcommand `after_help`'s second example: reordered
    `--sched system.posix.yaml` before the `mode:=velodyne` launch argument
    (see "flag-ordering bug" below — the original order silently dropped
    `--sched`).
  - Also fixed the SAME flag-ordering bug in the top-level example (moved
    `--output` before `launch`).

## Command-verification evidence (built binary, `just build-rust`,
rt_workspace fixture + real Autoware 1.5.0 where noted)

All runs against `install/play_launch/lib/play_launch/play_launch` built
from this branch (`just build-rust`, ROS Humble + this worktree's
`install/setup.bash` sourced). PGID-killed every spawned process; confirmed
clean afterward (`ps aux` empty).

**dump → model (default), no record companion:**
```
$ play_launch dump -o m.yaml launch rt_demo bringup.launch.xml
SystemModel: m.yaml (4 nodes, 3 topics, 5 contracts-carrying endpoints, 3 tier(s), 2 warning(s))
To replay: play_launch replay --model m.yaml
$ ls m.record.json
ls: cannot access 'm.record.json': No such file or directory
```

**replay --model — spawns cleanly, no companion record present:**
```
$ play_launch replay --model m.yaml --disable-web-ui --disable-monitoring --disable-diagnostics
Spawn source: SystemModel (structure.nodes)
Spawning 4 nodes (2 pure nodes, 1 containers, 1 composable nodes)
Startup complete: all nodes ready (nodes 2/2, containers 1/1, composable 1/1)
```
4 processes (play_launch + control_node + sensor_node + perception_container)
confirmed via `ps aux`; SIGTERM via PGID shut down cleanly, all gone.

**dump --format record (deprecated escape hatch):**
```
$ play_launch dump -o record.json --format record launch rt_demo bringup.launch.xml
Dump completed successfully: record.json
To replay (deprecated): play_launch replay --input-file record.json
$ python3 -c "import json; print(list(json.load(open('record.json')).keys())[:10])"
['container', 'file_data', 'lifecycle_node', 'load_node', 'node', 'scopes']
```

**replay --input-file (legacy, no --model) — warns, still spawns 4/4:**
```
$ play_launch replay --input-file record.json --disable-web-ui --disable-monitoring --disable-diagnostics
WARN play_launch::commands::replay: record.json replay is deprecated; use the SystemModel
     (`play_launch resolve`/`dump` then `replay --model <system_model.yaml>`). Continuing
     with the legacy --input-file record.json path.
Spawning 4 nodes (2 pure nodes, 1 containers, 1 composable nodes)
```

**Autoware 1.5.0, real install (dump default → model, 119 nodes):**
```
$ play_launch dump --output autoware.yaml launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning
SystemModel: autoware.yaml (119 nodes, 0 topics, 0 contracts-carrying endpoints, 0 tier(s), 0 warning(s))
```
(Full replay of the 119-node model was not re-run this wave — heavy/GPU;
the W5 report already exercised replay-from-a-119-node-model. This wave's
Autoware run verified dump/resolve output shape and CLI flag ordering only.)

**Python parser → same complete model as Rust (model-parity claim in
CLAUDE.md), rt_workspace:**
```
$ export PYTHONPATH=<repo>/python:$PYTHONPATH   # current source, not the stale pip 0.8.2 install
$ play_launch dump -o py_model.yaml launch rt_demo bringup.launch.xml --parser python
SystemModel: py_model.yaml (4 nodes, 3 topics, 5 contracts-carrying endpoints, 3 tier(s), 2 warning(s))
```
Identical counts to the Rust path above — confirms the CLAUDE.md claim that
both parsers produce the same complete model when contract/sched inputs
resolve. (Without `PYTHONPATH` pointed at current source, this host's stale
pip-installed `play_launch` 0.8.2 triggers the documented fail-loud guard —
reproduced live, matching the W5 report.)

**`resolve --record` reuse mode + `--sched` flag (Manual Workflow doc
example):**
```
$ play_launch resolve --record rec_test.json --sched <path>/bringup.system.posix.yaml -o reuse_test.yaml
Resolving from record: rec_test.json
SystemModel: reuse_test.yaml (4 nodes, 3 topics, 5 contracts-carrying endpoints, 3 tier(s), 2 warning(s))
```

**`context record.json --tree`:**
```
$ play_launch context record.json --tree
[ 0] rt_demo bringup.launch.xml  ns=/  (4 entities)
```

`cargo build -p play_launch`: clean. `cargo test -p play_launch`: 249 + 21 +
2 passed, 0 failed (unchanged from baseline — the help-text edit doesn't
touch any test-asserted string). `cargo clippy -p play_launch --all-targets`
on the touched file: no findings.

## A real bug found while verifying: dump/resolve flag-ordering

Several pre-existing example commands (in the original `options.rs`
`after_help` text AND throughout `cli-interface.md`) placed dump/resolve
flags (`--output`/`-o`, `--format`, `--debug`, `--sched`) **after** the
launch file / `KEY:=VALUE` launch arguments. Verified live that this
silently breaks:
- `dump launch <pkg> <file> --output X` — with ≥1 preceding launch argument
  (e.g. a `map_path:=...` token), `--output`/`--format`/`--debug` get
  silently absorbed into the trailing `launch_arguments` vec (a
  `WARN ... Ignoring invalid launch argument` per token) and the flag never
  takes effect (falls back to the default output name).
- With **zero** preceding launch arguments, the nested `dump launch`
  subcommand instead hard-errors (`error: unexpected argument '--debug'
  found`, exit 2) — clap parses it as an unrecognized flag rather than
  swallowing it.
- `resolve <file> key:=val --sched X` has the identical silent-absorption
  bug (confirmed: `--sched` swallowed, sched resolved via auto-discovery
  instead — a different plan than the one requested, no error).

**Fix applied**: dump/resolve-level flags must precede the `launch`/`run`
subcommand (for `dump`) or precede any `KEY:=VALUE` launch argument (for
`resolve`, which has no nested subcommand). Reordered every example in
`options.rs` and `cli-interface.md` accordingly, and added an explicit
inline note in `cli-interface.md` explaining why. Every reordered command
was re-run against the built binary to confirm it now works (evidence
above + the autoware `--output`/`--sched` reorder tests).

This bug is orthogonal to record.json/SystemModel — it's a CLI/clap
ordering trap that happened to be baked into the exact example lines this
wave's brief asked me to verify. Fixed because "verify every command runs
as written" surfaced it directly; did not go looking for it elsewhere.

## Stale references found and fixed

- **CLAUDE.md**: "Parses launch files to `record.json`" → SystemModel
  primary; "Execution Flow: Load `record.json`" → load the SystemModel;
  added the model-parity note (both parsers → same complete model) and a
  note that the web-UI launch-tree API degrades to empty on a model-only
  replay (documented gap from the W5 report, not fixed this wave — see
  "Left as-is").
- **rt-scheduling.md**: §2.2's "parsed to `record.json`" step rewritten;
  added the "one artifact" callout; fixed the troubleshooting-table
  `context` tip to show how to actually produce a `record.json` now that
  `dump`'s default no longer does.
- **cli-interface.md**: this was the most stale file found — it still
  described the **retired** Phase 43 two-artifact model verbatim ("The
  model is cryptographically bound to `record.json` (sha256 in
  `meta.record`)", "resolve — emits the checked `system_model.yaml` +
  `*.record.json` pair", "refuses a stale pair … re-run `resolve` to
  rebind"). None of that binding exists anymore (removed in 46.4/46.5).
  Rewrote the Workflow Explanation, three-verb-split table, and Manual
  Workflow sections around the shipped one-artifact reality, with a clearly
  separated "legacy record.json path (deprecated)" block for parity
  tooling.
- **README.md**: top-level replay example still showed `--input-file
  record.json`; switched to `--model system_model.yaml`.
- **unified-system-model.md / phase-46 doc / roadmap README**: status
  headers and the "drop `file_data`/`variables`" claim (corrected to
  "confirmed nothing to drop" — the model never embedded a `LaunchDump` to
  begin with, per the W5 report; nothing was actually removed at 46.5).
- **phase-43 doc**: added a one-line "superseded by Phase 46" pointer so it
  doesn't read as the current model to someone who lands on it directly.
- **options.rs**: fixed the flag-ordering bug described above in the two
  `after_help` blocks it appeared in (top-level, `resolve`).

## What was left as-is (and why)

- **Historical/internal/archive docs** that mention `record.json` as the
  system's data format or the parser's output (not as a user-facing
  "primary artifact" claim): `docs/design/record-format.md`,
  `docs/design/parser-context.md`, `docs/design/launch-context-tool.md`,
  `docs/design/architecture-review.md`, `docs/design/rcl-interception.md`,
  everything under `docs/roadmap/archive/`, `docs/research/`,
  `docs/superpowers/`. These describe the `record.json` schema/internals or
  historical designs, which is still accurate — `record.json` still exists
  as the deprecated compat format; these docs were never claiming it's the
  primary runtime artifact. Per the brief's instruction to leave
  dev/internal/historical references alone.
- **`docs/roadmap/phase-45-sched_ssot_unification.md`**: one "Non-goals"
  line ("Spawn-artifact changes (record.json stays)") — accurate as a
  scope-boundary statement at the time Phase 45 shipped; not misleading
  enough to warrant a retroactive edit, and not in the brief's named list.
- **`play_launch context`**: still genuinely reads `record.json` only (not
  the model) — this is real, unchanged behavior (`commands/context.rs`
  calls `load_launch_dump` directly), so every doc reference to `play_launch
  context record.json --tree` is accurate as-is, not stale.
- **Web-UI launch-tree scope map** (`GET /api/launch-tree`): documented (in
  CLAUDE.md, this wave) as degrading to `{}` on a model-only replay, per the
  W5 report §6's explicit decision to defer this — not something this
  docs-only wave should silently paper over or newly claim is fixed.
- **`docs/design/system-model.md`**: general SystemModel schema/design doc,
  still accurate (mentions `--record` path once, in a parser-fidelity
  section, correctly). Not in the brief's named list; left untouched beyond
  the one CLAUDE.md doc-index addition pointing at it.
- **Pre-existing, unrelated CLI-doc drift found in `cli-interface.md`
  while verifying commands, fixed only inside blocks I substantively
  rewrote, left alone elsewhere** (a broader audit is out of this wave's
  scope):
  - `--enable-monitoring` is not a real flag (monitoring is on by default;
    the real flags are `--disable-monitoring` / `--enable <FEATURE>`).
    Fixed in the "Replay Only" and "Manual Workflow" examples I rewrote;
    left in the untouched "Launch Files" example and "Common Options"
    bullet list.
  - `--wait-for-service-ready` / `--service-ready-timeout-secs` /
    `--service-poll-interval-ms` are not CLI flags at all anymore — they're
    `container_readiness.*` fields in the `--config` YAML
    (`cli/config.rs`). Fixed in the "Manual Workflow" example I rewrote;
    left in the untouched "Common Options" bullet list, "Container Service
    Readiness" example, and Troubleshooting section.
  - `demo_nodes_cpp talker_listener.launch.py` doesn't resolve as a package
    example on this host — the real file is at
    `demo_nodes_cpp/launch/topics/talker_listener.launch.py`, and
    play_launch's package-based launch-file lookup only checks
    `share/<pkg>/launch/<file>` directly (no subdirectory search). This
    example appears unchanged throughout `cli-interface.md`, `README.md`,
    `CLAUDE.md`, and `options.rs`'s own top-level example — pre-existing,
    repo-wide, unrelated to record.json. Not fixed (would require either a
    parser-resolution code change or picking a different canonical example
    package everywhere — out of a docs-only, record.json-scoped wave).
  - `make build` / `dump_launch` binary references in `cli-interface.md`'s
    "Environment Requirements"/"Troubleshooting" sections (project
    convention is `just build`; `dump_launch` as a standalone binary in
    PATH doesn't match the current PyO3-embedded-Python architecture) — not
    touched, unrelated to record.json.

## Verification summary

- Every command block I added or rewrote was executed against the branch's
  own built binary (`just build-rust`, this worktree) — evidence pasted
  above for the representative ones; every reordered/fixed flag combination
  was individually re-run to confirm the fix.
- `cargo build -p play_launch`: clean.
- `cargo test -p play_launch`: 249 (unit) + 21 (lib) + 2 (proc_io) passed, 0
  failed — unchanged from the pre-edit baseline.
- Relative markdown links added/touched (`../design/unified-system-model.md`,
  `./phase-46-unified_system_model.md`, and the doc-index additions in
  CLAUDE.md) all resolve (`test -f` checked from each source file's
  directory).
- No logic changed — `options.rs` diff is entirely inside string literals
  (`after_help`/doc comments).
