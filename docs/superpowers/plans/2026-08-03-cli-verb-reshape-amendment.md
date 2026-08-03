# CLI Verb Reshape — Amendment: play_launch is the only surface users see

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Make `play_launch` the complete user-facing CLI again. `ros-launch-resolve` becomes a developer/integration binary that no end user sees, installs, or is told to run.

**Why this amendment exists:** the original plan's decisions D2 and D5 split `check` into a layer-2 verb and removed `resolve`, then pointed users at `ros-launch-resolve` from README, the migration guide and three error messages — and shipped that binary to PyPI as a console script to make the advice true. That was wrong. **`ros-launch-resolve` is an intermediate package for integrating with other projects (nano-ros). Users install `play_launch` and see nothing else.**

It also left a functional dead end: after the reshape **no `play_launch` verb could write a `system_model.yaml`** (`resolve` and `dump` had both left), yet `up` requires one. A pip-only user had `up` and no way to feed it.

**Architecture:** `play_launch` already *links* the resolve library (`src/play_launch/Cargo.toml:38`), so it never needed a second binary. The five launch-tree handlers move from `ros-launch-resolve/cli/src/` into the resolve **library**, parameterised on plain values instead of clap `*Args` structs. Both CLIs become thin arg-mapping wrappers over one implementation — no duplication, which is the trap that produced this mess.

## Global Constraints

- **`ros-launch-resolve` must not appear in any user-facing artifact:** `README.md`, `docs/guide/`, or any error message a user can trigger. `CLAUDE.md` is a developer doc and SHOULD keep it.
- **The wheel ships `play_launch` only.** Revert the `ros-launch-resolve` bundling and its console-script entry point.
- Layer 2 must keep building and running with NO ROS: `./src/ros-launch-resolve/scripts/check-layer2-isolation.sh` must pass.
- ONE implementation per verb. If you find yourself copying a handler between the two CLIs, stop — it belongs in the library.
- `info!` is for end users only; never promote `debug!` to pass a test.
- Temp files in `tmp/`, never `/tmp`.
- `cargo clippy --all-targets --all-features -- -D warnings` and `cargo +nightly fmt --check` gate `src/play_launch`; clippy+fmt also gate layer 2.
- Commit messages end with `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`.
- ENVIRONMENT: sourcing ROS needs `bash -c '...'` — the default shell is zsh and leaves `BASH_SOURCE` unset.
- TEST-FILTER TRAP: `cargo nextest run -E "test(NAME)"` filters on the test FUNCTION name; a file-name filter matches ZERO tests and exits 0. Use `binary(NAME)` and confirm a non-zero match count.

## Target surface

```
play_launch  (what `pip install play_launch` gives you — the whole product)
  launch [--check]   run [--check]   up
  resolve   dump   check   plot   contract
  context   setcap   verify

ros-launch-resolve  (developer / nano-ros integration; NOT in the wheel,
                     named only in CLAUDE.md)
  resolve   dump   check   contract   plot
```

`replay` → `up` stays renamed; its hidden migration variant stays. The hidden
`check` and `resolve` variants are DELETED — those are real verbs again.

---

### Task 8: Move the five handlers into the resolve library

**Files:**
- Create: `src/ros-launch-resolve/resolve/src/verbs/{mod,resolve,dump,check,plot,contract}.rs`
- Modify: `src/ros-launch-resolve/resolve/src/lib.rs`, `resolve/Cargo.toml`
- Rewrite as thin wrappers: `src/ros-launch-resolve/cli/src/{resolve,dump,check,plot,contract}.rs`

**Interfaces produced** (both CLIs call these; keep the names stable):
- `verbs::resolve::run(inputs: ResolveInputs) -> Result<()>`
- `verbs::dump::run(inputs: DumpInputs) -> Result<()>`
- `verbs::check::run(inputs: CheckInputs) -> Result<i32>` — returns the intended exit code rather than calling `exit`
- `verbs::plot::run(inputs: PlotInputs) -> Result<()>`
- `verbs::contract::eject(inputs: ContractEjectInputs) -> Result<()>`

Each `*Inputs` is a plain struct of owned values (`String`, `PathBuf`, `Vec<String>`, `bool`) with NO clap derive and NO dependency on either CLI's `options` module. That is the whole point: the library must not know a CLI exists.

- [ ] **Step 1: Read the five current handlers** in `src/ros-launch-resolve/cli/src/` — `resolve.rs` (78 lines), `dump.rs` (51), `check.rs` (352), `plot.rs` (46), `contract.rs` (113). Note for each exactly which fields of its `*Args` struct it reads. Those fields become the `*Inputs` struct.

- [ ] **Step 2: Create the library module.** `resolve/src/verbs/mod.rs` declaring the five submodules, and `pub mod verbs;` in `resolve/src/lib.rs`. Move each handler's body across, replacing `args.foo` with `inputs.foo`. Move any dependency the library lacks into `resolve/Cargo.toml` (`check.rs` needs `ros-launch-manifest-types`).

- [ ] **Step 3: `check` must not call `std::process::exit`.** It currently signals failure through the process exit code. Return `Result<i32>` and let each CLI decide. Verify by reading the current code what its exit contract is, and preserve it exactly.

- [ ] **Step 4: Rewrite the five layer-2 CLI handlers as wrappers** — build the `*Inputs` from the `*Args` and call the library. Each should be roughly 10–20 lines.

- [ ] **Step 5: Verify layer 2 is unchanged in behaviour.**
```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve
cargo test && cargo clippy --all-targets --all-features -- -D warnings && cargo fmt --check
cd /home/aeon/repos/play_launch && ./src/ros-launch-resolve/scripts/check-layer2-isolation.sh
```
Then run each verb by hand and confirm the output is identical to before the move. Capture before/after output for at least `resolve` and `check`.

- [ ] **Step 6: Commit.**

---

### Task 9: play_launch regains the five verbs

**Files:**
- Modify: `src/play_launch/src/cli/options.rs`, `commands/mod.rs`, `commands/migrated.rs`, `main.rs`
- Create: `src/play_launch/src/commands/{resolve,dump,check,plot,contract}.rs` (thin wrappers on `verbs::*`)

- [ ] **Step 1: Restore the deleted arg structs.** `DumpArgs`/`DumpSubcommand` were deleted in `b1161bd`; `PlotArgs`, `ContractArgs`, `ContractSubcommand`, `ContractEjectArgs` in `16b2aca`. Recover them from git rather than retyping:
```bash
cd /home/aeon/repos/play_launch
git show b1161bd^:src/play_launch/src/cli/options.rs > tmp/opts-before-dump.rs
git show 16b2aca^:src/play_launch/src/cli/options.rs > tmp/opts-before-residue.rs
```
`CheckArgs` and `ResolveArgs` are already present — they survived as hidden-variant parsers. Update their comments: they are live verb arguments again, not compatibility scaffolding.

- [ ] **Step 2: Add the five `Command` variants** — `Resolve`, `Dump`, `Check`, `Plot`, `Contract` — with help text and `after_help` examples that invoke **`play_launch`**, never `ros-launch-resolve`.

- [ ] **Step 3: DELETE the hidden `Check` and `Resolve` migration variants** and their handlers `check_removed` / `resolve_removed` in `commands/migrated.rs`. They are real verbs again, so a hidden variant would shadow them. Keep `replay_renamed` and the `Replay` variant — that rename stands. Keep `target_and_launch_args` only if `replay_renamed` still needs it; delete it if not.

- [ ] **Step 4: Write the five thin command handlers** in `src/play_launch/src/commands/`, each mapping its `*Args` to the library's `*Inputs` and calling `verbs::*::run`. If a handler exceeds ~25 lines, logic is leaking out of the library — push it back.

- [ ] **Step 5: Update the help-integrity tests.** `help_advertises_only_verbs_this_binary_has` currently bans `dump`, `plot`, `contract`, `check`, `resolve` — all five are now legitimate. Invert it: assert each of the five IS present in `--help`, and that `replay` is still absent-but-parseable. `hidden_migration_verbs_parse_but_are_not_advertised` should now cover only `replay`.

- [ ] **Step 6: Update `tests/tests/migrated_verbs.rs`.** Delete `check_names_both_replacements` and `resolve_names_the_layer_two_binary` — those verbs work now, so those tests assert the wrong thing. Keep the `replay` test. Add a test that `play_launch resolve <pkg> <file> -o m.yaml` actually writes a model, and one that `play_launch check` runs the checker — the dead end this amendment exists to close.

- [ ] **Step 7: Verify the round trip a pip user depends on**, end to end, and paste the output:
```bash
bash -c 'source /opt/ros/humble/setup.bash && source /home/aeon/repos/play_launch/install/setup.bash
  cd /home/aeon/repos/play_launch
  ./install/play_launch/lib/play_launch/play_launch resolve \
      tests/fixtures/simple_test/launch/pure_nodes.launch.xml -o tmp/amend.yaml
  ./install/play_launch/lib/play_launch/play_launch up tmp/amend.yaml --disable-all &
  sleep 5; kill %1'
```
`resolve` must write the model and `up` must accept it. That round trip was impossible before this task.

- [ ] **Step 8:** clippy, fmt, `cargo test --lib`, `binary(migrated_verbs)` with a non-zero match count. Commit.

---

### Task 10: Unship the binary, scrub the user-facing docs

- [ ] **Step 1: Revert the wheel bundling.** Remove `ros-launch-resolve` from `scripts/bundle_wheel.sh`'s ARTIFACTS, remove its `[project.scripts]` entry in `pyproject.toml`, and delete the `python/play_launch/resolve_cli.py` shim. Keep the release-only guard that commit added for the binaries the wheel *does* ship — that fix was independently correct.

- [ ] **Step 2: Scrub `README.md`** (11 mentions) and **`docs/guide/cli-migration-0.9.md`** (7). Every user-facing instruction becomes a `play_launch` verb. The migration table's `check`/`resolve` rows are now wrong in both columns — 0.8.x `play_launch check` maps to 0.9.0 `play_launch check`, i.e. unchanged, so those rows should simply go. What genuinely changed for a user is `replay` → `up`, and the new `--check` flags.

- [ ] **Step 3: Update `CLAUDE.md`** — it KEEPS `ros-launch-resolve`, and should state the rule explicitly:
```markdown
**`ros-launch-resolve` is a developer/integration binary.** It exists so
nano-ros and other consumers can resolve launch trees without linking a ROS
runtime. It is NOT shipped in the wheel and must NEVER appear in `README.md`,
`docs/guide/`, or any error message a user can trigger — `pip install
play_launch` is the whole product as far as a user is concerned. Both CLIs are
thin wrappers over `ros_launch_resolve::verbs::*`; put shared logic there, never
in a CLI.
```

- [ ] **Step 4: Update the phase doc** `docs/roadmap/phase-56-cli-verb-reshape.md` and the design doc `docs/superpowers/specs/2026-08-03-cli-verb-reshape-design.md` to record that D2 and D5 were REVERSED, and why. Do not quietly rewrite them as if the original decisions never happened — the reversal and its reason are the useful record.

- [ ] **Step 5: Full gates.** `just build`, `just test-all` (pass count + skip summary), `just test-unit`, `just check`, layer-2 isolation gate. Then rebuild the wheel and prove `ros-launch-resolve` is NOT in it:
```bash
unzip -l dist/play_launch-0.9.0-py3-none-any.whl | grep -i resolve   # expect: nothing
```
And confirm `play_launch --help` lists all eleven verbs.

- [ ] **Step 6: Commit.**
