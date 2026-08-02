# CLI Verb Reshape Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reshape the `play_launch` CLI for 0.9.0 — fold `check` into a `--check` flag, rename `replay` to `up`, remove `resolve`, and move the contract diagnostics to `ros-launch-resolve`.

**Architecture:** Two binaries in one repository since phase-55 W1. `play_launch` (links `rclrs`, needs a ROS runtime) keeps only verbs that spawn processes, plus `context`. `ros-launch-resolve` (layer 2, no ROS) gains the `check` verb, because the checker already lives there. Removed verbs stay in the clap enum as hidden variants that accept their old arguments and error with the replacement spelled out.

**Tech Stack:** Rust 2024, clap 4 derive, eyre, nextest, `just`.

## Global Constraints

- **Version:** `src/play_launch/Cargo.toml` goes `0.8.2` → `0.9.0`. Nothing else changes version.
- **Hidden verbs are deleted at 1.0.0.** Every hidden variant carries a comment saying so.
- **Errors must name the replacement and echo the user's own arguments.** A generic "see --help" is the nano-ros 0285 failure with better grammar.
- **Never add a ROS dependency to layer 2.** `just check-layer2-isolation` must stay green after every task.
- **`info!` is for end users only** — never promote `debug!` to make a test pass; set `RUST_LOG` in the test instead.
- **Build with `just build`**, never `colcon build` directly.
- **Temp files in `tmp/`**, never `/tmp`.
- Commit messages end with:
  `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`

## File Structure

| File | Responsibility | Task |
|---|---|---|
| `src/ros-launch-resolve/cli/src/options.rs` | restore `CheckArgs`, add `Command::Check` | 1 |
| `src/ros-launch-resolve/cli/src/check.rs` | **new** — the moved diagnostics handler | 1 |
| `src/ros-launch-resolve/cli/Cargo.toml` | add `ros-launch-manifest-types` | 1 |
| `src/play_launch/src/cli/options.rs` | `--check` flags, `Up`, hidden variants | 2,3,4,5,6 |
| `src/play_launch/src/commands/launch.rs` | `--check` gate | 2 |
| `src/play_launch/src/commands/run.rs` | sched-only `--check` | 3 |
| `src/play_launch/src/commands/up.rs` | renamed from `replay.rs` | 4 |
| `src/play_launch/src/commands/migrated.rs` | **new** — all hidden-verb handlers | 4 |
| `src/play_launch/src/commands/manifest.rs` | **deleted** (moved to layer 2) | 5 |
| `src/play_launch/src/commands/resolve_compat.rs` | **deleted** | 6 |

---

### Task 1: Add `check` to `ros-launch-resolve`

Additive only. Nothing is removed from `play_launch` yet, so the tree stays green.

`src/play_launch/src/commands/manifest.rs` (351 lines) has **no play_launch-specific dependencies** — it uses `ros_launch_manifest_check`, `ros_launch_manifest_types` and `ros_launch_resolve::ros::manifest_loader`, all of which layer 2 already has or can have. Layer 2's CLI also already provides both helpers it needs: `crate::launch::resolve_launch_file` and `crate::common::parse_launch_arguments`. So this is a **move with three import rewrites**, not a rewrite.

**Files:**
- Create: `src/ros-launch-resolve/cli/src/check.rs`
- Modify: `src/ros-launch-resolve/cli/src/options.rs`, `src/ros-launch-resolve/cli/src/main.rs`, `src/ros-launch-resolve/cli/Cargo.toml`

**Interfaces:**
- Consumes: `crate::launch::resolve_launch_file(&str, Option<&str>) -> Result<PathBuf>`; `crate::common::parse_launch_arguments(&[String]) -> HashMap<String, String>`
- Produces: `pub fn handle_check(args: &CheckArgs) -> Result<()>`; `pub struct CheckArgs`; `Command::Check(CheckArgs)`

- [ ] **Step 1: Add the missing dependency**

`src/ros-launch-resolve/cli/Cargo.toml`, in `[dependencies]` after the `ros-launch-manifest-check` line:

```toml
ros-launch-manifest-types = { workspace = true }
```

- [ ] **Step 2: Restore `CheckArgs`**

It was deleted as dead by issue 0013 (`a996e97`). Recover the exact original rather than retyping it:

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve
git show a996e97^:cli/src/options.rs | sed -n '/\/\/\/ Arguments for `play_launch check`/,/^}/p' > /tmp/checkargs.txt
wc -l /tmp/checkargs.txt   # expect ~70 lines ending in `}`
```

Append its contents to `cli/src/options.rs`, then change the first doc line from
``/// Arguments for `play_launch check` `` to:

```rust
/// Arguments for `ros-launch-resolve check`.
///
/// Deleted as dead by issue 0013 when the verb still lived in play_launch;
/// restored verbatim from `a996e97^` now that the verb lives here. The 2026
/// CLI verb reshape moved the diagnostics to layer 2 because the checker
/// (`ros-launch-manifest-check`) is a layer-2 crate and needs no ROS install.
```

Also confirm the struct ends with the `export_graph` field; if `git show` truncated before it, take the whole struct.

- [ ] **Step 3: Add the `Check` variant**

In `cli/src/options.rs`, inside `pub enum Command`, after the `Dump` variant:

```rust
    /// Check manifest contracts against a launch file. No ROS install
    /// required — the checker is a layer-2 crate.
    #[command(after_help = "Examples:\n  \
        ros-launch-resolve check my_bringup system.launch.xml\n  \
        ros-launch-resolve check my_bringup system.launch.xml --format json\n  \
        ros-launch-resolve check --contracts ~/contracts launch/system.launch.xml mode:=lidar")]
    Check(CheckArgs),
```

- [ ] **Step 4: Move the handler**

```bash
cd /home/aeon/repos/play_launch
git mv src/play_launch/src/commands/manifest.rs src/ros-launch-resolve/cli/src/check.rs
```

Then rewrite exactly three lines in the new file:

| Old | New |
|---|---|
| `use crate::cli::options::CheckArgs;` | `use crate::options::CheckArgs;` |
| `super::launch::resolve_launch_file(` | `crate::launch::resolve_launch_file(` |
| `super::parse_launch_arguments(` | `crate::common::parse_launch_arguments(` |

Rename the entry point from `handle_check_manifest` to `handle_check`.

`resolve_launch_file` is `pub(super)` in `cli/src/launch.rs`; change it to `pub(crate)` so `check.rs` can call it.

- [ ] **Step 5: Wire it into `main.rs`**

Add `mod check;` alongside the other module declarations, and a dispatch arm:

```rust
        Command::Check(args) => check::handle_check(args)?,
```

- [ ] **Step 6: Write the test**

Append to the `mod tests` block in `src/ros-launch-resolve/cli/src/options.rs`:

```rust
    #[test]
    fn check_accepts_the_full_diagnostic_surface() {
        let opts = parse(&[
            "check",
            "my_bringup",
            "system.launch.xml",
            "mode:=lidar",
            "--format",
            "json",
            "--rule",
            "satisfiability",
            "--rule",
            "consistency",
            "--explain",
        ])
        .expect("check must accept its diagnostic options after launch arguments");
        let Command::Check(args) = opts.command else {
            panic!("expected Check");
        };
        assert_eq!(args.package_or_path, "my_bringup");
        assert_eq!(args.launch_file.as_deref(), Some("system.launch.xml"));
        assert_eq!(args.launch_arguments, vec!["mode:=lidar".to_string()]);
        assert_eq!(args.format, "json");
        assert_eq!(args.rule, vec!["satisfiability".to_string(), "consistency".to_string()]);
        assert!(args.explain);
    }
```

- [ ] **Step 7: Run the tests**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve
cargo test -p ros-launch-resolve-cli
```
Expected: PASS, including the pre-existing `help_examples_name_only_verbs_this_binary_has`.

- [ ] **Step 8: Prove layer 2 is still ROS-free**

```bash
cd /home/aeon/repos/play_launch
./src/ros-launch-resolve/scripts/check-layer2-isolation.sh
```
Expected: all PASS. This is the constraint that matters most — the new verb must not have dragged ROS in.

- [ ] **Step 9: Verify it runs**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve
cargo run -q -p ros-launch-resolve-cli -- check --help
```
Expected: help text listing `--format`, `--rule`, `--explain`, `--export-graph`.

- [ ] **Step 10: Commit**

```bash
git add src/ros-launch-resolve
git commit -m "feat(resolve-cli): add the check verb, moved from play_launch

The checker is ros-launch-manifest-check, a layer-2 crate that needs no ROS
install, but the verb driving it lived in play_launch where running it meant
sourcing ROS first. Moving it puts the diagnostics where they can actually
be used.

A move, not a rewrite: the handler had no play_launch-specific dependencies,
and layer 2's CLI already provided both helpers it needed. Three import
lines changed. CheckArgs is restored verbatim from a996e97^ -- issue 0013
deleted it as dead when the verb still lived elsewhere.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: `launch --check`

**Files:**
- Modify: `src/play_launch/src/cli/options.rs` (`LaunchArgs`), `src/play_launch/src/commands/launch.rs`

**Interfaces:**
- Consumes: `ros_launch_resolve::model::build_checked_model(ModelBuildInputs) -> Result<SystemModel>` (already called by `handle_launch`)
- Produces: `LaunchArgs::check: bool`

- [ ] **Step 1: Write the failing test**

Append to `mod flag_ordering_tests` in `src/play_launch/src/cli/options.rs`:

```rust
    #[test]
    fn launch_accepts_check_after_launch_arguments() {
        let opts = parse(&[
            "launch",
            "pkg",
            "file.launch.xml",
            "mode:=velodyne",
            "--check",
        ])
        .expect("--check must parse after KEY:=VALUE launch arguments");
        let Command::Launch(args) = opts.command else {
            panic!("expected Launch");
        };
        assert!(args.check, "--check must set the flag");
        assert_eq!(args.launch_arguments, vec!["mode:=velodyne".to_string()]);
    }

    #[test]
    fn launch_check_defaults_off() {
        let opts = parse(&["launch", "pkg", "file.launch.xml"]).expect("must parse");
        let Command::Launch(args) = opts.command else {
            panic!("expected Launch");
        };
        assert!(!args.check, "--check must default to false");
    }
```

- [ ] **Step 2: Run it and watch it fail**

```bash
cd /home/aeon/repos/play_launch/src/play_launch
cargo test --lib launch_accepts_check_after_launch_arguments
```
Expected: FAIL, `no field 'check' on type 'LaunchArgs'`.

- [ ] **Step 3: Add the flag**

In `pub struct LaunchArgs` in `src/play_launch/src/cli/options.rs`, immediately before `#[command(flatten)] pub common: CommonOptions`:

```rust
    /// Validate contracts and scheduling, print the diagnostics, and exit
    /// without spawning anything. Exit status is 0 when clean, 1 when the
    /// checker reports errors.
    ///
    /// This is a pass/fail gate. For `--format json`, rule filters,
    /// `--explain` or graph export, use `ros-launch-resolve check`, which
    /// carries the full diagnostic surface and needs no ROS install.
    #[arg(long)]
    pub check: bool,
```

- [ ] **Step 4: Implement the gate**

In `src/play_launch/src/commands/launch.rs`, inside `handle_launch`'s async block, replace the line

```rust
        info!("Step 3/3: Replaying launch execution...");
```

and the `super::replay::play(...)` call that follows it with:

```rust
        if args.check {
            // `build_checked_model` above already ran contracts and scheduling
            // and returns Err when the checker reports errors, so reaching
            // here means the model is clean. Report and stop before spawning.
            let warnings = model.meta.warnings.len();
            info!(
                "check passed: {} node(s), {} warning(s)",
                model.structure.nodes.len(),
                warnings
            );
            for w in &model.meta.warnings {
                info!("  warning: {w}");
            }
            return Ok(());
        }

        info!("Step 3/3: Replaying launch execution...");
        super::replay::play(dump, std::sync::Arc::new(model), &args.common).await
```

If `model.meta.warnings` is not a `Vec<String>`, adapt the two uses to its actual shape; run `cargo check` and read the error rather than guessing.

- [ ] **Step 5: Run the tests**

```bash
cd /home/aeon/repos/play_launch/src/play_launch
cargo test --lib
```
Expected: PASS.

- [ ] **Step 6: Verify against a real launch file**

```bash
cd /home/aeon/repos/play_launch
source /opt/ros/humble/setup.bash && source install/setup.bash
just build-rust
./install/play_launch/lib/play_launch/play_launch launch \
    tests/fixtures/simple_test/launch/simple.launch.xml --check
echo "exit=$?"
```
Expected: prints `check passed: N node(s)`, exit 0, and **no nodes spawned** — confirm with `ps -ef | grep -c talker` before and after.

- [ ] **Step 7: Commit**

```bash
git add src/play_launch
git commit -m "feat(launch): add --check, a pass/fail contract gate

handle_launch already called build_checked_model, so contracts and
scheduling were validated on every launch; --check simply stops before the
spawn. That is what made the separate check verb redundant.

Deliberately just a flag. The diagnostic options went to
ros-launch-resolve check, where they run without a ROS install.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: `run --check` — scheduling only, and it says so

`run` cannot contract-check. Contracts are keyed by launch file (`<pkg>/launch/<stem>.contract.yaml`); `run` has a package and an executable and no launch file, so no sidecar can be located. `run.rs:204` already says this in a comment.

The mandatory message is the deliverable here, not a nicety. Without it this is a check that always passes.

**Files:**
- Modify: `src/play_launch/src/cli/options.rs` (`RunArgs`), `src/play_launch/src/commands/run.rs`

**Interfaces:**
- Consumes: `ros_launch_resolve::ros::sched_loader::resolve_platform_file(...)` — already called at `run.rs:210`
- Produces: `RunArgs::check: bool`

- [ ] **Step 1: Write the failing test**

Append to `mod flag_ordering_tests` in `src/play_launch/src/cli/options.rs`:

```rust
    #[test]
    fn run_accepts_check() {
        let opts = parse(&["run", "demo_nodes_cpp", "talker", "--check"])
            .expect("run --check must parse");
        let Command::Run(args) = opts.command else {
            panic!("expected Run");
        };
        assert!(args.check);
    }
```

- [ ] **Step 2: Run it and watch it fail**

```bash
cd /home/aeon/repos/play_launch/src/play_launch
cargo test --lib run_accepts_check
```
Expected: FAIL, `no field 'check' on type 'RunArgs'`.

- [ ] **Step 3: Add the flag**

In `pub struct RunArgs`, before `pub common: CommonOptions`:

```rust
    /// Resolve and validate the scheduling platform file for `--target`,
    /// then exit without spawning.
    ///
    /// Contracts are NOT checked and cannot be: they are keyed by launch
    /// file (`<pkg>/launch/<stem>.contract.yaml`), and `run` has no launch
    /// file, so no sidecar can apply. The output says so explicitly rather
    /// than reporting a pass over an empty check.
    #[arg(long)]
    pub check: bool,
```

- [ ] **Step 4: Implement**

In `src/play_launch/src/commands/run.rs`, immediately after the `resolved_sched` binding completes (the `resolve_platform_file(...)` call ending near line 216) and **before** the `let (sched_plan, sched_helper, sched_helper_join) = ...` block:

```rust
    if args.check {
        // Structural, not a shortcut: contracts are keyed by launch file and
        // `run` has none, so no sidecar can be located. Saying that out loud
        // is the whole point -- a silent skip here would exit 0 having
        // checked nothing checkable.
        println!(
            "no contracts checked: `run` has no launch file, so no contract \
             sidecar can apply."
        );
        match &resolved_sched {
            Some(resolved) => {
                println!(
                    "Platform file: {} (target: {}) — OK",
                    resolved.path.display(),
                    common.sched_opts.target
                );
                return Ok(());
            }
            None => {
                println!(
                    "Platform file: none resolved (target: {}) — nothing to validate",
                    common.sched_opts.target
                );
                return Ok(());
            }
        }
    }
```

`resolved_sched`'s inner type may not expose `.path`; run `cargo check` and use whatever field holds the platform-file path. Do not invent one.

Adjust `args`/`common` binding names to whatever is in scope at that point in `handle_run`.

- [ ] **Step 5: Write the behaviour test**

Create `tests/tests/run_check.rs`:

```rust
//! `run --check` must SAY that it checked no contracts.
//!
//! Without this test, a later refactor can quietly turn `run --check` into
//! an always-pass -- the vacuous-green shape of issues 0008, 0012 and 0014.

mod common;
use play_launch_tests::fixtures;

#[test]
fn run_check_states_that_no_contracts_were_checked() {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args(["run", "demo_nodes_cpp", "talker", "--check"]);
    let out = cmd.output().expect("failed to run play_launch");

    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        stdout.contains("no contracts checked"),
        "run --check must state that no contracts were checked, else it \
         reports a pass over an empty check.\nstdout: {stdout}\nstderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(out.status.success(), "expected exit 0 with no platform file");
}
```

Drop the `mod common;` line if `tests/tests/` has no such module — check a neighbouring file such as `tests/tests/manifest_check.rs` for the exact preamble and copy it.

- [ ] **Step 6: Run both tests**

```bash
cd /home/aeon/repos/play_launch
source /opt/ros/humble/setup.bash && source install/setup.bash
just build-rust
cd tests && cargo nextest run -E 'test(run_check) or test(run_accepts_check)'
```
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
cd /home/aeon/repos/play_launch
git add src/play_launch tests
git commit -m "feat(run): add --check for the scheduling platform file only

run cannot contract-check, structurally: contracts are keyed by launch file
and run has none, so no sidecar can be located. It validates the platform
file for --target and exits on that alone.

The output MUST state that no contracts were checked, and a test enforces
it. A uniform --check that silently skipped the contract stage would exit 0
having checked nothing checkable -- the vacuous-pass shape of issues 0008,
0012 and 0014.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: `replay` → `up`, with the hidden-verb machinery

This task introduces `commands/migrated.rs`, which Tasks 5 and 6 extend.

**Files:**
- Rename: `src/play_launch/src/commands/replay.rs` → `up.rs`
- Create: `src/play_launch/src/commands/migrated.rs`
- Modify: `src/play_launch/src/cli/options.rs`, `commands/mod.rs`, `main.rs`, `commands/launch.rs`

**Interfaces:**
- Consumes: `commands::up::play(LaunchDump, Arc<SystemModel>, &CommonOptions) -> Result<()>` (unchanged, only its module moves)
- Produces: `Command::Up(UpArgs)`; `Command::Replay(UpArgs)` hidden; `commands::migrated::replay_renamed(&UpArgs) -> Result<()>`

- [ ] **Step 1: Write the failing tests**

Append to `mod flag_ordering_tests` in `src/play_launch/src/cli/options.rs`:

```rust
    #[test]
    fn up_takes_a_model_positionally_and_via_flag() {
        let a = parse(&["up", "system_model.yaml"]).expect("positional model must parse");
        let Command::Up(args) = a.command else { panic!("expected Up") };
        assert_eq!(args.model_path, Some(PathBuf::from("system_model.yaml")));

        let b = parse(&["up", "--model", "system_model.yaml"]).expect("--model must parse");
        let Command::Up(args) = b.command else { panic!("expected Up") };
        assert_eq!(args.model, Some(PathBuf::from("system_model.yaml")));
    }

    #[test]
    fn replay_still_parses_so_it_can_be_redirected() {
        // Hidden, not deleted: it must accept the old arguments so the error
        // can echo the user's own invocation back in the new form. clap's
        // bare "unrecognized subcommand" is what took down every nano-ros
        // platform's fixture build (issue 0285).
        let opts = parse(&["replay", "system_model.yaml"])
            .expect("`replay` must still PARSE so it can be redirected");
        assert!(matches!(opts.command, Command::Replay(_)));
    }
```

- [ ] **Step 2: Run them and watch them fail**

```bash
cd /home/aeon/repos/play_launch/src/play_launch
cargo test --lib up_takes_a_model
```
Expected: FAIL, no `Up` variant.

- [ ] **Step 3: Rename the module**

```bash
cd /home/aeon/repos/play_launch
git mv src/play_launch/src/commands/replay.rs src/play_launch/src/commands/up.rs
```

In `commands/mod.rs`: `pub mod replay;` → `pub mod up;`, and
`pub use replay::handle_replay;` → `pub use up::handle_up;`.

In `commands/up.rs`: rename `handle_replay` → `handle_up`. Leave `play()` alone — it is the shared engine `launch` also calls, and the name is already neutral.

In `commands/launch.rs`: `super::replay::play(` → `super::up::play(`.

- [ ] **Step 4: Rename the args and add both variants**

In `src/play_launch/src/cli/options.rs`, rename `pub struct ReplayArgs` to `pub struct UpArgs` (keep every field and doc comment). Replace the `Replay(ReplayArgs)` variant with:

```rust
    /// Bring a system up from a resolved SystemModel and supervise it.
    ///
    /// Renamed from `replay` in 0.9.0: it replays nothing. It loads a
    /// declarative artifact and spawns from it — the old name was a fossil
    /// of `record.json`, which Phase 47 removed.
    #[command(after_help = "Examples:\n  \
        play_launch up system_model.yaml\n  \
        play_launch up --model system_model.yaml --disable-all\n  \
        play_launch up system_model.yaml --web-addr 0.0.0.0:8080")]
    Up(UpArgs),

    /// Renamed to `up` in 0.9.0. Hidden; accepts the old arguments so the
    /// error can name the replacement. DELETE AT 1.0.0.
    #[command(hide = true)]
    Replay(UpArgs),
```

- [ ] **Step 5: Create the migration handlers**

Create `src/play_launch/src/commands/migrated.rs`:

```rust
//! Handlers for verbs removed or renamed in 0.9.0.
//!
//! These variants are hidden from `--help` but still PARSE, accepting their
//! old arguments, so the error can echo the user's own invocation back in
//! the new form.
//!
//! That is the whole point. nano-ros issue 0285 was a subcommand vanishing
//! and clap answering `unrecognized subcommand 'resolve'` from inside a
//! cmake configure, which took down every platform's fixture build. The
//! failure was the error, not the removal.
//!
//! DELETE THIS MODULE AT 1.0.0.

use crate::cli::options::UpArgs;
use eyre::{Result, bail};

/// Reconstruct the model argument the user passed, for echoing back.
fn model_arg(args: &UpArgs) -> String {
    match (&args.model_path, &args.model) {
        (Some(p), _) => p.display().to_string(),
        (None, Some(p)) => format!("--model {}", p.display()),
        (None, None) => "<model.yaml>".to_string(),
    }
}

pub fn replay_renamed(args: &UpArgs) -> Result<()> {
    bail!(
        "`replay` was renamed to `up` in 0.9.0.\n       play_launch up {}",
        model_arg(args)
    )
}
```

Add `pub mod migrated;` to `commands/mod.rs`.

- [ ] **Step 6: Dispatch both**

In `src/play_launch/src/main.rs`, replace the `Command::Replay(args)` arm with:

```rust
        play_launch::cli::options::Command::Up(args) => {
            play_launch::commands::handle_up(args)?;
        }
        play_launch::cli::options::Command::Replay(args) => {
            play_launch::commands::migrated::replay_renamed(args)?;
        }
```

- [ ] **Step 7: Extend the help-integrity test**

In `src/play_launch/src/cli/options.rs`, in `help_advertises_only_verbs_this_binary_has`, change the loop list from `["dump", "plot", "contract"]` to include the newly hidden verb:

```rust
        for gone in ["dump", "plot", "contract", "replay"] {
```

This asserts `replay` is absent from `--help`. It is still in the enum, so also add:

```rust
    #[test]
    fn hidden_migration_verbs_parse_but_are_not_advertised() {
        use clap::CommandFactory;
        let help = Options::command().render_long_help().to_string();
        assert!(!help.contains("  replay"), "`replay` must not be listed in --help");
        assert!(
            parse(&["replay", "m.yaml"]).is_ok(),
            "`replay` must still parse so its error can name the replacement"
        );
    }
```

If the existing loop asserts `!verbs.iter().any(...)` for each name, that assertion will now fail for `replay` because it IS a variant. Change that inner assertion to check the rendered help only, not the subcommand list — hidden subcommands are still present in `get_subcommands()`.

- [ ] **Step 8: Write the error-quality test**

Create `tests/tests/migrated_verbs.rs`:

```rust
//! Removed verbs must name their replacement.
//!
//! Asserting only that the command "fails" is what nano-ros issue 0285 did:
//! it failed, with `unrecognized subcommand 'resolve'`, from inside a cmake
//! configure. These tests assert the error is USEFUL.

use play_launch_tests::fixtures;

#[test]
fn replay_names_up_as_its_replacement() {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args(["replay", "system_model.yaml"]);
    let out = cmd.output().expect("failed to run play_launch");

    assert!(!out.status.success(), "`replay` must exit non-zero");
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(err.contains("renamed to `up`"), "must name the rename: {err}");
    assert!(
        err.contains("play_launch up system_model.yaml"),
        "must echo the user's own argument in the new form: {err}"
    );
}
```

Copy the exact preamble (`mod common;` or similar) from a neighbouring file in `tests/tests/`.

- [ ] **Step 9: Run everything**

```bash
cd /home/aeon/repos/play_launch
source /opt/ros/humble/setup.bash && source install/setup.bash
just build-rust
cd src/play_launch && cargo test --lib
cd ../../tests && cargo nextest run -E 'test(migrated_verbs)'
```
Expected: PASS.

- [ ] **Step 10: Migrate the 16 in-repo references**

```bash
cd /home/aeon/repos/play_launch
grep -rn '"replay"\|play_launch replay' tests/ docs/ scripts/ justfile 2>/dev/null | grep -v /target/
```
Change each to `up`, except the ones in `tests/tests/migrated_verbs.rs` and any doc text deliberately describing the old name.

- [ ] **Step 11: Commit**

```bash
git add -A src/play_launch tests docs scripts justfile
git commit -m "feat(cli)!: rename replay to up

It replayed nothing. It loads a declarative SystemModel and spawns from it;
the name was a fossil of record.json, which Phase 47 removed as a
user-facing artifact. replay outlived the thing it was named after.

replay stays in the enum as a hidden variant that accepts the old arguments
and errors naming the replacement, echoing the user's own model path back in
the new form. nano-ros 0285 was a subcommand vanishing and clap answering
'unrecognized subcommand' from inside a cmake configure -- that failure was
the error, not the removal. Delete at 1.0.0.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Remove the `check` verb from `play_launch`

Task 1 must be complete — the error text points at `ros-launch-resolve check`, which has to exist.

**Files:**
- Delete: `src/play_launch/src/commands/manifest.rs` (already moved in Task 1; this removes the stale registration)
- Modify: `src/play_launch/src/cli/options.rs`, `commands/mod.rs`, `commands/migrated.rs`, `main.rs`

**Interfaces:**
- Consumes: `CheckArgs` (stays in play_launch's options solely so the hidden verb can parse the old arguments)
- Produces: `commands::migrated::check_removed(&CheckArgs) -> Result<()>`

- [ ] **Step 1: Write the failing test**

Append to `tests/tests/migrated_verbs.rs`:

```rust
#[test]
fn check_names_both_replacements() {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args(["check", "demo_pkg", "a.launch.xml", "--format", "json"]);
    let out = cmd.output().expect("failed to run play_launch");

    assert!(!out.status.success(), "`check` must exit non-zero");
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(
        err.contains("launch demo_pkg a.launch.xml --check"),
        "must name the gate replacement, echoing the user's args: {err}"
    );
    assert!(
        err.contains("ros-launch-resolve check"),
        "must name where the diagnostics went: {err}"
    );
}
```

- [ ] **Step 2: Run it and watch it fail**

```bash
cd /home/aeon/repos/play_launch/tests
cargo nextest run -E 'test(check_names_both_replacements)'
```
Expected: FAIL — the real `check` verb still runs and exits 0 or 1 from the checker.

- [ ] **Step 3: Hide the variant**

In `src/play_launch/src/cli/options.rs`, replace the `Check(CheckArgs)` variant's doc and attributes with:

```rust
    /// Removed in 0.9.0 — split into `launch --check` (gate) and
    /// `ros-launch-resolve check` (diagnostics). Hidden; accepts the old
    /// arguments so the error can name both replacements. DELETE AT 1.0.0.
    #[command(hide = true)]
    Check(CheckArgs),
```

Keep `CheckArgs` in this file. It exists only to let the hidden verb parse; add that as a comment on the struct so nobody deletes it as dead the way 0013 did.

- [ ] **Step 4: Add the handler**

Append to `src/play_launch/src/commands/migrated.rs`:

```rust
use crate::cli::options::CheckArgs;

pub fn check_removed(args: &CheckArgs) -> Result<()> {
    let target = match &args.launch_file {
        Some(f) => format!("{} {}", args.package_or_path, f),
        None => args.package_or_path.clone(),
    };
    let launch_args = if args.launch_arguments.is_empty() {
        String::new()
    } else {
        format!(" {}", args.launch_arguments.join(" "))
    };
    bail!(
        "`check` was removed in 0.9.0. Two replacements:\n       \
         play_launch launch {target}{launch_args} --check   (pass/fail gate)\n       \
         ros-launch-resolve check {target}{launch_args}   (diagnostics: --format, \
         --rule, --explain, --export-graph; no ROS install needed)"
    )
}
```

- [ ] **Step 5: Rewire the dispatch and drop the dead module**

In `main.rs`:

```rust
        play_launch::cli::options::Command::Check(args) => {
            play_launch::commands::migrated::check_removed(args)?;
        }
```

In `commands/mod.rs`, remove `pub mod manifest;` and `pub use manifest::handle_check_manifest;`.

- [ ] **Step 6: Extend the help-integrity test**

In `help_advertises_only_verbs_this_binary_has`, add `"check"` to the banned list, and in `hidden_migration_verbs_parse_but_are_not_advertised` add the same assertions for `check`.

- [ ] **Step 7: Run the tests**

```bash
cd /home/aeon/repos/play_launch
source /opt/ros/humble/setup.bash && source install/setup.bash
just build-rust
cd src/play_launch && cargo test --lib
cd ../../tests && cargo nextest run -E 'test(migrated_verbs)'
```
Expected: PASS.

- [ ] **Step 8: Migrate the 66 in-repo references**

```bash
cd /home/aeon/repos/play_launch
grep -rn 'play_launch check\|"check",' tests/ docs/ scripts/ justfile 2>/dev/null | grep -v /target/
```

Route each by what it needs: a bare pass/fail becomes `launch ... --check`; anything using `--format`, `--rule`, `--explain` or `--export-graph` becomes `ros-launch-resolve check ...`. `tests/tests/manifest_check.rs` is the big one — it already has a `cargo_bin` helper for the layer-2 binary, so point it at that.

- [ ] **Step 9: Full suite**

```bash
cd /home/aeon/repos/play_launch
just build
source /opt/ros/humble/setup.bash && source install/setup.bash
just test-all
```
Expected: all pass, 0 unexpected skips.

- [ ] **Step 10: Commit**

```bash
git add -A
git commit -m "feat(cli)!: remove the check verb, split into a flag and a layer-2 verb

handle_launch already called build_checked_model, so check was the launch
pipeline minus the spawn wearing a separate verb. It becomes launch --check.
The diagnostic options moved to ros-launch-resolve check in the previous
commit, where they run without a ROS install.

The hidden variant names BOTH replacements and echoes the user's own
package, launch file and arguments into each. CheckArgs stays in
play_launch's options purely to let it parse -- commented as such, since
issue 0013 deleted exactly this kind of struct as dead. Delete at 1.0.0.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Remove the `resolve` verb

**Files:**
- Delete: `src/play_launch/src/commands/resolve_compat.rs`
- Modify: `src/play_launch/src/cli/options.rs`, `commands/mod.rs`, `commands/migrated.rs`, `main.rs`

- [ ] **Step 1: Write the failing test**

Append to `tests/tests/migrated_verbs.rs`:

```rust
#[test]
fn resolve_names_the_layer_two_binary() {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args(["resolve", "demo_pkg", "a.launch.xml", "-o", "m.yaml"]);
    let out = cmd.output().expect("failed to run play_launch");

    assert!(!out.status.success(), "`resolve` must exit non-zero");
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(
        err.contains("ros-launch-resolve resolve demo_pkg a.launch.xml"),
        "must echo the user's own invocation against the new binary: {err}"
    );
}
```

- [ ] **Step 2: Run it and watch it fail**

```bash
cd /home/aeon/repos/play_launch/tests
cargo nextest run -E 'test(resolve_names_the_layer_two_binary)'
```
Expected: FAIL — the delegate still runs and writes `m.yaml`.

- [ ] **Step 3: Hide the variant**

In `src/play_launch/src/cli/options.rs`:

```rust
    /// Removed in 0.9.0 — it delegated to layer 2 since RFC-0060. Hidden;
    /// accepts the old arguments so the error can name the replacement.
    /// DELETE AT 1.0.0.
    #[command(hide = true)]
    Resolve(ResolveArgs),
```

- [ ] **Step 4: Add the handler**

Append to `src/play_launch/src/commands/migrated.rs`:

```rust
use crate::cli::options::ResolveArgs;

pub fn resolve_removed(args: &ResolveArgs) -> Result<()> {
    let target = match &args.launch_file {
        Some(f) => format!("{} {}", args.package_or_path, f),
        None => args.package_or_path.clone(),
    };
    let launch_args = if args.launch_arguments.is_empty() {
        String::new()
    } else {
        format!(" {}", args.launch_arguments.join(" "))
    };
    bail!(
        "`resolve` was removed in 0.9.0 — it delegated to layer 2 since \
         RFC-0060.\n       ros-launch-resolve resolve {target}{launch_args} -o {}",
        args.out
    )
}
```

- [ ] **Step 5: Rewire and delete the delegate**

In `main.rs`:

```rust
        play_launch::cli::options::Command::Resolve(args) => {
            play_launch::commands::migrated::resolve_removed(args)?;
        }
```

```bash
git rm src/play_launch/src/commands/resolve_compat.rs
```

Remove `pub mod resolve_compat;` and `pub use resolve_compat::handle_resolve;` from `commands/mod.rs`.

- [ ] **Step 6: Extend the help-integrity test**

Add `"resolve"` to the banned list in `help_advertises_only_verbs_this_binary_has` and to `hidden_migration_verbs_parse_but_are_not_advertised`.

- [ ] **Step 7: Migrate the 56 in-repo references**

```bash
cd /home/aeon/repos/play_launch
grep -rn 'play_launch resolve\|"resolve",' tests/ docs/ scripts/ justfile 2>/dev/null \
    | grep -v /target/ | grep -v 'ros-launch-resolve resolve'
```
Each becomes `ros-launch-resolve resolve ...`. In integration tests use the existing layer-2 `cargo_bin` helper rather than assuming the binary is on `PATH`.

- [ ] **Step 8: Full suite**

```bash
cd /home/aeon/repos/play_launch
just build
source /opt/ros/humble/setup.bash && source install/setup.bash
just test-all
just check
./src/ros-launch-resolve/scripts/check-layer2-isolation.sh
```
Expected: all pass.

- [ ] **Step 9: Commit**

```bash
git add -A
git commit -m "feat(cli)!: remove the resolve verb from play_launch

It had been a deprecated delegate since RFC-0060 W3 -- printing a warning
and calling build_checked_model, which is layer 2's own library. It survived
this long precisely because removing it silently is what caused nano-ros
0285; the hidden variant removes that objection by erroring with the user's
own invocation rewritten against ros-launch-resolve.

Known casualty, accepted: simple-autoware-safety-island embeds
'play_launch resolve' in sentinel_bringup/launch/pilot.launch.xml. It breaks
on upgrade, with a message naming the replacement rather than clap's
'unrecognized subcommand'.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: Version bump and migration documentation

**Files:**
- Modify: `src/play_launch/Cargo.toml`, `CLAUDE.md`, `docs/roadmap/README.md`
- Create: `docs/guide/cli-migration-0.9.md`, `docs/roadmap/phase-56-cli-verb-reshape.md`

- [ ] **Step 1: Bump the version**

`src/play_launch/Cargo.toml`: `version = "0.8.2"` → `version = "0.9.0"`.

Check whether `just set-version` / `scripts/bump_version.py` is the sanctioned path:

```bash
cd /home/aeon/repos/play_launch
grep -n -A6 '^set-version' justfile
```
If that recipe exists, use it instead of editing by hand — it likely syncs the Python package and the wheel metadata too.

- [ ] **Step 2: Write the migration guide**

Create `docs/guide/cli-migration-0.9.md`:

```markdown
# CLI migration: 0.8.x → 0.9.0

0.9.0 reshapes the verbs. Every removed verb still parses and errors with
its replacement spelled out, echoing your own arguments back — you will not
get `unrecognized subcommand`.

| 0.8.x | 0.9.0 |
|---|---|
| `play_launch replay m.yaml` | `play_launch up m.yaml` |
| `play_launch check <pkg> <f>` | `play_launch launch <pkg> <f> --check` |
| `play_launch check <pkg> <f> --format json` | `ros-launch-resolve check <pkg> <f> --format json` |
| `play_launch resolve <pkg> <f> -o m.yaml` | `ros-launch-resolve resolve <pkg> <f> -o m.yaml` |

## Why

**`replay` → `up`.** It replayed nothing. It loads a resolved SystemModel and
supervises what it spawns. The name was a fossil of `record.json`, retired in
Phase 47.

**`check` → a flag plus a layer-2 verb.** `launch` already validated contracts
and scheduling on every run, so `check` was the same pipeline minus the spawn.
The pass/fail gate is now `--check`. The diagnostics moved to
`ros-launch-resolve check`, where they run **without a ROS install** — the
checker never needed one.

**`resolve` removed.** It had been a delegate to `ros-launch-resolve` since
RFC-0060, printing a deprecation warning on every call.

## `run --check`

`run` validates the scheduling platform file only, and says so. Contracts are
keyed by launch file and `run` has none, so no contract sidecar can apply.

## Compatibility

The hidden verbs are removed at 1.0.0. Migrate rather than relying on them.
```

- [ ] **Step 3: Update `CLAUDE.md`**

Update the `## Installation & Usage` block to use `up` rather than `replay`, and add under it:

```markdown
**Verb layering (0.9.0):** `play_launch` keeps only verbs that spawn
processes — `launch`, `run`, `up` — plus `context`, `setcap`, `verify`.
Everything that only reads or checks files lives in `ros-launch-resolve`
(`resolve`, `dump`, `check`, `contract`, `plot`) and runs with no ROS
install. `context` is a deliberate exception: it needs no ROS but is used
while debugging a launch you just ran, where ROS is already sourced.

Removed verbs (`replay`, `check`, `resolve`) are hidden clap variants that
error naming their replacement — see `src/play_launch/src/commands/migrated.rs`.
**Delete that module at 1.0.0.**
```

- [ ] **Step 4: Write the phase doc**

Create `docs/roadmap/phase-56-cli-verb-reshape.md` summarising the six decisions, linking
`docs/superpowers/specs/2026-08-03-cli-verb-reshape-design.md` as the design of record,
and recording the accepted casualty (simple-autoware-safety-island).

Add a line to `docs/roadmap/README.md`'s trailing index:

```markdown
- **Phase 56** — ✅ CLI verb reshape for 0.9.0: `check` → `--check` + a layer-2
  verb, `replay` → `up`, `resolve` removed. Removed verbs error naming their
  replacement; hidden variants delete at 1.0.0.
  [phase-56-cli-verb-reshape.md](./phase-56-cli-verb-reshape.md).
```

- [ ] **Step 5: Final verification**

```bash
cd /home/aeon/repos/play_launch
just build
source /opt/ros/humble/setup.bash && source install/setup.bash
just test-all
just test-unit
just check
./install/play_launch/lib/play_launch/play_launch --help
./src/ros-launch-resolve/target/debug/ros-launch-resolve --help
```
Expected: suites green; `play_launch --help` lists exactly `launch run up context setcap verify`; `ros-launch-resolve --help` lists `resolve dump check contract plot`.

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "docs: 0.9.0 CLI migration guide and phase 56

Bumps to 0.9.0 -- the verb reshape is breaking. Adds the migration table,
the reasoning behind each rename, and the 1.0.0 deletion date for the hidden
compatibility variants so they do not become permanent.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Self-Review

**Spec coverage:** D1 hard cut → Task 7 version bump plus the hidden variants in Tasks 4–6. D2 check split → Tasks 1, 2, 5. D3 `replay`→`up` → Task 4. D4 helpful errors → Tasks 4, 5, 6 (each with an error-quality test). D5 `resolve` removed / `context` stays → Task 6; `context` is untouched everywhere, which is the requirement. D6 `run --check` → Task 3, with the mandatory-message test. Blast radius → Steps 10/8/7 of Tasks 4/5/6. Testing section → error-quality tests, the `run --check` message test, extended help-integrity tests, `just test-all` and the isolation gate in Tasks 1, 5, 6.

**Placeholders:** none. Three places instruct the implementer to run `cargo check` and read the real error rather than guess a field name (`model.meta.warnings`, `resolved_sched`'s path field, the `tests/tests/` preamble) — these are deliberate, since guessing a signature is how a plan produces code that does not compile.

**Type consistency:** `UpArgs` is introduced in Task 4 and used consistently in `migrated.rs`. `CheckArgs` exists in both binaries by design — layer 2's is the live one (Task 1), play_launch's survives only to let the hidden verb parse (Task 5), and Task 5 comments it so it is not deleted as dead the way issue 0013 deleted its predecessor. `handle_check` (layer 2) and `handle_check_manifest` (removed from play_launch) are distinct names, no collision.
