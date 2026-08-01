# Removing the dead CLI surface from `ros-launch-resolve` (issue 0013)

- **Date**: 2026-08-02
- **Status**: Approved, not yet implemented
- **Repo touched**: `ros-launch-resolve` only (`src/ros-launch-resolve`)
- **Issue**: `docs/issues/0013-orphaned-cli-helpers-need-a-disposition.md`

## Problem

`adc33a7` ("depend on ros-launch-resolve; drop the resolve pipeline",
RFC-0060 W3) split the launch-tree layer out of `play_launch`. It copied the
whole CLI module across but wired only four verbs — `resolve`, `dump`,
`contract`, `plot`. Everything serving the verbs that stayed behind
(`run`, `replay`, `check`, `context`) came along with no caller.

Task 6 of the `machine=` removal found four of those, marked them
`#[allow(dead_code)]` with an "UNRESOLVED DISPOSITION" comment, and deferred
the decision. This is that decision.

## What is actually dead

Measured, not inferred.

**`cli/src/config.rs` — the entire file, 615 lines.** A full mirror of
play_launch's monitoring / diagnostics / interception / container-readiness /
composable-loading settings (`RuntimeConfig`, `MonitoringSettings`,
`DiagnosticsSettings`, `InterceptionSettings`,
`ComposableNodeLoadingSettings`, `ContainerReadinessSettings`,
`ResolvedRuntimeConfig`, `load_runtime_config`). Nothing in the crate
references it — the only apparent hit,
`ros_launch_resolve::config::SchedApplyMode` in `options.rs:4`, is the
**library's** `config` module, an unrelated path.

**Four unreachable `Args` structs.** Ten are defined; six appear in the
`Command`/subcommand enums:

| Reachable | Dead |
|---|---|
| `ContractArgs`, `ContractEjectArgs`, `DumpArgs`, `LaunchArgs`, `PlotArgs`, `ResolveArgs` | `CheckArgs`, `ContextArgs`, `ReplayArgs`, `RunArgs` |

The issue annotated only `CheckArgs` and `ReplayArgs`. `ContextArgs` and
`RunArgs` are equally dead and were never marked — worse than being marked,
since unannotated dead code reads as intentional.

Their impls: `impl CheckArgs` (`contract_sources`) and `impl ReplayArgs`
(`model_path`). `ContextArgs` and `RunArgs` have no impls.

**Five `CommonOptions` helpers** serving only those verbs:
`is_monitoring_enabled`, `is_diagnostics_enabled`, `is_web_ui_enabled`,
`parse_web_addr`, and `CommonOptions::contract_sources`.

Roughly 900 lines total.

## Why delete rather than keep

Task 6 correctly refused to delete these as part of a lint cleanup: they are
*orphaned-but-live* — stripping the `allow` yields `dead_code` warnings, not
unresolved references — so removing them was a design decision, not
housekeeping. Three facts now make that decision straightforward.

1. **No consumer can reach them.** `ros-launch-resolve-cli` is `[[bin]]`-only
   with no `[lib]`, and nothing in the tree names it as a dependency. Unlike
   an orphaned library item, there is no plausible future caller — not
   because none exists yet, but because none *can*.
2. **The verbs they serve are staying in `play_launch`.** `play_launch` is
   the Linux frontend: `run` is the `ros2 run`-alike, `replay` is being
   reshaped around the SystemModel, and `check` is expected to fold into
   arguments on `launch`/`run`. None of them is migrating into this crate.
3. **`play_launch` already has live copies.** `src/play_launch/src/cli/`
   carries `config.rs` (595 lines) and `options.rs` (986 lines) defining the
   same concerns, actively maintained. Keeping a second copy nothing runs is
   a drift source, not a safety net.

`play_launch` depends on the **resolve library** (`ros-launch-resolve`) and
the four `ros-launch-manifest-*` crates. nano-ros links only
`ros-launch-manifest-{types,model,sched}` and *spawns* a resolver binary
rather than linking one — deliberately, because the resolver embeds CPython
via pyo3 `auto-initialize` and abi3 cannot unpin an embedding binary's
libpython. Neither consumer touches this CLI crate's Rust surface.

## Explicitly not in scope

- **`launch.rs` stays.** It has no verb handler and is not in the `Command`
  match, but `resolve_launch_file` is called from `common.rs:52` and
  `contract.rs:14`. It is live.
- **`CommonOptions` itself stays.** Only the five helpers above go.
- **`options.rs`'s remaining duplication with play_launch stays.** The two
  files still both define `ResolveArgs`/`DumpArgs`-shaped things. That is
  inherent to two binaries sharing a CLI vocabulary and is not what this
  issue is about.
- **No change to `play_launch`.** Its copies are the live ones.
- **The larger question of where `ros-launch-resolve` should live** — its own
  repo, folded into nano-ros, or split so only the schema crates are shared —
  is open and tracked separately. This work neither settles nor forecloses
  it; deleting unreachable code makes any of those moves smaller.

## Verification

The compiler is the completeness check: with the `#[allow(dead_code)]`
annotations gone along with the code they covered, anything still referencing
a deleted item fails to build, and anything left dead surfaces as a warning
under the crate's `-D warnings` gate.

1. `cargo build --all-targets` — no unresolved references.
2. `cargo clippy --all-targets --all-features -- -D warnings` — clean, with
   **no `#[allow(dead_code)]` reintroduced to achieve it**. If something new
   turns up dead, it is reported, not silenced.
3. `cargo test` in `ros-launch-resolve` — the CLI's own behaviour is covered
   by play_launch's `tests/tests/contract_eject.rs`, which drives the
   `ros-launch-resolve contract eject` binary; `ContractArgs`/
   `ContractEjectArgs` are reachable and untouched, so it must stay green.
4. `just test-all` in `play_launch` — 110/110 integration, confirming the
   submodule pointer bump breaks nothing downstream.
5. `--help` output for the four live verbs is unchanged: the dead `Args`
   never appeared in the `Command` enum, so clap never rendered them.

## Risk

Low, and bounded by the type system. The one way this bites is if a deleted
item were reachable through a path the measurement missed — which the build
in step 1 catches immediately. There is no runtime behaviour to regress:
none of this code executes today.
