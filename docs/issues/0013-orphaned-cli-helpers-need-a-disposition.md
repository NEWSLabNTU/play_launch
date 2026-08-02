---
id: 13
title: "Four orphaned CLI helpers in ros-launch-resolve are annotated UNRESOLVED DISPOSITION and need a real decision"
status: resolved
type: cleanup
severity: low
---

# 0013 — Orphaned `ros-launch-resolve` CLI helpers await a disposition

**Repo:** `ros-launch-resolve` (`src/ros-launch-resolve`)
**Affects:** `cli/src/config.rs`, `cli/src/options.rs`
**Found:** Task 6 of the `machine=` removal; escalated and ruled on
2026-07-31

## Summary

Four items in the extracted CLI have no caller:

- `config.rs`'s `RuntimeConfig` subsystem
- `CheckArgs::contract_sources`
- `ReplayArgs::model_path`
- five `CommonOptions` helpers

They are annotated `#[allow(dead_code)]` with an "UNRESOLVED DISPOSITION"
comment at each site naming what the item is, why it has no caller (the
RFC-0060 verb split left them behind when `main.rs`'s `Command` enum shrank),
and that removing them is a separate decision.

## Why they were not simply deleted

An empirical check distinguished them from genuinely dead code. Stripping each
`#[allow(dead_code)]` and building produced **zero compile errors** — only
`dead_code` warnings. Contrast `forward_state_events_and_wait`, deleted in the
same pass: it referenced `crate::member_actor` and `crate::web`, modules that
do not exist in that crate, and a `runtime` Cargo feature that is not
declared. It could never have compiled under any configuration.

So these four are *orphaned-but-live* — real, compilable code with no caller
yet — and deleting them would be a design decision smuggled into a lint
cleanup. The human ruling on 2026-07-31 was explicitly that the question stay
visible rather than be silenced.

## The decision to make

For each, one of:

1. **Wire it up** — it was meant to be called and the RFC-0060 split lost the
   call site.
2. **Relocate it** — it belongs to `play_launch` rather than the extracted
   CLI.
3. **Delete it** — it is genuinely obsolete.

`RuntimeConfig` is the one worth deciding first: it is a whole subsystem, not
a stray method, and its fate probably determines the other three.

## Constraint

The crate's gate is `cargo clippy --all-targets --all-features -- -D
warnings`, so the annotations cannot simply be removed to "let the warnings
show" — that turns them into hard errors. Any resolution has to either wire,
move, or delete.

## Resolution (2026-08-02) — deleted

Decision: **delete**. Unlike Task 6's judgement call, this was not close.
`ros-launch-resolve-cli` is `[[bin]]`-only with no `[lib]`, and nothing in
the tree names it as a dependency — so there is no plausible future caller,
not because none exists yet but because none *can* link it. The verbs these
helpers served (`run`/`replay`/`check`/`context`) stay in `play_launch`, the
Linux frontend, which already carries live copies (`cli/config.rs` 595 lines,
`cli/options.rs` 986 lines).

**Measurement corrected this issue.** It named four items; there were more:

- `cli/src/config.rs` — 615 lines, referenced NOWHERE in the crate. The one
  apparent hit was `ros_launch_resolve::config::SchedApplyMode`, the
  *library's* config module on an unrelated path.
- **Four** unreachable `Args` structs, not two. Ten are defined, six appear
  in the `Command`/subcommand enums. `CheckArgs` and `ReplayArgs` were
  annotated; **`ContextArgs` and `RunArgs` were equally dead and unmarked** —
  worse than being marked, since unannotated dead code reads as intentional.
- Removing the five `CommonOptions` helpers left that `impl` block EMPTY, so
  the block, its doc comment and its `#[allow(dead_code)]` went too. That one
  surfaced only because the verification required clippy to pass with no
  `allow` reintroduced.

Kept deliberately: `launch.rs` (no verb handler, but `resolve_launch_file` is
called from `common.rs:52` and `contract.rs:14` — live), and `CommonOptions`
itself.

**1934 → 1033 lines (−47%). No `#[allow(dead_code)]` remains anywhere in the
crate**, and `clippy -D warnings` passes without silencing anything.
995/995 crate tests; all four verbs unchanged in `--help`; play_launch's
`contract_eject` + `rt_workspace` (which drive this binary) 25/25;
`just test-all` 462/462 + 110/110.

Spec: `docs/superpowers/specs/2026-08-02-resolve-cli-dead-surface-design.md`
Commit: `ros-launch-resolve` `a996e97`.

**Left open:** where `ros-launch-resolve` should ultimately live — its own
repo, folded into nano-ros, or split so only the schema crates are shared.
This work neither settles nor forecloses it. Relevant facts gathered while
deciding: nano-ros LINKS only `ros-launch-manifest-{types,model,sched}`
(pure schema, no Python) but its `nros-launch-resolve` binary links our
`resolve` lib and `play_launch_parser` directly — so it takes the pyo3
dependency at BUILD time and avoids it only in the shipped `nros`. The
Python isolation is a link-boundary property of their final binary, not of
the crate split.
