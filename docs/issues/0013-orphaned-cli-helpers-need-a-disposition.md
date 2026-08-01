---
id: 13
title: "Four orphaned CLI helpers in ros-launch-resolve are annotated UNRESOLVED DISPOSITION and need a real decision"
status: open
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
