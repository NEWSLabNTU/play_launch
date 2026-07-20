# Phase 47: CLI Flag Cleanup + Hard record.json Removal

**Status:** ✅ Complete (2026-07-20) — A1–A3 and B1–B6 all shipped.
`record.json` is no longer a user-facing artifact anywhere; `dump`/`resolve`/
`replay`/`launch` are model-only. See `.superpowers/sdd/p47-wB1-report.md`
(B1, the parity-to-models gate) and `.superpowers/sdd/p47-wB-report.md`
(B2–B6, the hard removal) for implementation evidence.
**Builds on:** Phase 46 (unified SystemModel; `record.json` retired to deprecated dev/compat).
**Follow-ups source:** `.superpowers/sdd/p46-w5-report.md`, `p46-w6-report.md`, `.superpowers/sdd/p46-progress.md` (Phase 46 tracked follow-ups).

## Overview

Two independent tracks, both cleanup after Phase 46's unification:
1. **CLI flag cleanup** — fix the flag-ordering gotcha (flags after a
   `KEY:=VALUE` launch arg are silently swallowed) and stale/fictional flags.
2. **Hard record.json removal** — remove the deprecated `record.json`
   read/write/compat surface Phase 46 deliberately kept, and move the
   parser-parity tooling off `record.json` onto model comparison.

Plus a smaller carried follow-up (§C).

## A. CLI flag cleanup

**Root cause** (`cli/options.rs`): `launch_arguments: Vec<String>` on
`Launch`/`Replay`/`Resolve`/`Dump`/`Check` uses `#[arg(trailing_var_arg = true)]`,
which captures EVERYTHING after the first positional — including anything that
looks like a flag — into the Vec. So `dump launch pkg file --output x.yaml`
swallows `--output x.yaml` into `launch_arguments`, and `resolve file --sched s
mode:=v` mis-parses. Documented workaround today (Phase 46.6): put flags
BEFORE the `KEY:=VALUE` args. This is a real ergonomics trap.

- **47.A1** Fix the capture so flags work in any position. Options (pick one,
  document): (a) drop `trailing_var_arg`, validate `launch_arguments` entries
  match `KEY:=VALUE` (reject `--flags` with a clear error suggesting the flag
  go before, or `--` separator); (b) require a `--` separator before launch
  args (explicit, unambiguous, but a UX change); (c) a custom parser that
  splits `KEY:=VALUE` from flags. (a) is likely cleanest — launch args have a
  strict `KEY:=VALUE` shape, so non-matching tokens are unambiguously flags.
- **47.A2** Remove/fix stale + fictional CLI flags found in Phase 46.6
  (documented but not-existing): `--enable-monitoring`,
  `--wait-for-service-ready`, and a `demo_nodes_cpp` launch path in help/docs
  that doesn't resolve. Sweep `cli-interface.md` + `options.rs` help + README
  for other fictional flags; make help match reality.
- **47.A3** Tests: a CLI-parse test that flags after `KEY:=VALUE` parse
  correctly (or error clearly) for each affected subcommand; the help examples
  all run as written (extend the Phase 46.6 command-verification).

## B. Hard record.json removal

Phase 46 left a deliberate deprecated/dev surface. Hard removal deletes it —
**non-additive, breaks existing `record.json` files + the current parity
tooling**, so it needs the parity tooling moved FIRST.

**The remaining `record.json` surface** (Phase 46.5 report): `dump --format
record`, `replay --input-file record.json` (deprecated), `resolve --record
<path>` (reuse mode), `launch`'s internal record round-trip, and the
parser-parity tooling (`scripts/compare_records.py`, `scripts/compare_parsers.sh`,
the `dump-python`/`dump-rust`/`compare-dumps` recipes in 5 fixture justfiles).
`record.json` read/write touches 8 command files (`commands/{contract,run,
replay,dump,launch,context,resolve,manifest}.rs`).

- **47.B1 (prerequisite): move parser-parity off record.json onto the model.**
  The 100% cross-parser fidelity check currently compares two `record.json`s
  (`compare_records.py`). Switch it to compare two MODELS (structure
  equivalence — `compare_models.py` or extend compare_records to accept
  models), so parity survives record.json removal. Keep the fail-loud
  stale-Python guard (Phase 46.5) on the model path. Verify Autoware
  rust-vs-python parity (34/15/70) still passes on models.
- **47.B2** Remove `dump --format record` (once B1 no longer needs it) and the
  `<out>.record.json` write paths.
- **47.B3** Remove the deprecated `replay --input-file record.json` read path
  (hard cut — existing record.json files become unreplayable; that's the
  point). Decide `resolve --record <path>` reuse mode: keep (reads an existing
  record to skip re-parse) or remove — if the input is always a launch file
  now, remove it; if record-reuse has value, keep reading but note it's the
  last record.json consumer.
- **47.B4** `launch`'s internal record round-trip → in-memory model only (no
  record.json written to disk during launch).
- **47.B5** Drop dead LaunchDump-only plumbing no longer reachable
  (`file_data`, `variables` were never in the model; the LaunchDump struct
  stays as the parser's in-memory intermediate — that's fine, it's not the
  artifact anymore). Confirm nothing still WRITES a record.json to disk on the
  primary paths.
- **47.B6** Docs: drop the deprecated-compat mentions; `record.json` is an
  internal parser intermediate only (if kept for reuse), not a user artifact.

## C. Carried follow-up (smaller, optional)

- **47.C1** Web-UI launch-tree scope-map + chain-colocation are `launch_dump`-only
  → empty on pure `replay --model`. Model-source them (the model has
  `structure.scopes` + `execution.sched.chains`) so the UI works on model
  replay, or accept-empty + document. Degrades to empty JSON today (never
  errors), so this is polish.

## Order

A and B are independent (do in either order). Within B: **B1 (parity → model)
is the gate** — it must land before B2/B3 remove the record.json surface the
parity tooling depends on. C is optional polish.

## Out of scope

nano-ros's consumption of the model (their track); the contract/sched layers
(Phases 44/45 stand); the model schema (Phase 46 additive fields stand).
