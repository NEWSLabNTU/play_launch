---
id: 98
title: "play_log fills the disk: diagnostics.csv is most of every run, and no run is ever removed"
status: resolved
type: bug
severity: medium
---

# 0098 - play_log fills the disk

**Repo:** `play_launch` 0.39.0
**Reported:** 2026-11-18, from swb (Autoware ego stack) and golf (Autoware
planning simulator) on request

## What happened

Two teams measured their runs:

| | swb, ~90 nodes, 2.5 min | golf, 42 min |
|---|---|---|
| `diagnostics.csv` | 27.4 MB, 88% (~650 MB/h) | 380 MB, 97% (2.55 M rows, ~0.5 GB/h) |
| `node/` (out, err, metrics) | 2.0 MB | 9.7 MB |
| `play_launch.log` | 0.94 MB | 1.3 MB |

Neither had ever opened `diagnostics.csv`. Both debug from `node/*/out` and
`err` and `play_launch.log`; golf reads `/diagnostics` live or from bags.

And no run is ever removed: golf had 68 runs (3.7 GB), this repository's
Autoware fixture 251 (1 GB), and swb prunes with its own script.

## Cause

The diagnostics monitor wrote each accepted status with one row per
key/value, and accepted every status once its 100 ms debounce had passed.
Autoware publishes at about 10 Hz with numeric values, so every status was
written, and the file grew with node count times rate.

## Fix

- `diagnostics.record: changes | all | off`, default `changes`: a row when
  a diagnostic first appears and at each level change, with `key` and
  `value` empty. `all` is the old record. The registry the web UI and
  Health read is fed the same in every mode.
- `play_launch log list` shows each run's size by kind of file, and
  `play_launch log prune` removes runs selected by `--before` (date or
  age), `--keep` or `--max-total-mb`. With `--class` it removes only some
  kinds of file and keeps the runs. It prints the plan and removes nothing
  without `--yes` (`util/play_log.rs`, `commands/log.rs`).
- At the start of a run, a log directory over `play_log.remind_mb` (5000
  by default) is reported with the command to run.
- `play_log.keep_runs` and `play_log.max_total_mb`, unset by default,
  remove the oldest earlier runs before a new run's directory is created.
- Everywhere, only directories named like a run are candidates, never
  `latest` or anything else, and a run whose `run_info.json` names a live
  play_launch is skipped.

Kept as they were, since teams use them or they are small:
`node/*/out` and `err` (the main debugging source), `play_launch.log` at
debug (about 1 MB a run), `metrics.csv` and `system_stats.csv` (tens of MB
a day). `interception/events.jsonl` has been off by default since 0.15,
after it wrote 169 GB in 50 minutes on swb.
