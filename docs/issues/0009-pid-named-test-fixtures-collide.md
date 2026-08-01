---
id: 9
title: "Test fixture files are named by process id, so concurrently-run tests in one process clobber each other"
status: resolved
type: bug
severity: low
---

# 0009 — PID-named test fixtures collide under `cargo test`

**Repo:** `play_launch_parser` (`src/ros-launch-resolve/parser`)
**Affects:** `crates/play_launch_parser/tests/attr_strictness.rs`
**Found:** independently by four separate agents during the `machine=`
removal work (2026-07-31)

## Summary

`attr_strictness.rs` writes its scratch launch files to a path derived from
`std::process::id()`:

```rust
let path = dir.join(format!("attr_strict_{}.launch.xml", std::process::id()));
```

`cargo test` runs all tests in a single process on multiple threads, so every
test in the binary computes the **same** filename. Tests then overwrite each
other's fixture between write and parse, producing failures that depend on
thread scheduling.

`cargo nextest` runs one process per test, so each gets a distinct PID and the
collision does not occur. `just test-rust` and `just quality` both use nextest,
which is why the project's own gates are green.

## Why it matters more than it looks

The failure is *scheduling-dependent*, so it reads as flakiness rather than a
naming bug. During the `machine=` work this cost real investigation time: a
`cargo test` run in `ros-launch-resolve` reported 10 failures that vanished
under nextest, and had to be chased down before it was clear nothing was
actually broken. Anyone who reflexively runs `cargo test --test
attr_strictness` will hit it again.

This branch also added nine more tests using the same pattern, so the collision
surface grew.

## Reproduction

```sh
cd src/ros-launch-resolve/parser
cargo test --test attr_strictness      # intermittent failures
cargo nextest run --test attr_strictness   # always clean
```

## Fix direction

Use `tempfile::NamedTempFile` (or `TempDir`), which is already the pattern in
`ir_tests.rs` in the same crate — it allocates a unique path per call with no
PID assumption. Roughly a three-line change per helper.

An alternative that preserves readable filenames is a process-wide
`AtomicUsize` counter appended to the PID, but `tempfile` is simpler and
already a dependency.

## Resolution (2026-08-01)

Both helpers now share one `parse_scratch()` built on `tempfile::Builder`,
which allocates a unique path per call and removes it on drop. `tempfile_in`
keeps the file under the repo's own `tmp/` per project convention, and the
suffix is preserved because `parse_launch_file` dispatches XML vs YAML on the
extension.

Verified by before/after: `cargo test --test attr_strictness` gave 9 failed,
then 10 failed on a re-run (the varying count IS the race); after the change,
20 passed repeatably. Parser commit `37bef77`.
