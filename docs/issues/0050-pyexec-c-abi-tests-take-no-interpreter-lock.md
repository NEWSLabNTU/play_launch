---
id: 50
title: "`pyexec`'s `c_abi` tests take no interpreter lock, so they fail intermittently under parallel execution"
status: resolved
type: test-defect
severity: medium
---

# 0050 - a flake that looks exactly like issue #0028

**Repo:** `play_launch` at `a6506261`
**Affects:** `src/ros-launch-resolve/parser/crates/pyexec/src/c_abi.rs` (its
test module); compare `.../play_launch_parser/tests/python_tests.rs`, which
holds a guard

## Symptom

`c_abi::tests::exec_file_sees_the_global_parameters_it_was_sent` — the test
that pins issue #0028's ABI 3 → 4 boundary — fails roughly **1 run in 3**
under parallel execution with `KeyError: 'rear_overhang'`, and passes 3 of 3
with `--test-threads=1`.

That error string is #0028's exact symptom, so a flake here reads as a
regression of a shipped fix.

## Cause

`grep -c "python_test_guard\|Mutex" c_abi.rs` returns **0**: that binary's
tests embed and drive a CPython interpreter with no serialisation, while
`python_tests.rs` in the sibling crate takes a guard for the same reason.
Two tests initialising or tearing down the interpreter concurrently race.

## Impact

Worse than an ordinary flake because of what it imitates. #0028's defect was
the Python half losing global parameters across the `dlopen` boundary, and its
signature was `KeyError: 'rear_overhang'` — so an intermittent failure here
sends whoever hits it back into a fixed bug. It also erodes the value of the
loader test, which exists precisely because the in-process backend cannot see
that class of defect.

## Fix direction

Give `c_abi.rs`'s test module the same guard `python_tests.rs` uses rather
than inventing a second mechanism — read that file first; the guard is there
with a comment explaining why. If the guard lives in a place `pyexec` cannot
reach, the smallest honest fix is to move it somewhere both can, not to
duplicate it.

Confirm by running the binary's tests in a loop (10 runs) before and after.

## Provenance

Observed 2026-09-24 while fixing #0041; 1 in ~3 runs parallel, 3/3 serial. Not
caused by that work.

## Resolution 2026-09-25

Reproduced first, at the rate the report gives: the `pyexec` unit-test binary
run 10 times in a row, **5 of 10 failed**, every one of them
`c_abi::tests::exec_file_sees_the_global_parameters_it_was_sent` with
`KeyError: 'rear_overhang'`. After the fix, **0 of 10**, and 0 of a further
20 runs.

The guard `python_tests.rs` holds is a `static Mutex` in an integration-test
file — process-local, and not reachable from another crate. Rather than write
a third copy, the definition now lives in the crate that owns the interpreter:
`play_launch_parser_pyexec::python_test_guard()` (`#[doc(hidden)]`), which
takes the lock and then calls `register()` under it. `play_launch_parser`'s
Python suites already depend on this crate for `register()`, so one definition
covers every test binary that can start an interpreter.

`c_abi.rs` takes it in `call()` — the one helper every test that drives Python
goes through — rather than one line per test that a new test can forget. The
three tests that never touch the interpreter (`free_accepts_null`,
`null_request_is_answered`, `panic_message_reads_both_payload_shapes`) do not
take it and do not need to.

Not done, and worth a follow-up: `python_tests.rs` and
`this_launch_file_tests.rs` still define their own `python_test_guard()` with
the same body. They are separate binaries, so this is duplication rather than
a correctness problem, but they should collapse onto the shared one.
