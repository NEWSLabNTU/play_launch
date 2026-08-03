//! `run --check` must SAY that it checked no contracts, and its platform-file
//! verdict must come from actually loading the file.
//!
//! Without these tests, a later refactor can quietly turn `run --check` into
//! an always-pass -- the vacuous-green shape of issues 0008, 0012 and 0014.
//! The "OK" branch had no coverage at all until the 0.9.0 review found it
//! printing OK for `--sched /nonexistent/garbage.yaml`.
//!
//! EVERY test here runs in its own `TempDir` cwd. `run --check` creates
//! `play_log/<timestamp>/` BEFORE the check gate (deferred item M6), and that
//! name is second-granular with no uniquifier
//! (`src/play_launch/src/util/log_dir.rs`). Four invocations landing in the
//! same second in a shared cwd -- which is exactly what nextest's parallel
//! binaries produce -- collide with "unable to create directory ... File
//! exists". A per-test cwd is the fix at this level; the naming scheme is
//! product behaviour with its own consumers and is not a test's to change.
//! Same pattern as `container_events.rs`.

use play_launch_tests::fixtures;
use std::process::Output;
use tempfile::TempDir;

/// Run `play_launch run ... --check` with `args` appended, in a private
/// working directory so the `play_log/<second>` directory it creates before
/// the gate cannot collide with a concurrently-running test's.
///
/// The `TempDir` is returned so the caller keeps it alive; dropping it early
/// would delete the directory out from under the assertions' error messages.
fn run_check(args: &[&std::ffi::OsStr]) -> (Output, TempDir) {
    let env = fixtures::install_env();
    let work = TempDir::new().expect("failed to create tempdir");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work.path());
    cmd.args(["run", "demo_nodes_cpp", "talker", "--check"]);
    cmd.args(args);
    let out = cmd.output().expect("failed to run play_launch");
    (out, work)
}

/// `--sched <path>` for a fixture under `tests/fixtures/sched/`. Absolute:
/// the child runs in a temp cwd, so a relative path would not resolve.
fn sched_fixture(name: &str) -> std::ffi::OsString {
    fixtures::test_workspace_path("sched").join(name).into()
}

#[test]
fn run_check_states_that_no_contracts_were_checked() {
    let (out, _work) = run_check(&[]);

    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        stdout.contains("no contracts checked"),
        "run --check must state that no contracts were checked, else it \
         reports a pass over an empty check.\nstdout: {stdout}\nstderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(out.status.success(), "expected exit 0 with no platform file");
}

/// `resolve_platform_file` only LOCATES a path; on the explicit `--sched`
/// branch it does not even stat it. Reporting "OK" off that alone was a pass
/// over a file that was never opened.
#[test]
fn run_check_fails_on_a_nonexistent_platform_file() {
    let (out, _work) = run_check(&[
        std::ffi::OsStr::new("--sched"),
        std::ffi::OsStr::new("/nonexistent/garbage.yaml"),
    ]);

    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        !out.status.success(),
        "a platform file that does not exist must fail the check.\n\
         stdout: {stdout}\nstderr: {stderr}"
    );
    assert!(
        !stdout.contains("— OK"),
        "must not report OK for a file it never opened.\nstdout: {stdout}"
    );
    assert!(
        stderr.contains("garbage.yaml"),
        "the error must name the offending file.\nstderr: {stderr}"
    );
}

#[test]
fn run_check_fails_on_a_malformed_platform_file() {
    let sched = sched_fixture("malformed.system.posix.yaml");
    let (out, _work) = run_check(&[std::ffi::OsStr::new("--sched"), &sched]);

    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        !out.status.success(),
        "a platform file that does not parse must fail the check.\n\
         stdout: {stdout}\nstderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(
        !stdout.contains("— OK"),
        "must not report OK for a file that does not parse.\nstdout: {stdout}"
    );
}

#[test]
fn run_check_reports_ok_for_a_valid_platform_file() {
    let sched = sched_fixture("minimal.system.posix.yaml");
    let (out, _work) = run_check(&[std::ffi::OsStr::new("--sched"), &sched]);

    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        out.status.success(),
        "a valid platform file must pass the check.\nstdout: {stdout}\nstderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(
        stdout.contains("minimal.system.posix.yaml") && stdout.contains("OK"),
        "must report the platform file it loaded.\nstdout: {stdout}"
    );
}
