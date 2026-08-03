//! `run --check` must SAY that it checked no contracts, and its platform-file
//! verdict must come from actually loading the file.
//!
//! Without these tests, a later refactor can quietly turn `run --check` into
//! an always-pass -- the vacuous-green shape of issues 0008, 0012 and 0014.
//! The "OK" branch had no coverage at all until the 0.9.0 review found it
//! printing OK for `--sched /nonexistent/garbage.yaml`.

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

/// `resolve_platform_file` only LOCATES a path; on the explicit `--sched`
/// branch it does not even stat it. Reporting "OK" off that alone was a pass
/// over a file that was never opened.
#[test]
fn run_check_fails_on_a_nonexistent_platform_file() {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args([
        "run",
        "demo_nodes_cpp",
        "talker",
        "--check",
        "--sched",
        "/nonexistent/garbage.yaml",
    ]);
    let out = cmd.output().expect("failed to run play_launch");

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
    let env = fixtures::install_env();
    let sched = fixtures::test_workspace_path("sched").join("malformed.system.posix.yaml");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args(["run", "demo_nodes_cpp", "talker", "--check", "--sched"])
        .arg(&sched);
    let out = cmd.output().expect("failed to run play_launch");

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
    let env = fixtures::install_env();
    let sched = fixtures::test_workspace_path("sched").join("minimal.system.posix.yaml");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args(["run", "demo_nodes_cpp", "talker", "--check", "--sched"])
        .arg(&sched);
    let out = cmd.output().expect("failed to run play_launch");

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
