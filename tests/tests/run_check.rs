//! `run --check` must SAY that it checked no contracts.
//!
//! Without this test, a later refactor can quietly turn `run --check` into
//! an always-pass -- the vacuous-green shape of issues 0008, 0012 and 0014.

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
