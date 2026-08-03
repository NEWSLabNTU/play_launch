//! Removed verbs must name their replacement.
//!
//! Asserting only that the command "fails" is what nano-ros issue 0285 did:
//! it failed, with `unrecognized subcommand 'resolve'`, from inside a cmake
//! configure. These tests assert the error is USEFUL.

use play_launch_tests::fixtures;

#[test]
fn replay_names_up_as_its_replacement() {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args(["replay", "system_model.yaml"]);
    let out = cmd.output().expect("failed to run play_launch");

    assert!(!out.status.success(), "`replay` must exit non-zero");
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(err.contains("renamed to `up`"), "must name the rename: {err}");
    assert!(
        err.contains("play_launch up system_model.yaml"),
        "must echo the user's own argument in the new form: {err}"
    );
}

#[test]
fn check_names_both_replacements() {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args(["check", "demo_pkg", "a.launch.xml", "--format", "json"]);
    let out = cmd.output().expect("failed to run play_launch");

    assert!(!out.status.success(), "`check` must exit non-zero");
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(
        err.contains("launch demo_pkg a.launch.xml --check"),
        "must name the gate replacement, echoing the user's args: {err}"
    );
    assert!(
        err.contains("ros-launch-resolve check"),
        "must name where the diagnostics went: {err}"
    );
}

#[test]
fn resolve_names_the_layer_two_binary() {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args(["resolve", "demo_pkg", "a.launch.xml", "-o", "m.yaml"]);
    let out = cmd.output().expect("failed to run play_launch");

    assert!(!out.status.success(), "`resolve` must exit non-zero");
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(
        err.contains("ros-launch-resolve resolve demo_pkg a.launch.xml"),
        "must echo the user's own invocation against the new binary: {err}"
    );
}
