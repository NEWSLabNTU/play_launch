//! Bounded runs and per-run launch prefixes (phase 106 W3).
//!
//! `--run-for` ends a launch after a time; `--fail-on` makes it a test whose
//! exit code is the verdict (4 when a condition held); `--prefix` runs a
//! node under a command without editing the launch file.

use std::{
    process::{Command, Stdio},
    time::{Duration, Instant},
};

use play_launch_tests::fixtures;

fn launch(name: &str) -> String {
    fixtures::test_workspace_path("simple_test")
        .join("launch")
        .join(name)
        .to_str()
        .unwrap()
        .to_string()
}

/// Run play_launch to its end (bounded by `limit`); returns the exit code,
/// stdout and the run directory.
fn run(args: &[&str], limit: Duration) -> (Option<i32>, String, tempfile::TempDir) {
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().unwrap();
    let mut cmd: Command = fixtures::play_launch_cmd(&env);
    cmd.current_dir(tmp.path());
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
    ]);
    cmd.args(args);
    cmd.env_remove("RUST_LOG");
    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());
    let mut child = cmd.spawn().unwrap();
    let start = Instant::now();
    let status = loop {
        if let Some(s) = child.try_wait().unwrap() {
            break s;
        }
        if start.elapsed() > limit {
            let _ = child.kill();
            panic!("play_launch {args:?} did not end within {limit:?}");
        }
        std::thread::sleep(Duration::from_millis(200));
    };
    let out = child.wait_with_output().unwrap();
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    (status.code(), text, tmp)
}

fn verdict(tmp: &tempfile::TempDir) -> serde_json::Value {
    let info: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(tmp.path().join("play_log/latest/run_info.json")).unwrap(),
    )
    .unwrap();
    info["verdict"].clone()
}

#[test]
fn run_for_ends_the_launch_and_records_a_pass() {
    let (code, text, tmp) = run(
        &[
            &launch("pure_nodes.launch.xml"),
            "--run-for",
            "4s",
            "--fail-on",
            "exit:/**",
        ],
        Duration::from_secs(40),
    );
    assert_eq!(code, Some(0), "{text}");
    assert!(text.contains("--run-for 4s: time is up"), "{text}");
    let v = verdict(&tmp);
    assert_eq!(v["passed"], true, "{v}");
    assert_eq!(v["run_for_s"], 4.0, "{v}");
}

#[test]
fn a_held_condition_fails_the_run_with_its_own_exit_code() {
    // The crasher exits about a second in; --fail-fast ends the run there,
    // long before --run-for would.
    let start = Instant::now();
    let (code, text, tmp) = run(
        &[
            &launch("crash_loop.launch.xml"),
            "--run-for",
            "60s",
            "--fail-on",
            "exit:/**",
            "--fail-fast",
        ],
        Duration::from_secs(40),
    );
    assert_eq!(code, Some(4), "{text}");
    assert!(
        start.elapsed() < Duration::from_secs(30),
        "--fail-fast did not end the run"
    );
    assert!(text.contains("the run failed: exit:/**:"), "{text}");
    let v = verdict(&tmp);
    assert_eq!(v["passed"], false, "{v}");
    assert!(
        v["failure"].as_str().unwrap().contains("exited (signal"),
        "{v}"
    );
}

#[test]
fn prefix_runs_the_selected_node_under_a_command() {
    let (code, text, tmp) = run(
        &[
            &launch("pure_nodes.launch.xml"),
            "--prefix",
            "/pure_test/talker=env PL_PREFIX_PROBE=1",
            "--run-for",
            "3s",
        ],
        Duration::from_secs(40),
    );
    assert_eq!(code, Some(0), "{text}");
    let node = tmp.path().join("play_log/latest/node");
    let talker = std::fs::read_to_string(node.join("talker/cmdline")).unwrap();
    // The cmdline file shell-quotes each word (`PL_PREFIX_PROBE'=1'`).
    assert!(talker.starts_with("env PL_PREFIX_PROBE"), "{talker}");
    let listener = std::fs::read_to_string(node.join("listener/cmdline")).unwrap();
    assert!(!listener.contains("PL_PREFIX_PROBE"), "{listener}");
}

#[test]
fn bad_conditions_and_prefixes_are_refused_before_anything_runs() {
    for args in [
        vec!["--fail-on", "nope"],
        vec!["--fail-fast"],
        vec!["--prefix", "/pure_test/talker"],
        vec!["--run-for", "soon"],
    ] {
        let mut all = vec![launch("pure_nodes.launch.xml")];
        all.extend(args.iter().map(|s| s.to_string()));
        let refs: Vec<&str> = all.iter().map(|s| s.as_str()).collect();
        let (code, text, tmp) = run(&refs, Duration::from_secs(20));
        assert_eq!(code, Some(1), "{args:?}: {text}");
        assert!(!tmp.path().join("play_log").exists(), "{args:?} ran");
    }
}
