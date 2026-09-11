//! Issue #0023 — declared against loaded, per container, at startup-complete.
//!
//! The golf cart launch lost six of one container's sixteen composables and
//! printed a total. The launcher must now say WHICH container fell short and
//! name every missing composable by FQN, at `error` level, at the terminal's
//! default verbosity — and the same line must land in the bundle's own log.

use std::{
    process::Stdio,
    time::{Duration, Instant},
};

use play_launch_tests::{fixtures, process::ManagedProcess};

fn missing_plugin_launch() -> String {
    fixtures::test_workspace_path("container_events")
        .join("launch/missing_plugin.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

fn wait_for(path: &std::path::Path, needle: &str, timeout: Duration) -> String {
    let start = Instant::now();
    loop {
        let text = std::fs::read_to_string(path).unwrap_or_default();
        if text.contains(needle) {
            return text;
        }
        assert!(
            start.elapsed() < timeout,
            "{} did not contain {needle:?} within {timeout:?}; content:\n{text}",
            path.display()
        );
        std::thread::sleep(Duration::from_millis(500));
    }
}

/// One composable loads, one names a plugin that does not exist. Startup
/// completes with a shortfall, and the report names the container and the
/// missing FQN — not just a count.
#[test]
fn test_shortfall_names_the_container_and_the_missing_fqn() {
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let work_dir = tmp.path().to_path_buf();
    let stdout = work_dir.join("stdout.log");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--container-mode",
        "isolated",
        &missing_plugin_launch(),
    ]);
    cmd.stdout(Stdio::from(std::fs::File::create(&stdout).unwrap()));
    cmd.stderr(Stdio::from(
        std::fs::File::create(work_dir.join("stderr.log")).unwrap(),
    ));
    // Default verbosity on purpose: a shortfall is an ERROR, and it must be
    // visible without anyone having asked for debug output beforehand.
    cmd.env_remove("RUST_LOG");
    let _proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");

    let text = wait_for(&stdout, "Startup shortfall in", Duration::from_secs(90));

    let line = text
        .lines()
        .find(|l| l.contains("Startup shortfall in"))
        .unwrap();
    assert!(line.contains("ERROR"), "not at error level: {line}");
    assert!(
        line.contains("/shortfall_container"),
        "container not named: {line}"
    );
    assert!(
        line.contains("1/2 composables loaded — 1 MISSING"),
        "declared/loaded arithmetic wrong: {line}"
    );
    assert!(
        line.contains("/system/ghost (failed:"),
        "missing composable not named by FQN with its state: {line}"
    );
    assert!(
        !line.contains("/talker"),
        "a loaded composable was reported missing: {line}"
    );

    // The launch must NOT have claimed every node was ready.
    assert!(
        !text.contains("Startup complete: all nodes ready"),
        "reported all ready over a shortfall:\n{text}"
    );
    // The machine-readable summary carries the same reconciliation.
    let summary = text
        .lines()
        .find(|l| l.contains("STARTUP_SUMMARY"))
        .expect("STARTUP_SUMMARY line");
    let json: serde_json::Value = serde_json::from_str(&summary[summary.find('{').unwrap()..])
        .expect("STARTUP_SUMMARY is JSON");
    let shortfall = json["composable_shortfall"]
        .as_array()
        .expect("composable_shortfall array");
    assert_eq!(shortfall.len(), 1, "{json}");
    assert_eq!(shortfall[0]["container"], "/shortfall_container");
    assert_eq!(shortfall[0]["declared"], 2);
    assert_eq!(shortfall[0]["loaded"], 1);
    assert_eq!(shortfall[0]["missing"][0], "/system/ghost");

    // And the bundle's own log has the verdict too (issue #0023, part 1).
    let bundle_log = wait_for(
        &work_dir.join("play_log/latest/play_launch.log"),
        "Startup shortfall in",
        Duration::from_secs(10),
    );
    assert!(bundle_log.contains("/system/ghost"), "{bundle_log}");
}
