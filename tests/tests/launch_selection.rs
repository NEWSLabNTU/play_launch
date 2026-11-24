//! Running part of a launch tree (phase 106 W2).
//!
//! `selection.launch.xml` declares a container and includes
//! `selection_part.launch.xml` twice (`part:=a`, `part:=b`), each inclusion
//! adding a plain node and a composable loaded into that container, plus
//! `pure_nodes.launch.xml`. The whole tree is resolved; only the selected
//! members start, and the rest are Excluded, which startup completion and the
//! shortfall report must not count.

use std::{
    path::{Path, PathBuf},
    process::{Command, Stdio},
    time::{Duration, Instant},
};

use play_launch_tests::{fixtures, process::ManagedProcess};

fn launch_file() -> String {
    fixtures::test_workspace_path("simple_test")
        .join("launch/selection.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

fn wait_for(path: &Path, needle: &str, timeout: Duration) -> String {
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
        std::thread::sleep(Duration::from_millis(250));
    }
}

/// Start the fixture with `extra` arguments; returns the work dir and the
/// stdout path once startup has completed.
fn start(extra: &[&str]) -> (tempfile::TempDir, PathBuf, ManagedProcess) {
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().unwrap();
    let stdout = tmp.path().join("stdout.log");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(tmp.path());
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        &launch_file(),
    ]);
    cmd.args(extra);
    cmd.env_remove("RUST_LOG");
    cmd.stdout(Stdio::from(std::fs::File::create(&stdout).unwrap()));
    cmd.stderr(Stdio::from(
        std::fs::File::create(tmp.path().join("stderr.log")).unwrap(),
    ));
    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    wait_for(&stdout, "Startup complete", Duration::from_secs(60));
    (tmp, stdout, proc)
}

fn run_dir(work: &Path) -> PathBuf {
    work.join("play_log/latest")
}

fn spawned(work: &Path, node: &str) -> bool {
    run_dir(work).join("node").join(node).join("pid").exists()
}

#[test]
fn only_starts_the_selected_inclusion_and_the_container_it_needs() {
    let (tmp, stdout, _proc) = start(&["--only", "selection_part.launch.xml[part=a]"]);
    let text = std::fs::read_to_string(&stdout).unwrap();

    // The container came with its composable, and only `talker_a` was
    // declared to it, so startup is complete with no shortfall even though
    // `talker_b` targets the same container.
    assert!(
        text.contains(
            "Startup complete: all nodes ready (nodes 1/1, containers 1/1, composable 1/1)"
        ),
        "{text}"
    );
    assert!(!text.contains("shortfall"), "{text}");
    assert!(
        text.contains("kept /sel/sel_container for 1 composable(s)"),
        "{text}"
    );

    assert!(spawned(tmp.path(), "listener_a"));
    for excluded in ["listener_b", "talker", "listener"] {
        assert!(!spawned(tmp.path(), excluded), "{excluded} was spawned");
    }

    let info: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(run_dir(tmp.path()).join("run_info.json")).unwrap(),
    )
    .unwrap();
    let sel = &info["selection"];
    assert_eq!(
        sel["only"][0], "scope:selection_part.launch.xml[part=a]",
        "{sel}"
    );
    assert_eq!(sel["kept"], 3, "{sel}");
    assert_eq!(sel["added"][0], "/sel/sel_container", "{sel}");
    let excluded: Vec<&str> = sel["excluded"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_str().unwrap())
        .collect();
    assert_eq!(
        excluded,
        [
            "/pure_test/talker",
            "/pure_test/listener",
            "/sel/listener_b",
            "/sel/talker_b"
        ]
    );
}

#[test]
fn exclude_removes_a_part_and_start_none_starts_nothing() {
    let (tmp, stdout, proc) = start(&["--exclude", "pure_nodes.launch.xml"]);
    let text = std::fs::read_to_string(&stdout).unwrap();
    assert!(
        text.contains(
            "Startup complete: all nodes ready (nodes 2/2, containers 1/1, composable 2/2)"
        ),
        "{text}"
    );
    assert!(!spawned(tmp.path(), "talker") && !spawned(tmp.path(), "listener"));
    drop(proc);

    let (tmp, stdout, _proc) = start(&["--start", "none"]);
    let text = std::fs::read_to_string(&stdout).unwrap();
    assert!(text.contains("0 of 7 members kept"), "{text}");
    assert!(
        text.contains("(nodes 0/0, containers 0/0, composable 0/0)"),
        "{text}"
    );
    assert!(!run_dir(tmp.path()).join("node/listener_a/pid").exists());
}

#[test]
fn a_file_included_twice_must_say_which_inclusion() {
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().unwrap();
    let mut cmd: Command = fixtures::play_launch_cmd(&env);
    cmd.current_dir(tmp.path());
    cmd.args([
        "launch",
        "--disable-all",
        &launch_file(),
        "--only",
        "selection_part.launch.xml",
    ]);
    let out = cmd.output().unwrap();
    assert!(!out.status.success());
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(err.contains("is included 2 times; say which"), "{err}");
    assert!(
        err.contains("scope:selection_part.launch.xml[part=a]"),
        "{err}"
    );
    assert!(
        err.contains("scope:selection_part.launch.xml[part=b]"),
        "{err}"
    );
    // Refused before anything ran: no run directory was created.
    assert!(!tmp.path().join("play_log").exists());
}

#[test]
fn resolve_explain_prints_the_marked_tree_and_writes_no_model() {
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().unwrap();
    let mut cmd: Command = fixtures::play_launch_cmd(&env);
    cmd.current_dir(tmp.path());
    cmd.args([
        "resolve",
        &launch_file(),
        "--exclude",
        "/sel/talker_b",
        "--explain",
    ]);
    let out = cmd.output().unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let text = String::from_utf8_lossy(&out.stdout);
    assert!(
        text.contains("selection: 6 of 7 members kept, 1 excluded"),
        "{text}"
    );
    assert!(
        text.contains("[~] selection_part.launch.xml  (part=b)  1/2"),
        "{text}"
    );
    assert!(text.contains("[ ] /sel/talker_b"), "{text}");
    assert!(!tmp.path().join("system_model.yaml").exists());

    // Without --explain, a selection on `resolve` points at `export`.
    let mut cmd: Command = fixtures::play_launch_cmd(&env);
    cmd.current_dir(tmp.path());
    cmd.args(["resolve", &launch_file(), "--exclude", "/sel/talker_b"]);
    let out = cmd.output().unwrap();
    assert!(!out.status.success());
    assert!(String::from_utf8_lossy(&out.stderr).contains("play_launch export"));
}
