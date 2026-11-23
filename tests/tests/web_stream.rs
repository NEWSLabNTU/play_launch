//! The web UI's global stream carries the header (phase 105 W3, #0111).
//!
//! The header's member counts used to be fetched once per connection, so
//! "N/M nodes" froze while nodes stopped and crashed; the issue and
//! recording badges were polled; the diagnostics badges were known only
//! while the Diagnostics page was open; and the system metrics needed a
//! second connection. All of them are named events on
//! `/api/state/updates` now, sent on connect and when they change.

use play_launch_tests::{fixtures, process::ManagedProcess};
use std::{
    process::{Command, Stdio},
    time::{Duration, Instant},
};

/// `(event name, data)` pairs from an SSE body; unnamed events are
/// `message`.
fn parse_sse(body: &str) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for block in body.split("\n\n") {
        let mut name = "message".to_string();
        let mut data = String::new();
        for line in block.lines() {
            if let Some(v) = line.strip_prefix("event:") {
                name = v.trim().to_string();
            } else if let Some(v) = line.strip_prefix("data:") {
                data.push_str(v.trim_start());
            }
        }
        if !data.is_empty() {
            out.push((name, data));
        }
    }
    out
}

fn nodes_running(data: &str) -> Option<u64> {
    serde_json::from_str::<serde_json::Value>(data).ok()?["nodes_running"].as_u64()
}

#[test]
fn the_global_stream_carries_the_header_and_follows_a_stop() {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("simple_test").join("launch/pure_nodes.launch.xml");
    let work_dir = tempfile::TempDir::new().unwrap();
    let mut cmd = fixtures::play_launch_cmd(&env);
    let port = fixtures::web_port(&cmd);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--web-addr",
        &format!("127.0.0.1:{port}"),
        launch.to_str().unwrap(),
    ]);
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());
    let _proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    let base = format!("http://127.0.0.1:{port}");

    // Both nodes running, by the summary the old endpoint still serves.
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        let out = Command::new("curl")
            .args(["-s", "--max-time", "2", &format!("{base}/api/health")])
            .output()
            .unwrap();
        if nodes_running(&String::from_utf8_lossy(&out.stdout)) == Some(2) {
            break;
        }
        assert!(Instant::now() < deadline, "the two nodes never ran");
        std::thread::sleep(Duration::from_millis(200));
    }

    // Listen for 8 s; stop the talker 3 s in.
    let stream = Command::new("curl")
        .args([
            "-sN",
            "--max-time",
            "8",
            &format!("{base}/api/state/updates"),
        ])
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    std::thread::sleep(Duration::from_secs(3));
    let stop = Command::new("curl")
        .args([
            "-s",
            "-o",
            "/dev/null",
            "-w",
            "%{http_code}",
            "-X",
            "POST",
            &format!("{base}/api/nodes/node:%2Fpure_test%2Ftalker/stop"),
        ])
        .output()
        .unwrap();
    assert!(
        String::from_utf8_lossy(&stop.stdout).starts_with('2'),
        "stop refused: {}",
        String::from_utf8_lossy(&stop.stdout)
    );
    let out = stream.wait_with_output().unwrap();
    let events = parse_sse(&String::from_utf8_lossy(&out.stdout));

    for name in [
        "summary",
        "issue_counts",
        "diagnostic_counts",
        "recording",
        "activity",
        "metrics",
    ] {
        assert!(
            events.iter().any(|(n, _)| n == name),
            "no `{name}` event on the stream: {events:?}"
        );
    }
    let running: Vec<u64> = events
        .iter()
        .filter(|(n, _)| n == "summary")
        .filter_map(|(_, d)| nodes_running(d))
        .collect();
    assert_eq!(
        running.first(),
        Some(&2),
        "the first summary is the state on connect"
    );
    assert_eq!(
        running.last(),
        Some(&1),
        "the summary must follow the stop with no other request: {running:?}"
    );
    // Sent on change, not every tick: two values, two events.
    assert_eq!(running.len(), 2, "a summary per change only: {running:?}");
}

#[test]
fn a_quiet_diagnostics_stream_sends_its_table_once() {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("simple_test").join("launch/pure_nodes.launch.xml");
    let work_dir = tempfile::TempDir::new().unwrap();
    let mut cmd = fixtures::play_launch_cmd(&env);
    let port = fixtures::web_port(&cmd);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--web-addr",
        &format!("127.0.0.1:{port}"),
        "--disable-monitoring",
        launch.to_str().unwrap(),
    ]);
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());
    let _proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    let base = format!("http://127.0.0.1:{port}");

    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        let out = Command::new("curl")
            .args([
                "-s",
                "-o",
                "/dev/null",
                "-w",
                "%{http_code}",
                "--max-time",
                "2",
                &format!("{base}/api/diagnostics/counts"),
            ])
            .output()
            .unwrap();
        if String::from_utf8_lossy(&out.stdout) == "200" {
            break;
        }
        assert!(Instant::now() < deadline, "web UI never answered");
        std::thread::sleep(Duration::from_millis(200));
    }

    // Nothing in this launch publishes /diagnostics, so the table never
    // changes: one table, then keep-alives. It used to be re-sent every
    // second.
    let out = Command::new("curl")
        .args([
            "-sN",
            "--max-time",
            "5",
            &format!("{base}/api/diagnostics/stream"),
        ])
        .output()
        .unwrap();
    let tables = parse_sse(&String::from_utf8_lossy(&out.stdout))
        .into_iter()
        .filter(|(_, d)| d != "keep-alive")
        .count();
    assert_eq!(tables, 1, "an unchanged table must be sent once");
}
