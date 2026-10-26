//! Helping with DDS without touching it (phase 89, `docs/design/dds-assist.md`).
//!
//! Every finding here is reported, never acted on: play_launch changes no DDS
//! setting of the user's nodes.

use play_launch_tests::{fixtures, process::ManagedProcess};
use std::{process::Stdio, time::Duration};

/// One HTTP/1.1 GET to the web UI; returns (status, body).
fn http_get(port: u16, path: &str) -> (u16, String) {
    use std::io::{Read, Write};
    let Ok(mut stream) = std::net::TcpStream::connect(("127.0.0.1", port)) else {
        return (0, String::new());
    };
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    write!(
        stream,
        "GET {path} HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n"
    )
    .unwrap();
    let mut response = String::new();
    let _ = stream.read_to_string(&mut response);
    let status = response
        .split_whitespace()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let body = response
        .split_once("\r\n\r\n")
        .map(|(_, b)| b.to_string())
        .unwrap_or_default();
    (status, body)
}

fn domain_of(cmd: &std::process::Command) -> String {
    cmd.get_envs()
        .find(|(k, _)| *k == "ROS_DOMAIN_ID")
        .and_then(|(_, v)| v)
        .map(|v| v.to_string_lossy().into_owned())
        .expect("play_launch_cmd sets ROS_DOMAIN_ID")
}

/// D1: a node the launch file puts on another domain is reported before
/// anything spawns, by `check` and by `launch` (log and Health page).
#[test]
fn a_node_on_another_domain_is_reported_before_spawn() {
    let env = fixtures::install_env();
    let launch =
        fixtures::test_workspace_path("simple_test").join("launch/dds_env_split.launch.xml");
    // A second domain claimed for the listener, so it lands on no one else's.
    let other = fixtures::next_domain_id().to_string();

    // `check` spawns nothing at all.
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args([
        "check",
        launch.to_str().unwrap(),
        &format!("other_domain:={other}"),
    ]);
    let out = cmd.output().expect("run play_launch check");
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(
        out.status.success(),
        "a split is a warning, not a failure: {text}"
    );
    let line = text
        .lines()
        .find(|l| l.contains("warning[dds-env-split]"))
        .unwrap_or_else(|| panic!("no dds-env-split warning: {text}"));
    assert!(
        line.contains(&format!("ROS_DOMAIN_ID={other}")) && line.contains("/split/listener"),
        "{line}"
    );
    assert!(!text.contains("/split/talker sets"), "{text}");

    // `launch`: warned before the spawn, and an issue on the Health page.
    let work_dir = tempfile::TempDir::new().unwrap();
    let mut cmd = fixtures::play_launch_cmd(&env);
    let port = fixtures::web_port(&cmd);
    let ours = domain_of(&cmd);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--web-addr",
        &format!("127.0.0.1:{port}"),
        "--disable-monitoring",
        "--disable-diagnostics",
        launch.to_str().unwrap(),
        &format!("other_domain:={other}"),
    ]);
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());
    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    let deadline = std::time::Instant::now() + Duration::from_secs(30);
    let issue = loop {
        std::thread::sleep(Duration::from_millis(500));
        let (status, body) = http_get(port, "/api/health/issues");
        if status == 200 {
            let v: serde_json::Value = serde_json::from_str(&body).unwrap();
            if let Some(i) = v["open"]
                .as_array()
                .and_then(|a| a.iter().find(|i| i["kind"] == "dds_env_split"))
            {
                break i.clone();
            }
        }
        assert!(
            std::time::Instant::now() < deadline,
            "no dds_env_split issue"
        );
    };
    // D4: the report holds it, on the page and, at the end, in the bundle.
    // The host is read on the first tick and lands on the next.
    let report = loop {
        let (status, report) = http_get(port, "/api/health/dds_report");
        assert_eq!(status, 200);
        if report.contains("## Kernel") || std::time::Instant::now() > deadline {
            break report;
        }
        std::thread::sleep(Duration::from_millis(500));
    };
    for want in [
        "## Middleware",
        "## Kernel",
        "## Participants and discovery",
        "## Per-node DDS environment",
        "/split/listener sets ROS_DOMAIN_ID=",
        "[dds_env_split]",
    ] {
        assert!(report.contains(want), "missing {want:?} in\n{report}");
    }
    drop(proc);
    let bundled = work_dir.path().join("play_log/latest/dds_report.txt");
    assert!(
        std::fs::read_to_string(&bundled).is_ok_and(|t| t.contains("/split/listener")),
        "dds_report.txt not in the bundle"
    );
    assert_eq!(issue["severity"], "warning", "{issue}");
    let detail = issue["detail"].as_str().unwrap();
    assert!(
        detail.contains(&format!("ROS_DOMAIN_ID={other}"))
            && detail.contains(&format!("{ours} (inherited)")),
        "{detail}"
    );

    let log = std::fs::read_to_string(work_dir.path().join("play_log/latest/play_launch.log"))
        .expect("play_launch.log");
    let warned = log
        .find("dds-env-split")
        .unwrap_or_else(|| panic!("not in play_launch.log"));
    let spawned = log.find("Spawning").expect("a spawn line");
    assert!(warned < spawned, "warned after the spawn");
}

fn interception_so_path() -> std::path::PathBuf {
    let release = fixtures::repo_root()
        .join("src/play_launch_interception/target/release/libplay_launch_interception.so");
    assert!(release.is_file(), "build it: just build-interception");
    release
}

/// The rate in a `large_udp_message` title, `… at 5.0 Hz to …`.
fn rate_of(title: &str) -> Option<f64> {
    title
        .split(" at ")
        .nth(1)?
        .split(" Hz")
        .next()?
        .parse()
        .ok()
}

fn read_sysctl(path: &str) -> String {
    std::fs::read_to_string(path).unwrap().trim().to_string()
}

/// D2: a 200 KiB message taken in another process is reported with its
/// rate and the kernel's IP reassembly settings, read from this host.
#[test]
fn a_message_larger_than_a_datagram_is_reported_with_the_hosts_settings() {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("simple_test").join("launch/big_message.launch.xml");
    let work_dir = tempfile::TempDir::new().unwrap();
    let mut cmd = fixtures::play_launch_cmd(&env);
    let port = fixtures::web_port(&cmd);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--interception",
        "on",
        "--web-addr",
        &format!("127.0.0.1:{port}"),
        "--disable-monitoring",
        "--disable-diagnostics",
        launch.to_str().unwrap(),
    ]);
    cmd.env("PLAY_LAUNCH_INTERCEPTION_SO", interception_so_path());
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());
    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    let deadline = std::time::Instant::now() + Duration::from_secs(40);
    let issue = loop {
        std::thread::sleep(Duration::from_millis(1000));
        let (status, body) = http_get(port, "/api/health/issues");
        if status == 200 {
            let v: serde_json::Value = serde_json::from_str(&body).unwrap();
            // The rate is over 10 s: wait for a full window.
            if let Some(i) = v["open"]
                .as_array()
                .and_then(|a| a.iter().find(|i| i["kind"] == "large_udp_message"))
                && rate_of(i["title"].as_str().unwrap()).is_some_and(|r| r >= 4.0)
            {
                break i.clone();
            }
        }
        assert!(
            std::time::Instant::now() < deadline,
            "no large_udp_message issue"
        );
    };
    drop(proc);
    assert_eq!(issue["subject"]["name"], "/big/blob", "{issue}");
    let title = issue["title"].as_str().unwrap();
    assert!(title.starts_with("200 KiB messages at"), "{title}");
    assert!(title.contains("to 1 other process"), "{title}");
    assert!(rate_of(title).is_some_and(|r| r <= 6.0), "{title}");
    let detail = issue["detail"].as_str().unwrap();
    let t = read_sysctl("/proc/sys/net/ipv4/ipfrag_time");
    assert!(
        detail.contains(&format!("ipfrag_time ({t} s here)")),
        "{detail}"
    );
    assert!(detail.contains("ipfrag_high_thresh ("), "{detail}");
}
