//! Only the web UI's own page may change anything (phase 105 W1, #0111).
//!
//! A body-less POST is a CORS "simple request": any page the user has open
//! can send one, and CORS only hides the answer, after the node is stopped.
//! These run against a live `launch`, so the refusal is shown to leave the
//! node running, and the page's own request is shown to still work (without
//! it, a guard that refused everything would pass).

use play_launch_tests::{fixtures, process::ManagedProcess};
use std::{
    process::Stdio,
    time::{Duration, Instant},
};

/// One HTTP/1.1 request with the given extra headers; returns (status, body).
fn request(port: u16, method: &str, path: &str, host: &str, extra: &[&str]) -> (u16, String) {
    use std::io::{Read, Write};
    let Ok(mut stream) = std::net::TcpStream::connect(("127.0.0.1", port)) else {
        return (0, String::new());
    };
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let mut head = format!("{method} {path} HTTP/1.1\r\nHost: {host}\r\nConnection: close\r\n");
    for h in extra {
        head.push_str(h);
        head.push_str("\r\n");
    }
    head.push_str("Content-Length: 0\r\n\r\n");
    stream.write_all(head.as_bytes()).unwrap();
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

/// The talker's pid as `/api/nodes` reports it, `None` when not running.
fn talker_pid(port: u16) -> Option<u64> {
    let own = format!("127.0.0.1:{port}");
    let (status, body) = request(port, "GET", "/api/nodes", &own, &[]);
    if status != 200 {
        return None;
    }
    // A chunked body: take the JSON array between its first `[` and last `]`.
    let start = body.find('[')?;
    let end = body.rfind(']')?;
    let nodes: serde_json::Value = serde_json::from_str(&body[start..=end]).ok()?;
    nodes
        .as_array()?
        .iter()
        .find(|n| n["name"].as_str().is_some_and(|s| s.ends_with("talker")))?
        .get("pid")?
        .as_u64()
}

#[test]
fn a_foreign_page_cannot_stop_a_node_and_the_ui_can() {
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
        "--disable-diagnostics",
        launch.to_str().unwrap(),
    ]);
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());
    let _proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");

    let deadline = Instant::now() + Duration::from_secs(30);
    let pid = loop {
        if let Some(pid) = talker_pid(port) {
            break pid;
        }
        assert!(Instant::now() < deadline, "talker never reported running");
        std::thread::sleep(Duration::from_millis(200));
    };

    let own = format!("127.0.0.1:{port}");
    let stop = "/api/nodes/node:%2Fpure_test%2Ftalker/stop";

    // A cross-site page: its Origin is its own.
    let (status, body) = request(port, "POST", stop, &own, &["Origin: https://evil.example"]);
    assert_eq!(status, 403, "foreign Origin accepted: {body}");
    let (status, _) = request(port, "POST", stop, &own, &["Sec-Fetch-Site: cross-site"]);
    assert_eq!(status, 403, "cross-site fetch accepted");
    // DNS rebinding: the browser sends the hostile name as Host.
    let rebound = format!("evil.example:{port}");
    let (status, _) = request(port, "GET", "/api/nodes", &rebound, &[]);
    assert_eq!(status, 403, "rebound Host could read");
    let (status, _) = request(port, "POST", stop, &rebound, &[]);
    assert_eq!(status, 403, "rebound Host could stop");

    std::thread::sleep(Duration::from_millis(500));
    assert_eq!(
        talker_pid(port),
        Some(pid),
        "a refused request must leave the node running"
    );

    // The page itself.
    let origin = format!("Origin: http://{own}");
    let (status, body) = request(port, "POST", stop, &own, &[&origin]);
    assert!(
        (200..300).contains(&status),
        "own page refused ({status}): {body}"
    );
    let deadline = Instant::now() + Duration::from_secs(10);
    while talker_pid(port).is_some() {
        assert!(
            Instant::now() < deadline,
            "own stop did not stop the talker"
        );
        std::thread::sleep(Duration::from_millis(200));
    }
}

/// Whether a raw response sets the token cookie.
fn sets_token_cookie(port: u16, path: &str, host: &str) -> (u16, bool) {
    use std::io::{Read, Write};
    let mut stream = std::net::TcpStream::connect(("127.0.0.1", port)).unwrap();
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    write!(
        stream,
        "GET {path} HTTP/1.1\r\nHost: {host}\r\nConnection: close\r\n\r\n"
    )
    .unwrap();
    let mut response = String::new();
    let _ = stream.read_to_string(&mut response);
    let status = response
        .split_whitespace()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let head = response
        .split("\r\n\r\n")
        .next()
        .unwrap_or("")
        .to_lowercase();
    (
        status,
        head.contains("set-cookie: play_launch_token=t0ken; path=/; httponly; samesite=strict"),
    )
}

#[test]
fn a_token_gates_every_request_and_the_url_form_sets_the_cookie() {
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
        "--web-token",
        "t0ken",
        "--disable-monitoring",
        "--disable-diagnostics",
        launch.to_str().unwrap(),
    ]);
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());
    let _proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    let own = format!("127.0.0.1:{port}");

    // Wait for the server, using the header form.
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        let (status, _) = request(
            port,
            "GET",
            "/api/health",
            &own,
            &["X-Play-Launch-Token: t0ken"],
        );
        if status == 200 {
            break;
        }
        assert!(Instant::now() < deadline, "web UI never answered");
        std::thread::sleep(Duration::from_millis(200));
    }

    let (status, _) = request(port, "GET", "/api/health", &own, &[]);
    assert_eq!(status, 403, "no token accepted");
    let (status, _) = request(port, "GET", "/", &own, &["X-Play-Launch-Token: wrong"]);
    assert_eq!(status, 403, "wrong token accepted");
    let (status, _) = request(port, "GET", "/", &own, &["Cookie: play_launch_token=t0ken"]);
    assert_eq!(status, 200, "cookie refused");
    let (status, cookie) = sets_token_cookie(port, "/?token=t0ken", &own);
    assert_eq!(status, 200, "URL token refused");
    assert!(cookie, "URL token did not set the cookie");
    let (status, cookie) = sets_token_cookie(port, "/?token=nope", &own);
    assert_eq!(status, 403);
    assert!(!cookie);
}
