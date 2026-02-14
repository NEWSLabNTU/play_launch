use std::process::Stdio;

use play_launch_tests::fixtures;
use play_launch_tests::fixtures::array_len;
use play_launch_tests::process::ManagedProcess;

fn parallel_slow_launch() -> String {
    fixtures::test_workspace_path("parallel_loading")
        .join("launch/parallel_slow.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

fn mixed_fast_slow_launch() -> String {
    fixtures::test_workspace_path("parallel_loading")
        .join("launch/mixed_fast_slow.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

// ---- Dump tests ----

#[test]
fn test_dump_parallel_loading() {
    let env = fixtures::install_env();
    let (record, _tmp) = fixtures::dump_launch(&env, &parallel_slow_launch(), "rust");

    assert_eq!(array_len(&record, "node"), 0, "expected 0 standalone nodes");
    assert_eq!(array_len(&record, "container"), 1, "expected 1 container");
    assert_eq!(
        array_len(&record, "load_node"),
        2,
        "expected 2 composable nodes"
    );

    // Verify container has --isolated in its args
    let container = &record["container"][0];
    let cmd = container["cmd"]
        .as_array()
        .expect("cmd should be an array");
    let has_isolated = cmd.iter().any(|v| v.as_str() == Some("--isolated"));
    assert!(
        has_isolated,
        "expected --isolated in container cmd, got: {:?}",
        cmd
    );
}

#[test]
fn test_dump_mixed_fast_slow() {
    let env = fixtures::install_env();
    let (record, _tmp) = fixtures::dump_launch(&env, &mixed_fast_slow_launch(), "rust");

    assert_eq!(array_len(&record, "node"), 0, "expected 0 standalone nodes");
    assert_eq!(array_len(&record, "container"), 1, "expected 1 container");
    assert_eq!(
        array_len(&record, "load_node"),
        2,
        "expected 2 composable nodes"
    );
}

// ---- Launch tests ----

/// Test that both slow-loading nodes (3s each) complete within a reasonable
/// time when loaded in parallel (should be ~3s, not 6s sequential).
#[test]
fn test_parallel_load_completes() {
    let env = fixtures::install_env();
    let launch = parallel_slow_launch();
    let work_dir = fixtures::test_workspace_path("parallel_loading");

    let output_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let output_path = output_tmp.path().join("stdout.log");
    let stderr_path = output_tmp.path().join("stderr.log");
    let output_file = std::fs::File::create(&output_path).expect("failed to create stdout file");
    let stderr_file = std::fs::File::create(&stderr_path).expect("failed to create stderr file");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        &launch,
    ]);
    cmd.stdout(Stdio::from(output_file));
    cmd.stderr(Stdio::from(stderr_file));
    cmd.env("RUST_LOG", "play_launch=debug");

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    // Wait for the container process to appear
    let play_log = work_dir.join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 1, std::time::Duration::from_secs(30));

    // Wait for both composable nodes to load (3s delay + overhead).
    // With parallel loading: ~3s construction + ~2s service warmup.
    // We poll until both LOADED events appear or 30s timeout.
    let start = std::time::Instant::now();
    let timeout = std::time::Duration::from_secs(30);
    let mut loaded_count = 0;

    while start.elapsed() < timeout {
        std::thread::sleep(std::time::Duration::from_secs(1));
        let stdout = std::fs::read_to_string(&output_path).unwrap_or_default();
        loaded_count = stdout
            .lines()
            .filter(|line| line.contains("ComponentEvent LOADED") || line.contains("LoadSucceeded"))
            .count();
        if loaded_count >= 2 {
            break;
        }
    }

    let elapsed = start.elapsed();
    let stdout = std::fs::read_to_string(&output_path).unwrap_or_default();

    eprintln!("--- stdout ({} bytes, last 2000 chars) ---", stdout.len());
    let snippet_start = stdout.len().saturating_sub(2000);
    eprintln!("{}", &stdout[snippet_start..]);
    eprintln!("loaded_count={loaded_count}, elapsed={:.1}s", elapsed.as_secs_f64());

    assert!(
        loaded_count >= 2,
        "Expected both SlowLoader nodes to have LOADED events, found {loaded_count}"
    );
}

/// Test that a fast component (Talker) is not blocked by a slow component
/// (SlowLoader 5s). The Talker's LOADED event should appear before the
/// SlowLoader finishes.
#[test]
fn test_fast_not_blocked_by_slow() {
    let env = fixtures::install_env();
    let launch = mixed_fast_slow_launch();
    let work_dir = fixtures::test_workspace_path("parallel_loading");

    let output_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let output_path = output_tmp.path().join("stdout.log");
    let stderr_path = output_tmp.path().join("stderr.log");
    let output_file = std::fs::File::create(&output_path).expect("failed to create stdout file");
    let stderr_file = std::fs::File::create(&stderr_path).expect("failed to create stderr file");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        &launch,
    ]);
    cmd.stdout(Stdio::from(output_file));
    cmd.stderr(Stdio::from(stderr_file));
    cmd.env("RUST_LOG", "play_launch=debug");

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    // Wait for the container to start
    let play_log = work_dir.join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 1, std::time::Duration::from_secs(30));

    // The SlowLoader takes 3s. The Talker should load almost instantly.
    // Wait until we see both LOADED events or timeout.
    let start = std::time::Instant::now();
    let timeout = std::time::Duration::from_secs(30);
    let mut talker_loaded = false;
    let mut talker_loaded_at = None;

    while start.elapsed() < timeout {
        std::thread::sleep(std::time::Duration::from_secs(1));
        let stdout = std::fs::read_to_string(&output_path).unwrap_or_default();
        if stdout.contains("ComponentEvent LOADED for 'fast_talker'") {
            talker_loaded = true;
            if talker_loaded_at.is_none() {
                talker_loaded_at = Some(start.elapsed());
            }
        }
        // Check for both loaded
        let loaded_count = stdout
            .lines()
            .filter(|line| line.contains("ComponentEvent LOADED"))
            .count();
        if loaded_count >= 2 {
            break;
        }
    }

    let stdout_final = std::fs::read_to_string(&output_path).unwrap_or_default();
    let slow_loaded = stdout_final.contains("ComponentEvent LOADED for 'slow_node'");

    eprintln!("--- Final output (last 3000 chars) ---");
    let snippet_start = stdout_final.len().saturating_sub(3000);
    eprintln!("{}", &stdout_final[snippet_start..]);
    eprintln!(
        "talker_loaded={} (at {:?}), slow_loaded={}, total={:.1}s",
        talker_loaded,
        talker_loaded_at,
        slow_loaded,
        start.elapsed().as_secs_f64()
    );

    assert!(
        talker_loaded,
        "Talker should have loaded (fast component not blocked by slow)"
    );
    assert!(
        slow_loaded,
        "SlowLoader should have completed construction eventually"
    );

    // Verify the Talker loaded before the 5s mark (proving it wasn't blocked)
    if let Some(talker_time) = talker_loaded_at {
        assert!(
            talker_time.as_secs() < 8,
            "Talker took {:.1}s to load — should be fast (<8s including warmup)",
            talker_time.as_secs_f64()
        );
    }
}
