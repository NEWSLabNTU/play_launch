use std::process::Stdio;

use play_launch_tests::fixtures;
use play_launch_tests::health::HealthReport;
use play_launch_tests::process::ManagedProcess;

fn require_autoware() {
    let script = fixtures::test_workspace_path("autoware").join("activate_autoware.sh");
    assert!(
        script.is_file(),
        "activate_autoware.sh not found: {}. \
         Edit it to source your Autoware install's setup.bash",
        script.display()
    );
}

fn autoware_launch_args() -> Vec<String> {
    vec![
        "autoware_launch".to_string(),
        "planning_simulator.launch.xml".to_string(),
        format!("map_path:={}", fixtures::autoware_map_path()),
    ]
}

/// Resolve Autoware with the given parser, returning (parsed SystemModel
/// YAML as JSON, temp dir backing the model file). Phase 47.B6 — the
/// model-shaped sibling of the retired record.json-based `dump_autoware`.
fn resolve_autoware(parser: &str) -> (serde_json::Value, tempfile::TempDir) {
    require_autoware();
    let env = fixtures::autoware_env();
    let map_path_arg = format!("map_path:={}", fixtures::autoware_map_path());
    fixtures::resolve_model_with_args(
        &env,
        "autoware_launch",
        Some("planning_simulator.launch.xml"),
        &[&map_path_arg],
        parser,
    )
}

/// Assert that a resolved model matches the expected counts from
/// `activate_autoware.sh`. When an `EXPECTED_*` variable is set, the count
/// must match exactly; otherwise we just check > 0.
fn assert_entity_counts(model: &serde_json::Value, parser: &str) {
    let env = fixtures::autoware_env();
    let (exp_nodes, exp_containers, exp_load_nodes) = fixtures::autoware_expected_counts(&env);
    let (nodes, containers, load_nodes) = fixtures::model_entity_counts(model);

    let checks: &[(&str, usize, Option<usize>)] = &[
        ("node", nodes, exp_nodes),
        ("container", containers, exp_containers),
        ("load_node", load_nodes, exp_load_nodes),
    ];

    for &(key, actual, expected) in checks {
        if let Some(n) = expected {
            assert_eq!(
                actual,
                n,
                "{parser} parser: {key} count {actual} != expected {n} \
                 (from EXPECTED_{} in activate_autoware.sh)",
                key.to_uppercase()
            );
        } else {
            assert!(
                actual > 0,
                "{parser} parser: expected at least 1 {key}, got 0"
            );
        }
    }
}

// ---- Resolve tests ----

#[test]
fn test_autoware_resolve_rust() {
    let (model, _tmp) = resolve_autoware("rust");
    assert_entity_counts(&model, "rust");
}

#[test]
fn test_autoware_resolve_python() {
    let (model, _tmp) = resolve_autoware("python");
    assert_entity_counts(&model, "python");
}

#[test]
fn test_autoware_resolve_counts_match() {
    let (rust_model, _r_tmp) = resolve_autoware("rust");
    let (python_model, _p_tmp) = resolve_autoware("python");

    assert_eq!(
        fixtures::model_entity_counts(&rust_model),
        fixtures::model_entity_counts(&python_model),
        "entity counts mismatch (plain, containers, composables)"
    );
}

// ---- Parser parity ----

#[test]
fn test_autoware_parser_parity() {
    let (_, rust_tmp) = resolve_autoware("rust");
    let (_, python_tmp) = resolve_autoware("python");

    let rust_model = rust_tmp.path().join("system_model.yaml");
    let python_model = python_tmp.path().join("system_model.yaml");

    let (success, output) = fixtures::compare_models(&rust_model, &python_model);
    assert!(
        success,
        "Rust vs Python parser comparison failed:\n{output}"
    );
}

// ---- Process count tests ----

#[test]
fn test_autoware_process_count_rust() {
    require_autoware();
    let env = fixtures::autoware_env();
    let work_dir = fixtures::test_workspace_path("autoware");

    // Resolve to get expected count (Phase 47.B6: model, not record.json).
    let (model, _tmp) = resolve_autoware("rust");
    let expected = fixtures::count_expected_processes_from_model(&model);
    assert!(expected > 0, "expected at least 1 process from dump");

    // Launch
    let mut launch_args = vec![
        "launch".to_string(),
        "--disable-web-ui".to_string(),
        "--parser".to_string(),
        "rust".to_string(),
    ];
    launch_args.extend(autoware_launch_args());

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
    cmd.args(&launch_args);

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let play_log = work_dir.join("play_log/latest");
    fixtures::wait_for_processes(&play_log, expected, std::time::Duration::from_secs(60));

    let actual = fixtures::count_cmdline_files(&play_log);
    assert_eq!(actual, expected, "process count: {actual}/{expected}");

    // _proc dropped here — ManagedProcess::drop kills the process group
}

// ---- Smoke test ----

#[test]
fn test_autoware_smoke_test() {
    // 1. Resolve to get expected process count (Phase 47.B6: model, not
    // record.json).
    let (model, _resolve_tmp) = resolve_autoware("rust");
    let expected = fixtures::count_expected_processes_from_model(&model);
    assert!(expected > 0, "expected at least 1 process from dump");

    // 2. Set up stdout capture (play_launch writes tracing output to stdout)
    let output_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let output_path = output_tmp.path().join("play_launch_output.log");

    // 3. Launch play_launch
    let env = fixtures::autoware_env();
    let work_dir = fixtures::test_workspace_path("autoware");

    let mut launch_args = vec![
        "launch".to_string(),
        "--disable-web-ui".to_string(),
        "--parser".to_string(),
        "rust".to_string(),
    ];
    launch_args.extend(autoware_launch_args());

    let output_file = std::fs::File::create(&output_path).expect("failed to create output file");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
    cmd.args(&launch_args);
    cmd.stdout(Stdio::from(output_file));

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    // 4. Wait for processes to stabilize, then settle for LoadNode operations
    let play_log = work_dir.join("play_log/latest");
    fixtures::wait_for_processes(&play_log, expected, std::time::Duration::from_secs(60));
    std::thread::sleep(std::time::Duration::from_secs(15));

    // 5. Analyze health
    let report = HealthReport::analyze(&play_log, &output_path, expected);

    // 6. Print report (visible with --no-capture or on failure)
    eprintln!("\n{report}");

    // 7. Assert healthy (ignore environment-specific node exits and known upstream races)
    // shape_estimation: requires TensorRT (libnvinfer.so.8) which is GPU-specific
    // rviz2: requires X display server (not available in headless/CI environments)
    let ignored_exits = &["shape_estimation", "rviz2"];
    // rcl context shutdown race (ros2/rclcpp#812): SIGTERM signal handler
    // asynchronously invalidates the context while LoadNode is in progress
    let ignored_load_errors = &["context is not valid", "context is invalid"];
    assert!(
        report.is_healthy(ignored_exits, ignored_load_errors),
        "Smoke test failed:\n{report}"
    );

    // _proc dropped here — ManagedProcess::drop kills the process group
}

#[test]
fn test_autoware_process_count_python() {
    require_autoware();
    let env = fixtures::autoware_env();
    let work_dir = fixtures::test_workspace_path("autoware");

    // Resolve to get expected count (Phase 47.B6: model, not record.json).
    let (model, _tmp) = resolve_autoware("python");
    let expected = fixtures::count_expected_processes_from_model(&model);
    assert!(expected > 0, "expected at least 1 process from dump");

    // Launch
    let mut launch_args = vec![
        "launch".to_string(),
        "--disable-web-ui".to_string(),
        "--parser".to_string(),
        "python".to_string(),
    ];
    launch_args.extend(autoware_launch_args());

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
    cmd.args(&launch_args);

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let play_log = work_dir.join("play_log/latest");
    fixtures::wait_for_processes(&play_log, expected, std::time::Duration::from_secs(60));

    let actual = fixtures::count_cmdline_files(&play_log);
    assert_eq!(actual, expected, "process count: {actual}/{expected}");

    // _proc dropped here — ManagedProcess::drop kills the process group
}
