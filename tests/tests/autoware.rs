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
    // This one actually LAUNCHES Autoware, so it needs the sample map on disk:
    // `map_projection_loader` throws `No map projector info files found` and
    // exits, and the health report counts that as a node exit. Without the
    // guard the test reports FAIL for a missing prerequisite, which reads as a
    // code defect and is not one — the same class as the `shape_estimation`
    // (TensorRT) and `rviz2` (X display) exits already ignored below, except
    // those nodes at least start.
    //
    // Skipping rather than adding it to `ignored_exits`: ignoring it there
    // would also swallow a genuine map-loading regression on a machine that
    // HAS the map. `test-all`'s skip reporter surfaces this line, so an
    // always-skipping guard stays visible instead of passing quietly.
    let map_path = fixtures::autoware_map_path();
    if !std::path::Path::new(&map_path).exists() {
        eprintln!(
            "SKIP: test_autoware_smoke_test: map not found at {map_path} \
             (set MAP_PATH, or fetch the Autoware sample map)"
        );
        return;
    }

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

/// Phase 71 W4: Autoware's MRM chain as a contract, with its own parameter-
/// file numbers. Detection 500ms (the availability timeout) + reaction
/// 110 + 34 + 100 ms + settle 1200 = 1944ms: fits a 2 s interval with 56ms
/// of slack, and fails a 1.5 s one — arithmetic the parameter files imply
/// and nothing performed until now.
#[test]
fn test_autoware_mrm_chain_fault_reaction_budget() {
    require_autoware();
    let env = fixtures::autoware_env();
    let contracts = fixtures::repo_root().join("tests/fixtures/autoware/contracts");
    let map_path_arg = format!("map_path:={}", fixtures::autoware_map_path());
    let out = std::process::Command::new(fixtures::play_launch_bin())
        .envs(&env)
        .arg("check")
        .arg("--contracts")
        .arg(&contracts)
        .arg("autoware_launch")
        .arg("planning_simulator.launch.xml")
        .arg(&map_path_arg)
        .output()
        .expect("play_launch check runs");
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    let plain = strip_ansi(&text);
    assert!(
        plain.contains("hazard 'mode_unavailable': detection 500.00ms"),
        "{plain}"
    );
    assert!(plain.contains("= 1944.00ms fits the fault-tolerant time interval 2000.00ms"), "{plain}");
    assert!(
        plain.contains("hazard 'mode_unavailable_tight'") && plain.contains("1944.00ms exceeds"),
        "{plain}"
    );
    // Phase 75 — `mode_unavailable`'s reaction is the `autonomous` MODE, so
    // the ladder is the reaction: `comfortable_stop` is checked in its own
    // right (it decelerates over ~4 s and cannot make the 2 s interval),
    // while `emergency_stop`, the floor, is what the budget above measures.
    assert!(
        plain.contains("error[ladder-rung-budget]") && plain.contains("rung 'comfortable_stop'"),
        "a graded rung must be checked in its own right:\n{plain}"
    );
}

fn strip_ansi(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '\x1b' {
            for c2 in chars.by_ref() {
                if c2.is_ascii_alphabetic() {
                    break;
                }
            }
        } else {
            out.push(c);
        }
    }
    out
}

/// Phase 76: the launch file's own remaps become a topic graph. Before this
/// the Autoware model resolved to 119 nodes and ZERO topics, so every graph
/// rule — criticality propagation, scope-budget, the fault-reaction walk —
/// was computing over nothing and reporting clean.
#[test]
fn test_autoware_topic_graph_derived_from_remaps() {
    let (model, _tmp) = resolve_autoware("rust");
    let topics = model["structure"]["topics"]
        .as_object()
        .expect("structure.topics");
    assert!(
        topics.len() > 80,
        "expected the remap-derived graph, got {} topics",
        topics.len()
    );
    let both = topics
        .values()
        .filter(|t| {
            !t["pub"].as_array().is_none_or(|a| a.is_empty())
                && !t["sub"].as_array().is_none_or(|a| a.is_empty())
        })
        .count();
    assert!(
        both > 30,
        "a graph needs edges, not just endpoints: {both} topics wired on both sides"
    );
    // The wiring is real: the command gate reads the trajectory follower.
    let gate_in = topics["/control/trajectory_follower/control_cmd"]["sub"]
        .as_array()
        .expect("sub side");
    assert!(
        gate_in.iter().any(|s| s.as_str().is_some_and(|s| s.contains("vehicle_cmd_gate"))),
        "{gate_in:?}"
    );
}
