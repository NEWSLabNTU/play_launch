use play_launch_tests::fixtures;
use play_launch_tests::process::ManagedProcess;

/// `(plain_nodes, containers, composables)` from resolving `launch_file`
/// with the given parser. Phase 47.B6 — the model-shaped sibling of the
/// retired record.json-based `dump_launch` helper.
fn resolve_counts(launch_file: &str, parser: &str) -> (usize, usize, usize) {
    let env = fixtures::install_env();
    let (model, _tmp) = fixtures::resolve_model(&env, launch_file, None, parser);
    fixtures::model_entity_counts(&model)
}

// ---- Resolve tests ----

#[test]
fn test_resolve_pure_nodes_rust() {
    let launch = fixtures::test_workspace_path("simple_test").join("launch/pure_nodes.launch.xml");
    let (nodes, containers, load_nodes) = resolve_counts(launch.to_str().unwrap(), "rust");

    assert!(
        nodes > 0,
        "pure_nodes should produce at least 1 node, got {nodes}"
    );
    // pure_nodes should not have containers or load_nodes
    assert_eq!(containers, 0);
    assert_eq!(load_nodes, 0);
}

#[test]
fn test_resolve_composition_rust() {
    let launch = fixtures::test_workspace_path("simple_test").join("launch/composition.launch.xml");
    let (_nodes, containers, load_nodes) = resolve_counts(launch.to_str().unwrap(), "rust");

    assert!(
        containers > 0,
        "composition should have at least 1 container, got {containers}"
    );
    assert!(
        load_nodes > 0,
        "composition should have at least 1 load_node, got {load_nodes}"
    );
}

#[test]
fn test_resolve_pure_nodes_python() {
    let launch = fixtures::test_workspace_path("simple_test").join("launch/pure_nodes.launch.xml");
    let (nodes, _containers, _load_nodes) = resolve_counts(launch.to_str().unwrap(), "python");

    assert!(
        nodes > 0,
        "pure_nodes (python) should produce at least 1 node, got {nodes}"
    );
}

#[test]
fn test_resolve_composition_python() {
    let launch = fixtures::test_workspace_path("simple_test").join("launch/composition.launch.xml");
    let (_nodes, containers, load_nodes) = resolve_counts(launch.to_str().unwrap(), "python");

    assert!(
        containers > 0,
        "composition (python) should have at least 1 container, got {containers}"
    );
    assert!(
        load_nodes > 0,
        "composition (python) should have at least 1 load_node, got {load_nodes}"
    );
}

// ---- Parser parity tests ----

#[test]
fn test_parser_parity_pure_nodes() {
    let launch = fixtures::test_workspace_path("simple_test").join("launch/pure_nodes.launch.xml");
    let launch_str = launch.to_str().unwrap();

    let rust = resolve_counts(launch_str, "rust");
    let python = resolve_counts(launch_str, "python");

    assert_eq!(
        rust, python,
        "entity counts mismatch (plain, containers, composables): rust={rust:?}, python={python:?}"
    );
}

#[test]
fn test_parser_parity_composition() {
    let launch = fixtures::test_workspace_path("simple_test").join("launch/composition.launch.xml");
    let launch_str = launch.to_str().unwrap();

    let rust = resolve_counts(launch_str, "rust");
    let python = resolve_counts(launch_str, "python");

    assert_eq!(
        rust, python,
        "entity counts mismatch (plain, containers, composables): rust={rust:?}, python={python:?}"
    );
}

// ---- Launch test ----

#[test]
fn test_launch_pure_nodes() {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("simple_test").join("launch/pure_nodes.launch.xml");

    // First resolve to get expected process count (Phase 47.B6: model, not
    // record.json — `dump`'s only artifact now).
    let (model, _tmp) = fixtures::resolve_model(&env, launch.to_str().unwrap(), None, "rust");
    let expected = fixtures::count_expected_processes_from_model(&model);
    assert!(expected > 0, "expected at least 1 process");

    // Launch and wait for processes
    let work_dir = fixtures::test_workspace_path("simple_test");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        launch.to_str().unwrap(),
    ]);

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let play_log = work_dir.join("play_log/latest");
    fixtures::wait_for_processes(&play_log, expected, std::time::Duration::from_secs(15));

    let actual = fixtures::count_cmdline_files(&play_log);
    assert_eq!(
        actual, expected,
        "process count mismatch: actual={actual}, expected={expected}"
    );

    // _proc dropped here — ManagedProcess::drop kills the process group
}
