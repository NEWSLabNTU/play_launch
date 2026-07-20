use play_launch_tests::fixtures;
use play_launch_tests::process::ManagedProcess;

fn launch_file() -> String {
    fixtures::test_workspace_path("sequential_loading")
        .join("launch/sequential.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

// ---- Resolve tests (Phase 47.B6: model, not record.json) ----

#[test]
fn test_resolve_sequential_rust() {
    let env = fixtures::install_env();
    let (model, _tmp) = fixtures::resolve_model(&env, &launch_file(), None, "rust");
    let (plain, containers, composables) = fixtures::model_entity_counts(&model);

    assert_eq!(plain, 0, "expected 0 standalone nodes");
    assert_eq!(containers, 1, "expected 1 container");
    assert_eq!(composables, 5, "expected 5 composable nodes");
}

#[test]
fn test_resolve_sequential_python() {
    let env = fixtures::install_env();
    let (model, _tmp) = fixtures::resolve_model(&env, &launch_file(), None, "python");
    let (plain, containers, composables) = fixtures::model_entity_counts(&model);

    assert_eq!(plain, 0, "expected 0 standalone nodes");
    assert_eq!(containers, 1, "expected 1 container");
    assert_eq!(composables, 5, "expected 5 composable nodes");
}

// ---- Parser parity ----

#[test]
fn test_parser_parity_sequential() {
    let env = fixtures::install_env();
    let launch = launch_file();
    let (rust, _r) = fixtures::resolve_model(&env, &launch, None, "rust");
    let (python, _p) = fixtures::resolve_model(&env, &launch, None, "python");

    assert_eq!(
        fixtures::model_entity_counts(&rust),
        fixtures::model_entity_counts(&python),
        "entity counts mismatch (plain, containers, composables)"
    );
}

// ---- Launch test ----

#[test]
fn test_launch_sequential() {
    let env = fixtures::install_env();
    let launch = launch_file();
    let work_dir = fixtures::test_workspace_path("sequential_loading");

    // Resolve to get expected process count (1 container)
    let (model, _tmp) = fixtures::resolve_model(&env, &launch, None, "rust");
    let expected = fixtures::count_expected_processes_from_model(&model);
    assert_eq!(expected, 1, "expected 1 process (the container)");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        &launch,
    ]);

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let play_log = work_dir.join("play_log/latest");
    fixtures::wait_for_processes(&play_log, expected, std::time::Duration::from_secs(30));

    let actual = fixtures::count_cmdline_files(&play_log);
    assert_eq!(
        actual, expected,
        "process count mismatch: actual={actual}, expected={expected}"
    );
}
