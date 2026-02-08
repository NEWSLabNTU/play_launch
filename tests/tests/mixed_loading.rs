use play_launch_tests::fixtures;
use play_launch_tests::fixtures::array_len;
use play_launch_tests::process::ManagedProcess;

fn launch_file() -> String {
    fixtures::test_workspace_path("mixed_loading")
        .join("launch/mixed.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

// ---- Dump tests ----

#[test]
fn test_dump_mixed_rust() {
    let env = fixtures::install_env();
    let (record, _tmp) = fixtures::dump_launch(&env, &launch_file(), "rust");

    assert_eq!(array_len(&record, "node"), 0, "expected 0 standalone nodes");
    assert_eq!(array_len(&record, "container"), 2, "expected 2 containers");
    assert_eq!(
        array_len(&record, "load_node"),
        5,
        "expected 5 composable nodes"
    );
}

#[test]
fn test_dump_mixed_python() {
    let env = fixtures::install_env();
    let (record, _tmp) = fixtures::dump_launch(&env, &launch_file(), "python");

    assert_eq!(array_len(&record, "node"), 0, "expected 0 standalone nodes");
    assert_eq!(array_len(&record, "container"), 2, "expected 2 containers");
    assert_eq!(
        array_len(&record, "load_node"),
        5,
        "expected 5 composable nodes"
    );
}

// ---- Parser parity ----

#[test]
fn test_parser_parity_mixed() {
    let env = fixtures::install_env();
    let launch = launch_file();
    let (rust, _r) = fixtures::dump_launch(&env, &launch, "rust");
    let (python, _p) = fixtures::dump_launch(&env, &launch, "python");

    for key in ["node", "container", "load_node"] {
        assert_eq!(
            array_len(&rust, key),
            array_len(&python, key),
            "{key} count mismatch: rust={}, python={}",
            array_len(&rust, key),
            array_len(&python, key)
        );
    }
}

// ---- Launch test ----

#[test]
fn test_launch_mixed() {
    let env = fixtures::install_env();
    let launch = launch_file();
    let work_dir = fixtures::test_workspace_path("mixed_loading");

    // Dump to get expected process count (2 containers)
    let (record, _tmp) = fixtures::dump_launch(&env, &launch, "rust");
    let expected = array_len(&record, "node") + array_len(&record, "container");
    assert_eq!(expected, 2, "expected 2 processes (2 containers)");

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
