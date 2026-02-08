use play_launch_tests::fixtures;
use play_launch_tests::fixtures::array_len;
use play_launch_tests::process::ManagedProcess;

fn dump_launch(launch_file: &str, parser: &str) -> serde_json::Value {
    let env = fixtures::install_env();
    let (record, _tmp) = fixtures::dump_launch(&env, launch_file, parser);
    record
}

// ---- Dump tests ----

#[test]
fn test_dump_pure_nodes_rust() {
    let launch = fixtures::repo_root()
        .join("test/simple_test/launch/pure_nodes.launch.xml");
    let record = dump_launch(launch.to_str().unwrap(), "rust");

    let nodes = array_len(&record, "node");
    assert!(
        nodes > 0,
        "pure_nodes should produce at least 1 node, got {nodes}"
    );
    // pure_nodes should not have containers or load_nodes
    assert_eq!(array_len(&record, "container"), 0);
    assert_eq!(array_len(&record, "load_node"), 0);
}

#[test]
fn test_dump_composition_rust() {
    let launch = fixtures::repo_root()
        .join("test/simple_test/launch/composition.launch.xml");
    let record = dump_launch(launch.to_str().unwrap(), "rust");

    let containers = array_len(&record, "container");
    let load_nodes = array_len(&record, "load_node");
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
fn test_dump_pure_nodes_python() {
    let launch = fixtures::repo_root()
        .join("test/simple_test/launch/pure_nodes.launch.xml");
    let record = dump_launch(launch.to_str().unwrap(), "python");

    let nodes = array_len(&record, "node");
    assert!(
        nodes > 0,
        "pure_nodes (python) should produce at least 1 node, got {nodes}"
    );
}

#[test]
fn test_dump_composition_python() {
    let launch = fixtures::repo_root()
        .join("test/simple_test/launch/composition.launch.xml");
    let record = dump_launch(launch.to_str().unwrap(), "python");

    let containers = array_len(&record, "container");
    let load_nodes = array_len(&record, "load_node");
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
    let launch = fixtures::repo_root()
        .join("test/simple_test/launch/pure_nodes.launch.xml");
    let launch_str = launch.to_str().unwrap();

    let rust = dump_launch(launch_str, "rust");
    let python = dump_launch(launch_str, "python");

    assert_eq!(
        array_len(&rust, "node"),
        array_len(&python, "node"),
        "node count mismatch: rust={}, python={}",
        array_len(&rust, "node"),
        array_len(&python, "node")
    );
    assert_eq!(
        array_len(&rust, "container"),
        array_len(&python, "container"),
    );
    assert_eq!(
        array_len(&rust, "load_node"),
        array_len(&python, "load_node"),
    );
}

#[test]
fn test_parser_parity_composition() {
    let launch = fixtures::repo_root()
        .join("test/simple_test/launch/composition.launch.xml");
    let launch_str = launch.to_str().unwrap();

    let rust = dump_launch(launch_str, "rust");
    let python = dump_launch(launch_str, "python");

    assert_eq!(
        array_len(&rust, "node"),
        array_len(&python, "node"),
        "node count mismatch: rust={}, python={}",
        array_len(&rust, "node"),
        array_len(&python, "node")
    );
    assert_eq!(
        array_len(&rust, "container"),
        array_len(&python, "container"),
        "container count mismatch: rust={}, python={}",
        array_len(&rust, "container"),
        array_len(&python, "container")
    );
    assert_eq!(
        array_len(&rust, "load_node"),
        array_len(&python, "load_node"),
        "load_node count mismatch: rust={}, python={}",
        array_len(&rust, "load_node"),
        array_len(&python, "load_node")
    );
}

// ---- Launch test ----

#[test]
fn test_launch_pure_nodes() {
    let env = fixtures::install_env();
    let launch = fixtures::repo_root()
        .join("test/simple_test/launch/pure_nodes.launch.xml");

    // First dump to get expected process count
    let tmp = tempfile::TempDir::new().unwrap();
    let record_path = tmp.path().join("record.json");

    let mut dump_proc = ManagedProcess::spawn(
        fixtures::play_launch_cmd(&env).args([
            "dump",
            "--output",
            record_path.to_str().unwrap(),
            "launch",
            "--parser",
            "rust",
            launch.to_str().unwrap(),
        ]),
    )
    .expect("failed to spawn dump");

    let status = dump_proc.wait_with_timeout(std::time::Duration::from_secs(60));
    assert!(status.success());

    let expected = fixtures::count_expected_processes(&record_path);
    assert!(expected > 0, "expected at least 1 process");

    // Launch and wait for processes
    let work_dir = fixtures::repo_root().join("test/simple_test");
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
