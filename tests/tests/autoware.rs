use std::process::Stdio;

use play_launch_tests::fixtures;
use play_launch_tests::fixtures::array_len;
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

/// Dump Autoware with the given parser, returning (parsed JSON, temp dir).
///
/// The temp dir is returned so callers can access the record.json file path
/// (at `tmp.path().join("record.json")`) for script-based comparison.
fn dump_autoware(parser: &str) -> (serde_json::Value, tempfile::TempDir) {
    require_autoware();
    let env = fixtures::autoware_env();
    let tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let output_path = tmp.path().join("record.json");

    let mut args = vec![
        "dump".to_string(),
        "--output".to_string(),
        output_path.to_str().unwrap().to_string(),
        "launch".to_string(),
        "--parser".to_string(),
        parser.to_string(),
    ];
    args.extend(autoware_launch_args());

    let mut proc = ManagedProcess::spawn(
        fixtures::play_launch_cmd(&env).args(&args),
    )
    .expect("failed to spawn play_launch dump");

    let status = proc.wait_with_timeout(std::time::Duration::from_secs(60));
    assert!(
        status.success(),
        "play_launch dump (parser={parser}) failed"
    );

    let data = std::fs::read_to_string(&output_path).expect("failed to read record.json");
    let record = serde_json::from_str(&data).expect("failed to parse record.json");
    (record, tmp)
}


// ---- Dump tests ----

#[test]
fn test_autoware_dump_rust() {
    let (record, _tmp) = dump_autoware("rust");
    assert_eq!(array_len(&record, "node"), 46, "expected 46 nodes");
    assert_eq!(
        array_len(&record, "container"),
        15,
        "expected 15 containers"
    );
    assert_eq!(
        array_len(&record, "load_node"),
        54,
        "expected 54 load_nodes"
    );
}

#[test]
fn test_autoware_dump_python() {
    let (record, _tmp) = dump_autoware("python");
    assert_eq!(array_len(&record, "node"), 46, "expected 46 nodes");
    assert_eq!(
        array_len(&record, "container"),
        15,
        "expected 15 containers"
    );
    assert_eq!(
        array_len(&record, "load_node"),
        54,
        "expected 54 load_nodes"
    );
}

// ---- Parser parity ----

#[test]
fn test_autoware_parser_parity() {
    let (_, rust_tmp) = dump_autoware("rust");
    let (_, python_tmp) = dump_autoware("python");

    let rust_record = rust_tmp.path().join("record.json");
    let python_record = python_tmp.path().join("record.json");

    let (success, output) = fixtures::compare_records(&rust_record, &python_record);
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

    // Dump to get expected count
    let tmp = tempfile::TempDir::new().unwrap();
    let record_path = tmp.path().join("record.json");

    let mut dump_args = vec![
        "dump".to_string(),
        "--output".to_string(),
        record_path.to_str().unwrap().to_string(),
        "launch".to_string(),
        "--parser".to_string(),
        "rust".to_string(),
    ];
    dump_args.extend(autoware_launch_args());

    let mut dump_proc = ManagedProcess::spawn(
        fixtures::play_launch_cmd(&env).args(&dump_args),
    )
    .expect("failed to spawn dump");

    let status = dump_proc.wait_with_timeout(std::time::Duration::from_secs(60));
    assert!(status.success());

    let expected = fixtures::count_expected_processes(&record_path);
    assert_eq!(expected, 61, "expected 61 processes (46 nodes + 15 containers)");

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
    assert_eq!(
        actual, expected,
        "process count: {actual}/{expected}"
    );

    // _proc dropped here — ManagedProcess::drop kills the process group
}

// ---- Smoke test ----

#[test]
fn test_autoware_smoke_test() {
    // 1. Dump record.json to get expected process count
    let (record, _dump_tmp) = dump_autoware("rust");
    let expected = array_len(&record, "node") + array_len(&record, "container");
    assert_eq!(expected, 61, "expected 61 processes (46 nodes + 15 containers)");

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

    let output_file =
        std::fs::File::create(&output_path).expect("failed to create output file");

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

    // 7. Assert healthy (ignore environment-specific node exits)
    // shape_estimation: requires TensorRT (libnvinfer.so.8) which is GPU-specific
    // rviz2: requires X display server (not available in headless/CI environments)
    let ignored_exits = &["shape_estimation", "rviz2"];
    assert!(
        report.is_healthy(ignored_exits),
        "Smoke test failed:\n{report}"
    );

    // _proc dropped here — ManagedProcess::drop kills the process group
}

#[test]
fn test_autoware_process_count_python() {
    require_autoware();
    let env = fixtures::autoware_env();
    let work_dir = fixtures::test_workspace_path("autoware");

    // Dump to get expected count
    let tmp = tempfile::TempDir::new().unwrap();
    let record_path = tmp.path().join("record.json");

    let mut dump_args = vec![
        "dump".to_string(),
        "--output".to_string(),
        record_path.to_str().unwrap().to_string(),
        "launch".to_string(),
        "--parser".to_string(),
        "python".to_string(),
    ];
    dump_args.extend(autoware_launch_args());

    let mut dump_proc = ManagedProcess::spawn(
        fixtures::play_launch_cmd(&env).args(&dump_args),
    )
    .expect("failed to spawn dump");

    let status = dump_proc.wait_with_timeout(std::time::Duration::from_secs(60));
    assert!(status.success());

    let expected = fixtures::count_expected_processes(&record_path);
    assert_eq!(expected, 61, "expected 61 processes (46 nodes + 15 containers)");

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
    assert_eq!(
        actual, expected,
        "process count: {actual}/{expected}"
    );

    // _proc dropped here — ManagedProcess::drop kills the process group
}
