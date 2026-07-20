use std::process::Command;

use play_launch_tests::fixtures;
use play_launch_tests::process::ManagedProcess;

fn launch_file() -> String {
    fixtures::test_workspace_path("io_stress")
        .join("launch/io_stress.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

/// Check if the io_stress ROS package is available.
fn io_stress_available(env: &std::collections::HashMap<String, String>) -> bool {
    Command::new("ros2")
        .env_clear()
        .envs(env)
        .args(["pkg", "prefix", "io_stress"])
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

// ---- Resolve tests (Phase 47.B6: model, not record.json) ----

#[test]
fn test_resolve_io_stress_rust() {
    let env = fixtures::install_env();
    if !io_stress_available(&env) {
        eprintln!("SKIP: io_stress package not built");
        return;
    }

    let (model, _tmp) = fixtures::resolve_model(&env, &launch_file(), None, "rust");
    let (plain, containers, composables) = fixtures::model_entity_counts(&model);

    assert_eq!(plain, 10, "expected 10 nodes");
    assert_eq!(containers, 0, "expected 0 containers");
    assert_eq!(composables, 0, "expected 0 composable nodes");
}

#[test]
fn test_resolve_io_stress_python() {
    let env = fixtures::install_env();
    if !io_stress_available(&env) {
        eprintln!("SKIP: io_stress package not built");
        return;
    }

    let (model, _tmp) = fixtures::resolve_model(&env, &launch_file(), None, "python");
    let (plain, containers, composables) = fixtures::model_entity_counts(&model);

    assert_eq!(plain, 10, "expected 10 nodes");
    assert_eq!(containers, 0, "expected 0 containers");
    assert_eq!(composables, 0, "expected 0 composable nodes");
}

// ---- Launch test ----

#[test]
fn test_launch_io_stress() {
    let env = fixtures::install_env();
    if !io_stress_available(&env) {
        eprintln!("SKIP: io_stress package not built");
        return;
    }

    let launch = launch_file();
    let work_dir = fixtures::test_workspace_path("io_stress");

    let (model, _tmp) = fixtures::resolve_model(&env, &launch, None, "rust");
    let expected = fixtures::count_expected_processes_from_model(&model);
    assert_eq!(expected, 10, "expected 10 processes");

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
