//! Issue #0066: `--parser python` must RECORD a plain `ExecuteProcess`, not run it.
//!
//! The Python dumper's action visitor had no `ExecuteProcess` branch, so such
//! an action fell through to stock `execute()`, which starts the process. A
//! `resolve --parser python` therefore ran every command a launch file
//! declared, and the processes were missing from the model it wrote.
//!
//! The fixture's process creates a file. After `resolve`, the file must not
//! exist, and the model must contain the process.

use play_launch_tests::fixtures;

const LAUNCH_PY: &str = r#"
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['touch', 'MARKER'], name='toucher'),
    ])
"#;

#[test]
fn resolve_python_records_execute_process_without_running_it() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let dir = tmp.path();
    let marker = dir.join("ran.marker");
    let launch = dir.join("touch.launch.py");
    std::fs::write(
        &launch,
        LAUNCH_PY.replace("MARKER", marker.to_str().unwrap()),
    )
    .expect("write launch");
    let model = dir.join("model.yaml");

    let out = fixtures::play_launch_cmd(&env)
        .current_dir(dir)
        .args(["resolve", "--parser", "python"])
        .arg(&launch)
        .arg("-o")
        .arg(&model)
        .output()
        .expect("run play_launch resolve");
    let log = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(out.status.success(), "resolve failed:\n{log}");

    assert!(
        !marker.exists(),
        "`resolve --parser python` RAN the ExecuteProcess: {} exists (issue #0066)\n{log}",
        marker.display()
    );

    let yaml = std::fs::read_to_string(&model).expect("model written");
    assert!(
        yaml.contains("toucher") && yaml.contains("touch"),
        "the ExecuteProcess is missing from the model (issue #0066):\n{yaml}"
    );
}
