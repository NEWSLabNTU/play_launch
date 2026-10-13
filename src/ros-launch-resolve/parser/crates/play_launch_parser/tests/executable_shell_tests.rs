//! `shell=True` / `shell="true"` on a process that is not a ROS node.
//!
//! launch hands such a process to osrf_pycommon's `async_execute_process`,
//! which joins the command line with spaces and runs it through `/bin/sh -c`.
//! The parsers kept the parts as argv, so `ros_gz_sim/gz_sim.launch.py` —
//! `ExecuteProcess(cmd=['ruby $(which gz) sim', gz_args, ...], shell=True)` —
//! gave `gz sim` its whole `gz_args` ("-r -s world.sdf") as one argument,
//! and Gazebo exited at once.

use play_launch_parser::parse_launch_file;
use play_launch_parser_pyexec::python_test_guard;
use std::{collections::HashMap, io::Write};
use tempfile::NamedTempFile;

fn parse(suffix: &str, body: &str) -> serde_json::Value {
    let mut file = NamedTempFile::with_suffix(suffix).unwrap();
    file.write_all(body.as_bytes()).unwrap();
    file.flush().unwrap();
    let dump = parse_launch_file(file.path(), HashMap::new())
        .unwrap_or_else(|e| panic!("{suffix} should parse: {e}"));
    serde_json::to_value(dump).unwrap()
}

fn by_name<'a>(json: &'a serde_json::Value, name: &str) -> &'a serde_json::Value {
    json["node"]
        .as_array()
        .unwrap()
        .iter()
        .find(|e| e["name"] == name)
        .unwrap_or_else(|| panic!("{name} missing: {json}"))
}

fn cmd(entry: &serde_json::Value) -> Vec<String> {
    serde_json::from_value(entry["cmd"].clone()).unwrap()
}

#[test]
fn xml_shell_executable_runs_through_sh() {
    let json = parse(
        ".launch.xml",
        r#"<launch>
  <arg name="world" default="office.sdf"/>
  <executable cmd="gz sim" name="sim" shell="true">
    <arg value="-r -s $(var world)"/>
  </executable>
  <executable cmd="gz sim -g" name="gui"/>
</launch>"#,
    );
    let sim = by_name(&json, "sim");
    assert_eq!(
        cmd(sim),
        ["/bin/sh", "-c", "gz sim -r -s office.sdf"],
        "{sim}"
    );
    assert_eq!(sim["executable"], "/bin/sh", "{sim}");
    let gui = by_name(&json, "gui");
    assert_eq!(
        cmd(gui),
        ["gz", "sim", "-g"],
        "no shell, argv as before: {gui}"
    );
}

#[test]
fn yaml_shell_executable_runs_through_sh() {
    let json = parse(
        ".launch.yaml",
        r#"launch:
- executable:
    cmd: echo one two
    name: echo
    shell: "true"
"#,
    );
    let e = by_name(&json, "echo");
    assert_eq!(cmd(e), ["/bin/sh", "-c", "echo one two"], "{e}");
}

#[test]
fn python_shell_execute_process_runs_through_sh() {
    let _guard = python_test_guard();
    let json = parse(
        ".launch.py",
        r#"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='-r -s office.sdf'),
        ExecuteProcess(cmd=['ruby gz sim', LaunchConfiguration('gz_args'),
                            '--force-version', '8'],
                       name='gazebo', shell=True),
        ExecuteProcess(cmd=['sleep', '1'], name='plain'),
    ])
"#,
    );
    let gz = by_name(&json, "gazebo");
    assert_eq!(
        cmd(gz),
        [
            "/bin/sh",
            "-c",
            "ruby gz sim -r -s office.sdf --force-version 8"
        ],
        "{gz}"
    );
    let plain = by_name(&json, "plain");
    assert_eq!(
        cmd(plain),
        ["sleep", "1"],
        "no shell, argv as before: {plain}"
    );
}
