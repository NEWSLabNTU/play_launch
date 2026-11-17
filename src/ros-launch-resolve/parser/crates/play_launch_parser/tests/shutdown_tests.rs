//! How a process ENDS, as `ros2 launch` decides it.
//!
//! - #0084: `sigterm_timeout` / `sigkill_timeout`, the action's own (Jazzy
//!   XML, and the Python API on every distro), else the launch
//!   configuration of the same name, else 5 (recorded as absent).
//! - #0090: an `on_exit` that is a FUNCTION of the exit event, in an
//!   `OnProcessExit` handler or on the action itself, is evaluated for every
//!   exit code; the codes it returns a shutdown for are recorded.

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

fn find<'a>(json: &'a serde_json::Value, list: &str, name: &str) -> &'a serde_json::Value {
    json[list]
        .as_array()
        .unwrap()
        .iter()
        .find(|e| e["name"] == name || e["name"] == format!("/{name}"))
        .unwrap_or_else(|| panic!("{name} missing from {list}: {json}"))
}

#[test]
fn xml_timeouts_are_the_actions_else_the_configurations() {
    let json = parse(
        ".launch.xml",
        r#"<launch>
  <executable cmd="true" name="default"/>
  <executable cmd="true" name="own" sigterm_timeout="20" sigkill_timeout="7.5"/>
  <node pkg="demo_nodes_cpp" exec="talker" name="talker" sigterm_timeout="5"/>
  <node_container pkg="rclcpp_components" exec="component_container" name="c"
                  namespace="/t" sigkill_timeout="1"/>
  <let name="sigterm_timeout" value="30"/>
  <executable cmd="true" name="configured"/>
</launch>"#,
    );
    let d = find(&json, "node", "default");
    assert!(d.get("sigterm_timeout").is_none() && d.get("sigkill_timeout").is_none());
    let own = find(&json, "node", "own");
    assert_eq!(own["sigterm_timeout"], 20.0);
    assert_eq!(own["sigkill_timeout"], 7.5);
    // 5 is launch's default, recorded as absent.
    assert!(
        find(&json, "node", "talker")
            .get("sigterm_timeout")
            .is_none()
    );
    assert_eq!(find(&json, "container", "c")["sigkill_timeout"], 1.0);
    let c = find(&json, "node", "configured");
    assert_eq!(c["sigterm_timeout"], 30.0);
    assert!(c.get("sigkill_timeout").is_none());
}

#[test]
fn yaml_reads_a_native_number() {
    let json = parse(
        ".launch.yaml",
        r#"launch:
- executable:
    cmd: "true"
    name: own
    sigterm_timeout: 20
    sigkill_timeout: "3"
"#,
    );
    let own = find(&json, "node", "own");
    assert_eq!(own["sigterm_timeout"], 20.0);
    assert_eq!(own["sigkill_timeout"], 3.0);
}

#[test]
fn a_timeout_that_is_not_a_number_is_refused() {
    let mut file = NamedTempFile::with_suffix(".launch.xml").unwrap();
    file.write_all(br#"<launch><executable cmd="true" sigterm_timeout="soon"/></launch>"#)
        .unwrap();
    let err = parse_launch_file(file.path(), HashMap::new()).unwrap_err();
    assert!(err.to_string().contains("sigterm_timeout 'soon'"), "{err}");
}

const PY: &str = r#"
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown as ShutdownEvent
from launch.substitutions import LaunchConfiguration


def only_two(event, _context):
    print('probed', event.returncode)
    if event.returncode != 2:
        return []
    return [LogInfo(msg='usage error'), EmitEvent(event=ShutdownEvent(reason='usage'))]


def on_failure(event, _context):
    return None if event.returncode == 0 else Shutdown()


def always(_event, _context):
    return [Shutdown()]


calls = []


def stateful(event, _context):
    calls.append(event.returncode)
    return [Shutdown()] if len(calls) > 400 else []


def raises(event, _context):
    raise RuntimeError('needs a real process')


def generate_launch_description():
    cond = ExecuteProcess(cmd=['true'], name='cond', respawn=True)
    early = ExecuteProcess(cmd=['true'], name='early')
    stated = ExecuteProcess(cmd=['true'], name='stated')
    refused = ExecuteProcess(cmd=['true'], name='refused')
    return LaunchDescription([
        DeclareLaunchArgument('t', default_value='12'),
        cond,
        RegisterEventHandler(OnProcessExit(target_action=cond, on_exit=only_two)),
        RegisterEventHandler(OnProcessExit(target_action=early, on_exit=always)),
        early,
        ExecuteProcess(cmd=['true'], name='own_fn', on_exit=on_failure,
                       sigterm_timeout=LaunchConfiguration('t'), sigkill_timeout='20'),
        stated,
        RegisterEventHandler(OnProcessExit(target_action=stated, on_exit=stateful)),
        refused,
        RegisterEventHandler(OnProcessExit(target_action=refused, on_exit=raises)),
    ])
"#;

#[test]
fn python_on_exit_functions_and_timeouts() {
    let _guard = python_test_guard();
    let json = parse(".launch.py", PY);

    let cond = find(&json, "node", "cond");
    assert_eq!(
        cond["shutdown_on_exit_codes"],
        serde_json::json!({ "only": [2] })
    );
    assert!(cond.get("on_exit_shutdown").is_none(), "{cond}");

    // A function that shuts down on every exit is `on_exit=Shutdown()`,
    // registered before its target or not.
    let early = find(&json, "node", "early");
    assert_eq!(early["on_exit_shutdown"], true, "{early}");
    assert!(early.get("shutdown_on_exit_codes").is_none(), "{early}");

    let own = find(&json, "node", "own_fn");
    assert_eq!(
        own["shutdown_on_exit_codes"],
        serde_json::json!({ "except": [0] })
    );
    assert_eq!(own["sigterm_timeout"], 12.0);
    assert_eq!(own["sigkill_timeout"], 20.0);

    // A function that keeps state, or raises, is refused: nothing recorded.
    for name in ["stated", "refused"] {
        let e = find(&json, "node", name);
        assert!(
            e.get("shutdown_on_exit_codes").is_none() && e.get("on_exit_shutdown").is_none(),
            "{name}: {e}"
        );
    }
}
