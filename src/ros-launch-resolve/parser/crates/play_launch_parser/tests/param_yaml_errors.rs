//! An untyped parameter value is read with YAML rules, and where the text
//! came from a substitution a YAML error fails the launch, as in stock
//! `ros2 launch` (Humble): `launch` coerces a performed value with
//! `can_be_str=False` (#0096). Literal text in the launch file keeps itself as
//! a string instead, as the frontend's `can_be_str=True` keeps it.
//!
//! Found on AutoSDV: `$(command 'xacro ...')` produced a URDF with `: ` in a
//! comment, `ros2 launch` refused it, and the Rust parser produced a model.

use play_launch_parser::parse_launch_file;
use std::collections::HashMap;

const URDF: &str = "<?xml version=\"1.0\"?>\n<robot name=\"r\">\n  <!-- note: a: b -->\n</robot>\n";

fn launch(dir: &std::path::Path, value: &str, ty: &str) -> std::path::PathBuf {
    let p = dir.join("p.launch.xml");
    std::fs::write(
        &p,
        format!(
            "<launch>\n  <node pkg=\"demo_nodes_cpp\" exec=\"talker\" name=\"t\">\n    \
             <param name=\"robot_description\" value=\"{value}\"{ty}/>\n  </node>\n</launch>\n"
        ),
    )
    .unwrap();
    p
}

#[test]
fn a_substituted_value_yaml_cannot_read_is_an_error() {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("r.urdf");
    std::fs::write(&urdf, URDF).unwrap();
    let root = launch(
        dir.path(),
        &format!("$(command 'cat {}')", urdf.display()),
        "",
    );
    let msg = parse_launch_file(&root, HashMap::new())
        .unwrap_err()
        .to_string();
    assert!(
        msg.contains("robot_description")
            && msg.contains("using yaml rules: yaml.safe_load() failed"),
        "{msg}"
    );
}

#[test]
fn the_same_value_with_type_str_is_kept() {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("r.urdf");
    std::fs::write(&urdf, URDF).unwrap();
    let root = launch(
        dir.path(),
        &format!("$(command 'cat {}')", urdf.display()),
        " type=\"str\"",
    );
    let record = parse_launch_file(&root, HashMap::new()).unwrap();
    assert_eq!(record.node.len(), 1);
}

/// `evaluate_parameter_dict` raises on a substitution's text too, and so
/// does `ParameterValue` with no `value_type`; a plain Python string, given
/// either way, is kept (each case checked against stock `ros2 launch`).
#[test]
fn a_launch_py_substitution_yaml_cannot_read_is_an_error() {
    let _guard = play_launch_parser_pyexec::python_test_guard();
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("r.urdf");
    std::fs::write(&urdf, URDF).unwrap();
    let command = format!("Command(['cat ', '{}'])", urdf.display());
    let literal = format!("{URDF:?}");
    for (value, refused) in [
        (command.clone(), true),
        (format!("ParameterValue({command})"), true),
        (literal.clone(), false),
        (format!("ParameterValue({literal})"), false),
    ] {
        let root = dir.path().join("p.launch.py");
        std::fs::write(
            &root,
            format!(
                "from launch import LaunchDescription\n\
                 from launch.substitutions import Command\n\
                 from launch_ros.actions import Node\n\
                 from launch_ros.parameter_descriptions import ParameterValue\n\
                 def generate_launch_description():\n    \
                 return LaunchDescription([Node(package='demo_nodes_cpp', executable='talker', \
                 name='t', parameters=[{{'robot_description': {value}}}])])\n"
            ),
        )
        .unwrap();
        match parse_launch_file(&root, HashMap::new()) {
            Err(e) if refused => {
                let msg = e.to_string();
                assert!(msg.contains("yaml.safe_load() failed"), "{value}: {msg}");
            }
            Ok(r) if !refused => assert_eq!(r.node.len(), 1, "{value}"),
            other => panic!(
                "{value}: refused={refused}, got {:?}",
                other.map(|r| r.node.len())
            ),
        }
    }
}

#[test]
fn literal_text_yaml_cannot_read_is_kept_as_a_string() {
    let dir = tempfile::tempdir().unwrap();
    let escaped = URDF
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;");
    let root = launch(dir.path(), &escaped, "");
    let record = parse_launch_file(&root, HashMap::new()).unwrap();
    assert_eq!(record.node.len(), 1);
}
