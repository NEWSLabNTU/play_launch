//! What a launch file's own `OpaqueFunction` may do with `launch_ros` (#0100).
//!
//! The fixture is written after ceilbot's `nav2_composed.launch.py`: it waits
//! for a container's `list_nodes` through
//! `launch_ros.ros_adapters.get_ros_node`, reading `context.is_shutdown`, and
//! opens the copy of a parameter file whose substitutions
//! `ParameterFile(..., allow_substs=True).evaluate(context)` performed. Each
//! function returns a node only when it got that far.

use play_launch_parser::parse_launch_file;
use play_launch_parser_pyexec::python_test_guard;
use std::{collections::HashMap, path::PathBuf};

fn resolve(args: &[(&str, &str)]) -> serde_json::Value {
    let _guard = python_test_guard();
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/opaque_ros/opaque_ros.launch.py");
    let args: HashMap<String, String> = args
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect();
    let record =
        parse_launch_file(&path, args).unwrap_or_else(|e| panic!("{}: {e:#}", path.display()));
    serde_json::to_value(record).unwrap()
}

fn node<'a>(json: &'a serde_json::Value, name: &str) -> &'a serde_json::Value {
    json["node"]
        .as_array()
        .unwrap()
        .iter()
        .find(|n| n["name"] == name)
        .unwrap_or_else(|| panic!("no node {name}: {json}"))
}

/// A walk has no graph: the service is found and the request answered at
/// once, so the wait returns where the running launch goes on.
#[test]
fn a_wait_for_a_service_returns() {
    node(&resolve(&[]), "after_wait");
}

/// The copy has the file's `$(var robot)` performed in the walk's context,
/// and `evaluate` gives the same file twice, as `launch_ros` keeps it.
#[test]
fn parameter_file_evaluate_performs_its_substitutions() {
    for robot in ["r1", "r2"] {
        let json = resolve(&[("robot", robot)]);
        let params = &node(&json, "after_params")["params"];
        assert!(
            params
                .as_array()
                .unwrap()
                .iter()
                .any(|p| p[0] == "copied" && p[1] == robot),
            "{params}"
        );
    }
}
