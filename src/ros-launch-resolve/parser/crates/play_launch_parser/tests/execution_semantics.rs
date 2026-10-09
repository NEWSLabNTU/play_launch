//! How a launch description EXECUTES — in order, each action when it is
//! reached — measured against stock `launch` and asserted here.
//!
//! Every expectation is what stock `launch` (Humble, launch 1.0.14,
//! launch_ros 0.19.14) resolves for the fixture under
//! `tests/fixtures/execution_semantics/<case>/`. They were read off a real
//! `LaunchService` run over the file in which `ExecuteLocal.execute` and
//! `LoadComposableNodes.execute` record the command line and the LoadNode
//! request instead of starting anything; everything else — conditions,
//! scoping, includes, `OpaqueFunction` — was stock code. Re-check with:
//!
//! ```text
//! ros2 launch tests/fixtures/execution_semantics/<case>/parent.launch.<ext>
//! ```
//!
//! Each case is a defect the Python frontend had, or a launch semantic the
//! XML frontend got wrong, before C ABI 7.

use play_launch_parser::parse_launch_file;
use play_launch_parser_pyexec::python_test_guard;
use std::{collections::HashMap, path::PathBuf};

fn case(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/execution_semantics")
        .join(name);
    std::fs::read_dir(&dir)
        .unwrap_or_else(|e| panic!("fixture {}: {e}", dir.display()))
        .map(|e| e.unwrap().path())
        .find(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with("parent.launch."))
        })
        .unwrap_or_else(|| panic!("fixture {} has no parent.launch.*", dir.display()))
}

fn resolve(name: &str) -> serde_json::Value {
    let path = case(name);
    let record = parse_launch_file(&path, HashMap::new())
        .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    serde_json::to_value(record).unwrap()
}

/// `namespace/name` of every node, sorted — the set stock starts.
fn fqns(json: &serde_json::Value) -> Vec<String> {
    let mut out: Vec<String> = json["node"]
        .as_array()
        .unwrap()
        .iter()
        .map(|n| {
            let ns = n["namespace"].as_str().unwrap_or("");
            let name = n["name"].as_str().unwrap();
            if ns.is_empty() || ns == "/" {
                format!("/{name}")
            } else {
                format!("{ns}/{name}")
            }
        })
        .collect();
    out.sort();
    out
}

fn node<'a>(json: &'a serde_json::Value, name: &str) -> &'a serde_json::Value {
    json["node"]
        .as_array()
        .unwrap()
        .iter()
        .find(|n| n["name"] == name)
        .unwrap_or_else(|| panic!("no node {name}"))
}

/// The global parameters a node was started with, in order.
fn global_param_names(json: &serde_json::Value, name: &str) -> Vec<String> {
    node(json, name)["global_params"]
        .as_array()
        .map(|gp| {
            gp.iter()
                .map(|p| p[0].as_str().unwrap().to_string())
                .collect()
        })
        .unwrap_or_default()
}

/// The remappings a node was started with, as `from:=to`, in order.
fn remaps(json: &serde_json::Value, name: &str) -> Vec<String> {
    node(json, name)["remaps"]
        .as_array()
        .unwrap()
        .iter()
        .map(|r| format!("{}:={}", r[0].as_str().unwrap(), r[1].as_str().unwrap()))
        .collect()
}

/// A `.launch.py` included from XML declares, sets and includes XML and YAML
/// files. What it sets reaches each of its includes AS OF that include (the
/// XML child sees `py_set=one`, the YAML child `two`), what the XML child
/// `<let>` reaches the YAML child and the `.launch.py`'s own later node, and
/// all of it — the argument it was passed included — is the XML parent's
/// afterwards.
///
/// Before C ABI 7 none of the `.launch.py`'s configurations left the Python
/// half and its includes ran after it had finished: the parent failed with
/// `Undefined variable: 'py_set'`.
#[test]
fn a_launch_py_s_configurations_flow_to_its_includes_and_back() {
    let _guard = python_test_guard();
    assert_eq!(
        fqns(&resolve("py_flow")),
        [
            "/after_two_decl_from_parent_xl",
            "/cx_one_decl_from_parent",
            "/cy_two_decl_xl",
            "/mid_from_parent_two",
        ]
    );
}

/// `condition=` on a Python `GroupAction`, `SetLaunchConfiguration` and
/// `IncludeLaunchDescription`, and `if=`/`unless=` on XML groups, includes,
/// nodes and `<let>`.
///
/// The Python mocks did their work in their constructors, which run before
/// the group or condition holding them exists: `py_group_if_false` was
/// started and `SetLaunchConfiguration(..., condition=IfCondition('false'))`
/// was applied (`py_cs_should_not` instead of `py_cs_unset`).
#[test]
fn conditions_are_evaluated_when_the_action_executes() {
    let _guard = python_test_guard();
    assert_eq!(
        fqns(&resolve("conditions")),
        [
            "/c_inc_unless_false",
            "/g_unless_false",
            "/last_py_if_true",
            "/n_True",
            "/n_eval",
            "/n_one",
            "/py_cs_unset",
            "/py_eq",
            "/py_group_unless_false",
            "/py_py_if_true",
        ]
    );
}

/// `ros_namespace` is a launch configuration: a scoped group pops it, an
/// include does not. So a `<push-ros-namespace>` at the top of an included
/// XML file, and a `PushRosNamespace` at the top of an included `.launch.py`,
/// apply to the includer's later nodes.
///
/// The Python one used to stop at the file boundary
/// (`/pushed_by_child/after_push_py`).
#[test]
fn a_namespace_pushed_by_an_included_file_applies_after_it() {
    let _guard = python_test_guard();
    assert_eq!(
        fqns(&resolve("namespace")),
        [
            "/abs/abs_ns",
            "/g1/back_in_g1",
            "/g1/g2/in_g1_g2",
            "/g1/g2/rel/rel_ns",
            "/g1/in_g1",
            "/gns/in_gns",
            "/pushed_by_child/after_push_child",
            "/pushed_by_child/in_child",
            "/pushed_by_child/py_pushed/after_push_py",
            "/pushed_by_child/py_pushed/in_py",
            "/pushed_by_child/py_pushed/in_py_after_group",
            "/pushed_by_child/py_pushed/py_group/in_py_group",
            "/pushed_by_child/unscoped/after_unscoped_group",
        ]
    );
}

/// `<set_parameter>` / `SetParameter` and `<set_remap>` / `SetRemap` append to
/// two lists `launch_ros` keeps INSIDE the launch configurations, which a
/// scoped group saves by shallow copy. So:
///
/// - they are positional: a node declared before one does not get it;
/// - a group pops one only if the list did not exist before the group
///   (`p_first_in_group` is gone after its group); once it exists, an append
///   inside a group mutates the list the group saved and survives the group
///   (`p_second_in_group`, `r_second_in_group`, and the Python
///   `p_py_group`).
///
/// Every node used to get the FINAL global parameter set wherever it was
/// declared, groups restored remaps by count (dropping
/// `r_second_in_group`), and a Python `SetRemap` did nothing at all.
#[test]
fn global_parameters_and_remaps_are_positional_lists() {
    let _guard = python_test_guard();
    let json = resolve("global_lists");
    let expect: &[(&str, &[&str], &[&str])] = &[
        ("before_any", &[], &[]),
        (
            "in_first_group",
            &["p_first_in_group"],
            &["r_first_in_group:=x"],
        ),
        ("after_first_group", &[], &[]),
        ("after_top", &["p_top"], &["r_top:=y"]),
        (
            "after_second_group",
            &["p_top", "p_second_in_group"],
            &["r_top:=y", "r_second_in_group:=z"],
        ),
        (
            "py_before",
            &["p_top", "p_second_in_group"],
            &["r_top:=y", "r_second_in_group:=z"],
        ),
        (
            "py_after",
            &["p_top", "p_second_in_group", "p_py"],
            &["r_top:=y", "r_second_in_group:=z", "r_py:=w"],
        ),
        (
            "py_in_group",
            &["p_top", "p_second_in_group", "p_py", "p_py_group"],
            &["r_top:=y", "r_second_in_group:=z", "r_py:=w"],
        ),
        (
            "py_after_group",
            &["p_top", "p_second_in_group", "p_py", "p_py_group"],
            &["r_top:=y", "r_second_in_group:=z", "r_py:=w"],
        ),
        (
            "after_py",
            &["p_top", "p_second_in_group", "p_py", "p_py_group"],
            &["r_top:=y", "r_second_in_group:=z", "r_py:=w"],
        ),
    ];
    for (name, params, remapped) in expect {
        assert_eq!(
            &global_param_names(&json, name),
            params,
            "{name}: global params"
        );
        assert_eq!(&remaps(&json, name), remapped, "{name}: remaps");
    }
}
