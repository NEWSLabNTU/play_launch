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

/// A launch file's OWN `launch.Substitution` subclass is performed when the
/// action that holds it executes, in the walk's context, as `launch` does
/// (#0073). The fixture defines nav2_common-shaped `RewrittenYaml` and
/// `HasNodeParams`: the first becomes a parameter file holding the rewritten
/// value of a launch configuration, the second decides two conditions.
///
/// Before the fix the file failed to load with
/// `AttributeError: module 'launch' has no attribute 'Substitution'`.
#[test]
fn a_substitution_the_launch_file_defines_is_performed() {
    let _guard = python_test_guard();
    let json = resolve("user_substitution");
    assert_eq!(
        fqns(&json),
        ["/has_params", "/planner", "/unless_lacks"],
        "HasNodeParams decides IfCondition and UnlessCondition"
    );
    let files = node(&json, "planner")["params_files"]
        .as_array()
        .unwrap_or_else(|| panic!("planner has no params_files: {}", node(&json, "planner")));
    assert_eq!(files.len(), 1, "one rewritten file: {files:?}");
    let text = files[0].as_str().unwrap();
    // A file's CONTENT or its path, depending on how the record carries it.
    let content = if text.contains("ros__parameters") {
        text.to_string()
    } else {
        std::fs::read_to_string(text).unwrap_or_else(|e| panic!("{text}: {e}"))
    };
    let doc: serde_yaml_ng::Value = serde_yaml_ng::from_str(&content).unwrap();
    let params = &doc["planner"]["ros__parameters"];
    assert_eq!(
        params["use_sim_time"],
        serde_yaml_ng::Value::Bool(true),
        "rewritten: {content}"
    );
    assert_eq!(
        params["frequency"],
        serde_yaml_ng::Value::from(10.0),
        "kept: {content}"
    );
}

/// Issue #0078: a description built empty and filled with `add_action` and
/// `add_entity`, as `nav2_bringup`'s launch files do, and built from any
/// iterable or with stock's keywords. Stock (Jazzy, `stock_oracle.py`):
/// these six nodes, `b_unless` skipped by its condition.
///
/// Before the fix the file failed to load with
/// `TypeError: LaunchDescription.__new__() missing 1 required positional
/// argument: 'actions'`.
#[test]
fn a_description_filled_with_add_action_executes() {
    let _guard = python_test_guard();
    assert_eq!(
        fqns(&resolve("add_action")),
        [
            "/a",
            "/b_if",
            "/c_entity",
            "/d_tuple",
            "/e_generator",
            "/f_keyword",
        ]
    );
}

/// Issue #0078: a `PythonExpression` performs its parts when it is
/// performed, not when it is built. `generate_launch_description()` builds
/// `PythonExpression(['not ', LaunchConfiguration('use_composition')])`
/// (nav2_bringup) before the argument is declared. Stock (Jazzy,
/// `stock_oracle.py`): these three nodes.
///
/// Before the fix the parts were joined at construction, into
/// `not $(var use_composition)`; `eval` refused it and the condition read the
/// spelled-out text, `invalid condition expression: Unexpected value 'not
/// False'`.
#[test]
fn a_python_expression_is_performed_when_its_condition_is() {
    let _guard = python_test_guard();
    assert_eq!(
        fqns(&resolve("python_expression")),
        ["/literal", "/not_composed", "/sim_mode"]
    );
}

/// Issue #0086's reproduction: `context.perform_substitution` of an
/// `EnvironmentVariable` with a default, as Autoware 2.1.0's
/// `component_state_monitor.launch.py` does. Stock (Humble,
/// `stock_oracle.py`): the default, `0`.
///
/// Before the fix: `AttributeError: 'launch.substitutions.EnvironmentVariable'
/// object has no attribute 'perform'`.
#[test]
fn an_environment_variable_performs_from_an_opaque_function() {
    let _guard = python_test_guard();
    assert_eq!(fqns(&resolve("env_perform_repro")), ["/agnocast_0"]);
}

/// Issue #0092: launch's environment actions write `os.environ`, so a
/// `.launch.py` reads at its top level what an including XML file's
/// `<set_env>` set, and an `OpaqueFunction` reads what a
/// `SetEnvironmentVariable` before it set, scoped by a `GroupAction`. A
/// launch file's own `os.environ` write stays. Stock Humble starts exactly
/// these four (`tests/stock_parity/cases/env_os_environ`, which also compares
/// each node's `env` and `env_unset`).
///
/// The fixture's three `OpaqueFunction`s each build a fresh `Node`: before
/// the walk held the actions it executed, Python reused a dropped one's
/// address and the next node was discarded as "executed more than once".
#[test]
fn environment_actions_write_os_environ() {
    let _guard = python_test_guard();
    assert_eq!(
        fqns(&resolve("env_os_environ")),
        ["/a1_7_home_d", "/a2_8_nohome_d", "/a3_7_home_d", "/top_x"]
    );
    // The parser runs in play_launch's own process: nothing a launch file
    // set stays there.
    for name in ["PL_XML", "PL_SET", "PL_DIRECT"] {
        assert!(std::env::var_os(name).is_none(), "{name} leaked");
    }
    assert!(std::env::var_os("HOME").is_some(), "HOME not restored");
}

/// Issue #0086: every mock substitution performs, through
/// `context.perform_substitution` and `launch.utilities.perform_substitutions`,
/// and `DeclareLaunchArgument(default_value=EnvironmentVariable(...))` (Autoware
/// 2.1.0's `agnocast_env.launch.py`) sets a configuration a later node and an
/// included file read. Stock (Humble, `tests/stock_parity/stock_oracle.py`):
/// these 24 nodes.
///
/// Before the fix the file failed to load with `AttributeError:
/// 'launch.substitutions.EnvironmentVariable' object has no attribute
/// 'perform'`, which every Autoware 2.1.0 launch hit.
#[test]
fn every_substitution_performs_from_python() {
    let _guard = python_test_guard();
    let mut expected = vec![
        "/agnocast_0",
        "/set_from_set_env",
        "/missing_raised",
        "/subdefault_0",
        "/text_literal",
        "/lc_0",
        "/preload_empty",
        "/lcdefault_dflt",
        "/lcmissing_raised",
        "/join_a_b_cd",
        "/joinabs__abs_b",
        "/dir_ok",
        "/file_ok",
        "/anon_stable",
        "/not_true",
        "/and_false",
        "/or_true",
        "/eq_true",
        "/neq_true",
        "/ifelse_yes",
        "/pyexpr_5",
        "/subs_afrom_set_env",
        "/later_0",
        "/child_0_from_set_env",
    ];
    expected.sort();
    assert_eq!(fqns(&resolve("env_perform")), expected);
}
