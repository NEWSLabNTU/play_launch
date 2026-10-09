//! What an include does to launch configurations, across every pairing of
//! launch frontends — the semantics `launch` gives an `<include>`, measured.
//!
//! `IncludeLaunchDescription.execute` returns
//! `[SetLaunchConfiguration(name, value) for each <arg>, description]` and the
//! description's entities run in the includer's own context. Nothing is scoped
//! by the include; only a scoped `<group>` (`PushLaunchConfigurations` ...
//! `PopLaunchConfigurations`) is. So, for every frontend pair:
//!
//! - an include's `<arg>`s reach the included file, and beat its defaults;
//! - an argument set by one include PERSISTS into a later sibling include that
//!   does not pass it — the included file's default never applies there;
//! - what the included file declares or `<let>`s is visible to the includer
//!   afterwards;
//! - a scoped `<group>` around the include undoes all three.
//!
//! The parser used to split this by frontend: an XML target ran in an isolated
//! child context, and a YAML target ran in the includer's context without ever
//! receiving the include's arguments, so every YAML file included with
//! `<arg>`s silently took its own defaults. Neither matched `launch`.
//!
//! Every expectation below is what stock `ros2 launch` (Humble, launch 1.0.14)
//! starts for the fixture, read off the running talkers' logger names; the
//! fixtures are ordinary launch files and can be re-checked the same way:
//!
//! ```text
//! ROS_DOMAIN_ID=<free> ros2 launch tests/fixtures/include_semantics/<case>/parent.launch.<ext>
//! ```
//!
//! Each matrix parent includes `child` four times — `who=a flag=false`,
//! `who=b`, no arguments, `who=d flag=true` — then starts `p_<fmt>_$(var
//! child_let)` with `child_let` declared `none` in the parent and `<let>` to
//! `set_by_child` in the child. The child starts `c_<fmt>_$(var who)_$(var
//! parent_let)` if `$(var flag)`. Under launch's semantics `flag=false` from
//! the first include persists through the second and third, so only the
//! fourth starts a child node, and the parent sees the child's `<let>`.

use play_launch_parser::parse_launch_file;
use play_launch_parser_pyexec::python_test_guard;
use std::{collections::HashMap, path::PathBuf};

fn case(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/include_semantics")
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

/// Node names, sorted — the same set `ros2 launch` starts.
fn node_names(json: &serde_json::Value) -> Vec<String> {
    let mut names: Vec<String> = json["node"]
        .as_array()
        .unwrap()
        .iter()
        .map(|n| n["name"].as_str().unwrap().to_string())
        .collect();
    names.sort();
    names
}

fn assert_nodes(name: &str, expected: &[&str]) {
    let json = resolve(name);
    assert_eq!(
        node_names(&json),
        expected,
        "{name}: node set differs from ros2 launch"
    );
}

/// The file scope the named node was stamped with.
fn scope_file_of(json: &serde_json::Value, node: &str) -> String {
    let scope = json["node"]
        .as_array()
        .unwrap()
        .iter()
        .find(|n| n["name"] == node)
        .unwrap_or_else(|| panic!("no node {node}"))["scope"]
        .as_u64()
        .unwrap_or_else(|| panic!("node {node} carries no scope"));
    json["scopes"][scope as usize]["origin"]["file"]
        .as_str()
        .unwrap_or_else(|| panic!("scope {scope} of {node} is not a file scope"))
        .to_string()
}

// ---- XML and YAML on both sides ----------------------------------------

#[test]
fn xml_includes_yaml_passes_args_and_leaks_like_launch() {
    // The reported bug: the YAML child took `who=default_who flag=true` for
    // every include, so four nodes named `c_yaml_default_who_P` came out.
    assert_nodes("xml_to_yaml", &["c_yaml_d_P", "p_xml_set_by_child"]);
}

#[test]
fn yaml_includes_yaml_passes_args_and_leaks_like_launch() {
    assert_nodes("yaml_to_yaml", &["c_yaml_d_P", "p_yaml_set_by_child"]);
}

#[test]
fn xml_includes_xml_is_not_scoped_by_the_include() {
    // Previously an isolated child context: `flag=false` did not persist, so
    // `c_xml_b_P` and `c_xml_default_who_P` appeared and the parent saw
    // `child_let` as `none`.
    assert_nodes("xml_to_xml", &["c_xml_d_P", "p_xml_set_by_child"]);
}

#[test]
fn yaml_includes_xml_is_not_scoped_by_the_include() {
    assert_nodes("yaml_to_xml", &["c_xml_d_P", "p_yaml_set_by_child"]);
}

#[test]
fn a_node_from_an_included_yaml_file_is_in_that_files_scope() {
    // The YAML path never stamped its records, so they fell back to the root
    // scope and the launch tree put a YAML child's nodes in its includer.
    let json = resolve("xml_to_yaml");
    assert_eq!(scope_file_of(&json, "c_yaml_d_P"), "child.launch.yaml");
    assert_eq!(
        scope_file_of(&json, "p_xml_set_by_child"),
        "parent.launch.xml"
    );
}

// ---- A scoped group is what scopes an include ----------------------------

#[test]
fn a_group_around_an_xml_to_yaml_include_scopes_its_args() {
    // Each include in its own `<group>`: `flag=false` is popped with the
    // first group, the third include falls back to the child's defaults, and
    // the child's `<let>` never reaches the parent.
    assert_nodes(
        "group_xml_to_yaml",
        &["c_yaml_b_P", "c_yaml_default_who_P", "p_xml_none"],
    );
}

#[test]
fn a_group_around_a_yaml_to_xml_include_scopes_its_args() {
    assert_nodes(
        "group_yaml_to_xml",
        &["c_xml_b_P", "c_xml_default_who_P", "p_yaml_none"],
    );
}

#[test]
fn a_let_inside_a_scoped_xml_group_does_not_leak() {
    // `scoped="false"` lets it through; the default `scoped` does not. The
    // group used to restore only namespace and remaps, never configurations.
    assert_nodes(
        "group_let_xml",
        &[
            "after_scoped_outer",
            "after_unscoped_unscoped",
            "inside_scoped",
        ],
    );
}

#[test]
fn a_let_inside_a_scoped_yaml_group_does_not_leak() {
    // The YAML frontend also ignored `scoped:` — every YAML group was scoped
    // for namespace and unscoped for configurations.
    assert_nodes(
        "group_let_yaml",
        &[
            "after_scoped_outer",
            "after_unscoped_unscoped",
            "inside_scoped",
        ],
    );
}

// ---- Python on one side ----------------------------------------------------
//
// A `.launch.py` is the same include as any other (C ABI 7): its includes run
// when it reaches them, in the configurations it has built up, and what it
// sets — `SetLaunchConfiguration`, a `DeclareLaunchArgument` default, what a
// file it included `<let>` — is the includer's afterwards. Before ABI 7 the
// Python half kept its configurations to itself: the includer's `p_*` node
// read `child_let` as `none`, and a `.launch.py` parent's own configurations
// never reached the files it included.

#[test]
fn xml_includes_python_like_launch() {
    let _guard = python_test_guard();
    assert_nodes("xml_to_py", &["c_py_d_P", "p_xml_set_by_child"]);
}

#[test]
fn yaml_includes_python_like_launch() {
    let _guard = python_test_guard();
    assert_nodes("yaml_to_py", &["c_py_d_P", "p_yaml_set_by_child"]);
}

#[test]
fn python_includes_yaml_like_launch() {
    // `p_py_set_by_child`: the `.launch.py`'s own node, AFTER the include,
    // reads what the YAML file it included `let`.
    let _guard = python_test_guard();
    assert_nodes("py_to_yaml_args", &["c_yaml_d", "p_py_set_by_child"]);
}

#[test]
fn python_includes_xml_like_launch() {
    // The Python→XML path also ran its target in an isolated child context,
    // so `c_xml_b` and `c_xml_default_who` appeared here too.
    let _guard = python_test_guard();
    assert_nodes("py_to_xml_args", &["c_xml_d", "p_py_set_by_child"]);
}
