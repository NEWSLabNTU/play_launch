//! Issue 0041 — `ThisLaunchFile()` must reach the record as an absolute path.
//!
//! The `pyexec` stand-in returned the literal `$(this-launch-file)` under a
//! comment saying the real path "would be set during parsing"; nothing ever
//! set it, and the host grammar knew no such token, so the literal reached the
//! record, the model and the spawned command line.
//!
//! The oracle is `launch` itself, and it settles the open question in the
//! issue: `ThisLaunchFile` IS the frontend token `filename`
//! (`@expose_substitution('filename')` in
//! `launch/substitutions/this_launch_file.py`) and its `perform` returns
//! `context.locals.current_launch_file_path` — the absolute PATH. So there is
//! no third concept to add to the grammar; `$(filename)` was simply returning
//! the basename where ROS 2 returns the path, and both halves are fixed by
//! making it the path.
//!
//! Like `dirname_tests.rs`, these live in their own test binary: the Python
//! cases enter the interpreter, and the tests share one fixture directory.

use play_launch_parser::parse_launch_file;
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::{Mutex, MutexGuard, OnceLock},
};

/// Registers the Python backend and serialises entry into the interpreter, the
/// way `python_tests.rs` does — tests in one binary run on separate threads.
fn python_test_guard() -> MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    play_launch_parser_pyexec::register();
    LOCK.get_or_init(|| Mutex::new(()))
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/launch")
}

/// Parse `name` from the fixture directory and return its whole node list.
fn parsed_nodes(name: &str) -> serde_json::Value {
    let path = fixture_dir().join(name);
    let record = parse_launch_file(&path, HashMap::new())
        .unwrap_or_else(|e| panic!("parsing '{name}' should succeed: {e}"));
    serde_json::to_value(record).unwrap()["node"].clone()
}

fn node<'a>(nodes: &'a serde_json::Value, name: &str) -> &'a serde_json::Value {
    nodes
        .as_array()
        .expect("node list")
        .iter()
        .find(|n| n["name"].as_str() == Some(name))
        .unwrap_or_else(|| panic!("no node named {name}"))
}

fn param(node: &serde_json::Value, key: &str) -> String {
    node["params"]
        .as_array()
        .expect("params")
        .iter()
        .find(|p| p[0].as_str() == Some(key))
        .unwrap_or_else(|| panic!("no parameter {key}"))[1]
        .as_str()
        .expect("parameter value")
        .to_string()
}

/// `value` must be the absolute path of the fixture `name`, not its basename
/// and not a token.
fn assert_is_fixture_file(value: &str, name: &str, form: &str) {
    let path = Path::new(value);
    assert!(
        path.is_absolute(),
        "{form}: ThisLaunchFile must be absolute, got {value:?}"
    );
    assert_eq!(
        std::fs::canonicalize(path).unwrap_or_else(|e| panic!("{form}: {value}: {e}")),
        std::fs::canonicalize(fixture_dir().join(name)).unwrap(),
        "{form}: ThisLaunchFile must be the launch file's own path"
    );
}

/// The reported defect, in every field a node has.
#[test]
fn this_launch_file_resolves_in_every_field_of_a_python_node() {
    let _guard = python_test_guard();
    let name = "test_this_launch_file.launch.py";
    let nodes = parsed_nodes(name);
    let probe = node(&nodes, "this_launch_file_probe");

    // Case 1: a substitution as a parameter VALUE.
    assert_is_fixture_file(&param(probe, "launch_file"), name, "parameter value");

    // Case 2: a substitution naming a parameter FILE. `to_record` stores the
    // file's CONTENT, so an unresolved path is not merely ugly — the file is
    // silently not read and the path is stored in its place. The fixture names
    // ITSELF, so the marker in its docstring is the proof it was read.
    let params_files = probe["params_files"].as_array().expect("params_files");
    assert_eq!(params_files.len(), 1);
    let content = params_files[0].as_str().unwrap();
    assert!(
        content.contains("this_launch_file_probe_loaded"),
        "the parameter file must have been READ, got {content:?}"
    );

    // Case 3: arguments and remappings.
    let args = probe["args"].as_array().expect("args");
    assert_is_fixture_file(args[0].as_str().unwrap(), name, "argument");
    let remap = probe["remaps"].as_array().expect("remaps")[0][1]
        .as_str()
        .unwrap();
    assert_eq!(
        remap,
        fixture_dir().join(name).join("topic").display().to_string(),
        "a PathJoinSubstitution over ThisLaunchFile must be resolved"
    );

    // Nothing may reach the command line — what actually gets spawned — with
    // either spelling of the token still in it.
    for word in probe["cmd"].as_array().expect("cmd") {
        let word = word.as_str().unwrap();
        for token in ["$(this-launch-file)", "$(filename)"] {
            assert!(
                !word.contains(token),
                "unresolved {token} reached the command line: {word:?}"
            );
        }
    }
}

/// The deliberate non-change: `$(var ...)` is preserved for replay-time
/// resolution (`execution/node_cmdline.rs`), and resolving the file-location
/// tokens must not touch it.
#[test]
fn a_launch_configuration_parameter_is_still_preserved() {
    let _guard = python_test_guard();
    let nodes = parsed_nodes("test_this_launch_file.launch.py");
    assert_eq!(
        param(node(&nodes, "this_launch_file_probe"), "replay_var"),
        "$(var an_unset_argument)",
    );
}

/// `ThisLaunchFile()` belongs to the file that DECLARED the node, not to the
/// root launch file — which is why the captures are resolved per execution
/// rather than at record-conversion time, where the context has been restored
/// to the root.
#[test]
fn an_included_python_file_resolves_to_its_own_path() {
    let _guard = python_test_guard();
    let nodes = parsed_nodes("test_this_launch_file.launch.py");
    let included = param(
        node(&nodes, "this_launch_file_included_probe"),
        "launch_file",
    );
    let expected = fixture_dir()
        .parent()
        .unwrap()
        .join("includes/test_this_launch_file_included.launch.py");
    assert_eq!(
        std::fs::canonicalize(&included).unwrap_or_else(|e| panic!("{included}: {e}")),
        std::fs::canonicalize(expected).unwrap(),
        "an included .launch.py must resolve ThisLaunchFile to ITS own path"
    );
}

/// The XML half of the same decision: `$(filename)` is the path, because that
/// is what the token means in ROS 2. It used to be the basename.
#[test]
fn filename_in_xml_is_the_launch_file_path() {
    let name = "test_this_launch_file.launch.xml";
    let nodes = parsed_nodes(name);
    assert_is_fixture_file(
        &param(node(&nodes, "filename_probe"), "launch_file"),
        name,
        "XML $(filename)",
    );
}
