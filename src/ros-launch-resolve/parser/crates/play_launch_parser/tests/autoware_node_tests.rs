//! `<autoware_node>`, Autoware 2.1.0's launch action (#0095).
//!
//! Every expectation is what stock `launch` (Humble) resolves for the
//! fixture under `tests/fixtures/autoware_node/<case>/`, with the upstream
//! action (autowarefoundation/autoware_core,
//! `common/autoware_agnocast_wrapper/autoware_agnocast_wrapper/actions/
//! autoware_node.py`) importable. Re-check with:
//!
//! ```text
//! PYTHONPATH=tests/stock_parity/agnocast_wrapper/python:$PYTHONPATH \
//!   STOCK_OUT=out.json python3 tests/stock_parity/stock_oracle.py \
//!   <this crate>/tests/fixtures/autoware_node/<case>/parent.launch.xml
//! ```
//!
//! (repository-relative paths). The fixtures point `AMENT_PREFIX_PATH` at
//! `fixtures/autoware_node/prefix` with `<set_env>`, which registers
//! `autoware_node_plugins` resources for demo_nodes_cpp's `talker` (built
//! with Agnocast) and `listener` (without).

use play_launch_parser::parse_launch_file;
use std::{collections::HashMap, path::PathBuf};

fn fixture(case: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/autoware_node")
        .join(case);
    std::fs::read_dir(&dir)
        .unwrap_or_else(|e| panic!("fixture {}: {e}", dir.display()))
        .map(|e| e.unwrap().path())
        .find(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with("parent.launch."))
        })
        .unwrap()
}

fn resolve(case: &str) -> serde_json::Value {
    let path = fixture(case);
    let record = parse_launch_file(&path, HashMap::new())
        .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    serde_json::to_value(record).unwrap()
}

fn error(case: &str) -> String {
    let path = fixture(case);
    match parse_launch_file(&path, HashMap::new()) {
        Ok(_) => panic!("{} resolved, but stock launch refuses it", path.display()),
        Err(e) => format!("{e:#}"),
    }
}

fn names<'a>(json: &'a serde_json::Value, list: &str, key: &str) -> Vec<&'a str> {
    let mut v: Vec<&str> = json[list]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| n[key].as_str())
        .collect();
    v.sort_unstable();
    v
}

fn by<'a>(json: &'a serde_json::Value, list: &str, key: &str, name: &str) -> &'a serde_json::Value {
    json[list]
        .as_array()
        .unwrap()
        .iter()
        .find(|n| n[key] == name)
        .unwrap_or_else(|| panic!("no {list} {name}"))
}

fn env<'a>(node: &'a serde_json::Value, key: &str) -> Option<&'a str> {
    node["env"]
        .as_array()
        .and_then(|e| e.iter().find(|kv| kv[0] == key))
        .and_then(|kv| kv[1].as_str())
}

/// `target` given: a component in that container, its plugin from the
/// resource, with the params, remaps and extra arguments; no target: a
/// process with the process attributes and ENABLE_AGNOCAST=0 over the
/// launch file's own `<env>`. A false `if` drops it.
fn assert_forms(json: &serde_json::Value) {
    assert_eq!(names(json, "node", "name"), ["listener_proc"]);
    assert_eq!(names(json, "load_node", "node_name"), ["talker_comp"]);

    let comp = by(json, "load_node", "node_name", "talker_comp");
    assert_eq!(comp["plugin"], "demo_nodes_cpp::Talker");
    assert_eq!(comp["package"], "demo_nodes_cpp");
    assert_eq!(comp["target_container_name"], "/c");
    assert_eq!(comp["namespace"], "/aw");
    assert_eq!(comp["remaps"], serde_json::json!([["chatter", "chat"]]));
    // A bool: the XML/YAML frontends spell it `True`, the Python one `true`,
    // and the model types both alike.
    let is_true =
        |v: &serde_json::Value| v.as_str().is_some_and(|s| s.eq_ignore_ascii_case("true"));
    assert!(
        comp["params"]
            .as_array()
            .unwrap()
            .iter()
            .any(|p| p[0] == "negative" && is_true(&p[1])),
        "{comp}"
    );
    assert!(
        is_true(&comp["extra_args"]["use_intra_process_comms"]),
        "{comp}"
    );

    let proc_ = by(json, "node", "name", "listener_proc");
    assert_eq!(proc_["executable"], "listener");
    assert_eq!(proc_["namespace"], "/aw");
    assert_eq!(env(proc_, "FOO"), Some("bar"));
    // The launch file said 1; the action's transport wins.
    assert_eq!(env(proc_, "ENABLE_AGNOCAST"), Some("0"));
    assert_eq!(env(proc_, "LD_PRELOAD"), None);
}

#[test]
fn target_decides_component_or_process() {
    assert_forms(&resolve("forms"));
}

#[test]
fn the_yaml_frontend_reads_it_the_same_way() {
    assert_forms(&resolve("forms_yaml"));
}

/// ENABLE_AGNOCAST=1: a node built with Agnocast runs on it as a process,
/// target or not, unless `mode="rclcpp"`; one built without it is a plain
/// component. The heaphook goes first in LD_PRELOAD, and a node kept on
/// rclcpp loses one its `<env>` carried.
#[test]
fn agnocast_moves_a_node_out_of_its_container_unless_mode_rclcpp() {
    let json = resolve("agnocast");
    assert_eq!(
        names(&json, "node", "name"),
        ["talker_auto", "talker_proc_auto", "talker_proc_rclcpp"]
    );
    assert_eq!(
        names(&json, "load_node", "node_name"),
        ["listener_comp", "talker_rclcpp"]
    );
    let dir = fixture("agnocast").parent().unwrap().to_path_buf();
    let heaphook = format!("{}/libagnocast_heaphook.so.1", dir.display());

    let auto = by(&json, "node", "name", "talker_auto");
    assert_eq!(env(auto, "ENABLE_AGNOCAST"), Some("1"));
    assert_eq!(env(auto, "LD_PRELOAD"), Some(heaphook.as_str()));

    let with_base = by(&json, "node", "name", "talker_proc_auto");
    assert_eq!(env(with_base, "ENABLE_AGNOCAST"), Some("1"));
    assert_eq!(
        env(with_base, "LD_PRELOAD"),
        Some(format!("{heaphook}:/x/libother.so").as_str())
    );

    let rclcpp = by(&json, "node", "name", "talker_proc_rclcpp");
    assert_eq!(env(rclcpp, "ENABLE_AGNOCAST"), Some("0"));
    assert_eq!(env(rclcpp, "LD_PRELOAD"), Some("/x/libother.so"));
}

/// The `use_agnocast` configuration overrides ENABLE_AGNOCAST either way.
#[test]
fn use_agnocast_overrides_the_environment() {
    let json = resolve("use_agnocast");
    assert_eq!(names(&json, "node", "name"), ["on_by_config"]);
    assert_eq!(names(&json, "load_node", "node_name"), ["off_by_config"]);
    let on = by(&json, "node", "name", "on_by_config");
    assert_eq!(env(on, "ENABLE_AGNOCAST"), Some("1"));
}

#[test]
fn a_target_without_a_registered_component_is_an_error() {
    let e = error("no_plugin");
    assert!(
        e.contains(
            "'add_two_ints_server' of demo_nodes_cpp registers no autoware_node_plugins \
             resource, so the component to load as 'srv' is unknown"
        ),
        "{e}"
    );
}

#[test]
fn an_unknown_mode_is_an_error() {
    let e = error("bad_mode");
    assert!(
        e.contains("'l' has an unknown mode 'agnocast', expected one of auto, rclcpp"),
        "{e}"
    );
}

fn write(suffix: &str, text: &str) -> tempfile::NamedTempFile {
    use std::io::Write;
    let mut file = tempfile::NamedTempFile::with_suffix(suffix).unwrap();
    file.write_all(text.as_bytes()).unwrap();
    file.flush().unwrap();
    file
}

/// Before #0095 any tag this parser did not dispatch was dropped with a
/// WARN, its nodes with it, and `resolve` exited 0. A tag that is no stock
/// action is now refused in both frontends, as `launch`'s frontend refuses
/// it; a stock action this parser does not implement (`<log>`) is still
/// dropped and recorded for `check`.
#[test]
fn an_unknown_action_is_refused_and_a_stock_one_is_recorded() {
    for (suffix, text) in [
        (
            ".launch.xml",
            r#"<launch><autoware_nod pkg="demo_nodes_cpp" exec="talker"/></launch>"#,
        ),
        (
            ".launch.yaml",
            "launch:\n  - autoware_nod:\n      pkg: demo_nodes_cpp\n      exec: talker\n",
        ),
    ] {
        let file = write(suffix, text);
        let e = parse_launch_file(file.path(), HashMap::new())
            .map(|_| ())
            .unwrap_err()
            .to_string();
        assert!(
            e.contains("Unrecognized entity of the type: autoware_nod"),
            "{suffix}: {e}"
        );
    }

    let file = write(
        ".launch.xml",
        r#"<launch><log message="hi"/><node pkg="demo_nodes_cpp" exec="talker"/></launch>"#,
    );
    let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
    assert_eq!(record.node.len(), 1);
    assert_eq!(record.dropped_actions.len(), 1);
    assert_eq!(record.dropped_actions[0].action, "log");
}

/// The Python spelling: the upstream `AutowareNode` class, imported by the
/// launch file, subclasses `launch.Action` and does its work in `execute`,
/// returning a `LoadComposableNodes` or a `Node`. pyexec runs a user
/// action's `execute` as `launch` does and walks what it returns.
#[test]
fn a_launch_py_runs_the_upstream_action() {
    let _guard = play_launch_parser_pyexec::python_test_guard();
    assert_forms(&resolve("forms_py"));
}
