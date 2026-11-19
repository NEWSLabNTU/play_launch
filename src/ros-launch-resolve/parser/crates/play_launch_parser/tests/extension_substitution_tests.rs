//! Substitutions only a launch extension defines (#0099).
//!
//! `$(name …)` that is none of `launch`'s own is performed by the class an
//! extension registers with `@expose_substitution(name)`, loaded from the
//! `launch.frontend.launch_extension` entry points as `launch`'s
//! `Parser.load_launch_extensions` loads them. The extension is the fixture's
//! `python/fixture_ext`, written as `ceilbot_profiles` writes its
//! `$(robot-value …)`; `python/broken_ext` fails to import, as an extension
//! may, and `launch` skips it.
//!
//! The fixture directory is put on `PYTHONPATH` before the interpreter
//! starts, which is how `ros2 launch` finds an installed extension too.

use play_launch_parser::parse_launch_file;
use play_launch_parser_pyexec::python_test_guard;
use std::{collections::HashMap, path::PathBuf, sync::OnceLock};

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/extension_substitution")
        .join(name)
}

/// The fixture's extensions on `PYTHONPATH`, once, before anything in this
/// process starts the interpreter: every test calls this first, and only this
/// file's tests run in this process.
fn extensions_importable() {
    static DONE: OnceLock<()> = OnceLock::new();
    DONE.get_or_init(|| {
        let mut path = fixture("python").into_os_string();
        if let Some(rest) = std::env::var_os("PYTHONPATH") {
            path.push(":");
            path.push(rest);
        }
        // SAFETY: every test in this process blocks on this `OnceLock` before
        // it does anything else, so no other thread reads the environment
        // while it is written.
        unsafe { std::env::set_var("PYTHONPATH", path) };
    });
}

fn resolve(name: &str, args: &[(&str, &str)]) -> serde_json::Value {
    extensions_importable();
    let _guard = python_test_guard();
    let args: HashMap<String, String> = args
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect();
    let path = fixture(name);
    let record =
        parse_launch_file(&path, args).unwrap_or_else(|e| panic!("{}: {e:#}", path.display()));
    serde_json::to_value(record).unwrap()
}

/// The one node, with what the extension's three substitutions gave it: a
/// class performing its arguments, a class reading the launch configuration
/// from the context, and a parse function exposed by itself.
fn assert_performed(json: &serde_json::Value, robot: &str) {
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "{json}");
    let node = &nodes[0];
    assert_eq!(node["name"], format!("talker_{robot}"), "{node}");
    let param = |name: &str| {
        node["params"]
            .as_array()
            .unwrap()
            .iter()
            .find(|p| p[0] == name)
            .unwrap_or_else(|| panic!("no param {name}: {node}"))[1]
            .clone()
    };
    assert_eq!(param("robot"), robot, "{node}");
    assert_eq!(param("shout"), robot.to_uppercase(), "{node}");
}

#[test]
fn xml_performs_the_extensions_substitutions() {
    assert_performed(&resolve("parent.launch.xml", &[]), "r1");
}

#[test]
fn yaml_performs_the_extensions_substitutions() {
    assert_performed(&resolve("parent.launch.yaml", &[]), "r1");
}

/// The extension performs in the caller's context: an argument given on the
/// command line reaches both its performed arguments and
/// `context.launch_configurations`.
#[test]
fn the_extension_reads_the_callers_context() {
    assert_performed(&resolve("parent.launch.xml", &[("robot", "r2")]), "r2");
}

/// `launch`'s own refusal, with the extension that could not be loaded named:
/// it may be the one that would have exposed the substitution.
#[test]
fn a_substitution_no_extension_exposes_is_refused() {
    extensions_importable();
    let _guard = python_test_guard();
    let path = fixture("unknown.launch.xml");
    let e = match parse_launch_file(&path, HashMap::new()) {
        Ok(_) => panic!("{} resolved, but launch refuses it", path.display()),
        Err(e) => format!("{e:#}"),
    };
    assert!(e.contains("Unknown substitution type: fixture-nope"), "{e}");
    assert!(
        e.contains("broken_ext"),
        "names the extension that failed: {e}"
    );
}
