//! Issue 0034 — `$(dirname)` for a launch file named without a directory.
//!
//! The oracle is `launch` itself:
//! `IncludeLaunchDescription._get_launch_file_directory()` takes
//! `os.path.abspath(location)` FIRST and only then `os.path.dirname()`, so
//! `$(dirname)` is an ABSOLUTE path for every invocation form — a bare
//! filename, `./f.launch.xml`, or an absolute path. This parser used to store
//! the path exactly as typed and call `Path::parent()` on it, which is
//! `Some("")` for a bare filename and `Some(".")` for the `./` form; the first
//! turned `$(dirname)/../includes/x.yaml` into `/includes/x.yaml` and failed
//! the parse, the second left the value relative to the cwd.
//!
//! These tests live in their own test binary on purpose: the reproduction
//! needs the process cwd to BE the fixture directory (a test that passes an
//! absolute path passes against the defect too), and `set_current_dir` is
//! process-global. Every test here takes `CWD_LOCK` before touching it, and
//! nothing else shares this binary.

use play_launch_parser::{parse_launch_file, substitution::LaunchContext};
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::{Mutex, MutexGuard, OnceLock},
};

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/launch")
}

fn cwd_lock() -> MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    // A panicking test poisons the mutex; the guard below still restores the
    // cwd, so the lock is usable afterwards and a poisoned one must not turn
    // one failure into several.
    LOCK.get_or_init(|| Mutex::new(()))
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Restores the working directory on drop, panic included.
struct CwdGuard {
    previous: PathBuf,
    _lock: MutexGuard<'static, ()>,
}

impl CwdGuard {
    fn enter(dir: &Path) -> Self {
        let lock = cwd_lock();
        let previous = std::env::current_dir().expect("cwd readable");
        std::env::set_current_dir(dir).expect("fixture dir enterable");
        Self {
            previous,
            _lock: lock,
        }
    }
}

impl Drop for CwdGuard {
    fn drop(&mut self) {
        let _ = std::env::set_current_dir(&self.previous);
    }
}

/// Parse `name` with the process cwd set to the fixture directory, and return
/// the `launch_dir` parameter of the single node — the resolved `$(dirname)`.
fn resolved_dirname(name: &str) -> String {
    let _cwd = CwdGuard::enter(&fixture_dir());

    let record = parse_launch_file(Path::new(name), HashMap::new())
        .unwrap_or_else(|e| panic!("parsing '{name}' from its own directory should succeed: {e}"));
    let json = serde_json::to_value(record).unwrap();

    let nodes = json["node"].as_array().expect("node list");
    let node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("dirname_probe"))
        .expect("dirname_probe node");
    let params = node["params"].as_array().expect("params");
    params
        .iter()
        .find(|p| p[0].as_str() == Some("launch_dir"))
        .expect("launch_dir param")[1]
        .as_str()
        .expect("launch_dir value")
        .to_string()
}

fn assert_is_fixture_dir(value: &str, form: &str) {
    let path = Path::new(value);
    assert!(
        path.is_absolute(),
        "$(dirname) must be absolute for the {form} form, got {value:?}"
    );
    assert_eq!(
        std::fs::canonicalize(path).unwrap_or_else(|e| panic!("{value}: {e}")),
        std::fs::canonicalize(fixture_dir()).unwrap(),
        "$(dirname) must be the launch file's own directory for the {form} form"
    );
}

#[test]
fn dirname_is_the_absolute_fixture_dir_for_a_bare_xml_filename() {
    // The reported reproduction: `play_launch check safety_island.launch.xml`
    // from inside the launch directory. Before the fix this panicked in the
    // parse, on `<param from="/../includes/test_dirname_params.yaml">`.
    assert_is_fixture_dir(
        &resolved_dirname("test_dirname_bare_invocation.launch.xml"),
        "bare filename",
    );
}

#[test]
fn dirname_is_the_absolute_fixture_dir_for_a_dot_slash_xml_path() {
    // The other half of the divergence: this form parsed fine before the fix
    // but resolved `$(dirname)` to ".", where ROS 2 gives an absolute path.
    assert_is_fixture_dir(
        &resolved_dirname("./test_dirname_bare_invocation.launch.xml"),
        "./relative",
    );
}

#[test]
fn dirname_is_the_absolute_fixture_dir_for_an_absolute_xml_path() {
    // The control: the form that always worked must not move.
    let absolute = fixture_dir().join("test_dirname_bare_invocation.launch.xml");
    assert_is_fixture_dir(
        &resolved_dirname(absolute.to_str().unwrap()),
        "absolute path",
    );
}

#[test]
fn dirname_is_the_absolute_fixture_dir_for_a_bare_yaml_filename() {
    // `traverse_file`'s YAML branch returns before the XML branch sets the
    // current file, so this frontend needs its own coverage.
    assert_is_fixture_dir(
        &resolved_dirname("test_dirname_bare_invocation.launch.yaml"),
        "bare YAML filename",
    );
}

#[test]
fn dirname_is_the_absolute_fixture_dir_for_a_dot_slash_yaml_path() {
    assert_is_fixture_dir(
        &resolved_dirname("./test_dirname_bare_invocation.launch.yaml"),
        "./relative YAML",
    );
}

#[test]
fn dirname_from_a_root_python_launch_file_is_its_own_directory() {
    // `ThisLaunchFileDir()` is captured as `$(dirname)` and resolved by the
    // host, which never set a current file for the Python path: as a root file
    // this failed with "Failed to resolve Python include path", and as an
    // included one it silently meant the INCLUDING file's directory. Nothing
    // else runs in this test binary, and every test here holds the same lock,
    // so the interpreter is not entered concurrently.
    play_launch_parser_pyexec::register();
    assert_is_fixture_dir(
        &resolved_dirname("test_dirname_python_include.launch.py"),
        "bare Python filename",
    );
}

#[test]
fn a_relative_current_file_still_yields_an_absolute_current_dir() {
    // The unit-level statement of the same rule, for callers that seed the
    // context directly: `Path::new("f.launch.xml").parent()` is `Some("")`,
    // which is what reached `$(dirname)`.
    let _cwd = CwdGuard::enter(&fixture_dir());
    let cwd = std::env::current_dir().unwrap();

    for form in [
        "test_dirname_bare_invocation.launch.xml",
        "./test_dirname_bare_invocation.launch.xml",
    ] {
        let mut context = LaunchContext::new();
        context.set_current_file(PathBuf::from(form));
        let dir = context.current_dir().expect("current_dir");
        assert!(dir.is_absolute(), "{form}: current_dir was {dir:?}");
        assert_eq!(dir, cwd, "{form}: current_dir must be the cwd");
        assert_eq!(
            context.current_filename().as_deref(),
            Some("test_dirname_bare_invocation.launch.xml"),
            "{form}: absolutizing must not disturb $(filename)"
        );
    }
}

/// Parse `name` from the fixture directory and return its whole node list.
fn parsed_nodes(name: &str) -> serde_json::Value {
    let _cwd = CwdGuard::enter(&fixture_dir());
    let record = parse_launch_file(Path::new(name), HashMap::new())
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

/// Issue 0034's residual: `$(dirname)` in a node's OWN fields inside a
/// `.launch.py`. The include path was the only captured string anything
/// resolved, so a parameter, a parameter FILE, an argument and a remapping all
/// carried the literal into the record and into `cmd` — the spawned command
/// line — and the parameter file was never read.
#[test]
fn dirname_resolves_in_every_field_of_a_python_node() {
    play_launch_parser_pyexec::register();
    let nodes = parsed_nodes("test_dirname_python_params.launch.py");
    let probe = node(&nodes, "dirname_param_probe");

    // Case 1: a substitution as a parameter VALUE.
    assert_is_fixture_dir(&param(probe, "launch_dir"), "Python parameter value");
    assert_eq!(
        param(probe, "config_path"),
        fixture_dir().join("sub/thing.txt").display().to_string(),
        "a PathJoinSubstitution parameter value must be resolved"
    );

    // Case 2: a substitution naming a parameter FILE. `to_record` stores the
    // file's CONTENT, so an unresolved path is not merely ugly — the file is
    // silently not read and the path is stored in its place.
    let params_files = probe["params_files"].as_array().expect("params_files");
    assert_eq!(params_files.len(), 1);
    let content = params_files[0].as_str().unwrap();
    assert!(
        content.contains("dirname_probe_loaded"),
        "the parameter file must have been READ, got {content:?}"
    );

    // Case 3: arguments and remappings.
    let args = probe["args"].as_array().expect("args");
    assert_eq!(
        args[0].as_str().unwrap(),
        fixture_dir().join("arg.txt").display().to_string(),
    );
    let remap = probe["remaps"].as_array().expect("remaps")[0][1]
        .as_str()
        .unwrap();
    assert_eq!(remap, fixture_dir().join("topic").display().to_string());

    // Nothing may reach the command line — what actually gets spawned — with
    // the token still in it.
    for word in probe["cmd"].as_array().expect("cmd") {
        let word = word.as_str().unwrap();
        assert!(
            !word.contains("$(dirname)"),
            "unresolved $(dirname) reached the command line: {word:?}"
        );
    }
}

/// The deliberate non-change: `$(var ...)` is preserved for replay-time
/// resolution (`execution/node_cmdline.rs`), and resolving `$(dirname)` must
/// not touch it.
#[test]
fn a_launch_configuration_parameter_is_still_preserved() {
    play_launch_parser_pyexec::register();
    let nodes = parsed_nodes("test_dirname_python_params.launch.py");
    assert_eq!(
        param(node(&nodes, "dirname_param_probe"), "replay_var"),
        "$(var an_unset_argument)",
    );
}

/// `$(dirname)` belongs to the file that DECLARED the node, not to the root
/// launch file — which is why the captures are resolved per execution rather
/// than at record-conversion time, where the context has been restored to the
/// root.
#[test]
fn an_included_python_file_resolves_dirname_to_its_own_directory() {
    play_launch_parser_pyexec::register();
    let nodes = parsed_nodes("test_dirname_python_params.launch.py");
    let included = param(node(&nodes, "dirname_included_probe"), "launch_dir");
    assert_eq!(
        std::fs::canonicalize(&included).unwrap_or_else(|e| panic!("{included}: {e}")),
        std::fs::canonicalize(fixture_dir().parent().unwrap().join("includes")).unwrap(),
        "an included .launch.py must resolve $(dirname) to ITS directory"
    );
}
