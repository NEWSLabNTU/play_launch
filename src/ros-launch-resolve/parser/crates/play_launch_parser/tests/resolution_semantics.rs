//! What a launch file RESOLVES to — substitutions, parameter values, command
//! lines, setters, groups — measured against stock `launch` and asserted here.
//!
//! Every expectation is what stock `launch` (Humble, launch 1.0.14,
//! launch_ros 0.19.14) resolves for the fixture under
//! `tests/fixtures/resolution_semantics/<case>/`, read off a real
//! `LaunchService` run in which `ExecuteLocal.execute` and
//! `LoadComposableNodes.execute` record the command line and the LoadNode
//! request instead of starting anything (see `execution_semantics.rs`).
//!
//! A record carries a parameter as TEXT that the model and the spawner
//! re-type by shape, so a value stock passes as a string but which would
//! re-type as something else is written YAML single-quoted (`'1.0'`). The
//! comments give the value stock passes to the node.

use play_launch_parser::parse_launch_file;
use play_launch_parser_pyexec::python_test_guard;
use std::{collections::HashMap, path::PathBuf};

fn fixture(case: &str, file: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/resolution_semantics")
        .join(case)
        .join(file)
}

fn resolve_file(case: &str, file: &str) -> serde_json::Value {
    let path = fixture(case, file);
    let record = parse_launch_file(&path, HashMap::new())
        .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    serde_json::to_value(record).unwrap()
}

fn resolve(case: &str) -> serde_json::Value {
    resolve_file(case, "parent.launch.xml")
}

fn error(case: &str, file: &str) -> String {
    let path = fixture(case, file);
    match parse_launch_file(&path, HashMap::new()) {
        Ok(_) => panic!("{}: stock refuses this file; it resolved", path.display()),
        Err(e) => e.to_string(),
    }
}

fn node<'a>(json: &'a serde_json::Value, name: &str) -> &'a serde_json::Value {
    json["node"]
        .as_array()
        .unwrap()
        .iter()
        .find(|n| n["name"] == name)
        .unwrap_or_else(|| panic!("no node {name} in {json:#}"))
}

fn names(json: &serde_json::Value) -> Vec<String> {
    let mut out: Vec<String> = json["node"]
        .as_array()
        .unwrap()
        .iter()
        .map(|n| n["name"].as_str().unwrap().to_string())
        .collect();
    out.sort();
    out
}

fn pairs(value: &serde_json::Value) -> Vec<(String, String)> {
    value
        .as_array()
        .map(|a| {
            a.iter()
                .map(|p| {
                    (
                        p[0].as_str().unwrap().to_string(),
                        p[1].as_str().unwrap().to_string(),
                    )
                })
                .collect()
        })
        .unwrap_or_default()
}

fn strings(value: &serde_json::Value) -> Vec<String> {
    value
        .as_array()
        .map(|a| a.iter().map(|s| s.as_str().unwrap().to_string()).collect())
        .unwrap_or_default()
}

fn own<'a>(list: &[(&'a str, &'a str)]) -> Vec<(String, String)> {
    list.iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect()
}

/// The substitution grammar is `launch`'s: quoted arguments lose their
/// quotes, `\$` escapes, `var` takes a default, `eval` yields Python's
/// `str()` (`True`), `command` runs without a shell, `anon` is cached per
/// name. Before: `find-pkg-prefix`/`find-exec` were unknown, `\$` kept the
/// backslash, and `anon` used its own format.
#[test]
fn substitutions_resolve_as_launch_performs_them() {
    let _guard = python_test_guard();
    let json = resolve("substitutions");
    assert_eq!(
        pairs(&node(&json, "subs")["params"]),
        own(&[
            ("p_nested", "nested_ok"),
            ("p_var_default", "dflt"),
            ("p_env_default", "fallback"),
            ("p_eval", "6"),
            // stock: bool True
            ("p_eval_bool", "True"),
            // stock: "hello world" — the trailing newline is YAML-folded away
            ("p_command", "hello world"),
            ("p_command_nested", "inner"),
            ("p_mixed", "pre-3-post"),
            ("p_escape", "dollar $(var num)"),
        ])
    );

    // `$(anon twin)` twice is ONE name: `AnonName` caches per name.
    let anon: Vec<String> = names(&json)
        .into_iter()
        .filter(|n| n.starts_with("twin_"))
        .collect();
    assert_eq!(anon.len(), 2, "{anon:?}");
    assert_eq!(anon[0], anon[1]);
    let host = hostname_part();
    assert!(
        anon[0].starts_with(&format!("twin_{host}_{}_", std::process::id())),
        "{} is not twin_<host>_<pid>_<rand>",
        anon[0]
    );
}

fn hostname_part() -> String {
    let host = std::fs::read_to_string("/proc/sys/kernel/hostname").unwrap_or_default();
    host.trim().replace(['.', '-', ':'], "_")
}

/// `<param>` typing is `launch_ros`'s: `type="str"` forces a string,
/// `value-sep` makes a list, a quoted value stays a string, nested `<param>`
/// joins with dots. Before: `type` was ignored (`'1.0'` became a float,
/// `true` became `True`), `value-sep` and nested `<param>` were refused,
/// `'5'` kept its quotes.
#[test]
fn parameter_values_are_typed_as_launch_ros_types_them() {
    let _guard = python_test_guard();
    let json = resolve("params");
    assert_eq!(
        pairs(&node(&json, "params_order")["params"]),
        own(&[
            ("x", "inline_x"),
            // stock: str "1.0"
            ("s_float", "'1.0'"),
            // stock: str "[1, 2]"
            ("s_list", "'[1, 2]'"),
            // stock: str "true"
            ("s_bool", "'true'"),
            ("auto_float", "1.0"),
            ("auto_int", "7"),
            // stock: bool True
            ("auto_bool", "True"),
            ("auto_list", "[1, 2, 3]"),
            ("auto_strlist", "['a', 'b']"),
            // stock: str "5"
            ("auto_str_num", "'5'"),
            // stock: ["a", "b", "c"]
            ("value_sep", "['a', 'b', 'c']"),
            ("nested.child", "nc"),
            // stock: float 2.5
            ("nested.deeper.leaf", "2.5"),
            ("subst", "from_arg"),
        ])
    );
    // `allow_substs` performs the file's substitutions.
    let files = strings(&node(&json, "sub_yaml")["params_files"]);
    assert_eq!(files.len(), 1);
    assert!(files[0].contains("from_var: from_arg"), "{}", files[0]);
}

/// `args`/`cmd` are split the way `ExecuteProcess._parse_cmdline` splits
/// them: `shlex` over the literal text, a substitution's result never split,
/// an empty attribute one empty argument. Before: split on whitespace, quotes
/// kept (`'b`, `c'`), and an `<executable cmd>` was one argv element.
#[test]
fn command_lines_split_as_execute_process_splits_them() {
    let _guard = python_test_guard();
    let json = resolve("arguments");
    let quoted = node(&json, "quoted");
    assert_eq!(strings(&quoted["args"]), ["a", "b c", "d e", "f", "g"]);
    assert_eq!(strings(&quoted["ros_args"]), ["--log-level", "debug"]);
    assert_eq!(strings(&node(&json, "empty_args")["args"]), [""]);
    assert_eq!(
        strings(&node(&json, "ex")["cmd"]),
        ["echo", "one two", "three"]
    );
    assert_eq!(
        strings(&node(&json, "ex_subst")["cmd"]),
        ["/usr/bin/env", "-i"]
    );
}

/// Global parameters, remaps and environment reach every kind of process —
/// containers and the composables loaded into them included — with the
/// scoping `launch` gives them. Before: a container took none of them and a
/// composable no global remap.
#[test]
fn setters_reach_containers_and_composables() {
    let _guard = python_test_guard();
    let json = resolve("setters");
    let global = own(&[
        ("global_top", "1"),
        ("global_in_group", "g"),
        ("global_from_child", "c"),
    ]);
    let remaps = own(&[
        ("top_from", "top_to"),
        ("grp_from", "grp_to"),
        ("child_from", "child_to"),
    ]);

    let container = &json["container"][0];
    assert_eq!(pairs(&container["global_params"]), global);
    assert_eq!(pairs(&container["remaps"]), remaps);
    let mut env = pairs(&container["env"]);
    env.sort();
    assert_eq!(env, own(&[("PL_CHILD", "child"), ("PL_TOP", "top")]));

    let load = &json["load_node"][0];
    let mut load_params = global.clone();
    load_params.push(("cp".into(), "1".into()));
    assert_eq!(pairs(&load["params"]), load_params);
    let mut load_remaps = remaps.clone();
    load_remaps.push(("cfrom".into(), "cto".into()));
    assert_eq!(pairs(&load["remaps"]), load_remaps);

    // The environment is scoped by the group; the lists are not, because a
    // group pushes the list a configuration holds, not a copy of it.
    let after = node(&json, "after_group");
    assert_eq!(pairs(&after["env"]), own(&[("PL_TOP", "top")]));
    assert_eq!(
        pairs(&after["remaps"]),
        own(&[("top_from", "top_to"), ("grp_from", "grp_to")])
    );
}

/// `<group forwarding="false">` starts its body from nothing but `<keep>`:
/// no configurations, no namespace, no environment. Before: `forwarding`
/// was ignored and `<keep>` refused.
#[test]
fn a_group_without_forwarding_keeps_only_what_it_names() {
    let _guard = python_test_guard();
    for (case, file) in [
        ("group_forwarding", "parent.launch.xml"),
        ("group_forwarding_yaml", "parent.launch.yaml"),
    ] {
        let json = resolve_file(case, file);
        let off = node(&json, "fwd_off");
        assert_eq!(
            pairs(&off["params"]),
            own(&[("kept", "o_k"), ("drop", "missing"), ("envv", "none")]),
            "{case}"
        );
        assert!(pairs(&off["env"]).is_empty(), "{case}");
        assert_eq!(
            pairs(&node(&json, "after")["params"]).last().unwrap(),
            &("drop".to_string(), "d".to_string()),
            "{case}: the group pops what it reset"
        );
    }

    let json = resolve("group_forwarding");
    assert_eq!(node(&json, "fwd_off")["namespace"], serde_json::Value::Null);
    let on = node(&json, "fwd_on");
    assert_eq!(on["namespace"], "/pushed");
    assert_eq!(pairs(&on["params"]), own(&[("fk", "d"), ("drop", "d")]));
    // An unscoped group's `<keep>` reaches the siblings.
    assert_eq!(
        pairs(&node(&json, "after")["params"])[0],
        ("leak".into(), "L".into())
    );
}

/// The YAML frontend types natively: `true` is a bool, `[1, 2, 3]` a list,
/// a nested `param:` joins with dots, `env:` sets the environment. Before:
/// lists and nested params were dropped, `env:` was ignored and `cmd` was
/// one argv element.
#[test]
fn the_yaml_frontend_types_values_natively() {
    let _guard = python_test_guard();
    let json = resolve_file("yaml_frontend", "parent.launch.yaml");
    let typed = node(&json, "y_typed");
    assert_eq!(
        pairs(&typed["params"]),
        own(&[
            ("int_p", "5"),
            ("float_p", "2.5"),
            ("bool_p", "True"),
            ("str_p", "hello"),
            ("list_p", "[1, 2, 3]"),
            ("strlist_p", "['a', 'b']"),
            ("subst_p", "let_4"),
            ("subst_num", "4"),
            ("grp.inner", "1"),
        ])
    );
    assert_eq!(pairs(&typed["env"]), own(&[("PL_Y", "yv")]));
    assert_eq!(strings(&typed["args"]), ["x", "y"]);
    assert_eq!(
        strings(&node(&json, "yex")["cmd"]),
        ["echo", "yaml", "exec"]
    );
    assert_eq!(
        names(&json),
        ["y_leaked_kept", "y_typed", "yc_from_yaml", "yex"]
    );
}

/// `DeclareLaunchArgument` performs its default when it executes, a
/// re-declaration keeps the first value, `choices` is checked; `<let>`
/// stores the performed value, which is never parsed again.
#[test]
fn arguments_and_lets_store_performed_values() {
    let _guard = python_test_guard();
    let json = resolve("arguments_and_lets");
    assert_eq!(
        pairs(&node(&json, "n")["params"]),
        own(&[
            ("eager", "first"),
            ("lit", "a$(var num)"),
            ("re", "r1"),
            ("mode", "b"),
        ])
    );
    let e = error("conditions", "bad_choice.launch.xml");
    assert!(e.contains("'c' is not valid"), "{e}");
}

/// A condition is `true`/`1`/`false`/`0` in any case and nothing else, and
/// `if` with `unless` is an error. Before: `yes`/`on`/`enabled` were true,
/// anything else — a typo included — silently false.
#[test]
fn conditions_accept_exactly_what_launch_accepts() {
    let _guard = python_test_guard();
    let json = resolve("conditions");
    assert_eq!(names(&json), ["one", "unless_false", "upper_true"]);

    let e = error("conditions", "yes.launch.xml");
    assert!(e.contains("but got 'yes'"), "{e}");
    let e = error("conditions", "if_and_unless.launch.xml");
    assert!(e.contains("can't be used simultaneously"), "{e}");
}
