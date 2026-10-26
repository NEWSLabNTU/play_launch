//! Phase 85 "cheap first": what `play_launch` says about itself and how a CI
//! script or a log reads its verdict (T4-T8 of
//! `docs/roadmap/phase-85-what-the-island-left-open.md`).
//!
//! Every test drives the installed `play_launch` binary as a subprocess, so
//! the version, exit codes and bytes asserted here are the ones a user's
//! shell sees.

use play_launch_tests::fixtures;
use std::{
    collections::HashMap,
    process::{Command, Output},
    sync::OnceLock,
};

fn test_env() -> &'static HashMap<String, String> {
    static ENV: OnceLock<HashMap<String, String>> = OnceLock::new();
    ENV.get_or_init(fixtures::install_env)
}

fn play_launch() -> Command {
    fixtures::play_launch_cmd(test_env())
}

fn text(out: &Output) -> String {
    format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    )
}

/// T4 (I2): `--version` is `play_launch X.Y.Z (<describe>, rlm vA.B.C)`.
/// A bare semver let two binaries that both said `0.12.0` disagree on the
/// island contract.
#[test]
fn version_names_the_commit_and_the_rlm_tag() {
    let out = play_launch().arg("--version").output().expect("runs");
    assert!(out.status.success());
    let v = String::from_utf8_lossy(&out.stdout).trim().to_string();
    let rest = v
        .strip_prefix("play_launch ")
        .unwrap_or_else(|| panic!("no `play_launch ` prefix: {v}"));
    let (semver, paren) = rest
        .split_once(" (")
        .unwrap_or_else(|| panic!("no build detail: {v}"));
    assert_eq!(semver.split('.').count(), 3, "{v}");
    let inner = paren
        .strip_suffix(')')
        .unwrap_or_else(|| panic!("unclosed: {v}"));
    let rlm = inner.rsplit(", ").next().unwrap();
    // A release pins a tag; between releases the pin may be a commit, which
    // build.rs prints as its short hash.
    let pin = rlm.strip_prefix("rlm ").unwrap_or("");
    let is_commit = pin.len() == 7 && pin.chars().all(|c| c.is_ascii_hexdigit());
    assert!(pin.starts_with('v') || is_commit, "no rlm pin last: {v}");
    // Built from this checkout: when git can read it here, the describe
    // must be there too. (Without git, build.rs omits it rather than
    // inventing one, and the version is still well-formed.)
    let git_works = Command::new("git")
        .args(["-c", "safe.directory=*", "-C"])
        .arg(fixtures::repo_root())
        .args(["rev-parse", "HEAD"])
        .output()
        .is_ok_and(|o| o.status.success());
    if git_works {
        assert!(inner.contains(", "), "no git describe: {v}");
    }
}

/// Run `play_launch check` on `tests/fixtures/<dir>/launch/bringup.launch.xml`.
fn check(dir: &str, extra: &[&str]) -> Output {
    let launch = fixtures::repo_root()
        .join("tests/fixtures")
        .join(dir)
        .join("launch/bringup.launch.xml");
    play_launch()
        .arg("check")
        .arg(&launch)
        .args(extra)
        .output()
        .expect("play_launch check runs")
}

/// T4 (I3): an unknown key's refusal names this checker and its grammar, so
/// an old binary does not read as the author's typo. The existing message is
/// kept; one line is appended.
#[test]
fn a_refusal_names_the_checker_and_its_rlm_tag() {
    let out = check("contract_unknown_key", &[]);
    let s = text(&out);
    assert!(s.contains("error[manifest-parse]"), "{s}");
    assert!(s.contains("unknown key"), "{s}");
    assert!(s.contains("is now UNCHECKED"), "{s}");
    let version = String::from_utf8_lossy(
        &play_launch()
            .arg("--version")
            .output()
            .expect("runs")
            .stdout,
    )
    .trim()
    .to_string();
    assert!(
        s.contains(&format!(
            "this checker: {version}; the contract's grammar may be newer"
        )),
        "no checker line naming `{version}`:\n{s}"
    );
}

/// `check` on a takeover fixture, with the fixture ament tree on the path
/// (the settle derivation reads the operators' parameter files from it).
fn check_takeover(dir: &str, extra: &[&str]) -> Output {
    let root = fixtures::repo_root().join("tests/fixtures");
    let ament = root.join("contract_takeover/ament");
    let prefix = match test_env().get("AMENT_PREFIX_PATH") {
        Some(p) if !p.is_empty() => format!("{}:{p}", ament.display()),
        _ => ament.display().to_string(),
    };
    play_launch()
        .arg("check")
        .arg(root.join(dir).join("launch/bringup.launch.xml"))
        .args(extra)
        .env("AMENT_PREFIX_PATH", prefix)
        .output()
        .expect("play_launch check runs")
}

/// T5 (I4): a refused contract exits 3, not 1, and the JSON report names the
/// refusal as a `manifest-parse` entry under `<load>`.
#[test]
fn a_refusal_has_its_own_exit_code_and_json_entry() {
    let out = check("contract_unknown_key", &[]);
    assert_eq!(out.status.code(), Some(3), "{}", text(&out));

    let out = check("contract_unknown_key", &["--format", "json"]);
    assert_eq!(out.status.code(), Some(3), "{}", text(&out));
    let stdout = String::from_utf8_lossy(&out.stdout);
    let report: serde_json::Value = serde_json::from_str(stdout.trim())
        .unwrap_or_else(|e| panic!("stdout is not one JSON array ({e}):\n{stdout}"));
    let entries = report.as_array().expect("an array");
    assert!(
        entries.iter().any(|d| d["rule"] == "manifest-parse"
            && d["file"] == "<load>"
            && d["severity"] == "error"),
        "{stdout}"
    );

    // A rule failure is still 1.
    let out = check("contract_error", &[]);
    assert_eq!(out.status.code(), Some(1), "{}", text(&out));
}

/// T5 (I4): `--expect` passes only on exactly the expected errors.
#[test]
fn expect_passes_only_on_exactly_the_expected_rules() {
    // contract_error fails rate-hierarchy and nothing else.
    let out = check("contract_error", &["--expect", "rate-hierarchy"]);
    assert_eq!(out.status.code(), Some(0), "{}", text(&out));
    assert!(text(&out).contains("expect: ok"), "{}", text(&out));

    // The expected rule did not fire.
    let out = check("contract_error", &["--expect", "ladder-rung-budget"]);
    assert_eq!(out.status.code(), Some(1), "{}", text(&out));
    let s = text(&out);
    assert!(
        s.contains("expected error[ladder-rung-budget] did not fire"),
        "{s}"
    );
    assert!(s.contains("unexpected error[rate-hierarchy]"), "{s}");

    // It fired, alongside rules that were not expected.
    let out = check_takeover(
        "contract_takeover_budget",
        &["--expect", "ladder-rung-budget"],
    );
    assert_eq!(out.status.code(), Some(1), "{}", text(&out));
    assert!(
        text(&out).contains("unexpected error[fault-reaction-budget]"),
        "{}",
        text(&out)
    );

    // Every rule it fails, expected: a pass.
    let out = check_takeover(
        "contract_takeover_budget",
        &[
            "--expect",
            "ladder-rung-budget",
            "--expect",
            "fault-reaction-budget",
            "--expect",
            "settle-param-unresolved",
            "--expect",
            "window-param",
        ],
    );
    assert_eq!(out.status.code(), Some(0), "{}", text(&out));

    // A refusal never passes an expectation, and keeps its own code.
    let out = check("contract_unknown_key", &["--expect", "rate-hierarchy"]);
    assert_eq!(out.status.code(), Some(3), "{}", text(&out));
    assert!(
        text(&out).contains("expect: FAILED -- 1 contract file(s) were refused"),
        "{}",
        text(&out)
    );
}

/// T6 (I5): through a pipe, `check --explain` is plain ASCII with no colour,
/// and `--width` bounds every line, the log lines included.
#[test]
fn piped_output_is_ascii_without_colour_and_width_bounds_it() {
    let out = check_takeover("contract_takeover", &["--explain"]);
    let all = [out.stdout.as_slice(), out.stderr.as_slice()].concat();
    assert!(
        String::from_utf8_lossy(&out.stderr).contains("Fault-reaction budgets"),
        "{}",
        text(&out)
    );
    assert!(
        all.iter().all(|b| *b < 0x80),
        "a non-ASCII byte:\n{}",
        text(&out)
    );
    assert!(!all.contains(&0x1b), "an ESC byte:\n{}", text(&out));
    // The route arrow survives as `->`.
    assert!(text(&out).contains(" -> "), "{}", text(&out));

    let out = check_takeover("contract_takeover", &["--explain", "--width", "100"]);
    let s = text(&out);
    assert!(
        s.lines().all(|l| l.chars().count() <= 100),
        "a line over 100:\n{s}"
    );
}

/// T6 (I5): on a terminal, colour is on unless NO_COLOR is set. Uses
/// util-linux `script` for the pty.
#[test]
fn no_color_turns_colour_off_on_a_terminal() {
    if which::which("script").is_err() {
        eprintln!("SKIP: no_color_turns_colour_off_on_a_terminal: `script` not installed");
        return;
    }
    let launch =
        fixtures::repo_root().join("tests/fixtures/contract_error/launch/bringup.launch.xml");
    let cmdline = format!(
        "{} check {}",
        fixtures::play_launch_bin().display(),
        launch.display()
    );
    let run = |no_color: bool| -> Vec<u8> {
        let mut cmd = Command::new("script");
        fixtures::apply_test_env(&mut cmd, test_env());
        cmd.env("TERM", "xterm");
        if no_color {
            cmd.env("NO_COLOR", "1");
        }
        cmd.args(["-qec", &cmdline, "/dev/null"])
            .output()
            .expect("script runs")
            .stdout
    };
    assert!(
        run(false).contains(&0x1b),
        "a terminal without NO_COLOR is coloured"
    );
    let plain = run(true);
    assert!(
        !plain.contains(&0x1b),
        "{}",
        String::from_utf8_lossy(&plain)
    );
    assert!(String::from_utf8_lossy(&plain).contains("error[rate-hierarchy]"));
}

/// T7 (I7): every `max_transport` declaration, subscriber- or topic-level,
/// that no reaction route charges gets one `declared-not-charged` info
/// naming the key. This fixture has no hazard, so neither is on a guard
/// edge; T1 (`takeover_link_is_charged_on_the_guard_edge`) is the charged
/// case.
#[test]
fn a_transport_bound_says_it_is_not_charged() {
    let out = check("contract_transport_notice", &[]);
    let s = text(&out);
    assert_eq!(out.status.code(), Some(0), "{s}");
    assert_eq!(s.matches("info[declared-not-charged]").count(), 2, "{s}");
    assert!(
        s.contains("`nodes.control_node.sub.scan.max_transport: 57ms` on '/dx/control_node/scan'"),
        "{s}"
    );
    assert!(
        s.contains("`topics./dx/cmd.max_transport: 3ms` on topic '/dx/cmd'"),
        "{s}"
    );
    assert!(
        s.contains("is not charged by the fault-reaction arithmetic: no hazard's reaction route enters through it"),
        "{s}"
    );
}

/// T7 (I8): an endpoint under `sub:`/`pub:` that no topic wires is a
/// warning naming the node and the key; a wired one and a path's own
/// endpoint (the `wiring` rule's) are not.
#[test]
fn an_unwired_endpoint_is_a_warning() {
    let out = check("contract_unwired", &[]);
    let s = text(&out);
    assert_eq!(out.status.code(), Some(0), "a warning, not an error:\n{s}");
    assert_eq!(s.matches("warning[endpoint-unwired]").count(), 1, "{s}");
    assert!(
        s.contains("node '/dx/control_node' declares `sub: odom` ('/dx/control_node/odom')"),
        "{s}"
    );
}

/// T4 (D5): a contract whose `rlm:` header names a grammar newer than the
/// checker's is refused BEFORE its body is read (so not for the key a newer
/// grammar might know), naming both releases and this checker, with the
/// refusal exit code. The refusal names the grammar the binary was built
/// with (`GRAMMAR_VERSION`); `--version` names the rlm tag. They differ
/// after a release that changes only the model (rlm v0.1.63 kept grammar
/// 0.1.62), so each is compared with its own source.
#[test]
fn a_contract_needing_a_newer_grammar_names_both_releases() {
    let out = check("contract_grammar_newer", &[]);
    let s = text(&out);
    assert_eq!(out.status.code(), Some(3), "{s}");
    let version = String::from_utf8_lossy(
        &play_launch()
            .arg("--version")
            .output()
            .expect("runs")
            .stdout,
    )
    .trim()
    .to_string();
    let tag = version
        .rsplit("rlm ")
        .next()
        .and_then(|t| t.strip_suffix(')'))
        .unwrap_or_else(|| panic!("no rlm tag: {version}"));
    assert!(
        tag.starts_with('v'),
        "--version names an rlm tag: {version}"
    );
    let grammar = ros_launch_manifest_types::GRAMMAR_VERSION;
    assert!(
        s.contains(&format!(
            "this contract needs rlm >= v99.0.0; this checker reads rlm v{grammar}"
        )),
        "{s}"
    );
    assert!(s.contains("this checker: play_launch"), "{s}");
    assert!(!s.contains("not_a_key_yet"), "{s}");
}

/// T7 (I6): `--export-graph` carries what the fault walk reads, so a figure
/// can be drawn from the export alone (the deck's `contract_graph.py`
/// re-read the contract YAML for services, externals, hazards, modes and
/// path triggers): on the island-shaped fixture, the operate service edges,
/// the external publisher, both hazards and the ladder, the detector, and a
/// timer trigger with its rate.
#[test]
fn the_graph_export_carries_what_the_walk_reads() {
    let dir = tempfile::tempdir().expect("tempdir");
    let json = dir.path().join("graph.json");
    let out = check(
        "contract_takeover_link",
        &["--export-graph", json.to_str().unwrap()],
    );
    assert_eq!(out.status.code(), Some(0), "{}", text(&out));
    let g: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&json).expect("export written"))
            .expect("export is JSON");
    assert_eq!(g["version"], 2);
    let find = |list: &str, key: &str, value: &str| -> serde_json::Value {
        g[list]
            .as_array()
            .unwrap_or_else(|| panic!("no `{list}`: {g}"))
            .iter()
            .find(|e| e[key] == value)
            .unwrap_or_else(|| panic!("no {list} with {key} = {value}: {g}"))
            .clone()
    };
    let svc = find("services", "fqn", "/comfort/operate");
    assert_eq!(svc["servers"][0]["node"], "/comfort_op");
    assert_eq!(svc["clients"][0]["endpoint"], "comfort_operate");
    assert_eq!(find("topics", "fqn", "/availability")["external"], "pub");
    assert_eq!(find("topics", "fqn", "/estop_cmd")["external"], "sub");
    let hpc = find("hazards", "name", "hpc_loss");
    assert_eq!(hpc["guards"][0]["members"][0], "/availability");
    assert_eq!(hpc["on"][0], "omission");
    assert_eq!(hpc["reaction"], "engaged");
    assert_eq!(find("hazards", "name", "odd_exit")["on"][0], "reported");
    let engaged = find("modes", "name", "engaged");
    assert_eq!(
        engaged["fallback"],
        serde_json::json!(["takeover", "comfortable", "estop"])
    );
    assert_eq!(find("modes", "name", "takeover")["window_ms"], 10000.0);
    let detector = g["sub_edges"]
        .as_array()
        .unwrap()
        .iter()
        .find(|e| e["node"] == "/handler" && e["endpoint"] == "availability")
        .expect("the detector's edge")
        .clone();
    assert_eq!(detector["on_violation"]["reaction"], "call_mrm");
    assert_eq!(detector["max_transport_ms"], 57.0);
    let paths = g["node_paths"].as_array().unwrap();
    let path = |node: &str, name: &str| {
        paths
            .iter()
            .find(|p| p["node"] == node && p["path_name"] == name)
            .unwrap_or_else(|| panic!("no path {node}/{name}"))
            .clone()
    };
    assert_eq!(
        path("/estop_op", "on_timer")["trigger"]["timer"]["rate_hz"],
        30.0
    );
    assert_eq!(path("/estop_op", "on_timer")["safe_state"]["emits"], "cmd");
    let call_mrm = path("/handler", "call_mrm");
    assert_eq!(call_mrm["trigger"]["input"][0], "availability");
    assert_eq!(call_mrm["input"][0], "availability");
    assert_eq!(find("nodes", "fqn", "/handler")["contracted"], true);
}

/// Parse `check --format json`'s stdout: zero or more arrays, concatenated.
fn json_elements(out: &Output) -> Vec<serde_json::Value> {
    let stdout = String::from_utf8_lossy(&out.stdout).into_owned();
    let mut all = Vec::new();
    let stream = serde_json::Deserializer::from_str(&stdout).into_iter::<serde_json::Value>();
    for v in stream {
        let v = v.unwrap_or_else(|e| panic!("stdout is not a JSON stream ({e}):\n{stdout}"));
        all.extend(
            v.as_array()
                .expect("each group is an array")
                .iter()
                .cloned(),
        );
    }
    all
}

/// D8: a warning a contract acknowledges is printed as `ack[<rule>]` with its
/// reason and counted apart; an entry that matches nothing is
/// `warning[ack-stale]`; the exit status is unchanged.
#[test]
fn an_acknowledged_warning_is_counted_apart_and_a_stale_entry_warns() {
    let out = check("contract_ack", &[]);
    let s = text(&out);
    assert_eq!(out.status.code(), Some(0), "{s}");
    assert!(
        s.contains("(0 errors, 2 warnings, 1 acknowledged)"),
        "summary:\n{s}"
    );
    assert_eq!(s.matches("ack[endpoint-unwired]").count(), 1, "{s}");
    assert!(!s.contains("warning[endpoint-unwired]"), "{s}");
    assert!(
        s.contains("at nodes.control_node.sub.odom -- acknowledged[0] in bringup.contract.yaml"),
        "{s}"
    );
    assert!(
        s.contains("reason: odometry is read from a bag in this demo; nothing here publishes it"),
        "{s}"
    );
    // The unacknowledged warning is still one.
    assert!(s.contains("warning[wiring]"), "{s}");
    assert_eq!(s.matches("warning[ack-stale]").count(), 1, "{s}");
    assert!(
        s.contains(
            "acknowledged[1] (`wiring` at `nodes.control_node.paths.retired`) matches no diagnostic"
        ),
        "{s}"
    );
    // A cross-scope diagnostic names its path, which is what an entry names.
    assert!(s.contains("    at acknowledged[1]"), "{s}");

    let out = check("contract_ack", &["--format", "json"]);
    assert_eq!(out.status.code(), Some(0), "{}", text(&out));
    let all = json_elements(&out);
    let acked: Vec<&serde_json::Value> = all.iter().filter(|d| d["acknowledged"] == true).collect();
    assert_eq!(acked.len(), 1, "{all:#?}");
    let a = acked[0];
    assert_eq!(a["file"], "<acknowledged>");
    assert_eq!(a["rule"], "endpoint-unwired");
    assert_eq!(a["severity"], "warning");
    assert_eq!(a["path"], "nodes.control_node.sub.odom");
    assert_eq!(
        a["reason"],
        "odometry is read from a bag in this demo; nothing here publishes it"
    );
    assert_eq!(a["acknowledged_by"]["entry"], 0);
    assert!(
        a["acknowledged_by"]["contract_path"]
            .as_str()
            .unwrap()
            .ends_with("contract_ack/launch/bringup.contract.yaml"),
        "{a}"
    );
    assert_eq!(a["scope"], "<cross-scope>");
    // Every other element says it is not acknowledged.
    assert!(
        all.iter()
            .filter(|d| d["file"] != "<acknowledged>")
            .all(|d| d["acknowledged"] == false),
        "{all:#?}"
    );
    assert!(
        all.iter()
            .any(|d| d["rule"] == "ack-stale" && d["path"] == "acknowledged[1]"),
        "{all:#?}"
    );
}

/// D8 on the model path: `resolve` applies `acknowledged:` too, so what a
/// contract acknowledges reaches `meta.diagnostics` as `acknowledged`, with
/// its reason, and not as a warning (nano-ros keeps the `] warning:` lines).
/// The stale entry is a warning there as in `check`, and the loader's line
/// carries no counts that disagree with the verdict.
#[test]
fn resolve_embeds_an_acknowledged_warning_as_acknowledged() {
    let launch =
        fixtures::repo_root().join("tests/fixtures/contract_ack/launch/bringup.launch.xml");
    let dir = tempfile::TempDir::new().expect("temp dir");
    let model_path = dir.path().join("system_model.yaml");
    let out = play_launch()
        .arg("resolve")
        .arg(&launch)
        .arg("-o")
        .arg(&model_path)
        .output()
        .expect("play_launch resolve runs");
    let s = text(&out);
    assert_eq!(out.status.code(), Some(0), "{s}");
    assert!(s.contains("2 warning(s), 1 acknowledged)"), "{s}");
    let loaded: Vec<&str> = s
        .lines()
        .filter(|l| l.contains("Loaded 1 manifest(s)"))
        .collect();
    assert!(!loaded.is_empty(), "{s}");
    assert!(loaded.iter().all(|l| !l.contains("warnings")), "{s}");
    let yaml = std::fs::read_to_string(&model_path).expect("model written");
    let model = ros_launch_manifest_model::SystemModel::from_yaml_str(&yaml).expect("model parses");
    let diags = &model.meta.diagnostics;
    let acked: Vec<&String> = diags
        .iter()
        .filter(|d| d.contains("[endpoint-unwired] acknowledged:"))
        .collect();
    assert_eq!(acked.len(), 1, "{diags:#?}");
    assert!(
        acked[0].contains(
            "(at nodes.control_node.sub.odom; acknowledged[0] in bringup.contract.yaml: \
             odometry is read from a bag in this demo; nothing here publishes it)"
        ),
        "{}",
        acked[0]
    );
    assert!(
        !diags
            .iter()
            .any(|d| d.contains("[endpoint-unwired] warning:")),
        "{diags:#?}"
    );
    assert!(
        diags.iter().any(|d| d.contains("[wiring] warning:")),
        "{diags:#?}"
    );
    assert!(
        diags.iter().any(|d| d.contains("[ack-stale] warning:")),
        "{diags:#?}"
    );
}

/// D8: an error cannot be acknowledged. The error still prints, the entry is
/// `error[ack-error]`, and `--expect` sees it as an error it did not expect.
/// An info can be acknowledged.
#[test]
fn an_acknowledged_error_is_refused_and_the_error_stands() {
    let out = check("contract_ack_error", &[]);
    let s = text(&out);
    assert_eq!(out.status.code(), Some(1), "{s}");
    assert!(
        s.contains(
            "acknowledged[0] names `rate-hierarchy` at `topics./control/cmd`, which is an ERROR"
        ),
        "{s}"
    );
    assert_eq!(s.matches("error[ack-error]").count(), 1, "{s}");
    // contract_error's four rate-hierarchy errors, all still printed.
    assert_eq!(s.matches("error[rate-hierarchy]").count(), 4, "{s}");
    assert!(s.contains("ack[derivable-min-rate]"), "{s}");
    assert!(s.contains("(5 errors, 3 warnings, 1 acknowledged)"), "{s}");

    let out = check("contract_ack_error", &["--expect", "rate-hierarchy"]);
    let s = text(&out);
    assert_eq!(out.status.code(), Some(1), "{s}");
    assert!(s.contains("unexpected error[ack-error] x1"), "{s}");
}
