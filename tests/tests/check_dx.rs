//! Phase 85 "cheap first": what `play_launch` says about itself and how a CI
//! script or a log reads its verdict (T4-T8 of
//! `docs/roadmap/phase-85-what-the-island-left-open.md`).
//!
//! Every test drives the installed `play_launch` binary as a subprocess, so
//! the version, exit codes and bytes asserted here are the ones a user's
//! shell sees.

use play_launch_tests::fixtures;
use std::collections::HashMap;
use std::process::{Command, Output};
use std::sync::OnceLock;

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
    assert!(rlm.starts_with("rlm v"), "no rlm tag last: {v}");
    // Built from this checkout, so the describe is present too.
    assert!(inner.contains(", "), "no git describe: {v}");
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
        &play_launch().arg("--version").output().expect("runs").stdout,
    )
    .trim()
    .to_string();
    assert!(
        s.contains(&format!("this checker: {version}; the contract's grammar may be newer")),
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
        entries
            .iter()
            .any(|d| d["rule"] == "manifest-parse" && d["file"] == "<load>" && d["severity"] == "error"),
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
    assert!(s.contains("expected error[ladder-rung-budget] did not fire"), "{s}");
    assert!(s.contains("unexpected error[rate-hierarchy]"), "{s}");

    // It fired, alongside rules that were not expected.
    let out = check_takeover("contract_takeover_budget", &["--expect", "ladder-rung-budget"]);
    assert_eq!(out.status.code(), Some(1), "{}", text(&out));
    assert!(text(&out).contains("unexpected error[fault-reaction-budget]"), "{}", text(&out));

    // Every rule it fails, expected: a pass.
    let out = check_takeover(
        "contract_takeover_budget",
        &[
            "--expect", "ladder-rung-budget",
            "--expect", "fault-reaction-budget",
            "--expect", "settle-param-unresolved",
            "--expect", "window-param",
        ],
    );
    assert_eq!(out.status.code(), Some(0), "{}", text(&out));

    // A refusal never passes an expectation, and keeps its own code.
    let out = check("contract_unknown_key", &["--expect", "rate-hierarchy"]);
    assert_eq!(out.status.code(), Some(3), "{}", text(&out));
    assert!(text(&out).contains("expect: FAILED -- 1 contract file(s) were refused"), "{}", text(&out));
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
    assert!(all.iter().all(|b| *b < 0x80), "a non-ASCII byte:\n{}", text(&out));
    assert!(!all.contains(&0x1b), "an ESC byte:\n{}", text(&out));
    // The route arrow survives as `->`.
    assert!(text(&out).contains(" -> "), "{}", text(&out));

    let out = check_takeover("contract_takeover", &["--explain", "--width", "100"]);
    let s = text(&out);
    assert!(s.lines().all(|l| l.chars().count() <= 100), "a line over 100:\n{s}");
}

/// T6 (I5): on a terminal, colour is on unless NO_COLOR is set. Uses
/// util-linux `script` for the pty.
#[test]
fn no_color_turns_colour_off_on_a_terminal() {
    if which::which("script").is_err() {
        eprintln!("SKIP: no_color_turns_colour_off_on_a_terminal: `script` not installed");
        return;
    }
    let launch = fixtures::repo_root().join("tests/fixtures/contract_error/launch/bringup.launch.xml");
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
    assert!(run(false).contains(&0x1b), "a terminal without NO_COLOR is coloured");
    let plain = run(true);
    assert!(!plain.contains(&0x1b), "{}", String::from_utf8_lossy(&plain));
    assert!(String::from_utf8_lossy(&plain).contains("error[rate-hierarchy]"));
}

/// T7 (I7): every `max_transport` declaration, subscriber- or topic-level,
/// gets one `declared-not-charged` info naming the key, until phase 85 I1
/// charges it in the fault arithmetic.
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
    assert!(s.contains("`topics./dx/cmd.max_transport: 3ms` on topic '/dx/cmd'"), "{s}");
    assert!(s.contains("is not charged by the fault-reaction arithmetic yet"), "{s}");
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
