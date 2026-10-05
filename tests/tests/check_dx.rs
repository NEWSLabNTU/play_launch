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
