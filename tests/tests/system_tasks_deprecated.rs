//! Phase 99 T2: a `system.toml` written with the pre-rename keys
//! (`[tiers.*]`, `group_tiers`) still resolves this release, with one
//! `deprecated-spelling` warning per key naming the new one and `migrate`;
//! after `play_launch migrate --write` it resolves to the same execution
//! layer with no warning. The model's key stays `execution.tiers` (schema 4).

use play_launch_tests::fixtures;

/// Resolve the multihost fixture (every host) against `system`; returns the
/// model and the resolver's stderr.
fn resolve(system: &std::path::Path, out: &std::path::Path) -> (serde_json::Value, String) {
    let env = fixtures::install_env();
    let launch = fixtures::repo_root().join("tests/fixtures/multihost/launch/multihost.launch.xml");
    let mut cmd = fixtures::ros_launch_resolve_cmd(&env);
    cmd.args([
        "resolve",
        "--parser",
        "rust",
        launch.to_str().unwrap(),
        "host:=all",
        "--system",
        system.to_str().unwrap(),
        "-o",
        out.to_str().unwrap(),
    ]);
    let output = cmd.output().expect("run ros-launch-resolve resolve");
    let stderr = String::from_utf8_lossy(&output.stderr).to_string();
    assert!(
        output.status.success(),
        "resolve --system failed:\n{stderr}"
    );
    let yaml = std::fs::read_to_string(out).expect("read model");
    (
        serde_yaml_ng::from_str(&yaml).expect("parse model yaml"),
        stderr,
    )
}

#[test]
fn old_task_keys_warn_and_migrate_to_the_same_model() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let system = tmp.path().join("system.toml");
    std::fs::copy(
        fixtures::repo_root().join("tests/fixtures/system_tasks_deprecated/system.toml"),
        &system,
    )
    .expect("copy fixture");

    let (old, old_err) = resolve(&system, &tmp.path().join("old.yaml"));
    let warnings: Vec<&str> = old_err
        .lines()
        .filter(|l| l.contains("warning[deprecated-spelling]"))
        .collect();
    assert_eq!(warnings.len(), 2, "one warning per old key:\n{old_err}");
    assert!(
        warnings
            .iter()
            .any(|w| w.contains("`[tiers.<name>]` is now `[tasks.<name>]` (ctrl, telem)")),
        "{old_err}"
    );
    assert!(
        warnings.iter().any(|w| w.contains(
            "`[[component]] group_tiers` is now `[[component]] group_tasks` (hub, talker1)"
        )),
        "{old_err}"
    );
    assert!(warnings.iter().all(|w| w.contains("migrate")), "{old_err}");
    assert_eq!(
        old["execution"]["tiers"]["ctrl"]["posix"]["priority"], 40,
        "{old}"
    );
    assert_eq!(
        old["execution"]["bindings"]["/hub/hub/main"], "ctrl",
        "{old}"
    );
    assert_eq!(
        old["execution"]["bindings"]["/robot1/talker1/main"], "telem",
        "{old}"
    );

    let out = fixtures::play_launch_cmd(&env)
        .args(["migrate", "--write"])
        .arg(&system)
        .output()
        .expect("play_launch migrate runs");
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert_eq!(out.status.code(), Some(0), "{stdout}");
    assert!(stdout.contains("1 rewritten (6 edit(s))"), "{stdout}");
    let migrated = std::fs::read_to_string(&system).unwrap();
    // Comments keep their words; no key is left in the old spelling.
    let keys: Vec<&str> = migrated
        .lines()
        .map(|l| l.split('#').next().unwrap_or("").trim())
        .filter(|l| l.starts_with("[tiers.") || l.starts_with("group_tiers"))
        .collect();
    assert!(keys.is_empty(), "{migrated}");
    assert!(migrated.contains("group_tasks = { main = \"ctrl\" }   # the hub's one group"));

    let (new, new_err) = resolve(&system, &tmp.path().join("new.yaml"));
    assert!(!new_err.contains("deprecated-spelling"), "{new_err}");
    assert_eq!(
        old["execution"], new["execution"],
        "the same execution layer"
    );
}
