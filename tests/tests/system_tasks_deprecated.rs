//! Phase 99 T4: a `system.toml` written with the pre-rename keys
//! (`[tiers.*]`, `group_tiers`) is refused by `resolve --system` since rlm
//! v0.3.11 (play_launch 0.28), the message naming each new key and
//! `migrate`; after `play_launch migrate --write` it resolves to the same
//! execution layer as its twin written in the new spelling
//! (`twin.system.toml`). The model's key is `execution.tasks` (schema 5).

use play_launch_tests::fixtures;

/// Resolve the multihost fixture (every host) against `system`; returns
/// whether it succeeded, the model when it did, and the resolver's stderr.
fn resolve(
    system: &std::path::Path,
    out: &std::path::Path,
) -> (bool, Option<serde_json::Value>, String) {
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
    if !output.status.success() {
        return (false, None, stderr);
    }
    let yaml = std::fs::read_to_string(out).expect("read model");
    (
        true,
        Some(serde_yaml_ng::from_str(&yaml).expect("parse model yaml")),
        stderr,
    )
}

#[test]
fn old_task_keys_are_refused_and_migrate_to_the_twin() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let env = fixtures::install_env();
    let dir = fixtures::repo_root().join("tests/fixtures/system_tasks_deprecated");
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let system = tmp.path().join("system.toml");
    std::fs::copy(dir.join("system.toml"), &system).expect("copy fixture");

    // The twin, in the new spelling, resolves clean.
    let (ok, twin, twin_err) =
        resolve(&dir.join("twin.system.toml"), &tmp.path().join("twin.yaml"));
    assert!(ok, "the twin resolves:\n{twin_err}");
    let twin = twin.unwrap();
    assert!(!twin_err.contains("deprecated-spelling"), "{twin_err}");
    assert_eq!(
        twin["execution"]["tasks"]["ctrl"]["posix"]["priority"], 40,
        "{twin}"
    );
    assert!(twin["execution"].get("tiers").is_none(), "{twin}");
    assert_eq!(
        twin["execution"]["bindings"]["/hub/hub/main"], "ctrl",
        "{twin}"
    );
    assert_eq!(
        twin["execution"]["bindings"]["/robot1/talker1/main"], "telem",
        "{twin}"
    );

    // The old spelling is refused, naming the new key and `migrate`. The
    // reader checks `[tiers.*]` first, so that is the key it names.
    let (ok, _, old_err) = resolve(&system, &tmp.path().join("old.yaml"));
    assert!(!ok, "the old spelling must be refused:\n{old_err}");
    assert!(
        old_err.contains("`[tiers.<name>]` was renamed `[tasks.<name>]`"),
        "{old_err}"
    );
    assert!(old_err.contains("(ctrl, telem)"), "{old_err}");
    assert!(old_err.contains("`migrate`"), "{old_err}");

    // `group_tiers` alone is refused too, naming its own replacement.
    let groups_only = tmp.path().join("groups.system.toml");
    std::fs::write(
        &groups_only,
        std::fs::read_to_string(dir.join("twin.system.toml"))
            .unwrap()
            .replace("group_tasks", "group_tiers"),
    )
    .unwrap();
    let (ok, _, g_err) = resolve(&groups_only, &tmp.path().join("groups.yaml"));
    assert!(!ok, "group_tiers must be refused:\n{g_err}");
    assert!(
        g_err.contains("`[[component]] group_tiers` was renamed `[[component]] group_tasks`"),
        "{g_err}"
    );
    assert!(g_err.contains("(hub, talker1)"), "{g_err}");

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

    let (ok, new, new_err) = resolve(&system, &tmp.path().join("new.yaml"));
    assert!(ok, "the migrated file resolves:\n{new_err}");
    assert_eq!(
        new.unwrap()["execution"],
        twin["execution"],
        "the migrated file is the twin's execution layer"
    );
}
