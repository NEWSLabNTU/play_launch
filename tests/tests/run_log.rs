//! Issue #0023 — the launcher writes its own log into the run's `log_dir`.
//!
//! A bundle from a failed launch used to contain every node's output and
//! nothing from play_launch itself; the version that produced it was not
//! recorded anywhere either. All three run verbs (`launch`, `up`, `run`)
//! must leave `play_launch.log` and `run_info.json` in `play_log/<ts>/`.

use std::{
    process::Stdio,
    time::{Duration, Instant},
};

use play_launch_tests::{fixtures, process::ManagedProcess};

fn container_events_launch() -> String {
    fixtures::test_workspace_path("container_events")
        .join("launch/container_events.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

/// `play_launch --version` → `0.10.0` (whatever the binary says).
fn binary_version(env: &std::collections::HashMap<String, String>) -> String {
    let out = fixtures::play_launch_cmd(env)
        .arg("--version")
        .output()
        .expect("run play_launch --version");
    let text = String::from_utf8_lossy(&out.stdout);
    text.trim()
        .rsplit(' ')
        .next()
        .expect("version string")
        .to_string()
}

fn wait_for_file_containing(path: &std::path::Path, needle: &str, timeout: Duration) -> String {
    let start = Instant::now();
    loop {
        let text = std::fs::read_to_string(path).unwrap_or_default();
        if text.contains(needle) {
            return text;
        }
        assert!(
            start.elapsed() < timeout,
            "{} did not contain {needle:?} within {timeout:?}; content:\n{text}",
            path.display()
        );
        std::thread::sleep(Duration::from_millis(250));
    }
}

struct Run {
    _proc: ManagedProcess,
    _tmp: tempfile::TempDir,
    work_dir: std::path::PathBuf,
}

impl Run {
    fn bundle(&self) -> std::path::PathBuf {
        self.work_dir.join("play_log/latest")
    }
}

/// Spawn one of the run verbs in a fresh work directory with NO `RUST_LOG`:
/// the file must carry debug detail without the terminal being asked for it.
fn spawn(env: &std::collections::HashMap<String, String>, args: &[&str]) -> Run {
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let work_dir = tmp.path().to_path_buf();
    let mut cmd = fixtures::play_launch_cmd(env);
    cmd.current_dir(&work_dir);
    cmd.args(args);
    cmd.stdout(Stdio::from(
        std::fs::File::create(work_dir.join("stdout.log")).unwrap(),
    ));
    cmd.stderr(Stdio::from(
        std::fs::File::create(work_dir.join("stderr.log")).unwrap(),
    ));
    cmd.env_remove("RUST_LOG");
    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    Run {
        _proc: proc,
        _tmp: tmp,
        work_dir,
    }
}

/// `launch`: the log exists, opens with the version, holds the lines that
/// were emitted BEFORE the directory existed (the parse step), holds debug
/// detail the terminal was never asked for, and `run_info.json` names the
/// version, the argv and the config that was copied beside it.
#[test]
fn test_launch_writes_launcher_log_with_version_argv_and_config() {
    let env = fixtures::install_env();
    let version = binary_version(&env);
    let launch = container_events_launch();

    // A config file, so the copy path is exercised too.
    let cfg_tmp = tempfile::TempDir::new().unwrap();
    let cfg = cfg_tmp.path().join("my_runtime.yaml");
    std::fs::write(&cfg, "monitoring:\n  sample_interval_ms: 2000\n").unwrap();

    let run = spawn(
        &env,
        &[
            "launch",
            "--disable-web-ui",
            "--disable-monitoring",
            "--disable-diagnostics",
            "--config",
            cfg.to_str().unwrap(),
            &launch,
        ],
    );

    let log_path = run.bundle().join("play_launch.log");
    let text = wait_for_file_containing(&log_path, "Startup complete", Duration::from_secs(60));

    // Header first.
    assert!(
        text.starts_with(&format!("# play_launch {version} ")),
        "header should open with the version; got:\n{}",
        text.lines().take(8).collect::<Vec<_>>().join("\n")
    );
    assert!(
        text.contains(&format!(
            "# config: {} (copied to config.yaml)",
            cfg.display()
        )),
        "{text}"
    );

    // The parse step runs before `play_log/<ts>/` exists; its lines were
    // buffered and must precede everything the run wrote afterwards.
    let parse_at = text
        .find("Step 1/3: Parsing launch file")
        .expect("buffered parse line");
    let created_at = text
        .find("Log directory created")
        .expect("a DEBUG line — the file takes debug without RUST_LOG");
    assert!(parse_at < created_at, "{text}");
    assert!(
        text.contains("DEBUG"),
        "no debug lines in the file:\n{text}"
    );
    assert!(text.contains("Launcher log:"), "{text}");

    // The bundle also says what produced it.
    let info: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(run.bundle().join("run_info.json")).expect("run_info.json"),
    )
    .expect("run_info.json is JSON");
    assert_eq!(info["version"], version);
    assert_eq!(info["config_copy"], "config.yaml");
    let argv: Vec<String> = serde_json::from_value(info["argv"].clone()).unwrap();
    assert!(argv.iter().any(|a| a == "launch"), "{argv:?}");
    assert!(argv.iter().any(|a| a == &launch), "{argv:?}");
    assert!(info["pid"].as_u64().unwrap() > 0);
    assert_eq!(
        std::fs::read_to_string(run.bundle().join("config.yaml")).unwrap(),
        "monitoring:\n  sample_interval_ms: 2000\n"
    );

    // And the terminal stayed at INFO: no DEBUG leaked there.
    let stdout = std::fs::read_to_string(run.work_dir.join("stdout.log")).unwrap();
    assert!(
        !stdout.contains("DEBUG"),
        "terminal got debug lines:\n{stdout}"
    );
}

/// `up` from a resolved model gets the same file.
#[test]
fn test_up_writes_launcher_log() {
    let env = fixtures::install_env();
    let version = binary_version(&env);
    let (_, model_tmp) = fixtures::resolve_model(&env, &container_events_launch(), None, "rust");
    let model = model_tmp.path().join("system_model.yaml");
    assert!(
        model.is_file(),
        "resolve_model should leave system_model.yaml"
    );

    let run = spawn(
        &env,
        &[
            "up",
            "--disable-web-ui",
            "--disable-monitoring",
            "--disable-diagnostics",
            model.to_str().unwrap(),
        ],
    );

    let text = wait_for_file_containing(
        &run.bundle().join("play_launch.log"),
        "Startup complete",
        Duration::from_secs(60),
    );
    assert!(
        text.starts_with(&format!("# play_launch {version} ")),
        "{text}"
    );
    let info: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(run.bundle().join("run_info.json")).unwrap())
            .unwrap();
    assert_eq!(info["version"], version);
    assert!(info["config_path"].is_null());
    assert!(info["config_copy"].is_null());
}

/// `run` (single node, no launch file) gets it too.
#[test]
fn test_run_writes_launcher_log() {
    let env = fixtures::install_env();
    let version = binary_version(&env);
    let run = spawn(&env, &["run", "--disable-all", "demo_nodes_cpp", "talker"]);

    let text = wait_for_file_containing(
        &run.bundle().join("play_launch.log"),
        "Launcher log:",
        Duration::from_secs(30),
    );
    assert!(
        text.starts_with(&format!("# play_launch {version} ")),
        "{text}"
    );
    assert!(
        run.bundle().join("run_info.json").is_file(),
        "run_info.json missing from {}",
        run.bundle().display()
    );
}
