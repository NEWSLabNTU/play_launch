use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicU32, Ordering};

/// Per-process counter combined with PID to produce a unique ROS_DOMAIN_ID
/// across all nextest worker processes (each test binary is a separate process).
static CALL_COUNTER: AtomicU32 = AtomicU32::new(0);

/// Return a ROS_DOMAIN_ID unique across concurrent nextest processes.
/// Combines PID (unique per process) with a per-process call counter,
/// then maps into the valid DDS domain range 1..=232 (FastDDS port
/// formula overflows u16 above 232).
fn next_domain_id() -> u32 {
    let pid = std::process::id();
    let seq = CALL_COUNTER.fetch_add(1, Ordering::Relaxed);
    // Mix pid and seq to spread across domain space; avoid domain 0 (system default)
    (pid.wrapping_mul(7).wrapping_add(seq) % 232) + 1
}

/// Repository root (parent of `tests/`).
pub fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("tests/ crate must live inside the repo root")
        .to_path_buf()
}

/// Path to the `play_launch` binary.
///
/// Searches (in order):
/// 1. `install/play_launch/lib/play_launch/play_launch` (colcon build output)
/// 2. `$PATH` via `which`
pub fn play_launch_bin() -> PathBuf {
    let colcon_bin = repo_root()
        .join("install/play_launch/lib/play_launch/play_launch");
    if colcon_bin.is_file() {
        return colcon_bin;
    }
    which::which("play_launch").expect(
        "play_launch binary not found. Run `just build` first, or ensure it is on PATH.",
    )
}

/// Source a bash setup file and return the resulting environment variables.
fn source_env(script: &Path) -> HashMap<String, String> {
    assert!(
        script.is_file(),
        "setup script not found: {}",
        script.display()
    );

    let output = Command::new("bash")
        .arg("-c")
        .arg(format!(
            "source {} >/dev/null 2>&1 && env -0",
            script.display()
        ))
        .output()
        .expect("failed to run bash");

    assert!(output.status.success(), "failed to source {}", script.display());

    let stdout = String::from_utf8_lossy(&output.stdout);
    stdout
        .split('\0')
        .filter(|s| !s.is_empty())
        .filter_map(|entry| {
            let (k, v) = entry.split_once('=')?;
            Some((k.to_string(), v.to_string()))
        })
        .collect()
}

/// Return environment variables with ROS sourced.
pub fn ros_env() -> HashMap<String, String> {
    let ros_setup = Path::new("/opt/ros/humble/setup.bash");
    if !ros_setup.is_file() {
        // Try jazzy
        let jazzy = Path::new("/opt/ros/jazzy/setup.bash");
        assert!(
            jazzy.is_file(),
            "No ROS setup.bash found (tried humble and jazzy)"
        );
        return source_env(jazzy);
    }
    source_env(ros_setup)
}

/// Return environment variables with ROS + colcon install sourced.
pub fn install_env() -> HashMap<String, String> {
    let install_setup = repo_root().join("install/setup.bash");
    assert!(
        install_setup.is_file(),
        "install/setup.bash not found — run `just build` first"
    );

    // Source both ROS and install
    let ros_setup = if Path::new("/opt/ros/humble/setup.bash").is_file() {
        "/opt/ros/humble/setup.bash"
    } else {
        "/opt/ros/jazzy/setup.bash"
    };

    let output = Command::new("bash")
        .arg("-c")
        .arg(format!(
            "source {} >/dev/null 2>&1 && source {} >/dev/null 2>&1 && env -0",
            ros_setup,
            install_setup.display()
        ))
        .output()
        .expect("failed to run bash");

    assert!(output.status.success(), "failed to source install env");

    let stdout = String::from_utf8_lossy(&output.stdout);
    stdout
        .split('\0')
        .filter(|s| !s.is_empty())
        .filter_map(|entry| {
            let (k, v) = entry.split_once('=')?;
            Some((k.to_string(), v.to_string()))
        })
        .collect()
}

/// Return environment variables with ROS + install + Autoware sourced.
pub fn autoware_env() -> HashMap<String, String> {
    let activate_script = test_workspace_path("autoware").join("activate_autoware.sh");
    assert!(
        activate_script.is_file(),
        "activate_autoware.sh not found: {}. \
         Edit it to source your Autoware install's setup.bash",
        activate_script.display()
    );

    let install_setup = repo_root().join("install/setup.bash");
    assert!(
        install_setup.is_file(),
        "install/setup.bash not found — run `just build` first"
    );

    let ros_setup = if Path::new("/opt/ros/humble/setup.bash").is_file() {
        "/opt/ros/humble/setup.bash"
    } else {
        "/opt/ros/jazzy/setup.bash"
    };

    let cyclonedds_xml = test_workspace_path("autoware").join("cyclonedds.xml");

    let output = Command::new("bash")
        .arg("-c")
        .arg(format!(
            "source {} >/dev/null 2>&1 && source {} >/dev/null 2>&1 && source {} >/dev/null 2>&1 && env -0",
            ros_setup,
            install_setup.display(),
            activate_script.display(),
        ))
        .output()
        .expect("failed to run bash");

    assert!(output.status.success(), "failed to source Autoware env");

    let stdout = String::from_utf8_lossy(&output.stdout);
    let mut env: HashMap<String, String> = stdout
        .split('\0')
        .filter(|s| !s.is_empty())
        .filter_map(|entry| {
            let (k, v) = entry.split_once('=')?;
            Some((k.to_string(), v.to_string()))
        })
        .collect();

    if cyclonedds_xml.is_file() {
        env.insert(
            "CYCLONEDDS_URI".to_string(),
            format!("file://{}", cyclonedds_xml.display()),
        );
    }

    env
}

/// Expected entity counts from `activate_autoware.sh` env vars.
///
/// Returns `(nodes, containers, load_nodes)` as `Option<usize>` — `None` if
/// the corresponding `EXPECTED_*` variable is not set.
pub fn autoware_expected_counts(
    env: &HashMap<String, String>,
) -> (Option<usize>, Option<usize>, Option<usize>) {
    let get = |key: &str| env.get(key).and_then(|v| v.parse::<usize>().ok());
    (
        get("EXPECTED_NODES"),
        get("EXPECTED_CONTAINERS"),
        get("EXPECTED_LOAD_NODES"),
    )
}

/// Default map_path for Autoware tests.
pub fn autoware_map_path() -> String {
    std::env::var("MAP_PATH").unwrap_or_else(|_| {
        let home = std::env::var("HOME").expect("HOME not set");
        format!("{}/autoware_map/sample-map-planning", home)
    })
}

/// Shorthand for the fixture workspace at `tests/fixtures/<name>`.
pub fn test_workspace_path(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("fixtures")
        .join(name)
}

/// Build a `Command` for `play_launch` with the given environment.
///
/// Each call gets a unique `ROS_DOMAIN_ID` (10, 11, 12, …) so concurrent
/// nextest processes don't interfere over DDS discovery/services/topics.
pub fn play_launch_cmd(env: &HashMap<String, String>) -> Command {
    let mut cmd = Command::new(play_launch_bin());
    cmd.env_clear();
    cmd.envs(env);
    // Unique DDS domain per invocation — prevents cross-talk between
    // concurrent tests that spawn containers on the same machine.
    cmd.env("ROS_DOMAIN_ID", next_domain_id().to_string());
    // Disable FastDDS SHM transport in tests.  Clone children clean up SHM
    // segments on graceful SIGTERM shutdown, but SIGKILL'd processes (crash
    // tests, ManagedProcess timeout) leak segments.  UDP-only prevents
    // accumulation in CI where tests run back-to-back indefinitely.
    let fastdds_profile = repo_root().join("tests/fixtures/fastdds_no_shm.xml");
    if fastdds_profile.is_file() {
        cmd.env("FASTRTPS_DEFAULT_PROFILES_FILE", &fastdds_profile);
    }
    cmd
}

/// Count elements in a JSON array field, returning 0 if the field is missing.
pub fn array_len(val: &serde_json::Value, key: &str) -> usize {
    val.get(key)
        .and_then(|v| v.as_array())
        .map_or(0, |a| a.len())
}

/// Dump a launch file with the given environment and parser, returning the
/// parsed `record.json` and the temp directory (kept alive for the caller).
pub fn dump_launch(
    env: &HashMap<String, String>,
    launch_file: &str,
    parser: &str,
) -> (serde_json::Value, tempfile::TempDir) {
    let tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let output_path = tmp.path().join("record.json");

    let mut proc = crate::process::ManagedProcess::spawn(
        play_launch_cmd(env).args([
            "dump",
            "--output",
            output_path.to_str().unwrap(),
            "launch",
            "--parser",
            parser,
            launch_file,
        ]),
    )
    .expect("failed to spawn play_launch dump");

    let status = proc.wait_with_timeout(std::time::Duration::from_secs(60));
    assert!(
        status.success(),
        "play_launch dump (parser={parser}) failed for {launch_file}"
    );
    assert!(
        output_path.is_file(),
        "record.json not created for {launch_file}"
    );

    let data = std::fs::read_to_string(&output_path).expect("failed to read record.json");
    let record = serde_json::from_str(&data).expect("failed to parse record.json");
    (record, tmp)
}

/// Count non-empty `cmdline` files under `play_log_dir/node/*/cmdline`.
pub fn count_cmdline_files(play_log_dir: &Path) -> usize {
    let node_dir = play_log_dir.join("node");
    if !node_dir.is_dir() {
        return 0;
    }

    let mut count = 0;
    if let Ok(entries) = std::fs::read_dir(&node_dir) {
        for entry in entries.flatten() {
            let cmdline = entry.path().join("cmdline");
            if cmdline.is_file() {
                if let Ok(meta) = cmdline.metadata() {
                    if meta.len() > 0 {
                        count += 1;
                    }
                }
            }
        }
    }
    count
}

/// Parse record.json and return `len(node) + len(container)` as the expected
/// process count.
pub fn count_expected_processes(record_path: &Path) -> usize {
    let data = std::fs::read_to_string(record_path).expect("failed to read record.json");
    let record: serde_json::Value =
        serde_json::from_str(&data).expect("failed to parse record.json");

    let nodes = record
        .get("node")
        .and_then(|v| v.as_array())
        .map_or(0, |a| a.len());
    let containers = record
        .get("container")
        .and_then(|v| v.as_array())
        .map_or(0, |a| a.len());
    nodes + containers
}

/// Run `scripts/compare_records.py` on two record.json files.
///
/// Returns `(success, output)` where `success` is true when the script exits 0
/// (records are functionally equivalent) and `output` is the combined
/// stdout+stderr for diagnostic printing on failure.
pub fn compare_records(rust_record: &Path, python_record: &Path) -> (bool, String) {
    let script = repo_root().join("scripts/compare_records.py");
    assert!(script.is_file(), "compare_records.py not found");

    let output = Command::new("python3")
        .arg(&script)
        .arg(rust_record)
        .arg(python_record)
        .output()
        .expect("failed to run compare_records.py");

    let combined = format!(
        "{}\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
    (output.status.success(), combined)
}

/// Wait until the cmdline file count in `play_log_dir` reaches `expected` or
/// stabilizes, up to `timeout`.
pub fn wait_for_processes(play_log_dir: &Path, expected: usize, timeout: std::time::Duration) {
    let start = std::time::Instant::now();
    let poll_interval = std::time::Duration::from_secs(2);
    let mut prev_count = 0usize;
    let mut stable_polls = 0u32;

    while start.elapsed() < timeout {
        std::thread::sleep(poll_interval);
        let actual = count_cmdline_files(play_log_dir);

        eprintln!(
            "  {:>3}s: {}/{} processes",
            start.elapsed().as_secs(),
            actual,
            expected
        );

        if actual == prev_count && actual > 0 {
            stable_polls += 1;
        } else {
            stable_polls = 0;
        }
        prev_count = actual;

        // Stabilized for 3 consecutive polls
        if stable_polls >= 3 {
            break;
        }

        // Reached expected count and confirmed once
        if actual >= expected && stable_polls >= 1 {
            break;
        }
    }
}
