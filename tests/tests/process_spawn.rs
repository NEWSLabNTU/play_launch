//! What a process starts IN and UNDER, against stock `ros2 launch` (#0085,
//! #0087, GitHub #27).
//!
//! `tests/fixtures/process_spawn/` holds one launch file per frontend. Every
//! process in it is `probe.sh`, or is wrapped by `probe.sh` as its
//! `launch-prefix`, and writes its working directory and a few environment
//! variables to `$PL_PROBE_OUT/<label>`. The same file runs under `ros2
//! launch` and under play_launch with the same environment, and each label's
//! file must be identical:
//!
//! - `<unset_env>` in a scoped group: gone inside (`g_in`, `node_wrap`,
//!   `cont_wrap`), back after (`g_after`)
//! - a top-level unset of an inherited variable (`top`), and only for what
//!   follows it (`g_after` still has it)
//! - `<set_env>` then `<unset_env>` of the same name (`set_unset`)
//! - `launch-prefix` on an `<executable>` (`env -u`, `exe_pfx`; a wrapper
//!   that records it ran, `exe_wrap` then `exe_inner`), on a `<node>`
//!   (`node_wrap`) and on a `<node_container>` under the default container
//!   mode, where play_launch replaces the binary (`cont_wrap`)
//! - `cwd` with a substitution (`cwd_exe`, `node_wrap`)
//! - `shell="true"` (`shell_exe`: a shell assignment in front of the command)
//!
//! The expectations are also asserted outright, so a host whose stock launch
//! is skipped still checks play_launch, and so both launchers agreeing on a
//! wrong answer cannot pass.

use play_launch_tests::{fixtures, process::ManagedProcess};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
    process::{Command, Stdio},
    time::{Duration, Instant},
};

const LABELS: &[&str] = &[
    "g_in",
    "node_wrap",
    "cont_wrap",
    "g_after",
    "set_unset",
    "top",
    "exe_pfx",
    "exe_wrap",
    "exe_inner",
    "cwd_exe",
    "shell_exe",
];

fn fixture(name: &str) -> PathBuf {
    fixtures::repo_root()
        .join("tests/fixtures/process_spawn")
        .join(name)
}

/// A scratch directory holding a copy of the probe and a `work/` cwd target.
struct Workspace {
    _tmp: tempfile::TempDir,
    dir: PathBuf,
}

impl Workspace {
    fn new() -> Self {
        let tmp = tempfile::TempDir::new().expect("tempdir");
        // Canonical, so `pwd -P` and the expected paths agree.
        let dir = tmp.path().canonicalize().unwrap();
        std::fs::create_dir_all(dir.join("work")).unwrap();
        let probe = dir.join("probe.sh");
        std::fs::copy(fixture("probe.sh"), &probe).unwrap();
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&probe, std::fs::Permissions::from_mode(0o755)).unwrap();
        Self { _tmp: tmp, dir }
    }

    /// The environment every launcher gets: what the processes inherit.
    fn apply(&self, cmd: &mut Command, out: &Path) {
        std::fs::create_dir_all(out).unwrap();
        cmd.current_dir(&self.dir)
            .env("PL_PROBE_DIR", &self.dir)
            .env("PL_PROBE_OUT", out)
            .env("DISPLAY", ":77")
            .env("PL_PROBE_INHERITED", "inh")
            .env("PL_PROBE_TOP", "top")
            .env("PL_PROBE_PFX", "pfx")
            .env("ROS_LOG_DIR", self.dir.join("roslog"));
    }
}

/// Run `cmd` until every label has written its file, then stop it, and
/// return each label's record.
fn run_until_probed(
    label: &str,
    mut cmd: Command,
    out: &Path,
    log: &Path,
) -> BTreeMap<String, String> {
    cmd.stdout(Stdio::from(std::fs::File::create(log).unwrap()));
    cmd.stderr(Stdio::from(
        std::fs::File::create(log.with_extension("err")).unwrap(),
    ));
    let mut proc = ManagedProcess::spawn(&mut cmd).expect("spawn launcher");
    let started = Instant::now();
    let deadline = Duration::from_secs(30);
    loop {
        if LABELS.iter().all(|l| out.join(l).is_file()) {
            break;
        }
        if proc.try_wait().is_some() || started.elapsed() > deadline {
            let missing: Vec<_> = LABELS.iter().filter(|l| !out.join(l).is_file()).collect();
            let text = std::fs::read_to_string(log).unwrap_or_default()
                + &std::fs::read_to_string(log.with_extension("err")).unwrap_or_default();
            panic!(
                "{label}: no record from {missing:?} after {:?}:\n{}",
                started.elapsed(),
                &text[text.len().saturating_sub(4000)..]
            );
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    // Graceful: play_launch stops the processes it started in its own group.
    proc.terminate(Duration::from_secs(20));
    LABELS
        .iter()
        .map(|l| (l.to_string(), std::fs::read_to_string(out.join(l)).unwrap()))
        .collect()
}

fn play_launch(ws: &Workspace, out: &Path, args: &[&str]) -> BTreeMap<String, String> {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    ws.apply(&mut cmd, out);
    cmd.args(args).args([
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
    ]);
    let log = out.with_extension("log");
    run_until_probed("play_launch", cmd, out, &log)
}

/// `ros2 launch <file>`, or `None` with a visible SKIP where it cannot run.
fn stock(ws: &Workspace, file: &Path) -> Option<BTreeMap<String, String>> {
    let env = fixtures::install_env();
    let Some(ros2) = env.get("PATH").and_then(|path| {
        std::env::split_paths(path)
            .map(|p| p.join("ros2"))
            .find(|p| p.is_file())
    }) else {
        eprintln!("SKIP: process_spawn: `ros2` is not on the ROS PATH; checking play_launch alone");
        return None;
    };
    let out = ws.dir.join("out_stock");
    let mut cmd = Command::new(ros2);
    fixtures::apply_test_env(&mut cmd, &env);
    ws.apply(&mut cmd, &out);
    cmd.arg("launch").arg(file);
    Some(run_until_probed(
        "ros2 launch",
        cmd,
        &out,
        &ws.dir.join("out_stock.log"),
    ))
}

fn field<'a>(record: &'a str, key: &str) -> &'a str {
    record
        .lines()
        .find_map(|l| l.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("no {key} in {record}"))
}

/// What `ros2 launch` gives each label, stated outright.
fn assert_expected(who: &str, ws: &Workspace, got: &BTreeMap<String, String>) {
    let base = ws.dir.display().to_string();
    let work = ws.dir.join("work").display().to_string();
    let r = |l: &str| got[l].as_str();
    for l in ["g_in", "node_wrap", "cont_wrap"] {
        assert_eq!(field(r(l), "DISPLAY"), "<unset>", "{who} {l}: {}", r(l));
        assert_eq!(field(r(l), "PL_PROBE_INHERITED"), "<unset>", "{who} {l}");
        assert_eq!(
            field(r(l), "PL_PROBE_TOP"),
            "top",
            "{who} {l}: a later unset"
        );
    }
    assert_eq!(
        field(r("g_after"), "DISPLAY"),
        ":77",
        "{who}: the group restored it"
    );
    assert_eq!(field(r("g_after"), "PL_PROBE_INHERITED"), "inh", "{who}");
    assert_eq!(field(r("g_after"), "PL_PROBE_TOP"), "top", "{who}");
    assert_eq!(field(r("set_unset"), "PL_PROBE_SET"), "<unset>", "{who}");
    assert_eq!(field(r("top"), "PL_PROBE_TOP"), "<unset>", "{who}");
    assert_eq!(
        field(r("exe_pfx"), "PL_PROBE_PFX"),
        "<unset>",
        "{who}: env -u"
    );
    assert_eq!(field(r("exe_inner"), "PL_PROBE_PFX"), "pfx", "{who}");
    assert_eq!(field(r("node_wrap"), "pwd"), work, "{who}");
    assert_eq!(field(r("cwd_exe"), "pwd"), work, "{who}");
    assert_eq!(
        field(r("g_in"), "pwd"),
        base,
        "{who}: no cwd is the launcher's"
    );
    assert_eq!(
        field(r("shell_exe"), "PL_PROBE_SHELL"),
        "yes",
        "{who}: through a shell"
    );
}

fn check(file: &str, play_launch_args: &[&str]) {
    if fixtures::install_env().is_empty() {
        eprintln!("SKIP: process_spawn: no ROS environment");
        return;
    }
    let ws = Workspace::new();
    let launch = fixture(file);
    let mut args = vec!["launch", launch.to_str().unwrap()];
    args.extend_from_slice(play_launch_args);
    let ours = play_launch(&ws, &ws.dir.join("out_pl"), &args);
    assert_expected("play_launch", &ws, &ours);
    if let Some(theirs) = stock(&ws, &launch) {
        assert_expected("ros2 launch", &ws, &theirs);
        assert_eq!(ours, theirs, "play_launch differs from ros2 launch");
    }
    // The prefix is on the container's recorded command line too, in front
    // of play_launch's own container binary.
    let cmdline = std::fs::read_to_string(ws.dir.join("play_log/latest/node/pl_probe_c/cmdline"))
        .expect("container cmdline");
    assert!(
        cmdline.starts_with(&format!("{}/probe.sh cont_wrap ", ws.dir.display())),
        "{cmdline}"
    );
    assert!(cmdline.contains("play_launch_container"), "{cmdline}");
}

#[test]
fn xml_matches_ros2_launch() {
    check("spawn.launch.xml", &[]);
}

#[test]
fn python_frontend_matches_ros2_launch() {
    check("spawn.launch.py", &[]);
}

#[test]
fn python_parser_on_xml_matches_ros2_launch() {
    check("spawn.launch.xml", &["--parser", "python"]);
}

#[test]
fn python_parser_on_python_matches_ros2_launch() {
    check("spawn.launch.py", &["--parser", "python"]);
}

/// `up <model.yaml>` reads only the model, so what it applies must be there.
#[test]
fn up_from_a_resolved_model_matches() {
    if fixtures::install_env().is_empty() {
        eprintln!("SKIP: process_spawn: no ROS environment");
        return;
    }
    let ws = Workspace::new();
    let env = fixtures::install_env();
    let model = ws.dir.join("model.yaml");
    let mut resolve = fixtures::play_launch_cmd(&env);
    ws.apply(&mut resolve, &ws.dir.join("unused"));
    let out = resolve
        .arg("resolve")
        .arg(fixture("spawn.launch.xml"))
        .arg("-o")
        .arg(&model)
        .output()
        .expect("run resolve");
    assert!(
        out.status.success(),
        "resolve failed:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let text = std::fs::read_to_string(&model).unwrap();
    for key in ["env_unset:", "launch_prefix:", "cwd:"] {
        assert!(text.contains(key), "{key} missing from the model:\n{text}");
    }
    let ours = play_launch(
        &ws,
        &ws.dir.join("out_up"),
        &["up", model.to_str().unwrap()],
    );
    assert_expected("play_launch up", &ws, &ours);
}

/// The two parsers produce the same model for both frontends, fields
/// included.
#[test]
fn both_parsers_resolve_the_same_spawn_fields() {
    if fixtures::install_env().is_empty() {
        eprintln!("SKIP: process_spawn: no ROS environment");
        return;
    }
    let env = fixtures::install_env();
    for file in ["spawn.launch.xml", "spawn.launch.py"] {
        let ws = Workspace::new();
        let mut models = Vec::new();
        for parser in ["rust", "python"] {
            let model = ws.dir.join(format!("{parser}.yaml"));
            let mut resolve = fixtures::play_launch_cmd(&env);
            ws.apply(&mut resolve, &ws.dir.join("unused"));
            let out = resolve
                .args(["resolve", "--parser", parser])
                .arg(fixture(file))
                .arg("-o")
                .arg(&model)
                .output()
                .expect("run resolve");
            assert!(
                out.status.success(),
                "{file} {parser}: resolve failed:\n{}",
                String::from_utf8_lossy(&out.stderr)
            );
            models.push(model);
        }
        let (same, report) = fixtures::compare_models(&models[0], &models[1]);
        assert!(same, "{file}: the parsers disagree:\n{report}");
    }
}
