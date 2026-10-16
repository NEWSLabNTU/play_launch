//! A respawned process does not destroy the log of the one before it.
//!
//! `NodeContext::to_exec_context` opens `out` and `err` with `File::create`,
//! and a respawn comes back through it, so every restart truncated the files
//! the dead process had written -- the one log that says why it died. Found
//! by carla-scenario-bridge (roadmap 034): its bridge drains and `_exit(75)`s
//! after a hung CARLA call, play_launch respawns it, and the dying bridge's
//! account of the hang was gone; it had to be captured with `tail -F`.
//!
//! The layout now: the live process writes the plain names, and each earlier
//! process's `out`/`err`/`pid`/`status` are kept as `out.N`/`err.N`/... in
//! start order (`execution::context::rotate_previous_incarnation`).
//!
//! The fixture: a script that prints its PID to both streams and exits 3,
//! declared `respawn="true"`, beside the same script without respawn. Each
//! process is told apart by its PID, so a file holding the wrong process's
//! output, or two processes' output, is caught -- not only a missing file.

use std::{
    path::{Path, PathBuf},
    process::Stdio,
    time::{Duration, Instant},
};

use play_launch_tests::{fixtures, process::ManagedProcess};

const SCRIPT: &str = "#!/bin/sh\n\
echo \"out of pid $$\"\n\
echo \"err of pid $$\" >&2\n\
sleep 0.5\n\
exit 3\n";

fn launch_xml(script: &Path) -> String {
    let s = script.display();
    format!(
        r#"<launch>
  <executable cmd="{s}" name="crasher" respawn="true" respawn_delay="0.2"/>
  <executable cmd="{s}" name="oneshot"/>
  <executable cmd="sleep 300" name="bystander"/>
</launch>
"#
    )
}

fn read(dir: &Path, f: &str) -> String {
    std::fs::read_to_string(dir.join(f))
        .unwrap_or_else(|e| panic!("{}: {e}", dir.join(f).display()))
}

/// The one member directory whose `cmdline` names `name`'s command and that
/// is not another member's -- found by name, falling back to a listing in the
/// failure message so a naming change is diagnosable.
fn member_dir(node_dir: &Path, name: &str) -> PathBuf {
    let dir = node_dir.join(name);
    if dir.is_dir() {
        return dir;
    }
    let names: Vec<String> = std::fs::read_dir(node_dir)
        .map(|it| {
            it.filter_map(|e| e.ok())
                .map(|e| e.file_name().to_string_lossy().into_owned())
                .collect()
        })
        .unwrap_or_default();
    panic!(
        "no member directory {name} under {}: {names:?}",
        node_dir.display()
    );
}

fn listing(dir: &Path) -> Vec<String> {
    let mut v: Vec<String> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    v.sort();
    v
}

#[test]
fn a_respawn_keeps_every_earlier_processs_logs_and_a_oneshot_is_unchanged() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("SKIP: respawn_logs: ROS env not available");
        return;
    }
    let work_tmp = tempfile::TempDir::new().expect("tempdir");
    let work_dir = work_tmp.path();
    let script = work_dir.join("crash.sh");
    std::fs::write(&script, SCRIPT).unwrap();
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o755)).unwrap();
    }
    let launch = work_dir.join("respawn.launch.xml");
    std::fs::write(&launch, launch_xml(&script)).unwrap();

    let stdout = work_dir.join("stdout.log");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_dir)
        .arg("launch")
        .arg(&launch)
        .args([
            "--disable-web-ui",
            "--disable-monitoring",
            "--disable-diagnostics",
        ])
        .stdout(Stdio::from(std::fs::File::create(&stdout).unwrap()))
        .stderr(Stdio::from(
            std::fs::File::create(work_dir.join("stderr.log")).unwrap(),
        ));
    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");

    // Three lives: the first two kept, the third live (or about to be).
    let node_dir = work_dir.join("play_log/latest/node");
    let started = Instant::now();
    let crasher = loop {
        let dir = node_dir.join("crasher");
        if dir.join("status.2").exists() && dir.join("pid").exists() {
            break dir;
        }
        if started.elapsed() > Duration::from_secs(90) {
            let out = std::fs::read_to_string(&stdout).unwrap_or_default();
            let have = if dir.is_dir() { listing(&dir) } else { vec![] };
            panic!(
                "crasher never reached its third process within 90 s; its directory \
                 holds {have:?}.\n{}",
                &out[out.len().saturating_sub(3000)..]
            );
        }
        std::thread::sleep(Duration::from_millis(100));
    };
    drop(proc);

    // Each kept process: its own PID in pid.N, in out.N and in err.N, and
    // its exit status. All different, so no two lives share a file.
    let mut pids = Vec::new();
    for n in 1..=2 {
        let pid = read(&crasher, &format!("pid.{n}")).trim().to_string();
        assert!(!pid.is_empty(), "pid.{n} is empty");
        assert_eq!(
            read(&crasher, &format!("out.{n}")),
            format!("out of pid {pid}\n"),
            "out.{n} is not process {n}'s whole stdout"
        );
        assert_eq!(
            read(&crasher, &format!("err.{n}")),
            format!("err of pid {pid}\n"),
            "err.{n} is not process {n}'s whole stderr"
        );
        assert_eq!(read(&crasher, &format!("status.{n}")).trim(), "3");
        pids.push(pid);
    }
    let live = read(&crasher, "pid").trim().to_string();
    pids.push(live.clone());
    let mut unique = pids.clone();
    unique.sort();
    unique.dedup();
    assert_eq!(unique.len(), 3, "three lives, three PIDs: {pids:?}");
    // The plain names belong to the newest process (or are empty if it was
    // stopped before it printed): never an earlier one's. The shell may add
    // its own line when the stop interrupts its `sleep`.
    let err = read(&crasher, "err");
    assert!(
        err.is_empty() || err.starts_with(&format!("err of pid {live}\n")),
        "err must be the live process's, got {err:?} (live pid {live}, earlier {pids:?})"
    );
    assert!(
        !crasher.join("cmdline.1").exists(),
        "cmdline is the member's, not a process's: {:?}",
        listing(&crasher)
    );

    // The member that never restarts has the layout it always had.
    let oneshot = member_dir(&node_dir, "oneshot");
    let files = listing(&oneshot);
    assert!(
        files
            .iter()
            .all(|f| !f.contains('.') || f.ends_with(".json") || f.ends_with(".csv")),
        "a process that ran once must have no numbered files: {files:?}"
    );
    let pid = read(&oneshot, "pid").trim().to_string();
    assert_eq!(read(&oneshot, "err"), format!("err of pid {pid}\n"));
    assert_eq!(read(&oneshot, "out"), format!("out of pid {pid}\n"));
    assert_eq!(read(&oneshot, "status").trim(), "3");
}
