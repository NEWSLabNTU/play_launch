//! `play_launch log list|prune` (#0098), through the binary: a prune
//! without `--yes` removes nothing, `--class` keeps the run and removes only
//! that kind of file, and a whole-run prune leaves `latest` and anything
//! that is not a run alone.

use play_launch_tests::fixtures::play_launch_bin;
use std::{fs, path::Path, process::Command};

fn make_run(base: &Path, name: &str) {
    let d = base.join(name);
    fs::create_dir_all(d.join("node/talker")).unwrap();
    fs::write(d.join("diagnostics.csv"), vec![b'x'; 2_000_000]).unwrap();
    fs::write(d.join("node/talker/out"), "hello\n").unwrap();
}

fn log(base: &Path, args: &[&str]) -> (bool, String) {
    let out = Command::new(play_launch_bin())
        .arg("log")
        .args(args)
        .arg("--log-dir")
        .arg(base)
        .output()
        .expect("play_launch log runs");
    (
        out.status.success(),
        format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ),
    )
}

#[test]
fn log_prune_selects_by_date_and_class_and_needs_yes() {
    let t = tempfile::tempdir().unwrap();
    let base = t.path();
    make_run(base, "2026-01-01_00-00-00");
    make_run(base, "2026-01-02_00-00-00");
    make_run(base, "2099-01-01_00-00-00");
    fs::create_dir(base.join("keepme")).unwrap();
    std::os::unix::fs::symlink("2099-01-01_00-00-00", base.join("latest")).unwrap();

    let (ok, out) = log(base, &["list"]);
    assert!(ok, "{out}");
    assert!(out.contains("3 run(s), 6 MB"), "{out}");

    // A dry run removes nothing.
    let (ok, out) = log(
        base,
        &["prune", "--before", "2026-01-02", "--class", "diagnostics"],
    );
    assert!(ok && out.contains("Nothing removed; add --yes"), "{out}");
    assert!(base.join("2026-01-01_00-00-00/diagnostics.csv").exists());

    // By class: the run stays, its other files too.
    let (ok, out) = log(
        base,
        &[
            "prune",
            "--before",
            "2026-01-02",
            "--class",
            "diagnostics",
            "--yes",
        ],
    );
    assert!(
        ok && out.contains("Removed 2 MB from 1 run(s) (diagnostics)"),
        "{out}"
    );
    assert!(!base.join("2026-01-01_00-00-00/diagnostics.csv").exists());
    assert!(base.join("2026-01-01_00-00-00/node/talker/out").exists());

    // Whole runs, keeping the newest one.
    let (ok, out) = log(base, &["prune", "--keep", "1", "--yes"]);
    assert!(ok && out.contains("from 2 run(s) (whole runs)"), "{out}");
    let mut left: Vec<String> = fs::read_dir(base)
        .unwrap()
        .map(|e| e.unwrap().file_name().into_string().unwrap())
        .collect();
    left.sort();
    assert_eq!(left, vec!["2099-01-01_00-00-00", "keepme", "latest"]);

    // No selection is refused.
    let (ok, out) = log(base, &["prune", "--yes"]);
    assert!(!ok && out.contains("say which runs"), "{out}");
}
