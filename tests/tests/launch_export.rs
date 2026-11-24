//! `play_launch export` (phase 106 W5).
//!
//! The exported file must start exactly the selected members: export checks
//! that itself by resolving its own output, and here the file is also run
//! by play_launch and parsed by stock `ros2 launch --print`.

use std::{path::Path, process::Command};

use play_launch_tests::fixtures;

fn fixture() -> String {
    fixtures::test_workspace_path("simple_test")
        .join("launch/selection.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

fn play_launch(args: &[&str], cwd: &Path) -> (bool, String) {
    let env = fixtures::install_env();
    let mut cmd: Command = fixtures::play_launch_cmd(&env);
    cmd.current_dir(cwd).args(args);
    let out = cmd.output().unwrap();
    (
        out.status.success(),
        format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ),
    )
}

/// The members a launch file resolves to (`resolve -o -`).
fn members_of(file: &Path, cwd: &Path) -> Vec<String> {
    let env = fixtures::install_env();
    let mut cmd: Command = fixtures::play_launch_cmd(&env);
    cmd.current_dir(cwd)
        .args(["resolve", file.to_str().unwrap(), "-o", "-"]);
    let out = cmd.output().unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let model: serde_yaml_ng::Value = serde_yaml_ng::from_slice(&out.stdout).unwrap();
    let mut keys: Vec<String> = model["structure"]["nodes"]
        .as_mapping()
        .unwrap()
        .keys()
        .map(|k| k.as_str().unwrap().to_string())
        .collect();
    keys.sort();
    keys
}

/// Stock `ros2 launch --print` parses the file (skipped without ros2).
fn stock_parses(file: &Path) {
    let env = fixtures::install_env();
    let mut cmd = Command::new("ros2");
    for (k, v) in &env {
        cmd.env(k, v);
    }
    let Ok(out) = cmd
        .args(["launch", "--print", file.to_str().unwrap()])
        .output()
    else {
        eprintln!("SKIP: ros2 not on PATH");
        return;
    };
    assert!(
        out.status.success(),
        "ros2 launch --print refused {}: {}",
        file.display(),
        String::from_utf8_lossy(&out.stderr)
    );
}

#[test]
fn include_form_keeps_includes_and_starts_exactly_the_selection() {
    let tmp = tempfile::TempDir::new().unwrap();
    let (ok, text) = play_launch(
        &[
            "export",
            &fixture(),
            "--only",
            "selection_part.launch.xml[part=a]",
            "-o",
            "out",
            "--name",
            "part_a",
        ],
        tmp.path(),
    );
    assert!(ok, "{text}");
    assert!(!text.contains("does not reproduce"), "{text}");
    let file = tmp.path().join("out/part_a.launch.xml");
    let xml = std::fs::read_to_string(&file).unwrap();
    // The inclusion is kept as an include with its argument; the container
    // it needs, declared in the parent file, is written flat.
    assert!(xml.contains("selection_part.launch.xml\">"), "{xml}");
    assert!(xml.contains(r#"<arg name="part" value="a"/>"#), "{xml}");
    assert!(xml.contains(r#"name="sel_container""#), "{xml}");
    assert!(!xml.contains("pure_nodes"), "{xml}");
    assert_eq!(
        members_of(&file, tmp.path()),
        ["/sel/listener_a", "/sel/sel_container", "/sel/talker_a"]
    );
    stock_parses(&file);

    // --again regenerates it from the command in its header.
    std::fs::remove_file(&file).unwrap();
    let (ok, text) = play_launch(&["export", "--again", "out/part_a.launch.xml"], tmp.path());
    assert!(!ok, "a missing file cannot be regenerated: {text}");
    std::fs::write(&file, &xml).unwrap();
    let (ok, text) = play_launch(&["export", "--again", "out/part_a.launch.xml"], tmp.path());
    assert!(ok, "{text}");
    assert_eq!(std::fs::read_to_string(&file).unwrap(), xml);
}

#[test]
fn self_contained_form_is_flat_and_runs() {
    let tmp = tempfile::TempDir::new().unwrap();
    let (ok, text) = play_launch(
        &[
            "export",
            &fixture(),
            "--exclude",
            "/sel/talker_b",
            "--form",
            "self-contained",
            "-o",
            "flat",
            "--name",
            "flat",
        ],
        tmp.path(),
    );
    assert!(ok, "{text}");
    let file = tmp.path().join("flat/flat.launch.xml");
    let xml = std::fs::read_to_string(&file).unwrap();
    assert!(!xml.contains("<include"), "{xml}");
    // The talker's inline parameter went into a parameter file beside it.
    assert!(xml.contains(r#"<param from="$(dirname)/params/"#), "{xml}");
    let params: String = std::fs::read_dir(tmp.path().join("flat/params"))
        .unwrap()
        .map(|e| std::fs::read_to_string(e.unwrap().path()).unwrap())
        .collect();
    assert!(params.contains("msg_prefix: pure"), "{params}");
    assert_eq!(
        members_of(&file, tmp.path()),
        [
            "/pure_test/listener",
            "/pure_test/talker",
            "/sel/listener_a",
            "/sel/listener_b",
            "/sel/sel_container",
            "/sel/talker_a"
        ]
    );
    stock_parses(&file);

    // And it runs: play_launch brings it up to startup complete.
    let (ok, text) = play_launch(
        &[
            "launch",
            "--disable-all",
            file.to_str().unwrap(),
            "--run-for",
            "4s",
        ],
        tmp.path(),
    );
    assert!(ok, "{text}");
}

#[test]
fn command_form_writes_one_node_and_refuses_several() {
    let tmp = tempfile::TempDir::new().unwrap();
    let (ok, text) = play_launch(
        &[
            "export",
            &fixture(),
            "--only",
            "/pure_test/talker",
            "--form",
            "command",
            "-o",
            "cmd",
        ],
        tmp.path(),
    );
    assert!(ok, "{text}");
    assert!(
        text.contains("ros2 run demo_nodes_cpp talker --ros-args -r __node:=talker -r __ns:=/pure_test --params-file"),
        "{text}"
    );
    let (ok, text) = play_launch(
        &[
            "export",
            &fixture(),
            "--only",
            "pure_nodes.launch.xml",
            "--form",
            "command",
            "-o",
            "cmd",
        ],
        tmp.path(),
    );
    assert!(!ok && text.contains("select exactly one"), "{text}");
}
