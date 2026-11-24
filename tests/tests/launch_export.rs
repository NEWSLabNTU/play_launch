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
    // What the include saw from before it is recreated in front of it.
    assert!(
        xml.contains(r#"<set_parameter name="sel_level" value="2"/>"#),
        "{xml}"
    );
    assert!(
        xml.contains(r#"<set_env name="SEL_ENV" value="on"/>"#),
        "{xml}"
    );
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

#[test]
fn command_form_loads_one_composable_into_its_container() {
    let tmp = tempfile::TempDir::new().unwrap();
    // Without its container: a load into one already running.
    let (ok, text) = play_launch(
        &[
            "export",
            &fixture(),
            "--only",
            "/sel/talker_a",
            "--no-deps",
            "--form",
            "command",
            "-o",
            "cmd",
            "--name",
            "load_only",
        ],
        tmp.path(),
    );
    assert!(ok, "{text}");
    assert!(
        text.contains(
            "ros2 component load --no-daemon /sel/sel_container composition composition::Talker \
             -n talker_a --node-namespace /sel -p sel_level:=2"
        ),
        "{text}"
    );
    let script = std::fs::read_to_string(tmp.path().join("cmd/load_only.sh")).unwrap();
    assert!(!script.contains("component_container"), "{script}");

    // With it (the closure keeps it): the script starts the container,
    // loads the composable and stays up.
    let (ok, text) = play_launch(
        &[
            "export",
            &fixture(),
            "--only",
            "/sel/talker_a",
            "--form",
            "command",
            "-o",
            "cmd",
            "--name",
            "with_container",
        ],
        tmp.path(),
    );
    assert!(ok, "{text}");
    let file = tmp.path().join("cmd/with_container.sh");
    let script = std::fs::read_to_string(&file).unwrap();
    assert!(
        script.contains("ros2 run rclcpp_components component_container"),
        "{script}"
    );

    let mut env = fixtures::install_env();
    env.insert(
        "ROS_DOMAIN_ID".into(),
        fixtures::next_domain_id().to_string(),
    );
    let mut cmd = Command::new(&file);
    cmd.envs(&env)
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null());
    let _proc = play_launch_tests::process::ManagedProcess::spawn(&mut cmd).unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
    loop {
        let out = Command::new("ros2")
            .envs(&env)
            .args(["node", "list", "--no-daemon"])
            .output()
            .unwrap();
        if String::from_utf8_lossy(&out.stdout)
            .lines()
            .any(|l| l == "/sel/talker_a")
        {
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "talker_a never loaded: {}",
            String::from_utf8_lossy(&out.stdout)
        );
        std::thread::sleep(std::time::Duration::from_millis(500));
    }

    // Two composables are refused.
    let (ok, text) = play_launch(
        &[
            "export",
            &fixture(),
            "--only",
            "/sel/talker_a",
            "--only",
            "/sel/talker_b",
            "--form",
            "command",
            "-o",
            "cmd",
        ],
        tmp.path(),
    );
    assert!(!ok && text.contains("select exactly one"), "{text}");
}
