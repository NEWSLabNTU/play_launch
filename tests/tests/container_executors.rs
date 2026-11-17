//! #0082: `--container-mode observable` keeps the stock container's executor
//! model.
//!
//! Stock `component_container_isolated` gives each component an executor and a
//! thread of its own; `component_container` runs all of them on one. Under
//! `observable` both used to become `play_launch_container` with one
//! executor, so the isolated container's components shared a thread, and on
//! the golf cart a starved NVENC encoder beside a ZED X emitted each frame
//! twice.
//!
//! Stock is the oracle: the number of threads an isolated container holds
//! beyond a plain one with the same components must be the same under
//! `observable` as under `stock`.

use std::{
    process::Stdio,
    time::{Duration, Instant},
};

use play_launch_tests::{fixtures, process::ManagedProcess};

const COMPONENTS: usize = 3;

fn launch_file(dir: &std::path::Path) -> std::path::PathBuf {
    let mut xml = String::from(
        "<launch>\n  <arg name=\"exec\"/>\n  \
         <node_container pkg=\"rclcpp_components\" exec=\"$(var exec)\" name=\"exec_container\" \
         namespace=\"\">\n",
    );
    for i in 0..COMPONENTS {
        xml.push_str(&format!(
            "    <composable_node pkg=\"demo_nodes_cpp\" plugin=\"demo_nodes_cpp::Talker\" \
             name=\"talker_{i}\"/>\n"
        ));
    }
    xml.push_str("  </node_container>\n</launch>\n");
    let path = dir.join("executors.launch.xml");
    std::fs::write(&path, xml).unwrap();
    path
}

/// Threads of the running container, once every component has loaded.
fn container_threads(mode: &str, exec: &str) -> usize {
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let dir = tmp.path();
    let launch = launch_file(dir);
    let stdout = dir.join("stdout.log");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--container-mode",
        mode,
        launch.to_str().unwrap(),
        &format!("exec:={exec}"),
    ]);
    cmd.stdout(Stdio::from(std::fs::File::create(&stdout).unwrap()));
    cmd.stderr(Stdio::from(
        std::fs::File::create(dir.join("stderr.log")).unwrap(),
    ));
    let _proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");

    let start = Instant::now();
    loop {
        let text = std::fs::read_to_string(&stdout).unwrap_or_default();
        if text.contains("Startup complete: all nodes ready") {
            break;
        }
        assert!(
            !text.contains("Startup complete with failures"),
            "{mode}/{exec}: a component failed:\n{text}"
        );
        assert!(
            start.elapsed() < Duration::from_secs(60),
            "{mode}/{exec}: startup did not complete:\n{text}"
        );
        std::thread::sleep(Duration::from_millis(200));
    }
    // Let each dedicated executor's thread start.
    std::thread::sleep(Duration::from_secs(1));
    // The container is the one process whose command line names this
    // launch's container node and is a descendant of play_launch.
    let pid = container_pid(_proc.id()).unwrap_or_else(|| panic!("{mode}/{exec}: no container"));
    std::fs::read_dir(format!("/proc/{pid}/task"))
        .expect("container alive")
        .count()
}

/// The descendant of `root` whose command line remaps the node name to
/// `exec_container`.
fn container_pid(root: u32) -> Option<u32> {
    let parent = |pid: u32| -> Option<u32> {
        let stat = std::fs::read_to_string(format!("/proc/{pid}/stat")).ok()?;
        let after = &stat[stat.rfind(')')? + 2..];
        after.split_whitespace().nth(1)?.parse().ok()
    };
    for entry in std::fs::read_dir("/proc").ok()? {
        let Some(pid) = entry
            .ok()
            .and_then(|e| e.file_name().to_str().and_then(|n| n.parse::<u32>().ok()))
        else {
            continue;
        };
        let cmdline = std::fs::read(format!("/proc/{pid}/cmdline")).unwrap_or_default();
        if !String::from_utf8_lossy(&cmdline).contains("__node:=exec_container") {
            continue;
        }
        let mut p = pid;
        while let Some(pp) = parent(p) {
            if pp == root {
                return Some(pid);
            }
            if pp <= 1 {
                break;
            }
            p = pp;
        }
    }
    None
}

#[test]
fn observable_keeps_component_container_isolateds_thread_per_component() {
    let stock = container_threads("stock", "component_container_isolated") as i64
        - container_threads("stock", "component_container") as i64;
    let observable = container_threads("observable", "component_container_isolated") as i64
        - container_threads("observable", "component_container") as i64;
    assert!(
        stock >= COMPONENTS as i64,
        "stock gives each of {COMPONENTS} components a thread, measured {stock} extra"
    );
    assert_eq!(
        observable, stock,
        "extra threads for an isolated container: observable {observable}, stock {stock}"
    );
}
