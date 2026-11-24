//! The Launch tree's switches over the web API (phase 106 W4).
//!
//! `--start none` registers every member Excluded; `POST
//! /api/selection/apply` switches parts on and off, live or fresh; `GET
//! /api/selection` gives the command for what is on; `POST
//! /api/selection/reload` re-reads an edited launch file, and the next
//! Apply restarts the changed members with their new resolution and starts
//! the new ones.

use std::{
    path::Path,
    process::{Command, Stdio},
    time::{Duration, Instant},
};

use play_launch_tests::{fixtures, process::ManagedProcess};
use serde_json::Value;

struct Ui {
    base: String,
}

impl Ui {
    fn get(&self, path: &str) -> Value {
        let out = Command::new("curl")
            .args(["-s", "--max-time", "5", &format!("{}{path}", self.base)])
            .output()
            .unwrap();
        serde_json::from_slice(&out.stdout).unwrap_or(Value::Null)
    }

    fn post(&self, path: &str, body: &str) -> Value {
        let out = Command::new("curl")
            .args([
                "-s",
                "--max-time",
                "30",
                "-X",
                "POST",
                "-H",
                "Content-Type: application/json",
                "-H",
                &format!("Origin: {}", self.base),
                "-d",
                body,
                &format!("{}{path}", self.base),
            ])
            .output()
            .unwrap();
        serde_json::from_slice(&out.stdout)
            .unwrap_or_else(|_| panic!("{path}: {}", String::from_utf8_lossy(&out.stdout)))
    }

    /// `(id, excluded, status)` per member.
    fn members(&self) -> Vec<(String, bool, String)> {
        self.get("/api/nodes")
            .as_array()
            .cloned()
            .unwrap_or_default()
            .iter()
            .map(|n| {
                let v = &n["status"]["value"];
                let status = v
                    .as_str()
                    .map(str::to_string)
                    .unwrap_or_else(|| v["status"].as_str().unwrap_or("?").to_string());
                (
                    n["id"].as_str().unwrap().to_string(),
                    n["excluded"].as_bool().unwrap(),
                    status,
                )
            })
            .collect()
    }

    fn wait_for(&self, what: &str, f: impl Fn(&[(String, bool, String)]) -> bool) {
        let deadline = Instant::now() + Duration::from_secs(30);
        loop {
            let m = self.members();
            if !m.is_empty() && f(&m) {
                return;
            }
            assert!(Instant::now() < deadline, "{what}: {m:?}");
            std::thread::sleep(Duration::from_millis(250));
        }
    }
}

fn state<'a>(m: &'a [(String, bool, String)], id: &str) -> (&'a bool, &'a str) {
    let e = m
        .iter()
        .find(|(i, _, _)| i == id)
        .unwrap_or_else(|| panic!("{id}"));
    (&e.1, e.2.as_str())
}

fn start(launch: &Path, extra: &[&str], work: &Path) -> (Ui, ManagedProcess) {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    let port = fixtures::web_port(&cmd);
    cmd.current_dir(work);
    cmd.args([
        "launch",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--web-addr",
        &format!("127.0.0.1:{port}"),
        launch.to_str().unwrap(),
    ]);
    cmd.args(extra);
    cmd.stdout(Stdio::from(
        std::fs::File::create(work.join("stdout.log")).unwrap(),
    ));
    cmd.stderr(Stdio::null());
    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    let ui = Ui {
        base: format!("http://127.0.0.1:{port}"),
    };
    ui.wait_for("the UI never answered", |_| true);
    (ui, proc)
}

fn fixture(name: &str) -> std::path::PathBuf {
    fixtures::test_workspace_path("simple_test")
        .join("launch")
        .join(name)
}

#[test]
fn switches_start_and_stop_parts_live_and_fresh() {
    let work = tempfile::TempDir::new().unwrap();
    let (ui, _proc) = start(
        &fixture("selection.launch.xml"),
        &["--start", "none"],
        work.path(),
    );
    ui.wait_for("everything Excluded", |m| m.iter().all(|(_, ex, _)| *ex));

    // Switching on one composable brings its container (the closure).
    let check = ui.post(
        "/api/selection/check",
        r#"{"on":["composable:/sel/talker_a","node:/sel/listener_a"]}"#,
    );
    assert_eq!(
        check["added"][0]["container"], "container:/sel/sel_container",
        "{check}"
    );
    ui.post(
        "/api/selection/apply",
        r#"{"on":["composable:/sel/talker_a","node:/sel/listener_a"],"mode":"live"}"#,
    );
    ui.wait_for("part a running", |m| {
        state(m, "composable:/sel/talker_a") == (&false, "loaded")
            && state(m, "node:/sel/listener_a") == (&false, "running")
            && state(m, "container:/sel/sel_container") == (&false, "running")
    });
    let m = ui.members();
    // The other composable of the same container stays off.
    assert_eq!(state(&m, "composable:/sel/talker_b"), (&true, "unloaded"));
    assert_eq!(state(&m, "node:/pure_test/talker"), (&true, "stopped"));

    // Copy command: the stable selectors for what is on.
    let sel = ui.get("/api/selection");
    let flags: Vec<&str> = sel["flags"]
        .as_array()
        .unwrap()
        .iter()
        .map(|f| f.as_str().unwrap())
        .collect();
    assert_eq!(
        flags,
        [
            "--only node:/sel/sel_container",
            "--only 'scope:selection_part.launch.xml[part=a]'"
        ]
    );

    // Off, live: unloaded, Excluded, the rest untouched.
    ui.post(
        "/api/selection/apply",
        r#"{"off":["composable:/sel/talker_a"],"mode":"live"}"#,
    );
    ui.wait_for("talker_a off", |m| {
        state(m, "composable:/sel/talker_a") == (&true, "unloaded")
            && state(m, "node:/sel/listener_a") == (&false, "running")
    });

    // Fresh: every kept process starts again (a second `pid` file is the
    // rotated first process's).
    ui.post(
        "/api/selection/apply",
        r#"{"on":["node:/pure_test/talker"],"mode":"fresh"}"#,
    );
    ui.wait_for("fresh restart", |m| {
        state(m, "node:/pure_test/talker") == (&false, "running")
            && state(m, "node:/sel/listener_a") == (&false, "running")
    });
    let node = work.path().join("play_log/latest/node");
    let deadline = Instant::now() + Duration::from_secs(15);
    while !node.join("listener_a/pid.1").exists() {
        assert!(Instant::now() < deadline, "listener_a was not restarted");
        std::thread::sleep(Duration::from_millis(200));
    }

    // The step record has one line per Apply.
    let steps =
        std::fs::read_to_string(work.path().join("play_log/latest/selection.jsonl")).unwrap();
    let kinds: Vec<String> = steps
        .lines()
        .map(|l| {
            serde_json::from_str::<Value>(l).unwrap()["step"]
                .as_str()
                .unwrap()
                .to_string()
        })
        .collect();
    assert_eq!(kinds, ["apply_live", "apply_live", "apply_fresh"]);
    // Each carries the resources in use before it (null here: this run has
    // monitoring off).
    assert!(
        steps.lines().all(
            |l| serde_json::from_str::<Value>(l).unwrap()["before"].is_null()
                && l.contains("\"before\"")
        ),
        "{steps}"
    );

    // Switching a container off under a composable that stays on is refused.
    let r = ui.post(
        "/api/selection/check",
        r#"{"on":["composable:/sel/talker_b"],"off":["container:/sel/sel_container"]}"#,
    );
    assert!(
        r["error"]
            .as_str()
            .unwrap_or("")
            .contains("needs its container"),
        "{r}"
    );

    // Export what is on, into the run's own log directory, never a path the
    // browser names; the zip carries the launch file and its parameters.
    let r = ui.post(
        "/api/selection/export",
        r#"{"form":"self-contained","name":"bench"}"#,
    );
    let file = r["file"].as_str().unwrap_or_else(|| panic!("{r}"));
    assert!(file.contains("/export/bench/bench.launch.xml"), "{r}");
    let xml = std::fs::read_to_string(file).unwrap();
    assert!(
        xml.contains(r#"name="listener_a""#) && !xml.contains(r#"name="talker_a""#),
        "{xml}"
    );
    let r = ui.post(
        "/api/selection/export",
        r#"{"form":"include","name":"../x"}"#,
    );
    assert!(
        r["error"].is_string(),
        "a name with a path in it is refused: {r}"
    );
    let zip = work.path().join("bench.zip");
    let ok = Command::new("curl")
        .args([
            "-s",
            "-o",
            zip.to_str().unwrap(),
            &format!("{}/api/selection/export/bench.zip", ui.base),
        ])
        .status()
        .unwrap()
        .success();
    assert!(ok);
    let bytes = std::fs::read(&zip).unwrap();
    assert!(bytes.starts_with(b"PK\x03\x04"), "not a zip");
}

#[test]
fn reload_marks_a_changed_node_and_apply_restarts_it_with_the_new_resolution() {
    let work = tempfile::TempDir::new().unwrap();
    let fx = work.path().join("fx");
    std::fs::create_dir(&fx).unwrap();
    for f in [
        "selection.launch.xml",
        "selection_part.launch.xml",
        "pure_nodes.launch.xml",
    ] {
        std::fs::copy(fixture(f), fx.join(f)).unwrap();
    }
    let (ui, _proc) = start(
        &fx.join("selection.launch.xml"),
        &["--exclude", "pure_nodes.launch.xml"],
        work.path(),
    );
    ui.wait_for("part a running", |m| {
        state(m, "node:/sel/listener_a") == (&false, "running")
    });

    let part = fx.join("selection_part.launch.xml");
    let text = std::fs::read_to_string(&part).unwrap();
    std::fs::write(
        &part,
        text.replace(
            r#"name="listener_$(var part)" namespace="/sel"/>"#,
            r#"name="listener_$(var part)" namespace="/sel"><param name="probe" value="reloaded"/></node>"#,
        ),
    )
    .unwrap();
    let r = ui.post("/api/selection/reload", "{}");
    let mut changed: Vec<&str> = r["changed"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_str().unwrap())
        .collect();
    changed.sort();
    assert_eq!(
        changed,
        ["node:/sel/listener_a", "node:/sel/listener_b"],
        "{r}"
    );

    let r = ui.post(
        "/api/selection/apply",
        r#"{"on":["node:/sel/listener_a"],"mode":"live"}"#,
    );
    assert_eq!(r["restart"][0], "node:/sel/listener_a", "{r}");
    let dir = work.path().join("play_log/latest/node/listener_a");
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        let params: String = std::fs::read_dir(dir.join("params_files"))
            .unwrap()
            .filter_map(|e| std::fs::read_to_string(e.unwrap().path()).ok())
            .collect();
        if dir.join("pid.1").exists() && params.contains("probe: reloaded") {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "not restarted with the new parameter"
        );
        std::thread::sleep(Duration::from_millis(200));
    }
    // listener_b was not asked for; it stays marked changed.
    let sel = ui.get("/api/selection");
    assert_eq!(
        sel["changed"],
        serde_json::json!(["node:/sel/listener_b"]),
        "{sel}"
    );
}

#[test]
fn reload_adds_new_members_off_and_apply_starts_them() {
    let work = tempfile::TempDir::new().unwrap();
    let fx = work.path().join("fx");
    std::fs::create_dir(&fx).unwrap();
    for f in [
        "selection.launch.xml",
        "selection_part.launch.xml",
        "pure_nodes.launch.xml",
    ] {
        std::fs::copy(fixture(f), fx.join(f)).unwrap();
    }
    let (ui, _proc) = start(
        &fx.join("selection.launch.xml"),
        &["--exclude", "pure_nodes.launch.xml"],
        work.path(),
    );
    ui.wait_for("parts a and b running", |m| {
        state(m, "composable:/sel/talker_b") == (&false, "loaded")
            && state(m, "node:/sel/listener_a") == (&false, "running")
    });

    // A third part (a node, and a composable for the running container) and
    // a container of its own with a composable.
    let main = fx.join("selection.launch.xml");
    let text = std::fs::read_to_string(&main).unwrap();
    std::fs::write(
        &main,
        text.replace(
            "</launch>",
            r#"  <include file="$(dirname)/selection_part.launch.xml">
    <arg name="part" value="c"/>
  </include>
  <node_container pkg="rclcpp_components" exec="component_container" name="new_container" namespace="/sel2">
    <composable_node pkg="composition" plugin="composition::Listener" name="new_listener" namespace="/sel2"/>
  </node_container>
</launch>"#,
        ),
    )
    .unwrap();
    let new_ids = [
        "composable:/sel/talker_c",
        "composable:/sel2/new_listener",
        "container:/sel2/new_container",
        "node:/sel/listener_c",
    ];
    let r = ui.post("/api/selection/reload", "{}");
    let sorted = |v: &Value| {
        let mut ids: Vec<String> = v
            .as_array()
            .unwrap_or_else(|| panic!("{r}"))
            .iter()
            .map(|v| v.as_str().unwrap().to_string())
            .collect();
        ids.sort();
        ids
    };
    assert_eq!(sorted(&r["added"]), new_ids, "{r}");
    // The selection was "everything but pure_nodes", which covers them.
    assert_eq!(sorted(&r["covered"]), new_ids, "{r}");
    assert_eq!(r["changed"], serde_json::json!([]), "{r}");

    // They joined the run off, and the Launch tree shows them.
    let m = ui.members();
    assert_eq!(state(&m, "node:/sel/listener_c"), (&true, "stopped"));
    assert_eq!(
        state(&m, "container:/sel2/new_container"),
        (&true, "stopped")
    );
    assert_eq!(state(&m, "composable:/sel/talker_c"), (&true, "unloaded"));
    let tree = ui.get("/api/launch-tree");
    for id in new_ids {
        assert!(tree["node_scopes"][id].is_u64(), "{id} not in the tree");
    }
    assert!(
        tree["node_scopes"]["node:/sel/listener_a"].is_u64(),
        "a launch member left the tree"
    );

    let body = serde_json::json!({ "on": new_ids, "mode": "live" }).to_string();
    let r = ui.post("/api/selection/apply", &body);
    assert_eq!(sorted(&r["start"]), new_ids, "{r}");
    ui.wait_for("the new members running", |m| {
        state(m, "node:/sel/listener_c") == (&false, "running")
            && state(m, "composable:/sel/talker_c") == (&false, "loaded")
            && state(m, "container:/sel2/new_container") == (&false, "running")
            && state(m, "composable:/sel2/new_listener") == (&false, "loaded")
            && state(m, "node:/sel/listener_a") == (&false, "running")
    });
    let node = work.path().join("play_log/latest/node");
    assert!(node.join("listener_c/pid").exists());
    assert!(node.join("new_container/out").exists());

    // The step record has the reload with what it added.
    let steps =
        std::fs::read_to_string(work.path().join("play_log/latest/selection.jsonl")).unwrap();
    assert!(
        steps
            .lines()
            .any(|l| l.contains(r#""step":"reload""#) && l.contains("listener_c")),
        "{steps}"
    );
}

/// The Launch tree places an un-named node (#0017: its model key and
/// member id carry an ordinal its FQN does not), or the page cannot show or
/// switch it under its file while `--only <file>` keeps it.
#[test]
fn the_launch_tree_places_an_unnamed_node() {
    let work = tempfile::TempDir::new().unwrap();
    let (ui, _proc) = start(
        &fixture("unnamed_node.launch.xml"),
        &["--start", "none"],
        work.path(),
    );
    let tree = ui.get("/api/launch-tree");
    for id in [
        "node:/identity_test/talker-1",
        "node:/identity_test/listener",
    ] {
        assert!(
            tree["node_scopes"][id].is_u64(),
            "{id} not in the tree: {tree}"
        );
    }
}

#[test]
fn restart_with_runs_a_node_under_a_prefix() {
    let work = tempfile::TempDir::new().unwrap();
    let (ui, _proc) = start(&fixture("selection.launch.xml"), &[], work.path());
    ui.wait_for("listener_a running", |m| {
        state(m, "node:/sel/listener_a") == (&false, "running")
    });

    let r = ui.post(
        "/api/nodes/node:%2Fsel%2Flistener_a/restart-with",
        r#"{"prefix":"nice -n 7"}"#,
    );
    assert_eq!(r["restarted"], "node:/sel/listener_a", "{r}");
    let dir = work.path().join("play_log/latest/node/listener_a");
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        let cmdline = std::fs::read_to_string(dir.join("cmdline")).unwrap_or_default();
        if dir.join("pid.1").exists() && cmdline.starts_with("nice -n 7 ") {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "not restarted under the prefix: {cmdline}"
        );
        std::thread::sleep(Duration::from_millis(200));
    }

    // A composable shares its container's process: the prefix goes there.
    let r = ui.post(
        "/api/nodes/composable:%2Fsel%2Ftalker_a/restart-with",
        r#"{"prefix":"nice -n 3"}"#,
    );
    assert_eq!(r["restarted"], "container:/sel/sel_container", "{r}");

    // An empty prefix goes back to the launch file's own.
    let r = ui.post("/api/nodes/node:%2Fsel%2Flistener_a/restart-with", "{}");
    assert!(r["prefix"].is_null(), "{r}");
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        let cmdline = std::fs::read_to_string(dir.join("cmdline")).unwrap_or_default();
        if dir.join("pid.2").exists() && !cmdline.starts_with("nice") {
            break;
        }
        assert!(Instant::now() < deadline, "prefix not cleared: {cmdline}");
        std::thread::sleep(Duration::from_millis(200));
    }
}
