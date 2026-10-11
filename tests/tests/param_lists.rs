//! Issue #0067: a string parameter whose text looks like a list stays a
//! string, and a list stays a list, from the launch file to the node.
//!
//! Schema 1 carried an inline list as a `Str` of its flow text, so
//! `<param value="[a, b]" type="str"/>` arrived as the same `Str("[a, b]")` as
//! a list and was spawned as one. Schema 2 gives a list its own variant
//! (`StrList`); a schema-1 model is upgraded on read, keeping what it meant.

use std::{
    path::{Path, PathBuf},
    process::Stdio,
    time::{Duration, Instant},
};

use play_launch_tests::{fixtures, process::ManagedProcess};

const LAUNCH_XML: &str = r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t">
    <param name="as_str" value="[a, b]" type="str"/>
    <param name="as_list" value="[a, b]"/>
    <param name="ints" value="[1, 2]"/>
  </node>
</launch>
"#;

fn resolve(env: &std::collections::HashMap<String, String>, dir: &Path) -> PathBuf {
    let launch = dir.join("p.launch.xml");
    std::fs::write(&launch, LAUNCH_XML).unwrap();
    let model = dir.join("model.yaml");
    let out = fixtures::play_launch_cmd(env)
        .current_dir(dir)
        .arg("resolve")
        .arg(&launch)
        .arg("-o")
        .arg(&model)
        .output()
        .expect("run resolve");
    assert!(
        out.status.success(),
        "resolve failed:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
    model
}

/// `up` the model just long enough to write the node's params file, and return
/// its `ros__parameters` block.
fn spawned_params(
    env: &std::collections::HashMap<String, String>,
    dir: &Path,
    model: &Path,
) -> serde_yaml_ng::Mapping {
    let mut cmd = fixtures::play_launch_cmd(env);
    cmd.current_dir(dir)
        .arg("up")
        .arg(model)
        .arg("--disable-all")
        .stdout(Stdio::null())
        .stderr(Stdio::null());
    let _proc = ManagedProcess::spawn(&mut cmd).expect("spawn up");
    let params_dir = dir.join("play_log/latest/node/t/params_files");
    let start = Instant::now();
    let file = loop {
        if let Some(f) = std::fs::read_dir(&params_dir)
            .ok()
            .and_then(|mut it| it.next())
            .and_then(|e| e.ok())
        {
            break f.path();
        }
        assert!(
            start.elapsed() < Duration::from_secs(30),
            "no params file under {}",
            params_dir.display()
        );
        std::thread::sleep(Duration::from_millis(100));
    };
    let yaml: serde_yaml_ng::Value =
        serde_yaml_ng::from_str(&std::fs::read_to_string(&file).unwrap()).unwrap();
    yaml["/**"]["ros__parameters"]
        .as_mapping()
        .unwrap_or_else(|| panic!("no ros__parameters in {}", file.display()))
        .clone()
}

fn assert_spawned_apart(params: &serde_yaml_ng::Mapping) {
    let get = |k: &str| params.get(k).unwrap_or_else(|| panic!("{k}: {params:?}"));
    assert_eq!(
        get("as_str"),
        &serde_yaml_ng::Value::String("[a, b]".into()),
        "a type=\"str\" value must reach the node as a string (#0067)"
    );
    assert!(
        get("as_list").is_sequence(),
        "as_list: {:?}",
        get("as_list")
    );
    let ints = get("ints").as_sequence().expect("ints is a list");
    assert!(
        ints.iter().all(|v| v.is_i64()),
        "ints keep their type: {ints:?}"
    );
}

#[test]
fn a_string_that_looks_like_a_list_stays_a_string() {
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().unwrap();
    let model = resolve(&env, tmp.path());

    let text = std::fs::read_to_string(&model).unwrap();
    let yaml: serde_yaml_ng::Value = serde_yaml_ng::from_str(&text).unwrap();
    let params = &yaml["structure"]["nodes"]["/t"]["params"];
    assert_eq!(
        params["as_str"],
        serde_yaml_ng::Value::String("[a, b]".into())
    );
    assert!(params["as_list"].is_sequence(), "model: {text}");

    assert_spawned_apart(&spawned_params(&env, tmp.path(), &model));
}

/// A schema-1 model meant a list by flow text: `up` on one still spawns it as
/// a list, because the model crate upgrades it on read.
#[test]
fn a_schema_1_model_still_spawns_its_lists_as_lists() {
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().unwrap();
    let model = resolve(&env, tmp.path());

    // Rewrite the model as schema 1 wrote it: lists as flow-text strings,
    // and no string that looks like one (schema 1 could not carry it).
    let mut yaml: serde_yaml_ng::Value =
        serde_yaml_ng::from_str(&std::fs::read_to_string(&model).unwrap()).unwrap();
    yaml["meta"]["version"] = 1.into();
    let node = &mut yaml["structure"]["nodes"]["/t"];
    node["params"]["as_list"] = "['a', 'b']".into();
    node["params"]["ints"] = "[1, 2]".into();
    node["params"]["as_str"] = "[a, b]".into();
    if let Some(sources) = node["param_sources"].as_sequence_mut() {
        for s in sources {
            match s["name"].as_str() {
                Some("as_list") => s["value"] = "['a', 'b']".into(),
                Some("ints") => s["value"] = "[1, 2]".into(),
                _ => {}
            }
        }
    }
    let v1 = tmp.path().join("model_v1.yaml");
    std::fs::write(&v1, serde_yaml_ng::to_string(&yaml).unwrap()).unwrap();

    let params = spawned_params(&env, tmp.path(), &v1);
    let get = |k: &str| params.get(k).unwrap_or_else(|| panic!("{k}: {params:?}"));
    assert!(
        get("as_list").is_sequence(),
        "as_list: {:?}",
        get("as_list")
    );
    assert!(get("ints").is_sequence(), "ints: {:?}", get("ints"));
    // Schema 1 read `[a, b]` as a list too; the upgrade keeps that meaning.
    assert!(get("as_str").is_sequence(), "as_str: {:?}", get("as_str"));
}
