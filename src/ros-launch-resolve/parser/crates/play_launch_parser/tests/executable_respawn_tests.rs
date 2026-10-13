//! `respawn` / `respawn_delay` on `<executable>`, XML and YAML.
//!
//! Stock `ExecuteProcess` takes both, and `launch_xml`/`launch_yaml` read them
//! for `<executable>` as for `<node>`. The Rust parser had them on the
//! attribute allowlist and dropped them, so an `<executable respawn="true">`
//! that died stayed dead (found by play_launch's crash-loop test, W3 of
//! `docs/design/health-page.md`).

use play_launch_parser::parse_launch_file;
use std::{collections::HashMap, io::Write};
use tempfile::NamedTempFile;

fn parse(suffix: &str, body: &str) -> serde_json::Value {
    let mut file = NamedTempFile::with_suffix(suffix).unwrap();
    file.write_all(body.as_bytes()).unwrap();
    file.flush().unwrap();
    let dump = parse_launch_file(file.path(), HashMap::new())
        .unwrap_or_else(|e| panic!("{suffix} should parse: {e}"));
    serde_json::to_value(dump).unwrap()
}

fn by_name<'a>(json: &'a serde_json::Value, name: &str) -> &'a serde_json::Value {
    json["node"]
        .as_array()
        .unwrap()
        .iter()
        .find(|e| e["name"] == name)
        .unwrap_or_else(|| panic!("{name} missing: {json}"))
}

#[test]
fn xml_executable_carries_respawn() {
    let json = parse(
        ".launch.xml",
        r#"<launch>
  <arg name="again" default="true"/>
  <executable cmd="sleep 1" name="respawned" respawn="$(var again)" respawn_delay="0.5"/>
  <executable cmd="sleep 1" name="plain"/>
</launch>"#,
    );
    let r = by_name(&json, "respawned");
    assert_eq!(r["respawn"], true, "{r}");
    assert_eq!(r["respawn_delay"], 0.5, "{r}");
    let p = by_name(&json, "plain");
    assert!(p["respawn"].is_null(), "absent stays absent: {p}");
}

#[test]
fn yaml_executable_carries_respawn() {
    let json = parse(
        ".launch.yaml",
        r#"launch:
- executable:
    cmd: sleep 1
    name: respawned
    respawn: "true"
    respawn_delay: "2.0"
"#,
    );
    let r = by_name(&json, "respawned");
    assert_eq!(r["respawn"], true, "{r}");
    assert_eq!(r["respawn_delay"], 2.0, "{r}");
}
