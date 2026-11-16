//! What a process starts IN and UNDER (#0085, #0087): `<unset_env>`,
//! `launch-prefix` and `cwd`, on every frontend, as `ros2 launch` resolves
//! them.
//!
//! - launch's `context.environment` starts as a copy of `os.environ`;
//!   `UnsetEnvironmentVariable` DELETES from it, so a variable the launcher's
//!   shell exported is gone for every later process, and `$(env X)` no longer
//!   sees it. A scoped group's push/pop puts it back.
//! - `prefix` on the action wins; otherwise the `launch-prefix` launch
//!   configuration, filtered by `launch-prefix-filter`
//!   (`launch.descriptions.Executable.prepare`).
//! - `cwd` is the substituted text.
//!
//! `HOME` is the inherited variable throughout: it is set in every test
//! environment, so no test has to mutate the process environment.

use play_launch_parser::parse_launch_file;
use play_launch_parser_pyexec::python_test_guard;
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

fn find<'a>(json: &'a serde_json::Value, list: &str, name: &str) -> &'a serde_json::Value {
    json[list]
        .as_array()
        .unwrap()
        .iter()
        .find(|e| e["name"] == name || e["name"] == format!("/{name}"))
        .unwrap_or_else(|| panic!("{name} missing from {list}: {json}"))
}

fn unset(entry: &serde_json::Value) -> Vec<String> {
    serde_json::from_value(entry["env_unset"].clone()).unwrap_or_default()
}

fn env_value<'a>(entry: &'a serde_json::Value, name: &str) -> Option<&'a str> {
    entry["env"]
        .as_array()?
        .iter()
        .find(|kv| kv[0] == name)
        .and_then(|kv| kv[1].as_str())
}

fn cmd(entry: &serde_json::Value) -> Vec<String> {
    serde_json::from_value(entry["cmd"].clone()).unwrap()
}

const XML: &str = r#"<launch>
  <let name="base" value="/srv"/>
  <group scoped="true">
    <unset_env name="HOME"/>
    <executable cmd="true" name="inside" cwd="$(var base)/bags"/>
    <executable cmd="true" name="home_again">
      <env name="HOME" value="/h"/>
    </executable>
    <executable cmd="echo $(env HOME nohome)" name="env_sub"/>
    <node_container pkg="rclcpp_components" exec="component_container" name="c_unset"
                    namespace="/t"/>
  </group>
  <executable cmd="true" name="after"/>
  <set_env name="PL_SET" value="1"/>
  <unset_env name="PL_SET"/>
  <executable cmd="true" name="set_then_unset"/>
  <set_env name="PATH" value="/nowhere"/>
  <unset_env name="PATH"/>
  <executable cmd="true" name="inherited_set_then_unset"/>
  <executable cmd="env" name="prefixed" launch-prefix="env -u DISPLAY"/>
  <executable cmd="echo a" name="shelled" shell="true" launch-prefix="nice -n 5"/>
  <node pkg="demo_nodes_cpp" exec="talker" name="talker" launch-prefix="env -u DISPLAY"
        cwd="/tmp"/>
  <node_container pkg="rclcpp_components" exec="component_container" name="c_prefix"
                  namespace="/t" launch-prefix="env -u DISPLAY -u LD_PRELOAD"/>
</launch>"#;

#[test]
fn xml_unset_in_a_scoped_group_is_carried_and_restored() {
    let json = parse(".launch.xml", XML);
    let inside = find(&json, "node", "inside");
    assert_eq!(unset(inside), ["HOME"], "{inside}");
    assert_eq!(inside["cwd"], "/srv/bags", "{inside}");
    // A process's own `<env>` sets the name again: it leaves the list.
    let again = find(&json, "node", "home_again");
    assert!(unset(again).is_empty(), "{again}");
    assert_eq!(env_value(again, "HOME"), Some("/h"), "{again}");
    // `$(env HOME)` no longer sees the inherited value.
    let sub = find(&json, "node", "env_sub");
    assert_eq!(cmd(sub), ["echo", "nohome"], "{sub}");
    let c = find(&json, "container", "c_unset");
    assert_eq!(unset(c), ["HOME"], "{c}");
    // The group popped it.
    let after = find(&json, "node", "after");
    assert!(unset(after).is_empty(), "{after}");
    assert!(after.get("cwd").is_none(), "{after}");
}

#[test]
fn xml_set_then_unset_is_an_unset() {
    let json = parse(".launch.xml", XML);
    // A name the launcher never had: nothing to remove, nothing set.
    let e = find(&json, "node", "set_then_unset");
    assert!(unset(e).is_empty(), "{e}");
    assert_eq!(env_value(e, "PL_SET"), None, "{e}");
    // An inherited one: removed.
    let e = find(&json, "node", "inherited_set_then_unset");
    assert_eq!(unset(e), ["PATH"], "{e}");
    assert_eq!(env_value(e, "PATH"), None, "{e}");
}

#[test]
fn xml_launch_prefix_reaches_executable_node_and_container() {
    let json = parse(".launch.xml", XML);
    assert_eq!(
        find(&json, "node", "prefixed")["launch_prefix"],
        "env -u DISPLAY"
    );
    let talker = find(&json, "node", "talker");
    assert_eq!(talker["launch_prefix"], "env -u DISPLAY", "{talker}");
    assert_eq!(talker["cwd"], "/tmp", "{talker}");
    assert_eq!(
        find(&json, "container", "c_prefix")["launch_prefix"],
        "env -u DISPLAY -u LD_PRELOAD"
    );
}

/// launch prepends the prefix before joining for the shell, so it is part of
/// the shell's command line, not a program in front of `/bin/sh`.
#[test]
fn a_shell_process_carries_its_prefix_inside_the_shell_command() {
    let json = parse(".launch.xml", XML);
    let e = find(&json, "node", "shelled");
    assert_eq!(cmd(e), ["/bin/sh", "-c", "nice -n 5 echo a"], "{e}");
    assert!(e.get("launch_prefix").is_none(), "{e}");
}

/// `launch-prefix` as a launch configuration applies to every process whose
/// command's basename `launch-prefix-filter` matches from its start.
#[test]
fn the_launch_prefix_configuration_applies_through_its_filter() {
    let json = parse(
        ".launch.xml",
        r#"<launch>
  <let name="launch-prefix" value="valgrind"/>
  <let name="launch-prefix-filter" value="tal"/>
  <executable cmd="/usr/bin/talker" name="t"/>
  <executable cmd="/usr/bin/listener" name="l"/>
  <executable cmd="/usr/bin/listener" name="own" launch-prefix="gdb --args"/>
</launch>"#,
    );
    assert_eq!(find(&json, "node", "t")["launch_prefix"], "valgrind");
    assert!(find(&json, "node", "l").get("launch_prefix").is_none());
    assert_eq!(find(&json, "node", "own")["launch_prefix"], "gdb --args");
}

#[test]
fn yaml_carries_unset_prefix_and_cwd() {
    let json = parse(
        ".launch.yaml",
        r#"launch:
- group:
    scoped: "true"
    children:
    - unset_env:
        name: HOME
    - executable:
        cmd: "true"
        name: inside
        cwd: /srv
        launch-prefix: env -u DISPLAY
- executable:
    cmd: "true"
    name: after
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: talker
    launch-prefix: nice
    cwd: /tmp
- executable:
    cmd: echo b
    name: shelled
    shell: true
"#,
    );
    let inside = find(&json, "node", "inside");
    assert_eq!(unset(inside), ["HOME"], "{inside}");
    assert_eq!(inside["cwd"], "/srv", "{inside}");
    assert_eq!(inside["launch_prefix"], "env -u DISPLAY", "{inside}");
    assert!(unset(find(&json, "node", "after")).is_empty());
    let talker = find(&json, "node", "talker");
    assert_eq!(talker["launch_prefix"], "nice", "{talker}");
    assert_eq!(talker["cwd"], "/tmp", "{talker}");
    // A native YAML bool, which the frontend used to read as false.
    assert_eq!(
        cmd(find(&json, "node", "shelled")),
        ["/bin/sh", "-c", "echo b"]
    );
}

#[test]
fn python_carries_unset_prefix_and_cwd() {
    let _guard = python_test_guard();
    let json = parse(
        ".launch.py",
        r#"
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, GroupAction, SetEnvironmentVariable,
                            UnsetEnvironmentVariable)
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node


def generate_launch_description():
    return LaunchDescription([
        GroupAction([
            UnsetEnvironmentVariable('HOME'),
            ExecuteProcess(cmd=['true'], name='inside', cwd='/srv',
                           prefix='env -u DISPLAY'),
            ExecuteProcess(cmd=['echo', EnvironmentVariable('HOME', default_value='nohome')],
                           name='env_sub'),
            ComposableNodeContainer(name='c_unset', namespace='/t',
                                    package='rclcpp_components',
                                    executable='component_container',
                                    prefix='env -u LD_PRELOAD'),
        ], scoped=True),
        ExecuteProcess(cmd=['true'], name='after'),
        SetEnvironmentVariable('PL_SET', '1'),
        UnsetEnvironmentVariable('PL_SET'),
        Node(package='demo_nodes_cpp', executable='talker', name='talker',
             prefix='nice', cwd='/tmp'),
        ExecuteProcess(cmd=['echo', 'a'], name='shelled', shell=True, prefix='nice -n 5'),
    ])
"#,
    );
    let inside = find(&json, "node", "inside");
    assert_eq!(unset(inside), ["HOME"], "{inside}");
    assert_eq!(inside["cwd"], "/srv", "{inside}");
    assert_eq!(inside["launch_prefix"], "env -u DISPLAY", "{inside}");
    assert_eq!(
        cmd(find(&json, "node", "env_sub")),
        ["echo", "nohome"],
        "EnvironmentVariable after an unset reads the default"
    );
    let c = find(&json, "container", "c_unset");
    assert_eq!(unset(c), ["HOME"], "{c}");
    assert_eq!(c["launch_prefix"], "env -u LD_PRELOAD", "{c}");
    assert!(unset(find(&json, "node", "after")).is_empty());
    let talker = find(&json, "node", "talker");
    // Set and then unset, and never inherited: nothing to remove.
    assert!(unset(talker).is_empty(), "{talker}");
    assert_eq!(talker["launch_prefix"], "nice", "{talker}");
    assert_eq!(talker["cwd"], "/tmp", "{talker}");
    let shelled = find(&json, "node", "shelled");
    assert_eq!(
        cmd(shelled),
        ["/bin/sh", "-c", "nice -n 5 echo a"],
        "{shelled}"
    );
}
