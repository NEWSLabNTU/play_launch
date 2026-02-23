//! Phase 22.8: XML/YAML fixture round-trip tests.
//!
//! For each test: parse XML → IR → compile to WASM → execute → compare with parse_launch_file().
//! Validates that the WASM pipeline produces identical output to direct evaluation.

use play_launch_parser::record::{LoadNodeRecord, NodeRecord, RecordJson};
use play_launch_wasm_codegen::compile_to_wasm;
use play_launch_wasm_runtime::execute_wasm;
use std::collections::HashMap;
use std::io::Write;
use tempfile::NamedTempFile;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn write_xml(content: &str) -> NamedTempFile {
    let mut f = NamedTempFile::with_suffix(".launch.xml").unwrap();
    f.write_all(content.as_bytes()).unwrap();
    f.flush().unwrap();
    f
}

/// Run a launch file through both pipelines and return (direct, wasm) RecordJson.
fn both_pipelines(
    path: &std::path::Path,
    args: HashMap<String, String>,
) -> (RecordJson, RecordJson) {
    // Direct: parse_launch_file
    let direct = play_launch_parser::parse_launch_file(path, args.clone())
        .expect("parse_launch_file failed");

    // WASM: analyze → compile → execute
    let program = play_launch_parser::analyze_launch_file_with_args(path, args.clone())
        .expect("analyze_launch_file_with_args failed");
    let wasm_bytes = compile_to_wasm(&program).expect("compile_to_wasm failed");
    let wasm = execute_wasm(&wasm_bytes, args).expect("execute_wasm failed");

    (direct, wasm)
}

/// Sort env/params vecs for stable comparison (HashMap iteration order varies).
fn sorted_pairs(pairs: &[(String, String)]) -> Vec<(String, String)> {
    let mut v = pairs.to_vec();
    v.sort();
    v
}

fn sorted_opt_pairs(pairs: &Option<Vec<(String, String)>>) -> Option<Vec<(String, String)>> {
    pairs.as_ref().map(|p| sorted_pairs(p))
}

/// Compare two NodeRecord slices field by field.
fn assert_nodes_eq(label: &str, direct: &[NodeRecord], wasm: &[NodeRecord]) {
    assert_eq!(
        direct.len(),
        wasm.len(),
        "{label}: node count mismatch (direct={}, wasm={})",
        direct.len(),
        wasm.len()
    );

    for (i, (d, w)) in direct.iter().zip(wasm.iter()).enumerate() {
        let ctx = format!("{label} node[{i}] ({})", d.executable);
        assert_eq!(d.executable, w.executable, "{ctx}: executable");
        assert_eq!(d.name, w.name, "{ctx}: name");
        assert_eq!(d.package, w.package, "{ctx}: package");
        assert_eq!(d.namespace, w.namespace, "{ctx}: namespace");
        assert_eq!(d.cmd, w.cmd, "{ctx}: cmd");
        assert_eq!(sorted_pairs(&d.params), sorted_pairs(&w.params), "{ctx}: params");
        assert_eq!(
            sorted_opt_pairs(&d.global_params),
            sorted_opt_pairs(&w.global_params),
            "{ctx}: global_params"
        );
        assert_eq!(sorted_pairs(&d.remaps), sorted_pairs(&w.remaps), "{ctx}: remaps");
        assert_eq!(sorted_opt_pairs(&d.env), sorted_opt_pairs(&w.env), "{ctx}: env");
        assert_eq!(d.respawn, w.respawn, "{ctx}: respawn");
        assert_eq!(d.respawn_delay, w.respawn_delay, "{ctx}: respawn_delay");
    }
}

/// Sort load_node records by node_name for stable comparison.
fn sorted_load_nodes(nodes: &[LoadNodeRecord]) -> Vec<LoadNodeRecord> {
    let mut v = nodes.to_vec();
    v.sort_by(|a, b| a.node_name.cmp(&b.node_name));
    v
}

/// Compare two LoadNodeRecord slices (sorted by name) field by field.
fn assert_load_nodes_eq(label: &str, direct: &[LoadNodeRecord], wasm: &[LoadNodeRecord]) {
    let direct = sorted_load_nodes(direct);
    let wasm = sorted_load_nodes(wasm);
    assert_eq!(
        direct.len(),
        wasm.len(),
        "{label}: load_node count mismatch"
    );
    for (i, (d, w)) in direct.iter().zip(wasm.iter()).enumerate() {
        let ctx = format!("{label} load_node[{i}] ({})", d.node_name);
        assert_eq!(d.node_name, w.node_name, "{ctx}: node_name");
        assert_eq!(
            d.target_container_name, w.target_container_name,
            "{ctx}: target_container_name"
        );
        assert_eq!(d.plugin, w.plugin, "{ctx}: plugin");
        assert_eq!(d.package, w.package, "{ctx}: package");
        assert_eq!(d.namespace, w.namespace, "{ctx}: namespace");
        assert_eq!(
            sorted_pairs(&d.params),
            sorted_pairs(&w.params),
            "{ctx}: params"
        );
        assert_eq!(
            sorted_pairs(&d.remaps),
            sorted_pairs(&w.remaps),
            "{ctx}: remaps"
        );
    }
}

// ---------------------------------------------------------------------------
// Round-trip tests: inline XML → both pipelines → compare
// ---------------------------------------------------------------------------

/// Basic args, var substitution, params, remaps, node env.
#[test]
fn test_round_trip_args() {
    let file = write_xml(
        r#"<launch>
            <arg name="node_name" default="my_talker" />
            <arg name="use_sim_time" default="false" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var node_name)" output="screen">
                <param name="use_sim_time" value="$(var use_sim_time)" />
                <remap from="chatter" to="/my_topic" />
                <env name="MY_ENV_VAR" value="test_value" />
            </node>
        </launch>"#,
    );

    let (direct, wasm) = both_pipelines(file.path(), HashMap::new());
    assert_nodes_eq("args", &direct.node, &wasm.node);
}

/// Args with CLI override.
#[test]
fn test_round_trip_args_override() {
    let file = write_xml(
        r#"<launch>
            <arg name="node_name" default="my_talker" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var node_name)" />
        </launch>"#,
    );

    let mut args = HashMap::new();
    args.insert("node_name".to_string(), "custom_name".to_string());

    let (direct, wasm) = both_pipelines(file.path(), args);
    assert_nodes_eq("args_override", &direct.node, &wasm.node);
}

/// If/unless conditions with default and overridden args.
#[test]
fn test_round_trip_conditions() {
    let file = write_xml(
        r#"<launch>
            <arg name="use_sim" default="true" />
            <arg name="use_rviz" default="false" />
            <node pkg="demo_nodes_cpp" exec="talker" name="sim_talker" if="$(var use_sim)" />
            <node pkg="demo_nodes_cpp" exec="listener" name="rviz_listener" if="$(var use_rviz)" />
            <node pkg="demo_nodes_cpp" exec="listener" name="normal_listener" unless="$(var use_rviz)" />
            <node pkg="demo_nodes_cpp" exec="talker" name="real_talker" unless="$(var use_sim)" />
        </launch>"#,
    );

    let (direct, wasm) = both_pipelines(file.path(), HashMap::new());
    assert_nodes_eq("conditions", &direct.node, &wasm.node);
    // With defaults: sim_talker (if true) + normal_listener (unless false) = 2 nodes
    assert_eq!(direct.node.len(), 2);
}

/// Conditions with CLI override flipping branches.
#[test]
fn test_round_trip_conditions_override() {
    let file = write_xml(
        r#"<launch>
            <arg name="use_sim" default="true" />
            <node pkg="demo_nodes_cpp" exec="talker" name="sim_node" if="$(var use_sim)" />
            <node pkg="demo_nodes_cpp" exec="listener" name="real_node" unless="$(var use_sim)" />
        </launch>"#,
    );

    let mut args = HashMap::new();
    args.insert("use_sim".to_string(), "false".to_string());

    let (direct, wasm) = both_pipelines(file.path(), args);
    assert_nodes_eq("conditions_override", &direct.node, &wasm.node);
    assert_eq!(direct.node.len(), 1);
    assert_eq!(direct.node[0].name, Some("real_node".to_string()));
}

/// XML include with relative path and arg override.
#[test]
fn test_round_trip_include() {
    // Create inner file
    let mut inner = NamedTempFile::with_suffix(".launch.xml").unwrap();
    inner
        .write_all(
            br#"<launch>
                <arg name="node_name" default="default_node" />
                <node pkg="demo_nodes_cpp" exec="talker" name="$(var node_name)" />
            </launch>"#,
        )
        .unwrap();
    inner.flush().unwrap();
    let inner_path = inner.path().to_str().unwrap().to_string();

    let xml = format!(
        r#"<launch>
            <include file="{inner_path}" />
            <include file="{inner_path}">
                <arg name="node_name" value="custom_talker" />
            </include>
            <node pkg="demo_nodes_cpp" exec="listener" name="local_listener" />
        </launch>"#
    );
    let outer = write_xml(&xml);

    let (direct, wasm) = both_pipelines(outer.path(), HashMap::new());
    assert_nodes_eq("include", &direct.node, &wasm.node);
    assert_eq!(direct.node.len(), 3);
}

/// Sequential let variable re-assignment.
#[test]
fn test_round_trip_let_ordering() {
    let file = write_xml(
        r#"<launch>
            <arg name="test_var" default="initial_value"/>
            <node pkg="demo_nodes_cpp" exec="talker" name="node1">
                <param name="before_let" value="$(var test_var)"/>
            </node>
            <let name="test_var" value="changed_value"/>
            <node pkg="demo_nodes_cpp" exec="talker" name="node2">
                <param name="after_let" value="$(var test_var)"/>
            </node>
            <let name="test_var" value="final_value"/>
            <node pkg="demo_nodes_cpp" exec="talker" name="node3">
                <param name="after_second_let" value="$(var test_var)"/>
            </node>
        </launch>"#,
    );

    let (direct, wasm) = both_pipelines(file.path(), HashMap::new());
    assert_nodes_eq("let_ordering", &direct.node, &wasm.node);
    assert_eq!(direct.node.len(), 3);
}

/// Group namespace + push-ros-namespace.
#[test]
fn test_round_trip_group_namespace() {
    let file = write_xml(
        r#"<launch>
            <group ns="robot1">
                <node pkg="demo_nodes_cpp" exec="talker" name="talker1" />
            </group>
            <group ns="robot2">
                <node pkg="demo_nodes_cpp" exec="listener" name="listener1" />
            </group>
        </launch>"#,
    );

    let (direct, wasm) = both_pipelines(file.path(), HashMap::new());
    assert_nodes_eq("group_namespace", &direct.node, &wasm.node);
    assert_eq!(direct.node[0].namespace, Some("/robot1".to_string()));
    assert_eq!(direct.node[1].namespace, Some("/robot2".to_string()));
}

/// Nested groups with push-ros-namespace.
#[test]
fn test_round_trip_nested_namespace() {
    let file = write_xml(
        r#"<launch>
            <group>
                <push-ros-namespace namespace="planning"/>
                <group>
                    <push-ros-namespace namespace="scenario"/>
                    <node pkg="demo_nodes_cpp" exec="talker" name="planner" />
                </group>
            </group>
            <node pkg="demo_nodes_cpp" exec="listener" name="outside" />
        </launch>"#,
    );

    let (direct, wasm) = both_pipelines(file.path(), HashMap::new());
    assert_nodes_eq("nested_namespace", &direct.node, &wasm.node);
}

/// Global set_env, set_parameter, and set_remap.
#[test]
fn test_round_trip_global_context() {
    let file = write_xml(
        r#"<launch>
            <arg name="use_sim_time" default="true" />
            <set_env name="ROS_DOMAIN_ID" value="42" />
            <set_parameter name="use_sim_time" value="$(var use_sim_time)" />
            <node pkg="demo_nodes_cpp" exec="talker" name="talker1" />
        </launch>"#,
    );

    let (direct, wasm) = both_pipelines(file.path(), HashMap::new());
    assert_nodes_eq("global_context", &direct.node, &wasm.node);
}

/// Unset env: set then unset.
#[test]
fn test_round_trip_unset_env() {
    let file = write_xml(
        r#"<launch>
            <set_env name="VAR1" value="val1" />
            <set_env name="VAR2" value="val2" />
            <unset_env name="VAR1" />
            <node pkg="demo_nodes_cpp" exec="talker" name="node1" />
        </launch>"#,
    );

    let (direct, wasm) = both_pipelines(file.path(), HashMap::new());
    assert_nodes_eq("unset_env", &direct.node, &wasm.node);
}

/// Container with composable nodes.
#[test]
fn test_round_trip_container() {
    let file = write_xml(
        r#"<launch>
            <node_container pkg="rclcpp_components" exec="component_container" name="my_container" namespace="/test">
                <composable_node pkg="composition" plugin="composition::Talker" name="talker" />
                <composable_node pkg="composition" plugin="composition::Listener" name="listener" />
            </node_container>
        </launch>"#,
    );

    let (direct, wasm) = both_pipelines(file.path(), HashMap::new());
    assert_eq!(
        direct.container.len(),
        wasm.container.len(),
        "container count"
    );
    assert_eq!(
        direct.load_node.len(),
        wasm.load_node.len(),
        "load_node count"
    );

    // Compare container records
    for (d, w) in direct.container.iter().zip(wasm.container.iter()) {
        assert_eq!(d.name, w.name, "container name");
        assert_eq!(d.namespace, w.namespace, "container namespace");
        assert_eq!(d.package, w.package, "container package");
        assert_eq!(d.executable, w.executable, "container executable");
        assert_eq!(d.cmd, w.cmd, "container cmd");
    }

    assert_load_nodes_eq("container", &direct.load_node, &wasm.load_node);
}

/// Container with push-ros-namespace + composable nodes.
#[test]
fn test_round_trip_container_with_namespace() {
    let file = write_xml(
        r#"<launch>
            <group>
                <push-ros-namespace namespace="planning"/>
                <node_container pkg="rclcpp_components" exec="component_container"
                                name="planning_container" namespace="">
                    <composable_node pkg="demo_nodes_cpp" plugin="demo_nodes_cpp::Talker"
                                     name="talker" />
                    <composable_node pkg="demo_nodes_cpp" plugin="demo_nodes_cpp::Listener"
                                     name="listener" />
                </node_container>
            </group>
        </launch>"#,
    );

    let (direct, wasm) = both_pipelines(file.path(), HashMap::new());
    assert_eq!(direct.container.len(), wasm.container.len());

    for (d, w) in direct.container.iter().zip(wasm.container.iter()) {
        assert_eq!(d.name, w.name, "container name");
        assert_eq!(d.namespace, w.namespace, "container namespace");
        assert_eq!(d.cmd, w.cmd, "container cmd");
    }

    assert_load_nodes_eq("container_with_namespace", &direct.load_node, &wasm.load_node);
}

/// Load composable node into existing container.
#[test]
fn test_round_trip_load_composable_node() {
    let file = write_xml(
        r#"<launch>
            <arg name="container_name" default="my_container"/>
            <node_container name="$(var container_name)" namespace="/test">
                <composable_node pkg="initial_pkg" plugin="initial_plugin" name="initial_node"/>
            </node_container>
            <load_composable_node target="/test/$(var container_name)">
                <composable_node pkg="dynamic_pkg" plugin="DynamicPlugin" name="dynamic_node">
                    <param name="param1" value="value1"/>
                    <remap from="input" to="/test/input"/>
                </composable_node>
            </load_composable_node>
        </launch>"#,
    );

    let (direct, wasm) = both_pipelines(file.path(), HashMap::new());
    assert_eq!(direct.container.len(), wasm.container.len());
    assert_load_nodes_eq("load_composable_node", &direct.load_node, &wasm.load_node);
}

/// Executable with arguments.
#[test]
fn test_round_trip_executable() {
    let file = write_xml(
        r#"<launch>
            <executable cmd="ros2">
                <arg value="bag" />
                <arg value="play" />
                <arg value="/opt/bags/test.bag" />
            </executable>
        </launch>"#,
    );

    let (direct, wasm) = both_pipelines(file.path(), HashMap::new());
    assert_nodes_eq("executable", &direct.node, &wasm.node);
    assert_eq!(direct.node.len(), 1);
}

/// Complex multi-feature scenario (simplified from test_complex_nested).
#[test]
fn test_round_trip_complex() {
    let file = write_xml(
        r#"<launch>
            <arg name="robot_name" default="robot1" />
            <arg name="use_sim_time" default="true" />
            <arg name="use_rviz" default="false" />
            <set_env name="ROS_DOMAIN_ID" value="42" />
            <set_parameter name="use_sim_time" value="$(var use_sim_time)" />
            <group ns="$(var robot_name)">
                <push-ros-namespace ns="sensors" />
                <group>
                    <push-ros-namespace ns="camera" />
                    <node pkg="camera_driver" exec="camera_node" name="front_camera">
                        <param name="frame_id" value="camera_link" />
                        <param name="fps" value="30" />
                        <remap from="image_raw" to="front/image_raw" />
                    </node>
                    <pop-ros-namespace />
                </group>
                <group>
                    <push-ros-namespace ns="lidar" />
                    <node pkg="lidar_driver" exec="lidar_node" name="front_lidar"
                          if="$(var use_sim_time)">
                        <param name="frame_id" value="lidar_link" />
                    </node>
                    <pop-ros-namespace />
                </group>
                <pop-ros-namespace />
                <group>
                    <push-ros-namespace ns="navigation" />
                    <node pkg="nav2_map_server" exec="map_server" name="map_server"
                          unless="$(var use_sim_time)" />
                    <node pkg="nav2_amcl" exec="amcl" name="amcl">
                        <param name="min_particles" value="500" />
                        <remap from="scan" to="/$(var robot_name)/sensors/lidar/scan" />
                    </node>
                    <pop-ros-namespace />
                </group>
            </group>
            <group if="$(var use_rviz)">
                <node pkg="rviz2" exec="rviz2" name="rviz2" />
            </group>
            <executable cmd="ros2" if="$(var use_sim_time)">
                <arg value="bag" />
                <arg value="play" />
            </executable>
        </launch>"#,
    );

    let (direct, wasm) = both_pipelines(file.path(), HashMap::new());
    assert_nodes_eq("complex", &direct.node, &wasm.node);
}

/// Multi-feature with CLI arg override changing branches.
#[test]
fn test_round_trip_complex_override() {
    let file = write_xml(
        r#"<launch>
            <arg name="mode" default="sim" />
            <set_parameter name="use_sim_time" value="true" />
            <node pkg="demo_nodes_cpp" exec="talker" name="sim_node" if="$(var mode)" />
            <group ns="robot">
                <node pkg="demo_nodes_cpp" exec="listener" name="listener1" />
            </group>
        </launch>"#,
    );

    let mut args = HashMap::new();
    args.insert("mode".to_string(), "true".to_string());

    let (direct, wasm) = both_pipelines(file.path(), args);
    assert_nodes_eq("complex_override", &direct.node, &wasm.node);
}

/// Container + nodes + groups in a single launch file (comparison test pattern).
#[test]
fn test_round_trip_mixed_nodes_and_container() {
    let file = write_xml(
        r#"<launch>
            <arg name="test_arg" default="value1"/>
            <node pkg="demo_nodes_cpp" exec="talker" name="talker_node" namespace="/test">
                <param name="use_sim_time" value="false"/>
            </node>
            <group>
                <push-ros-namespace namespace="planning"/>
                <node_container pkg="rclcpp_components" exec="component_container"
                                name="planning_container" namespace="">
                    <composable_node pkg="demo_nodes_cpp" plugin="demo_nodes_cpp::Talker"
                                     name="talker"/>
                    <composable_node pkg="demo_nodes_cpp" plugin="demo_nodes_cpp::Listener"
                                     name="listener"/>
                </node_container>
            </group>
        </launch>"#,
    );

    let (direct, wasm) = both_pipelines(file.path(), HashMap::new());
    assert_nodes_eq("mixed", &direct.node, &wasm.node);
    assert_eq!(direct.container.len(), wasm.container.len());

    for (d, w) in direct.container.iter().zip(wasm.container.iter()) {
        assert_eq!(d.name, w.name, "container name");
        assert_eq!(d.namespace, w.namespace, "container namespace");
        assert_eq!(d.cmd, w.cmd, "container cmd");
    }

    assert_load_nodes_eq("mixed", &direct.load_node, &wasm.load_node);
}

// ---------------------------------------------------------------------------
// Benchmark: timing comparison
// ---------------------------------------------------------------------------

/// Measure and print timing for direct parsing vs WASM pipeline.
/// Not a correctness test — just logs performance data.
#[test]
fn benchmark_wasm_vs_direct() {
    // Use a moderately complex launch file
    let file = write_xml(
        r#"<launch>
            <arg name="robot" default="bot1" />
            <arg name="mode" default="sim" />
            <set_env name="ROS_DOMAIN_ID" value="42" />
            <set_parameter name="use_sim_time" value="true" />
            <group ns="$(var robot)">
                <push-ros-namespace ns="sensors" />
                <node pkg="camera_driver" exec="camera" name="cam1">
                    <param name="fps" value="30" />
                    <remap from="image" to="front/image" />
                </node>
                <node pkg="lidar_driver" exec="lidar" name="lidar1" if="$(var mode)">
                    <param name="rate" value="10" />
                </node>
                <pop-ros-namespace />
                <push-ros-namespace ns="nav" />
                <node pkg="nav2" exec="planner" name="planner1">
                    <param name="algorithm" value="smac" />
                </node>
                <pop-ros-namespace />
            </group>
            <node_container pkg="rclcpp_components" exec="component_container"
                            name="container1" namespace="/comp">
                <composable_node pkg="pkg_a" plugin="PluginA" name="node_a" />
                <composable_node pkg="pkg_b" plugin="PluginB" name="node_b">
                    <param name="p1" value="v1" />
                </composable_node>
            </node_container>
            <executable cmd="ros2">
                <arg value="bag" />
                <arg value="play" />
            </executable>
        </launch>"#,
    );

    let args = HashMap::new();
    let iterations = 10;

    // Warm up
    let _ = play_launch_parser::parse_launch_file(file.path(), args.clone());

    // Direct parsing
    let start = std::time::Instant::now();
    for _ in 0..iterations {
        let _ = play_launch_parser::parse_launch_file(file.path(), args.clone()).unwrap();
    }
    let direct_total = start.elapsed();

    // WASM: analyze
    let start = std::time::Instant::now();
    let mut programs = Vec::new();
    for _ in 0..iterations {
        programs.push(
            play_launch_parser::analyze_launch_file_with_args(file.path(), args.clone()).unwrap(),
        );
    }
    let analyze_total = start.elapsed();

    // WASM: compile (use first program for all iterations)
    let program = &programs[0];
    let start = std::time::Instant::now();
    let mut wasm_bytes_vec = Vec::new();
    for _ in 0..iterations {
        wasm_bytes_vec.push(compile_to_wasm(program).unwrap());
    }
    let compile_total = start.elapsed();

    // WASM: execute
    let wasm_bytes = &wasm_bytes_vec[0];
    let start = std::time::Instant::now();
    for _ in 0..iterations {
        let _ = execute_wasm(wasm_bytes, args.clone()).unwrap();
    }
    let execute_total = start.elapsed();

    let wasm_pipeline_total = analyze_total + compile_total + execute_total;
    let module_size = wasm_bytes.len();

    eprintln!();
    eprintln!("=== WASM Pipeline Benchmark ({iterations} iterations) ===");
    eprintln!(
        "  Direct parse:      {:>8.2}ms avg ({:.2}ms total)",
        direct_total.as_secs_f64() * 1000.0 / iterations as f64,
        direct_total.as_secs_f64() * 1000.0
    );
    eprintln!(
        "  WASM analyze:      {:>8.2}ms avg ({:.2}ms total)",
        analyze_total.as_secs_f64() * 1000.0 / iterations as f64,
        analyze_total.as_secs_f64() * 1000.0
    );
    eprintln!(
        "  WASM compile:      {:>8.2}ms avg ({:.2}ms total)",
        compile_total.as_secs_f64() * 1000.0 / iterations as f64,
        compile_total.as_secs_f64() * 1000.0
    );
    eprintln!(
        "  WASM execute:      {:>8.2}ms avg ({:.2}ms total)",
        execute_total.as_secs_f64() * 1000.0 / iterations as f64,
        execute_total.as_secs_f64() * 1000.0
    );
    eprintln!(
        "  WASM full pipeline:{:>8.2}ms avg ({:.2}ms total)",
        wasm_pipeline_total.as_secs_f64() * 1000.0 / iterations as f64,
        wasm_pipeline_total.as_secs_f64() * 1000.0
    );
    eprintln!("  WASM module size:  {} bytes", module_size);
    eprintln!(
        "  Ratio (WASM/direct): {:.2}x",
        wasm_pipeline_total.as_secs_f64() / direct_total.as_secs_f64()
    );
    eprintln!();
}
