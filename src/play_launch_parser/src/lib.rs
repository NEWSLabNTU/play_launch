//! play_launch_parser library

// PyO3 0.20 macros generate unsafe calls inside unsafe fn without explicit
// unsafe blocks. This lint became warn-by-default in edition 2024.
// Remove this once PyO3 is upgraded to 0.23+.
#![allow(unsafe_op_in_unsafe_fn)]

pub mod actions;
pub mod captures;
pub mod condition;
pub mod error;
mod file_cache;
#[cfg(feature = "ir")]
pub mod ir;
pub mod params;
pub mod python;
pub mod record;
pub mod substitution;
mod traverser;
pub mod xml;

use error::Result;
use file_cache::read_file_cached;
use record::RecordJson;

use std::{
    collections::HashMap,
    path::{Path, PathBuf},
};
use substitution::LaunchContext;
pub use substitution::types::block_command_substitution;

/// Default maximum include depth (non-circular nesting limit).
///
/// Autoware's deepest include chain is ~10 levels. 100 is generous enough to
/// never false-positive, conservative enough to prevent stack overflow.
pub const DEFAULT_MAX_INCLUDE_DEPTH: usize = 100;

/// Options for configuring the launch file parser.
#[derive(Debug, Clone)]
pub struct ParseOptions {
    /// Maximum allowed include nesting depth (default: 100).
    pub max_include_depth: usize,
}

impl Default for ParseOptions {
    fn default() -> Self {
        Self {
            max_include_depth: DEFAULT_MAX_INCLUDE_DEPTH,
        }
    }
}

/// Launch tree traverser for parsing launch files
pub struct LaunchTraverser {
    /// Unified context for substitution resolution, namespace management, and entity captures
    pub(crate) context: LaunchContext,
    /// Track include chain to detect circular includes (thread-local stack)
    pub(crate) include_chain: Vec<PathBuf>,
    /// Maximum allowed include nesting depth
    pub(crate) max_include_depth: usize,
    /// Temporary storage for records during XML parsing
    pub(crate) records: Vec<record::NodeRecord>,
    pub(crate) containers: Vec<record::ComposableNodeContainerRecord>,
    pub(crate) load_nodes: Vec<record::LoadNodeRecord>,
    /// Launch tree scope table — tracks which launch file each node comes from
    pub(crate) scope_table: record::ScopeTable,
    /// Current scope ID (set when entering a launch file, used to stamp records)
    pub(crate) current_scope_id: usize,
}

impl LaunchTraverser {
    pub fn new(cli_args: HashMap<String, String>) -> Self {
        Self::with_options(cli_args, ParseOptions::default())
    }

    pub fn with_options(cli_args: HashMap<String, String>, options: ParseOptions) -> Self {
        let mut context = LaunchContext::new();
        // Apply CLI args to LaunchContext for substitution resolution
        for (k, v) in &cli_args {
            context.set_configuration(k.clone(), v.clone());
        }

        Self {
            context,
            include_chain: Vec::new(),
            max_include_depth: options.max_include_depth,
            records: Vec::new(),
            containers: Vec::new(),
            load_nodes: Vec::new(),
            scope_table: record::ScopeTable::new(),
            current_scope_id: 0,
        }
    }

    pub fn traverse_file(&mut self, path: &Path) -> Result<()> {
        // Create root scope if the scope table is empty (first file)
        if self.scope_table.is_empty() {
            let file_name = path
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or("unknown")
                .to_string();
            let pkg = record::extract_package_from_path(path);
            let ns = self.context.current_namespace();
            let root_id =
                self.scope_table
                    .push(pkg, file_name, ns, self.context.configurations(), None);
            self.current_scope_id = root_id;
        }

        // Check file extension and handle non-XML files
        if let Some(ext) = path.extension().and_then(|s| s.to_str()) {
            match ext {
                "py" => {
                    log::debug!("Executing Python launch file: {}", path.display());
                    return self.execute_python_file(path, &self.context.configurations());
                }
                "yaml" | "yml" => {
                    // YAML files in traverse_file are always launch files
                    // (parameter files are handled in <param from="..."> context)
                    log::debug!("Processing YAML launch file: {}", path.display());
                    self.process_yaml_launch_file(path)?;
                    return Ok(());
                }
                _ => {}
            }
        }

        // Set current file in context
        self.context.set_current_file(path.to_path_buf());

        let content = read_file_cached(path)?;
        let doc = roxmltree::Document::parse(&content)?;
        let root = xml::XmlEntity::new(doc.root_element());
        self.traverse_entity(&root)?;
        Ok(())
    }
}

/// Parse launch file and generate record.json
pub fn parse_launch_file(path: &Path, cli_args: HashMap<String, String>) -> Result<RecordJson> {
    let mut traverser = LaunchTraverser::new(cli_args);
    traverser.traverse_file(path)?;
    traverser.into_record_json()
}

/// Parse launch file with custom options.
pub fn parse_launch_file_with_options(
    path: &Path,
    cli_args: HashMap<String, String>,
    options: ParseOptions,
) -> Result<RecordJson> {
    let mut traverser = LaunchTraverser::with_options(cli_args, options);
    traverser.traverse_file(path)?;
    traverser.into_record_json()
}

#[cfg(feature = "ir")]
/// Parse a launch file into its IR without evaluating conditions.
///
/// Returns a `LaunchProgram` tree that preserves all conditional branches,
/// unevaluated substitution expressions, and include hierarchy.
pub fn analyze_launch_file(path: &Path) -> Result<ir::LaunchProgram> {
    let mut traverser = LaunchTraverser::new(HashMap::new());
    traverser.build_ir_file(path)
}

#[cfg(feature = "ir")]
/// Parse a launch file into its IR with the given arguments applied.
///
/// Arguments are applied to the context so that include file paths and
/// conditional expressions that reference them can be resolved during
/// IR construction. Conditions are still stored (not evaluated).
pub fn analyze_launch_file_with_args(
    path: &Path,
    args: HashMap<String, String>,
) -> Result<ir::LaunchProgram> {
    let mut traverser = LaunchTraverser::new(args);
    traverser.build_ir_file(path)
}

#[cfg(feature = "ir")]
/// Evaluate a launch file via its IR representation.
///
/// Builds the IR tree with `analyze_launch_file_with_args`, then walks the tree
/// resolving expressions and conditions to produce the same `RecordJson` as
/// `parse_launch_file()`. Only pure-XML launch files produce complete results;
/// Python/YAML includes are skipped with a debug log.
pub fn evaluate_launch_file(path: &Path, args: HashMap<String, String>) -> Result<RecordJson> {
    let program = analyze_launch_file_with_args(path, args.clone())?;
    let mut traverser = LaunchTraverser::new(args);
    traverser.evaluate_ir(&program)?;
    traverser.into_record_json()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    #[test]
    fn test_parse_simple_launch() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].executable, "talker");
        assert_eq!(record.node[0].package, Some("demo_nodes_cpp".to_string()));
    }

    #[test]
    fn test_parse_with_args() {
        let xml = r#"<launch>
            <arg name="node_name" default="my_talker" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var node_name)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].name, Some("my_talker".to_string()));
    }

    #[test]
    fn test_parse_with_cli_override() {
        let xml = r#"<launch>
            <arg name="node_name" default="default_name" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var node_name)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let mut cli_args = HashMap::new();
        cli_args.insert("node_name".to_string(), "cli_name".to_string());

        let record = parse_launch_file(file.path(), cli_args).unwrap();
        assert_eq!(record.node[0].name, Some("cli_name".to_string()));
    }

    #[test]
    fn test_parse_multiple_nodes() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" />
            <node pkg="demo_nodes_cpp" exec="listener" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 2);
        assert_eq!(record.node[0].executable, "talker");
        assert_eq!(record.node[1].executable, "listener");
    }

    #[test]
    fn test_parse_executable() {
        let xml = r#"<launch>
            <executable cmd="rosbag" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].executable, "rosbag");
        assert_eq!(record.node[0].package, None);
    }

    #[test]
    fn test_parse_executable_with_args() {
        let xml = r#"<launch>
            <executable cmd="rosbag">
                <arg value="record" />
                <arg value="-a" />
            </executable>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].executable, "rosbag");
        assert_eq!(record.node[0].cmd, vec!["rosbag", "record", "-a"]);
        assert_eq!(
            record.node[0].args,
            Some(vec!["record".to_string(), "-a".to_string()])
        );
    }

    #[test]
    fn test_parse_mixed_nodes_and_executables() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" />
            <executable cmd="rviz2" />
            <node pkg="demo_nodes_cpp" exec="listener" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 3);
        assert_eq!(record.node[0].executable, "talker");
        assert_eq!(record.node[0].package, Some("demo_nodes_cpp".to_string()));
        assert_eq!(record.node[1].executable, "rviz2");
        assert_eq!(record.node[1].package, None);
        assert_eq!(record.node[2].executable, "listener");
        assert_eq!(record.node[2].package, Some("demo_nodes_cpp".to_string()));
    }

    #[test]
    fn test_parse_node_with_param_file() {
        use std::io::Write;

        // Create a parameter YAML file
        let yaml_content = r#"
test_node:
  ros__parameters:
    param1: "value1"
    param2: 42
    nested:
      param3: "nested_value"
"#;
        let mut param_file = NamedTempFile::new().unwrap();
        param_file.write_all(yaml_content.as_bytes()).unwrap();
        param_file.flush().unwrap();

        let param_file_path = param_file.path().to_str().unwrap();

        // Create a launch file that references the parameter file
        let xml = format!(
            r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker">
                <param name="inline_param" value="inline_value" />
                <param from="{}" />
            </node>
        </launch>"#,
            param_file_path
        );

        let mut launch_file = NamedTempFile::new().unwrap();
        launch_file.write_all(xml.as_bytes()).unwrap();
        launch_file.flush().unwrap();

        let record = parse_launch_file(launch_file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        // Check that inline params are in params array, but file params are NOT
        // (file params should only be referenced via param_files)
        let node = &record.node[0];
        assert_eq!(node.params.len(), 1); // Only inline_param

        // Check inline parameter
        let inline_param = node
            .params
            .iter()
            .find(|(k, _)| k == "inline_param")
            .expect("inline_param not found");
        assert_eq!(inline_param.1, "inline_value");

        // Check that parameter file contents are recorded
        // Note: Parameters from files are NOT expanded into params array
        // to avoid issues with special characters (colons, spaces) that
        // cannot be passed via command-line `-p` arguments.
        // The file contents are stored for play_launch to write out.
        assert_eq!(node.params_files.len(), 1);
        assert!(
            node.params_files[0].contains("param1"),
            "File contents should contain param1"
        );
        assert!(
            node.params_files[0].contains("param2"),
            "File contents should contain param2"
        );

        // These parameters would be in the file, but NOT in params array
        let param1 = node.params.iter().find(|(k, _)| k == "param1");
        assert!(param1.is_none(), "param1 should not be in params array");

        let param2 = node.params.iter().find(|(k, _)| k == "param2");
        assert!(param2.is_none(), "param2 should not be in params array");

        let nested_param = node.params.iter().find(|(k, _)| k == "nested.param3");
        assert!(
            nested_param.is_none(),
            "nested.param3 should not be in params array"
        );

        // Param file path is already checked above
    }

    #[test]
    fn test_namespace_scoping_simple() {
        let xml = r#"<launch>
            <group>
                <push-ros-namespace ns="robot1" />
                <node pkg="demo_nodes_cpp" exec="talker" />
            </group>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].namespace, Some("/robot1".to_string()));
    }

    #[test]
    fn test_namespace_scoping_nested() {
        let xml = r#"<launch>
            <group>
                <push-ros-namespace ns="robot1" />
                <group>
                    <push-ros-namespace ns="sensors" />
                    <node pkg="demo_nodes_cpp" exec="talker" />
                </group>
            </group>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(
            record.node[0].namespace,
            Some("/robot1/sensors".to_string())
        );
    }

    #[test]
    fn test_namespace_scoping_with_push_ros_namespace() {
        let xml = r#"<launch>
            <group>
                <push-ros-namespace namespace="robot1"/>
                <node pkg="demo_nodes_cpp" exec="talker" />
                <group>
                    <push-ros-namespace namespace="sensors"/>
                    <node pkg="demo_nodes_cpp" exec="listener" />
                </group>
            </group>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 2);
        assert_eq!(record.node[0].namespace, Some("/robot1".to_string()));
        assert_eq!(
            record.node[1].namespace,
            Some("/robot1/sensors".to_string())
        );
    }

    #[test]
    fn test_namespace_explicit_overrides_group() {
        let xml = r#"<launch>
            <group>
                <push-ros-namespace namespace="robot1"/>
                <node pkg="demo_nodes_cpp" exec="talker" namespace="/override" />
            </group>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Explicit namespace should override group namespace
        assert_eq!(record.node[0].namespace, Some("/override".to_string()));
    }

    #[test]
    fn test_namespace_absolute() {
        let xml = r#"<launch>
            <group>
                <push-ros-namespace namespace="robot1"/>
                <group>
                    <push-ros-namespace namespace="/absolute"/>
                    <node pkg="demo_nodes_cpp" exec="talker" />
                </group>
            </group>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Absolute namespace should override parent
        assert_eq!(record.node[0].namespace, Some("/absolute".to_string()));
    }

    #[test]
    fn test_set_env_literal() {
        let xml = r#"<launch>
            <set_env name="ROS_DOMAIN_ID" value="42" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        // Check that environment was set
        let env = record.node[0].env.as_ref().unwrap();
        assert!(env.iter().any(|(k, v)| k == "ROS_DOMAIN_ID" && v == "42"));
    }

    #[test]
    fn test_set_env_with_substitution() {
        let xml = r#"<launch>
            <arg name="domain_id" default="99" />
            <set_env name="ROS_DOMAIN_ID" value="$(var domain_id)" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let env = record.node[0].env.as_ref().unwrap();
        assert!(env.iter().any(|(k, v)| k == "ROS_DOMAIN_ID" && v == "99"));
    }

    #[test]
    fn test_unset_env() {
        let xml = r#"<launch>
            <set_env name="VAR1" value="value1" />
            <set_env name="VAR2" value="value2" />
            <unset_env name="VAR1" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let env = record.node[0].env.as_ref().unwrap();
        // VAR1 should not be present
        assert!(!env.iter().any(|(k, _)| k == "VAR1"));
        // VAR2 should be present
        assert!(env.iter().any(|(k, v)| k == "VAR2" && v == "value2"));
    }

    #[test]
    fn test_env_inheritance_in_nodes() {
        let xml = r#"<launch>
            <set_env name="GLOBAL_VAR" value="global" />
            <node pkg="demo_nodes_cpp" exec="talker">
                <env name="NODE_VAR" value="node_specific" />
            </node>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let env = record.node[0].env.as_ref().unwrap();
        // Both global and node-specific environment should be present
        assert!(env.iter().any(|(k, v)| k == "GLOBAL_VAR" && v == "global"));
        assert!(
            env.iter()
                .any(|(k, v)| k == "NODE_VAR" && v == "node_specific")
        );
    }

    #[test]
    fn test_node_env_overrides_context() {
        let xml = r#"<launch>
            <set_env name="MY_VAR" value="context_value" />
            <node pkg="demo_nodes_cpp" exec="talker">
                <env name="MY_VAR" value="node_value" />
            </node>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let env = record.node[0].env.as_ref().unwrap();
        // Node-specific value should override context value
        let my_var = env.iter().find(|(k, _)| k == "MY_VAR").unwrap();
        assert_eq!(my_var.1, "node_value");
    }

    #[test]
    fn test_env_with_executable() {
        let xml = r#"<launch>
            <set_env name="EXEC_VAR" value="exec_value" />
            <executable cmd="echo" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let env = record.node[0].env.as_ref().unwrap();
        assert!(
            env.iter()
                .any(|(k, v)| k == "EXEC_VAR" && v == "exec_value")
        );
    }

    #[test]
    fn test_declare_argument_with_default() {
        let xml = r#"<launch>
            <declare_argument name="my_arg" default="default_value" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var my_arg)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // The default value should be used
        assert_eq!(record.node[0].name, Some("default_value".to_string()));
    }

    #[test]
    fn test_declare_argument_cli_override() {
        let xml = r#"<launch>
            <declare_argument name="my_arg" default="default_value" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var my_arg)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let mut cli_args = HashMap::new();
        cli_args.insert("my_arg".to_string(), "cli_value".to_string());

        let record = parse_launch_file(file.path(), cli_args).unwrap();
        assert_eq!(record.node.len(), 1);
        // CLI value should override default
        assert_eq!(record.node[0].name, Some("cli_value".to_string()));
    }

    #[test]
    fn test_declare_argument_without_default() {
        let xml = r#"<launch>
            <declare_argument name="required_arg" description="A required argument" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        // This should not crash even though required_arg is declared without a default
        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
    }

    #[test]
    fn test_declare_argument_then_arg() {
        let xml = r#"<launch>
            <declare_argument name="my_param" description="Test parameter" />
            <arg name="my_param" default="set_value" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var my_param)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Value set by <arg> should be used
        assert_eq!(record.node[0].name, Some("set_value".to_string()));
    }

    #[test]
    fn test_declare_argument_with_description() {
        let xml = r#"<launch>
            <declare_argument
                name="robot_name"
                default="robot1"
                description="Name of the robot"
            />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var robot_name)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].name, Some("robot1".to_string()));
    }

    #[test]
    fn test_set_parameter_simple() {
        let xml = r#"<launch>
            <set_parameter name="use_sim_time" value="true" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        // Check that global parameter was set
        let global_params = record.node[0].global_params.as_ref().unwrap();
        assert!(
            global_params
                .iter()
                .any(|(k, v)| k == "use_sim_time" && v == "True")
        );
    }

    #[test]
    fn test_set_parameter_with_substitution() {
        let xml = r#"<launch>
            <arg name="sim_mode" default="true" />
            <set_parameter name="use_sim_time" value="$(var sim_mode)" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let global_params = record.node[0].global_params.as_ref().unwrap();
        assert!(
            global_params
                .iter()
                .any(|(k, v)| k == "use_sim_time" && v == "True")
        );
    }

    #[test]
    fn test_multiple_set_parameters() {
        let xml = r#"<launch>
            <set_parameter name="use_sim_time" value="true" />
            <set_parameter name="log_level" value="debug" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);

        let global_params = record.node[0].global_params.as_ref().unwrap();
        assert_eq!(global_params.len(), 2);
        assert!(
            global_params
                .iter()
                .any(|(k, v)| k == "use_sim_time" && v == "True")
        );
        assert!(
            global_params
                .iter()
                .any(|(k, v)| k == "log_level" && v == "debug")
        );
    }

    #[test]
    fn test_set_parameter_applies_to_all_nodes() {
        let xml = r#"<launch>
            <set_parameter name="use_sim_time" value="true" />
            <node pkg="demo_nodes_cpp" exec="talker" />
            <node pkg="demo_nodes_cpp" exec="listener" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 2);

        // Both nodes should have the global parameter
        for node_record in &record.node {
            let global_params = node_record.global_params.as_ref().unwrap();
            assert!(
                global_params
                    .iter()
                    .any(|(k, v)| k == "use_sim_time" && v == "True")
            );
        }
    }

    #[test]
    fn test_optenv_with_existing_var() {
        unsafe { std::env::set_var("TEST_OPTENV_LAUNCH", "from_env") };
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(optenv TEST_OPTENV_LAUNCH)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].name, Some("from_env".to_string()));
    }

    #[test]
    fn test_optenv_with_missing_var_no_default() {
        unsafe { std::env::remove_var("NONEXISTENT_OPTENV_LAUNCH") };
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="node_$(optenv NONEXISTENT_OPTENV_LAUNCH)_end" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Should use empty string for missing var
        assert_eq!(record.node[0].name, Some("node__end".to_string()));
    }

    #[test]
    fn test_optenv_with_missing_var_with_default() {
        unsafe { std::env::remove_var("NONEXISTENT_OPTENV_LAUNCH2") };
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(optenv NONEXISTENT_OPTENV_LAUNCH2 default_name)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Should use default value
        assert_eq!(record.node[0].name, Some("default_name".to_string()));
    }

    #[test]
    fn test_optenv_vs_env_missing_var() {
        unsafe { std::env::remove_var("MISSING_TEST_VAR") };

        // optenv should work with missing variable
        let xml_optenv = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(optenv MISSING_TEST_VAR fallback)" />
        </launch>"#;
        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml_optenv.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node[0].name, Some("fallback".to_string()));

        // env without default should error
        let xml_env = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(env MISSING_TEST_VAR)" />
        </launch>"#;
        let mut file2 = NamedTempFile::new().unwrap();
        file2.write_all(xml_env.as_bytes()).unwrap();

        let result = parse_launch_file(file2.path(), HashMap::new());
        assert!(result.is_err());
    }

    #[test]
    fn test_command_simple() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(command echo test_node)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].name, Some("test_node".to_string()));
    }

    #[test]
    fn test_command_with_args() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(command echo foo bar)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].name, Some("foo bar".to_string()));
    }

    #[test]
    fn test_command_output_trimming() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(command printf '  test  \n')" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Output should be trimmed
        assert_eq!(record.node[0].name, Some("test".to_string()));
    }

    #[test]
    fn test_command_in_parameter() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker">
                <param name="hostname" value="$(command hostname)" />
            </node>
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        // Should have the hostname parameter with some value
        let hostname_param = record.node[0]
            .params
            .iter()
            .find(|(k, _)| k == "hostname")
            .expect("hostname param not found");
        // Just verify it's not empty
        assert!(!hostname_param.1.is_empty());
    }

    #[test]
    fn test_command_with_pipe() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(command echo test | tr a-z A-Z)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        let record = parse_launch_file(file.path(), HashMap::new()).unwrap();
        assert_eq!(record.node.len(), 1);
        assert_eq!(record.node[0].name, Some("TEST".to_string()));
    }

    #[test]
    fn test_command_failed() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="$(command exit 1)" />
        </launch>"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(xml.as_bytes()).unwrap();

        // Should fail because the command exits with non-zero status
        let result = parse_launch_file(file.path(), HashMap::new());
        assert!(result.is_err());
    }
}
