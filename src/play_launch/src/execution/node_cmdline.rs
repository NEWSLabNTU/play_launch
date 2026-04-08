use crate::ros::launch_dump::NodeRecord;
use eyre::{Context, bail};
use itertools::{Itertools, chain};
use serde::{Deserialize, Serialize};
use std::{
    borrow::{Borrow, Cow},
    collections::{HashMap, HashSet},
    fs,
    path::{Path, PathBuf},
    process::Command,
};

/// The command line information to execute a ROS node.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NodeCommandLine {
    pub command: Vec<String>,
    pub user_args: Vec<String>,
    pub remaps: HashMap<String, String>,
    pub params: HashMap<String, String>,
    pub params_files: HashSet<PathBuf>,
    /// Overrides YAML file written from inline params (global + node-specific).
    /// Rendered as the last `--params-file` so it overrides all earlier files.
    pub overrides_file: Option<PathBuf>,
    pub log_level: Option<String>,
    pub log_config_file: Option<PathBuf>,
    pub rosout_logs: Option<bool>,
    pub stdout_logs: Option<bool>,
    pub enclave: Option<String>,
    pub env: HashMap<String, String>,
}

/// Substitute $(var name) patterns with values from the variables map
fn substitute_variables(text: &str, variables: &HashMap<String, String>) -> String {
    let mut result = text.to_string();

    // Pattern: $(var name)
    // Use regex-free approach for simplicity and performance
    let mut search_from = 0;
    while let Some(offset) = result[search_from..].find("$(var ") {
        let start = search_from + offset;
        if let Some(end_offset) = result[start..].find(')') {
            let end = start + end_offset;
            let var_name = result[start + 6..end].trim();
            let pattern = &result[start..=end];
            if let Some(value) = variables.get(var_name) {
                // Skip self-referencing values (e.g., global_frame_rate = "$(var global_frame_rate)")
                if value == pattern {
                    search_from = end + 1;
                    continue;
                }
                let value = value.clone();
                result.replace_range(start..=end, &value);
                // Don't advance search_from — the replacement might contain more $(var)
            } else {
                // Variable not found — skip past it
                search_from = end + 1;
            }
        } else {
            // No closing parenthesis found
            break;
        }
    }

    result
}

/// Serialize parameters into a ROS 2 YAML params file format.
/// Uses `/**:` wildcard namespace so params apply regardless of node name/namespace.
/// Uses `yaml-rust2` for correct YAML output — multiline strings (e.g., URDF XML)
/// use literal block scalar (`|`), strings with quotes use proper escaping.
/// Serialize parameters into a ROS 2 YAML params file format.
/// Uses `/**:` wildcard namespace so params apply regardless of node name/namespace.
///
/// Uses `yaml-rust2` for correct YAML output:
/// - Multiline strings (e.g., URDF XML) → literal block scalar (`|`)
/// - Strings with quotes/special chars → properly escaped double-quoted strings
/// - Booleans, integers, floats → native YAML types
fn params_to_yaml(params: &HashMap<String, String>) -> String {
    use yaml_rust2::{Yaml, YamlEmitter, yaml::Hash};

    // Build ros__parameters mapping with type-aware values
    let mut ros_params = Hash::new();
    let mut sorted: Vec<_> = params.iter().collect();
    sorted.sort_by_key(|(k, _)| k.as_str());
    for (name, value) in sorted {
        if value.is_empty() {
            continue;
        }
        ros_params.insert(Yaml::String(name.clone()), str_to_yaml(value));
    }

    // Build: {"/**": {"ros__parameters": {params...}}}
    let mut namespace = Hash::new();
    namespace.insert(
        Yaml::String("ros__parameters".to_string()),
        Yaml::Hash(ros_params),
    );
    let mut root = Hash::new();
    root.insert(Yaml::String("/**".to_string()), Yaml::Hash(namespace));

    let doc = Yaml::Hash(root);
    let mut output = String::new();
    let mut emitter = YamlEmitter::new(&mut output);
    emitter.multiline_strings(true);
    emitter.dump(&doc).expect("YAML emit failed");

    // YamlEmitter prepends "---\n"; strip it for ROS compatibility
    output.strip_prefix("---\n").unwrap_or(&output).to_string()
}

/// Parse a string value into the most specific YAML type.
/// Matches rcl's type inference for `-p` arguments.
fn str_to_yaml(s: &str) -> yaml_rust2::Yaml {
    use yaml_rust2::Yaml;

    // Boolean
    match s {
        "true" | "True" => return Yaml::Boolean(true),
        "false" | "False" => return Yaml::Boolean(false),
        _ => {}
    }

    // Integer
    if let Ok(i) = s.parse::<i64>() {
        return Yaml::Integer(i);
    }

    // Float (only if it looks like a float — has decimal point or scientific notation)
    if (s.contains('.') || s.contains('e') || s.contains('E')) && s.parse::<f64>().is_ok() {
        return Yaml::Real(s.to_string());
    }

    // Everything else is a string
    Yaml::String(s.to_string())
}

impl NodeCommandLine {
    /// Construct from a ROS node record in the launch dump.
    pub fn from_node_record(
        record: &NodeRecord,
        params_files_dir: &Path,
        variables: &HashMap<String, String>,
    ) -> eyre::Result<Self> {
        // Raw executables (no package) — use cmd directly
        if record.package.is_none() {
            return Self::from_raw_executable(record);
        }

        let NodeRecord {
            executable,
            package,
            name,
            namespace,
            params,
            params_files: params_file_contents,
            remaps,
            ros_args: user_ros_args,
            args: user_nonros_args,
            exec_name: _,
            cmd: _,
            env,
            respawn: _,
            respawn_delay: _,
            global_params,
            scope: _,
        } = record;

        let Some(package) = package else {
            bail!(r#""package" is not set"#);
        };

        // Resolve any substitutions in the executable name (e.g., $(var container_executable))
        // This is needed for Python launch files that use LaunchConfiguration
        let resolved_executable = substitute_variables(executable, variables);

        // Find the executable directly using ament index instead of ros2 run CLI
        let exe_path = crate::ros::ament_index::find_executable(package, &resolved_executable)
            .wrap_err_with(|| {
                format!(
                    "Failed to find executable '{}' in package '{}' (original: '{}', resolved: '{}')",
                    resolved_executable, package, executable, resolved_executable
                )
            })?;

        let command: Vec<_> = vec![
            exe_path
                .to_str()
                .ok_or_else(|| eyre::eyre!("Executable path contains invalid UTF-8"))?
                .to_string(),
        ];

        let user_args: Vec<_> = {
            let user_nonros_args = user_nonros_args.iter().flatten().map(|s| s.as_str());
            let user_ros_args = user_ros_args
                .iter()
                .flat_map(|args| chain!(["--ros-args"], args.iter().map(|s| s.as_str()), ["--"]));
            chain!(user_nonros_args, user_ros_args)
                .map(|s| substitute_variables(s, variables))
                .collect()
        };

        let remaps: HashMap<_, _> = {
            // Only set __node when the launch file explicitly declares a name.
            // When name is None, omit __node so the node uses its internal default
            // (e.g., LifecycleNodes that expose services under their declared name).
            let name_remap = name
                .as_ref()
                .map(|name| ("__node".to_string(), name.to_string()));
            let namespace_remap = namespace
                .as_ref()
                .map(|namespace| ("__ns".to_string(), namespace.to_string()));
            let other_remaps = remaps
                .iter()
                .map(|(name, value)| (name.to_string(), value.to_string()));
            chain!(name_remap, namespace_remap, other_remaps).collect()
        };

        // Build params: global_params first, then node-specific params (so node-specific can override)
        let all_params: HashMap<_, _> = {
            let global = global_params
                .iter()
                .flatten()
                .map(|(name, value)| (name.to_string(), value.to_string()));
            let node_specific = params
                .iter()
                .map(|(name, value)| (name.to_string(), value.to_string()));
            // Chain global first, then node-specific (later values override earlier in HashMap)
            chain!(global, node_specific).collect()
        };

        let params_files: HashSet<_> = params_file_contents
            .iter()
            .enumerate()
            .map(|(idx, data)| {
                let file_name = format!("{idx}.yaml");
                let path = params_files_dir.join(file_name);
                fs::write(&path, data)
                    .wrap_err_with(|| format!("unable to write {}", path.display()))?;
                eyre::Ok(path)
            })
            .try_collect()?;

        // Write all inline params to overrides.yaml instead of passing them as -p flags.
        // This avoids rcl's argument parser limitations (e.g., "::" in param names,
        // empty values) and matches the official launch's approach of using temp YAML files.
        // Stored separately and rendered as the last --params-file so it overrides all earlier files.
        let overrides_file = if !all_params.is_empty() {
            let overrides_path = params_files_dir.join("overrides.yaml");
            let yaml = params_to_yaml(&all_params);
            fs::write(&overrides_path, yaml)
                .wrap_err_with(|| format!("unable to write {}", overrides_path.display()))?;
            Some(overrides_path)
        } else {
            None
        };

        let env: HashMap<_, _> = env
            .as_ref()
            .map(|vec| {
                vec.iter()
                    .map(|(key, value)| (key.clone(), value.clone()))
                    .collect()
            })
            .unwrap_or_default();

        Ok(Self {
            command,
            user_args,
            remaps,
            params: HashMap::new(), // All params written to overrides.yaml
            params_files,
            overrides_file,
            log_level: None,
            log_config_file: None,
            rosout_logs: None,
            stdout_logs: None,
            enclave: None,
            env,
        })
    }

    /// Construct from a raw executable record (no ROS package — uses cmd directly).
    fn from_raw_executable(record: &NodeRecord) -> eyre::Result<Self> {
        let cmd = &record.cmd;

        // cmd[0] is the full command string, remaining elements are extra args
        let command: Vec<String> = if cmd.is_empty() {
            bail!("raw executable record has empty cmd");
        } else {
            // Split the first element by whitespace to handle "script.py --arg val" as one string
            cmd[0].split_whitespace().map(|s| s.to_string()).collect()
        };

        // Extra args from cmd (beyond the first element, which is the command itself)
        let user_args: Vec<String> = cmd[1..].to_vec();

        let env: HashMap<_, _> = record
            .env
            .as_ref()
            .map(|vec| {
                vec.iter()
                    .map(|(key, value)| (key.clone(), value.clone()))
                    .collect()
            })
            .unwrap_or_default();

        Ok(Self {
            command,
            user_args,
            remaps: HashMap::new(),
            params: HashMap::new(),
            params_files: HashSet::new(),
            overrides_file: None,
            log_level: None,
            log_config_file: None,
            rosout_logs: None,
            stdout_logs: None,
            enclave: None,
            env,
        })
    }

    /// Create command line arguments.
    pub fn to_cmdline(&self, long_args: bool) -> Vec<String> {
        let Self {
            command,
            user_args,
            remaps,
            params,
            params_files,
            overrides_file,
            log_level,
            log_config_file,
            rosout_logs,
            stdout_logs,
            enclave,
            env: _,
        } = self;

        let has_ros_args = !(remaps.is_empty()
            && params.is_empty()
            && params_files.is_empty()
            && overrides_file.is_none()
            && log_level.is_none()
            && log_config_file.is_none()
            && rosout_logs.is_none()
            && stdout_logs.is_none()
            && enclave.is_none());

        let ros_args: Vec<_> = has_ros_args
            .then(|| {
                let param_switch = if long_args { "--param" } else { "-p" };
                let remap_switch = if long_args { "--remap" } else { "-r" };

                let log_level_args = log_level
                    .as_ref()
                    .map(|level| ["--log-level", level])
                    .into_iter()
                    .flatten()
                    .map(Cow::from);
                let log_config_file_args = log_config_file
                    .as_ref()
                    .map(|path| {
                        let path_str = path
                            .to_str()
                            .expect("log config file path contains invalid UTF-8");
                        ["--log-config-file", path_str]
                    })
                    .into_iter()
                    .flatten()
                    .map(Cow::from);
                let rosout_logs_args = rosout_logs
                    .map(|yes| {
                        if yes {
                            "--enable-rosout-logs"
                        } else {
                            "--disable-rosout-logs"
                        }
                    })
                    .map(Cow::from);
                let stdout_logs_args = stdout_logs
                    .map(|yes| {
                        if yes {
                            "--enable-stdout-logs"
                        } else {
                            "--disable-stdout-logs"
                        }
                    })
                    .map(Cow::from);
                let enclave_args = enclave
                    .as_ref()
                    .map(|value| ["--enclave", value])
                    .into_iter()
                    .flatten()
                    .map(Cow::from);
                let remap_args = remaps.iter().flat_map(move |(name, value)| {
                    [Cow::from(remap_switch), format!("{name}:={value}").into()]
                });
                let params_args = params
                    .iter()
                    .filter(|(name, value)| {
                        // Skip empty values — rcl rejects bare "-p name:="
                        // Skip names with "::" — rcl's parser treats "::" as a
                        // separator token, not a literal. These params are already
                        // loaded via --params-file so the inline -p is redundant.
                        !value.is_empty() && !name.contains("::")
                    })
                    .flat_map(move |(name, value)| {
                        [Cow::from(param_switch), format!("{name}:={value}").into()]
                    });
                let params_file_args = params_files
                    .iter()
                    .flat_map(|path| {
                        let path_str = path
                            .to_str()
                            .expect("params file path contains invalid UTF-8");
                        ["--params-file", path_str]
                    })
                    .map(Cow::from);
                // overrides.yaml comes last so it overrides all earlier --params-file entries
                let overrides_file_args = overrides_file
                    .as_ref()
                    .map(|path| {
                        let path_str = path
                            .to_str()
                            .expect("overrides file path contains invalid UTF-8");
                        ["--params-file", path_str]
                    })
                    .into_iter()
                    .flatten()
                    .map(Cow::from);

                chain!(
                    [Cow::from("--ros-args")],
                    log_level_args,
                    log_config_file_args,
                    rosout_logs_args,
                    stdout_logs_args,
                    enclave_args,
                    remap_args,
                    params_args,
                    params_file_args,
                    overrides_file_args,
                )
            })
            .into_iter()
            .flatten()
            .collect();

        let words: Vec<_> = chain!(
            command.iter().map(|arg| arg.as_str()),
            user_args.iter().map(|arg| arg.as_str()),
            ros_args.iter().map(|arg| arg.borrow()),
        )
        .map(|arg| arg.to_string())
        .collect();

        words
    }

    /// Create a command object.
    pub fn to_command(&self, long_args: bool, pgid: Option<i32>) -> Command {
        let cmdline = self.to_cmdline(long_args);
        let (program, args) = cmdline
            .split_first()
            .expect("command line must not be empty");
        let mut command = Command::new(program);
        command.args(args);

        // Apply environment variables from launch file
        command.envs(&self.env);

        // Explicitly preserve AMENT_PREFIX_PATH to ensure containers have access to all workspaces
        // This is critical for ament index lookups when loading composable nodes
        if let Ok(ament_prefix_path) = std::env::var("AMENT_PREFIX_PATH") {
            command.env("AMENT_PREFIX_PATH", ament_prefix_path);
        }

        // Set process group: join shared PGID if provided, otherwise create new process group
        #[cfg(unix)]
        {
            use std::os::unix::process::CommandExt;
            if let Some(pgid) = pgid {
                command.process_group(pgid);
            } else {
                command.process_group(0);
            }

            // Set parent death signal to prevent orphan processes
            // When play_launch dies (even with SIGKILL), kernel sends SIGKILL to all children
            unsafe {
                command.pre_exec(|| {
                    nix::sys::prctl::set_pdeathsig(nix::sys::signal::Signal::SIGKILL)
                        .map_err(std::io::Error::other)
                });
            }
        }

        command
    }

    /// Generate the shell script.
    pub fn to_shell(&self, long_args: bool) -> Vec<u8> {
        Itertools::intersperse(
            self.to_cmdline(long_args)
                .into_iter()
                .map(|arg| shell_quote::Sh::quote_vec(&arg)),
            vec![b' '],
        )
        .flatten()
        .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{HashMap, HashSet};

    #[test]
    fn test_generate_cmdline_with_remaps() {
        let cmdline = NodeCommandLine {
            command: vec!["cmd".to_string()],
            user_args: vec![],
            remaps: HashMap::from([("a".to_string(), "b".to_string())]),
            params: HashMap::new(),
            params_files: HashSet::new(),
            overrides_file: None,
            log_level: None,
            log_config_file: None,
            rosout_logs: None,
            stdout_logs: None,
            enclave: None,
            env: HashMap::new(),
        };
        let result = cmdline.to_cmdline(false);
        assert!(result.contains(&"--ros-args".to_string()));
        assert!(result.contains(&"-r".to_string()));
        assert!(result.contains(&"a:=b".to_string()));
    }

    #[test]
    #[should_panic(expected = "command line must not be empty")]
    fn test_command_line_must_not_be_empty() {
        let cmdline = NodeCommandLine {
            command: vec![],
            user_args: vec![],
            remaps: HashMap::new(),
            params: HashMap::new(),
            params_files: HashSet::new(),
            overrides_file: None,
            log_level: None,
            log_config_file: None,
            rosout_logs: None,
            stdout_logs: None,
            enclave: None,
            env: HashMap::new(),
        };
        let _ = cmdline.to_command(false, None);
    }

    #[test]
    fn test_substitute_variables() {
        let mut variables = HashMap::new();
        variables.insert("log_level".to_string(), "info".to_string());
        variables.insert("config_dir".to_string(), "/path/to/config".to_string());

        // Test simple substitution
        assert_eq!(
            substitute_variables("--log-level $(var log_level)", &variables),
            "--log-level info"
        );

        // Test multiple substitutions
        assert_eq!(
            substitute_variables("$(var log_level) and $(var config_dir)", &variables),
            "info and /path/to/config"
        );

        // Test no substitution
        assert_eq!(
            substitute_variables("--no-vars-here", &variables),
            "--no-vars-here"
        );

        // Test unknown variable (should leave as-is)
        assert_eq!(
            substitute_variables("--value $(var unknown)", &variables),
            "--value $(var unknown)"
        );
    }

    /// Self-referencing variables should be skipped, not loop infinitely.
    /// SSv2's Rust parser generates variables like `global_frame_rate = "$(var global_frame_rate)"`
    /// when LaunchConfiguration defaults aren't fully resolved.
    #[test]
    fn test_substitute_variables_self_referencing() {
        let mut variables = HashMap::new();
        variables.insert(
            "global_frame_rate".to_string(),
            "$(var global_frame_rate)".to_string(),
        );
        variables.insert("timeout".to_string(), "180".to_string());

        // Self-referencing variable should be left as-is (not infinite loop)
        assert_eq!(
            substitute_variables("--rate $(var global_frame_rate)", &variables),
            "--rate $(var global_frame_rate)"
        );

        // Non-self-referencing variable should still resolve
        assert_eq!(
            substitute_variables("--timeout $(var timeout)", &variables),
            "--timeout 180"
        );

        // Mixed: one self-referencing, one normal
        assert_eq!(
            substitute_variables("$(var global_frame_rate) $(var timeout)", &variables),
            "$(var global_frame_rate) 180"
        );
    }

    /// Helper to build a NodeCommandLine with controlled name/namespace for remap testing.
    /// Bypasses from_node_record (which needs ament_index) and directly tests the
    /// remap construction logic.
    fn build_remaps(
        name: Option<&str>,
        exec_name: Option<&str>,
        namespace: Option<&str>,
    ) -> HashMap<String, String> {
        // Mirror the remap logic from from_node_record
        let name = name.map(|s| s.to_string());
        let namespace = namespace.map(|s| s.to_string());
        let _exec_name = exec_name.map(|s| s.to_string());

        let name_remap = name
            .as_ref()
            .map(|name| ("__node".to_string(), name.to_string()));
        let namespace_remap = namespace
            .as_ref()
            .map(|namespace| ("__ns".to_string(), namespace.to_string()));
        chain!(name_remap, namespace_remap).collect()
    }

    /// When the launch file declares an explicit name, __node remap should be set.
    #[test]
    fn test_node_remap_with_explicit_name() {
        let remaps = build_remaps(Some("my_custom_name"), Some("my_node_exec"), None);
        assert_eq!(
            remaps.get("__node").map(|s| s.as_str()),
            Some("my_custom_name"),
            "__node should use the explicit name from the launch file"
        );
    }

    /// When the launch file does NOT declare a name, __node should NOT be set.
    /// This lets the node use its internal default name (important for LifecycleNodes
    /// whose lifecycle services are registered under the internal name).
    #[test]
    fn test_node_remap_without_name_omits_node() {
        let remaps = build_remaps(None, Some("my_node_exec"), None);
        assert!(
            !remaps.contains_key("__node"),
            "__node should NOT be set when launch file omits name (node uses internal default)"
        );
    }

    /// __ns remap should be set when namespace is provided.
    #[test]
    fn test_namespace_remap() {
        let remaps = build_remaps(None, Some("my_node"), Some("/simulation"));
        assert_eq!(remaps.get("__ns").map(|s| s.as_str()), Some("/simulation"));
    }

    /// Both __node and __ns should be set when both name and namespace are provided.
    #[test]
    fn test_node_and_namespace_remap() {
        let remaps = build_remaps(Some("my_node"), None, Some("/ns"));
        assert_eq!(remaps.get("__node").map(|s| s.as_str()), Some("my_node"));
        assert_eq!(remaps.get("__ns").map(|s| s.as_str()), Some("/ns"));
    }

    /// Parameters with "::" in names should be excluded from -p args.
    /// rcl's parser treats "::" as a separator token, causing parse failures.
    /// These params are loaded via --params-file instead.
    #[test]
    fn test_params_with_double_colon_excluded() {
        let cmdline = NodeCommandLine {
            command: vec!["cmd".to_string()],
            user_args: vec![],
            remaps: HashMap::new(),
            params: HashMap::from([
                ("normal_param".to_string(), "1.0".to_string()),
                (
                    "sensor_msgs::msg::Imu.angular_velocity.y".to_string(),
                    "0.0".to_string(),
                ),
            ]),
            params_files: HashSet::new(),
            overrides_file: None,
            log_level: None,
            log_config_file: None,
            rosout_logs: None,
            stdout_logs: None,
            enclave: None,
            env: HashMap::new(),
        };
        let result = cmdline.to_cmdline(false);
        assert!(
            result.contains(&"normal_param:=1.0".to_string()),
            "Normal params should be included"
        );
        assert!(
            !result.iter().any(|a| a.contains("sensor_msgs::msg")),
            "Params with :: should be excluded from -p args"
        );
    }

    /// Parameters with empty values should be excluded from -p args.
    /// rcl rejects bare "-p name:=" without a value.
    #[test]
    fn test_params_with_empty_value_excluded() {
        let cmdline = NodeCommandLine {
            command: vec!["cmd".to_string()],
            user_args: vec![],
            remaps: HashMap::new(),
            params: HashMap::from([
                ("has_value".to_string(), "true".to_string()),
                ("empty_value".to_string(), "".to_string()),
            ]),
            params_files: HashSet::new(),
            overrides_file: None,
            log_level: None,
            log_config_file: None,
            rosout_logs: None,
            stdout_logs: None,
            enclave: None,
            env: HashMap::new(),
        };
        let result = cmdline.to_cmdline(false);
        assert!(
            result.contains(&"has_value:=true".to_string()),
            "Params with values should be included"
        );
        assert!(
            !result.iter().any(|a| a.contains("empty_value")),
            "Params with empty values should be excluded"
        );
    }

    /// Helper to roundtrip-parse params_to_yaml output and extract a param value.
    fn parse_param(yaml: &str, key: &str) -> serde_yaml_ng::Value {
        let parsed: serde_yaml_ng::Value = serde_yaml_ng::from_str(yaml)
            .unwrap_or_else(|e| panic!("Invalid YAML:\n{yaml}\nError: {e}"));
        parsed["/**"]["ros__parameters"][key].clone()
    }

    /// params_to_yaml produces valid ROS 2 YAML with correct types.
    #[test]
    fn test_params_to_yaml_basic() {
        let params = HashMap::from([
            ("use_sim_time".to_string(), "true".to_string()),
            ("frequency".to_string(), "10.0".to_string()),
        ]);
        let yaml = params_to_yaml(&params);
        assert_eq!(
            parse_param(&yaml, "use_sim_time"),
            serde_yaml_ng::Value::Bool(true)
        );
        // Float value
        let freq = parse_param(&yaml, "frequency");
        assert!(
            freq.is_f64() || freq.is_number(),
            "frequency should be numeric: {freq:?}"
        );
    }

    /// params_to_yaml handles large XML string values with embedded quotes.
    #[test]
    fn test_params_to_yaml_xml_value() {
        let xml = r#"<?xml version="1.0" ?><robot name="test"><link name="base"/></robot>"#;
        let params = HashMap::from([("robot_description".to_string(), xml.to_string())]);
        let yaml = params_to_yaml(&params);
        let rd = parse_param(&yaml, "robot_description");
        // yaml-rust2 uses escape_str for quotes — roundtrip should preserve content
        assert_eq!(rd.as_str().unwrap().trim_end(), xml);
    }

    /// params_to_yaml handles :: in parameter names.
    #[test]
    fn test_params_to_yaml_with_double_colon() {
        let params = HashMap::from([(
            "sensor_msgs::msg::Imu.angular_velocity.y".to_string(),
            "0.0".to_string(),
        )]);
        let yaml = params_to_yaml(&params);
        let v = parse_param(&yaml, "sensor_msgs::msg::Imu.angular_velocity.y");
        assert!(!v.is_null(), ":: param should be present in YAML");
    }

    /// params_to_yaml skips empty values.
    #[test]
    fn test_params_to_yaml_skips_empty() {
        let params = HashMap::from([
            ("good".to_string(), "1".to_string()),
            ("empty".to_string(), "".to_string()),
        ]);
        let yaml = params_to_yaml(&params);
        assert!(!parse_param(&yaml, "good").is_null());
        assert!(parse_param(&yaml, "empty").is_null());
    }

    /// params_to_yaml produces sorted output for deterministic results.
    #[test]
    fn test_params_to_yaml_sorted() {
        let params = HashMap::from([
            ("zebra".to_string(), "z".to_string()),
            ("alpha".to_string(), "a".to_string()),
            ("middle".to_string(), "m".to_string()),
        ]);
        let yaml = params_to_yaml(&params);
        let alpha_pos = yaml.find("alpha").unwrap();
        let middle_pos = yaml.find("middle").unwrap();
        let zebra_pos = yaml.find("zebra").unwrap();
        assert!(alpha_pos < middle_pos);
        assert!(middle_pos < zebra_pos);
    }

    /// overrides.yaml is rendered as the last --params-file in the command line.
    #[test]
    fn test_overrides_file_comes_last() {
        let cmdline = NodeCommandLine {
            command: vec!["cmd".to_string()],
            user_args: vec![],
            remaps: HashMap::new(),
            params: HashMap::new(),
            params_files: HashSet::from([PathBuf::from("/tmp/0.yaml")]),
            overrides_file: Some(PathBuf::from("/tmp/overrides.yaml")),
            log_level: None,
            log_config_file: None,
            rosout_logs: None,
            stdout_logs: None,
            enclave: None,
            env: HashMap::new(),
        };
        let result = cmdline.to_cmdline(false);
        // Find positions of --params-file args
        let params_file_positions: Vec<_> = result
            .iter()
            .enumerate()
            .filter(|(_, a)| a.ends_with(".yaml"))
            .map(|(i, a)| (i, a.clone()))
            .collect();
        assert_eq!(params_file_positions.len(), 2);
        // overrides.yaml must be the last one
        assert!(
            params_file_positions
                .last()
                .unwrap()
                .1
                .contains("overrides"),
            "overrides.yaml should be the last --params-file"
        );
    }
}
