use crate::ros::launch_dump::NodeRecord;
use eyre::{bail, Context};
use itertools::{chain, Itertools};
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

    // Pattern: $(var name) or $(var name)
    // Use regex-free approach for simplicity and performance
    while let Some(start) = result.find("$(var ") {
        if let Some(end) = result[start..].find(')') {
            let var_name = &result[start + 6..start + end].trim();
            if let Some(value) = variables.get(*var_name) {
                result.replace_range(start..start + end + 1, value);
            } else {
                // Variable not found - leave it as is and move past it to avoid infinite loop
                break;
            }
        } else {
            // No closing parenthesis found
            break;
        }
    }

    result
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

        let command: Vec<_> = vec![exe_path
            .to_str()
            .ok_or_else(|| eyre::eyre!("Executable path contains invalid UTF-8"))?
            .to_string()];

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
        let params: HashMap<_, _> = {
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
            params,
            params_files,
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
                let params_args = params.iter().flat_map(move |(name, value)| {
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
}
