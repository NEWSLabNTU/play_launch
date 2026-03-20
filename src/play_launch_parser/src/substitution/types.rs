//! Substitution types

use super::eval::evaluate_expression;
use crate::{error::SubstitutionError, substitution::context::LaunchContext};
use dashmap::DashMap;
use once_cell::sync::Lazy;
use std::sync::atomic::{AtomicBool, Ordering};

/// Global flag controlling whether `$(command ...)` substitutions are blocked.
/// Default: false (allowed with warning). Set to true via `block_command_substitution(true)`.
///
/// # Security
/// `$(command)` executes arbitrary shell commands. Use `--block-commands` to
/// reject them entirely when parsing untrusted launch files.
static BLOCK_COMMANDS: AtomicBool = AtomicBool::new(false);

/// Block or allow `$(command ...)` substitution execution.
/// When blocked, the parser returns an error on any `$(command)`.
/// When allowed (default), commands execute with a security warning.
pub fn block_command_substitution(block: bool) {
    BLOCK_COMMANDS.store(block, Ordering::Relaxed);
}

/// Error handling mode for command substitutions
#[derive(Debug, Clone, PartialEq, Default)]
pub enum CommandErrorMode {
    /// Fail on any error (default)
    #[default]
    Strict,
    /// Log stderr as warning but continue (return stdout)
    Warn,
    /// Ignore all errors (return stdout regardless)
    Ignore,
}

/// Substitution enum representing different types of substitutions
#[derive(Debug, Clone, PartialEq)]
pub enum Substitution {
    /// Plain text (no substitution)
    Text(String),
    /// $(var name) - Launch configuration variable (name can contain nested substitutions)
    LaunchConfiguration(Vec<Substitution>),
    /// $(env VAR [default]) - Environment variable with optional default
    EnvironmentVariable {
        name: Vec<Substitution>,
        default: Option<Vec<Substitution>>,
    },
    /// $(optenv VAR [default]) - Optional environment variable (returns empty string if not set)
    OptionalEnvironmentVariable {
        name: Vec<Substitution>,
        default: Option<Vec<Substitution>>,
    },
    /// $(command cmd [error_mode]) - Execute shell command and capture output
    Command {
        cmd: Vec<Substitution>,
        error_mode: CommandErrorMode,
    },
    /// $(find-pkg-share package_name) - Find ROS 2 package share directory
    FindPackageShare(Vec<Substitution>),
    /// $(dirname) - Directory of the current launch file
    Dirname,
    /// $(filename) - Filename of the current launch file
    Filename,
    /// $(anon name) - Generate anonymous unique name
    Anon(Vec<Substitution>),
    /// $(eval expr) - Evaluate simple expression
    Eval(Vec<Substitution>),
}

impl Substitution {
    /// Resolve substitution to string value
    pub fn resolve(&self, context: &LaunchContext) -> Result<String, SubstitutionError> {
        match self {
            Substitution::Text(s) => Ok(s.clone()),
            Substitution::LaunchConfiguration(name_subs) => {
                let name = resolve_substitutions(name_subs, context)?;
                // Use lenient resolution to allow variables with unresolved nested substitutions
                // This is important for static parsing where not all packages may be available
                context
                    .get_configuration_lenient(&name)
                    .ok_or(SubstitutionError::UndefinedVariable(name))
            }
            Substitution::EnvironmentVariable { name, default } => {
                let name_str = resolve_substitutions(name, context)?;
                // Check context environment first, then process environment
                if let Some(value) = context.get_environment_variable(&name_str) {
                    return Ok(value);
                }
                std::env::var(&name_str).or_else(|_| {
                    if let Some(default_subs) = default {
                        resolve_substitutions(default_subs, context)
                    } else {
                        Err(SubstitutionError::UndefinedEnvVar(name_str))
                    }
                })
            }
            Substitution::OptionalEnvironmentVariable { name, default } => {
                // Never errors - returns default or empty string if not set
                let name_str = resolve_substitutions(name, context)?;
                // Check context environment first, then process environment
                if let Some(value) = context.get_environment_variable(&name_str) {
                    return Ok(value);
                }
                Ok(std::env::var(&name_str).unwrap_or_else(|_| {
                    if let Some(default_subs) = default {
                        resolve_substitutions(default_subs, context)
                            .unwrap_or_else(|_| String::new())
                    } else {
                        String::new()
                    }
                }))
            }
            Substitution::Command { cmd, error_mode } => {
                let cmd_str = resolve_substitutions(cmd, context)?;
                execute_command(&cmd_str, error_mode)
            }
            Substitution::FindPackageShare(package_subs) => {
                let package_name = resolve_substitutions(package_subs, context)?;
                find_package_share(&package_name)
                    .ok_or(SubstitutionError::PackageNotFound(package_name))
            }
            Substitution::Dirname => context
                .current_dir()
                .and_then(|p| p.to_str().map(String::from))
                .ok_or_else(|| {
                    SubstitutionError::InvalidSubstitution(
                        "dirname: no current file set".to_string(),
                    )
                }),
            Substitution::Filename => context.current_filename().ok_or_else(|| {
                SubstitutionError::InvalidSubstitution("filename: no current file set".to_string())
            }),
            Substitution::Anon(name_subs) => {
                // Generate a unique anonymous name
                // Format: name_<timestamp>_<random>
                let name = resolve_substitutions(name_subs, context)?;
                use std::time::{SystemTime, UNIX_EPOCH};
                let timestamp = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_micros();
                let random: u32 = rand::random();
                Ok(format!("{}_{:x}_{:x}", name, timestamp, random))
            }
            Substitution::Eval(expr_subs) => {
                // Resolve the expression string first
                let expr = resolve_substitutions(expr_subs, context)?;
                // Evaluate the expression
                evaluate_expression(&expr)
            }
        }
    }
}

/// Common ROS 2 distribution names, tried as fallback when `ROS_DISTRO` is unset.
pub const KNOWN_ROS_DISTROS: &[&str] = &["jazzy", "iron", "humble", "galactic", "foxy"];

/// Global package resolution cache
///
/// Thread-safe, lock-free reads, bounded by actual ROS packages.
/// Expected size: ~50 packages × ~200 bytes/entry = ~10KB total.
static PACKAGE_CACHE: Lazy<DashMap<String, String>> = Lazy::new(DashMap::new);

/// Find ROS 2 package share directory with caching
pub fn find_package_share(package_name: &str) -> Option<String> {
    // Fast path: Check cache (lock-free read)
    if let Some(entry) = PACKAGE_CACHE.get(package_name) {
        log::trace!("Package cache hit: {}", package_name);
        return Some(entry.value().clone());
    }

    log::debug!("Package cache miss: {}", package_name);

    // Slow path: Expensive filesystem lookup
    let result = find_package_share_uncached(package_name)?;

    // Cache result
    PACKAGE_CACHE.insert(package_name.to_string(), result.clone());
    Some(result)
}

/// Find ROS 2 package share directory (uncached implementation)
fn find_package_share_uncached(package_name: &str) -> Option<String> {
    // Try ROS_DISTRO environment variable first
    if let Ok(distro) = std::env::var("ROS_DISTRO") {
        let share_path = format!("/opt/ros/{}/share/{}", distro, package_name);
        if std::path::Path::new(&share_path).exists() {
            return Some(share_path);
        }
    }

    // Fallback: Try common ROS 2 distributions
    for distro in KNOWN_ROS_DISTROS {
        let share_path = format!("/opt/ros/{}/share/{}", distro, package_name);
        if std::path::Path::new(&share_path).exists() {
            return Some(share_path);
        }
    }

    // Try AMENT_PREFIX_PATH
    if let Ok(prefix_path) = std::env::var("AMENT_PREFIX_PATH") {
        for prefix in prefix_path.split(':') {
            let share_path = format!("{}/share/{}", prefix, package_name);
            if std::path::Path::new(&share_path).exists() {
                return Some(share_path);
            }
        }
    }

    None
}

/// Execute shell command and capture output.
///
/// # Security
/// This function executes arbitrary shell commands. Gated behind the
/// `ALLOW_COMMANDS` flag — returns an error when disabled.
/// Disable with `block_command_substitution(true)` or `--block-commands` CLI flag.
fn execute_command(cmd: &str, error_mode: &CommandErrorMode) -> Result<String, SubstitutionError> {
    use std::process::Command;

    if BLOCK_COMMANDS.load(Ordering::Relaxed) {
        return Err(SubstitutionError::CommandFailed(format!(
            "$(command {}) blocked: command substitutions are disabled via --block-commands.",
            cmd
        )));
    }
    log::warn!(
        "Executing $(command {}) — command substitutions run arbitrary shell commands. \
         Use --block-commands to reject them.",
        cmd
    );

    // Use bash instead of sh to ensure environment variables are preserved
    // (sh is often dash which doesn't source the same RC files)
    let output = Command::new("bash")
        .arg("-c")
        .arg(cmd)
        .output()
        .map_err(|e| {
            SubstitutionError::CommandFailed(format!("Failed to execute '{}': {}", cmd, e))
        })?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        let error_msg = if stderr.trim().is_empty() {
            format!(
                "Command '{}' failed with exit code {}",
                cmd,
                output.status.code().unwrap_or(-1)
            )
        } else {
            format!(
                "Command '{}' failed with exit code {}: {}",
                cmd,
                output.status.code().unwrap_or(-1),
                stderr.trim()
            )
        };

        // Handle error based on error mode
        match error_mode {
            CommandErrorMode::Strict => {
                // Fail on error
                return Err(SubstitutionError::CommandFailed(error_msg));
            }
            CommandErrorMode::Warn => {
                // Log warning but continue with stdout
                log::warn!("Command failed: {}", error_msg);
            }
            CommandErrorMode::Ignore => {
                // Silently ignore errors
            }
        }
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    // Trim whitespace from output as per ROS 2 behavior
    Ok(stdout.trim().to_string())
}

/// Resolve list of substitutions to single string
pub fn resolve_substitutions(
    subs: &[Substitution],
    context: &LaunchContext,
) -> Result<String, SubstitutionError> {
    let mut result = String::new();
    for sub in subs {
        result.push_str(&sub.resolve(context)?);
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::{super::eval::is_string_concatenation, *};

    /// Ensure command substitutions are enabled for tests (they are by default).
    fn enable_commands() {
        block_command_substitution(false);
    }

    #[test]
    fn test_text_substitution() {
        let sub = Substitution::Text("hello".to_string());
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "hello");
    }

    #[test]
    fn test_launch_configuration() {
        let sub = Substitution::LaunchConfiguration(vec![Substitution::Text("my_var".to_string())]);
        let mut context = LaunchContext::new();
        context.set_configuration("my_var".to_string(), "value123".to_string());
        assert_eq!(sub.resolve(&context).unwrap(), "value123");
    }

    #[test]
    fn test_undefined_variable() {
        let sub =
            Substitution::LaunchConfiguration(vec![Substitution::Text("undefined".to_string())]);
        let context = LaunchContext::new();
        assert!(sub.resolve(&context).is_err());
    }

    #[test]
    fn test_env_var() {
        unsafe { std::env::set_var("TEST_VAR", "test_value") };
        let sub = Substitution::EnvironmentVariable {
            name: vec![Substitution::Text("TEST_VAR".to_string())],
            default: None,
        };
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "test_value");
    }

    #[test]
    fn test_env_var_with_default() {
        let sub = Substitution::EnvironmentVariable {
            name: vec![Substitution::Text("NONEXISTENT_VAR".to_string())],
            default: Some(vec![Substitution::Text("default_value".to_string())]),
        };
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "default_value");
    }

    #[test]
    fn test_resolve_multiple() {
        let subs = vec![
            Substitution::Text("Hello ".to_string()),
            Substitution::LaunchConfiguration(vec![Substitution::Text("name".to_string())]),
            Substitution::Text("!".to_string()),
        ];
        let mut context = LaunchContext::new();
        context.set_configuration("name".to_string(), "World".to_string());
        assert_eq!(
            resolve_substitutions(&subs, &context).unwrap(),
            "Hello World!"
        );
    }

    #[test]
    fn test_dirname_substitution() {
        use std::path::PathBuf;

        let sub = Substitution::Dirname;
        let mut context = LaunchContext::new();
        context.set_current_file(PathBuf::from("/home/user/launch/test.launch.xml"));

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "/home/user/launch");
    }

    #[test]
    fn test_filename_substitution() {
        use std::path::PathBuf;

        let sub = Substitution::Filename;
        let mut context = LaunchContext::new();
        context.set_current_file(PathBuf::from("/home/user/launch/test.launch.xml"));

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "test.launch.xml");
    }

    #[test]
    fn test_dirname_no_file_set() {
        let sub = Substitution::Dirname;
        let context = LaunchContext::new();
        assert!(sub.resolve(&context).is_err());
    }

    #[test]
    fn test_anon_substitution() {
        let sub = Substitution::Anon(vec![Substitution::Text("my_node".to_string())]);
        let context = LaunchContext::new();

        let result = sub.resolve(&context).unwrap();
        // Should start with the name and have a timestamp and random suffix
        assert!(result.starts_with("my_node_"));
        // Should have the format name_timestamp_random
        let parts: Vec<&str> = result.split('_').collect();
        assert!(parts.len() >= 3);
    }

    #[test]
    fn test_anon_uniqueness() {
        let sub1 = Substitution::Anon(vec![Substitution::Text("node".to_string())]);
        let sub2 = Substitution::Anon(vec![Substitution::Text("node".to_string())]);
        let context = LaunchContext::new();

        let result1 = sub1.resolve(&context).unwrap();
        let result2 = sub2.resolve(&context).unwrap();

        // Should generate different names
        assert_ne!(result1, result2);
    }

    #[test]
    fn test_optenv_with_existing_var() {
        unsafe { std::env::set_var("TEST_OPTENV_VAR", "test_value") };
        let sub = Substitution::OptionalEnvironmentVariable {
            name: vec![Substitution::Text("TEST_OPTENV_VAR".to_string())],
            default: None,
        };
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "test_value");
    }

    #[test]
    fn test_optenv_with_missing_var_no_default() {
        // Make sure the variable doesn't exist
        unsafe { std::env::remove_var("NONEXISTENT_OPTENV_VAR") };
        let sub = Substitution::OptionalEnvironmentVariable {
            name: vec![Substitution::Text("NONEXISTENT_OPTENV_VAR".to_string())],
            default: None,
        };
        let context = LaunchContext::new();
        // Should return empty string, not error
        assert_eq!(sub.resolve(&context).unwrap(), "");
    }

    #[test]
    fn test_optenv_with_missing_var_with_default() {
        unsafe { std::env::remove_var("NONEXISTENT_OPTENV_VAR2") };
        let sub = Substitution::OptionalEnvironmentVariable {
            name: vec![Substitution::Text("NONEXISTENT_OPTENV_VAR2".to_string())],
            default: Some(vec![Substitution::Text("default_value".to_string())]),
        };
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "default_value");
    }

    #[test]
    fn test_optenv_vs_env_behavior() {
        unsafe { std::env::remove_var("MISSING_VAR_TEST") };

        // optenv should not error
        let optenv = Substitution::OptionalEnvironmentVariable {
            name: vec![Substitution::Text("MISSING_VAR_TEST".to_string())],
            default: None,
        };
        let context = LaunchContext::new();
        assert!(optenv.resolve(&context).is_ok());

        // env should error without default
        let env = Substitution::EnvironmentVariable {
            name: vec![Substitution::Text("MISSING_VAR_TEST".to_string())],
            default: None,
        };
        assert!(env.resolve(&context).is_err());
    }

    #[test]
    fn test_command_simple() {
        enable_commands();
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("echo hello".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "hello");
    }

    #[test]
    fn test_command_with_output_trimming() {
        enable_commands();
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("echo '  spaces  '".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        // Output should be trimmed
        assert_eq!(result, "spaces");
    }

    #[test]
    fn test_command_with_newlines() {
        enable_commands();
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("printf 'line1\\nline2\\n'".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        // Trailing newline should be trimmed
        assert_eq!(result, "line1\nline2");
    }

    #[test]
    fn test_command_failed() {
        enable_commands();
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("exit 1".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        assert!(result.is_err());
    }

    #[test]
    fn test_command_with_args() {
        enable_commands();
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("echo foo bar".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "foo bar");
    }

    #[test]
    fn test_command_pwd() {
        enable_commands();
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("pwd".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        // Should return some directory path (non-empty)
        assert!(!result.is_empty());
        assert!(result.starts_with('/'));
    }

    #[test]
    fn test_command_env_access() {
        enable_commands();
        unsafe { std::env::set_var("TEST_CMD_VAR", "test_value") };
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("echo $TEST_CMD_VAR".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "test_value");
    }

    // Nested substitution resolution tests
    #[test]
    fn test_resolve_nested_var_in_var() {
        // $(var $(var name)_config) where name=robot, robot_config=my_robot.yaml
        let sub = Substitution::LaunchConfiguration(vec![
            Substitution::LaunchConfiguration(vec![Substitution::Text("name".to_string())]),
            Substitution::Text("_config".to_string()),
        ]);

        let mut context = LaunchContext::new();
        context.set_configuration("name".to_string(), "robot".to_string());
        context.set_configuration("robot_config".to_string(), "my_robot.yaml".to_string());

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "my_robot.yaml");
    }

    #[test]
    fn test_resolve_nested_env_in_var() {
        // $(var $(env ROBOT_NAME)_config) where ROBOT_NAME=turtlebot, turtlebot_config=tb3.yaml
        unsafe { std::env::set_var("TEST_ROBOT_NAME", "turtlebot") };

        let sub = Substitution::LaunchConfiguration(vec![
            Substitution::EnvironmentVariable {
                name: vec![Substitution::Text("TEST_ROBOT_NAME".to_string())],
                default: None,
            },
            Substitution::Text("_config".to_string()),
        ]);

        let mut context = LaunchContext::new();
        context.set_configuration("turtlebot_config".to_string(), "tb3.yaml".to_string());

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "tb3.yaml");
    }

    #[test]
    fn test_resolve_nested_var_in_env_default() {
        // $(env MISSING_VAR $(var my_default)) where my_default=fallback_value
        let sub = Substitution::EnvironmentVariable {
            name: vec![Substitution::Text("NONEXISTENT_NESTED_VAR".to_string())],
            default: Some(vec![Substitution::LaunchConfiguration(vec![
                Substitution::Text("my_default".to_string()),
            ])]),
        };

        let mut context = LaunchContext::new();
        context.set_configuration("my_default".to_string(), "fallback_value".to_string());

        unsafe { std::env::remove_var("NONEXISTENT_NESTED_VAR") };
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "fallback_value");
    }

    #[test]
    fn test_resolve_nested_var_in_command() {
        enable_commands();
        // $(command echo $(var greeting)) where greeting=hello
        let sub = Substitution::Command {
            cmd: vec![
                Substitution::Text("echo ".to_string()),
                Substitution::LaunchConfiguration(vec![Substitution::Text("greeting".to_string())]),
            ],
            error_mode: CommandErrorMode::Strict,
        };

        let mut context = LaunchContext::new();
        context.set_configuration("greeting".to_string(), "hello".to_string());

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "hello");
    }

    #[test]
    fn test_resolve_triple_nested() {
        // $(var $(env $(var prefix)_NAME)_suffix)
        // prefix=ROBOT, ROBOT_NAME=turtlebot, turtlebot_suffix=final_value
        unsafe { std::env::set_var("TEST_ROBOT_NAME_TRIPLE", "turtlebot") };

        let sub = Substitution::LaunchConfiguration(vec![
            Substitution::EnvironmentVariable {
                name: vec![
                    Substitution::LaunchConfiguration(vec![Substitution::Text(
                        "prefix".to_string(),
                    )]),
                    Substitution::Text("_NAME_TRIPLE".to_string()),
                ],
                default: None,
            },
            Substitution::Text("_suffix".to_string()),
        ]);

        let mut context = LaunchContext::new();
        context.set_configuration("prefix".to_string(), "TEST_ROBOT".to_string());
        context.set_configuration("turtlebot_suffix".to_string(), "final_value".to_string());

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "final_value");
    }

    #[test]
    fn test_resolve_nested_in_optenv() {
        // $(optenv MISSING $(var backup)) where backup=default_val
        let sub = Substitution::OptionalEnvironmentVariable {
            name: vec![Substitution::Text("MISSING_OPTENV_NESTED".to_string())],
            default: Some(vec![Substitution::LaunchConfiguration(vec![
                Substitution::Text("backup".to_string()),
            ])]),
        };

        let mut context = LaunchContext::new();
        context.set_configuration("backup".to_string(), "default_val".to_string());

        unsafe { std::env::remove_var("MISSING_OPTENV_NESTED") };
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "default_val");
    }

    #[test]
    fn test_resolve_complex_nested_string() {
        // "prefix_$(var $(env TYPE)_name)_suffix" where TYPE=robot, robot_name=turtlebot
        unsafe { std::env::set_var("TEST_TYPE_NESTED", "robot") };

        let subs = vec![
            Substitution::Text("prefix_".to_string()),
            Substitution::LaunchConfiguration(vec![
                Substitution::EnvironmentVariable {
                    name: vec![Substitution::Text("TEST_TYPE_NESTED".to_string())],
                    default: None,
                },
                Substitution::Text("_name".to_string()),
            ]),
            Substitution::Text("_suffix".to_string()),
        ];

        let mut context = LaunchContext::new();
        context.set_configuration("robot_name".to_string(), "turtlebot".to_string());

        let result = resolve_substitutions(&subs, &context).unwrap();
        assert_eq!(result, "prefix_turtlebot_suffix");
    }

    #[test]
    fn test_resolve_nested_anon() {
        // $(anon $(var base_name)) where base_name=my_node
        let sub = Substitution::Anon(vec![Substitution::LaunchConfiguration(vec![
            Substitution::Text("base_name".to_string()),
        ])]);

        let mut context = LaunchContext::new();
        context.set_configuration("base_name".to_string(), "my_node".to_string());

        let result = sub.resolve(&context).unwrap();
        assert!(result.starts_with("my_node_"));
        let parts: Vec<&str> = result.split('_').collect();
        assert!(parts.len() >= 3);
    }

    #[test]
    fn test_resolve_nested_command_with_env() {
        enable_commands();
        // $(command echo $(env USER))
        unsafe { std::env::set_var("TEST_USER_NESTED", "testuser") };

        let sub = Substitution::Command {
            cmd: vec![
                Substitution::Text("echo ".to_string()),
                Substitution::EnvironmentVariable {
                    name: vec![Substitution::Text("TEST_USER_NESTED".to_string())],
                    default: None,
                },
            ],
            error_mode: CommandErrorMode::Strict,
        };

        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "testuser");
    }

    // Eval tests
    #[test]
    fn test_eval_simple_addition() {
        let sub = Substitution::Eval(vec![Substitution::Text("1 + 2".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "3");
    }

    #[test]
    fn test_eval_multiplication() {
        let sub = Substitution::Eval(vec![Substitution::Text("4 * 5".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "20");
    }

    #[test]
    fn test_eval_division() {
        let sub = Substitution::Eval(vec![Substitution::Text("10 / 2".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "5");
    }

    #[test]
    fn test_eval_modulo() {
        let sub = Substitution::Eval(vec![Substitution::Text("10 % 3".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "1");
    }

    #[test]
    fn test_eval_subtraction() {
        let sub = Substitution::Eval(vec![Substitution::Text("10 - 3".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "7");
    }

    #[test]
    fn test_eval_with_parentheses() {
        let sub = Substitution::Eval(vec![Substitution::Text("(3 + 4) * 2".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "14");
    }

    #[test]
    fn test_eval_order_of_operations() {
        let sub = Substitution::Eval(vec![Substitution::Text("2 + 3 * 4".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "14");
    }

    #[test]
    fn test_eval_negative_numbers() {
        let sub = Substitution::Eval(vec![Substitution::Text("-5 + 10".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "5");
    }

    #[test]
    fn test_eval_float_result() {
        let sub = Substitution::Eval(vec![Substitution::Text("5 / 2".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "2.5");
    }

    #[test]
    fn test_eval_with_var_substitution() {
        // $(eval $(var x) + 5) where x=10
        let sub = Substitution::Eval(vec![
            Substitution::LaunchConfiguration(vec![Substitution::Text("x".to_string())]),
            Substitution::Text(" + 5".to_string()),
        ]);

        let mut context = LaunchContext::new();
        context.set_configuration("x".to_string(), "10".to_string());

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "15");
    }

    #[test]
    fn test_eval_complex_with_vars() {
        // $(eval ($(var a) + $(var b)) * 2) where a=3, b=4
        let sub = Substitution::Eval(vec![
            Substitution::Text("(".to_string()),
            Substitution::LaunchConfiguration(vec![Substitution::Text("a".to_string())]),
            Substitution::Text(" + ".to_string()),
            Substitution::LaunchConfiguration(vec![Substitution::Text("b".to_string())]),
            Substitution::Text(") * 2".to_string()),
        ]);

        let mut context = LaunchContext::new();
        context.set_configuration("a".to_string(), "3".to_string());
        context.set_configuration("b".to_string(), "4".to_string());

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "14");
    }

    #[test]
    fn test_eval_division_by_zero() {
        let sub = Substitution::Eval(vec![Substitution::Text("1 / 0".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        assert!(result.is_err());
    }

    #[test]
    fn test_eval_invalid_expression() {
        // "1 + + 2" is valid Python (unary +), so with the Python fallback it evaluates to 3
        let sub = Substitution::Eval(vec![Substitution::Text("1 + + 2".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "3");

        // Truly invalid expression
        let sub = Substitution::Eval(vec![Substitution::Text("@@@".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        assert!(result.is_err());
    }

    // String comparison tests
    #[test]
    fn test_eval_string_equals_true() {
        let sub = Substitution::Eval(vec![Substitution::Text(
            "'elastic_band' == 'elastic_band'".to_string(),
        )]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "true");
    }

    #[test]
    fn test_eval_string_equals_false() {
        let sub = Substitution::Eval(vec![Substitution::Text("'foo' == 'bar'".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "false");
    }

    #[test]
    fn test_eval_string_not_equals_true() {
        let sub = Substitution::Eval(vec![Substitution::Text("'foo' != 'bar'".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "true");
    }

    #[test]
    fn test_eval_string_not_equals_false() {
        let sub = Substitution::Eval(vec![Substitution::Text("'foo' != 'foo'".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "false");
    }

    #[test]
    fn test_eval_string_with_outer_quotes() {
        // Expression wrapped in double quotes (from XML attribute)
        let sub = Substitution::Eval(vec![Substitution::Text(
            "\"'elastic_band' == 'elastic_band'\"".to_string(),
        )]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "true");
    }

    #[test]
    fn test_eval_string_double_quotes() {
        let sub = Substitution::Eval(vec![Substitution::Text("\"foo\" == \"foo\"".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "true");
    }

    // Command error mode execution tests
    #[test]
    fn test_command_strict_mode_fails_on_error() {
        enable_commands();
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("exit 1".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        // Should fail with Strict mode
        assert!(result.is_err());
    }

    #[test]
    fn test_command_warn_mode_continues_on_error() {
        enable_commands();
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("echo output && exit 1".to_string())],
            error_mode: CommandErrorMode::Warn,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        // Should succeed with Warn mode even though command failed
        assert!(result.is_ok());
        // Should return stdout (output)
        assert_eq!(result.unwrap(), "output");
    }

    #[test]
    fn test_command_ignore_mode_continues_on_error() {
        enable_commands();
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("echo output && exit 1".to_string())],
            error_mode: CommandErrorMode::Ignore,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        // Should succeed with Ignore mode even though command failed
        assert!(result.is_ok());
        // Should return stdout (output)
        assert_eq!(result.unwrap(), "output");
    }

    #[test]
    fn test_command_all_modes_succeed_on_success() {
        enable_commands();
        // All modes should succeed when command succeeds
        for error_mode in [
            CommandErrorMode::Strict,
            CommandErrorMode::Warn,
            CommandErrorMode::Ignore,
        ] {
            let sub = Substitution::Command {
                cmd: vec![Substitution::Text("echo success".to_string())],
                error_mode,
            };
            let context = LaunchContext::new();
            let result = sub.resolve(&context);
            assert!(result.is_ok());
            assert_eq!(result.unwrap(), "success");
        }
    }

    // String concatenation tests
    #[test]
    fn test_eval_string_concat_simple() {
        let sub = Substitution::Eval(vec![Substitution::Text("'foo' + 'bar'".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "foobar");
    }

    #[test]
    fn test_eval_string_concat_multi() {
        let sub = Substitution::Eval(vec![Substitution::Text(
            "'[' + 'Module1, ' + 'Module2, ' + ']'".to_string(),
        )]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "[Module1, Module2, ]");
    }

    #[test]
    fn test_eval_string_concat_with_var() {
        // $(eval '$(var prefix)' + '_suffix') — var already resolved by the time eval sees it
        let sub = Substitution::Eval(vec![
            Substitution::Text("'".to_string()),
            Substitution::LaunchConfiguration(vec![Substitution::Text("prefix".to_string())]),
            Substitution::Text("' + '_suffix'".to_string()),
        ]);
        let mut context = LaunchContext::new();
        context.set_configuration("prefix".to_string(), "hello".to_string());
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "hello_suffix");
    }

    #[test]
    fn test_eval_string_concat_empty_quotes() {
        let sub = Substitution::Eval(vec![Substitution::Text(
            "\"\" + 'content' + \"\"".to_string(),
        )]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "content");
    }

    #[test]
    fn test_is_string_concatenation() {
        assert!(is_string_concatenation("'a' + 'b'"));
        assert!(is_string_concatenation("\"a\" + \"b\""));
        assert!(is_string_concatenation("'[' + 'Module, ' + ']'"));
        assert!(!is_string_concatenation("1 + 2"));
        assert!(!is_string_concatenation("(3 + 4) * 2"));
        assert!(!is_string_concatenation("'hello'"));
    }
}
