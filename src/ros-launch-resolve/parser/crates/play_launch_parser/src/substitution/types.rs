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
/// Timeout for `$(command ...)` substitutions (seconds).
const COMMAND_TIMEOUT_SECS: u32 = 30;

/// Grace period before SIGKILL after SIGTERM on timeout (seconds).
const COMMAND_KILL_AFTER_SECS: u32 = 5;

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

    // Use shlex::split() for POSIX shell-style word splitting, matching ROS 2's
    // Command substitution which uses Python's shlex.split() + subprocess.run(list).
    // This correctly handles quoted arguments, e.g.:
    //   'xacro /path/to/file arg:=val'  →  ["xacro", "/path/to/file", "arg:=val"]
    let args = shlex::split(cmd.trim()).ok_or_else(|| {
        SubstitutionError::CommandFailed(format!(
            "Failed to parse command '{}': unclosed quote",
            cmd
        ))
    })?;
    if args.is_empty() {
        return Err(SubstitutionError::CommandFailed(format!(
            "Empty command in $(command {})",
            cmd
        )));
    }

    // Wrap with `timeout` to prevent indefinite hangs (e.g., xacro on missing files).
    // `timeout` handles cleanup: SIGTERM after COMMAND_TIMEOUT_SECS, SIGKILL after
    // COMMAND_KILL_AFTER_SECS grace period. Exit code 124 = timed out.
    // `output()` drains pipes correctly, avoiding deadlocks.
    let output = Command::new("timeout")
        .arg(format!("--kill-after={}", COMMAND_KILL_AFTER_SECS))
        .arg(COMMAND_TIMEOUT_SECS.to_string())
        .args(&args)
        .output()
        .map_err(|e| {
            SubstitutionError::CommandFailed(format!("Failed to execute '{}': {}", cmd, e))
        })?;

    // Exit code 124 = timeout killed the command
    if output.status.code() == Some(124) {
        return Err(SubstitutionError::CommandFailed(format!(
            "$(command {}) timed out after {}s",
            cmd, COMMAND_TIMEOUT_SECS
        )));
    }

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
