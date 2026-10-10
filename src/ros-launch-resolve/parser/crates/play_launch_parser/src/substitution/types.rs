//! Substitution types

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
    /// Ignore stderr output
    Ignore,
    /// Return stderr together with stdout
    Capture,
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
    /// $(filename) - ABSOLUTE PATH of the current launch file.
    ///
    /// Not the basename, which is what this used to return. In ROS 2 the
    /// frontend token `filename` is `ThisLaunchFile`
    /// (`@expose_substitution('filename')` in
    /// `launch/substitutions/this_launch_file.py`), whose `perform` returns
    /// `context.locals.current_launch_file_path` — the same absolute path
    /// `ThisLaunchFile()` gives a `.launch.py`. There is no second concept
    /// and no `$(this-launch-file)` token to add: the two spellings are one
    /// substitution class (issue 0041).
    Filename,
    /// $(anon name) - Generate anonymous unique name
    Anon(Vec<Substitution>),
    /// $(eval expr) - Evaluate simple expression
    Eval(Vec<Substitution>),
    /// Every other substitution `launch`'s frontends expose, by name, with
    /// its arguments: `var` with a default, `find-pkg-prefix`, `find-exec`,
    /// `exec-in-pkg`, `file-content`, `if`, `equals`, `not-equals`, `not`,
    /// `and`, `or`, `any`, `all`, `param`, `launch_log_dir`.
    Call {
        name: String,
        args: Vec<Vec<Substitution>>,
    },
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
            Substitution::Filename => context
                .current_file()
                .and_then(|p| p.to_str().map(String::from))
                .ok_or_else(|| {
                    SubstitutionError::InvalidSubstitution(
                        "filename: no current file set".to_string(),
                    )
                }),
            Substitution::Anon(name_subs) => {
                // `AnonName`: `<name>_<host>_<pid>_<random>` with `.`, `-` and
                // `:` made `_`, computed ONCE per name for the launch —
                // `$(anon x)` twice is the same name twice.
                let name = resolve_substitutions(name_subs, context)?;
                let mut names = ANON_NAMES.lock().unwrap_or_else(|e| e.into_inner());
                Ok(names
                    .entry(name.clone())
                    .or_insert_with(|| {
                        let host = std::fs::read_to_string("/proc/sys/kernel/hostname")
                            .map(|h| h.trim().to_string())
                            .unwrap_or_else(|_| "localhost".to_string());
                        let random: u64 = rand::random::<u64>() >> 1;
                        format!("{name}_{host}_{}_{random}", std::process::id())
                            .replace(['.', '-', ':'], "_")
                    })
                    .clone())
            }
            Substitution::Eval(expr_subs) => {
                // The argument's quotes were consumed by the grammar; what is
                // left is exactly the Python `launch` evaluates.
                let expr = resolve_substitutions(expr_subs, context)?;
                super::eval::evaluate_python(&expr)
            }
            Substitution::Call { name, args } => resolve_call(name, args, context),
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

/// `$(anon name)` values already computed, by name.
static ANON_NAMES: Lazy<std::sync::Mutex<std::collections::HashMap<String, String>>> =
    Lazy::new(Default::default);

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
    let mut command = Command::new("timeout");
    command
        .arg(format!("--kill-after={}", COMMAND_KILL_AFTER_SECS))
        .arg(COMMAND_TIMEOUT_SECS.to_string())
        .args(&args);
    let spawn_err = |e: std::io::Error| {
        SubstitutionError::CommandFailed(format!("Failed to execute '{}': {}", cmd, e))
    };
    let output = if *error_mode == CommandErrorMode::Capture {
        // `stderr=subprocess.STDOUT`: one stream, interleaved as written.
        use std::io::Read;
        let (mut reader, writer) = std::io::pipe().map_err(spawn_err)?;
        let mut child = command
            .stdout(writer.try_clone().map_err(spawn_err)?)
            .stderr(writer)
            .spawn()
            .map_err(spawn_err)?;
        // The command's own copies of the write end must be the only ones
        // left, or the read below never sees EOF.
        drop(command);
        let mut merged = Vec::new();
        reader.read_to_end(&mut merged).map_err(spawn_err)?;
        let status = child.wait().map_err(spawn_err)?;
        std::process::Output {
            status,
            stdout: merged,
            stderr: Vec::new(),
        }
    } else {
        command.output().map_err(spawn_err)?
    };

    // Exit code 124 = timeout killed the command
    if output.status.code() == Some(124) {
        return Err(SubstitutionError::CommandFailed(format!(
            "$(command {}) timed out after {}s",
            cmd, COMMAND_TIMEOUT_SECS
        )));
    }

    let stdout = String::from_utf8_lossy(&output.stdout).into_owned();
    let stderr = String::from_utf8_lossy(&output.stderr).into_owned();

    // `Command.perform`: a non-zero exit is always a failure; stderr output
    // from a successful command is a failure, a warning, ignored or kept,
    // per `on_stderr`. The output is returned AS IS — trailing newline
    // included — which is what `launch` substitutes.
    if !output.status.success() {
        let mut msg = format!("executed command failed. Command: {cmd}");
        if !stderr.is_empty() {
            msg.push_str(&format!("\nCaptured stderr output: {stderr}"));
        }
        return Err(SubstitutionError::CommandFailed(msg));
    }
    if !stderr.is_empty() {
        let msg = format!(
            "executed command showed stderr output. Command: {cmd}\nCaptured stderr output:\n{stderr}"
        );
        match error_mode {
            CommandErrorMode::Strict => return Err(SubstitutionError::CommandFailed(msg)),
            CommandErrorMode::Warn => log::warn!("{msg}"),
            CommandErrorMode::Ignore => {}
            CommandErrorMode::Capture => return Ok(format!("{stdout}{stderr}")),
        }
    }
    Ok(stdout)
}

/// `launch`'s boolean coercion for `if`, `not`, `and`, `or`, `any`, `all`.
fn to_bool(name: &str, value: &str) -> Result<bool, SubstitutionError> {
    match value.trim().to_lowercase().as_str() {
        "true" | "1" => Ok(true),
        "false" | "0" => Ok(false),
        _ => Err(SubstitutionError::InvalidSubstitution(format!(
            "{name}: '{value}' is not a boolean"
        ))),
    }
}

fn bool_str(b: bool) -> String {
    if b { "true" } else { "false" }.to_string()
}

/// The package's install prefix, the way `ament_index` finds it.
pub fn find_package_prefix(package: &str) -> Option<String> {
    let share = find_package_share(package)?;
    std::path::Path::new(&share)
        .parent()?
        .parent()
        .and_then(|p| p.to_str().map(String::from))
}

fn resolve_call(
    name: &str,
    args: &[Vec<Substitution>],
    context: &LaunchContext,
) -> Result<String, SubstitutionError> {
    let arg = |i: usize| resolve_substitutions(&args[i], context);
    match name {
        // `$(var name default)`: the default when the configuration is unset.
        "var" => {
            let var = arg(0)?;
            match context.get_configuration_lenient(&var) {
                Some(v) => Ok(v),
                None => arg(1),
            }
        }
        "find-pkg-prefix" => {
            let pkg = arg(0)?;
            find_package_prefix(&pkg).ok_or(SubstitutionError::PackageNotFound(pkg))
        }
        // `FindExecutable`: the first match on `PATH`.
        "find-exec" => {
            let exe = arg(0)?;
            std::env::var_os("PATH")
                .and_then(|paths| {
                    std::env::split_paths(&paths)
                        .map(|dir| dir.join(&exe))
                        .find(|p| p.is_file())
                })
                .and_then(|p| p.to_str().map(String::from))
                .ok_or_else(|| {
                    SubstitutionError::InvalidSubstitution(format!(
                        "executable '{exe}' not found on the PATH"
                    ))
                })
        }
        // `ExecutableInPackage(executable, package)`: `<prefix>/lib/<package>/<executable>`.
        "exec-in-pkg" => {
            let exe = arg(0)?;
            let pkg = arg(1)?;
            let prefix =
                find_package_prefix(&pkg).ok_or(SubstitutionError::PackageNotFound(pkg.clone()))?;
            let path = std::path::Path::new(&prefix)
                .join("lib")
                .join(&pkg)
                .join(&exe);
            if path.is_file() {
                Ok(path.to_string_lossy().into_owned())
            } else {
                Err(SubstitutionError::InvalidSubstitution(format!(
                    "executable '{exe}' not found in package '{pkg}' ({})",
                    path.display()
                )))
            }
        }
        "file-content" => {
            let path = arg(0)?;
            std::fs::read_to_string(&path).map_err(|e| {
                SubstitutionError::InvalidSubstitution(format!("file-content: {path}: {e}"))
            })
        }
        // `EqualsSubstitution`: booleans and floats compare by value.
        "equals" | "not-equals" => {
            let left = arg(0)?;
            let right = arg(1)?;
            let is_bool =
                |s: &str| matches!(s.to_lowercase().as_str(), "true" | "false" | "1" | "0");
            let truthy = |s: &str| matches!(s.to_lowercase().as_str(), "true" | "1");
            let equal = if is_bool(&left) && is_bool(&right) {
                truthy(&left) == truthy(&right)
            } else if let (Ok(l), Ok(r)) = (left.trim().parse::<f64>(), right.trim().parse::<f64>())
            {
                (l - r).abs() <= 1e-9 * l.abs().max(r.abs())
            } else {
                left == right
            };
            Ok(bool_str(if name == "equals" { equal } else { !equal }))
        }
        "not" => Ok(bool_str(!to_bool(name, &arg(0)?)?)),
        "and" => Ok(bool_str(
            to_bool(name, &arg(0)?)? && to_bool(name, &arg(1)?)?,
        )),
        "or" => Ok(bool_str(
            to_bool(name, &arg(0)?)? || to_bool(name, &arg(1)?)?,
        )),
        "any" | "all" => {
            let mut values = Vec::new();
            for i in 0..args.len() {
                values.push(to_bool(name, &arg(i)?)?);
            }
            Ok(bool_str(if name == "any" {
                values.iter().any(|b| *b)
            } else {
                values.iter().all(|b| *b)
            }))
        }
        "if" => {
            if to_bool(name, &arg(0)?)? {
                arg(1)
            } else if args.len() == 3 {
                arg(2)
            } else {
                Ok(String::new())
            }
        }
        // `launch_ros`'s `Parameter`: a global parameter set by `SetParameter`.
        "param" => {
            let param = arg(0)?;
            context.get_global_parameter(&param).ok_or_else(|| {
                SubstitutionError::InvalidSubstitution(format!(
                    "parameter '{param}' not found (only parameters set with set_parameter \
                     are visible to $(param))"
                ))
            })
        }
        // The launch's log directory: `ROS_LOG_DIR`, else `~/.ros/log`.
        "launch_log_dir" => Ok(std::env::var("ROS_LOG_DIR").unwrap_or_else(|_| {
            format!(
                "{}/.ros/log",
                std::env::var("HOME").unwrap_or_else(|_| "~".to_string())
            )
        })),
        other => Err(SubstitutionError::InvalidSubstitution(format!(
            "Unknown substitution type: {other}"
        ))),
    }
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
