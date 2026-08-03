//! Handlers for verbs removed or renamed in 0.9.0.
//!
//! These variants are hidden from `--help` but still PARSE, accepting their
//! old arguments, so the error can echo the user's own invocation back in
//! the new form.
//!
//! That is the whole point. nano-ros issue 0285 was a subcommand vanishing
//! and clap answering `unrecognized subcommand 'resolve'` from inside a
//! cmake configure, which took down every platform's fixture build. The
//! failure was the error, not the removal.
//!
//! DELETE THIS MODULE AT 1.0.0.

use crate::cli::options::{CheckArgs, ResolveArgs, UpArgs};
use eyre::Result;

/// Print `msg` to stderr and exit non-zero.
///
/// Deliberately NOT `eyre::bail!`: an eyre error carries a `Location:
/// src/play_launch/src/commands/migrated.rs:NN` footer, and in the exact
/// scenario these handlers exist for — the error surfacing from inside a
/// cmake configure (nano-ros issue 0285) — a source location reads as an
/// internal crash rather than as migration guidance. Same message, same
/// non-zero exit, no footer.
fn migration_error(msg: String) -> ! {
    eprintln!("Error: {msg}");
    std::process::exit(1)
}

/// Reconstruct the model argument the user passed, for echoing back.
fn model_arg(args: &UpArgs) -> String {
    match (&args.model_path, &args.model) {
        (Some(p), _) => p.display().to_string(),
        (None, Some(p)) => format!("--model {}", p.display()),
        (None, None) => "<model.yaml>".to_string(),
    }
}

/// Reconstruct `PKG FILE` (or just `PKG`/path) and the trailing
/// `KEY:=VALUE` launch arguments the user passed, for echoing back against
/// a replacement binary. Shared by `check_removed` and `resolve_removed`,
/// whose old argument shapes (package_or_path/launch_file/launch_arguments)
/// are identical.
fn target_and_launch_args(
    package_or_path: &str,
    launch_file: &Option<String>,
    launch_arguments: &[String],
) -> (String, String) {
    let target = match launch_file {
        Some(f) => format!("{package_or_path} {f}"),
        None => package_or_path.to_string(),
    };
    let launch_args = if launch_arguments.is_empty() {
        String::new()
    } else {
        format!(" {}", launch_arguments.join(" "))
    };
    (target, launch_args)
}

/// Reconstruct the diagnostic/contract flags the user passed to the old
/// `check`, so the replacement invocation we print is the one they can
/// actually paste. Every one of these flags exists verbatim on
/// `ros-launch-resolve check`, so the echo is a faithful translation, not an
/// approximation. Non-default values only — an echo padded with defaults the
/// user never typed is noise.
fn check_diagnostic_flags(args: &CheckArgs) -> String {
    let mut parts: Vec<String> = Vec::new();
    if let Some(p) = &args.contracts {
        parts.push(format!("--contracts {}", p.display()));
    }
    if args.no_provider_contracts {
        parts.push("--no-provider-contracts".to_string());
    }
    if let Some(p) = &args.sched {
        parts.push(format!("--sched {}", p.display()));
    }
    if args.target != "posix" {
        parts.push(format!("--target {}", args.target));
    }
    if args.format != "terminal" {
        parts.push(format!("--format {}", args.format));
    }
    for rule in &args.rule {
        parts.push(format!("--rule {rule}"));
    }
    if args.explain {
        parts.push("--explain".to_string());
    }
    if let Some(p) = &args.export_graph {
        parts.push(format!("--export-graph {}", p.display()));
    }
    if parts.is_empty() {
        String::new()
    } else {
        format!(" {}", parts.join(" "))
    }
}

pub fn replay_renamed(args: &UpArgs) -> Result<()> {
    migration_error(format!(
        "`replay` was renamed to `up` in 0.9.0.\n       play_launch up {}",
        model_arg(args)
    ))
}

pub fn check_removed(args: &CheckArgs) -> Result<()> {
    let (target, launch_args) = target_and_launch_args(
        &args.package_or_path,
        &args.launch_file,
        &args.launch_arguments,
    );
    let flags = check_diagnostic_flags(args);
    migration_error(format!(
        "`check` was removed in 0.9.0. Two replacements:\n       \
         play_launch launch {target}{launch_args} --check   (pass/fail gate)\n       \
         ros-launch-resolve check {target}{launch_args}{flags}\n       \
         (diagnostics, no ROS install needed)"
    ))
}

pub fn resolve_removed(args: &ResolveArgs) -> Result<()> {
    let (target, launch_args) = target_and_launch_args(
        &args.package_or_path,
        &args.launch_file,
        &args.launch_arguments,
    );
    migration_error(format!(
        "`resolve` was removed in 0.9.0 — it delegated to layer 2 since \
         RFC-0060.\n       ros-launch-resolve resolve {target}{launch_args} -o {}",
        args.out
    ))
}
