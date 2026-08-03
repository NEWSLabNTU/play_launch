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
use eyre::{Result, bail};

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

pub fn replay_renamed(args: &UpArgs) -> Result<()> {
    bail!(
        "`replay` was renamed to `up` in 0.9.0.\n       play_launch up {}",
        model_arg(args)
    )
}

pub fn check_removed(args: &CheckArgs) -> Result<()> {
    let (target, launch_args) = target_and_launch_args(
        &args.package_or_path,
        &args.launch_file,
        &args.launch_arguments,
    );
    bail!(
        "`check` was removed in 0.9.0. Two replacements:\n       \
         play_launch launch {target}{launch_args} --check   (pass/fail gate)\n       \
         ros-launch-resolve check {target}{launch_args}   (diagnostics: --format, \
         --rule, --explain, --export-graph; no ROS install needed)"
    )
}

pub fn resolve_removed(args: &ResolveArgs) -> Result<()> {
    let (target, launch_args) = target_and_launch_args(
        &args.package_or_path,
        &args.launch_file,
        &args.launch_arguments,
    );
    bail!(
        "`resolve` was removed in 0.9.0 — it delegated to layer 2 since \
         RFC-0060.\n       ros-launch-resolve resolve {target}{launch_args} -o {}",
        args.out
    )
}
