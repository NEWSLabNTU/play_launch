//! Logging utilities

use crate::cli::options::Options;
use std::sync::OnceLock;

/// Global flag for verbose logging
static VERBOSE_LOGGING: OnceLock<bool> = OnceLock::new();

/// Check if verbose logging is enabled
pub fn is_verbose() -> bool {
    VERBOSE_LOGGING.get().copied().unwrap_or(false)
}

/// Initialize verbose logging flag from command-line options
pub fn init_verbose(opts: &Options) {
    let verbose = get_verbose_flag(opts);
    VERBOSE_LOGGING.set(verbose).ok();
}

/// Extract verbose flag from Options based on subcommand
fn get_verbose_flag(opts: &Options) -> bool {
    match &opts.command {
        crate::cli::options::Command::Launch(args) => args.launch.common.verbose,
        crate::cli::options::Command::Run(args) => args.common.verbose,
        crate::cli::options::Command::Up(args) => args.common.verbose,
        crate::cli::options::Command::Replay(args) => args.common.verbose, // hidden; DELETE AT 1.0.0
        crate::cli::options::Command::Setcap => false,
        crate::cli::options::Command::Verify => false,
        crate::cli::options::Command::Context(_) => false,
        // The five launch-tree verbs delegate to `ros_launch_resolve::verbs`,
        // which logs through `tracing` and is filtered by `RUST_LOG` alone —
        // this crate-local verbosity flag has no effect on them. `dump
        // launch` does carry `CommonOptions` (it shares `LaunchArgs`), but
        // honouring `--verbose` there and nowhere else in the group would be
        // an inconsistency, not a feature.
        crate::cli::options::Command::Resolve(_) => false,
        crate::cli::options::Command::Dump(_) => false,
        crate::cli::options::Command::Check(_) => false,
        crate::cli::options::Command::Plot(_) => false,
        crate::cli::options::Command::Contract(_) => false,
    }
}
