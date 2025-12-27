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
        crate::cli::options::Command::Launch(args) => args.common.verbose,
        crate::cli::options::Command::Run(args) => args.common.verbose,
        crate::cli::options::Command::Dump(_) => false, // Dump doesn't use CommonOptions
        crate::cli::options::Command::Replay(args) => args.common.verbose,
        crate::cli::options::Command::Plot(_) => false, // Plot doesn't use CommonOptions
        crate::cli::options::Command::SetcapIoHelper => false,
        crate::cli::options::Command::VerifyIoHelper => false,
    }
}
