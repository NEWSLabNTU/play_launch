//! Verbosity flag shared by the pipeline.

use std::sync::OnceLock;

/// Global flag for verbose logging.
static VERBOSE_LOGGING: OnceLock<bool> = OnceLock::new();

/// Whether verbose logging is enabled.
pub fn is_verbose() -> bool {
    VERBOSE_LOGGING.get().copied().unwrap_or(false)
}

/// Record the verbosity the caller resolved.
///
/// Takes a plain `bool`. This used to take the consumer's whole `Options` enum
/// and pattern-match every subcommand just to read one flag — a library
/// reaching into its own consumer's CLI types, which is exactly the inversion
/// the three-layer split exists to remove (RFC-0060). Deciding which
/// subcommand carries `--verbose` is the CLI's job.
pub fn init_verbose(verbose: bool) {
    let _ = VERBOSE_LOGGING.set(verbose);
}
