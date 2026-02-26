//! Command handlers

pub(crate) mod common;
pub mod dump;
pub mod io_helper;
pub mod launch;
pub mod plot;
pub mod replay;
pub mod run;
pub(crate) mod signal_handler;

#[cfg(feature = "wasm-compile")]
pub mod compile;
#[cfg(feature = "wasm-exec")]
pub mod exec;

// Re-export command handlers
pub use dump::handle_dump;
pub use io_helper::{handle_setcap_io_helper, handle_verify_io_helper};
pub use launch::handle_launch;
pub use plot::handle_plot;
pub use replay::handle_replay;
pub use run::handle_run;

#[cfg(feature = "wasm-compile")]
pub use compile::handle_compile;
#[cfg(feature = "wasm-exec")]
pub use exec::handle_exec;

use std::collections::HashMap;
use tracing::warn;

/// Parse KEY:=VALUE launch arguments into a HashMap.
pub(crate) fn parse_launch_arguments(args: &[String]) -> HashMap<String, String> {
    args.iter()
        .filter_map(|arg| {
            let parts: Vec<&str> = arg.splitn(2, ":=").collect();
            if parts.len() == 2 {
                Some((parts[0].to_string(), parts[1].to_string()))
            } else {
                warn!(
                    "Ignoring invalid launch argument (expected KEY:=VALUE): {}",
                    arg
                );
                None
            }
        })
        .collect()
}
