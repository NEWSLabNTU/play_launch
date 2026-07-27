//! Command handlers

#[cfg(feature = "runtime")]
pub mod capabilities;
pub(crate) mod common;
pub mod context;
pub mod contract;
pub mod dump;
pub mod launch;
#[cfg(feature = "runtime")]
pub mod manifest;
pub mod plot;
#[cfg(feature = "runtime")]
pub mod replay;
pub mod resolve;
#[cfg(feature = "runtime")]
pub mod run;
#[cfg(feature = "runtime")]
pub(crate) mod signal_handler;

// Re-export command handlers
#[cfg(feature = "runtime")]
pub use capabilities::{handle_setcap, handle_verify};
pub use context::handle_context;
pub use contract::handle_contract_eject;
pub use dump::handle_dump;
#[cfg(feature = "runtime")]
pub use launch::handle_launch;
#[cfg(feature = "runtime")]
pub use manifest::handle_check_manifest;
pub use plot::handle_plot;
#[cfg(feature = "runtime")]
pub use replay::handle_replay;
pub use resolve::handle_resolve;
#[cfg(feature = "runtime")]
pub use run::handle_run;

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
