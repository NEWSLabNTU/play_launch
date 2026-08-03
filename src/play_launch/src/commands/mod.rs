//! Command handlers

pub mod capabilities;
pub mod check;
pub(crate) mod common;
pub mod context;
pub mod contract;
pub mod dump;
pub mod launch;
pub mod migrated;
pub mod plot;
pub mod resolve;
pub mod run;
pub(crate) mod signal_handler;
pub mod up;

// Re-export command handlers
pub use capabilities::{handle_setcap, handle_verify};
pub use check::handle_check;
pub use context::handle_context;
pub use contract::handle_contract_eject;
pub use dump::handle_dump;
pub use launch::handle_launch;
pub use plot::handle_plot;
pub use resolve::handle_resolve;
pub use run::handle_run;
pub use up::handle_up;

// `parse_launch_arguments` was a byte-identical copy of the library's. One
// implementation, in the library, re-exported here so call sites read
// unchanged (CLAUDE.md: shared logic goes in the library, never in a CLI).
pub(crate) use ros_launch_resolve::verbs::parse_launch_arguments;
