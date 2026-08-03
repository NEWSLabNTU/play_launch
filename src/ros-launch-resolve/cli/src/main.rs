//! `ros-launch-resolve` — resolve a ROS 2 launch tree into a SystemModel.
//!
//! The launch-tree verbs that need no ROS graph: `resolve`, `dump`, `check`,
//! `contract`, `plot`. Running, replaying and supervising nodes stayed in
//! `play_launch`, which is the layer that legitimately needs rclrs and a
//! colcon-built message set (RFC-0060).

mod check;
mod common;
mod contract;
mod dump;
mod launch;
mod options;
mod plot;
mod resolve;

use clap::Parser;
use options::{Command, Options};

fn main() -> eyre::Result<()> {
    // Initialize a tracing subscriber so `info!`/`warn!` calls in the shared
    // `ros-launch-resolve` crate (e.g. `manifest_loader::load_manifests`'s
    // "Loaded N manifest(s) [...]" summary) are actually emitted -- without
    // this, `tracing`'s macros are no-ops (no subscriber registered) and
    // those lines silently vanish. Mirrors `play_launch`'s own init
    // (`src/play_launch/src/main.rs`): `RUST_LOG` takes precedence; default
    // to INFO otherwise. Written to stderr, same as this CLI's `eprintln!`
    // diagnostics, so callers see one interleaved stream.
    if std::env::var("RUST_LOG").is_ok() {
        tracing_subscriber::fmt()
            .with_writer(std::io::stderr)
            .init();
    } else {
        tracing_subscriber::fmt()
            .with_max_level(tracing::Level::INFO)
            .with_writer(std::io::stderr)
            .init();
    }

    let opts = Options::parse();
    match &opts.command {
        Command::Resolve(args) => resolve::handle_resolve(args),
        Command::Dump(args) => dump::handle_dump(args),
        Command::Check(args) => check::handle_check(args),
        // `contract` is a subcommand group; today `eject` is its only verb.
        Command::Contract(args) => match &args.subcommand {
            options::ContractSubcommand::Eject(eject) => contract::handle_contract_eject(eject),
        },
        Command::Plot(args) => plot::handle_plot(args),
    }
}
