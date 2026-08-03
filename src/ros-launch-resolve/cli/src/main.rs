//! `ros-launch-resolve` — resolve a ROS 2 launch tree into a SystemModel.
//!
//! The launch-tree verbs that need no ROS graph: `resolve`, `dump`, `check`,
//! `contract`, `plot`. Running, replaying and supervising nodes stayed in
//! `play_launch`, which is the layer that legitimately needs rclrs and a
//! colcon-built message set (RFC-0060).

//! Every module here is argument mapping. The verbs themselves live in
//! `ros_launch_resolve::verbs`, shared with `play_launch` so the two CLIs
//! cannot drift apart.

mod check;
mod contract;
mod dump;
mod options;
mod plot;
mod resolve;

use clap::Parser;
use options::{Command, Options};

fn main() -> eyre::Result<()> {
    // Report errors as message + causes, with no `Location:` footer — see
    // `ros_launch_resolve::util::cli_errors`. Kept identical to
    // `play_launch`'s: the two CLIs share one verb implementation, so they
    // must not differ in how that implementation's failures read.
    ros_launch_resolve::util::cli_errors::install();

    // Announce this binary's identity for `meta.resolver` (see
    // `ros_launch_resolve::producer`). The BINARY name, not the crate's
    // (`ros-launch-resolve-cli`), which is an artifact of the workspace
    // layout and not something anyone types.
    ros_launch_resolve::producer::set("ros-launch-resolve", env!("CARGO_PKG_VERSION"));

    // Initialize a tracing subscriber so `info!`/`warn!` calls in the shared
    // `ros-launch-resolve` crate (e.g. `manifest_loader::load_manifests`'s
    // "Loaded N manifest(s) [...]" summary) are actually emitted -- without
    // this, `tracing`'s macros are no-ops (no subscriber registered) and
    // those lines silently vanish. Mirrors `play_launch`'s own init
    // (`src/play_launch/src/main.rs`): `RUST_LOG` takes precedence; default
    // to INFO otherwise. Written to stderr, same as this CLI's `eprintln!`
    // diagnostics, so callers see one interleaved stream.
    //
    // The filter must be an explicit `EnvFilter` (and `tracing-subscriber`
    // must carry the non-default `env-filter` feature): the builder form of
    // `fmt()` applies NO env filter of its own, unlike the free-function
    // `fmt::init()` play_launch uses. The first cut of this init branched on
    // `RUST_LOG` and then built the same subscriber either way, so
    // `RUST_LOG=debug` and `RUST_LOG=error` produced byte-identical output.
    //
    // The emitting module is shown only when `RUST_LOG` is set — i.e. to a
    // developer who asked for it. On by default it printed
    // `ros_launch_resolve::verbs::dump` beside ordinary progress lines.
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .with_target(std::env::var("RUST_LOG").is_ok())
        .init();

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
