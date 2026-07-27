//! Thin entry point. All modules live in the library crate (see `lib.rs`) so
//! that nano-ros can link the resolve pipeline instead of shelling out to a
//! `play_launch` binary resolved through PATH (nano-ros #285).

use play_launch::{cli::options::Options, util::logging::init_verbose};
use clap::Parser;
use tracing::{debug, warn};

fn main() -> eyre::Result<()> {
    // Parse command-line options first (before initializing tracing)
    let opts = Options::parse();

    // Store verbose flag globally for conditional logging
    init_verbose(&opts);

    // Initialize tracing subscriber with INFO as default level
    // Priority: RUST_LOG > default (INFO)
    if std::env::var("RUST_LOG").is_ok() {
        // RUST_LOG env var takes precedence (for development/debugging)
        tracing_subscriber::fmt::init();
    } else {
        // Default to INFO level - verbose flag controls detail, not level
        tracing_subscriber::fmt()
            .with_max_level(tracing::Level::INFO)
            .init();
    }

    // Debug: Check AMENT_PREFIX_PATH at startup
    if let Ok(ament_path) = std::env::var("AMENT_PREFIX_PATH") {
        debug!(
            "AMENT_PREFIX_PATH first 200 chars: {}",
            ament_path.chars().take(200).collect::<String>()
        );
    } else {
        warn!("AMENT_PREFIX_PATH NOT SET!");
    }

    // Route to appropriate handler based on subcommand
    match &opts.command {
        play_launch::cli::options::Command::Launch(args) => {
            play_launch::commands::handle_launch(args)?;
        }
        play_launch::cli::options::Command::Run(args) => {
            play_launch::commands::handle_run(args)?;
        }
        play_launch::cli::options::Command::Dump(args) => {
            play_launch::commands::handle_dump(args)?;
        }
        play_launch::cli::options::Command::Replay(args) => {
            play_launch::commands::handle_replay(args)?;
        }
        play_launch::cli::options::Command::Plot(args) => {
            play_launch::commands::handle_plot(args)?;
        }
        play_launch::cli::options::Command::Setcap => {
            play_launch::commands::handle_setcap()?;
        }
        play_launch::cli::options::Command::Verify => {
            play_launch::commands::handle_verify()?;
        }
        play_launch::cli::options::Command::Context(args) => {
            play_launch::commands::handle_context(args)?;
        }
        play_launch::cli::options::Command::Check(args) => {
            play_launch::commands::handle_check_manifest(args)?;
        }
        play_launch::cli::options::Command::Resolve(args) => {
            play_launch::commands::handle_resolve(args)?;
        }
        play_launch::cli::options::Command::Contract(args) => match &args.subcommand {
            play_launch::cli::options::ContractSubcommand::Eject(eject_args) => {
                play_launch::commands::handle_contract_eject(eject_args)?;
            }
        },
    }

    Ok(())
}
