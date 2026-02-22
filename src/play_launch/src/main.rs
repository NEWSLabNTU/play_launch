mod cli;
mod commands;
mod diagnostics;
mod execution;
mod member_actor;
mod monitoring;
mod process;
mod python;
mod ros;
mod util;
mod web;

use crate::{cli::options::Options, util::logging::init_verbose};
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
        cli::options::Command::Launch(args) => {
            commands::handle_launch(args)?;
        }
        cli::options::Command::Run(args) => {
            commands::handle_run(args)?;
        }
        cli::options::Command::Dump(args) => {
            commands::handle_dump(args)?;
        }
        cli::options::Command::Replay(args) => {
            commands::handle_replay(args)?;
        }
        cli::options::Command::Plot(args) => {
            commands::handle_plot(args)?;
        }
        cli::options::Command::Compile(args) => {
            #[cfg(feature = "wasm-compile")]
            {
                commands::handle_compile(args)?;
            }
            #[cfg(not(feature = "wasm-compile"))]
            {
                let _ = args;
                return Err(eyre::eyre!(
                    "WASM compilation support not compiled. Rebuild with: cargo build --features wasm-compile"
                ));
            }
        }
        cli::options::Command::Exec(args) => {
            #[cfg(feature = "wasm-exec")]
            {
                commands::handle_exec(args)?;
            }
            #[cfg(not(feature = "wasm-exec"))]
            {
                let _ = args;
                return Err(eyre::eyre!(
                    "WASM execution support not compiled. Rebuild with: cargo build --features wasm-exec"
                ));
            }
        }
        cli::options::Command::SetcapIoHelper => {
            commands::handle_setcap_io_helper()?;
        }
        cli::options::Command::VerifyIoHelper => {
            commands::handle_verify_io_helper()?;
        }
    }

    Ok(())
}
