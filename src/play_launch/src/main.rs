//! Thin entry point. All modules live in the library crate (see `lib.rs`) so
//! that nano-ros can link the resolve pipeline instead of shelling out to a
//! `play_launch` binary resolved through PATH (nano-ros #285).

use clap::Parser;
use play_launch::{
    cli::options::{Command, Options},
    util::logging::init_verbose,
};
use tracing::{debug, warn};

/// Does this verb use stdout as a DATA channel rather than a log channel?
///
/// `resolve -o -` writes the SystemModel YAML to stdout and `check --format
/// json` writes a JSON report there; both exist to be piped into another
/// program. play_launch's subscriber has always written to stdout, so the
/// moment those two verbs arrived (they had lived in `ros-launch-resolve`,
/// whose subscriber writes to stderr) every `tracing` line from the shared
/// resolve library landed in the middle of the artifact — `... resolve -o - |
/// yaml.safe_load` died on an ANSI escape, `... check --format json | jq`
/// on a WARN line. Logs go to stderr for these five.
///
/// The other verbs keep stdout: they emit no machine-readable artifact, and
/// their log stream is what `tests/tests/autoware.rs` and
/// `tests/tests/sched_apply.rs` capture. Moving those too is a defensible
/// cleanup but a bigger, separate change.
fn logs_to_stderr(command: &Command) -> bool {
    matches!(
        command,
        Command::Resolve(_)
            | Command::Dump(_)
            | Command::Check(_)
            | Command::Plot(_)
            | Command::Contract(_)
    )
}

/// Initialize the tracing subscriber. `RUST_LOG` takes precedence for
/// development/debugging; INFO otherwise (the `--verbose` flag controls
/// detail, not level).
fn init_tracing(command: &Command) {
    let use_env_filter = std::env::var("RUST_LOG").is_ok();
    // `fmt()`'s builder needs the writer chosen before `.init()`, and the two
    // writer types differ, so the branch is duplicated rather than factored.
    if logs_to_stderr(command) {
        let builder = tracing_subscriber::fmt().with_writer(std::io::stderr);
        if use_env_filter {
            builder
                .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
                .init();
        } else {
            builder.with_max_level(tracing::Level::INFO).init();
        }
    } else if use_env_filter {
        tracing_subscriber::fmt::init();
    } else {
        tracing_subscriber::fmt()
            .with_max_level(tracing::Level::INFO)
            .init();
    }
}

fn main() -> eyre::Result<()> {
    // Parse command-line options first (before initializing tracing)
    let opts = Options::parse();

    // Store verbose flag globally for conditional logging
    init_verbose(&opts);

    init_tracing(&opts.command);

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
        play_launch::cli::options::Command::Up(args) => {
            play_launch::commands::handle_up(args)?;
        }
        play_launch::cli::options::Command::Replay(args) => {
            play_launch::commands::migrated::replay_renamed(args)?;
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
        play_launch::cli::options::Command::Resolve(args) => {
            play_launch::commands::handle_resolve(args)?;
        }
        play_launch::cli::options::Command::Dump(args) => {
            play_launch::commands::handle_dump(args)?;
        }
        play_launch::cli::options::Command::Check(args) => {
            play_launch::commands::handle_check(args)?;
        }
        play_launch::cli::options::Command::Plot(args) => {
            play_launch::commands::handle_plot(args)?;
        }
        // `contract` is a subcommand group; today `eject` is its only verb.
        play_launch::cli::options::Command::Contract(args) => match &args.subcommand {
            play_launch::cli::options::ContractSubcommand::Eject(eject) => {
                play_launch::commands::handle_contract_eject(eject)?;
            }
        },
    }

    Ok(())
}
