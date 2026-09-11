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

/// Does this verb create a `play_log/<ts>/` directory and supervise
/// processes? Those are the runs whose account must survive the terminal
/// (issue #0023): they get a second, file-backed layer that buffers until the
/// directory exists (`util::run_log`). The launch-tree verbs write no
/// directory and get no buffer to hold.
fn writes_run_log(command: &Command) -> bool {
    matches!(
        command,
        Command::Launch(_) | Command::Run(_) | Command::Up(_) | Command::Replay(_)
    )
}

/// Initialize the tracing subscriber. `RUST_LOG` takes precedence for
/// development/debugging; INFO otherwise (the `--verbose` flag controls
/// detail, not level).
fn init_tracing(command: &Command) {
    use tracing_subscriber::{
        EnvFilter,
        layer::{Layer, SubscriberExt},
        util::SubscriberInitExt,
    };

    let use_env_filter = std::env::var("RUST_LOG").is_ok();
    // Show the emitting module ONLY when the user asked for `RUST_LOG` — i.e.
    // is debugging. Left on by default, every ordinary `play_launch dump` run
    // printed `ros_launch_resolve::verbs::dump` beside its progress lines,
    // naming the developer-only binary on the happy path. A target is a
    // developer's routing key, not part of a user-facing message.
    let with_target = use_env_filter;
    let terminal_filter = if use_env_filter {
        EnvFilter::from_default_env()
    } else {
        EnvFilter::new("info")
    };

    // The file layer is independent of the terminal's filter on purpose: the
    // bundle is where the detail is wanted AFTER a run went wrong, which is
    // exactly when nobody had set `RUST_LOG` beforehand. It carries the
    // target always — a file is read by whoever is debugging.
    //
    // Generic over the subscriber it stacks on: a `Filtered` layer is typed
    // by the subscriber beneath it, and the two terminal writers below build
    // different ones, so one value cannot serve both branches.
    fn file_layer<S>(enabled: bool) -> Option<impl Layer<S>>
    where
        S: tracing::Subscriber + for<'a> tracing_subscriber::registry::LookupSpan<'a>,
    {
        enabled.then(|| {
            tracing_subscriber::fmt::layer()
                .with_ansi(false)
                .with_target(true)
                .with_writer(play_launch::util::run_log::RunLogWriter)
                .with_filter(play_launch::util::run_log::file_filter())
        })
    }
    let to_file = writes_run_log(command);

    // The two terminal writers are different types, so the branch is
    // duplicated rather than factored.
    if logs_to_stderr(command) {
        tracing_subscriber::registry()
            .with(
                tracing_subscriber::fmt::layer()
                    .with_writer(std::io::stderr)
                    .with_target(with_target)
                    .with_filter(terminal_filter),
            )
            .with(file_layer(to_file))
            .init();
    } else {
        tracing_subscriber::registry()
            .with(
                tracing_subscriber::fmt::layer()
                    .with_target(with_target)
                    .with_filter(terminal_filter),
            )
            .with(file_layer(to_file))
            .init();
    }
}

fn main() -> eyre::Result<()> {
    // Report errors as message + causes, with no `Location:` footer pointing
    // into `src/ros-launch-resolve/` — see `util::cli_errors`.
    ros_launch_resolve::util::cli_errors::install();

    // Announce which binary is producing SystemModels, so `meta.resolver`
    // names `play_launch` at the version the user installed rather than the
    // shared library crate. See `ros_launch_resolve::producer`.
    ros_launch_resolve::producer::set(env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));

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
        play_launch::cli::options::Command::Measure(args) => {
            play_launch::commands::handle_measure(args)?;
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
