//! Handlers for verbs renamed in 0.9.0.
//!
//! The `replay` variant is hidden from `--help` but still PARSES, accepting
//! its old arguments, so the error can echo the user's own invocation back in
//! the new form.
//!
//! That is the whole point. nano-ros issue 0285 was a subcommand vanishing
//! and clap answering `unrecognized subcommand 'resolve'` from inside a
//! cmake configure, which took down every platform's fixture build. The
//! failure was the error, not the removal.
//!
//! `check` and `resolve` used to be redirected from here too. They are real,
//! advertised verbs again (`commands::check`, `commands::resolve`), so their
//! hidden variants and `check_removed`/`resolve_removed` are GONE — a hidden
//! variant of the same name would shadow the live verb. `replay` → `up` is
//! the only rename that still stands.
//!
//! DELETE THIS MODULE AT 1.0.0.

use crate::cli::options::UpArgs;
use eyre::Result;

/// Print `msg` to stderr and exit non-zero.
///
/// Deliberately NOT `eyre::bail!`: an eyre error carries a `Location:
/// src/play_launch/src/commands/migrated.rs:NN` footer, and in the exact
/// scenario this handler exists for — the error surfacing from inside a
/// cmake configure (nano-ros issue 0285) — a source location reads as an
/// internal crash rather than as migration guidance. Same message, same
/// non-zero exit, no footer.
fn migration_error(msg: String) -> ! {
    eprintln!("Error: {msg}");
    std::process::exit(1)
}

/// Reconstruct the model argument the user passed, for echoing back.
fn model_arg(args: &UpArgs) -> String {
    match (&args.model_path, &args.model) {
        (Some(p), _) => p.display().to_string(),
        (None, Some(p)) => format!("--model {}", p.display()),
        (None, None) => "<model.yaml>".to_string(),
    }
}

pub fn replay_renamed(args: &UpArgs) -> Result<()> {
    migration_error(format!(
        "`replay` was renamed to `up` in 0.9.0.\n       play_launch up {}",
        model_arg(args)
    ))
}
