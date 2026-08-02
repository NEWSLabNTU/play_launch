//! Handlers for verbs removed or renamed in 0.9.0.
//!
//! These variants are hidden from `--help` but still PARSE, accepting their
//! old arguments, so the error can echo the user's own invocation back in
//! the new form.
//!
//! That is the whole point. nano-ros issue 0285 was a subcommand vanishing
//! and clap answering `unrecognized subcommand 'resolve'` from inside a
//! cmake configure, which took down every platform's fixture build. The
//! failure was the error, not the removal.
//!
//! DELETE THIS MODULE AT 1.0.0.

use crate::cli::options::UpArgs;
use eyre::{Result, bail};

/// Reconstruct the model argument the user passed, for echoing back.
fn model_arg(args: &UpArgs) -> String {
    match (&args.model_path, &args.model) {
        (Some(p), _) => p.display().to_string(),
        (None, Some(p)) => format!("--model {}", p.display()),
        (None, None) => "<model.yaml>".to_string(),
    }
}

pub fn replay_renamed(args: &UpArgs) -> Result<()> {
    bail!(
        "`replay` was renamed to `up` in 0.9.0.\n       play_launch up {}",
        model_arg(args)
    )
}
