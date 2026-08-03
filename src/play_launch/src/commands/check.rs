//! `play_launch check` — argument mapping, plus the one decision the library
//! deliberately leaves to its caller: what to do with the exit code.
//!
//! `verbs::check::run` returns the status it INTENDS (0 clean or
//! no-manifests, 1 when an Error-severity diagnostic survives `--rule`
//! filtering) instead of calling `std::process::exit` itself, so a library
//! consumer can check contracts without the process dying underneath it.
//! This binary's contract is the process exit status, so here — and only
//! here — that code is applied.
//!
//! Dropping the `i32` (`verbs::check::run(..)?; Ok(())`) compiles, prints
//! every diagnostic exactly as before, and exits 0 — a checker that cannot
//! fail. Issues 0008, 0012 and 0014 are three separate instances of that bug
//! in this repo. `check_exits_nonzero_on_a_contract_error` in
//! `tests/tests/migrated_verbs.rs` is what stops a fourth.

use eyre::Result;

use crate::cli::options::CheckArgs;
use ros_launch_resolve::verbs::{self, CheckInputs};

pub fn handle_check(args: &CheckArgs) -> Result<()> {
    let code = verbs::check::run(CheckInputs {
        package_or_path: args.package_or_path.clone(),
        launch_file: args.launch_file.clone(),
        launch_arguments: args.launch_arguments.clone(),
        contracts: args.contracts.clone(),
        no_provider_contracts: args.no_provider_contracts,
        sched: args.sched.clone(),
        target: args.target.clone(),
        format: args.format.clone(),
        rule: args.rule.clone(),
        explain: args.explain,
        export_graph: args.export_graph.clone(),
    })?;

    if code != 0 {
        std::process::exit(code);
    }
    Ok(())
}
