//! `ros-launch-resolve check` — argument mapping, plus the one decision the
//! library deliberately leaves to its caller: what to do with the exit code.
//!
//! `verbs::check::run` returns the status it INTENDS (0 clean, 1 when an
//! Error-severity diagnostic survives `--rule` filtering) instead of calling
//! `std::process::exit` itself, so a library consumer can check contracts
//! without the process dying underneath it. This binary's contract is the
//! process exit status, so here — and only here — that code is applied.

use eyre::Result;

use crate::options::CheckArgs;
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
