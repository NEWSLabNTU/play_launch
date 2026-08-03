//! `play_launch dump` — argument mapping only.
//!
//! The verb itself is `ros_launch_resolve::verbs::dump`, which delegates to
//! `verbs::resolve` (Phase 46.5: `dump` and `resolve` emit the same
//! SystemModel through one code path).

use eyre::Result;

use crate::cli::options::{DumpArgs, DumpSubcommand};
use ros_launch_resolve::verbs::{self, DumpInputs};

pub fn handle_dump(args: &DumpArgs) -> Result<()> {
    let DumpSubcommand::Launch(launch_args) = &args.subcommand;
    verbs::dump::run(DumpInputs {
        package_or_path: launch_args.package_or_path.clone(),
        launch_file: launch_args.launch_file.clone(),
        launch_arguments: launch_args.launch_arguments.clone(),
        contracts: launch_args.common.contract_opts.contracts.clone(),
        no_provider_contracts: launch_args.common.contract_opts.no_provider_contracts,
        sched: launch_args.common.sched_opts.sched.clone(),
        target: launch_args.common.sched_opts.target.clone(),
        parser: launch_args.parser.into(),
        block_commands: launch_args.block_commands,
        // `None` is meaningful: the library applies the
        // `system_model.yaml` default itself (clap declares `--output`
        // without one), so do NOT substitute a default here.
        output: args.output.clone(),
    })
}
