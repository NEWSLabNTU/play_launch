//! `ros-launch-resolve migrate` -- argument mapping onto `verbs::migrate::run`, plus the
//! exit status, which only the binary applies (see `check`).

use eyre::Result;

use crate::options::MigrateArgs;
use ros_launch_resolve::verbs::{self, MigrateInputs};

pub fn handle_migrate(args: &MigrateArgs) -> Result<()> {
    let code = verbs::migrate::run(MigrateInputs {
        paths: args.paths.clone(),
        write: args.write,
        check: args.check,
        list: args.list,
        quiet: args.quiet,
    })?;
    if code != 0 {
        std::process::exit(code);
    }
    Ok(())
}
