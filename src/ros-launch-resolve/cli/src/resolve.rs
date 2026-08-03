//! `ros-launch-resolve resolve` — argument mapping only.
//!
//! The verb itself is `ros_launch_resolve::verbs::resolve`, shared with
//! `play_launch`'s `resolve`. Nothing but the `*Args` → `*Inputs` translation
//! belongs here.

use eyre::Result;

use crate::options::ResolveArgs;
use ros_launch_resolve::verbs::{self, ResolveInputs};

pub fn handle_resolve(args: &ResolveArgs) -> Result<()> {
    verbs::resolve::run(ResolveInputs {
        package_or_path: args.package_or_path.clone(),
        launch_file: args.launch_file.clone(),
        launch_arguments: args.launch_arguments.clone(),
        bringup_root: args.bringup_root.clone(),
        contracts: args.contracts.clone(),
        no_provider_contracts: args.no_provider_contracts,
        sched: args.sched.clone(),
        system: args.system.clone(),
        target: args.target.clone(),
        parser: args.parser.into(),
        out: args.out.clone(),
        explain: args.explain,
    })
}
