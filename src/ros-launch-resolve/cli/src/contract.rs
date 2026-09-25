//! `ros-launch-resolve contract` — argument mapping only.
//!
//! The verbs themselves are `ros_launch_resolve::verbs::contract::eject` and
//! `ros_launch_resolve::verbs::capture::capture`.

use eyre::Result;

use crate::options::{ContractCaptureArgs, ContractEjectArgs};
use ros_launch_resolve::verbs::{self, CaptureInputs, ContractEjectInputs};

pub fn handle_contract_eject(args: &ContractEjectArgs) -> Result<()> {
    verbs::contract::eject(ContractEjectInputs {
        package_or_path: args.package_or_path.clone(),
        launch_file: args.launch_file.clone(),
        target: args.target.clone(),
        into: args.into.clone(),
        force: args.force,
    })
}

/// Prints to stdout and writes nothing — see `verbs::capture`.
pub fn handle_contract_capture(args: &ContractCaptureArgs) -> Result<()> {
    let text = verbs::capture::capture(CaptureInputs {
        run_dir: args.run_dir.clone(),
        model: args.model.clone(),
        include_infra: args.include_infra,
    })?;
    print!("{text}");
    Ok(())
}
