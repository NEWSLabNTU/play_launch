//! `ros-launch-resolve contract eject` — argument mapping only.
//!
//! The verb itself is `ros_launch_resolve::verbs::contract::eject`.

use eyre::Result;

use crate::options::ContractEjectArgs;
use ros_launch_resolve::verbs::{self, ContractEjectInputs};

pub fn handle_contract_eject(args: &ContractEjectArgs) -> Result<()> {
    verbs::contract::eject(ContractEjectInputs {
        package_or_path: args.package_or_path.clone(),
        launch_file: args.launch_file.clone(),
        target: args.target.clone(),
        into: args.into.clone(),
        force: args.force,
    })
}
