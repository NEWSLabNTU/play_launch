//! `ros-launch-resolve` — resolve a ROS 2 launch tree into a SystemModel.
//!
//! The launch-tree verbs that need no ROS graph: `resolve`, `dump`,
//! `contract`, `plot`. Running, replaying and supervising nodes stayed in
//! `play_launch`, which is the layer that legitimately needs rclrs and a
//! colcon-built message set (RFC-0060).

mod common;
mod config;
mod contract;
mod dump;
mod launch;
mod options;
mod plot;
mod resolve;

use clap::Parser;
use options::{Command, Options};

fn main() -> eyre::Result<()> {
    let opts = Options::parse();
    match &opts.command {
        Command::Resolve(args) => resolve::handle_resolve(args),
        Command::Dump(args) => dump::handle_dump(args),
        Command::Contract(args) => contract::handle_contract_eject(args),
        Command::Plot(args) => plot::handle_plot(args),
    }
}
