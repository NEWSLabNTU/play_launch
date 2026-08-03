//! `play_launch plot` — argument mapping only.
//!
//! The verb itself is `ros_launch_resolve::verbs::plot`.

use eyre::Result;

use crate::cli::options::PlotArgs;
use ros_launch_resolve::verbs::{self, PlotInputs};

pub fn handle_plot(args: &PlotArgs) -> Result<()> {
    verbs::plot::run(PlotInputs {
        log_dir: args.log_dir.clone(),
        base_log_dir: args.base_log_dir.clone(),
        output_dir: args.output_dir.clone(),
        metrics: args.metrics.clone(),
        list_metrics: args.list_metrics,
    })
}
