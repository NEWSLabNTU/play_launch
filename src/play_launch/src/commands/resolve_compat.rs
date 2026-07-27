//! `play_launch resolve` — a thin delegate to layer 2, kept for compatibility.
//!
//! Resolution moved to `ros-launch-resolve` (RFC-0060). The verb stays because
//! it has live downstream users — simple-autoware-safety-island embeds the
//! command in `sentinel_bringup/launch/pilot.launch.xml` as its documented
//! recipe for regenerating a model — and because silently removing a
//! subcommand people invoke is exactly the failure that took every nano-ros
//! platform's fixture build down (nano-ros issue 0285: `unrecognized
//! subcommand 'resolve'`, surfacing from inside a cmake configure).
//!
//! It prints one deprecation line naming the replacement and is scheduled for
//! removal once downstream recipes have moved.

use eyre::Result;
use ros_launch_resolve::model::{ModelBuildInputs, build_checked_model};

use crate::cli::options::ResolveArgs;

pub fn handle_resolve(args: &ResolveArgs) -> Result<()> {
    eprintln!(
        "play_launch: `resolve` is deprecated and will be removed — use the \
         `ros-launch-resolve` binary (https://github.com/NEWSLabNTU/ros-launch-resolve)."
    );

    let launch_path = std::path::PathBuf::from(&args.package_or_path);
    let record = play_launch_parser::parse_launch_file(&launch_path, Default::default())
        .map_err(|e| eyre::eyre!("parsing {}: {e}", launch_path.display()))?;
    let dump = serde_json::from_str(&serde_json::to_string(&record)?)?;

    let arg_binding = args
        .launch_arguments
        .iter()
        .filter_map(|a| a.split_once(":=").map(|(k, v)| (k.to_string(), v.to_string())))
        .collect();

    let model = build_checked_model(ModelBuildInputs {
        dump: &dump,
        launch_path: Some(&launch_path),
        arg_binding,
        contracts: args.contracts.as_deref(),
        no_provider_contracts: args.no_provider_contracts,
        sched: args.sched.as_deref(),
        system: args.system.as_deref(),
        target: args.target.as_str(),
        explain: args.explain,
    })?;

    let yaml = model.to_yaml_string()?;
    if args.out == "-" {
        print!("{yaml}");
    } else {
        std::fs::write(&args.out, &yaml)
            .map_err(|e| eyre::eyre!("writing SystemModel to {}: {e}", args.out))?;
        eprintln!("SystemModel: {} ({} nodes)", args.out, model.structure.nodes.len());
    }
    Ok(())
}
