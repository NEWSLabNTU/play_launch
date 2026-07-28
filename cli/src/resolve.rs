//! `play_launch resolve` — emit a SystemModel (RFC-0050 /
//! docs/design/system-model.md): parse the launch tree, bind args, filter
//! conditions, merge scopes, run the contract checker, derive the
//! scheduling plan, and serialize the fully-resolved artifact.
//!
//! Refuses to emit when the checker reports Error severity — a SystemModel
//! in hand is always a checked one. Warnings embed in `meta.diagnostics`.

use std::{
    collections::{BTreeMap, BTreeSet},
    path::{Path, PathBuf},
};

use eyre::{Context, Result};
use ros_launch_manifest_check::Severity;

use crate::options::ResolveArgs;
use ros_launch_resolve::model::{ModelBuildInputs, build_checked_model};
use ros_launch_resolve::ros::{launch_dump::LaunchDump, manifest_loader, model_builder, sched_loader};

pub fn handle_resolve(args: &ResolveArgs) -> Result<()> {
    // Positional quirk: with a direct launch-file PATH, the second
    // positional (`launch_file`) can swallow the first `KEY:=VALUE` arg.
    // Reclassify it so the binding isn't silently lost.
    let mut launch_arguments = args.launch_arguments.clone();
    let mut launch_file = args.launch_file.as_deref();
    if let Some(lf) = launch_file
        && lf.contains(":=")
    {
        launch_arguments.insert(0, lf.to_string());
        launch_file = None;
    }

    let runtime = crate::common::build_tokio_runtime()?;
    let (dump, launch_path) = runtime.block_on(crate::common::parse_to_launch_dump(
        &args.package_or_path,
        launch_file,
        &launch_arguments,
        args.parser,
    ))?;

    let arg_binding: BTreeMap<String, String> = crate::common::parse_launch_arguments(&launch_arguments)
        .into_iter()
        .collect();

    let model = build_checked_model(ModelBuildInputs {
        dump: &dump,
        launch_path: Some(&launch_path),
        bringup_root: args.bringup_root.as_deref(),
        arg_binding,
        contracts: args.contracts.as_deref(),
        no_provider_contracts: args.no_provider_contracts,
        sched: args.sched.as_deref(),
        system: args.system.as_deref(),
        target: args.target.as_str(),
        explain: args.explain,
    })?;

    let yaml = model
        .to_yaml_string()
        .wrap_err("serializing SystemModel to YAML")?;

    if args.out == "-" {
        print!("{yaml}");
    } else {
        std::fs::write(&args.out, &yaml)
            .wrap_err_with(|| format!("writing SystemModel to {}", args.out))?;
        eprintln!(
            "SystemModel: {} ({} nodes, {} topics, {} contracts-carrying endpoints, \
             {} tier(s), {} warning(s))",
            args.out,
            model.structure.nodes.len(),
            model.structure.topics.len(),
            model.contracts.pub_endpoints.len()
                + model.contracts.sub_endpoints.len()
                + model.contracts.srv_endpoints.len(),
            model.execution.tiers.len(),
            model.meta.diagnostics.len(),
        );
    }
    Ok(())
}
