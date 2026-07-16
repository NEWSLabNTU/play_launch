//! `play_launch resolve` — emit a SystemModel (RFC-0050 /
//! docs/design/system-model.md): parse the launch tree, bind args, filter
//! conditions, merge scopes, run the contract checker, derive the
//! scheduling plan, and serialize the fully-resolved artifact.
//!
//! Refuses to emit when the checker reports Error severity — a SystemModel
//! in hand is always a checked one. Warnings embed in `meta.diagnostics`.

use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};

use eyre::{Context, Result};
use ros_launch_manifest_check::Severity;

use crate::{
    cli::options::ResolveArgs,
    ros::{manifest_loader, model_builder, sched_loader},
};

pub fn handle_resolve(args: &ResolveArgs) -> Result<()> {
    let launch_path =
        super::launch::resolve_launch_file(&args.package_or_path, args.launch_file.as_deref())?;
    eprintln!("Resolving launch file: {}", launch_path.display());

    let cli_args = super::parse_launch_arguments(&args.launch_arguments);
    let arg_binding: BTreeMap<String, String> = cli_args
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();

    let record = play_launch_parser::parse_launch_file(&launch_path, cli_args)
        .map_err(|e| eyre::eyre!("Parser error: {e}"))?;
    let json = serde_json::to_string(&record)?;
    let dump: crate::ros::launch_dump::LaunchDump = serde_json::from_str(&json)?;

    let sources = args.contract_sources();
    let index = manifest_loader::load_manifests(&dump, &sources)?;

    // Gate: Error severity anywhere refuses emission (validity by
    // construction — RFC-0050).
    let merge_errors = index
        .merge_diagnostics
        .iter()
        .filter(|d| matches!(d.severity, Severity::Error))
        .count();
    let total_errors = index.total_errors + merge_errors;
    if total_errors > 0 {
        for m in index.manifests.values() {
            for d in &m.diagnostics {
                if matches!(d.severity, Severity::Error) {
                    eprintln!("error [{}]: {d}", m.file);
                }
            }
        }
        for d in &index.merge_diagnostics {
            if matches!(d.severity, Severity::Error) {
                eprintln!("error: {d}");
            }
        }
        eyre::bail!(
            "refusing to emit a SystemModel: {total_errors} contract error(s) \
             (see `play_launch check` for source excerpts)"
        );
    }

    // Provenance inputs: the root launch file, every file-scope launch
    // file, every resolved contract file, and (below) the platform file.
    let canon = |p: PathBuf| std::fs::canonicalize(&p).unwrap_or(p);
    let mut input_paths: BTreeSet<PathBuf> = BTreeSet::new();
    input_paths.insert(canon(launch_path.clone()));
    for s in &dump.scopes {
        if let Some(p) = s.path() {
            input_paths.insert(canon(PathBuf::from(p)));
        }
    }
    for m in index.manifests.values() {
        input_paths.insert(canon(m.contract_path.clone()));
    }

    // Scheduling: same channel resolution as `check` (--sched > overlay >
    // provider sidecar). Optional — a model without an execution layer is
    // still a valid structure+contracts artifact.
    let resolved_platform = sched_loader::resolve_platform_file(
        &dump,
        args.sched.as_deref(),
        sources.overlay.as_deref(),
        sources.provider,
        &args.target,
    );
    let derived = match &resolved_platform {
        Some(resolved) => {
            eprintln!(
                "Scheduling platform file [{}]: {}",
                resolved.channel,
                resolved.path.display()
            );
            input_paths.insert(
                std::fs::canonicalize(&resolved.path).unwrap_or_else(|_| resolved.path.clone()),
            );
            let derived = sched_loader::derive_sched_plan(
                &dump,
                Some(&index),
                &resolved.path,
                &args.target,
                crate::execution::sched_apply::SchedApplyMode::Warn,
            )?;
            let declared_tiers = ros_launch_manifest_sched::parse_platform_file(&resolved.path)
                .ok()
                .and_then(|f| f.legacy)
                .map(|l| l.tiers);
            Some((derived, declared_tiers))
        }
        None => None,
    };
    let sched_inputs = derived
        .as_ref()
        .map(|(d, tiers)| model_builder::SchedInputs {
            derived: d,
            declared_tiers: tiers.clone(),
        });

    let model = model_builder::build_system_model(
        &dump,
        &index,
        sched_inputs.as_ref(),
        arg_binding,
        &input_paths,
    );

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
