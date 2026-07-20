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

use crate::{
    cli::options::ResolveArgs,
    ros::{launch_dump::LaunchDump, manifest_loader, model_builder, sched_loader},
};

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

    let runtime = super::common::build_tokio_runtime()?;
    let (dump, launch_path) = runtime.block_on(super::common::parse_to_launch_dump(
        &args.package_or_path,
        launch_file,
        &launch_arguments,
        args.parser,
    ))?;

    let arg_binding: BTreeMap<String, String> = super::parse_launch_arguments(&launch_arguments)
        .into_iter()
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

/// Shared inputs for building a checked SystemModel from an in-memory
/// [`LaunchDump`] — used by both `resolve` (file-based, above) and
/// `launch`'s in-memory internal round-trip (`commands::launch`, Phase
/// 47.B4: no `record.json` is written on either path).
pub struct ModelBuildInputs<'a> {
    pub dump: &'a LaunchDump,
    pub launch_path: Option<&'a Path>,
    pub arg_binding: BTreeMap<String, String>,
    pub contracts: Option<&'a Path>,
    pub no_provider_contracts: bool,
    pub sched: Option<&'a Path>,
    pub system: Option<&'a Path>,
    pub target: &'a str,
    pub explain: bool,
}

/// Build a checked [`ros_launch_manifest_model::SystemModel`] from an
/// in-memory launch dump: resolve contracts, gate on checker errors, derive
/// the scheduling plan, and (optionally) apply an integrator system config.
/// No disk I/O beyond reading the inputs the caller already resolved
/// (`sched`/`system` paths) — the model itself is returned in memory; it's
/// the caller's job to write it out (or not, on the `launch` in-memory
/// path).
pub fn build_checked_model(
    inputs: ModelBuildInputs<'_>,
) -> Result<ros_launch_manifest_model::SystemModel> {
    let ModelBuildInputs {
        dump,
        launch_path,
        arg_binding,
        contracts,
        no_provider_contracts,
        sched,
        system,
        target,
        explain,
    } = inputs;

    let sources = crate::ros::manifest_loader::ContractSources {
        overlay: manifest_loader::discover_overlay_root(contracts),
        provider: !no_provider_contracts,
    };
    let index = manifest_loader::load_manifests(dump, &sources)?;

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
    // file, and every resolved contract file (the platform file, below).
    let canon = |p: PathBuf| std::fs::canonicalize(&p).unwrap_or(p);
    let mut input_paths: BTreeSet<PathBuf> = BTreeSet::new();
    if let Some(lp) = launch_path {
        input_paths.insert(canon(lp.to_path_buf()));
    }
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
        dump,
        sched,
        sources.overlay.as_deref(),
        sources.provider,
        target,
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
                dump,
                Some(&index),
                &resolved.path,
                target,
                crate::execution::sched_apply::SchedApplyMode::Warn,
            )?;
            // Single authoritative surfacing point for `resolve` (45.1a) —
            // `derive_sched_plan` only collects now (no internal
            // `tracing::warn!`), so the caller must log its own returned
            // warnings exactly once. `check`/`run`/`replay` do the same at
            // their own single call sites (`check_sched`, `SchedPlan::build`).
            for w in &derived.warnings {
                tracing::warn!("{w}");
            }
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

    let mut model = model_builder::build_system_model(
        dump,
        &index,
        sched_inputs.as_ref(),
        arg_binding,
        &input_paths,
    );

    // R1-P1 — integrator system config fills the execution layer
    // (deploy/transports/bridges/features). Fail-loud on unplaced nodes;
    // lenient diagnostics embed like checker warnings.
    if let Some(sys_path) = system {
        let text = std::fs::read_to_string(sys_path)
            .wrap_err_with(|| format!("reading system config {}", sys_path.display()))?;
        let cfg = ros_launch_manifest_model::system_config::parse_system_config(&text)
            .map_err(|e| eyre::eyre!("parsing {}: {e}", sys_path.display()))?;
        let node_fqns: Vec<&str> = model.structure.nodes.keys().map(|s| s.as_str()).collect();
        let diags = cfg
            .apply_to(&mut model.execution, &node_fqns)
            .map_err(|e| eyre::eyre!(e))?;
        // `[lifecycle] autostart` lives in the structure layer (per-node), so
        // it is applied here rather than in the execution-only `apply_to`.
        if let Some(autostart) = cfg.lifecycle_autostart() {
            for inst in model.structure.nodes.values_mut() {
                inst.lifecycle_autostart = Some(autostart);
            }
        }
        model.meta.diagnostics.extend(diags);
        // Hash the system config into provenance.
        {
            use sha2::Digest as _;
            let bytes = std::fs::read(sys_path)?;
            let canon = std::fs::canonicalize(sys_path).unwrap_or_else(|_| sys_path.to_path_buf());
            model
                .meta
                .inputs
                .push(ros_launch_manifest_model::InputHash {
                    path: canon.display().to_string(),
                    sha256: format!("{:x}", sha2::Sha256::digest(&bytes)),
                });
            model.meta.inputs.sort_by(|a, b| a.path.cmp(&b.path));
        }
        eprintln!(
            "System config: {} ({} deploy, {} transport(s), {} bridge(s))",
            sys_path.display(),
            model.execution.deploy.len(),
            model.execution.transports.len(),
            model.execution.bridges.len(),
        );
    }

    // Phase 45.10 — `--explain` renders from the FRESH derive this invocation
    // already produced (`derived` + `resolved_platform` + `index`), not from
    // the model (the resolved sched plan is no longer embedded — the model is
    // INPUT only). Same renderer `check --sched --explain` uses, so output is
    // byte-identical for the same inputs.
    if explain {
        match (&derived, &resolved_platform) {
            (Some((d, _)), Some(platform)) => {
                sched_loader::print_explain(d, platform, Some(&index));
            }
            _ => {
                eprintln!(
                    "note: --explain has no effect without a resolved scheduling platform file \
                     (pass --sched <path>, or ship one via the overlay/provider channels)"
                );
            }
        }
    }

    Ok(model)
}
