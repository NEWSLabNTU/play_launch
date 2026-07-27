//! Building a checked [`SystemModel`] from a launch tree.
//!
//! The pipeline entry point, in the LIBRARY rather than the CLI: a consumer
//! that links this crate (nano-ros's helper, for one) needs to build a model
//! without re-implementing argument parsing. Reading files, printing progress
//! and choosing an output path stay in the binary.

use eyre::{Context, Result};
use ros_launch_manifest_check::Severity;
use std::{
    collections::{BTreeMap, BTreeSet},
    path::{Path, PathBuf},
};

use crate::ros::{launch_dump::LaunchDump, manifest_loader, model_builder, sched_loader};


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
                crate::config::SchedApplyMode::Warn,
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

    // Issue 0293 — `meta.inputs` paths are recorded relative to the bringup
    // package (the launch file's grandparent: `<bringup>/launch/x.launch.xml`)
    // so a COMMITTED model is portable. The consumer resolves them against its
    // own bringup dir; an absolute path from the resolving machine silently
    // failed `.exists()` everywhere else.
    let input_base = launch_path.and_then(|p| p.parent()).and_then(|p| p.parent());
    let mut model = model_builder::build_system_model(
        dump,
        &index,
        sched_inputs.as_ref(),
        arg_binding,
        &input_paths,
        input_base,
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
        // nano-ros issue 0291 — tell placement WHICH launch file this is, so
        // `[deploy.*]` blocks scoped with `launch = "…"` are filtered out when
        // they name a different one. Without it every scoped block counted
        // against every launch file and the multi-block rule fired on configs
        // that are correct.
        let diags = cfg
            .apply_to_launch(
                &mut model.execution,
                &node_fqns,
                launch_path.and_then(|p| p.file_name()).and_then(|n| n.to_str()),
            )
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
            model
                .meta
                .inputs
                .push(ros_launch_manifest_model::InputHash {
                    path: ros_launch_manifest_model::input_path_string(sys_path, input_base),
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
