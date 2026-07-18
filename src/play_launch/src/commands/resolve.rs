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
    let cli_args = super::parse_launch_arguments(&launch_arguments);
    let arg_binding: BTreeMap<String, String> = cli_args
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();

    // Phase 43.1 — two record modes. Either way `record_path` names the
    // spawn-info companion on disk, whose sha256 lands in meta.inputs so
    // `replay --model` can refuse a mismatched (model, record) pair.
    let (dump, launch_path, record_path): (
        crate::ros::launch_dump::LaunchDump,
        Option<std::path::PathBuf>,
        Option<std::path::PathBuf>,
    ) = match &args.record {
        Some(rec) => {
            eprintln!("Resolving from record: {}", rec.display());
            let dump = crate::ros::launch_dump::load_launch_dump(rec)
                .wrap_err_with(|| format!("loading record {}", rec.display()))?;
            (dump, None, Some(rec.clone()))
        }
        None => {
            let pkg_or_path = args.package_or_path.as_deref().ok_or_else(|| {
                eyre::eyre!("either a launch file (package/path) or --record is required")
            })?;
            let launch_path = super::launch::resolve_launch_file(pkg_or_path, launch_file)?;
            eprintln!("Resolving launch file: {}", launch_path.display());
            let record = play_launch_parser::parse_launch_file(&launch_path, cli_args.clone())
                .map_err(|e| eyre::eyre!("Parser error: {e}"))?;
            let json = serde_json::to_string_pretty(&record)?;
            let dump: crate::ros::launch_dump::LaunchDump = serde_json::from_str(&json)?;
            // Emit the spawn-info companion next to the model so the pair
            // ships together (skipped for stdout mode — nothing to bind).
            let record_path = if args.out == "-" {
                None
            } else {
                let p = std::path::PathBuf::from(&args.out).with_extension("record.json");
                std::fs::write(&p, &json)
                    .wrap_err_with(|| format!("writing record companion {}", p.display()))?;
                eprintln!("Record companion: {}", p.display());
                Some(p)
            };
            (dump, Some(launch_path), record_path)
        }
    };

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
    if let Some(lp) = &launch_path {
        input_paths.insert(canon(lp.clone()));
    }
    if let Some(rp) = &record_path {
        input_paths.insert(canon(rp.clone()));
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
        &dump,
        &index,
        sched_inputs.as_ref(),
        arg_binding,
        &input_paths,
    );
    // Phase 43.1 — bind the spawn-info companion explicitly (the gate
    // `replay --model` checks; a bare inputs match is too loose, since the
    // launch file itself is also hashed).
    if let Some(rp) = &record_path {
        use sha2::Digest as _;
        let bytes = std::fs::read(rp)
            .wrap_err_with(|| format!("hashing record companion {}", rp.display()))?;
        let canon_rp = std::fs::canonicalize(rp).unwrap_or_else(|_| rp.clone());
        model.meta.record = Some(ros_launch_manifest_model::InputHash {
            path: canon_rp.display().to_string(),
            sha256: format!("{:x}", sha2::Sha256::digest(&bytes)),
        });
    }

    // R1-P1 — integrator system config fills the execution layer
    // (deploy/transports/bridges/features). Fail-loud on unplaced nodes;
    // lenient diagnostics embed like checker warnings.
    if let Some(sys_path) = &args.system {
        let text = std::fs::read_to_string(sys_path)
            .wrap_err_with(|| format!("reading system config {}", sys_path.display()))?;
        let cfg = ros_launch_manifest_model::system_config::parse_system_config(&text)
            .map_err(|e| eyre::eyre!("parsing {}: {e}", sys_path.display()))?;
        let node_fqns: Vec<&str> = model.structure.nodes.keys().map(|s| s.as_str()).collect();
        let diags = cfg
            .apply_to(&mut model.execution, &node_fqns)
            .map_err(|e| eyre::eyre!(e))?;
        model.meta.diagnostics.extend(diags);
        // Hash the system config into provenance.
        {
            use sha2::Digest as _;
            let bytes = std::fs::read(sys_path)?;
            let canon = std::fs::canonicalize(sys_path).unwrap_or_else(|_| sys_path.clone());
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
