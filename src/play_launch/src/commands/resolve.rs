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
    cli::options::{ParserBackend, ResolveArgs},
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

    // Phase 46.5 — the record companion is retired: `resolve` no longer
    // writes a `<out>.record.json` next to the model (the model has been a
    // self-sufficient spawn source since 46.3b/46.4). `--record <path>`
    // (reuse mode) still reads an EXISTING record.json someone else
    // produced — that's a read path, not the companion write this wave
    // drops — and its path still lands in `meta.inputs` for provenance.
    let (dump, launch_path, record_reuse_path): (
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
            let dump: crate::ros::launch_dump::LaunchDump = match args.parser {
                ParserBackend::Rust => {
                    let record =
                        play_launch_parser::parse_launch_file(&launch_path, cli_args.clone())
                            .map_err(|e| eyre::eyre!("Parser error: {e}"))?;
                    let json = serde_json::to_string_pretty(&record)?;
                    serde_json::from_str(&json)?
                }
                ParserBackend::Python => {
                    // Phase 46.4 — one-step Python→model: run the Python
                    // parse path (same `dump_launch_python_wrapper` logic
                    // `dump` uses) into a SCRATCH temp file (never a
                    // record.json companion — 46.5 drops that artifact),
                    // then feed the SAME model_builder pipeline as the Rust
                    // path. The contract/sched layers apply on the shared
                    // scope table (`manifest_loader`/`sched_loader` key off
                    // `ScopeEntry.origin.path`, Phase 40.1 — both parsers
                    // emit it), independent of which parser produced the
                    // dump. They come back populated whenever a contract
                    // sidecar / platform file (or `--contracts`/`--sched`)
                    // resolves, and empty only when none does — NOT a Python
                    // limitation. (For rt_workspace, which ships both
                    // sidecars, the Python-produced model is byte-identical
                    // to the Rust one across structure/contracts/execution.)
                    eprintln!("Resolving via Python parser");
                    let scratch_path = std::env::temp_dir().join(format!(
                        "play_launch-resolve-{}.record.json",
                        std::process::id()
                    ));
                    let runtime = super::common::build_tokio_runtime()?;
                    runtime.block_on(async {
                        let launcher = crate::python::dump_launcher::DumpLauncher::new().wrap_err(
                            "Failed to initialize dump_launch. Ensure ROS workspace is sourced.",
                        )?;
                        launcher
                            .dump_launch(pkg_or_path, launch_file, &launch_arguments, &scratch_path)
                            .await
                    })?;
                    let dump = crate::ros::launch_dump::load_launch_dump(&scratch_path)
                        .wrap_err_with(|| {
                            format!("loading Python-parsed record {}", scratch_path.display())
                        })?;
                    let _ = std::fs::remove_file(&scratch_path);

                    // Phase 46.5 — fail loud on a stale pre-Phase-40.1
                    // Python install (missing `ScopeOrigin.path`, the 46.4
                    // report's caveat). Shared with the `dump --format
                    // record` path so stale usage can never silently pass
                    // anywhere, incl. the parser-parity tooling.
                    crate::ros::launch_dump::ensure_python_scope_paths(&dump)?;
                    dump
                }
            };
            (dump, Some(launch_path), None)
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
    if let Some(rp) = &record_reuse_path {
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
    // Phase 46.5 — the model↔record hash binding (`meta.record`,
    // `verify_model_record_binding`) is retired along with the companion:
    // the model has been a self-sufficient spawn source since 46.3b/46.4,
    // so there is no longer a second artifact to bind or mismatch. (The
    // `record_reuse_path`, when `--record <path>` was given, is already
    // folded into `meta.inputs` above — that's still real provenance.)

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

    // Phase 45.6 — `--explain` on `resolve`: render from the SystemModel
    // this invocation just built (`model.execution.sched`/`tiers`/
    // `bindings`), not from `derived` — proves the embedded artifact itself
    // carries enough to render without a fresh derive, the same guarantee
    // `replay --model --explain` depends on. We still have the fresh
    // `resolved_platform`/`index` in scope, so the footer is byte-identical
    // to what `check --sched --explain` would print for the same inputs.
    if args.explain {
        if model.execution.sched.is_some() || !model.execution.bindings.is_empty() {
            let footer = match &resolved_platform {
                Some(p) => sched_loader::explain_footer(p, Some(&index)),
                None => sched_loader::explain_footer_from_model(&model),
            };
            eprint!(
                "{}",
                sched_loader::render_explain_from_model(&model, &args.target, &footer)
            );
        } else {
            eprintln!(
                "note: --explain has no effect without a resolved scheduling platform file \
                 (pass --sched <path>, or ship one via the overlay/provider channels)"
            );
        }
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
