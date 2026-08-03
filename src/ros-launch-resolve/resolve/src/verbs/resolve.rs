//! `resolve` — emit a SystemModel (RFC-0050 /
//! docs/design/system-model.md): parse the launch tree, bind args, filter
//! conditions, merge scopes, run the contract checker, derive the
//! scheduling plan, and serialize the fully-resolved artifact.
//!
//! Refuses to emit when the checker reports Error severity — a SystemModel
//! in hand is always a checked one. Warnings embed in `meta.diagnostics`.

use std::collections::BTreeMap;
use std::path::PathBuf;

use eyre::{Context, Result};

use crate::model::{ModelBuildInputs, build_checked_model};
use crate::verbs::ParserBackend;

/// Everything `resolve` reads. Owned plain values — see [`crate::verbs`].
pub struct ResolveInputs {
    /// Package name, or a path to a launch file.
    pub package_or_path: String,
    /// Launch file name, when `package_or_path` is a package name.
    pub launch_file: Option<String>,
    /// `KEY:=VALUE` launch arguments.
    pub launch_arguments: Vec<String>,
    /// Bringup package root that `meta.inputs[].path` are recorded relative to.
    pub bringup_root: Option<PathBuf>,
    /// Overlay root for user-supplied contracts.
    pub contracts: Option<PathBuf>,
    /// Disable the provider-sidecar channel.
    pub no_provider_contracts: bool,
    /// Scheduling platform file (v2 `.yaml` or legacy `.toml`).
    pub sched: Option<PathBuf>,
    /// The integrator `system.toml`.
    pub system: Option<PathBuf>,
    /// Scheduling target the platform file must declare.
    pub target: String,
    /// Which launch frontend to parse with.
    pub parser: ParserBackend,
    /// Output path for the SystemModel YAML. `-` writes to stdout.
    pub out: String,
    /// Print the merged scheduling plan with per-node provenance.
    pub explain: bool,
}

pub fn run(inputs: ResolveInputs) -> Result<()> {
    // Positional quirk: with a direct launch-file PATH, the second
    // positional (`launch_file`) can swallow the first `KEY:=VALUE` arg.
    // See [`super::reclassify_launch_file_arg`].
    let (launch_file, launch_arguments) =
        super::reclassify_launch_file_arg(inputs.launch_file.as_deref(), &inputs.launch_arguments);

    let runtime = super::build_tokio_runtime()?;
    let (dump, launch_path) = runtime.block_on(super::parse_to_launch_dump(
        &inputs.package_or_path,
        launch_file.as_deref(),
        &launch_arguments,
        inputs.parser,
    ))?;

    let arg_binding: BTreeMap<String, String> = super::parse_launch_arguments(&launch_arguments)
        .into_iter()
        .collect();

    let model = build_checked_model(ModelBuildInputs {
        dump: &dump,
        launch_path: Some(&launch_path),
        bringup_root: inputs.bringup_root.as_deref(),
        arg_binding,
        contracts: inputs.contracts.as_deref(),
        no_provider_contracts: inputs.no_provider_contracts,
        sched: inputs.sched.as_deref(),
        system: inputs.system.as_deref(),
        target: inputs.target.as_str(),
        explain: inputs.explain,
    })?;

    let yaml = model
        .to_yaml_string()
        .wrap_err("serializing SystemModel to YAML")?;

    if inputs.out == "-" {
        print!("{yaml}");
    } else {
        std::fs::write(&inputs.out, &yaml)
            .wrap_err_with(|| format!("writing SystemModel to {}", inputs.out))?;
        eprintln!(
            "SystemModel: {} ({} nodes, {} topics, {} contracts-carrying endpoints, \
             {} tier(s), {} warning(s))",
            inputs.out,
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
