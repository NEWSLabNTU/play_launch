//! Launch command - parse, resolve, and replay a launch file in one shot
//!
//! Phase 47.B4 — the internal round-trip is fully in-memory: no
//! `record.json` (or any other file) is written to disk anywhere on this
//! path. The launch file is parsed straight into a [`LaunchDump`], a
//! SystemModel is built from it in memory, and both are handed directly to
//! the replay engine (`commands::up::play`) — the same function
//! `play_launch up` uses, just called in-process instead of through a
//! second CLI invocation.

use crate::cli::options::LaunchCommandArgs;
use eyre::Result;
use std::collections::BTreeMap;
use tracing::info;

// `resolve_launch_file` was a byte-identical copy of the library's, whose
// error message this fix wave also had to correct in two places at once.
// One implementation now, in the library.

/// Handle the 'launch' subcommand: parse → resolve → replay, all in memory.
pub fn handle_launch(cmd_args: &LaunchCommandArgs) -> Result<()> {
    // `--check` is the `launch` command's own flag; everything else comes
    // from the `LaunchArgs` block shared with `dump launch`.
    let check = cmd_args.check;
    let args = &cmd_args.launch;

    play_launch_parser::block_command_substitution(args.block_commands);

    // Same positional quirk `resolve` and `check` correct: with a direct
    // launch-file PATH, clap binds the first `KEY:=VALUE` to the optional
    // `<launch_file>` positional. `launch` had the defect too — `play_launch
    // launch my.launch.xml mode:=lidar` spawned the default node set — so it
    // calls the one shared helper alongside the other verbs.
    let (launch_file, launch_arguments) = ros_launch_resolve::verbs::reclassify_launch_file_arg(
        args.launch_file.as_deref(),
        &args.launch_arguments,
    );

    let runtime = super::common::build_tokio_runtime()?;
    runtime.block_on(async move {
        info!("Step 1/3: Parsing launch file...");
        let (dump, launch_path) = ros_launch_resolve::verbs::parse_to_launch_dump(
            &args.package_or_path,
            launch_file.as_deref(),
            &launch_arguments,
            args.parser.into(),
        )
        .await?;
        info!(
            "Parsed: {} node(s), {} container(s), {} composable node(s)",
            dump.node.len(),
            dump.container.len(),
            dump.load_node.len()
        );

        info!("Step 2/3: Resolving SystemModel...");
        let arg_binding: BTreeMap<String, String> =
            super::parse_launch_arguments(&launch_arguments)
                .into_iter()
                .collect();
        let model = ros_launch_resolve::model::build_checked_model(
            ros_launch_resolve::model::ModelBuildInputs {
                dump: &dump,
                launch_path: Some(&launch_path),
                // `launch` builds the model in memory and never writes it, so
                // `meta.inputs` portability is moot here; the grandparent
                // fallback preserves the behavior this path had before the
                // field existed.
                bringup_root: None,
                arg_binding,
                contracts: args.common.contract_opts.contracts.as_deref(),
                no_provider_contracts: args.common.contract_opts.no_provider_contracts,
                sched: args.common.sched_opts.sched.as_deref(),
                system: None,
                target: args.common.sched_opts.target.as_str(),
                explain: false,
            },
        )?;

        if check {
            // `build_checked_model` above already ran contracts and scheduling
            // and returns Err when the checker reports errors, so reaching
            // here means the model is clean. Report and stop before spawning.
            //
            // `SystemModel::meta` has no `warnings` field; the closest analog
            // is `meta.diagnostics: Vec<String>` — lenient, non-fatal notes
            // (e.g. non-portable absolute input paths) embedded alongside the
            // checker's own warnings (see `model.rs`'s "lenient diagnostics
            // embed like checker warnings").
            //
            // `println!`, not `info!`: a verdict the user explicitly asked
            // for with `--check` is output, not logging. Under
            // `RUST_LOG=error` an `info!` verdict vanished entirely, leaving
            // `--check` silent. `run --check` prints the same way.
            let diagnostics = model.meta.diagnostics.len();
            println!(
                "check passed: {} node(s), {} diagnostic(s)",
                model.structure.nodes.len(),
                diagnostics
            );
            for d in &model.meta.diagnostics {
                println!("  diagnostic: {d}");
            }
            return Ok(());
        }

        info!("Step 3/3: Replaying launch execution...");
        super::up::play(dump, std::sync::Arc::new(model), &args.common).await
    })
}
