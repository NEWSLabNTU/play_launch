//! `play_launch export` — argument mapping only (phase 106 W5). The verb is
//! `ros_launch_resolve::verbs::export`, which the web UI's Export also calls.

use clap::Parser;
use eyre::{Result, bail};
use ros_launch_resolve::verbs::export::{self, ExportInputs, Form};

use crate::cli::options::{Command, ExportArgs, ExportForm, Options};

pub fn handle_export(args: &ExportArgs) -> Result<()> {
    if let Some(file) = &args.again {
        let words = export::header_command(file)?;
        let opts = Options::try_parse_from(&words).map_err(|e| {
            eyre::eyre!(
                "the command in {}'s header no longer parses: {e}",
                file.display()
            )
        })?;
        let Command::Export(again) = opts.command else {
            bail!(
                "{}'s header does not hold an export command",
                file.display()
            );
        };
        if again.again.is_some() {
            bail!("{}'s header asks for --again itself", file.display());
        }
        eprintln!("Regenerating {} from its header", file.display());
        return handle_export(&again);
    }

    let form = match args.form {
        ExportForm::Include => Form::Include,
        ExportForm::SelfContained => Form::SelfContained,
        ExportForm::Command => Form::Command,
    };
    let spec = args.selection.spec()?;
    let out_dir = std::path::absolute(&args.out)?;
    if args.package_or_path.is_none() && args.model.is_none() {
        bail!("export needs a launch file (`<pkg> <file>` or a path), --model, or --again");
    }

    // The command recorded in the header: canonical selectors, absolute
    // paths, so `--again` means the same from any directory.
    let mut source = vec!["play_launch".to_string(), "export".to_string()];
    if let Some(p) = &args.package_or_path {
        let path = std::path::Path::new(p);
        source.push(if path.exists() {
            std::fs::canonicalize(path)?.display().to_string()
        } else {
            p.clone()
        });
    }
    source.extend(args.launch_file.iter().cloned());
    source.extend(args.launch_arguments.iter().cloned());
    for s in &spec.only {
        source.extend(["--only".to_string(), s.to_string()]);
    }
    for s in &spec.exclude {
        source.extend(["--exclude".to_string(), s.to_string()]);
    }
    if spec.no_deps {
        source.push("--no-deps".into());
    }
    if let Some(m) = &args.model {
        source.push("--model".into());
        source.push(std::path::absolute(m)?.display().to_string());
    }
    source.extend(["--form".to_string(), form.as_str().to_string()]);
    source.extend(["-o".to_string(), out_dir.display().to_string()]);
    if let Some(n) = &args.name {
        source.extend(["--name".to_string(), n.clone()]);
    }

    let inputs = ExportInputs {
        package_or_path: args.package_or_path.clone(),
        launch_file: args.launch_file.clone(),
        launch_arguments: args.launch_arguments.clone(),
        parser: args.parser.into(),
        contracts: args.contracts.clone(),
        no_provider_contracts: args.no_provider_contracts,
        sched: None,
        target: "posix".to_string(),
        process_per_composable: false,
        model: args.model.clone(),
        selection: spec,
        form,
        out_dir,
        name: args.name.clone(),
        source_command: source,
    };
    let runtime = super::common::build_tokio_runtime()?;
    let report = runtime.block_on(export::export(inputs))?;
    print_report(&report);
    Ok(())
}

pub fn print_report(r: &export::ExportReport) {
    println!("Wrote {}", r.file.display());
    for o in &r.opened {
        println!("  opened (written one level down, the include alone did not match): {o}");
    }
    for l in &r.lost {
        println!("  not expressible, noted in a comment: {l}");
    }
    for d in &r.differences {
        eprintln!("warning: the exported file does not reproduce the selection exactly: {d}");
    }
    println!("Run it with:");
    for c in &r.run {
        println!("  {c}");
    }
}
