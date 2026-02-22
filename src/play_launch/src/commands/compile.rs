//! Compile command - compile a launch file to WASM

use crate::cli::options::CompileArgs;
use eyre::Result;
use std::time::Instant;
use tracing::info;

/// Handle the 'compile' subcommand
pub fn handle_compile(args: &CompileArgs) -> Result<()> {
    let start = Instant::now();

    // 1. Resolve launch file path
    let launch_path =
        super::launch::resolve_launch_file(&args.package_or_path, args.launch_file.as_deref())?;

    // 2. Parse launch arguments (KEY:=VALUE format)
    let cli_args = super::parse_launch_arguments(&args.launch_arguments);

    // 3. Build IR via Rust parser
    let program = play_launch_parser::analyze_launch_file_with_args(&launch_path, cli_args)
        .map_err(|e| eyre::eyre!("Failed to analyze launch file: {e}"))?;

    // 4. Compile IR to WASM
    let wasm_bytes = play_launch_wasm_codegen::compile_to_wasm(&program)
        .map_err(|e| eyre::eyre!("WASM compilation failed: {e}"))?;

    // 5. Write WASM output
    std::fs::write(&args.output, &wasm_bytes)
        .map_err(|e| eyre::eyre!("Failed to write {}: {e}", args.output.display()))?;

    let elapsed = start.elapsed();
    info!(
        "Compiled to {} ({} bytes) in {:.2}s",
        args.output.display(),
        wasm_bytes.len(),
        elapsed.as_secs_f64()
    );

    Ok(())
}
