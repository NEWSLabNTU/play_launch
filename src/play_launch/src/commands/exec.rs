//! Exec command - execute a compiled WASM launch module

use crate::cli::options::ExecArgs;
use eyre::{Context, Result};
use std::time::Instant;
use tracing::info;

/// Handle the 'exec' subcommand
pub fn handle_exec(args: &ExecArgs) -> Result<()> {
    let start = Instant::now();

    // 1. Read WASM bytes
    let wasm_bytes = std::fs::read(&args.wasm_file)
        .wrap_err_with(|| format!("Failed to read {}", args.wasm_file.display()))?;

    // 2. Parse runtime arguments (KEY:=VALUE format)
    let cli_args = super::parse_launch_arguments(&args.args);

    // 3. Execute WASM module
    let record = play_launch_wasm_runtime::execute_wasm(&wasm_bytes, cli_args)
        .map_err(|e| eyre::eyre!("WASM execution failed: {e}"))?;

    // 4. Write record.json
    let json_output =
        serde_json::to_string_pretty(&record).wrap_err("Failed to serialize record.json")?;
    std::fs::write(&args.output, json_output)
        .wrap_err_with(|| format!("Failed to write {}", args.output.display()))?;

    let elapsed = start.elapsed();
    info!(
        "Executed WASM and wrote {} in {:.2}s",
        args.output.display(),
        elapsed.as_secs_f64()
    );

    Ok(())
}
