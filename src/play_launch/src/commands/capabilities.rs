//! Capability management commands (`setcap` / `verify`)
//!
//! `play_launch` uses Linux capabilities instead of running as root:
//! - `CAP_SYS_NICE` on the main `play_launch` binary lets `--sched` apply RT
//!   scheduling (`SCHED_FIFO`/`SCHED_RR` + CPU affinity) to spawned processes.
//! - `CAP_SYS_PTRACE` on the `play_launch_io_helper` binary lets it read
//!   `/proc/[pid]/io` for privileged processes it doesn't own.

use eyre::Context;
use std::{path::PathBuf, process};

/// Find the path to the I/O helper binary.
///
/// Calls the wrapper script with `--binary-path` to get the actual binary
/// location. The `play_launch_io_helper` on PATH is a Python shim, so this
/// indirection is required — do NOT replace it with `which`.
fn find_io_helper_path() -> eyre::Result<PathBuf> {
    let output = process::Command::new("play_launch_io_helper")
        .arg("--binary-path")
        .output()
        .wrap_err("Failed to run 'play_launch_io_helper --binary-path'")?;

    if output.status.success() {
        let path = String::from_utf8_lossy(&output.stdout).trim().to_string();
        Ok(PathBuf::from(path))
    } else {
        let stderr = String::from_utf8_lossy(&output.stderr);
        eyre::bail!(
            "play_launch_io_helper not found.\n\
            Ensure play_launch is properly installed (pip install play_launch).\n\
            {stderr}"
        )
    }
}

/// The running play_launch binary — the process that calls
/// `sched_setscheduler`. `current_exe()` reads `/proc/self/exe` (the real ELF
/// even when launched via the pip shim); canonicalize so setcap targets the
/// file, not a symlink.
fn main_binary_path() -> eyre::Result<PathBuf> {
    std::env::current_exe()
        .wrap_err("Failed to resolve the play_launch binary path")?
        .canonicalize()
        .wrap_err("Failed to canonicalize the play_launch binary path")
}

/// Handle the `setcap` subcommand.
///
/// Grants both capabilities `play_launch` needs:
/// - `CAP_SYS_NICE` on the main binary (for `--sched` RT scheduling).
/// - `CAP_SYS_PTRACE` on the I/O helper (for `/proc/[pid]/io` monitoring).
///
/// If the I/O helper path can't be resolved, it's skipped with a warning
/// rather than failing the whole command — the scheduling capability must
/// still be granted.
pub fn handle_setcap() -> eyre::Result<()> {
    println!("This requires sudo privileges.\n");

    let mut results = Vec::new();

    // CAP_SYS_NICE on the main binary.
    let bin = main_binary_path()?;
    println!("Setting CAP_SYS_NICE on {}", bin.display());
    let status = process::Command::new("sudo")
        .args(["setcap", "cap_sys_nice+ep"])
        .arg(&bin)
        .status()
        .wrap_err("Failed to run setcap")?;
    if !status.success() {
        eyre::bail!("Failed to set cap_sys_nice on {}", bin.display());
    }
    let output = process::Command::new("getcap")
        .arg(&bin)
        .output()
        .wrap_err("Failed to run getcap")?;
    let stdout = String::from_utf8_lossy(&output.stdout);
    if stdout.contains("cap_sys_nice") {
        results.push(format!(
            "✓ play_launch can apply RT scheduling: {}",
            stdout.trim()
        ));
    } else {
        eyre::bail!(
            "cap_sys_nice may not have been set correctly on {}. Output: {}",
            bin.display(),
            stdout
        );
    }

    // CAP_SYS_PTRACE on the I/O helper (skip, don't fail, if unresolvable).
    match find_io_helper_path() {
        Ok(io_helper) => {
            println!("Setting CAP_SYS_PTRACE on {}", io_helper.display());
            let status = process::Command::new("sudo")
                .args(["setcap", "cap_sys_ptrace+ep"])
                .arg(&io_helper)
                .status()
                .wrap_err("Failed to run setcap")?;
            if !status.success() {
                eyre::bail!("Failed to set cap_sys_ptrace on {}", io_helper.display());
            }
            let output = process::Command::new("getcap")
                .arg(&io_helper)
                .output()
                .wrap_err("Failed to run getcap")?;
            let stdout = String::from_utf8_lossy(&output.stdout);
            if stdout.contains("cap_sys_ptrace") {
                results.push(format!("✓ I/O helper ready: {}", stdout.trim()));
            } else {
                eyre::bail!(
                    "cap_sys_ptrace may not have been set correctly on {}. Output: {}",
                    io_helper.display(),
                    stdout
                );
            }
        }
        Err(err) => {
            println!("\n⚠ Could not resolve I/O helper path, skipping CAP_SYS_PTRACE: {err}");
        }
    }

    println!();
    for line in &results {
        println!("{line}");
    }
    println!("\nNote: Rerun this command after upgrading/rebuilding play_launch.");

    Ok(())
}

/// Handle the `verify` subcommand.
///
/// Checks both capabilities and exits non-zero if either is missing (usable
/// as a CI/preflight check).
pub fn handle_verify() -> eyre::Result<()> {
    let mut all_ok = true;

    // CAP_SYS_NICE on the main binary.
    match main_binary_path() {
        Ok(bin) => {
            let output = process::Command::new("getcap")
                .arg(&bin)
                .output()
                .wrap_err("Failed to run getcap")?;
            let stdout = String::from_utf8_lossy(&output.stdout);
            if stdout.contains("cap_sys_nice=ep") || stdout.contains("cap_sys_nice+ep") {
                println!("✓ RT scheduling (cap_sys_nice): {}", stdout.trim());
            } else if !stdout.trim().is_empty() {
                println!(
                    "⚠ play_launch binary has unexpected capabilities: {}",
                    stdout.trim()
                );
                println!("  Expected: cap_sys_nice=ep");
                println!("  Run: play_launch setcap");
                all_ok = false;
            } else {
                println!("✗ RT scheduling (cap_sys_nice) not set on play_launch binary");
                println!("  Run: play_launch setcap");
                all_ok = false;
            }
        }
        Err(err) => {
            println!("✗ Could not resolve play_launch binary path: {err}");
            println!("  Run: play_launch setcap");
            all_ok = false;
        }
    }

    // CAP_SYS_PTRACE on the I/O helper.
    match find_io_helper_path() {
        Ok(io_helper) => {
            let output = process::Command::new("getcap")
                .arg(&io_helper)
                .output()
                .wrap_err("Failed to run getcap")?;
            let stdout = String::from_utf8_lossy(&output.stdout);
            if stdout.contains("cap_sys_ptrace=ep") || stdout.contains("cap_sys_ptrace+ep") {
                println!("✓ I/O monitoring (cap_sys_ptrace): {}", stdout.trim());
            } else if !stdout.trim().is_empty() {
                println!(
                    "⚠ I/O helper has unexpected capabilities: {}",
                    stdout.trim()
                );
                println!("  Expected: cap_sys_ptrace=ep");
                println!("  Run: play_launch setcap");
                all_ok = false;
            } else {
                println!("✗ I/O monitoring (cap_sys_ptrace) not set on I/O helper");
                println!("  Run: play_launch setcap");
                all_ok = false;
            }
        }
        Err(err) => {
            println!("✗ Could not resolve I/O helper path: {err}");
            println!("  Run: play_launch setcap");
            all_ok = false;
        }
    }

    if !all_ok {
        std::process::exit(1);
    }

    Ok(())
}
