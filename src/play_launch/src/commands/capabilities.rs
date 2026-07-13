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

// ---------------------------------------------------------------------------
// WHY THE MAIN `play_launch` BINARY MUST NEVER BE GIVEN FILE CAPABILITIES
//
// Setting a file capability on an ELF makes the kernel run it in
// secure-execution mode (`AT_SECURE=1`). The dynamic linker then IGNORES
// `LD_LIBRARY_PATH` (and `LD_PRELOAD`). The main binary links ~22 ROS shared
// libraries that are found ONLY via `LD_LIBRARY_PATH` (set by ROS's
// `setup.bash`) and it carries no `DT_RUNPATH`, so a capability on it makes it
// fail at startup:
//
//     error while loading shared libraries: libcomposition_interfaces...so
//
// This is exactly why the privileged work is delegated to `play_launch_io_helper`,
// which links ZERO ROS libraries and is therefore safe to capability-grant.
//
// Do NOT add `setcap` on the main binary here. For RT scheduling either run
// play_launch as root (`sudo -E play_launch ...`) or use the privileged-helper
// path (phase 38.10).
// ---------------------------------------------------------------------------

/// Handle the `setcap` subcommand.
///
/// Grants `CAP_SYS_PTRACE` to the I/O helper (for `/proc/[pid]/io` monitoring).
/// The helper is ROS-free, so a file capability on it is safe — see the note
/// above for why the main binary is deliberately left uncapped.
pub fn handle_setcap() -> eyre::Result<()> {
    println!("This requires sudo privileges.\n");

    let io_helper = find_io_helper_path()?;

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
        println!("\n✓ I/O helper ready: {}", stdout.trim());
    } else {
        eyre::bail!(
            "cap_sys_ptrace may not have been set correctly on {}. Output: {}",
            io_helper.display(),
            stdout
        );
    }

    println!("\nNote: Rerun this command after upgrading/rebuilding play_launch.");
    println!(
        "RT scheduling (`--sched`) currently requires running play_launch as root, \
         e.g. `sudo -E play_launch launch ...` — the main binary cannot be given \
         CAP_SYS_NICE (it would break ROS library loading)."
    );

    Ok(())
}

/// Handle the `verify` subcommand.
///
/// Checks the I/O helper's `CAP_SYS_PTRACE` and exits non-zero if it's missing
/// (usable as a CI/preflight check). It also warns if the MAIN binary has picked
/// up a file capability, which would break ROS library loading (see the note at
/// the top of this file).
pub fn handle_verify() -> eyre::Result<()> {
    let mut all_ok = true;

    // The main binary must have NO capabilities — one would put it in
    // secure-execution mode and break LD_LIBRARY_PATH-based ROS lib loading.
    if let Ok(bin) = std::env::current_exe().and_then(|p| p.canonicalize())
        && let Ok(output) = process::Command::new("getcap").arg(&bin).output()
    {
        let stdout = String::from_utf8_lossy(&output.stdout);
        if stdout.contains("cap_") {
            println!(
                "✗ play_launch binary has file capabilities set: {}",
                stdout.trim()
            );
            println!(
                "  This BREAKS ROS library loading (AT_SECURE drops LD_LIBRARY_PATH)."
            );
            println!("  Remove them:  sudo setcap -r {}", bin.display());
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
