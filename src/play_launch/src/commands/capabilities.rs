//! Capability management commands (`setcap` / `verify`)
//!
//! `play_launch` uses Linux capabilities instead of running as root:
//! - `CAP_SYS_NICE` on the ROS-free `play_launch_rt_helper` binary lets it
//!   apply RT scheduling (`SCHED_FIFO`/`SCHED_RR` + CPU affinity) to spawned
//!   processes on behalf of the unprivileged main process (phase 38.10).
//! - `CAP_SYS_PTRACE` on the `play_launch_io_helper` binary lets it read
//!   `/proc/[pid]/io` for privileged processes it doesn't own.
//!
//! One capability per helper — they are never pooled onto a single binary.

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

/// Find the path to the RT helper binary.
///
/// Unlike the I/O helper, `play_launch_rt_helper` has no Python wrapper shim
/// (it is ROS-free and installed as a plain binary), so the `--binary-path`
/// indirection used by [`find_io_helper_path`] doesn't apply here. Instead
/// this reuses the exe-dir/env/ROS-path/PATH lookup chain that
/// `RtHelperClient::spawn()` already relies on
/// (`execution::rt_helper_client::find_helper_binary`), so the two lookups
/// can never drift apart.
fn find_rt_helper_path() -> eyre::Result<PathBuf> {
    crate::execution::rt_helper_client::find_helper_binary()
}

/// Does the resolved RT helper binary carry `CAP_SYS_NICE`?
///
/// Used by the pre-spawn scheduling preflight (`commands::run`,
/// `commands::replay`) to decide whether a non-root run can still apply RT
/// scheduling via the helper. `getcap` prints `cap_sys_nice=ep` while
/// `setcap` is invoked with `cap_sys_nice+ep` — both forms are accepted.
/// Returns `false` on any resolution/lookup failure (helper missing, `getcap`
/// not installed, etc.) rather than propagating an error — callers treat
/// "unknown" the same as "not privileged".
pub fn rt_helper_has_cap_sys_nice() -> bool {
    let Ok(rt_helper) = find_rt_helper_path() else {
        return false;
    };
    let Ok(output) = process::Command::new("getcap").arg(&rt_helper).output() else {
        return false;
    };
    let stdout = String::from_utf8_lossy(&output.stdout);
    stdout.contains("cap_sys_nice=ep") || stdout.contains("cap_sys_nice+ep")
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
// This is exactly why ALL privileged work is delegated to small, ROS-free
// helper binaries instead:
// - `play_launch_io_helper` (CAP_SYS_PTRACE) reads `/proc/[pid]/io` for
//   privileged processes it doesn't own.
// - `play_launch_rt_helper` (CAP_SYS_NICE, phase 38.10) applies RT scheduling
//   (SCHED_FIFO/RR + affinity) on behalf of the unprivileged main process.
//
// Do NOT add `setcap` on the main binary here — RT scheduling is now fully
// supported unprivileged via `play_launch_rt_helper`; running the whole ROS
// stack as root is no longer required or recommended.
// ---------------------------------------------------------------------------

/// Handle the `setcap` subcommand.
///
/// Grants `CAP_SYS_PTRACE` to the I/O helper (for `/proc/[pid]/io` monitoring)
/// and `CAP_SYS_NICE` to the RT helper (for unprivileged RT scheduling). Both
/// helpers are ROS-free, so a file capability on either is safe — see the
/// note above for why the main binary is deliberately left uncapped.
///
/// Each helper is granted independently: if one can't be resolved (e.g. not
/// built/installed), this warns and skips it rather than failing the whole
/// command, so `just setcap` still does what it can.
pub fn handle_setcap() -> eyre::Result<()> {
    println!("This requires sudo privileges.\n");

    // Self-heal: a file capability on the main binary is ALWAYS wrong (it sets
    // AT_SECURE, the loader drops LD_LIBRARY_PATH, and the ROS libs vanish).
    // Strip it so a previously-broken install is repaired by running `setcap`.
    if let Ok(bin) = std::env::current_exe().and_then(|p| p.canonicalize())
        && let Ok(output) = process::Command::new("getcap").arg(&bin).output()
        && String::from_utf8_lossy(&output.stdout).contains("cap_")
    {
        println!(
            "! main binary has file capabilities — removing (they break ROS library loading)"
        );
        let status = process::Command::new("sudo")
            .args(["setcap", "-r"])
            .arg(&bin)
            .status()
            .wrap_err("Failed to run setcap -r on the main binary")?;
        if !status.success() {
            eyre::bail!("Failed to remove capabilities from {}", bin.display());
        }
        println!("  removed.\n");
    }

    let mut any_failed = false;

    match find_io_helper_path() {
        Ok(io_helper) => {
            if let Err(e) = grant_cap(&io_helper, "cap_sys_ptrace", "I/O helper") {
                println!("! {e:#}");
                any_failed = true;
            }
        }
        Err(e) => {
            println!("! Could not resolve I/O helper path, skipping: {e:#}");
            any_failed = true;
        }
    }

    match find_rt_helper_path() {
        Ok(rt_helper) => {
            if let Err(e) = grant_cap(&rt_helper, "cap_sys_nice", "RT helper") {
                println!("! {e:#}");
                any_failed = true;
            }
        }
        Err(e) => {
            println!("! Could not resolve RT helper path, skipping: {e:#}");
            any_failed = true;
        }
    }

    println!("\nNote: Rerun this command after upgrading/rebuilding play_launch.");
    println!(
        "RT scheduling (`--sched`) now works unprivileged: play_launch delegates the \
         apply syscalls to play_launch_rt_helper (CAP_SYS_NICE), so no root/sudo is \
         needed to run play_launch itself."
    );

    if any_failed {
        eyre::bail!("one or more capabilities could not be granted (see warnings above)");
    }

    Ok(())
}

/// `sudo setcap <cap>+ep <path>`, then verify with `getcap` and print a `✓`
/// line. Shared by both helpers in [`handle_setcap`] so the grant+verify
/// dance isn't duplicated.
fn grant_cap(path: &std::path::Path, cap: &str, label: &str) -> eyre::Result<()> {
    println!("Setting {} on {}", cap, path.display());
    let status = process::Command::new("sudo")
        .args(["setcap", &format!("{cap}+ep")])
        .arg(path)
        .status()
        .wrap_err_with(|| format!("Failed to run setcap on {}", path.display()))?;
    if !status.success() {
        eyre::bail!("Failed to set {cap} on {}", path.display());
    }

    let output = process::Command::new("getcap")
        .arg(path)
        .output()
        .wrap_err("Failed to run getcap")?;
    let stdout = String::from_utf8_lossy(&output.stdout);

    if stdout.contains(cap) {
        println!("✓ {label} ready: {}", stdout.trim());
        Ok(())
    } else {
        eyre::bail!(
            "{cap} may not have been set correctly on {}. Output: {}",
            path.display(),
            stdout
        )
    }
}

/// Handle the `verify` subcommand.
///
/// Checks all three binaries in the capability matrix and exits non-zero if
/// any is wrong (usable as a CI/preflight check):
/// - main binary → must have NO capabilities
/// - I/O helper → `CAP_SYS_PTRACE`
/// - RT helper → `CAP_SYS_NICE`
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

    all_ok &= verify_cap(find_io_helper_path(), "cap_sys_ptrace", "I/O monitoring");
    all_ok &= verify_cap(find_rt_helper_path(), "cap_sys_nice", "RT scheduling");

    if !all_ok {
        std::process::exit(1);
    }

    Ok(())
}

/// Verify `label` (e.g. "RT scheduling") carries `cap` on the resolved helper
/// path, printing a `✓`/`⚠`/`✗` line. Accepts both `getcap`'s `=ep` output and
/// `setcap`'s `+ep` input form. Returns `false` (and never errors out — this
/// is a report-and-continue check) if the cap is missing/wrong or the helper
/// couldn't be resolved.
fn verify_cap(helper_path: eyre::Result<PathBuf>, cap: &str, label: &str) -> bool {
    let helper = match helper_path {
        Ok(p) => p,
        Err(err) => {
            println!("✗ Could not resolve helper for {label} ({cap}): {err}");
            println!("  Run: play_launch setcap");
            return false;
        }
    };

    let Ok(output) = process::Command::new("getcap").arg(&helper).output() else {
        println!("✗ Failed to run getcap on {}", helper.display());
        println!("  Run: play_launch setcap");
        return false;
    };
    let stdout = String::from_utf8_lossy(&output.stdout);

    let expected_eq = format!("{cap}=ep");
    let expected_plus = format!("{cap}+ep");
    if stdout.contains(&expected_eq) || stdout.contains(&expected_plus) {
        println!("✓ {label} ({cap}): {}", stdout.trim());
        true
    } else if !stdout.trim().is_empty() {
        println!("⚠ helper for {label} has unexpected capabilities: {}", stdout.trim());
        println!("  Expected: {expected_eq}");
        println!("  Run: play_launch setcap");
        false
    } else {
        println!("✗ {label} ({cap}) not set on {}", helper.display());
        println!("  Run: play_launch setcap");
        false
    }
}
