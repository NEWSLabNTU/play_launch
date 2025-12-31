//! I/O helper capability management commands

use eyre::Context;
use std::{path::PathBuf, process};

/// Find the path to the I/O helper binary
fn find_io_helper_path() -> eyre::Result<PathBuf> {
    // Call the wrapper script with --binary-path to get the actual binary location
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

/// Handle the 'setcap-io-helper' subcommand
pub fn handle_setcap_io_helper() -> eyre::Result<()> {
    let io_helper = find_io_helper_path()?;

    println!("Setting CAP_SYS_PTRACE on {}", io_helper.display());
    println!("This requires sudo privileges.\n");

    // Run setcap with sudo
    let status = process::Command::new("sudo")
        .args(["setcap", "cap_sys_ptrace+ep"])
        .arg(&io_helper)
        .status()
        .wrap_err("Failed to run setcap")?;

    if !status.success() {
        eyre::bail!("Failed to set capability");
    }

    // Verify the capability was set
    let output = process::Command::new("getcap")
        .arg(&io_helper)
        .output()
        .wrap_err("Failed to run getcap")?;

    let stdout = String::from_utf8_lossy(&output.stdout);

    if stdout.contains("cap_sys_ptrace") {
        println!("\n✓ I/O helper ready: {}", stdout.trim());
        println!("\nNote: Rerun this command after upgrading play_launch.");
    } else {
        eyre::bail!(
            "Capability may not have been set correctly. Output: {}",
            stdout
        );
    }

    Ok(())
}

/// Handle the 'verify-io-helper' subcommand
pub fn handle_verify_io_helper() -> eyre::Result<()> {
    let io_helper = find_io_helper_path()?;

    let output = process::Command::new("getcap")
        .arg(&io_helper)
        .output()
        .wrap_err("Failed to run getcap")?;

    let stdout = String::from_utf8_lossy(&output.stdout);

    // getcap outputs "=ep" but setcap uses "+ep" - check for both formats
    if stdout.contains("cap_sys_ptrace=ep") || stdout.contains("cap_sys_ptrace+ep") {
        println!("✓ I/O helper ready: {}", stdout.trim());
        Ok(())
    } else if !stdout.trim().is_empty() {
        println!(
            "⚠ I/O helper has unexpected capabilities: {}",
            stdout.trim()
        );
        println!("  Expected: cap_sys_ptrace=ep");
        println!("  Run: play_launch setcap-io-helper");
        std::process::exit(1);
    } else {
        println!("✗ I/O helper has no capabilities set");
        println!("  Run: play_launch setcap-io-helper");
        std::process::exit(1);
    }
}
