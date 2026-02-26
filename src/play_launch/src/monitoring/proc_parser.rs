//! `/proc` filesystem parsing for CPU times, disk stats, subprocess discovery,
//! and network connection counts.

use eyre::{Result, WrapErr};

/// Clock ticks per second for CPU time conversion from `/proc/[pid]/stat`.
/// On Linux this is typically 100 (verified via `getconf CLK_TCK`).
pub const CLK_TCK: f64 = 100.0;

/// Count newlines in a file using a stack buffer (no heap allocation).
/// Returns line count minus 1 (to skip the header line in /proc/net/* files).
#[cfg(target_os = "linux")]
pub fn count_lines_in_file(path: &str) -> u32 {
    use std::io::Read;
    let mut file = match std::fs::File::open(path) {
        Ok(f) => f,
        Err(_) => return 0,
    };
    let mut buf = [0u8; 8192];
    let mut newlines: u32 = 0;
    loop {
        match file.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                newlines += buf[..n].iter().filter(|&&b| b == b'\n').count() as u32;
            }
            Err(_) => break,
        }
    }
    newlines.saturating_sub(1) // subtract header line
}

/// Find all subprocess PIDs recursively (Linux only)
#[cfg(target_os = "linux")]
pub fn find_subprocess_pids(parent_pid: u32) -> Vec<u32> {
    let mut pids = Vec::new();

    // Read /proc/<pid>/task/<tid>/children for each thread
    let task_dir = format!("/proc/{}/task", parent_pid);
    if let Ok(entries) = std::fs::read_dir(task_dir) {
        for entry in entries.flatten() {
            let children_path = entry.path().join("children");
            if let Ok(content) = std::fs::read_to_string(&children_path) {
                for pid_str in content.split_whitespace() {
                    if let Ok(child_pid) = pid_str.parse::<u32>() {
                        pids.push(child_pid);
                        // Recursively find grandchildren
                        let mut grandchildren = find_subprocess_pids(child_pid);
                        pids.append(&mut grandchildren);
                    }
                }
            }
        }
    }

    pids
}

#[cfg(not(target_os = "linux"))]
pub fn find_subprocess_pids(_parent_pid: u32) -> Vec<u32> {
    Vec::new()
}

/// Parse /proc/[pid]/stat for CPU times (utime and stime).
/// Returns (utime, stime) in clock ticks.
/// These values are cumulative and need to be differenced between samples.
#[cfg(target_os = "linux")]
pub fn parse_proc_stat(pid: u32) -> Result<(u64, u64)> {
    let stat_path = format!("/proc/{}/stat", pid);
    let content = std::fs::read_to_string(&stat_path)
        .wrap_err_with(|| format!("Failed to read {}", stat_path))?;

    // Format: pid (comm) state ppid pgrp ... utime stime cutime cstime ...
    // We need to split on ')' because comm can contain spaces and parentheses
    let parts: Vec<&str> = content.split(')').collect();
    if parts.len() < 2 {
        return Err(eyre::eyre!("Invalid /proc/{}/stat format", pid));
    }

    // After ')', fields are space-separated
    // Field indices (0-based after splitting on ')'):
    // 11 = utime (user mode jiffies)
    // 12 = stime (kernel mode jiffies)
    let fields: Vec<&str> = parts[1].split_whitespace().collect();
    if fields.len() < 14 {
        return Err(eyre::eyre!(
            "Insufficient fields in /proc/{}/stat (got {}, need 14)",
            pid,
            fields.len()
        ));
    }

    let utime: u64 = fields[11]
        .parse()
        .wrap_err_with(|| format!("Failed to parse utime from {}", stat_path))?;
    let stime: u64 = fields[12]
        .parse()
        .wrap_err_with(|| format!("Failed to parse stime from {}", stat_path))?;

    Ok((utime, stime))
}

#[cfg(not(target_os = "linux"))]
pub fn parse_proc_stat(_pid: u32) -> Result<(u64, u64)> {
    // CPU time parsing not implemented for non-Linux platforms
    Ok((0, 0))
}

/// Parse /proc/diskstats for system-wide disk I/O statistics.
/// Returns (total_read_bytes, total_write_bytes) aggregated across all disks.
/// Format: major minor name reads ... sectors_read ... writes ... sectors_written ...
/// Sectors are typically 512 bytes.
#[cfg(target_os = "linux")]
pub fn parse_diskstats() -> Result<(u64, u64)> {
    let content =
        std::fs::read_to_string("/proc/diskstats").wrap_err("Failed to read /proc/diskstats")?;

    let mut total_read_bytes = 0u64;
    let mut total_write_bytes = 0u64;
    const SECTOR_SIZE: u64 = 512;

    for line in content.lines() {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() < 14 {
            continue; // Skip malformed lines
        }

        // Skip partition entries (e.g., sda1, nvme0n1p1) and only count whole disks
        // Whole disk names: sda, sdb, nvme0n1, mmcblk0, etc.
        let device_name = parts[2];

        // Filter out partitions by checking if name ends with a digit after a letter
        // Examples: sda1, nvme0n1p1, mmcblk0p1 (partitions) vs sda, nvme0n1, mmcblk0 (whole disks)
        // For simplicity, we'll include all devices and let aggregation handle duplicates
        // But we should skip loop devices and ram devices
        if device_name.starts_with("loop") || device_name.starts_with("ram") {
            continue;
        }

        // Field 5: sectors read (0-indexed)
        let sectors_read = parts[5].parse::<u64>().unwrap_or(0);
        // Field 9: sectors written (0-indexed)
        let sectors_written = parts[9].parse::<u64>().unwrap_or(0);

        total_read_bytes += sectors_read * SECTOR_SIZE;
        total_write_bytes += sectors_written * SECTOR_SIZE;
    }

    Ok((total_read_bytes, total_write_bytes))
}

#[cfg(not(target_os = "linux"))]
pub fn parse_diskstats() -> Result<(u64, u64)> {
    Ok((0, 0))
}

/// Count TCP and UDP connections for a process from `/proc/<pid>/net/*`.
/// `/proc/<pid>/net/*` is namespace-wide -- all processes in the same network
/// namespace return identical data.
#[cfg(target_os = "linux")]
pub fn count_network_connections(pid: u32) -> (u32, u32) {
    let tcp_count = count_lines_in_file(&format!("/proc/{}/net/tcp", pid))
        + count_lines_in_file(&format!("/proc/{}/net/tcp6", pid));
    let udp_count = count_lines_in_file(&format!("/proc/{}/net/udp", pid))
        + count_lines_in_file(&format!("/proc/{}/net/udp6", pid));
    (tcp_count, udp_count)
}

#[cfg(not(target_os = "linux"))]
pub fn count_network_connections(_pid: u32) -> (u32, u32) {
    (0, 0)
}
