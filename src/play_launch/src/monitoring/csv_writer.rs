//! CSV writing for per-process metrics and system-wide statistics.

use csv::Writer;
use eyre::{Result, WrapErr};
use std::{
    collections::HashMap,
    fs::{self, File},
    path::{Path, PathBuf},
    time::SystemTime,
};
use tracing::debug;

use super::resource_monitor::{ResourceMetrics, SystemStats};

/// Manages CSV writers for per-process metrics and system-wide stats.
#[derive(Default)]
pub struct CsvState {
    /// Keyed by CSV PATH, not by PID.
    ///
    /// The path depends only on the node's output directory, so keying by PID
    /// meant a respawn — new PID, same directory — took the vacant-entry
    /// branch and `File::create`d the file again, destroying every sample
    /// collected before the restart and permanently freezing the web UI's
    /// Metrics tab (its SSE handler tails from `SeekFrom::End(0)`, so after a
    /// truncation it sits past EOF and never emits another row). Keying by
    /// path means the writer is reused and rows APPEND; the `pid` column
    /// already distinguishes them. It also bounds the map by node count
    /// rather than by respawn count — each entry holds an open file handle.
    pub csv_writers: HashMap<PathBuf, Writer<File>>,
    pub system_csv_writer: Option<Writer<File>>, // System-wide stats CSV writer
}

impl CsvState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn write_csv(&mut self, output_dir: &Path, metrics: &ResourceMetrics) -> Result<()> {
        let pid = metrics.pid;
        let csv_path = get_csv_path(output_dir);

        // Get or create the writer for this FILE. First write for a path
        // truncates deliberately: `initialize_metrics_csv` seeds a shorter
        // legacy header, and this replaces it with METRICS_CSV_HEADER.
        // Subsequent writes — including after a respawn — reuse the writer.
        if let std::collections::hash_map::Entry::Vacant(e) =
            self.csv_writers.entry(csv_path.clone())
        {
            // Create parent directories
            if let Some(parent) = csv_path.parent() {
                fs::create_dir_all(parent).wrap_err_with(|| {
                    format!("Failed to create directory: {}", parent.display())
                })?;
            }

            let file = File::create(&csv_path)
                .wrap_err_with(|| format!("Failed to create CSV file: {}", csv_path.display()))?;
            let mut writer = Writer::from_writer(file);

            // Write header
            writer
                .write_record(METRICS_CSV_HEADER)
                .wrap_err("Failed to write CSV header")?;

            writer.flush().wrap_err("Failed to flush CSV header")?;
            e.insert(writer);
        }

        let writer = self
            .csv_writers
            .get_mut(&csv_path)
            .ok_or_else(|| eyre::eyre!("CSV writer not found for {}", csv_path.display()))?;

        // Format timestamp
        let timestamp_str = format_timestamp(metrics.timestamp);

        // Write data row
        writer
            .write_record(&[
                timestamp_str,
                metrics.pid.to_string(),
                format!("{:.2}", metrics.cpu_percent),
                metrics.cpu_user_time.to_string(),
                metrics.rss_bytes.to_string(),
                metrics.vms_bytes.to_string(),
                metrics.io_read_bytes.to_string(),
                metrics.io_write_bytes.to_string(),
                metrics.total_read_bytes.to_string(),
                metrics.total_write_bytes.to_string(),
                metrics.io_syscr.to_string(),
                metrics.io_syscw.to_string(),
                metrics.io_storage_read_bytes.to_string(),
                metrics.io_storage_write_bytes.to_string(),
                metrics.io_cancelled_write_bytes.to_string(),
                metrics
                    .total_read_rate_bps
                    .map(|v| format!("{:.2}", v))
                    .unwrap_or_default(),
                metrics
                    .total_write_rate_bps
                    .map(|v| format!("{:.2}", v))
                    .unwrap_or_default(),
                metrics.state.as_str().to_string(),
                metrics.num_threads.to_string(),
                metrics.num_fds.to_string(),
                metrics.num_processes.to_string(),
                metrics
                    .gpu_memory_bytes
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics
                    .gpu_utilization_percent
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics
                    .gpu_memory_utilization_percent
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics
                    .gpu_temperature_celsius
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics
                    .gpu_power_milliwatts
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics
                    .gpu_graphics_clock_mhz
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics
                    .gpu_memory_clock_mhz
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics.tcp_connections.to_string(),
                metrics.udp_connections.to_string(),
            ])
            .wrap_err_with(|| format!("Failed to write CSV row for PID {}", pid))?;

        writer
            .flush()
            .wrap_err_with(|| format!("Failed to flush CSV for PID {}", pid))?;

        Ok(())
    }

    /// Write system-wide statistics to CSV file.
    /// Creates system_stats.csv at the log directory root level.
    pub fn write_system_csv(&mut self, log_dir: &Path, stats: &SystemStats) -> Result<()> {
        // Initialize CSV writer on first call
        if self.system_csv_writer.is_none() {
            let csv_path = log_dir.join("system_stats.csv");

            // Create parent directories if needed
            if let Some(parent) = csv_path.parent() {
                fs::create_dir_all(parent).wrap_err_with(|| {
                    format!("Failed to create directory: {}", parent.display())
                })?;
            }

            let file = File::create(&csv_path).wrap_err_with(|| {
                format!(
                    "Failed to create system stats CSV file: {}",
                    csv_path.display()
                )
            })?;
            let mut writer = Writer::from_writer(file);

            // Write header
            writer
                .write_record(SYSTEM_CSV_HEADER)
                .wrap_err("Failed to write system stats CSV header")?;

            writer
                .flush()
                .wrap_err("Failed to flush system stats CSV header")?;
            self.system_csv_writer = Some(writer);
        }

        let writer = self
            .system_csv_writer
            .as_mut()
            .ok_or_else(|| eyre::eyre!("System CSV writer not initialized"))?;

        // Format timestamp
        let timestamp_str = format_timestamp(stats.timestamp);

        // Write data row
        writer
            .write_record(&[
                timestamp_str,
                format!("{:.2}", stats.cpu_percent),
                stats.cpu_count.to_string(),
                stats.total_memory_bytes.to_string(),
                stats.used_memory_bytes.to_string(),
                stats.available_memory_bytes.to_string(),
                stats.total_swap_bytes.to_string(),
                stats.used_swap_bytes.to_string(),
                stats.network_rx_bytes.to_string(),
                stats.network_tx_bytes.to_string(),
                stats
                    .network_rx_rate_bps
                    .map(|v| format!("{:.2}", v))
                    .unwrap_or_default(),
                stats
                    .network_tx_rate_bps
                    .map(|v| format!("{:.2}", v))
                    .unwrap_or_default(),
                stats.disk_read_bytes.to_string(),
                stats.disk_write_bytes.to_string(),
                stats
                    .disk_read_rate_bps
                    .map(|v| format!("{:.2}", v))
                    .unwrap_or_default(),
                stats
                    .disk_write_rate_bps
                    .map(|v| format!("{:.2}", v))
                    .unwrap_or_default(),
                stats
                    .gpu_utilization_percent
                    .map(|v| format!("{:.2}", v))
                    .unwrap_or_default(),
                stats
                    .gpu_memory_used_bytes
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                stats
                    .gpu_memory_total_bytes
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                stats
                    .gpu_frequency_mhz
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                stats
                    .gpu_power_milliwatts
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                stats
                    .gpu_temperature_celsius
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
            ])
            .wrap_err("Failed to write system stats CSV row")?;

        writer
            .flush()
            .wrap_err("Failed to flush system stats CSV")?;

        Ok(())
    }
}

/// Per-process metrics CSV header columns.
const METRICS_CSV_HEADER: &[&str] = &[
    "timestamp",
    "pid",
    "cpu_percent",
    "cpu_user_secs",
    "rss_bytes",
    "vms_bytes",
    "io_read_bytes",
    "io_write_bytes",
    "total_read_bytes",
    "total_write_bytes",
    "io_syscr",
    "io_syscw",
    "io_storage_read_bytes",
    "io_storage_write_bytes",
    "io_cancelled_write_bytes",
    "total_read_rate_bps",
    "total_write_rate_bps",
    "state",
    "num_threads",
    "num_fds",
    "num_processes",
    "gpu_memory_bytes",
    "gpu_utilization_percent",
    "gpu_memory_utilization_percent",
    "gpu_temperature_celsius",
    "gpu_power_milliwatts",
    "gpu_graphics_clock_mhz",
    "gpu_memory_clock_mhz",
    "tcp_connections",
    "udp_connections",
];

/// System-wide stats CSV header columns.
const SYSTEM_CSV_HEADER: &[&str] = &[
    "timestamp",
    "cpu_percent",
    "cpu_count",
    "total_memory_bytes",
    "used_memory_bytes",
    "available_memory_bytes",
    "total_swap_bytes",
    "used_swap_bytes",
    "network_rx_bytes",
    "network_tx_bytes",
    "network_rx_rate_bps",
    "network_tx_rate_bps",
    "disk_read_bytes",
    "disk_write_bytes",
    "disk_read_rate_bps",
    "disk_write_rate_bps",
    "gpu_utilization_percent",
    "gpu_memory_used_bytes",
    "gpu_memory_total_bytes",
    "gpu_frequency_mhz",
    "gpu_power_milliwatts",
    "gpu_temperature_celsius",
];

fn get_csv_path(output_dir: &Path) -> PathBuf {
    output_dir.join("metrics.csv")
}

/// Initialize CSV file with headers for a given output directory.
/// This should be called as soon as monitoring is enabled to ensure CSV files
/// exist even if the node dies before metrics are collected.
/// IDEMPOTENT: returns early if the file already exists. It is called on every
/// spawn, including a RESPAWN, and `File::create` truncates — so this used to
/// destroy the metrics collected before a node restarted, which is half of why
/// the web UI's Metrics tab froze mid-run. "Ensure it exists" must not mean
/// "destroy it if it does".
pub fn initialize_metrics_csv(output_dir: &Path) -> Result<()> {
    let csv_path = output_dir.join("metrics.csv");

    if csv_path.exists() {
        debug!(
            "metrics CSV already exists, keeping it: {}",
            csv_path.display()
        );
        return Ok(());
    }

    // Create parent directories if needed
    if let Some(parent) = csv_path.parent() {
        fs::create_dir_all(parent)
            .wrap_err_with(|| format!("Failed to create directory: {}", parent.display()))?;
    }

    let file = File::create(&csv_path)
        .wrap_err_with(|| format!("Failed to create CSV file: {}", csv_path.display()))?;
    let mut writer = Writer::from_writer(file);

    // Write header (note: initialize_metrics_csv uses a slightly different header
    // than the full METRICS_CSV_HEADER -- it omits the extended I/O fields that were
    // added later. We preserve this for backwards compatibility.)
    writer
        .write_record([
            "timestamp",
            "pid",
            "cpu_percent",
            "cpu_user_secs",
            "rss_bytes",
            "vms_bytes",
            "io_read_bytes",
            "io_write_bytes",
            "total_read_bytes",
            "total_write_bytes",
            "total_read_rate_bps",
            "total_write_rate_bps",
            "state",
            "num_threads",
            "num_fds",
            "num_processes",
            "gpu_memory_bytes",
            "gpu_utilization_percent",
            "gpu_memory_utilization_percent",
            "gpu_temperature_celsius",
            "gpu_power_milliwatts",
            "gpu_graphics_clock_mhz",
            "gpu_memory_clock_mhz",
            "tcp_connections",
            "udp_connections",
        ])
        .wrap_err("Failed to write CSV header")?;

    writer.flush().wrap_err("Failed to flush CSV header")?;

    debug!("Initialized metrics CSV at: {}", csv_path.display());
    Ok(())
}

/// Format SystemTime as ISO 8601 string.
pub fn format_timestamp(time: SystemTime) -> String {
    match time.duration_since(SystemTime::UNIX_EPOCH) {
        Ok(duration) => {
            let secs = duration.as_secs();
            let millis = duration.subsec_millis();
            // Format as YYYY-MM-DDTHH:MM:SS.mmmZ
            let datetime =
                chrono::DateTime::<chrono::Utc>::from_timestamp(secs as i64, millis * 1_000_000)
                    .unwrap_or_default();
            datetime.to_rfc3339_opts(chrono::SecondsFormat::Millis, true)
        }
        Err(_) => String::from("1970-01-01T00:00:00.000Z"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn test_format_timestamp() {
        let time = SystemTime::UNIX_EPOCH + Duration::from_secs(1234567890);
        let formatted = format_timestamp(time);
        assert!(formatted.starts_with("2009-02-13"));
    }

    fn sample(pid: u32) -> crate::monitoring::resource_monitor::ResourceMetrics {
        use crate::monitoring::resource_monitor::{ProcessState, ResourceMetrics};
        ResourceMetrics {
            timestamp: SystemTime::UNIX_EPOCH,
            pid,
            cpu_percent: 1.0,
            cpu_user_time: 0,
            rss_bytes: 0,
            vms_bytes: 0,
            io_read_bytes: 0,
            io_write_bytes: 0,
            total_read_bytes: 0,
            total_write_bytes: 0,
            io_syscr: 0,
            io_syscw: 0,
            io_storage_read_bytes: 0,
            io_storage_write_bytes: 0,
            io_cancelled_write_bytes: 0,
            total_read_rate_bps: None,
            total_write_rate_bps: None,
            state: ProcessState::Sleeping,
            num_threads: 1,
            num_fds: 1,
            num_processes: 1,
            gpu_memory_bytes: None,
            gpu_utilization_percent: None,
            gpu_memory_utilization_percent: None,
            gpu_temperature_celsius: None,
            gpu_power_milliwatts: None,
            gpu_graphics_clock_mhz: None,
            gpu_memory_clock_mhz: None,
            tcp_connections: 0,
            udp_connections: 0,
        }
    }

    /// `initialize_metrics_csv` runs on every spawn, including a respawn, so
    /// it must not clobber an existing file. Before this it called
    /// `File::create` unconditionally — the second of two truncation paths
    /// that emptied metrics.csv mid-run.
    #[test]
    fn initialize_metrics_csv_keeps_an_existing_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        let dir = tmp.path();

        initialize_metrics_csv(dir).unwrap();
        let mut state = CsvState::new();
        state.write_csv(dir, &sample(1001)).unwrap();

        // Node restarts: the actor calls this again for the same directory.
        initialize_metrics_csv(dir).unwrap();

        let body = std::fs::read_to_string(dir.join("metrics.csv")).unwrap();
        assert!(
            body.contains(",1001,"),
            "re-initialising destroyed the existing samples:\n{body}"
        );
    }

    /// A respawned node must APPEND to its metrics.csv, not truncate it.
    ///
    /// `csv_writers` was keyed by PID while the file path depends only on the
    /// output directory, so a respawn — new PID, same directory — took the
    /// vacant-entry branch and `File::create`d the file again, destroying
    /// every sample collected before the restart.
    ///
    /// It also froze the web UI's Metrics tab permanently: the SSE handler
    /// tails this file from `SeekFrom::End(0)`, so once it is truncated the
    /// reader sits past EOF and never emits another row. Reproduced against a
    /// real run before this test was written — 11 lines before the kill, 10
    /// after, containing only the new PID.
    #[test]
    fn a_respawned_pid_appends_instead_of_truncating() {
        let tmp = tempfile::TempDir::new().unwrap();
        let dir = tmp.path();
        let mut state = CsvState::new();

        state.write_csv(dir, &sample(1001)).unwrap();
        state.write_csv(dir, &sample(1001)).unwrap();
        // Same node, restarted: different PID, same output directory.
        state.write_csv(dir, &sample(2002)).unwrap();

        let body = std::fs::read_to_string(dir.join("metrics.csv")).unwrap();
        let rows: Vec<&str> = body.lines().skip(1).filter(|l| !l.is_empty()).collect();

        assert_eq!(rows.len(), 3, "all three samples must survive:\n{body}");
        assert!(
            rows.iter().any(|r| r.contains(",1001,")),
            "pre-respawn history was destroyed:\n{body}"
        );
        assert!(
            rows.iter().any(|r| r.contains(",2002,")),
            "post-respawn sample missing:\n{body}"
        );
        assert_eq!(
            body.matches("timestamp,pid").count(),
            1,
            "header must be written exactly once:\n{body}"
        );
    }
}
