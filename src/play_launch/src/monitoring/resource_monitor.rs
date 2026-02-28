use eyre::{Result, WrapErr};
use nvml_wrapper::Nvml;
use std::{
    collections::HashMap,
    path::PathBuf,
    sync::{Arc, Mutex},
    time::{Duration, SystemTime},
};
use sysinfo::{Networks, Pid, System};
use tokio::sync::watch;
use tracing::{debug, warn};

use super::{
    csv_writer::CsvState,
    proc_parser::{self, CLK_TCK},
    system_stats,
};

// Re-export types that external code references via `resource_monitor::`.
pub use super::{csv_writer::initialize_metrics_csv, system_stats::SystemStats};

/// Resource metrics for a single process at a point in time
#[derive(Debug, Clone)]
pub struct ResourceMetrics {
    pub timestamp: SystemTime,
    pub pid: u32,

    // CPU metrics
    pub cpu_percent: f64,
    pub cpu_user_time: u64, // Total accumulated CPU time (utime + stime) in seconds

    // Memory metrics
    pub rss_bytes: u64, // Resident Set Size
    pub vms_bytes: u64, // Virtual Memory Size

    // I/O metrics (disk only - from sysinfo)
    pub io_read_bytes: u64,
    pub io_write_bytes: u64,

    // Total I/O metrics (all I/O including network - from /proc/[pid]/io)
    pub total_read_bytes: u64,  // rchar field
    pub total_write_bytes: u64, // wchar field

    // Extended I/O metrics (from /proc/[pid]/io via helper daemon)
    pub io_syscr: u64,                 // Read syscalls count
    pub io_syscw: u64,                 // Write syscalls count
    pub io_storage_read_bytes: u64,    // Actual bytes read from storage (excludes cache)
    pub io_storage_write_bytes: u64,   // Actual bytes written to storage (excludes cache)
    pub io_cancelled_write_bytes: u64, // Write bytes later truncated

    // I/O rates (bytes per second, calculated from previous sample)
    pub total_read_rate_bps: Option<f64>,
    pub total_write_rate_bps: Option<f64>,

    // Process info
    pub state: ProcessState,
    pub num_threads: u32,
    pub num_fds: u32,       // File descriptors
    pub num_processes: u32, // Number of processes in tree (parent + children)

    // GPU metrics (optional - only populated if GPU monitoring enabled)
    pub gpu_memory_bytes: Option<u64>,
    pub gpu_utilization_percent: Option<u32>,
    pub gpu_memory_utilization_percent: Option<u32>,
    pub gpu_temperature_celsius: Option<u32>,
    pub gpu_power_milliwatts: Option<u32>,
    pub gpu_graphics_clock_mhz: Option<u32>,
    pub gpu_memory_clock_mhz: Option<u32>,

    // Network metrics (Linux-specific, counts of active connections)
    pub tcp_connections: u32,
    pub udp_connections: u32,
}

#[derive(Debug, Clone, Copy)]
pub enum ProcessState {
    Running,
    Sleeping,
    #[allow(dead_code)] // May be used in future OS implementations
    Waiting,
    Zombie,
    Stopped,
    Unknown,
}

impl ProcessState {
    pub fn as_str(&self) -> &'static str {
        match self {
            ProcessState::Running => "Running",
            ProcessState::Sleeping => "Sleeping",
            ProcessState::Waiting => "Waiting",
            ProcessState::Zombie => "Zombie",
            ProcessState::Stopped => "Stopped",
            ProcessState::Unknown => "Unknown",
        }
    }
}

/// Configuration for resource monitoring
#[derive(Debug, Clone)]
pub struct MonitorConfig {
    pub sample_interval_ms: u64,
}

/// Previous sample for rate calculation
#[derive(Debug, Clone)]
struct PreviousSample {
    timestamp: SystemTime,
    total_read_bytes: u64,
    total_write_bytes: u64,
    utime: u64, // User CPU time in clock ticks
    stime: u64, // System CPU time in clock ticks
}

/// Resource monitor with sysinfo and NVML integration
pub struct ResourceMonitor {
    system: System,
    networks: Networks, // Network interface monitor
    csv_state: CsvState,
    nvml: Option<Nvml>,
    gpu_device_count: u32,
    previous_samples: HashMap<u32, PreviousSample>, // PID -> previous I/O sample
    previous_system_sample: Option<system_stats::PreviousSystemSample>,

    // I/O helper daemon for reading /proc/[pid]/io with CAP_SYS_PTRACE
    io_helper: Option<super::io_helper_client::IoHelperClient>,
    io_helper_unavailable: bool, // Track if helper failed to spawn (warn once)
    io_stats_cache: HashMap<u32, play_launch::ipc::ProcIoStats>, // Cache for batch I/O reads

    // Network connection counts cached once per tick (/proc/net/* is namespace-wide)
    net_connections_cache: Option<(u32, u32)>, // (tcp, udp) -- same for all processes in namespace

    // Subprocess PID cache -- refreshed by time interval (process trees are stable in steady-state)
    subprocess_cache: HashMap<u32, Vec<u32>>, // parent PID -> child PIDs
    subprocess_cache_last_refresh: std::time::Instant,
}

impl ResourceMonitor {
    pub fn new(nvml: Option<Nvml>) -> Result<Self> {
        // Use System::new() instead of new_all() to avoid loading everything upfront
        // We'll refresh only the processes we need in the monitoring loop

        // Get GPU device count if NVML available
        let gpu_device_count = if let Some(ref nvml) = nvml {
            nvml.device_count()
                .wrap_err("Failed to get GPU device count")?
        } else {
            0
        };

        debug!(
            "ResourceMonitor initialized with {} GPU devices",
            gpu_device_count
        );

        // Test GPU process enumeration compatibility (one-time check at startup)
        if let Some(ref nvml) = nvml {
            if gpu_device_count > 0 {
                match nvml.device_by_index(0) {
                    Ok(device) => match device.running_compute_processes() {
                        Ok(_) => {
                            debug!("GPU process enumeration test: OK");
                        }
                        Err(e) => {
                            warn!(
                                    "GPU process enumeration not supported on this system: {}. \
                                     GPU metrics will not be collected. This is common on some GPU architectures \
                                     (e.g., Jetson/Tegra GPUs). CPU, memory, and I/O monitoring will work normally.",
                                    e
                                );
                        }
                    },
                    Err(e) => {
                        warn!("Failed to access GPU device 0: {}", e);
                    }
                }
            }
        }

        // Initialize System with CPU information
        // This is required for CPU usage calculation - sysinfo needs global CPU times
        // to compute per-process CPU percentages
        let mut system = System::new();
        system.refresh_cpu_all(); // Initial CPU refresh to establish baseline

        // I/O helper will be set externally by the async task (no nested runtime!)
        Ok(Self {
            system,
            networks: Networks::new_with_refreshed_list(),
            csv_state: CsvState::new(),
            nvml,
            gpu_device_count,
            previous_samples: HashMap::new(),
            previous_system_sample: None,
            io_helper: None,
            io_helper_unavailable: true, // Will be set to false if helper spawns successfully
            io_stats_cache: HashMap::new(),
            net_connections_cache: None,
            subprocess_cache: HashMap::new(),
            subprocess_cache_last_refresh: std::time::Instant::now(),
        })
    }

    fn collect_metrics(&mut self, pid: u32) -> Result<ResourceMetrics> {
        let pid_obj = Pid::from_u32(pid);

        // Get process from system
        let process = self
            .system
            .process(pid_obj)
            .ok_or_else(|| eyre::eyre!("Process {} not found", pid))?;

        // Discover subprocesses for aggregation (cached, refreshed by time interval)
        let subprocess_pids = self.subprocess_cache.get(&pid).cloned().unwrap_or_default();

        // Parse /proc/[pid]/stat for accurate CPU times (in clock ticks)
        // Aggregate from parent + all children
        let (mut utime, mut stime) = proc_parser::parse_proc_stat(pid)?;
        let mut rss_bytes = process.memory();
        let mut vms_bytes = process.virtual_memory();
        let disk_usage = process.disk_usage();
        let mut io_read_bytes = disk_usage.total_read_bytes;
        let mut io_write_bytes = disk_usage.total_written_bytes;
        let mut num_threads = process.tasks().map(|t| t.len() as u32).unwrap_or(1);

        // Aggregate from subprocesses
        for child_pid in &subprocess_pids {
            // Get CPU times from /proc/[child_pid]/stat
            if let Ok((child_utime, child_stime)) = proc_parser::parse_proc_stat(*child_pid) {
                utime += child_utime;
                stime += child_stime;
            }

            // Get other metrics from sysinfo
            let child_pid_obj = Pid::from_u32(*child_pid);
            if let Some(child_process) = self.system.process(child_pid_obj) {
                rss_bytes += child_process.memory();
                vms_bytes += child_process.virtual_memory();
                let child_disk = child_process.disk_usage();
                io_read_bytes += child_disk.total_read_bytes;
                io_write_bytes += child_disk.total_written_bytes;
                num_threads += child_process.tasks().map(|t| t.len() as u32).unwrap_or(1);
            }
        }

        // Count open file descriptors (Linux-specific)
        #[cfg(target_os = "linux")]
        let num_fds = std::fs::read_dir(format!("/proc/{}/fd", pid))
            .map(|entries| entries.count() as u32)
            .unwrap_or(0);
        #[cfg(not(target_os = "linux"))]
        let num_fds = 0;

        // Map sysinfo process status to our ProcessState enum
        let state = match process.status() {
            sysinfo::ProcessStatus::Run => ProcessState::Running,
            sysinfo::ProcessStatus::Sleep => ProcessState::Sleeping,
            sysinfo::ProcessStatus::Idle => ProcessState::Sleeping,
            sysinfo::ProcessStatus::Zombie => ProcessState::Zombie,
            sysinfo::ProcessStatus::Stop => ProcessState::Stopped,
            _ => ProcessState::Unknown,
        };

        let num_processes = 1 + subprocess_pids.len() as u32;

        // Collect GPU metrics if NVML available
        let (
            gpu_memory_bytes,
            gpu_utilization_percent,
            gpu_memory_utilization_percent,
            gpu_temperature_celsius,
            gpu_power_milliwatts,
            gpu_graphics_clock_mhz,
            gpu_memory_clock_mhz,
        ) = system_stats::collect_gpu_metrics(self.nvml.as_ref(), self.gpu_device_count, pid)?;

        // Collect network connection counts
        let (tcp_connections, udp_connections) = self.collect_network_connections(pid);

        // Get I/O stats from cache (populated by batch read in monitoring loop)
        let io_stats = self.io_stats_cache.get(&pid).cloned().unwrap_or({
            play_launch::ipc::ProcIoStats {
                rchar: 0,
                wchar: 0,
                syscr: 0,
                syscw: 0,
                read_bytes: 0,
                write_bytes: 0,
                cancelled_write_bytes: 0,
            }
        });

        // Extract all I/O fields
        let total_read_bytes = io_stats.rchar;
        let total_write_bytes = io_stats.wchar;
        let io_syscr = io_stats.syscr;
        let io_syscw = io_stats.syscw;
        let io_storage_read_bytes = io_stats.read_bytes;
        let io_storage_write_bytes = io_stats.write_bytes;
        let io_cancelled_write_bytes = io_stats.cancelled_write_bytes;

        // Calculate I/O rates and CPU percentage from previous sample
        let current_time = SystemTime::now();
        let (total_read_rate_bps, total_write_rate_bps, cpu_percent) = if let Some(prev) =
            self.previous_samples.get(&pid)
        {
            let time_diff = current_time
                .duration_since(prev.timestamp)
                .unwrap_or(Duration::from_secs(0))
                .as_secs_f64();

            if time_diff > 0.0 {
                // Calculate I/O rates
                let read_rate =
                    (total_read_bytes.saturating_sub(prev.total_read_bytes)) as f64 / time_diff;
                let write_rate =
                    (total_write_bytes.saturating_sub(prev.total_write_bytes)) as f64 / time_diff;

                // Calculate CPU percentage using /proc/[pid]/stat data (utime + stime)
                // utime and stime are in clock ticks (jiffies)
                // Formula: ((delta_utime + delta_stime) / CLK_TCK / delta_wall_time) * 100
                let cpu_ticks_delta =
                    (utime.saturating_sub(prev.utime) + stime.saturating_sub(prev.stime)) as f64;
                let cpu_time_seconds = cpu_ticks_delta / CLK_TCK;
                let cpu_pct = (cpu_time_seconds / time_diff) * 100.0;

                (Some(read_rate), Some(write_rate), cpu_pct)
            } else {
                (None, None, 0.0)
            }
        } else {
            // No previous sample, can't calculate rate - CPU% will be 0 for first sample
            (None, None, 0.0)
        };

        // Store current sample for next iteration
        self.previous_samples.insert(
            pid,
            PreviousSample {
                timestamp: current_time,
                total_read_bytes,
                total_write_bytes,
                utime,
                stime,
            },
        );

        // Convert total CPU time from clock ticks to seconds
        let cpu_user_time = ((utime + stime) as f64 / CLK_TCK) as u64;

        Ok(ResourceMetrics {
            timestamp: current_time,
            pid,
            cpu_percent,
            cpu_user_time,
            rss_bytes,
            vms_bytes,
            io_read_bytes,
            io_write_bytes,
            total_read_bytes,
            total_write_bytes,
            io_syscr,
            io_syscw,
            io_storage_read_bytes,
            io_storage_write_bytes,
            io_cancelled_write_bytes,
            total_read_rate_bps,
            total_write_rate_bps,
            state,
            num_threads,
            num_fds,
            num_processes,
            gpu_memory_bytes,
            gpu_utilization_percent,
            gpu_memory_utilization_percent,
            gpu_temperature_celsius,
            gpu_power_milliwatts,
            gpu_graphics_clock_mhz,
            gpu_memory_clock_mhz,
            tcp_connections,
            udp_connections,
        })
    }

    /// Count TCP and UDP connections (Linux-specific).
    /// /proc/<pid>/net/* is namespace-wide -- all processes in the same network
    /// namespace return identical data. We cache the result once per tick and
    /// reuse it for every process, turning ~400 reads into 4.
    #[cfg(target_os = "linux")]
    fn collect_network_connections(&mut self, pid: u32) -> (u32, u32) {
        if let Some(cached) = self.net_connections_cache {
            return cached;
        }
        let result = proc_parser::count_network_connections(pid);
        self.net_connections_cache = Some(result);
        result
    }

    #[cfg(not(target_os = "linux"))]
    fn collect_network_connections(&self, _pid: u32) -> (u32, u32) {
        (0, 0)
    }
}

/// Async monitoring task (tokio-based replacement for old thread-based monitoring)
pub async fn run_monitoring_task(
    config: MonitorConfig,
    log_dir: PathBuf,
    process_registry: Arc<Mutex<HashMap<u32, PathBuf>>>,
    nvml: Option<Nvml>,
    mut shutdown_rx: watch::Receiver<bool>,
    metrics_broadcaster: Option<Arc<crate::web::SystemMetricsBroadcaster>>,
) -> Result<()> {
    debug!("Starting async monitoring task...");

    let mut monitor = ResourceMonitor::new(nvml)?;

    // Try to spawn I/O helper (already in async context, no nested runtime!)
    match super::io_helper_client::IoHelperClient::spawn().await {
        Ok(client) => {
            debug!("I/O helper spawned successfully");
            monitor.io_helper = Some(client);
            monitor.io_helper_unavailable = false;
        }
        Err(e) => {
            warn!(
                "I/O helper unavailable: {}. Privileged processes will have zero I/O stats.",
                e
            );
            monitor.io_helper_unavailable = true;
        }
    }

    let mut interval = tokio::time::interval(Duration::from_millis(config.sample_interval_ms));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

    debug!(
        "Monitoring task started with interval: {}ms",
        config.sample_interval_ms
    );

    loop {
        tokio::select! {
            biased;

            // Check shutdown first (biased ensures this is checked before tick)
            _ = shutdown_rx.changed() => {
                if *shutdown_rx.borrow() {
                    debug!("Monitoring task received shutdown signal");
                    break;
                }
            }

            // Periodic monitoring tick
            _ = interval.tick() => {
                // Get snapshot of current processes to monitor
                let processes = process_registry.lock().unwrap().clone();

                if processes.is_empty() {
                    continue;
                }

                // Refresh only the specific processes we're monitoring
                let pids_to_refresh: Vec<Pid> =
                    processes.keys().map(|&pid| Pid::from_u32(pid)).collect();

                if !pids_to_refresh.is_empty() {

                    // Refresh global CPU times first (needed for per-process CPU %).
                    // refresh_cpu_all() is called later in collect_system_stats() which
                    // is a superset, but we need global times BEFORE process refresh.
                    monitor.system.refresh_cpu_all();

                    // Refresh processes
                    monitor.system.refresh_processes_specifics(
                        sysinfo::ProcessesToUpdate::Some(&pids_to_refresh),
                        false,
                        sysinfo::ProcessRefreshKind::new()
                            .with_cpu()
                            .with_disk_usage()
                            .with_memory(),
                    );
                }

                // Clear per-tick caches
                monitor.io_stats_cache.clear();
                monitor.net_connections_cache = None;

                // Refresh subprocess tree cache every 1 second,
                // OR immediately when the monitored PID set changes (new process registered).
                // Process trees are stable in steady-state; no need to walk /proc every tick.
                const SUBPROCESS_CACHE_INTERVAL: Duration = Duration::from_secs(1);
                let pids_changed = processes.len() != monitor.subprocess_cache.len()
                    || processes.keys().any(|pid| !monitor.subprocess_cache.contains_key(pid));
                if pids_changed || monitor.subprocess_cache_last_refresh.elapsed() >= SUBPROCESS_CACHE_INTERVAL {
                    monitor.subprocess_cache.clear();
                    for &pid in processes.keys() {
                        let children = proc_parser::find_subprocess_pids(pid);
                        monitor.subprocess_cache.insert(pid, children);
                    }
                    monitor.subprocess_cache_last_refresh = std::time::Instant::now();
                }

                // Batch read I/O stats for all PIDs
                if let Some(ref mut helper) = monitor.io_helper {
                    let pids: Vec<u32> = processes.keys().copied().collect();

                    if !pids.is_empty() {
                        match helper.read_proc_io_batch(&pids).await {
                            Ok(results) => {
                                for result in results {
                                    match result.result {
                                        Ok(stats) => {
                                            monitor.io_stats_cache.insert(result.pid, stats);
                                        }
                                        Err(e) => {
                                            debug!("I/O read failed for PID {}: {:?}", result.pid, e);
                                        }
                                    }
                                }
                            }
                            Err(e) => {
                                if !monitor.io_helper_unavailable {
                                    warn!(
                                        "I/O helper batch request failed: {}. I/O stats will be zero.",
                                        e
                                    );
                                    monitor.io_helper_unavailable = true;
                                }
                            }
                        }
                    }
                }

                // Collect and write per-process metrics
                for (pid, output_dir) in &processes {
                    match monitor.collect_metrics(*pid) {
                        Ok(metrics) => {
                            match monitor.csv_state.write_csv(output_dir, &metrics) {
                                Ok(_) => {
                                    // Success - no logging to avoid noise
                                }
                                Err(e) => {
                                    warn!(
                                        "Failed to write metrics for PID {} ({}): {}",
                                        pid,
                                        output_dir.display(),
                                        e
                                    );
                                }
                            }
                        }
                        Err(e) => {
                            debug!(
                                "Failed to collect metrics for PID {} ({}): {}",
                                pid,
                                output_dir.display(),
                                e
                            );
                        }
                    }
                }

                // Collect and write system-wide statistics
                match system_stats::collect_system_stats(
                    &mut monitor.system,
                    &mut monitor.networks,
                    &mut monitor.previous_system_sample,
                ) {
                    Ok(stats) => {
                        match monitor.csv_state.write_system_csv(&log_dir, &stats) {
                            Ok(_) => {
                                // Success - no logging to avoid noise
                            }
                            Err(e) => {
                                warn!("Failed to write system stats: {}", e);
                            }
                        }
                        // Broadcast to SSE clients
                        if let Some(ref broadcaster) = metrics_broadcaster {
                            let snapshot = crate::web::SystemStatsSnapshot::from(&stats);
                            broadcaster.broadcast(snapshot).await;
                        }
                    }
                    Err(e) => {
                        debug!("Failed to collect system stats: {}", e);
                    }
                }
            }
        }
    }

    debug!("Monitoring task shutting down");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_process_state_as_str() {
        assert_eq!(ProcessState::Running.as_str(), "Running");
        assert_eq!(ProcessState::Sleeping.as_str(), "Sleeping");
        assert_eq!(ProcessState::Zombie.as_str(), "Zombie");
    }
}
