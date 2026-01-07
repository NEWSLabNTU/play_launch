use csv::Writer;
use eyre::{Result, WrapErr};
use nvml_wrapper::{
    enum_wrappers::device::{Clock, TemperatureSensor},
    Nvml,
};
use std::{
    collections::HashMap,
    fs::{self, File},
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
    time::{Duration, SystemTime},
};
use sysinfo::{Networks, Pid, System};
use tokio::sync::watch;
use tracing::{debug, warn};

/// GPU metrics tuple: (memory_bytes, gpu_util%, mem_util%, temp_celsius, power_mw, graphics_clock_mhz, memory_clock_mhz)
type GpuMetricsTuple = (
    Option<u64>,
    Option<u32>,
    Option<u32>,
    Option<u32>,
    Option<u32>,
    Option<u32>,
    Option<u32>,
);

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
    fn as_str(&self) -> &'static str {
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

/// System-wide resource metrics at a point in time
#[derive(Debug, Clone)]
pub struct SystemStats {
    pub timestamp: SystemTime,

    // CPU metrics
    pub cpu_percent: f64, // Global CPU usage (all cores avg)
    pub cpu_count: usize, // Number of CPU cores

    // Memory metrics
    pub total_memory_bytes: u64,
    pub used_memory_bytes: u64,
    pub available_memory_bytes: u64,
    pub total_swap_bytes: u64,
    pub used_swap_bytes: u64,

    // Network metrics (aggregate all interfaces)
    pub network_rx_bytes: u64,            // Total received (cumulative)
    pub network_tx_bytes: u64,            // Total transmitted (cumulative)
    pub network_rx_rate_bps: Option<f64>, // Receive rate (bytes/sec)
    pub network_tx_rate_bps: Option<f64>, // Transmit rate (bytes/sec)

    // Disk I/O metrics (system-wide from /proc/diskstats)
    pub disk_read_bytes: u64,             // Cumulative
    pub disk_write_bytes: u64,            // Cumulative
    pub disk_read_rate_bps: Option<f64>,  // Bytes/sec
    pub disk_write_rate_bps: Option<f64>, // Bytes/sec

    // Jetson GPU metrics (from jtop)
    pub gpu_utilization_percent: Option<f64>,
    pub gpu_memory_used_bytes: Option<u64>,
    pub gpu_memory_total_bytes: Option<u64>,
    pub gpu_frequency_mhz: Option<u32>,
    pub gpu_power_milliwatts: Option<u32>,
    pub gpu_temperature_celsius: Option<i32>,
}

/// Previous system sample for rate calculation
#[derive(Debug, Clone)]
struct PreviousSystemSample {
    timestamp: SystemTime,
    network_rx_bytes: u64,
    network_tx_bytes: u64,
    disk_read_bytes: u64,
    disk_write_bytes: u64,
}

/// Resource monitor with sysinfo and NVML integration
pub struct ResourceMonitor {
    system: System,
    networks: Networks,                      // Network interface monitor
    csv_writers: HashMap<u32, Writer<File>>, // PID -> CSV writer
    system_csv_writer: Option<Writer<File>>, // System-wide stats CSV writer
    nvml: Option<Nvml>,
    gpu_device_count: u32,
    previous_samples: HashMap<u32, PreviousSample>, // PID -> previous I/O sample
    previous_system_sample: Option<PreviousSystemSample>, // Previous system sample for rate calculation

    // I/O helper daemon for reading /proc/[pid]/io with CAP_SYS_PTRACE
    io_helper: Option<super::io_helper_client::IoHelperClient>,
    io_helper_unavailable: bool, // Track if helper failed to spawn (warn once)
    io_stats_cache: std::collections::HashMap<u32, play_launch::ipc::ProcIoStats>, // Cache for batch I/O reads
}

/// Count network connections from /proc/<pid>/net/{tcp,udp} files
/// Each line (except header) represents a connection
#[cfg(target_os = "linux")]
fn count_connections_in_file(path: &str) -> u32 {
    match std::fs::read_to_string(path) {
        Ok(content) => {
            // Count lines minus 1 for header
            let line_count = content.lines().count();
            if line_count > 0 {
                (line_count - 1) as u32
            } else {
                0
            }
        }
        Err(_) => 0, // File may not exist if no connections
    }
}

/// Find all subprocess PIDs recursively (Linux only)
#[cfg(target_os = "linux")]
fn find_subprocess_pids(parent_pid: u32) -> Vec<u32> {
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
fn find_subprocess_pids(_parent_pid: u32) -> Vec<u32> {
    Vec::new()
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
            csv_writers: HashMap::new(),
            system_csv_writer: None,
            nvml,
            gpu_device_count,
            previous_samples: HashMap::new(),
            previous_system_sample: None,
            io_helper: None,
            io_helper_unavailable: true, // Will be set to false if helper spawns successfully
            io_stats_cache: HashMap::new(),
        })
    }

    fn collect_metrics(&mut self, pid: u32) -> Result<ResourceMetrics> {
        let pid_obj = Pid::from_u32(pid);

        // Get process from system
        let process = self
            .system
            .process(pid_obj)
            .ok_or_else(|| eyre::eyre!("Process {} not found", pid))?;

        // Discover subprocesses for aggregation
        let subprocess_pids = find_subprocess_pids(pid);

        // Parse /proc/[pid]/stat for accurate CPU times (in clock ticks)
        // Aggregate from parent + all children
        let (mut utime, mut stime) = self.parse_proc_stat(pid)?;
        let mut rss_bytes = process.memory();
        let mut vms_bytes = process.virtual_memory();
        let disk_usage = process.disk_usage();
        let mut io_read_bytes = disk_usage.total_read_bytes;
        let mut io_write_bytes = disk_usage.total_written_bytes;
        let mut num_threads = process.tasks().map(|t| t.len() as u32).unwrap_or(1);

        // Aggregate from subprocesses
        for child_pid in &subprocess_pids {
            // Get CPU times from /proc/[child_pid]/stat
            if let Ok((child_utime, child_stime)) = self.parse_proc_stat(*child_pid) {
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
        ) = self.collect_gpu_metrics(pid)?;

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
                // CLK_TCK on Linux is typically 100 (confirmed on this system via `getconf CLK_TCK`)
                // Formula: ((delta_utime + delta_stime) / CLK_TCK / delta_wall_time) * 100
                const CLK_TCK: f64 = 100.0;
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
        const CLK_TCK: f64 = 100.0;
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

    /// Collect system-wide resource metrics (CPU, memory, network, disk I/O, GPU)
    /// This method collects overall system statistics rather than per-process metrics.
    fn collect_system_stats(&mut self) -> Result<SystemStats> {
        let current_time = SystemTime::now();

        // Refresh system-wide information
        self.system.refresh_memory();
        self.system.refresh_cpu_all();
        self.networks.refresh();

        // Collect CPU metrics
        let cpu_percent = self.system.global_cpu_usage() as f64;
        let cpu_count = self.system.cpus().len();

        // Collect memory metrics
        let total_memory_bytes = self.system.total_memory();
        let used_memory_bytes = self.system.used_memory();
        let available_memory_bytes = self.system.available_memory();
        let total_swap_bytes = self.system.total_swap();
        let used_swap_bytes = self.system.used_swap();

        // Collect network metrics (aggregate all interfaces)
        let mut network_rx_bytes: u64 = 0;
        let mut network_tx_bytes: u64 = 0;
        for (_, network) in &self.networks {
            network_rx_bytes += network.total_received();
            network_tx_bytes += network.total_transmitted();
        }

        // Calculate network rates from previous sample
        let (network_rx_rate_bps, network_tx_rate_bps) =
            if let Some(ref prev) = self.previous_system_sample {
                let time_diff = current_time
                    .duration_since(prev.timestamp)
                    .unwrap_or(Duration::from_secs(0))
                    .as_secs_f64();

                if time_diff > 0.0 {
                    let rx_rate =
                        (network_rx_bytes.saturating_sub(prev.network_rx_bytes)) as f64 / time_diff;
                    let tx_rate =
                        (network_tx_bytes.saturating_sub(prev.network_tx_bytes)) as f64 / time_diff;
                    (Some(rx_rate), Some(tx_rate))
                } else {
                    (None, None)
                }
            } else {
                // No previous sample, can't calculate rate
                (None, None)
            };

        // Parse /proc/diskstats for disk I/O
        let (disk_read_bytes, disk_write_bytes) = self.parse_diskstats().unwrap_or((0, 0));

        // Calculate disk I/O rates from previous sample
        let (disk_read_rate_bps, disk_write_rate_bps) =
            if let Some(ref prev) = self.previous_system_sample {
                let time_diff = current_time
                    .duration_since(prev.timestamp)
                    .unwrap_or(Duration::from_secs(0))
                    .as_secs_f64();

                if time_diff > 0.0 {
                    let read_rate =
                        (disk_read_bytes.saturating_sub(prev.disk_read_bytes)) as f64 / time_diff;
                    let write_rate =
                        (disk_write_bytes.saturating_sub(prev.disk_write_bytes)) as f64 / time_diff;
                    (Some(read_rate), Some(write_rate))
                } else {
                    (None, None)
                }
            } else {
                // No previous sample, can't calculate rate
                (None, None)
            };

        // Collect GPU metrics (will be implemented via jtop in later task)
        let gpu_utilization_percent = None;
        let gpu_memory_used_bytes = None;
        let gpu_memory_total_bytes = None;
        let gpu_frequency_mhz = None;
        let gpu_power_milliwatts = None;
        let gpu_temperature_celsius = None;

        // Store current sample for next iteration
        self.previous_system_sample = Some(PreviousSystemSample {
            timestamp: current_time,
            network_rx_bytes,
            network_tx_bytes,
            disk_read_bytes,
            disk_write_bytes,
        });

        Ok(SystemStats {
            timestamp: current_time,
            cpu_percent,
            cpu_count,
            total_memory_bytes,
            used_memory_bytes,
            available_memory_bytes,
            total_swap_bytes,
            used_swap_bytes,
            network_rx_bytes,
            network_tx_bytes,
            network_rx_rate_bps,
            network_tx_rate_bps,
            disk_read_bytes,
            disk_write_bytes,
            disk_read_rate_bps,
            disk_write_rate_bps,
            gpu_utilization_percent,
            gpu_memory_used_bytes,
            gpu_memory_total_bytes,
            gpu_frequency_mhz,
            gpu_power_milliwatts,
            gpu_temperature_celsius,
        })
    }

    fn collect_gpu_metrics(&self, pid: u32) -> Result<GpuMetricsTuple> {
        let nvml = match &self.nvml {
            Some(nvml) => nvml,
            None => {
                // No NVML, return all None
                return Ok((None, None, None, None, None, None, None));
            }
        };

        // Search all GPU devices for this process
        for device_index in 0..self.gpu_device_count {
            // Get GPU device - if this fails, log warning and skip this GPU
            let device = match nvml.device_by_index(device_index) {
                Ok(dev) => dev,
                Err(e) => {
                    // Only warn once per monitoring session (avoid log spam)
                    debug!(
                        "Failed to get GPU device {} for PID {}: {}",
                        device_index, pid, e
                    );
                    continue;
                }
            };

            // Get running compute processes - if this fails, log warning and skip this GPU
            let processes = match device.running_compute_processes() {
                Ok(procs) => procs,
                Err(e) => {
                    // This can fail on some GPU architectures or driver configurations
                    // Log as debug to avoid spamming logs
                    debug!(
                        "Failed to get running processes for GPU {} (PID {}): {}",
                        device_index, pid, e
                    );
                    continue;
                }
            };

            // Check if our PID is using this GPU
            for proc in processes {
                if proc.pid == pid {
                    // Found process on this GPU - collect metrics
                    let utilization = device.utilization_rates().ok();
                    let temperature = device.temperature(TemperatureSensor::Gpu).ok();
                    let power = device.power_usage().ok();
                    let graphics_clock = device.clock_info(Clock::Graphics).ok();
                    let memory_clock = device.clock_info(Clock::Memory).ok();

                    // Extract GPU memory as u64 from UsedGpuMemory enum
                    let gpu_mem_bytes = match proc.used_gpu_memory {
                        nvml_wrapper::enums::device::UsedGpuMemory::Used(bytes) => Some(bytes),
                        nvml_wrapper::enums::device::UsedGpuMemory::Unavailable => None,
                    };

                    return Ok((
                        gpu_mem_bytes,
                        utilization.as_ref().map(|u| u.gpu),
                        utilization.as_ref().map(|u| u.memory),
                        temperature,
                        power,
                        graphics_clock,
                        memory_clock,
                    ));
                }
            }
        }

        // Process not found on any GPU (this is normal for most processes)
        Ok((None, None, None, None, None, None, None))
    }

    /// Count TCP and UDP connections for a process (Linux-specific)
    #[cfg(target_os = "linux")]
    fn collect_network_connections(&self, pid: u32) -> (u32, u32) {
        let tcp_count = count_connections_in_file(&format!("/proc/{}/net/tcp", pid))
            + count_connections_in_file(&format!("/proc/{}/net/tcp6", pid));
        let udp_count = count_connections_in_file(&format!("/proc/{}/net/udp", pid))
            + count_connections_in_file(&format!("/proc/{}/net/udp6", pid));

        (tcp_count, udp_count)
    }

    #[cfg(not(target_os = "linux"))]
    fn collect_network_connections(&self, _pid: u32) -> (u32, u32) {
        (0, 0)
    }

    /// Parse /proc/[pid]/stat for CPU times (utime and stime)
    /// Returns (utime, stime) in clock ticks
    /// These values are cumulative and need to be differenced between samples
    #[cfg(target_os = "linux")]
    fn parse_proc_stat(&self, pid: u32) -> Result<(u64, u64)> {
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
    fn parse_proc_stat(&self, _pid: u32) -> Result<(u64, u64)> {
        // CPU time parsing not implemented for non-Linux platforms
        Ok((0, 0))
    }

    /// Parse /proc/diskstats for system-wide disk I/O statistics
    /// Returns (total_read_bytes, total_write_bytes) aggregated across all disks
    /// Format: major minor name reads ... sectors_read ... writes ... sectors_written ...
    /// Sectors are typically 512 bytes
    #[cfg(target_os = "linux")]
    fn parse_diskstats(&self) -> Result<(u64, u64)> {
        let content = std::fs::read_to_string("/proc/diskstats")
            .wrap_err("Failed to read /proc/diskstats")?;

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
    fn parse_diskstats(&self) -> Result<(u64, u64)> {
        Ok((0, 0))
    }

    fn write_csv(&mut self, output_dir: &Path, metrics: &ResourceMetrics) -> Result<()> {
        let pid = metrics.pid;

        // Get or create CSV writer for this PID
        if !self.csv_writers.contains_key(&pid) {
            let csv_path = self.get_csv_path(output_dir)?;

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
                ])
                .wrap_err("Failed to write CSV header")?;

            writer.flush().wrap_err("Failed to flush CSV header")?;
            self.csv_writers.insert(pid, writer);
        }

        let writer = self
            .csv_writers
            .get_mut(&pid)
            .ok_or_else(|| eyre::eyre!("CSV writer not found for PID {}", pid))?;

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

    /// Write system-wide statistics to CSV file
    /// Creates system_stats.csv at the log directory root level
    fn write_system_csv(&mut self, log_dir: &Path, stats: &SystemStats) -> Result<()> {
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
                .write_record([
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
                ])
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

    fn get_csv_path(&self, output_dir: &Path) -> Result<PathBuf> {
        // Create path: <output_dir>/metrics.csv
        let csv_path = output_dir.join("metrics.csv");
        Ok(csv_path)
    }
}

/// Initialize CSV file with headers for a given output directory
/// This should be called as soon as monitoring is enabled to ensure CSV files
/// exist even if the node dies before metrics are collected
pub fn initialize_metrics_csv(output_dir: &Path) -> Result<()> {
    let csv_path = output_dir.join("metrics.csv");

    // Create parent directories if needed
    if let Some(parent) = csv_path.parent() {
        fs::create_dir_all(parent)
            .wrap_err_with(|| format!("Failed to create directory: {}", parent.display()))?;
    }

    let file = File::create(&csv_path)
        .wrap_err_with(|| format!("Failed to create CSV file: {}", csv_path.display()))?;
    let mut writer = Writer::from_writer(file);

    // Write header
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

/// Async monitoring task (tokio-based replacement for old thread-based monitoring)
pub async fn run_monitoring_task(
    config: MonitorConfig,
    log_dir: PathBuf,
    process_registry: Arc<Mutex<HashMap<u32, PathBuf>>>,
    nvml: Option<Nvml>,
    mut shutdown_rx: watch::Receiver<bool>,
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

                    // CRITICAL: Refresh global CPU times first!
                    monitor.system.refresh_cpu_usage();

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

                // Batch read I/O stats for all PIDs
                monitor.io_stats_cache.clear();
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
                for (pid, output_dir) in processes {
                    match monitor.collect_metrics(pid) {
                        Ok(metrics) => {
                            match monitor.write_csv(&output_dir, &metrics) {
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
                match monitor.collect_system_stats() {
                    Ok(stats) => {
                        match monitor.write_system_csv(&log_dir, &stats) {
                            Ok(_) => {
                                // Success - no logging to avoid noise
                            }
                            Err(e) => {
                                warn!("Failed to write system stats: {}", e);
                            }
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

/// Format SystemTime as ISO 8601 string
fn format_timestamp(time: SystemTime) -> String {
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

    #[test]
    fn test_format_timestamp() {
        let time = SystemTime::UNIX_EPOCH + Duration::from_secs(1234567890);
        let formatted = format_timestamp(time);
        assert!(formatted.starts_with("2009-02-13"));
    }

    #[test]
    fn test_process_state_as_str() {
        assert_eq!(ProcessState::Running.as_str(), "Running");
        assert_eq!(ProcessState::Sleeping.as_str(), "Sleeping");
        assert_eq!(ProcessState::Zombie.as_str(), "Zombie");
    }
}
