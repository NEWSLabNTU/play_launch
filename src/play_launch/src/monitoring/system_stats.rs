//! System-wide resource statistics and GPU metrics collection.

use eyre::Result;
use nvml_wrapper::{
    enum_wrappers::device::{Clock, TemperatureSensor},
    Nvml,
};
use std::time::{Duration, SystemTime};
use sysinfo::{Networks, System};
use tracing::debug;

use super::proc_parser;

/// GPU metrics tuple: (memory_bytes, gpu_util%, mem_util%, temp_celsius, power_mw, graphics_clock_mhz, memory_clock_mhz)
pub type GpuMetricsTuple = (
    Option<u64>,
    Option<u32>,
    Option<u32>,
    Option<u32>,
    Option<u32>,
    Option<u32>,
    Option<u32>,
);

/// System-wide resource metrics at a point in time.
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

/// Previous system sample for rate calculation.
#[derive(Debug, Clone)]
pub struct PreviousSystemSample {
    pub timestamp: SystemTime,
    pub network_rx_bytes: u64,
    pub network_tx_bytes: u64,
    pub disk_read_bytes: u64,
    pub disk_write_bytes: u64,
}

/// Collect system-wide resource metrics (CPU, memory, network, disk I/O, GPU).
///
/// Mutably borrows `system` and `networks` for refresh, reads and updates
/// `previous_system_sample` for rate calculation.
pub fn collect_system_stats(
    system: &mut System,
    networks: &mut Networks,
    previous_system_sample: &mut Option<PreviousSystemSample>,
) -> Result<SystemStats> {
    let current_time = SystemTime::now();

    // Refresh system-wide information
    // Note: refresh_cpu_all() is already called at the start of the tick
    // (before process refresh), so we only need memory and network here.
    system.refresh_memory();
    networks.refresh();

    // Collect CPU metrics
    let cpu_percent = system.global_cpu_usage() as f64;
    let cpu_count = system.cpus().len();

    // Collect memory metrics
    let total_memory_bytes = system.total_memory();
    let used_memory_bytes = system.used_memory();
    let available_memory_bytes = system.available_memory();
    let total_swap_bytes = system.total_swap();
    let used_swap_bytes = system.used_swap();

    // Collect network metrics (aggregate all interfaces)
    let mut network_rx_bytes: u64 = 0;
    let mut network_tx_bytes: u64 = 0;
    for (_, network) in networks.iter() {
        network_rx_bytes += network.total_received();
        network_tx_bytes += network.total_transmitted();
    }

    // Calculate network rates from previous sample
    let (network_rx_rate_bps, network_tx_rate_bps) = if let Some(ref prev) = *previous_system_sample
    {
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
    let (disk_read_bytes, disk_write_bytes) = proc_parser::parse_diskstats().unwrap_or((0, 0));

    // Calculate disk I/O rates from previous sample
    let (disk_read_rate_bps, disk_write_rate_bps) = if let Some(ref prev) = *previous_system_sample
    {
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
    *previous_system_sample = Some(PreviousSystemSample {
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

/// Collect GPU metrics for a specific process using NVML.
/// Searches all GPU devices for the given PID and returns metrics if found.
pub fn collect_gpu_metrics(
    nvml: Option<&Nvml>,
    gpu_device_count: u32,
    pid: u32,
) -> Result<GpuMetricsTuple> {
    let nvml = match nvml {
        Some(nvml) => nvml,
        None => {
            // No NVML, return all None
            return Ok((None, None, None, None, None, None, None));
        }
    };

    // Search all GPU devices for this process
    for device_index in 0..gpu_device_count {
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
