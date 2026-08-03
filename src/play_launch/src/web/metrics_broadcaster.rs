//! Broadcaster for streaming system metrics to SSE clients.
//!
//! Uses `tokio::sync::broadcast` — same non-blocking pattern as
//! `StateEventBroadcaster`.

use serde::Serialize;
use tokio::sync::broadcast;
#[cfg(test)]
use ts_rs::TS;

use crate::monitoring::system_stats::SystemStats;

/// Shared broadcast buffer capacity (metrics arrive every 2s — 32 ≈ 1 min).
const CHANNEL_CAPACITY: usize = 32;

/// Subset of SystemStats fields for SSE streaming.
///
/// Exported to TypeScript like every other type crossing the web boundary:
/// `store.js` referenced `types.SystemStatsSnapshot`, which did not exist,
/// so the frontend's view of this payload was unchecked.
#[derive(Debug, Clone, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
pub struct SystemStatsSnapshot {
    pub cpu_percent: f64,
    pub cpu_count: usize,
    // ts-rs maps u64 to `bigint`, but serde_json writes a JSON number and
    // `JSON.parse` yields a `number` — a binding that says `bigint` describes
    // a wire format we do not emit, and made arithmetic on these fields a
    // type error in the frontend.
    #[cfg_attr(test, ts(type = "number"))]
    pub memory_used_bytes: u64,
    #[cfg_attr(test, ts(type = "number"))]
    pub memory_total_bytes: u64,
    pub network_rx_rate_bps: Option<f64>,
    pub network_tx_rate_bps: Option<f64>,
    pub disk_read_rate_bps: Option<f64>,
    pub disk_write_rate_bps: Option<f64>,
    pub gpu_utilization_percent: Option<f64>,
    #[cfg_attr(test, ts(type = "number | null"))]
    pub gpu_memory_used_bytes: Option<u64>,
    #[cfg_attr(test, ts(type = "number | null"))]
    pub gpu_memory_total_bytes: Option<u64>,
}

impl From<&SystemStats> for SystemStatsSnapshot {
    fn from(stats: &SystemStats) -> Self {
        Self {
            cpu_percent: stats.cpu_percent,
            cpu_count: stats.cpu_count,
            memory_used_bytes: stats.used_memory_bytes,
            memory_total_bytes: stats.total_memory_bytes,
            network_rx_rate_bps: stats.network_rx_rate_bps,
            network_tx_rate_bps: stats.network_tx_rate_bps,
            disk_read_rate_bps: stats.disk_read_rate_bps,
            disk_write_rate_bps: stats.disk_write_rate_bps,
            gpu_utilization_percent: stats.gpu_utilization_percent,
            gpu_memory_used_bytes: stats.gpu_memory_used_bytes,
            gpu_memory_total_bytes: stats.gpu_memory_total_bytes,
        }
    }
}

/// Broadcaster for system metrics to SSE clients.
pub struct SystemMetricsBroadcaster {
    tx: broadcast::Sender<SystemStatsSnapshot>,
}

impl SystemMetricsBroadcaster {
    pub fn new() -> Self {
        let (tx, _) = broadcast::channel(CHANNEL_CAPACITY);
        Self { tx }
    }

    /// Subscribe to system metrics (for SSE connection).
    pub fn subscribe(&self) -> broadcast::Receiver<SystemStatsSnapshot> {
        self.tx.subscribe()
    }

    /// Broadcast a snapshot to all subscribers.
    pub fn broadcast(&self, snapshot: SystemStatsSnapshot) {
        let _ = self.tx.send(snapshot);
    }

    /// Get current subscriber count.
    pub fn subscriber_count(&self) -> usize {
        self.tx.receiver_count()
    }
}

impl Default for SystemMetricsBroadcaster {
    fn default() -> Self {
        Self::new()
    }
}
