//! Broadcaster for streaming system metrics to SSE clients.

use serde::Serialize;
use std::sync::Arc;
use tokio::sync::{mpsc, Mutex as TokioMutex};

use crate::monitoring::system_stats::SystemStats;

/// Channel capacity per SSE subscriber (small — metrics arrive every 2s)
const METRICS_SUBSCRIBER_CHANNEL_SIZE: usize = 16;

/// Subset of SystemStats fields for SSE streaming.
#[derive(Debug, Clone, Serialize)]
pub struct SystemStatsSnapshot {
    pub cpu_percent: f64,
    pub cpu_count: usize,
    pub memory_used_bytes: u64,
    pub memory_total_bytes: u64,
    pub network_rx_rate_bps: Option<f64>,
    pub network_tx_rate_bps: Option<f64>,
    pub disk_read_rate_bps: Option<f64>,
    pub disk_write_rate_bps: Option<f64>,
    pub gpu_utilization_percent: Option<f64>,
    pub gpu_memory_used_bytes: Option<u64>,
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
    subscribers: Arc<TokioMutex<Vec<mpsc::Sender<SystemStatsSnapshot>>>>,
}

impl SystemMetricsBroadcaster {
    pub fn new() -> Self {
        Self {
            subscribers: Arc::new(TokioMutex::new(Vec::new())),
        }
    }

    /// Subscribe to system metrics (for SSE connection).
    pub async fn subscribe(&self) -> mpsc::Receiver<SystemStatsSnapshot> {
        let (tx, rx) = mpsc::channel(METRICS_SUBSCRIBER_CHANNEL_SIZE);
        let mut subs = self.subscribers.lock().await;
        subs.push(tx);
        tracing::debug!("New metrics SSE subscriber added (total: {})", subs.len());
        rx
    }

    /// Broadcast a snapshot to all subscribers.
    ///
    /// Automatically removes disconnected subscribers. Skips entirely when
    /// there are no subscribers (zero overhead in the common case).
    pub async fn broadcast(&self, snapshot: SystemStatsSnapshot) {
        let mut subs = self.subscribers.lock().await;
        if subs.is_empty() {
            return;
        }

        // Remove disconnected subscribers
        let initial_count = subs.len();
        subs.retain(|tx| !tx.is_closed());
        let removed = initial_count - subs.len();
        if removed > 0 {
            tracing::debug!("Removed {} disconnected metrics SSE subscribers", removed);
        }

        // Send to all active subscribers
        for tx in subs.iter() {
            let _ = tx.send(snapshot.clone()).await;
        }
    }
}

impl Default for SystemMetricsBroadcaster {
    fn default() -> Self {
        Self::new()
    }
}
