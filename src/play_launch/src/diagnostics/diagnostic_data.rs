use dashmap::DashMap;
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, sync::Arc, time::SystemTime};
use tokio::sync::Mutex;

/// Diagnostic level matching ROS2 diagnostic_msgs/DiagnosticStatus
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum DiagnosticLevel {
    Ok = 0,
    Warning = 1,
    Error = 2,
    Stale = 3,
}

impl DiagnosticLevel {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => DiagnosticLevel::Ok,
            1 => DiagnosticLevel::Warning,
            2 => DiagnosticLevel::Error,
            3 => DiagnosticLevel::Stale,
            _ => DiagnosticLevel::Stale, // Default to STALE for unknown values
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            DiagnosticLevel::Ok => "OK",
            DiagnosticLevel::Warning => "WARNING",
            DiagnosticLevel::Error => "ERROR",
            DiagnosticLevel::Stale => "STALE",
        }
    }
}

/// A single diagnostic status message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticStatus {
    pub hardware_id: String,
    pub name: String,
    pub level: DiagnosticLevel,
    pub message: String,
    pub values: HashMap<String, String>,
    #[serde(with = "humantime_serde")]
    pub timestamp: SystemTime,
}

impl DiagnosticStatus {
    pub fn key(&self) -> String {
        format!("{}/{}", self.hardware_id, self.name)
    }
}

/// Diagnostic counts by level
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct DiagnosticCounts {
    pub ok: usize,
    pub warning: usize,
    pub error: usize,
    pub stale: usize,
    pub total: usize,
}

/// Registry for storing and accessing diagnostic data
#[derive(Clone)]
pub struct DiagnosticRegistry {
    // Latest status for each diagnostic (keyed by "hardware_id/name")
    diagnostics: Arc<DashMap<String, DiagnosticStatus>>,
    // Historical log of all updates
    history: Arc<Mutex<Vec<DiagnosticStatus>>>,
}

impl DiagnosticRegistry {
    pub fn new() -> Self {
        Self {
            diagnostics: Arc::new(DashMap::new()),
            history: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Update or insert a diagnostic status
    pub async fn update(&self, status: DiagnosticStatus) {
        let key = status.key();

        // Update latest status
        self.diagnostics.insert(key, status.clone());

        // Append to history
        let mut history = self.history.lock().await;
        history.push(status);
    }

    /// Get all current diagnostics (latest status for each)
    pub fn list_all(&self) -> Vec<DiagnosticStatus> {
        self.diagnostics
            .iter()
            .map(|entry| entry.value().clone())
            .collect()
    }

    /// Get diagnostic counts by level
    pub fn get_counts(&self) -> DiagnosticCounts {
        let mut counts = DiagnosticCounts::default();

        for entry in self.diagnostics.iter() {
            counts.total += 1;
            match entry.value().level {
                DiagnosticLevel::Ok => counts.ok += 1,
                DiagnosticLevel::Warning => counts.warning += 1,
                DiagnosticLevel::Error => counts.error += 1,
                DiagnosticLevel::Stale => counts.stale += 1,
            }
        }

        counts
    }

    /// Get full history (for CSV export)
    #[allow(dead_code)]
    pub async fn get_history(&self) -> Vec<DiagnosticStatus> {
        let history = self.history.lock().await;
        history.clone()
    }
}

impl Default for DiagnosticRegistry {
    fn default() -> Self {
        Self::new()
    }
}
