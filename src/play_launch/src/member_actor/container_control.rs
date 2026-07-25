//! Container control events for LoadNode management
//!
//! This module defines the event types for communication between composable node
//! actors and container actors. Composable nodes send LoadNode requests to their
//! container actor, which dispatches them concurrently.

use eyre::Result;
use std::{path::PathBuf, time::Instant};
use tokio::sync::oneshot;

/// Typed failure of the LoadNode path (phase-52.4, issue 0005) — callers
/// can distinguish a busy container from a hard rejection instead of
/// pattern-matching eyre strings. `Display` prefixes the kind, so the
/// web UI's Failed error text carries it too.
#[derive(Debug, Clone, thiserror::Error)]
pub enum LoadError {
    /// No LoadNode client — the container never started properly.
    #[error("service-missing: no LoadNode service client for container {container}")]
    ServiceMissing { container: String },
    /// The LoadNode service never appeared within the timeout.
    #[error("service-not-ready: LoadNode service not available after {secs}s")]
    ServiceNotReady { secs: u64 },
    /// The service call itself errored (transport/serialization).
    #[error("call-failed: {message}")]
    CallFailed { message: String },
    /// Every attempt timed out and ListNodes proved the component absent.
    #[error("timeout: LoadNode timed out after {attempts} attempts (component never appeared)")]
    TimedOut { attempts: usize },
    /// The container stayed busy past the total budget — no resend
    /// (LoadNode is not idempotent; a resend would double-load).
    #[error("container-busy: unresponsive for {budget_secs}s while loading {composable}")]
    Unresponsive {
        budget_secs: u64,
        composable: String,
    },
    /// Request construction failed (parameter conversion etc.).
    #[error("bad-request: {message}")]
    BadRequest { message: String },
}

/// Response from loading a composable node
#[derive(Clone, Debug, PartialEq)]
pub struct LoadNodeResponse {
    pub success: bool,
    pub error_message: String,
    pub full_node_name: String,
    pub unique_id: u64,
    /// Timing metrics for load operation
    pub timing: LoadTimingMetrics,
}

/// Response from unloading a composable node
#[derive(Clone, Debug, PartialEq)]
pub struct UnloadNodeResponse {
    pub success: bool,
    pub error_message: String,
}

/// Timing metrics for LoadNode operation
#[derive(Clone, Debug, PartialEq)]
pub struct LoadTimingMetrics {
    /// Time spent waiting in queue (ms)
    pub queue_wait_ms: u64,
    /// Time spent in ROS service call (ms)
    pub service_call_ms: u64,
    /// Total duration from request to completion (ms)
    pub total_duration_ms: u64,
}

/// Control events that can be sent to a container actor
#[derive(Debug)]
#[allow(dead_code)] // Variants constructed via channel messages from other modules
pub enum ContainerControlEvent {
    /// Request to load a composable node into this container
    LoadNode {
        /// Name of the composable node (for logging)
        composable_name: String,
        /// ROS package containing the plugin
        package: String,
        /// Plugin class name
        plugin: String,
        /// Node name
        node_name: String,
        /// Node namespace
        node_namespace: String,
        /// Remap rules
        remap_rules: Vec<String>,
        /// Parameters (key-value pairs)
        parameters: Vec<(String, String)>,
        /// Extra arguments (key-value pairs)
        extra_args: Vec<(String, String)>,
        /// Output directory for per-node logging (isolated mode)
        output_dir: PathBuf,
    },
    /// Request to unload a composable node from this container
    UnloadNode {
        /// Name of the composable node (for logging)
        composable_name: String,
        /// Unique ID from LoadNode response
        unique_id: u64,
        /// Channel to send the response
        response_tx: oneshot::Sender<Result<UnloadNodeResponse>>,
    },
}

/// Internal load request (queued in container actor)
#[derive(Debug)]
pub(super) struct LoadRequest {
    /// Name of the composable node (for logging)
    pub composable_name: String,
    /// ROS package containing the plugin
    pub package: String,
    /// Plugin class name
    pub plugin: String,
    /// Node name
    pub node_name: String,
    /// Node namespace
    pub node_namespace: String,
    /// Remap rules
    pub remap_rules: Vec<String>,
    /// Parameters (key-value pairs)
    pub parameters: Vec<(String, String)>,
    /// Extra arguments (key-value pairs)
    pub extra_args: Vec<(String, String)>,
    /// Output directory for per-node logging (isolated mode)
    pub output_dir: PathBuf,
    /// When the request was received (for metrics)
    pub request_time: Instant,
}

/// Parameters for LoadNode service call (to avoid too many function arguments)
#[derive(Debug, Clone)]
pub(super) struct LoadParams {
    pub composable_name: String,
    pub package: String,
    pub plugin: String,
    pub node_name: String,
    pub node_namespace: String,
    pub remap_rules: Vec<String>,
    pub parameters: Vec<(String, String)>,
    pub extra_args: Vec<(String, String)>,
    pub output_dir: PathBuf,
    pub request_time: Instant,
}

/// Completion of a LoadNode service call (sent from spawned task back to actor)
pub(super) struct LoadCompletion {
    pub composable_name: String,
    pub result: Result<LoadNodeResponse, LoadError>,
}

/// Current unload being processed
pub(super) struct CurrentUnload {
    /// Name of the composable node being unloaded
    pub composable_name: String,
    /// Task handle for the UnloadNode service call
    pub task: tokio::task::JoinHandle<eyre::Result<UnloadNodeResponse>>,
}
