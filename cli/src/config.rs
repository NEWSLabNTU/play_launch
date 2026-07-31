// UNRESOLVED DISPOSITION — what: this whole runtime-config subsystem
// (`RuntimeConfig`, `MonitoringSettings`, `DiagnosticsSettings`,
// `InterceptionSettings`, `ComposableNodeLoadingSettings`,
// `ContainerReadinessSettings`, `ResolvedRuntimeConfig`,
// `load_runtime_config`) — a full mirror of play_launch's own monitoring/
// diagnostics/interception/container-readiness/composable-loading
// settings. Why no caller: `resolve`/`dump`/`contract`/`plot` (the only
// verbs this crate wires up, see `main.rs`) never touch any of those
// runtime concerns — they stayed in `play_launch` per RFC-0060 when this
// crate was split out as the launch-tree-only layer. It compiles cleanly
// on its own (verified: removing this `allow` produces only `dead_code`
// warnings, no unresolved-reference errors), so it is not unreachable
// scaffolding the way `forward_state_events_and_wait` was — it is live,
// working code with nothing calling it yet.
// Separate decision, not made here: whether to wire it into a future
// command, move it back to `play_launch`, or delete it outright. This
// `#[allow(dead_code)]` only keeps the gate green while that decision is
// pending — it is not an endorsement of keeping the module indefinitely.
#![allow(dead_code)]

use eyre::{Result, WrapErr};
use glob::Pattern;
use serde::Deserialize;
use std::path::Path;

/// Runtime configuration for play_launch
#[derive(Debug, Clone, Deserialize, Default)]
pub struct RuntimeConfig {
    /// Resource monitoring settings
    #[serde(default)]
    pub monitoring: MonitoringSettings,

    /// Composable node loading settings
    #[serde(default)]
    pub composable_node_loading: ComposableNodeLoadingSettings,

    /// Container readiness checking settings
    #[serde(default)]
    pub container_readiness: ContainerReadinessSettings,

    /// Diagnostics monitoring settings
    #[serde(default)]
    pub diagnostics: DiagnosticsSettings,

    /// RCL interception settings (LD_PRELOAD-based message introspection)
    #[serde(default)]
    pub interception: InterceptionSettings,

    /// Per-process configurations
    #[serde(default)]
    pub processes: Vec<ProcessConfig>,
}

/// Global monitoring settings
#[derive(Debug, Clone, Deserialize)]
pub struct MonitoringSettings {
    /// Enable monitoring (default: false, overridden by --enable-monitoring flag)
    #[serde(default)]
    pub enabled: bool,

    /// Sampling interval in milliseconds (default: 1000)
    #[serde(default = "default_sample_interval")]
    pub sample_interval_ms: u64,

    /// Monitor all nodes by default (default: true when monitoring is enabled)
    #[serde(default = "default_true")]
    pub monitor_all_nodes: bool,

    /// Only monitor nodes matching these patterns (empty = all nodes)
    #[serde(default)]
    pub monitor_patterns: Vec<String>,
}

impl Default for MonitoringSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            sample_interval_ms: default_sample_interval(),
            monitor_all_nodes: default_true(),
            monitor_patterns: Vec::new(),
        }
    }
}

fn default_sample_interval() -> u64 {
    2000 // Increased from 1000ms to reduce CPU overhead
}
fn default_true() -> bool {
    true
}

/// Diagnostics monitoring settings
#[derive(Debug, Clone, Deserialize)]
pub struct DiagnosticsSettings {
    /// Enable diagnostics monitoring (default: false, overridden by --enable-diagnostics flag)
    #[serde(default)]
    pub enabled: bool,

    /// Topics to subscribe to for diagnostics
    #[serde(default = "default_diagnostics_topics")]
    pub topics: Vec<String>,

    /// Filter diagnostics by hardware_id (empty = all)
    #[serde(default)]
    pub filter_hardware_ids: Vec<String>,

    /// Debounce period in milliseconds (minimum time between logging same diagnostic)
    #[serde(default = "default_debounce_ms")]
    pub debounce_ms: u64,

    /// Store diagnostics per-node (future feature)
    #[serde(default = "default_true")]
    #[allow(dead_code)]
    pub store_per_node: bool,

    /// Store system-wide diagnostics
    #[serde(default = "default_true")]
    #[allow(dead_code)]
    pub store_system_wide: bool,
}

impl Default for DiagnosticsSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            topics: default_diagnostics_topics(),
            filter_hardware_ids: Vec::new(),
            debounce_ms: default_debounce_ms(),
            store_per_node: default_true(),
            store_system_wide: default_true(),
        }
    }
}

fn default_diagnostics_topics() -> Vec<String> {
    vec!["/diagnostics".to_string(), "/diagnostics_agg".to_string()]
}

fn default_debounce_ms() -> u64 {
    100
}

/// RCL interception settings — LD_PRELOAD-based message introspection.
///
/// When enabled, play_launch injects `libplay_launch_interception.so` via
/// LD_PRELOAD into all managed nodes, giving transparent data flow visibility
/// without modifying user code.
#[derive(Debug, Clone, Deserialize)]
pub struct InterceptionSettings {
    /// Enable interception (default: false)
    #[serde(default)]
    pub enabled: bool,

    /// Enable frontier tracking (default: true when interception is enabled)
    #[serde(default = "default_true")]
    pub frontier: bool,

    /// Enable message statistics (default: true when interception is enabled)
    #[serde(default = "default_true")]
    pub stats: bool,

    /// SPSC ring buffer capacity per child process (default: 65536)
    #[serde(default = "default_ring_capacity")]
    pub ring_capacity: usize,
}

impl Default for InterceptionSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            frontier: default_true(),
            stats: default_true(),
            ring_capacity: default_ring_capacity(),
        }
    }
}

fn default_ring_capacity() -> usize {
    65536
}

/// Composable node loading settings (phase-52: wired into the container
/// LoadNode path via `container_actor::timing::LoadTimings`).
///
/// Autoware-scale guidance: composable ctors that block for minutes (e.g.
/// TensorRT engine builds on first run) hold a blocking container's
/// executor — raise `load_total_budget_secs` (and consider
/// `load_retry_timeout_millis`) rather than the base timeout; the loader
/// polls ListNodes during the budget instead of resending (LoadNode is
/// not idempotent).
#[derive(Debug, Clone, Deserialize)]
pub struct ComposableNodeLoadingSettings {
    /// Delay before loading composable nodes (milliseconds)
    #[serde(default = "default_delay_load_node_millis")]
    #[allow(dead_code)]
    pub delay_load_node_millis: u64,

    /// Timeout for the FIRST LoadNode service call per composable (milliseconds)
    #[serde(default = "default_load_node_timeout_millis")]
    pub load_node_timeout_millis: u64,

    /// Maximum LoadNode attempts (first + retries) per composable
    #[serde(default = "default_load_node_attempts")]
    pub load_node_attempts: usize,

    /// Maximum concurrent composable node loading operations
    #[serde(default = "default_max_concurrent_load_node_spawn")]
    #[allow(dead_code)]
    pub max_concurrent_load_node_spawn: usize,

    /// Timeout for retry LoadNode calls (milliseconds) — longer than the
    /// first call because a busy single-threaded container answers late
    #[serde(default = "default_load_retry_timeout_millis")]
    pub load_retry_timeout_millis: u64,

    /// Total per-composable budget while the container is busy/unresponsive
    /// (seconds). The loader polls ListNodes within this budget instead of
    /// resending LoadNode. Raise for multi-minute ctors (TensorRT).
    #[serde(default = "default_load_total_budget_secs")]
    pub load_total_budget_secs: u64,

    /// Pause between ListNodes verification polls while busy (seconds)
    #[serde(default = "default_load_verify_poll_interval_secs")]
    pub load_verify_poll_interval_secs: u64,

    /// How long a composable may sit in Loading with a known unique_id
    /// before being promoted to Loaded (seconds) — covers DDS event loss
    #[serde(default = "default_loading_event_timeout_secs")]
    pub loading_event_timeout_secs: u64,

    /// Warmup delay after the LoadNode service appears before calling it
    /// (milliseconds) — the executor may register the service before it
    /// starts processing requests
    #[serde(default = "default_post_service_ready_warmup_ms")]
    pub post_service_ready_warmup_ms: u64,
}

impl Default for ComposableNodeLoadingSettings {
    fn default() -> Self {
        Self {
            delay_load_node_millis: default_delay_load_node_millis(),
            load_node_timeout_millis: default_load_node_timeout_millis(),
            load_node_attempts: default_load_node_attempts(),
            max_concurrent_load_node_spawn: default_max_concurrent_load_node_spawn(),
            load_retry_timeout_millis: default_load_retry_timeout_millis(),
            load_total_budget_secs: default_load_total_budget_secs(),
            load_verify_poll_interval_secs: default_load_verify_poll_interval_secs(),
            loading_event_timeout_secs: default_loading_event_timeout_secs(),
            post_service_ready_warmup_ms: default_post_service_ready_warmup_ms(),
        }
    }
}

fn default_delay_load_node_millis() -> u64 {
    2000
}
fn default_load_node_timeout_millis() -> u64 {
    30000
}
fn default_load_node_attempts() -> usize {
    3
}
fn default_max_concurrent_load_node_spawn() -> usize {
    10
}
fn default_load_retry_timeout_millis() -> u64 {
    60000
}
fn default_load_total_budget_secs() -> u64 {
    600
}
fn default_load_verify_poll_interval_secs() -> u64 {
    5
}
fn default_loading_event_timeout_secs() -> u64 {
    10
}
fn default_post_service_ready_warmup_ms() -> u64 {
    200
}

/// Container readiness checking settings
#[derive(Debug, Clone, Deserialize)]
pub struct ContainerReadinessSettings {
    /// Wait for container services to be available via ROS service discovery
    /// Default: true (changed from false - service readiness is now default behavior)
    #[serde(default = "default_wait_for_service_ready")]
    pub wait_for_service_ready: bool,

    /// Maximum time to wait for each container service (seconds)
    /// Set to 0 for unlimited wait time
    #[serde(default = "default_service_ready_timeout_secs")]
    pub service_ready_timeout_secs: u64,

    /// Interval for polling container service availability (milliseconds)
    #[serde(default = "default_service_poll_interval_ms")]
    #[allow(dead_code)]
    pub service_poll_interval_ms: u64,
}

impl Default for ContainerReadinessSettings {
    fn default() -> Self {
        Self {
            wait_for_service_ready: default_wait_for_service_ready(),
            service_ready_timeout_secs: default_service_ready_timeout_secs(),
            service_poll_interval_ms: default_service_poll_interval_ms(),
        }
    }
}

fn default_wait_for_service_ready() -> bool {
    true // Changed from false - service readiness is now the default
}
fn default_service_ready_timeout_secs() -> u64 {
    120
}
fn default_service_poll_interval_ms() -> u64 {
    500
}

/// Configuration for individual process control
#[derive(Debug, Clone, Deserialize)]
pub struct ProcessConfig {
    /// Node name or pattern (supports glob patterns)
    #[allow(dead_code)] // Used in spawn_nodes integration (not yet connected)
    pub node_pattern: String,

    /// Enable monitoring for this node (default: inherit from global)
    #[serde(default)]
    #[allow(dead_code)] // Used in spawn_nodes integration (not yet connected)
    pub monitor: Option<bool>,

    /// CPU cores to pin this process to (e.g., [0, 1, 2])
    #[serde(default)]
    pub cpu_affinity: Vec<usize>,

    /// Nice value (-20 to 19, lower = higher priority)
    #[serde(default)]
    pub nice: Option<i32>,
}

#[allow(dead_code)] // Methods used in spawn_nodes integration (not yet connected)
impl ProcessConfig {
    /// Check if this config matches a node name
    pub fn matches(&self, node_name: &str) -> bool {
        Pattern::new(&self.node_pattern)
            .map(|pattern| pattern.matches(node_name))
            .unwrap_or(false)
    }

    /// Check if monitoring is enabled for this node
    pub fn should_monitor(&self, global_enabled: bool) -> bool {
        self.monitor.unwrap_or(global_enabled)
    }
}

/// Resolved runtime configuration ready for use
#[derive(Debug, Clone)]
pub struct ResolvedRuntimeConfig {
    pub monitoring: ResolvedMonitoringConfig,
    pub composable_node_loading: ComposableNodeLoadingSettings,
    pub container_readiness: ContainerReadinessSettings,
    pub diagnostics: DiagnosticsSettings,
    pub interception: InterceptionSettings,
}

/// Resolved monitoring configuration
#[derive(Debug, Clone)]
pub struct ResolvedMonitoringConfig {
    pub enabled: bool,
    pub sample_interval_ms: u64,
    #[allow(dead_code)] // Used in spawn_nodes integration (not yet connected)
    pub monitor_all_nodes: bool,
    #[allow(dead_code)] // Used in spawn_nodes integration (not yet connected)
    pub monitor_patterns: Vec<String>,
    #[allow(dead_code)] // Used in spawn_nodes integration (not yet connected)
    pub process_configs: Vec<ProcessConfig>,
}

#[allow(dead_code)] // Methods used in spawn_nodes integration (not yet connected)
impl ResolvedMonitoringConfig {
    /// Check if a node should be monitored
    pub fn should_monitor(&self, node_name: &str) -> bool {
        if !self.enabled {
            return false;
        }

        // Check process-specific config first
        for config in &self.process_configs {
            if config.matches(node_name)
                && let Some(monitor) = config.monitor
            {
                return monitor;
            }
        }

        // Check monitor_patterns if specified
        if !self.monitor_patterns.is_empty() {
            return self.monitor_patterns.iter().any(|pattern| {
                Pattern::new(pattern)
                    .map(|p| p.matches(node_name))
                    .unwrap_or(false)
            });
        }

        // Default: monitor all nodes if enabled
        self.monitor_all_nodes
    }

    /// Get process config for a node
    pub fn get_process_config(&self, node_name: &str) -> Option<&ProcessConfig> {
        self.process_configs
            .iter()
            .find(|config| config.matches(node_name))
    }
}

/// Load and resolve runtime configuration
pub fn load_runtime_config(
    config_path: Option<&Path>,
    monitoring_enabled: bool,
    monitor_interval_override: Option<u64>,
    diagnostics_enabled: bool,
) -> Result<ResolvedRuntimeConfig> {
    // Load config file or use defaults
    let mut config = if let Some(path) = config_path {
        let content = std::fs::read_to_string(path)
            .wrap_err_with(|| format!("Failed to read config file: {}", path.display()))?;
        serde_yaml_ng::from_str::<RuntimeConfig>(&content)
            .wrap_err_with(|| format!("Failed to parse config file: {}", path.display()))?
    } else {
        RuntimeConfig::default()
    };

    // Enable monitoring by default (enabled unless explicitly disabled via CLI)
    // If config file doesn't specify, use CLI decision
    if config_path.is_none() || !config.monitoring.enabled {
        config.monitoring.enabled = monitoring_enabled;
    }

    // Enable diagnostics by default (enabled unless explicitly disabled via CLI)
    // If config file doesn't specify, use CLI decision
    if config_path.is_none() || !config.diagnostics.enabled {
        config.diagnostics.enabled = diagnostics_enabled;
    }

    // CLI interval overrides config file
    if let Some(interval) = monitor_interval_override {
        config.monitoring.sample_interval_ms = interval;
    }

    // Validate process configs
    for process_config in &config.processes {
        // Validate CPU affinity
        if !process_config.cpu_affinity.is_empty() {
            let num_cpus = num_cpus::get();
            for &cpu in &process_config.cpu_affinity {
                if cpu >= num_cpus {
                    return Err(eyre::eyre!(
                        "Invalid CPU affinity: CPU {} does not exist (max: {})",
                        cpu,
                        num_cpus - 1
                    ));
                }
            }
        }

        // Validate nice value
        if let Some(nice) = process_config.nice
            && !(-20..=19).contains(&nice)
        {
            return Err(eyre::eyre!(
                "Invalid nice value: {} (must be between -20 and 19)",
                nice
            ));
        }
    }

    Ok(ResolvedRuntimeConfig {
        monitoring: ResolvedMonitoringConfig {
            enabled: config.monitoring.enabled,
            sample_interval_ms: config.monitoring.sample_interval_ms,
            monitor_all_nodes: config.monitoring.monitor_all_nodes,
            monitor_patterns: config.monitoring.monitor_patterns,
            process_configs: config.processes,
        },
        composable_node_loading: config.composable_node_loading,
        container_readiness: config.container_readiness,
        diagnostics: config.diagnostics,
        interception: config.interception,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = RuntimeConfig::default();
        assert!(!config.monitoring.enabled);
        assert_eq!(config.monitoring.sample_interval_ms, 2000);
        assert!(config.monitoring.monitor_all_nodes);
        assert!(config.processes.is_empty());

        // Test composable node loading defaults
        assert_eq!(config.composable_node_loading.delay_load_node_millis, 2000);
        assert_eq!(
            config.composable_node_loading.load_node_timeout_millis,
            30000
        );
        assert_eq!(config.composable_node_loading.load_node_attempts, 3);
        assert_eq!(
            config
                .composable_node_loading
                .max_concurrent_load_node_spawn,
            10
        );

        // Test container readiness defaults
        assert!(config.container_readiness.wait_for_service_ready); // Now defaults to true
        assert_eq!(config.container_readiness.service_ready_timeout_secs, 120);
        assert_eq!(config.container_readiness.service_poll_interval_ms, 500);

        // Test interception defaults
        assert!(!config.interception.enabled);
        assert!(config.interception.frontier);
        assert!(config.interception.stats);
        assert_eq!(config.interception.ring_capacity, 65536);
    }

    #[test]
    fn test_pattern_matching() {
        let config = ProcessConfig {
            node_pattern: "/planning/*".to_string(),
            monitor: None,
            cpu_affinity: vec![],
            nice: None,
        };

        assert!(config.matches("/planning/behavior_planner"));
        assert!(config.matches("/planning/motion_planner"));
        assert!(!config.matches("/control/controller"));
    }

    #[test]
    fn test_should_monitor_logic() {
        let config = ResolvedMonitoringConfig {
            enabled: true,
            sample_interval_ms: 1000,
            monitor_all_nodes: true,
            monitor_patterns: vec![],
            process_configs: vec![],
        };

        assert!(config.should_monitor("/any/node"));

        // Disabled monitoring
        let config_disabled = ResolvedMonitoringConfig {
            enabled: false,
            ..config.clone()
        };
        assert!(!config_disabled.should_monitor("/any/node"));
    }

    #[test]
    fn test_should_monitor_with_patterns() {
        let config = ResolvedMonitoringConfig {
            enabled: true,
            sample_interval_ms: 1000,
            monitor_all_nodes: false,
            monitor_patterns: vec!["/planning/*".to_string(), "/control/*".to_string()],
            process_configs: vec![],
        };

        assert!(config.should_monitor("/planning/behavior"));
        assert!(config.should_monitor("/control/controller"));
        assert!(!config.should_monitor("/perception/detector"));
    }

    #[test]
    fn test_process_specific_override() {
        let config = ResolvedMonitoringConfig {
            enabled: true,
            sample_interval_ms: 1000,
            monitor_all_nodes: true,
            monitor_patterns: vec![],
            process_configs: vec![ProcessConfig {
                node_pattern: "/rviz*".to_string(),
                monitor: Some(false),
                cpu_affinity: vec![],
                nice: None,
            }],
        };

        assert!(config.should_monitor("/planning/behavior"));
        assert!(!config.should_monitor("/rviz2"));
    }

    #[test]
    fn test_nice_value_validation() {
        // Test that default config loads successfully
        let result = load_runtime_config(None, false, None, false);
        assert!(result.is_ok());

        // Test that config validates nice value ranges
        // (This would require a temp YAML file to fully test validation)
        let _config = ProcessConfig {
            node_pattern: "test".to_string(),
            monitor: None,
            cpu_affinity: vec![],
            nice: Some(10), // Valid nice value
        };
    }
}
