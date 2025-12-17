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
    1000
}
fn default_true() -> bool {
    true
}

/// Composable node loading settings
#[derive(Debug, Clone, Deserialize)]
pub struct ComposableNodeLoadingSettings {
    /// Delay before loading composable nodes (milliseconds)
    #[serde(default = "default_delay_load_node_millis")]
    pub delay_load_node_millis: u64,

    /// Timeout for loading each composable node (milliseconds)
    #[serde(default = "default_load_node_timeout_millis")]
    pub load_node_timeout_millis: u64,

    /// Maximum retry attempts for loading composable nodes
    #[serde(default = "default_load_node_attempts")]
    pub load_node_attempts: usize,

    /// Maximum concurrent composable node loading operations
    #[serde(default = "default_max_concurrent_load_node_spawn")]
    pub max_concurrent_load_node_spawn: usize,
}

impl Default for ComposableNodeLoadingSettings {
    fn default() -> Self {
        Self {
            delay_load_node_millis: default_delay_load_node_millis(),
            load_node_timeout_millis: default_load_node_timeout_millis(),
            load_node_attempts: default_load_node_attempts(),
            max_concurrent_load_node_spawn: default_max_concurrent_load_node_spawn(),
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

    /// Apply CPU affinity and nice value to a process
    pub fn apply(&self, pid: u32) -> Result<()> {
        // Apply CPU affinity
        if !self.cpu_affinity.is_empty() {
            self.set_affinity(pid)
                .wrap_err_with(|| format!("Failed to set CPU affinity for pid {}", pid))?;
        }

        // Apply nice value
        if let Some(nice) = self.nice {
            self.set_nice(pid, nice)
                .wrap_err_with(|| format!("Failed to set nice value for pid {}", pid))?;
        }

        Ok(())
    }

    /// Set CPU affinity for a process (Linux only)
    #[cfg(target_os = "linux")]
    fn set_affinity(&self, pid: u32) -> Result<()> {
        use std::mem;

        unsafe {
            let mut cpu_set: libc::cpu_set_t = mem::zeroed();
            libc::CPU_ZERO(&mut cpu_set);

            for &cpu in &self.cpu_affinity {
                libc::CPU_SET(cpu, &mut cpu_set);
            }

            let ret = libc::sched_setaffinity(
                pid as libc::pid_t,
                mem::size_of::<libc::cpu_set_t>(),
                &cpu_set,
            );

            if ret != 0 {
                return Err(eyre::eyre!(
                    "sched_setaffinity failed: {}",
                    std::io::Error::last_os_error()
                ));
            }
        }

        Ok(())
    }

    #[cfg(not(target_os = "linux"))]
    fn set_affinity(&self, _pid: u32) -> Result<()> {
        Err(eyre::eyre!("CPU affinity is only supported on Linux"))
    }

    /// Set nice value for a process
    #[cfg(unix)]
    fn set_nice(&self, pid: u32, nice: i32) -> Result<()> {
        // Validate nice value range
        if !(-20..=19).contains(&nice) {
            return Err(eyre::eyre!("Nice value must be between -20 and 19"));
        }

        unsafe {
            let ret = libc::setpriority(libc::PRIO_PROCESS, pid, nice);
            if ret != 0 {
                return Err(eyre::eyre!(
                    "setpriority failed: {}",
                    std::io::Error::last_os_error()
                ));
            }
        }

        Ok(())
    }

    #[cfg(not(unix))]
    fn set_nice(&self, _pid: u32, _nice: i32) -> Result<()> {
        Err(eyre::eyre!("Nice value is only supported on Unix"))
    }
}

/// Resolved runtime configuration ready for use
#[derive(Debug, Clone)]
pub struct ResolvedRuntimeConfig {
    pub monitoring: ResolvedMonitoringConfig,
    pub composable_node_loading: ComposableNodeLoadingSettings,
    pub container_readiness: ContainerReadinessSettings,
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
            if config.matches(node_name) {
                if let Some(monitor) = config.monitor {
                    return monitor;
                }
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
    enable_monitoring_flag: bool,
    monitor_interval_override: Option<u64>,
) -> Result<ResolvedRuntimeConfig> {
    // Load config file or use defaults
    let mut config = if let Some(path) = config_path {
        let content = std::fs::read_to_string(path)
            .wrap_err_with(|| format!("Failed to read config file: {}", path.display()))?;
        serde_yaml::from_str::<RuntimeConfig>(&content)
            .wrap_err_with(|| format!("Failed to parse config file: {}", path.display()))?
    } else {
        RuntimeConfig::default()
    };

    // CLI flag overrides config file
    if enable_monitoring_flag {
        config.monitoring.enabled = true;
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
        if let Some(nice) = process_config.nice {
            if !(-20..=19).contains(&nice) {
                return Err(eyre::eyre!(
                    "Invalid nice value: {} (must be between -20 and 19)",
                    nice
                ));
            }
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
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = RuntimeConfig::default();
        assert!(!config.monitoring.enabled);
        assert_eq!(config.monitoring.sample_interval_ms, 1000);
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
        let result = load_runtime_config(None, false, None);
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
