use clap::{Args, Parser, Subcommand, ValueEnum};
use std::path::PathBuf;

/// Parser backend selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum ParserBackend {
    /// Use Rust parser (default, no fallback)
    Rust,
    /// Use Python parser
    Python,
}

/// Features that can be selectively enabled
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum Feature {
    /// Resource monitoring (CPU, memory, I/O)
    Monitoring,
    /// Diagnostic monitoring (/diagnostics topic)
    Diagnostics,
    /// Web UI for node management
    WebUi,
}

/// Record and replay ROS 2 launches with inspection capabilities
#[derive(Parser)]
#[command(name = "play_launch")]
#[command(version)]
#[command(about = "Record and replay ROS 2 launches with inspection capabilities")]
#[command(after_help = "Examples:\n  \
    play_launch launch demo_nodes_cpp talker_listener.launch.py\n  \
    play_launch run demo_nodes_cpp talker\n  \
    play_launch dump launch autoware_launch planning_simulator.launch.xml --output autoware.json\n  \
    play_launch replay --input-file record.json")]
#[command(arg_required_else_help = true)]
pub struct Options {
    #[command(subcommand)]
    pub command: Command,
}

#[derive(Subcommand)]
pub enum Command {
    /// Launch a ROS 2 launch file (dump + replay)
    #[command(after_help = "Examples:\n  \
        play_launch launch demo_nodes_cpp talker_listener.launch.py\n  \
        play_launch launch /path/to/launch.py use_sim_time:=true")]
    Launch(LaunchArgs),

    /// Run a single ROS 2 node (dump + replay)
    #[command(after_help = "Examples:\n  \
        play_launch run demo_nodes_cpp talker\n  \
        play_launch run demo_nodes_cpp talker --ros-args -p topic:=chatter")]
    Run(RunArgs),

    /// Dump launch execution without replaying
    Dump(DumpArgs),

    /// Replay from existing record.json
    #[command(after_help = "Examples:\n  \
        play_launch replay\n  \
        play_launch replay --input-file autoware.json\n  \
        play_launch replay --disable-all\n  \
        play_launch replay --enable monitoring --enable web-ui\n  \
        play_launch replay --web-addr 0.0.0.0:8080")]
    Replay(ReplayArgs),

    /// Plot resource usage from execution logs
    #[command(after_help = "Examples:\n  \
        play_launch plot\n  \
        play_launch plot --log-dir play_log/2025-10-28_16-17-56\n  \
        play_launch plot --metrics cpu memory")]
    Plot(PlotArgs),

    /// Set CAP_SYS_PTRACE capability on I/O helper binary (requires sudo)
    #[command(name = "setcap-io-helper")]
    SetcapIoHelper,

    /// Check if I/O helper has required capabilities
    #[command(name = "verify-io-helper")]
    VerifyIoHelper,
}

/// Arguments for launching a launch file
#[derive(Args)]
pub struct LaunchArgs {
    /// Package name or path to launch file
    pub package_or_path: String,

    /// Launch file name (if package_or_path is a package name)
    pub launch_file: Option<String>,

    /// Launch arguments in KEY:=VALUE format
    #[arg(trailing_var_arg = true)]
    pub launch_arguments: Vec<String>,

    /// Parser backend to use for launch file parsing.
    /// - rust: Use Rust parser (default, fast, no fallback on error)
    /// - python: Use Python parser (slower, maximum compatibility)
    #[arg(long, value_enum, default_value = "rust")]
    pub parser: ParserBackend,

    #[command(flatten)]
    pub common: CommonOptions,
}

/// Arguments for running a single node
#[derive(Args)]
pub struct RunArgs {
    /// Package name
    pub package: String,

    /// Executable name
    pub executable: String,

    /// Node arguments
    #[arg(trailing_var_arg = true)]
    pub args: Vec<String>,

    #[command(flatten)]
    pub common: CommonOptions,
}

/// Arguments for dump command
#[derive(Args)]
pub struct DumpArgs {
    #[command(subcommand)]
    pub subcommand: DumpSubcommand,

    /// Output file for the dump
    #[arg(long, short = 'o', default_value = "record.json")]
    pub output: PathBuf,

    /// Enable debug output during dump
    #[arg(long)]
    pub debug: bool,
}

#[derive(Subcommand)]
pub enum DumpSubcommand {
    /// Dump a launch file execution
    Launch(LaunchArgs),
    /// Dump a single node execution
    Run(RunArgs),
}

/// Arguments for replay command
#[derive(Args, Default)]
pub struct ReplayArgs {
    /// Input record file to replay
    #[arg(long, default_value = "record.json")]
    pub input_file: PathBuf,

    #[command(flatten)]
    pub common: CommonOptions,
}

/// Common options shared across all commands
#[derive(Args, Clone)]
pub struct CommonOptions {
    /// Log directory for execution outputs
    #[arg(long, default_value = "play_log")]
    pub log_dir: PathBuf,

    /// Runtime configuration file (YAML).
    /// Contains composable node loading, container readiness, monitoring settings, and process control.
    /// Service readiness checking is enabled by default - disable in config if needed.
    #[arg(long, short = 'c', value_name = "PATH")]
    pub config: Option<PathBuf>,

    /// Enable only specific features. Can be specified multiple times.
    /// When used, only the specified features are enabled (others are disabled).
    /// Available features: monitoring, diagnostics, web-ui
    #[arg(long, value_enum, value_name = "FEATURE")]
    pub enable: Vec<Feature>,

    /// Disable resource monitoring (enabled by default).
    #[arg(long, conflicts_with = "enable")]
    pub disable_monitoring: bool,

    /// Disable diagnostic monitoring (enabled by default).
    #[arg(long, conflicts_with = "enable")]
    pub disable_diagnostics: bool,

    /// Disable web UI (enabled by default).
    #[arg(long, conflicts_with = "enable")]
    pub disable_web_ui: bool,

    /// Disable all features (monitoring, diagnostics, and web UI).
    #[arg(long, conflicts_with = "enable")]
    pub disable_all: bool,

    /// Resource sampling interval in milliseconds (overrides config file).
    #[arg(long, value_name = "MS")]
    pub monitor_interval_ms: Option<u64>,

    /// Enable verbose output (INFO level logging).
    /// Without this flag, only warnings and errors are shown.
    /// Use RUST_LOG env var for debug-level logging.
    #[arg(long, short = 'v')]
    pub verbose: bool,

    /// Run composable nodes in standalone mode instead of loading into containers
    #[arg(long)]
    pub standalone_composable_nodes: bool,

    /// Load composable nodes that have no matching container
    #[arg(long)]
    pub load_orphan_composable_nodes: bool,

    /// Disable automatic respawn even if configured in launch file
    #[arg(long)]
    pub disable_respawn: bool,

    /// Web UI address in IP:PORT format (default: 127.0.0.1:8080).
    /// Use 0.0.0.0:8080 to expose to network (insecure, use with caution).
    #[arg(long, value_name = "IP:PORT", default_value = "127.0.0.1:8080")]
    pub web_addr: String,
}

impl Default for CommonOptions {
    fn default() -> Self {
        Self {
            log_dir: PathBuf::from("play_log"),
            config: None,
            enable: Vec::new(),
            disable_monitoring: false,
            disable_diagnostics: false,
            disable_web_ui: false,
            disable_all: false,
            monitor_interval_ms: None,
            verbose: false,
            standalone_composable_nodes: false,
            load_orphan_composable_nodes: false,
            disable_respawn: false,
            web_addr: "127.0.0.1:8080".to_string(),
        }
    }
}

impl CommonOptions {
    /// Check if resource monitoring is enabled
    pub fn is_monitoring_enabled(&self) -> bool {
        // If --enable is used, check if monitoring is in the list
        if !self.enable.is_empty() {
            return self.enable.contains(&Feature::Monitoring);
        }
        // Otherwise, enabled by default unless explicitly disabled
        !self.disable_monitoring && !self.disable_all
    }

    /// Check if diagnostic monitoring is enabled
    pub fn is_diagnostics_enabled(&self) -> bool {
        // If --enable is used, check if diagnostics is in the list
        if !self.enable.is_empty() {
            return self.enable.contains(&Feature::Diagnostics);
        }
        // Otherwise, enabled by default unless explicitly disabled
        !self.disable_diagnostics && !self.disable_all
    }

    /// Check if web UI is enabled
    pub fn is_web_ui_enabled(&self) -> bool {
        // If --enable is used, check if web-ui is in the list
        if !self.enable.is_empty() {
            return self.enable.contains(&Feature::WebUi);
        }
        // Otherwise, enabled by default unless explicitly disabled
        !self.disable_web_ui && !self.disable_all
    }

    /// Parse web address into (IP, port) tuple
    pub fn parse_web_addr(&self) -> eyre::Result<(String, u16)> {
        let parts: Vec<&str> = self.web_addr.rsplitn(2, ':').collect();
        if parts.len() != 2 {
            return Err(eyre::eyre!(
                "Invalid web address format '{}'. Expected IP:PORT (e.g., 127.0.0.1:8080)",
                self.web_addr
            ));
        }

        let port_str = parts[0];
        let ip = parts[1].to_string();

        let port: u16 = port_str.parse().map_err(|_| {
            eyre::eyre!(
                "Invalid port number '{}' in web address '{}'",
                port_str,
                self.web_addr
            )
        })?;

        Ok((ip, port))
    }
}

/// Arguments for plot command
#[derive(Args)]
pub struct PlotArgs {
    /// Specific log directory to plot (e.g., play_log/2025-10-28_16-17-56)
    #[arg(long, value_name = "PATH")]
    pub log_dir: Option<PathBuf>,

    /// Base log directory to search for latest execution
    #[arg(long, value_name = "PATH", default_value = "./play_log")]
    pub base_log_dir: PathBuf,

    /// Output directory for generated plots
    #[arg(long, short = 'o', value_name = "PATH")]
    pub output_dir: Option<PathBuf>,

    /// Metrics to plot (can be specified multiple times)
    #[arg(long, short = 'm', value_name = "METRIC")]
    pub metrics: Vec<String>,

    /// List available metrics and exit
    #[arg(long)]
    pub list_metrics: bool,
}
