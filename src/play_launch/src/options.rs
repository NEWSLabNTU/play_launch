use clap::{Args, Parser, Subcommand};
use std::path::PathBuf;

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
        play_launch replay --input-file autoware.json --enable-monitoring")]
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
#[derive(Args, Default, Clone)]
pub struct CommonOptions {
    /// Log directory for execution outputs
    #[arg(long, default_value = "play_log")]
    pub log_dir: PathBuf,

    /// Runtime configuration file (YAML).
    /// Contains composable node loading, container readiness, monitoring settings, and process control.
    /// Service readiness checking is enabled by default - disable in config if needed.
    #[arg(long, short = 'c', value_name = "PATH")]
    pub config: Option<PathBuf>,

    /// Enable resource monitoring for all nodes.
    /// This overrides the 'monitoring.enabled' setting in the config file.
    #[arg(long)]
    pub enable_monitoring: bool,

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
