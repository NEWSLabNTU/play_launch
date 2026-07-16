use clap::{Args, Parser, Subcommand, ValueEnum};
use std::path::PathBuf;

use crate::execution::sched_apply::SchedApplyMode;

/// Parser backend selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum ParserBackend {
    /// Use Rust parser (default, no fallback)
    Rust,
    /// Use Python parser
    Python,
}

/// Container mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum ContainerMode {
    /// Use play_launch_container with ComponentEvent publishing
    Observable,
    /// Use play_launch_container with clone(CLONE_VM) per-node isolation (default)
    Isolated,
    /// Use the original container from the launch file (no override)
    Stock,
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

    /// Grant CAP_SYS_PTRACE to the I/O helper (for per-process I/O
    /// monitoring). Requires sudo. NOTE: the main binary is deliberately NOT
    /// capped — a file capability would put it in secure-execution mode and
    /// break ROS library loading. RT scheduling (`--sched`) needs root.
    #[command(name = "setcap")]
    Setcap,

    /// Check capabilities: the I/O helper has CAP_SYS_PTRACE, and the main
    /// binary has none (a capability on it breaks ROS library loading)
    #[command(name = "verify")]
    Verify,

    /// Extract per-node or per-launch-file context from record.json
    #[command(after_help = "Examples:\n  \
        play_launch context record.json --tree\n  \
        play_launch context record.json --node /perception/centerpoint\n  \
        play_launch context record.json --launch tier4_system_launch system.launch.xml")]
    Context(ContextArgs),

    /// Check manifest contracts against a launch file
    #[command(after_help = "Examples:\n  \
        play_launch check autoware_launch planning_simulator.launch.xml\n  \
        play_launch check --contracts ~/contracts /path/to/launch.py arg:=value")]
    Check(CheckArgs),

    /// Resolve launch + contracts + scheduling into a SystemModel YAML
    /// (RFC-0050 / docs/design/system-model.md): one fully-resolved,
    /// checked artifact per concrete arg-set. Refuses to emit when the
    /// contract checker reports errors; warnings are embedded in the model.
    #[command(after_help = "Examples:\n  \
        play_launch resolve demo_pkg pipeline.launch.xml --out system_model.yaml\n  \
        play_launch resolve /path/to/launch.xml mode:=velodyne --sched system.posix.yaml")]
    Resolve(ResolveArgs),

    /// Manage contract/platform-file overlays (Phase 41.4, design §3.3)
    #[command(after_help = "Examples:\n  \
        play_launch contract eject rt_demo bringup.launch.xml\n  \
        play_launch contract eject rt_demo bringup.launch.xml --into ~/.config/play_launch/contracts")]
    Contract(ContractArgs),
}

/// Arguments for `play_launch contract`
#[derive(Args)]
pub struct ContractArgs {
    #[command(subcommand)]
    pub subcommand: ContractSubcommand,
}

#[derive(Subcommand)]
pub enum ContractSubcommand {
    /// Copy the resolved provider contract (and target's platform file, if
    /// any) into the overlay tree, ready to edit — editing never touches
    /// `/opt` (design §3.3).
    Eject(ContractEjectArgs),
}

/// Arguments for `play_launch contract eject`
#[derive(Args)]
pub struct ContractEjectArgs {
    /// Package name or path to launch file
    pub package_or_path: String,

    /// Launch file name (if package_or_path is a package name)
    pub launch_file: Option<String>,

    /// Which scheduling target's platform file to eject (`<stem>.system.<target>.yaml`).
    #[arg(long, default_value = "posix")]
    pub target: String,

    /// Overlay root to eject into. Defaults to the discovered overlay root
    /// (same discovery as `check --contracts`: `$PLAY_LAUNCH_CONTRACTS`,
    /// then `$XDG_CONFIG_HOME/play_launch/contracts`, then
    /// `/etc/play_launch/contracts`) — errors if none of those exist yet
    /// and `--into` wasn't given.
    #[arg(long, value_name = "PATH")]
    pub into: Option<PathBuf>,

    /// Overwrite existing overlay files. Without this flag, `eject` refuses
    /// to touch a destination that already exists.
    #[arg(long)]
    pub force: bool,
}

/// Arguments for the context extraction command
#[derive(Args)]
pub struct ContextArgs {
    /// Path to record.json
    pub record: String,

    /// Show context for a specific node (by FQN)
    #[arg(long)]
    pub node: Option<String>,

    /// Show context for a launch file invocation (PKG FILE)
    #[arg(long, num_args = 2, value_names = ["PKG", "FILE"])]
    pub launch: Option<Vec<String>>,

    /// Disambiguate launch file by namespace
    #[arg(long)]
    pub namespace: Option<String>,

    /// Show all invocations of a launch file
    #[arg(long)]
    pub all: bool,

    /// Print the launch include tree
    #[arg(long)]
    pub tree: bool,
}

/// Arguments for `play_launch check`
#[derive(Args)]
pub struct CheckArgs {
    /// Package name or path to launch file
    pub package_or_path: String,

    /// Launch file name (if package_or_path is a package name)
    pub launch_file: Option<String>,

    /// Launch arguments in KEY:=VALUE format
    #[arg(trailing_var_arg = true)]
    pub launch_arguments: Vec<String>,

    /// Overlay root for user-supplied contracts, mirroring the launch tree:
    /// <dir>/<pkg>/launch/<stem>.contract.yaml. Checked before provider
    /// sidecars.
    #[arg(long, value_name = "PATH")]
    pub contracts: Option<PathBuf>,

    /// Disable the provider-sidecar channel for BOTH contracts
    /// (<stem>.contract.yaml) and scheduling platform files
    /// (<stem>.system.<target>.yaml) shipped next to the launch file.
    /// On by default.
    #[arg(long)]
    pub no_provider_contracts: bool,

    /// Path to a scheduling platform file — v2 `.yaml` schema (mapper +
    /// resources + overrides) or legacy `.toml` (dispatched by extension).
    /// When given, `check` also derives + validates a plan for `--target`.
    #[arg(long)]
    pub sched: Option<std::path::PathBuf>,

    /// Which scheduling target the platform file must declare (`target:` in
    /// the v2 schema; legacy `.toml` always implies `posix`). `posix` is
    /// Linux RT (the only target `play_launch` itself applies); RTOS targets
    /// (`zephyr`, `freertos`, ...) are for nano-ros's own consumption of the
    /// same file format.
    #[arg(long, default_value = "posix")]
    pub target: String,

    /// Output format: terminal (default, with source excerpts) or json
    #[arg(long, default_value = "terminal")]
    pub format: String,

    /// Show only diagnostics from these rules. Repeat to allow multiple.
    /// Example: --rule satisfiability --rule consistency
    #[arg(long, value_name = "RULE_ID")]
    pub rule: Vec<String>,

    /// Print the merged scheduling plan with provenance per node (design
    /// §7): which platform-file override, mapper-derived fact, or default
    /// placement produced each node's final class/priority/core, plus the
    /// platform-file and per-scope contract paths that fed the pipeline.
    /// Only meaningful together with a resolved scheduling platform file
    /// (`--sched`, or one resolved via the overlay/provider channels) — a
    /// no-op note is printed otherwise (not an error).
    #[arg(long)]
    pub explain: bool,
}

impl CheckArgs {
    /// Build the two-step `ContractSources` from this command's flags.
    ///
    /// The overlay root is discovered (Phase 41.3 §3.2) when `--contracts`
    /// isn't given: `$PLAY_LAUNCH_CONTRACTS`, then
    /// `$XDG_CONFIG_HOME/play_launch/contracts`, then
    /// `/etc/play_launch/contracts` — first existing wins.
    pub fn contract_sources(&self) -> crate::ros::manifest_loader::ContractSources {
        crate::ros::manifest_loader::ContractSources {
            overlay: crate::ros::manifest_loader::discover_overlay_root(self.contracts.as_deref()),
            provider: !self.no_provider_contracts,
        }
    }
}

/// Arguments for `play_launch resolve`
#[derive(Args)]
pub struct ResolveArgs {
    /// Package name or path to launch file
    pub package_or_path: String,

    /// Launch file name (if package_or_path is a package name)
    pub launch_file: Option<String>,

    /// Launch arguments in KEY:=VALUE format
    #[arg(trailing_var_arg = true)]
    pub launch_arguments: Vec<String>,

    /// Overlay root for user-supplied contracts (see `check --contracts`).
    #[arg(long, value_name = "PATH")]
    pub contracts: Option<PathBuf>,

    /// Disable the provider-sidecar channel (see `check`).
    #[arg(long)]
    pub no_provider_contracts: bool,

    /// Path to a scheduling platform file (v2 `.yaml` or legacy `.toml`).
    #[arg(long)]
    pub sched: Option<std::path::PathBuf>,

    /// Scheduling target the platform file must declare.
    #[arg(long, default_value = "posix")]
    pub target: String,

    /// Output path for the SystemModel YAML. `-` writes to stdout.
    #[arg(long, short = 'o', default_value = "system_model.yaml")]
    pub out: String,
}

impl ResolveArgs {
    /// Same two-step contract source resolution as `check`.
    pub fn contract_sources(&self) -> crate::ros::manifest_loader::ContractSources {
        crate::ros::manifest_loader::ContractSources {
            overlay: crate::ros::manifest_loader::discover_overlay_root(self.contracts.as_deref()),
            provider: !self.no_provider_contracts,
        }
    }
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

    /// Block $(command ...) substitutions in launch files.
    /// When set, the parser rejects any $(command) with an error.
    #[arg(long)]
    pub block_commands: bool,

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

    /// Container mode: which container binary to use for composable nodes.
    /// - observable: use play_launch_container with ComponentEvent publishing
    /// - isolated: use play_launch_container with clone(CLONE_VM) per-node isolation (default)
    /// - stock: use the original container from the launch file (no override)
    #[arg(long, value_enum, default_value = "isolated")]
    pub container_mode: ContainerMode,

    /// Web UI address in IP:PORT format (default: 127.0.0.1:8080).
    /// Use 0.0.0.0:8080 to expose to network (insecure, use with caution).
    #[arg(long, value_name = "IP:PORT", default_value = "127.0.0.1:8080")]
    pub web_addr: String,

    /// Overlay root for user-supplied contracts, mirroring the launch tree:
    /// <dir>/<pkg>/launch/<stem>.contract.yaml. Checked before provider
    /// sidecars.
    #[arg(long, value_name = "PATH")]
    pub contracts: Option<PathBuf>,

    /// Disable the provider-sidecar channel for BOTH contracts
    /// (<stem>.contract.yaml) and scheduling platform files
    /// (<stem>.system.<target>.yaml) shipped next to the launch file.
    /// On by default.
    #[arg(long)]
    pub no_provider_contracts: bool,

    /// Runtime enforcement mode for manifest contracts (Phase 36.3).
    /// Contracts come from any channel (overlay/provider); with no
    /// contracts resolved the engine has nothing to enforce. Off: no runtime
    /// checks. Warn: log violations. Strict: exit non-zero on first
    /// violation. RecordOnly: collect events without evaluating rules.
    #[arg(long, value_enum, default_value = "warn")]
    pub enforce_rules: EnforceMode,

    /// Phase 36.7: block unauthorized publisher/subscription creation
    /// at the rcl layer. The set of allowed topic FQNs is written from the
    /// merged ManifestIndex (any contract channel) and passed to every child
    /// via env var. Hooked rcl init calls for topics not in the set return
    /// `RCL_RET_TOPIC_INVALID` (1004) — the publisher/subscription is never
    /// created. If NO contract declares any topic, blocking is disabled with
    /// a warning (an empty allowlist would block every endpoint). Off by
    /// default because nodes that don't handle init failure may crash.
    #[arg(long, default_value_t = false)]
    pub block_unauthorized_endpoints: bool,

    /// Phase 38: path to a scheduling platform file — v2 `.yaml` schema
    /// (mapper + resources + overrides) or legacy `.toml` (dispatched by
    /// extension; Phase 41.2). When set, replay derives + validates a plan
    /// for `--target` and (per `--sched-apply`) applies SCHED_FIFO/RR +
    /// priority + CPU affinity to each spawned node/container process. Same
    /// file `play_launch check --sched` validates.
    #[arg(long, value_name = "PATH")]
    pub sched: Option<PathBuf>,

    /// Which scheduling target the platform file must declare (`target:` in
    /// the v2 schema; legacy `.toml` always implies `posix`). Only `posix`
    /// (Linux RT) is ever applied by `play_launch` itself.
    #[arg(long, default_value = "posix")]
    pub target: String,

    /// Phase 38: how to apply the scheduling spec. `off` = resolve + report
    /// only (no syscalls). `warn` = apply, log a warning and continue on
    /// failure. `strict` = abort the run on any capability/apply failure.
    /// Only meaningful with `--sched`.
    #[arg(long, value_enum, default_value = "warn")]
    pub sched_apply: SchedApplyMode,
}

/// Runtime enforcement mode for manifest contracts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum EnforceMode {
    /// Skip runtime checks entirely.
    Off,
    /// Log violations to `play_log/<ts>/runtime_violations.jsonl`. Never exit early.
    Warn,
    /// First violation triggers shutdown and non-zero exit (CI mode).
    Strict,
    /// Collect events without evaluating rules. For offline analysis.
    RecordOnly,
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
            container_mode: ContainerMode::Isolated,
            web_addr: "127.0.0.1:8080".to_string(),
            contracts: None,
            no_provider_contracts: false,
            enforce_rules: EnforceMode::Warn,
            block_unauthorized_endpoints: false,
            sched: None,
            target: "posix".to_string(),
            sched_apply: SchedApplyMode::Warn,
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

    /// Build the two-step `ContractSources` from this command's flags.
    ///
    /// The overlay root is discovered (Phase 41.3 §3.2) when `--contracts`
    /// isn't given: `$PLAY_LAUNCH_CONTRACTS`, then
    /// `$XDG_CONFIG_HOME/play_launch/contracts`, then
    /// `/etc/play_launch/contracts` — first existing wins.
    pub fn contract_sources(&self) -> crate::ros::manifest_loader::ContractSources {
        crate::ros::manifest_loader::ContractSources {
            overlay: crate::ros::manifest_loader::discover_overlay_root(self.contracts.as_deref()),
            provider: !self.no_provider_contracts,
        }
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
