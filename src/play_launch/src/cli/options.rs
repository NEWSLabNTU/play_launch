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
    play_launch dump --output autoware.yaml launch autoware_launch planning_simulator.launch.xml\n  \
    play_launch replay --model autoware.yaml")]
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

    /// Replay from a SystemModel (primary; Phase 46.5) or a legacy
    /// record.json (deprecated compat, warns)
    #[command(after_help = "Examples:\n  \
        play_launch replay --model system_model.yaml\n  \
        play_launch replay --model system_model.yaml --disable-all\n  \
        play_launch replay --model system_model.yaml --enable monitoring --enable web-ui\n  \
        play_launch replay --model system_model.yaml --web-addr 0.0.0.0:8080\n  \
        play_launch replay --input-file record.json   # deprecated: legacy record-only path")]
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
        play_launch resolve /path/to/launch.xml --sched system.posix.yaml mode:=velodyne")]
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

    /// Launch arguments in KEY:=VALUE format. Flags may be placed before or
    /// after these (clap parses flags in any position); use `--` to force
    /// remaining tokens to be treated as positional launch arguments.
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

    /// Export the DECLARED causal graph (nodes, topics, pub/sub edges,
    /// node/scope paths, cycle catalogue) to `<path>` instead of — in
    /// addition to — the normal validation output. Format is picked by
    /// extension: `.json` (default, for tooling) or `.dot` (Graphviz, for
    /// human inspection via `dot -Tsvg`). Extension-less paths are written
    /// as JSON. This is an export, not a validation step — existing rules
    /// and exit codes are unaffected (Phase 42.1).
    #[arg(long, value_name = "PATH")]
    pub export_graph: Option<PathBuf>,
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
    /// Package name or path to launch file (omit when --record is given)
    pub package_or_path: Option<String>,

    /// Launch file name (if package_or_path is a package name)
    pub launch_file: Option<String>,

    /// Reuse an existing record.json instead of re-parsing the launch file
    /// (Phase 43.1). The record file is hashed into meta.inputs.
    #[arg(long, value_name = "PATH", conflicts_with = "package_or_path")]
    pub record: Option<PathBuf>,

    /// Launch arguments in KEY:=VALUE format. Flags may be placed before or
    /// after these (clap parses flags in any position); use `--` to force
    /// remaining tokens to be treated as positional launch arguments.
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

    /// R1-P1 — the integrator `system.toml` (deploy placement, transports,
    /// bridges, capability features, domain/locator/rmw ladder). Fills the
    /// model's execution layer; consumers never parse system.toml
    /// themselves (canonical-path decision).
    #[arg(long, value_name = "system.toml")]
    pub system: Option<std::path::PathBuf>,

    /// Scheduling target the platform file must declare.
    #[arg(long, default_value = "posix")]
    pub target: String,

    /// Parser backend to use for launch file parsing (Phase 46.4).
    /// - rust: Use Rust parser (default, fast)
    /// - python: Use Python parser (maximum compatibility)
    ///
    /// Both produce the full model — the contract/sched layers apply on the
    /// shared scope table regardless of parser (Phase 40.1), so they're
    /// populated whenever a contract sidecar / platform file (or
    /// --contracts/--sched) resolves, and empty only when none does.
    #[arg(long, value_enum, default_value = "rust")]
    pub parser: ParserBackend,

    /// Output path for the SystemModel YAML. `-` writes to stdout.
    #[arg(long, short = 'o', default_value = "system_model.yaml")]
    pub out: String,

    /// Print the merged scheduling plan with provenance per node (Phase
    /// 45.6), rendered from the SystemModel this invocation just built —
    /// same table `check --sched --explain` shows for the same inputs, one
    /// renderer. Only meaningful together with a resolved scheduling
    /// platform file; a no-op note is printed otherwise (not an error).
    #[arg(long)]
    pub explain: bool,
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

    /// Launch arguments in KEY:=VALUE format. Flags may be placed before or
    /// after these (clap parses flags in any position); use `--` to force
    /// remaining tokens to be treated as positional launch arguments.
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

/// `dump`'s output artifact (Phase 46.5 — one user-facing dump: the
/// SystemModel; `record.json` is a dev/parser-parity escape hatch). Only
/// meaningful for `dump launch`; `dump run` (a single executable, no launch
/// scope tree to build a model from) always writes record.json regardless
/// of this flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum DumpFormat {
    /// The SystemModel — same artifact `resolve` produces (default). The
    /// user-facing "one kind of dump."
    Model,
    /// Legacy record.json. Kept for `scripts/compare_records.py` /
    /// `just compare-dumps` cross-parser parity tooling — not the
    /// user-facing default.
    Record,
}

/// Arguments for dump command
#[derive(Args)]
pub struct DumpArgs {
    #[command(subcommand)]
    pub subcommand: DumpSubcommand,

    /// Output file for the dump. Defaults to `system_model.yaml` for the
    /// SystemModel format (default), or `record.json` for `--format
    /// record` and `dump run`. May be given before or after the
    /// `launch`/`run` subcommand and its launch arguments.
    #[arg(long, short = 'o', global = true)]
    pub output: Option<PathBuf>,

    /// Dump format: `model` (default, the SystemModel) or `record` (legacy
    /// record.json, dev/parser-parity tooling only). See [`DumpFormat`].
    /// May be given before or after the subcommand.
    #[arg(long, value_enum, default_value = "model", global = true)]
    pub format: DumpFormat,

    /// Enable debug output during dump. May be given before or after the
    /// subcommand.
    #[arg(long, global = true)]
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
    /// Input record.json to replay when `--model` is absent. DEPRECATED
    /// (Phase 46.5): the SystemModel (`--model`) is the primary replay
    /// source now; this legacy record-only path prints a one-time
    /// deprecation warning and is kept as a compat path (no hard removal
    /// this wave — rollback safety / one release's grace).
    #[arg(long, default_value = "record.json")]
    pub input_file: PathBuf,

    /// SystemModel emitted by `play_launch resolve`/`dump` (Phase 43/46.5:
    /// the primary replay source). Spawns directly from the model's
    /// `structure.nodes` — no accompanying `--input-file` record is
    /// required (Phase 46.4: the model↔record binding gate was removed
    /// once the model became a self-sufficient spawn source).
    #[arg(long, value_name = "PATH")]
    pub model: Option<PathBuf>,

    /// Print the merged scheduling plan with provenance per node (Phase
    /// 45.6), rendered from the given `--model`'s `execution.sched` — same
    /// table `check --sched --explain`/`resolve --explain` show for the
    /// same inputs, one renderer, no re-derive. Requires `--model`; a note
    /// is printed and the replay proceeds otherwise (not an error).
    #[arg(long)]
    pub explain: bool,

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

#[cfg(test)]
mod flag_ordering_tests {
    //! 47.A1/47.A3: `launch_arguments` (`KEY:=VALUE`) used to be declared
    //! `#[arg(trailing_var_arg = true)]`, which made clap swallow *every*
    //! token after the first positional — including recognized flags —
    //! into the `Vec<String>` verbatim. That silently dropped flags placed
    //! after a launch argument (e.g. `dump launch pkg file mode:=v
    //! --output x.yaml` never honored `--output`). Removing
    //! `trailing_var_arg` restores clap's default behavior: a `Vec`-typed
    //! positional stops consuming once it hits a token that matches a
    //! known flag, so flags now parse in any position. These tests pin
    //! that behavior for every affected subcommand, both orders (flags
    //! before vs. after the launch arguments), and confirm an unrecognized
    //! bare flag still produces a clear parse error rather than being
    //! absorbed as a launch argument.
    use super::*;

    fn parse(args: &[&str]) -> Result<Options, clap::Error> {
        let mut full = vec!["play_launch"];
        full.extend_from_slice(args);
        Options::try_parse_from(full)
    }

    #[test]
    fn launch_flag_after_launch_arguments() {
        let opts = parse(&[
            "launch",
            "pkg",
            "file.launch.xml",
            "mode:=velodyne",
            "--parser",
            "python",
        ])
        .expect("flag after KEY:=VALUE must parse");
        let Command::Launch(args) = opts.command else {
            panic!("expected Launch");
        };
        assert_eq!(args.launch_arguments, vec!["mode:=velodyne".to_string()]);
        assert_eq!(args.parser, ParserBackend::Python);
    }

    #[test]
    fn launch_flag_before_launch_arguments_still_works() {
        let opts = parse(&[
            "launch",
            "pkg",
            "file.launch.xml",
            "--parser",
            "python",
            "mode:=velodyne",
        ])
        .expect("flag before KEY:=VALUE must still parse");
        let Command::Launch(args) = opts.command else {
            panic!("expected Launch");
        };
        assert_eq!(args.launch_arguments, vec!["mode:=velodyne".to_string()]);
        assert_eq!(args.parser, ParserBackend::Python);
    }

    #[test]
    fn resolve_flag_after_launch_arguments() {
        let opts = parse(&[
            "resolve",
            "pkg",
            "file.launch.xml",
            "mode:=velodyne",
            "--sched",
            "system.posix.yaml",
            "-o",
            "model.yaml",
        ])
        .expect("--sched/-o after KEY:=VALUE must parse");
        let Command::Resolve(args) = opts.command else {
            panic!("expected Resolve");
        };
        assert_eq!(args.launch_arguments, vec!["mode:=velodyne".to_string()]);
        assert_eq!(args.sched, Some(PathBuf::from("system.posix.yaml")));
        assert_eq!(args.out, "model.yaml");
    }

    #[test]
    fn check_flag_after_launch_arguments() {
        let opts = parse(&[
            "check",
            "pkg",
            "file.launch.xml",
            "mode:=velodyne",
            "--sched",
            "system.posix.yaml",
        ])
        .expect("--sched after KEY:=VALUE must parse");
        let Command::Check(args) = opts.command else {
            panic!("expected Check");
        };
        assert_eq!(args.launch_arguments, vec!["mode:=velodyne".to_string()]);
        assert_eq!(args.sched, Some(PathBuf::from("system.posix.yaml")));
    }

    #[test]
    fn dump_launch_flag_after_two_launch_arguments() {
        // The exact Autoware shape this wave must verify end-to-end:
        // multiple KEY:=VALUE args, then a flag, all after `dump launch`.
        let opts = parse(&[
            "dump",
            "launch",
            "pkg",
            "file.launch.xml",
            "vehicle_model:=sample_vehicle",
            "sensor_model:=sample_sensor_kit",
            "--output",
            "/tmp/aw.yaml",
        ])
        .expect("--output after KEY:=VALUE launch args must parse (dump's global flags)");
        let Command::Dump(dump_args) = opts.command else {
            panic!("expected Dump");
        };
        assert_eq!(dump_args.output, Some(PathBuf::from("/tmp/aw.yaml")));
        let DumpSubcommand::Launch(args) = dump_args.subcommand else {
            panic!("expected Dump::Launch");
        };
        assert_eq!(
            args.launch_arguments,
            vec![
                "vehicle_model:=sample_vehicle".to_string(),
                "sensor_model:=sample_sensor_kit".to_string(),
            ]
        );
    }

    #[test]
    fn dump_launch_flag_before_subcommand_still_works() {
        let opts = parse(&[
            "dump",
            "--output",
            "/tmp/aw.yaml",
            "launch",
            "pkg",
            "file.launch.xml",
            "vehicle_model:=sample_vehicle",
        ])
        .expect("--output before `launch` must still parse");
        let Command::Dump(dump_args) = opts.command else {
            panic!("expected Dump");
        };
        assert_eq!(dump_args.output, Some(PathBuf::from("/tmp/aw.yaml")));
    }

    #[test]
    fn unrecognized_flag_in_launch_arguments_errors_clearly() {
        // A bare unknown flag among launch arguments must be rejected by
        // clap with a clear error, not silently absorbed into
        // `launch_arguments` (the old trailing_var_arg behavior).
        let result = parse(&[
            "check",
            "pkg",
            "file.launch.xml",
            "mode:=velodyne",
            "--not-a-real-flag",
        ]);
        let err = match result {
            Ok(_) => panic!("unknown flag must not be silently swallowed"),
            Err(err) => err,
        };
        assert_eq!(err.kind(), clap::error::ErrorKind::UnknownArgument);
    }

    #[test]
    fn double_dash_separator_forces_positional_launch_arguments() {
        // `--` still works as an explicit separator: everything after it
        // is treated as a positional launch argument even if it looks
        // like a flag.
        let opts = parse(&[
            "launch",
            "pkg",
            "file.launch.xml",
            "--",
            "--looks-like-flag",
        ])
        .expect("-- separator must force positional parsing");
        let Command::Launch(args) = opts.command else {
            panic!("expected Launch");
        };
        assert_eq!(args.launch_arguments, vec!["--looks-like-flag".to_string()]);
    }
}
