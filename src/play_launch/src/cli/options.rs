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
// `dump` used to head the third example. RFC-0060 W3 moved that verb to
// ros-launch-resolve; `resolve` is the play_launch-side equivalent.
#[command(after_help = "Examples:\n  \
    play_launch launch demo_nodes_cpp topics/talker_listener.launch.py\n  \
    play_launch run demo_nodes_cpp talker\n  \
    play_launch resolve autoware_launch planning_simulator.launch.xml --out autoware.yaml\n  \
    play_launch up --model autoware.yaml")]
#[command(arg_required_else_help = true)]
pub struct Options {
    #[command(subcommand)]
    pub command: Command,
}

#[derive(Subcommand)]
pub enum Command {
    /// Launch a ROS 2 launch file (dump + replay)
    #[command(after_help = "Examples:\n  \
        play_launch launch demo_nodes_cpp topics/talker_listener.launch.py\n  \
        play_launch launch /path/to/launch.py use_sim_time:=true")]
    Launch(LaunchArgs),

    /// Run a single ROS 2 node (dump + replay)
    #[command(after_help = "Examples:\n  \
        play_launch run demo_nodes_cpp talker\n  \
        play_launch run demo_nodes_cpp talker --ros-args -p topic:=chatter")]
    Run(RunArgs),

    /// Bring a system up from a resolved SystemModel and supervise it.
    ///
    /// Renamed from `replay` in 0.9.0: it replays nothing. It loads a
    /// declarative artifact and spawns from it — the old name was a fossil
    /// of `record.json`, which Phase 47 removed.
    #[command(after_help = "Examples:\n  \
        play_launch up system_model.yaml\n  \
        play_launch up --model system_model.yaml --disable-all\n  \
        play_launch up system_model.yaml --web-addr 0.0.0.0:8080")]
    Up(UpArgs),

    /// Renamed to `up` in 0.9.0. Hidden; accepts the old arguments so the
    /// error can name the replacement. DELETE AT 1.0.0.
    #[command(hide = true)]
    Replay(UpArgs),

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

    /// Extract per-node or per-launch-file context from a SystemModel
    /// (`system_model.yaml`) — the launch include tree, per-node origin +
    /// launch fields, and per-include args (Phase 49: reads the model, not the
    /// retired record.json)
    #[command(after_help = "Examples:\n  \
        play_launch context system_model.yaml --tree\n  \
        play_launch context system_model.yaml --node /perception/centerpoint\n  \
        play_launch context system_model.yaml --launch tier4_system_launch system.launch.xml")]
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
}

/// Arguments for the context extraction command
#[derive(Args)]
pub struct ContextArgs {
    /// Path to a SystemModel (`system_model.yaml`, from `resolve`/`dump`)
    pub model: String,

    /// Show context for a specific node (by FQN)
    #[arg(long)]
    pub node: Option<String>,

    /// Show context for a launch file invocation (PKG FILE)
    #[arg(long, num_args = 2, value_names = ["PKG", "FILE"])]
    pub launch: Option<Vec<String>>,

    /// Show all invocations of a launch file (when a file is included more
    /// than once); otherwise a single match is shown and duplicates are listed.
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
    pub fn contract_sources(&self) -> ros_launch_resolve::ros::manifest_loader::ContractSources {
        ros_launch_resolve::ros::manifest_loader::ContractSources {
            overlay: ros_launch_resolve::ros::manifest_loader::discover_overlay_root(
                self.contracts.as_deref(),
            ),
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

    /// Validate contracts and scheduling, print the diagnostics, and exit
    /// without spawning anything. Exit status is 0 when clean, 1 when the
    /// checker reports errors.
    ///
    /// This is a pass/fail gate. For `--format json`, rule filters,
    /// `--explain` or graph export, use `ros-launch-resolve check`, which
    /// carries the full diagnostic surface and needs no ROS install.
    #[arg(long)]
    pub check: bool,

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

    /// Resolve and validate the scheduling platform file for `--target`,
    /// then exit without spawning.
    ///
    /// Contracts are NOT checked and cannot be: they are keyed by launch
    /// file (`<pkg>/launch/<stem>.contract.yaml`), and `run` has no launch
    /// file, so no sidecar can apply. The output says so explicitly rather
    /// than reporting a pass over an empty check.
    #[arg(long)]
    pub check: bool,

    #[command(flatten)]
    pub common: CommonOptions,
}

/// Arguments for the `up` command (renamed from `replay` in 0.9.0)
#[derive(Args, Default)]
pub struct UpArgs {
    /// SystemModel emitted by `play_launch resolve`/`dump` — the sole
    /// spawn source (Phase 47.B3: the deprecated `--input-file
    /// record.json` compat path is retired). May be given positionally or
    /// via `--model`; giving both is an error.
    #[arg(value_name = "MODEL", conflicts_with = "model")]
    pub model_path: Option<PathBuf>,

    /// SystemModel emitted by `play_launch resolve`/`dump`. Equivalent to
    /// passing the path positionally. Spawns directly from the model's
    /// `structure.nodes` — no companion file required (Phase 46.4: the
    /// model↔record binding gate was removed once the model became a
    /// self-sufficient spawn source).
    #[arg(long, value_name = "PATH")]
    pub model: Option<PathBuf>,

    /// Print the merged scheduling plan with provenance per node (Phase
    /// 45.6), rendered from the model's `execution.sched` — same table
    /// `check --sched --explain`/`resolve --explain` show for the same
    /// inputs, one renderer, no re-derive.
    #[arg(long)]
    pub explain: bool,

    /// (removed) The Phase 46 `--input-file record.json` replay flag.
    /// Retained HIDDEN only so `up --input-file ...` produces a helpful
    /// migration error (see `handle_up`) instead of clap's bare
    /// "unexpected argument" — Phase 47.B3 removed record.json replay
    /// entirely. Never used as a value.
    #[arg(long, hide = true, value_name = "PATH")]
    pub input_file: Option<PathBuf>,

    #[command(flatten)]
    pub common: CommonOptions,
}

impl UpArgs {
    /// The SystemModel path, however it was given (positional or `--model`).
    /// `clap`'s `conflicts_with` already rejects both being set.
    pub fn model_path(&self) -> Option<&PathBuf> {
        self.model_path.as_ref().or(self.model.as_ref())
    }
}

/// Common options shared across all commands (phase-52.1: grouped into
/// `#[command(flatten)]` concern structs — flag names are unchanged).
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

    /// Enable verbose output (INFO level logging).
    /// Without this flag, only warnings and errors are shown.
    /// Use RUST_LOG env var for debug-level logging.
    #[arg(long, short = 'v')]
    pub verbose: bool,

    #[command(flatten)]
    pub features: FeatureOptions,

    #[command(flatten)]
    pub containers: ContainerOptions,

    #[command(flatten)]
    pub web: WebOptions,

    #[command(flatten)]
    pub contract_opts: ContractOptions,

    #[command(flatten)]
    pub sched_opts: SchedOptions,
}

/// Feature toggles (monitoring / diagnostics / web UI)
#[derive(Args, Clone, Default)]
pub struct FeatureOptions {
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
}

/// Container / composable-node behavior
#[derive(Args, Clone)]
pub struct ContainerOptions {
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

    /// Phase 52: per-composable total LoadNode budget in SECONDS while a
    /// container is busy (overrides config
    /// `composable_node_loading.load_total_budget_secs`, default 600).
    /// Raise for containers whose composable ctors block for minutes
    /// (e.g. TensorRT engine builds).
    #[arg(long, value_name = "SECS")]
    pub load_total_budget: Option<u64>,

    /// Phase 52: timeout in SECONDS for the first LoadNode call per
    /// composable (overrides config
    /// `composable_node_loading.load_node_timeout_millis`, default 30s).
    #[arg(long, value_name = "SECS")]
    pub load_node_timeout: Option<u64>,

    /// Phase 52: what to do when startup completes with failed members.
    /// `continue`: keep running (web UI shows the failures) — the
    /// pre-phase-52 behavior. `exit`: shut everything down and exit
    /// non-zero, naming the failed members (CI mode).
    #[arg(long, value_enum, default_value = "continue")]
    pub on_startup_failure: OnStartupFailure,
}

/// Web UI options
#[derive(Args, Clone)]
pub struct WebOptions {
    /// Web UI address in IP:PORT format (default: 127.0.0.1:8080).
    /// Use 0.0.0.0:8080 to expose to network (insecure, use with caution).
    #[arg(long, value_name = "IP:PORT", default_value = "127.0.0.1:8080")]
    pub web_addr: String,
}

/// Contract / runtime-enforcement options
#[derive(Args, Clone)]
pub struct ContractOptions {
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
}

/// Scheduling (Phase 38) options
#[derive(Args, Clone)]
pub struct SchedOptions {
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

/// Policy for a startup that completes with failed members (phase-52.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum OnStartupFailure {
    /// Keep running; failures stay visible in the web UI and logs.
    Continue,
    /// Initiate shutdown and exit non-zero, naming the failed members.
    Exit,
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
            verbose: false,
            features: FeatureOptions::default(),
            containers: ContainerOptions::default(),
            web: WebOptions::default(),
            contract_opts: ContractOptions::default(),
            sched_opts: SchedOptions::default(),
        }
    }
}

impl Default for ContainerOptions {
    fn default() -> Self {
        Self {
            standalone_composable_nodes: false,
            load_orphan_composable_nodes: false,
            disable_respawn: false,
            container_mode: ContainerMode::Isolated,
            load_total_budget: None,
            load_node_timeout: None,
            on_startup_failure: OnStartupFailure::Continue,
        }
    }
}

impl Default for WebOptions {
    fn default() -> Self {
        Self {
            web_addr: "127.0.0.1:8080".to_string(),
        }
    }
}

impl Default for ContractOptions {
    fn default() -> Self {
        Self {
            contracts: None,
            no_provider_contracts: false,
            enforce_rules: EnforceMode::Warn,
            block_unauthorized_endpoints: false,
        }
    }
}

impl Default for SchedOptions {
    fn default() -> Self {
        Self {
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
        if !self.features.enable.is_empty() {
            return self.features.enable.contains(&Feature::Monitoring);
        }
        // Otherwise, enabled by default unless explicitly disabled
        !self.features.disable_monitoring && !self.features.disable_all
    }

    /// Check if diagnostic monitoring is enabled
    pub fn is_diagnostics_enabled(&self) -> bool {
        // If --enable is used, check if diagnostics is in the list
        if !self.features.enable.is_empty() {
            return self.features.enable.contains(&Feature::Diagnostics);
        }
        // Otherwise, enabled by default unless explicitly disabled
        !self.features.disable_diagnostics && !self.features.disable_all
    }

    /// Check if web UI is enabled
    pub fn is_web_ui_enabled(&self) -> bool {
        // If --enable is used, check if web-ui is in the list
        if !self.features.enable.is_empty() {
            return self.features.enable.contains(&Feature::WebUi);
        }
        // Otherwise, enabled by default unless explicitly disabled
        !self.features.disable_web_ui && !self.features.disable_all
    }

    /// Parse web address into (IP, port) tuple
    pub fn parse_web_addr(&self) -> eyre::Result<(String, u16)> {
        let parts: Vec<&str> = self.web.web_addr.rsplitn(2, ':').collect();
        if parts.len() != 2 {
            return Err(eyre::eyre!(
                "Invalid web address format '{}'. Expected IP:PORT (e.g., 127.0.0.1:8080)",
                self.web.web_addr
            ));
        }

        let port_str = parts[0];
        let ip = parts[1].to_string();

        let port: u16 = port_str.parse().map_err(|_| {
            eyre::eyre!(
                "Invalid port number '{}' in web address '{}'",
                port_str,
                self.web.web_addr
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
    pub fn contract_sources(&self) -> ros_launch_resolve::ros::manifest_loader::ContractSources {
        ros_launch_resolve::ros::manifest_loader::ContractSources {
            overlay: ros_launch_resolve::ros::manifest_loader::discover_overlay_root(
                self.contract_opts.contracts.as_deref(),
            ),
            provider: !self.contract_opts.no_provider_contracts,
        }
    }
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

    /// `--help` must not advertise a verb this binary does not implement.
    ///
    /// RFC-0060 W3 moved `dump`, `plot` and `contract` to
    /// ros-launch-resolve. The `Command` enum lost them, but the top-level
    /// examples kept invoking `play_launch dump ...`, and `PlotArgs`,
    /// `ContractArgs`, `ContractSubcommand` and `ContractEjectArgs` stayed
    /// behind as definitions with no variant to attach to. None of that
    /// warned: `pub` items in a `pub` module are never `dead_code`, so the
    /// residue of an extracted verb is invisible to the compiler. This test
    /// is the check that isn't.
    ///
    /// `replay` (0.9.0, renamed to `up`) is a different shape of "gone": it
    /// stays in the `Command` enum as a hidden variant on purpose (see
    /// `commands::migrated`), so it DOES still show up in
    /// `get_subcommands()` -- only the subcommand-list half of this
    /// assertion would be wrong for it. It still must never appear in
    /// rendered `--help`, so that half of the check still applies.
    #[test]
    fn help_advertises_only_verbs_this_binary_has() {
        use clap::CommandFactory;
        let cmd = Options::command();
        let verbs: Vec<String> = cmd
            .get_subcommands()
            .map(|c| c.get_name().to_string())
            .collect();
        let help = cmd.clone().render_long_help().to_string();

        // Fully gone: no variant at all, so absent from both the subcommand
        // list and the rendered help.
        for gone in ["dump", "plot", "contract"] {
            assert!(
                !verbs.iter().any(|v| v == gone),
                "`{gone}` is back as a verb -- update this test if that is intended"
            );
            assert!(
                !help.contains(&format!("play_launch {gone} ")),
                "--help invokes `play_launch {gone}`, a verb this binary does not have:\n{help}"
            );
        }
        // Hidden migration verbs: still a variant (so it DOES appear in
        // get_subcommands()), by design, so only the help-visibility half
        // applies here. A single element today; later verb migrations add
        // more (see `commands::migrated`), hence the loop shape.
        #[allow(clippy::single_element_loop)]
        for hidden in ["replay"] {
            assert!(
                verbs.iter().any(|v| v == hidden),
                "`{hidden}` must still be a parseable variant (see commands::migrated) \
                 so its error can name the replacement"
            );
            assert!(
                !help.contains(&format!("  {hidden}")),
                "`{hidden}` must not be listed in --help:\n{help}"
            );
        }
        for verb in &verbs {
            assert!(!verb.is_empty());
        }
    }

    /// `replay` must still parse -- so `commands::migrated::replay_renamed`
    /// can build a helpful error naming `up` -- while staying invisible to
    /// `--help`. Companion to `help_advertises_only_verbs_this_binary_has`,
    /// which covers the "still not advertised" half generically; this test
    /// pins the "still parses" half explicitly for the one verb where that
    /// matters (issue 0285).
    #[test]
    fn hidden_migration_verbs_parse_but_are_not_advertised() {
        use clap::CommandFactory;
        let help = Options::command().render_long_help().to_string();
        assert!(
            !help.contains("  replay"),
            "`replay` must not be listed in --help"
        );
        assert!(
            parse(&["replay", "m.yaml"]).is_ok(),
            "`replay` must still parse so its error can name the replacement"
        );
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

    #[test]
    fn launch_accepts_check_after_launch_arguments() {
        let opts = parse(&[
            "launch",
            "pkg",
            "file.launch.xml",
            "mode:=velodyne",
            "--check",
        ])
        .expect("--check must parse after KEY:=VALUE launch arguments");
        let Command::Launch(args) = opts.command else {
            panic!("expected Launch");
        };
        assert!(args.check, "--check must set the flag");
        assert_eq!(args.launch_arguments, vec!["mode:=velodyne".to_string()]);
    }

    #[test]
    fn launch_check_defaults_off() {
        let opts = parse(&["launch", "pkg", "file.launch.xml"]).expect("must parse");
        let Command::Launch(args) = opts.command else {
            panic!("expected Launch");
        };
        assert!(!args.check, "--check must default to false");
    }

    #[test]
    fn run_accepts_check() {
        let opts =
            parse(&["run", "demo_nodes_cpp", "talker", "--check"]).expect("run --check must parse");
        let Command::Run(args) = opts.command else {
            panic!("expected Run");
        };
        assert!(args.check);
    }

    #[test]
    fn up_takes_a_model_positionally_and_via_flag() {
        let a = parse(&["up", "system_model.yaml"]).expect("positional model must parse");
        let Command::Up(args) = a.command else {
            panic!("expected Up")
        };
        assert_eq!(args.model_path, Some(PathBuf::from("system_model.yaml")));

        let b = parse(&["up", "--model", "system_model.yaml"]).expect("--model must parse");
        let Command::Up(args) = b.command else {
            panic!("expected Up")
        };
        assert_eq!(args.model, Some(PathBuf::from("system_model.yaml")));
    }

    #[test]
    fn replay_still_parses_so_it_can_be_redirected() {
        // Hidden, not deleted: it must accept the old arguments so the error
        // can echo the user's own invocation back in the new form. clap's
        // bare "unrecognized subcommand" is what took down every nano-ros
        // platform's fixture build (issue 0285).
        let opts = parse(&["replay", "system_model.yaml"])
            .expect("`replay` must still PARSE so it can be redirected");
        assert!(matches!(opts.command, Command::Replay(_)));
    }
}
