use clap::{Args, Parser, Subcommand, ValueEnum};
use std::path::PathBuf;

use ros_launch_resolve::config::SchedApplyMode;

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
    play_launch launch demo_nodes_cpp topics/talker_listener.launch.py\n  \
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
    /// Resolve a launch tree into a checked SystemModel (RFC-0050).
    #[command(after_help = "Examples:\n  \
        ros-launch-resolve resolve my_bringup system.launch.xml -o system_model.yaml\n  \
        ros-launch-resolve resolve launch/system.launch.xml --system system.toml")]
    Resolve(ResolveArgs),

    /// Dump launch expansion without resolving contracts or scheduling.
    Dump(DumpArgs),

    /// Eject a contract sidecar for a package.
    Contract(ContractArgs),

    /// Plot resource usage from execution logs.
    Plot(PlotArgs),
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

// UNRESOLVED DISPOSITION — what: `CheckArgs::contract_sources`, the
// two-step `ContractSources` discovery helper. Why no caller: `CheckArgs`
// is not a `Command` variant in this crate (see `main.rs` — only
// `Resolve`/`Dump`/`Contract`/`Plot` are wired up); `check` stayed in
// `play_launch` per RFC-0060. It compiles cleanly on its own (verified:
// removing this `allow` produces only a `dead_code` warning, no
// unresolved-reference error), so it is live, working code with nothing
// calling it yet, not unreachable scaffolding.
// Separate decision, not made here: whether a `check` verb eventually
// lands in this crate, this helper moves to `play_launch`, or it gets
// deleted. This `#[allow(dead_code)]` only keeps the gate green while
// that decision is pending.
#[allow(dead_code)]
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

    /// nano-ros issue 0320 — the bringup package root that `meta.inputs[].path`
    /// are recorded relative to. When omitted, falls back to the launch file's
    /// grandparent (`<bringup>/launch/<f>.launch.xml`). Pass it to make model
    /// portability structural rather than inferred from the launch-path layout.
    #[arg(long, value_name = "PATH")]
    pub bringup_root: Option<PathBuf>,

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

    /// Output file for the dump. Defaults to `system_model.yaml`. May be
    /// given before or after the `launch` subcommand and its launch
    /// arguments.
    #[arg(long, short = 'o', global = true)]
    pub output: Option<PathBuf>,

    /// Enable debug output during dump. May be given before or after the
    /// subcommand.
    #[arg(long, global = true)]
    pub debug: bool,
}

#[derive(Subcommand)]
pub enum DumpSubcommand {
    /// Dump a launch file execution — emits the SystemModel
    /// (`system_model.yaml`), the one dump artifact (Phase 47.B2:
    /// `record.json`/`--format` retired; `dump` always emits the model).
    Launch(LaunchArgs),
}

/// Arguments for replay command
#[derive(Args, Default)]
pub struct ReplayArgs {
    /// SystemModel emitted by `play_launch resolve`/`dump` — the sole
    /// replay source (Phase 47.B3: the deprecated `--input-file
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
    /// Retained HIDDEN only so `replay --input-file ...` produces a helpful
    /// migration error (see `handle_replay`) instead of clap's bare
    /// "unexpected argument" — Phase 47.B3 removed record.json replay
    /// entirely. Never used as a value.
    #[arg(long, hide = true, value_name = "PATH")]
    pub input_file: Option<PathBuf>,

    #[command(flatten)]
    pub common: CommonOptions,
}

// UNRESOLVED DISPOSITION — what: `ReplayArgs::model_path`, resolving the
// SystemModel path from either the positional or `--model` form. Why no
// caller: `ReplayArgs` is not a `Command` variant in this crate (see
// `main.rs`); `replay` stayed in `play_launch` per RFC-0060. It compiles
// cleanly on its own (verified: removing this `allow` produces only a
// `dead_code` warning, no unresolved-reference error), so it is live,
// working code with nothing calling it yet, not unreachable scaffolding.
// Separate decision, not made here: whether a `replay` verb eventually
// lands in this crate, this helper moves to `play_launch`, or it gets
// deleted. This `#[allow(dead_code)]` only keeps the gate green while
// that decision is pending.
#[allow(dead_code)]
impl ReplayArgs {
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

// UNRESOLVED DISPOSITION — what: `CommonOptions`'s feature-toggle
// (`is_monitoring_enabled`/`is_diagnostics_enabled`/`is_web_ui_enabled`),
// `parse_web_addr`, and `contract_sources` helpers. Why no caller:
// `resolve`/`dump`/`contract`/`plot` (the only verbs this crate wires up,
// see `main.rs`) never check monitoring/diagnostics/web-ui feature flags
// or parse a web address — those are only meaningful for `run`/`replay`/
// `check`, which stayed in `play_launch` per RFC-0060. They compile
// cleanly on their own (verified: removing this `allow` produces only
// `dead_code` warnings, no unresolved-reference errors), so this is live,
// working code with nothing calling it yet, not unreachable scaffolding.
// Separate decision, not made here: whether `run`/`replay`/`check`
// eventually land in this crate, these helpers move to `play_launch`, or
// they get deleted. This `#[allow(dead_code)]` only keeps the gate green
// while that decision is pending.
#[allow(dead_code)]
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

// NOTE: the argument-parsing tests for `launch`, `run`, `replay` and `check`
// moved to play_launch with those verbs — this crate no longer defines them.
// Tests for the four launch-tree verbs belong here and are phase-312 W1.6
// follow-up.
